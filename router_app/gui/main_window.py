from __future__ import annotations

from datetime import datetime
from math import hypot
from pathlib import Path
from types import SimpleNamespace

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import (
    QAction,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QSpinBox,
    QToolBar,
    QVBoxLayout,
)

from router_app.freerouting_full import FreeroutingRunResult, run_freerouting_full
from router_app.kicad_parser import BoardData, TrackSegment, load_board
from router_app.pcb_router_full import PcbRouterRunResult, run_pcb_router_full
from router_app.reroute_engine import (
    RerouteOutcome,
    build_freerouting_external_selector_outcome,
    minimum_grid_steps_per_mm,
    run_dijkstra_reroute_test,
    select_reroute_candidates,
)
from router_app.gui.pcb_canvas import PcbCanvas, RoutePreviewCanvas


class MainWindow(QMainWindow):
    def __init__(
        self,
        initial_file: str | None = None,
        freerouting_full: bool = False,
        pcb_router: bool = False,
        router_profile_manifest: str | None = None,
        drc_feedback_pairwise: bool = False,
        drc_feedback_max_iterations: int = 0,
        drc_feedback_kicad_cli: str | None = None,
    ):
        super().__init__()
        self._program_started_at = datetime.now().astimezone()
        self._board: BoardData | None = None
        self._original_board: BoardData | None = None
        self._candidate_outcome = None
        self._candidate_ripped_net_ids: set[int] = set()
        self._external_selector_net_ids: set[int] = set()
        self._freerouting_full_enabled = freerouting_full
        self._pcb_router_enabled = pcb_router
        self._router_profile_manifest = router_profile_manifest
        self._drc_feedback_pairwise = drc_feedback_pairwise
        self._drc_feedback_max_iterations = drc_feedback_max_iterations
        self._drc_feedback_kicad_cli = drc_feedback_kicad_cli
        self._freerouting_run: FreeroutingRunResult | None = None
        self._pcb_router_run: PcbRouterRunResult | None = None
        self._freerouting_candidates_ready = False
        self._external_candidate_timing: dict[str, datetime] = {}

        self.setWindowTitle("KiCad Auto Router Viewer")
        self.resize(1280, 820)
        self._build_ui()
        self._build_toolbar()
        self._build_menu()

        if initial_file:
            self.open_board(initial_file)

    def _build_ui(self) -> None:
        splitter = QSplitter(Qt.Horizontal, self)

        left_column = QSplitter(Qt.Vertical)
        self.canvas = PcbCanvas()
        self.canvas.traceSelected.connect(self._show_trace)
        self.canvas.netSelectionChanged.connect(self._show_selected_nets)
        self.canvas.netSelectionChanged.connect(self._update_ripup_buttons)
        self.canvas.ripupStateChanged.connect(self._update_ripup_buttons)
        self.canvas.rippedNetsChanged.connect(self._show_ripped_nets)
        left_column.addWidget(self.canvas)

        self.route_preview = RoutePreviewCanvas()
        self.route_preview.setMinimumHeight(220)
        self.route_preview.show_message("Reroute preview")
        preview_panel = QFrame()
        preview_layout = QVBoxLayout(preview_panel)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(4)
        self.preview_reference_button = QPushButton("Show Original Reference")
        self.preview_reference_button.setCheckable(True)
        self.preview_reference_button.setChecked(True)
        self.preview_reference_button.setToolTip("Show or hide faint original routes in this lower preview")
        self.preview_reference_button.toggled.connect(self.route_preview.set_show_reference_routes)
        preview_layout.addWidget(self.preview_reference_button)
        preview_layout.addWidget(self.route_preview)
        left_column.addWidget(preview_panel)
        left_column.setStretchFactor(0, 3)
        left_column.setStretchFactor(1, 2)

        side_panel = TracePanel()
        self.trace_panel = side_panel

        splitter.addWidget(left_column)
        splitter.addWidget(side_panel)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

    def _build_menu(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        open_action = QAction("&Open .kicad_pcb", self)
        open_action.triggered.connect(self._choose_board)
        file_menu.addAction(open_action)

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("View", self)
        toolbar.setMovable(False)

        toolbar.addWidget(QLabel("Layer "))
        self.layer_combo = QComboBox()
        self.layer_combo.setMinimumWidth(110)
        self.layer_combo.currentTextChanged.connect(self.canvas.set_current_layer)
        toolbar.addWidget(self.layer_combo)

        self.all_layers_check = QCheckBox("All layers")
        self.all_layers_check.setChecked(True)
        self.all_layers_check.toggled.connect(self.canvas.set_show_all_layers)
        toolbar.addWidget(self.all_layers_check)

        self.components_check = QCheckBox("Components")
        self.components_check.setChecked(True)
        self.components_check.toggled.connect(self.canvas.set_show_components)
        toolbar.addWidget(self.components_check)

        self.two_pin_check = QCheckBox("2-pin nets")
        self.two_pin_check.setChecked(True)
        self.two_pin_check.toggled.connect(self.canvas.set_emphasize_two_pin_nets)
        toolbar.addWidget(self.two_pin_check)

        self.rip_up_button = QPushButton("Rip up")
        self.rip_up_button.setEnabled(False)
        self.rip_up_button.clicked.connect(self._rip_up_selected)
        toolbar.addWidget(self.rip_up_button)

        self.undo_rip_up_button = QPushButton("Undo")
        self.undo_rip_up_button.setEnabled(False)
        self.undo_rip_up_button.clicked.connect(self._undo_rip_up)
        toolbar.addWidget(self.undo_rip_up_button)

        self.generate_candidates_button = QPushButton("Generate Candidates")
        self.generate_candidates_button.setEnabled(False)
        self.generate_candidates_button.clicked.connect(self._generate_candidates_for_ripped_nets)
        toolbar.addWidget(self.generate_candidates_button)

        self.reroute_button = QPushButton("Reroute")
        self.reroute_button.setEnabled(False)
        self.reroute_button.clicked.connect(self._reroute_selected_removed_nets)
        toolbar.addWidget(self.reroute_button)

        self.addToolBar(toolbar)

    def _choose_board(self) -> None:
        file_name, _ = QFileDialog.getOpenFileName(
            self,
            "Open KiCad PCB",
            str(Path.cwd()),
            "KiCad PCB (*.kicad_pcb);;All Files (*)",
        )
        if file_name:
            self.open_board(file_name)

    def open_board(self, file_name: str) -> None:
        if self._freerouting_full_enabled and not self._pcb_router_enabled:
            self._open_board_with_freerouting(file_name)
            return
        if self._freerouting_full_enabled or self._pcb_router_enabled:
            self._open_board_with_external_backends(file_name)
            return
        try:
            board = load_board(file_name)
        except Exception as exc:
            QMessageBox.critical(self, "Could not open board", str(exc))
            return

        self._board = board
        self._reset_candidate_generation_state()
        self.canvas.load_board(board)
        self._set_layers(board.copper_layers)
        self.trace_panel.show_board(board)
        self.trace_panel.set_grid_density_minimum(minimum_grid_steps_per_mm(board))
        self._update_ripup_buttons()
        pad_count = sum(len(footprint.pads) for footprint in board.footprints)
        self.statusBar().showMessage(
            f"Loaded {board.path.name} via {board.backend}: "
            f"{len(board.tracks)} traces, {len(board.vias)} vias, "
            f"{len(board.footprints)} components, {pad_count} pads, "
            f"{len(board.two_pin_net_ids)} two-pin nets."
        )
        total_wire_length_mm = sum(
            hypot(track.end[0] - track.start[0], track.end[1] - track.start[1])
            for track in board.tracks
        )
        total_via_count = len(board.vias)
        failed_net_ids = self._estimate_failed_nets(board)
        failed_net_labels = [
            f"{net_id}:{board.nets.get(net_id, f'Net {net_id}')}"
            for net_id in failed_net_ids
        ]
        print(
            f"freerouting_total_wire_length_mm = {total_wire_length_mm:.3f}  # 總線長(mm)",
            flush=True,
        )
        print(
            f"freerouting_total_via_count = {total_via_count}  # 總 via 數",
            flush=True,
        )
        print(
            "freerouting_failed_nets_estimated = "
            + (", ".join(failed_net_labels) if failed_net_labels else "(none)")
            + "  # 估計繞失敗 net（有2個以上pad但沒有任何track/via）",
            flush=True,
        )

    def _open_board_with_external_backends(self, file_name: str) -> None:
        # Run requested external routers first, then keep every backend's final
        # board and payload available to the selector candidate builder.
        freerouting_result: FreeroutingRunResult | None = None
        pcbrouter_result: PcbRouterRunResult | None = None
        self._external_candidate_timing = {
            "program_started_at": self._program_started_at,
            "collection_started_at": datetime.now().astimezone(),
        }
        if self._freerouting_full_enabled:
            try:
                self._external_candidate_timing["freerouting_started_at"] = datetime.now().astimezone()
                freerouting_result = run_freerouting_full(file_name, profile_manifest_path=self._router_profile_manifest)
                self._external_candidate_timing["freerouting_finished_at"] = datetime.now().astimezone()
            except Exception as exc:
                QMessageBox.critical(self, "freerouting-full failed", str(exc))
                return
        if self._pcb_router_enabled:
            try:
                self._external_candidate_timing["pcbrouter_started_at"] = datetime.now().astimezone()
                pcbrouter_result = run_pcb_router_full(file_name, profile_manifest_path=self._router_profile_manifest)
                self._external_candidate_timing["pcbrouter_finished_at"] = datetime.now().astimezone()
            except Exception as exc:
                QMessageBox.critical(self, "pcb-router failed", str(exc))
                return
        self._external_candidate_timing["collection_finished_at"] = datetime.now().astimezone()

        self._freerouting_run = freerouting_result
        self._pcb_router_run = pcbrouter_result
        display_result = freerouting_result or pcbrouter_result
        if display_result is None:
            return

        display_board = self._display_board_for_external_backends(
            file_name,
            display_result,
            freerouting_result=freerouting_result,
            pcbrouter_result=pcbrouter_result,
        )
        self._original_board = display_result.original_board
        self._board = display_board
        self._reset_candidate_generation_state()
        self.canvas.load_board(display_board)
        self._set_layers(display_board.copper_layers)
        self.trace_panel.show_board(display_board)
        self.trace_panel.set_grid_density_minimum(minimum_grid_steps_per_mm(display_board))
        self._update_ripup_buttons()

        previews = self._external_ripup_previews()
        pad_count = sum(len(footprint.pads) for footprint in display_board.footprints)
        print(f"external_left_top_display_board = {Path(display_board.path).resolve()}", flush=True)
        self.statusBar().showMessage(
            f"Loaded {display_board.path.name} via external backend: "
            f"{len(display_board.tracks)} traces, {len(display_board.vias)} vias, "
            f"{len(display_board.footprints)} components, {pad_count} pads, "
            f"{len(display_board.two_pin_net_ids)} two-pin nets, "
            f"{len({item.net_id for item in previews})} unique ripped nets, "
            f"{len(previews)} ripped-net occurrences."
        )

        if freerouting_result is not None:
            self._print_freerouting_summary(freerouting_result)
        if pcbrouter_result is not None:
            self._print_pcbrouter_summary(pcbrouter_result)

    def _display_board_for_external_backends(
        self,
        file_name: str,
        display_result,
        freerouting_result: FreeroutingRunResult | None = None,
        pcbrouter_result: PcbRouterRunResult | None = None,
    ) -> BoardData:
        """Return the board shown in the main canvas for external-router runs.

        Routed reference inputs are user-provided comparison baselines. Keep
        them visible in the top-left canvas while external routers and selector
        candidates continue to use the canonical unrouted coordinate frame.
        For unrouted inputs with multiple external routers, show the backend
        final board with fewer vias so the visible baseline matches the simpler
        completed route.
        """
        input_path = Path(file_name)
        if self._is_routed_reference_input(input_path):
            return load_board(input_path)
        if freerouting_result is not None and pcbrouter_result is not None:
            freerouting_vias = len(freerouting_result.routed_board.vias)
            pcbrouter_vias = len(pcbrouter_result.routed_board.vias)
            if pcbrouter_vias < freerouting_vias:
                return pcbrouter_result.routed_board
        return display_result.routed_board

    def _is_routed_reference_input(self, board_path: Path) -> bool:
        """Return true when the opened file name represents a routed reference board."""
        return ".routed" in board_path.stem

    def _open_board_with_freerouting(self, file_name: str) -> None:
        self._external_candidate_timing = {
            "program_started_at": self._program_started_at,
            "collection_started_at": datetime.now().astimezone(),
            "freerouting_started_at": datetime.now().astimezone(),
        }
        try:
            result = run_freerouting_full(file_name, profile_manifest_path=self._router_profile_manifest)
            self._external_candidate_timing["freerouting_finished_at"] = datetime.now().astimezone()
            self._external_candidate_timing["collection_finished_at"] = datetime.now().astimezone()
        except Exception as exc:
            QMessageBox.critical(self, "freerouting-full failed", str(exc))
            return

        self._freerouting_run = result
        display_board = self._display_board_for_external_backends(file_name, result)
        self._original_board = result.original_board
        self._board = display_board
        self._reset_candidate_generation_state()
        self.canvas.load_board(display_board)
        self._set_layers(display_board.copper_layers)
        self.trace_panel.show_board(display_board)
        self.trace_panel.set_grid_density_minimum(minimum_grid_steps_per_mm(display_board))
        self._update_ripup_buttons()

        pad_count = sum(len(footprint.pads) for footprint in display_board.footprints)
        self.statusBar().showMessage(
            f"Loaded {display_board.path.name} via {display_board.backend}: "
            f"{len(display_board.tracks)} traces, {len(display_board.vias)} vias, "
            f"{len(display_board.footprints)} components, {pad_count} pads, "
            f"{len(display_board.two_pin_net_ids)} two-pin nets, "
            f"{result.unique_ripped_net_count} unique ripped nets, "
            f"{len(result.ripup_previews)} ripped-net occurrences."
        )

        print(
            f"freerouting_ripup_event_count = {result.ripup_event_count}  # freerouting 實際發生的拆線事件次數",
            flush=True,
        )
        print(
            f"interactive_router_unique_ripped_nets = {result.unique_ripped_net_count}  # Interactive-Router 拿到被拆 net 的去重總數",
            flush=True,
        )
        print(
            f"interactive_router_cumulative_ripped_net_count = {result.cumulative_ripped_net_count}  # 被拆 net 的不去重累計次數",
            flush=True,
        )
        print(
            f"freerouting_cumulative_ripped_item_count = {result.cumulative_ripped_item_count}  # 被拆 track/via 物件總數（累計）",
            flush=True,
        )
        print(
            f"freerouting_cumulative_ripped_segment_count = {result.cumulative_ripped_segment_count}  # 被拆線段總數（polyline 展開後累計）",
            flush=True,
        )
        total_wire_length_mm = sum(
            hypot(track.end[0] - track.start[0], track.end[1] - track.start[1])
            for track in result.routed_board.tracks
        )
        total_via_count = len(result.routed_board.vias)
        failed_net_ids = self._estimate_failed_nets(result.routed_board)
        failed_net_labels = [
            f"{net_id}:{result.routed_board.nets.get(net_id, f'Net {net_id}')}"
            for net_id in failed_net_ids
        ]
        print(
            f"freerouting_total_wire_length_mm = {total_wire_length_mm:.3f}  # 左上繞線結果總線長(mm)",
            flush=True,
        )
        print(
            f"freerouting_total_via_count = {total_via_count}  # 左上繞線結果總 via 數",
            flush=True,
        )
        print(
            "freerouting_failed_nets_estimated = "
            + (", ".join(failed_net_labels) if failed_net_labels else "(none)")
            + "  # 估計繞失敗 net（有2個以上pad但沒有任何track/via）",
            flush=True,
        )
        if result.still_unconnected:
            print(
                "freerouting_unconnected_status = still_unconnected"
                + (
                    f" ({result.unconnected_item_count} connection(s))"
                    if result.unconnected_item_count is not None
                    else ""
                ),
                flush=True,
            )
            if result.unconnected_report:
                print(result.unconnected_report, flush=True)
        else:
            print("freerouting_unconnected_status = fully_connected", flush=True)
        if result.attempted_profiles:
            print(
                "freerouting_profiles_attempted = " + ", ".join(result.attempted_profiles),
                flush=True,
            )
        for profile_report in result.profile_reports:
            profile = profile_report.get("profile", "(unknown)")
            if profile_report.get("still_unconnected"):
                count = profile_report.get("unconnected_item_count")
                suffix = f" ({count})" if count is not None else ""
                status = f"still_unconnected{suffix}"
            else:
                status = "fully_connected"
            print(f"freerouting_profile_result {profile} = {status}", flush=True)
        print(
            f"freerouting_profile_selected = {result.selected_profile}",
            flush=True,
        )
        if result.stdout_log_path is not None:
            print(f"freerouting_stdout_log = {result.stdout_log_path}", flush=True)
        if result.stderr_log_path is not None:
            print(f"freerouting_stderr_log = {result.stderr_log_path}", flush=True)

    def _print_freerouting_summary(self, result: FreeroutingRunResult) -> None:
        # Keep the existing freerouting terminal summary available when multiple
        # external backends are run together.
        print(
            f"freerouting_ripup_event_count = {result.ripup_event_count}  # freerouting 實際發生的拆線事件次數",
            flush=True,
        )
        print(
            f"interactive_router_unique_ripped_nets = {result.unique_ripped_net_count}  # Interactive-Router 拿到被拆 net 的去重總數",
            flush=True,
        )
        total_wire_length_mm = sum(
            hypot(track.end[0] - track.start[0], track.end[1] - track.start[1])
            for track in result.routed_board.tracks
        )
        failed_net_ids = self._estimate_failed_nets(result.routed_board)
        failed_net_labels = [
            f"{net_id}:{result.routed_board.nets.get(net_id, f'Net {net_id}')}"
            for net_id in failed_net_ids
        ]
        print(f"freerouting_total_wire_length_mm = {total_wire_length_mm:.3f}  # 左上繞線結果總線長(mm)", flush=True)
        print(f"freerouting_total_via_count = {len(result.routed_board.vias)}  # 左上繞線結果總 via 數", flush=True)
        print(
            "freerouting_failed_nets_estimated = "
            + (", ".join(failed_net_labels) if failed_net_labels else "(none)")
            + "  # 估計繞失敗 net（有2個以上pad但沒有任何track/via）",
            flush=True,
        )
        print(
            "freerouting_unconnected_status = "
            + (
                "still_unconnected"
                + (f" ({result.unconnected_item_count} connection(s))" if result.unconnected_item_count is not None else "")
                if result.still_unconnected
                else "fully_connected"
            ),
            flush=True,
        )
        if result.stdout_log_path is not None:
            print(f"freerouting_stdout_log = {Path(result.stdout_log_path).resolve()}", flush=True)
        if result.stderr_log_path is not None:
            print(f"freerouting_stderr_log = {Path(result.stderr_log_path).resolve()}", flush=True)

    def _print_pcbrouter_summary(self, result: PcbRouterRunResult) -> None:
        # Print absolute artifact paths so PcbRouter runs are reproducible and debuggable.
        total_wire_length_mm = sum(
            hypot(track.end[0] - track.start[0], track.end[1] - track.start[1])
            for track in result.routed_board.tracks
        )
        failed_net_labels = [
            f"{net_id}:{result.routed_board.nets.get(net_id, f'Net {net_id}')}"
            for net_id in result.failed_net_ids_estimated
        ]
        print(f"pcbrouter_ripup_event_count = {result.ripup_event_count}", flush=True)
        print(f"pcbrouter_unique_ripped_nets = {result.unique_ripped_net_count}", flush=True)
        if result.attempted_profiles:
            print("pcbrouter_profiles_attempted = " + ", ".join(result.attempted_profiles), flush=True)
            print(f"pcbrouter_profile_selected = {result.selected_profile}", flush=True)
        if result.profile_manifest_path is not None:
            print(f"pcbrouter_profile_manifest = {Path(result.profile_manifest_path).resolve()}", flush=True)
        print(f"pcbrouter_total_wire_length_mm = {total_wire_length_mm:.3f}", flush=True)
        print(f"pcbrouter_total_via_count = {len(result.routed_board.vias)}", flush=True)
        print(
            "pcbrouter_failed_nets_estimated = "
            + (", ".join(failed_net_labels) if failed_net_labels else "(none)"),
            flush=True,
        )
        print(f"pcbrouter_routed_board = {Path(result.routed_board_path).resolve()}", flush=True)
        if result.ripup_payload_path is not None:
            print(f"pcbrouter_ripup_payload = {Path(result.ripup_payload_path).resolve()}", flush=True)
        else:
            print("pcbrouter_ripup_payload = (none)", flush=True)
        if result.stdout_log_path is not None:
            print(f"pcbrouter_stdout_log = {Path(result.stdout_log_path).resolve()}", flush=True)
        if result.stderr_log_path is not None:
            print(f"pcbrouter_stderr_log = {Path(result.stderr_log_path).resolve()}", flush=True)

    def _show_trace(self, track: TrackSegment) -> None:
        if self._board is not None:
            self.trace_panel.show_trace(self._board, track)

    def _show_selected_nets(self, selected_net_ids: set[int]) -> None:
        if self._board is not None:
            self.trace_panel.show_selection(self._board, selected_net_ids)

    def _rip_up_selected(self) -> None:
        if self.canvas.rip_up_selected() and self._board is not None:
            self._reset_candidate_generation_state()
            self.trace_panel.show_selection(self._board, set())
            self.trace_panel.show_ripped_nets(self._board, self.canvas.ripped_net_ids)
            self._show_status_counts("Rip-up applied")

    def _undo_rip_up(self) -> None:
        if self.canvas.undo_rip_up():
            self._reset_candidate_generation_state()
            if self._board is not None:
                self.trace_panel.show_ripped_nets(self._board, self.canvas.ripped_net_ids)
            self._show_status_counts("Rip-up undone")

    def _update_ripup_buttons(self, *_args) -> None:
        if self._freerouting_full_enabled or self._pcb_router_enabled:
            self.rip_up_button.setEnabled(False)
            self.undo_rip_up_button.setEnabled(False)
            self.generate_candidates_button.setEnabled(bool(self._external_ripup_previews()))
            self.reroute_button.setEnabled(
                self._freerouting_candidates_ready
                and self._candidate_outcome is not None
                and bool(getattr(self._candidate_outcome, "result", None))
            )
            return
        self.rip_up_button.setEnabled(bool(self.canvas.selected_net_ids))
        self.undo_rip_up_button.setEnabled(self.canvas.can_undo_rip_up)
        has_ripped = bool(self.canvas.ripped_net_ids)
        self.generate_candidates_button.setEnabled(has_ripped)
        can_reroute = (
            has_ripped
            and self._candidate_outcome is not None
            and bool(getattr(self._candidate_outcome, "result", None))
            and self._candidate_ripped_net_ids == set(self.canvas.ripped_net_ids)
        )
        self.reroute_button.setEnabled(can_reroute)

    def _show_ripped_nets(self, ripped_net_ids: set[int]) -> None:
        if self._candidate_outcome is not None and self._candidate_ripped_net_ids != set(ripped_net_ids):
            self._reset_candidate_generation_state()
        if self._board is not None:
            self.trace_panel.show_ripped_nets(self._board, ripped_net_ids)
        self._update_ripup_buttons()

    def _generate_candidates_for_ripped_nets(self) -> None:
        if self._freerouting_full_enabled or self._pcb_router_enabled:
            self._show_external_backend_ripped_routes()
            return
        if self._board is None:
            return
        if not self.canvas.ripped_net_ids:
            self.statusBar().showMessage("No ripped-up nets to generate candidates.")
            return
        outcome = run_dijkstra_reroute_test(
            self._board,
            self.canvas.ripped_net_ids,
            self.trace_panel.grid_steps_per_mm,
        )
        self._candidate_outcome = outcome if outcome.result else None
        self._candidate_ripped_net_ids = set(self.canvas.ripped_net_ids) if outcome.result else set()
        elapsed = (
            f" Runtime: {outcome.elapsed_seconds:.3f} s"
            if outcome.elapsed_seconds is not None
            else ""
        )
        prefix = "Candidates generated." if outcome.result else "Candidate generation failed."
        self.statusBar().showMessage(f"{prefix} {outcome.message}{elapsed}")
        if outcome.result:
            self.route_preview.show_routes(outcome.result, self._board, self.canvas.ripped_net_ids)
        else:
            self.route_preview.show_message(f"{outcome.message}{elapsed}")
        self._update_ripup_buttons()

    def _reroute_selected_removed_nets(self) -> None:
        if self._freerouting_full_enabled or self._pcb_router_enabled:
            if self._board is None or not self._external_ripup_previews():
                self.statusBar().showMessage("External router result is not available.")
                return
            if self._candidate_outcome is None or not self._candidate_outcome.result:
                self.statusBar().showMessage("Generate candidates first, then reroute.")
                return

            selection = self._select_candidates_with_configured_feedback()
            self._print_external_candidate_collection_timing()
            if not selection.ok:
                self.statusBar().showMessage(f"Path selection failed: {selection.message}")
                return
            if not str(selection.solver).lower().startswith("gurobi"):
                self.statusBar().showMessage(
                    f"Reroute skipped: solver={selection.solver} (not gurobi). {selection.message}"
                )
                return

            # Use the selector net set here, not only the ripup preview net set:
            # final-board-only candidates need the same faint original-route
            # reference in the selected preview.
            ripped_net_ids = set(self._external_selector_net_ids) or {
                item.net_id for item in self._external_ripup_previews()
            }
            selected_preview = self._build_selected_preview_results(self._candidate_outcome.result, selection)
            if selected_preview:
                self.route_preview.show_routes(selected_preview, self._board, ripped_net_ids)
                self.statusBar().showMessage(
                    f"Reroute selected by {selection.solver}. {selection.message}"
                )
            else:
                self.statusBar().showMessage(
                    "Reroute has no selectable solution. Keep current preview unchanged."
                )
            self._print_selection_artifacts(selection)
            return
        if self._board is None:
            return
        if self._candidate_outcome is None or not self._candidate_outcome.result:
            self.statusBar().showMessage("Generate candidates first, then reroute.")
            return
        if self._candidate_ripped_net_ids != set(self.canvas.ripped_net_ids):
            self.statusBar().showMessage("Ripped-up nets changed. Please generate candidates again.")
            self._reset_candidate_generation_state()
            self._update_ripup_buttons()
            return

        selection = self._select_candidates_with_configured_feedback()
        if not selection.ok:
            self.statusBar().showMessage(f"Path selection failed: {selection.message}")
            return
        if not str(selection.solver).lower().startswith("gurobi"):
            self.statusBar().showMessage(
                f"Reroute skipped: solver={selection.solver} (not gurobi). {selection.message}"
            )
            return

        selected_preview = self._build_selected_preview_results(self._candidate_outcome.result, selection)
        if selected_preview:
            self.route_preview.show_routes(selected_preview, self._board, self.canvas.ripped_net_ids)
            self.statusBar().showMessage(
                f"Reroute selected by {selection.solver}. {selection.message}"
            )
        else:
            self.statusBar().showMessage(
                "Reroute has no selectable solution. Keep current preview unchanged."
            )
        self._print_selection_artifacts(selection)

    def _print_selection_artifacts(self, selection) -> None:
        """Print saved selector artifacts near the end of a GUI reroute action."""
        artifacts = getattr(selection, "artifacts", None) or {}
        gurobi_board_path = artifacts.get("gurobi_selected_board")
        if gurobi_board_path:
            print(f"gui_gurobi_board_saved = {gurobi_board_path}", flush=True)

    def _select_candidates_with_configured_feedback(self):
        """Run path selection with the optional KiCad DRC feedback CLI settings."""
        return select_reroute_candidates(
            self._candidate_outcome,
            max_paths_per_net=1,
            prefer_gurobi=True,
            drc_feedback_pairwise=self._drc_feedback_pairwise,
            drc_feedback_max_iterations=self._drc_feedback_max_iterations,
            drc_feedback_kicad_cli=self._drc_feedback_kicad_cli,
        )

    def _show_status_counts(self, prefix: str) -> None:
        if self._board is None:
            return
        self.statusBar().showMessage(
            f"{prefix}: {len(self.canvas.selected_net_ids)} selected nets, "
            f"{len(self._board.tracks)} original traces"
        )

    def _reset_candidate_generation_state(self) -> None:
        self._candidate_outcome = None
        self._candidate_ripped_net_ids = set()
        self._external_selector_net_ids = set()
        self._freerouting_candidates_ready = False
        if self._freerouting_full_enabled or self._pcb_router_enabled:
            self.route_preview.show_message("Generate candidates to show external ripped routes")
        else:
            self.route_preview.show_message("Generate candidates first")

    def _print_external_candidate_collection_timing(self) -> None:
        """Print wall-clock timing for external-router candidate collection.

        Timestamps are captured around the external router calls, then reported
        after the selector returns so the terminal ends with a single summary of
        command start, candidate collection completion, and per-router profile
        durations.
        """
        timing = self._external_candidate_timing
        program_started = timing.get("program_started_at")
        collection_finished = timing.get("collection_finished_at")
        if program_started is None or collection_finished is None:
            return

        elapsed_sec = (collection_finished - program_started).total_seconds()
        print(
            "external_candidate_collection_program_started_at = "
            + _format_wall_timestamp(program_started),
            flush=True,
        )
        print(
            "external_candidate_collection_finished_at = "
            + _format_wall_timestamp(collection_finished),
            flush=True,
        )
        print(
            f"external_candidate_collection_elapsed_sec = {elapsed_sec:.3f}",
            flush=True,
        )
        for router_key, label in (
            ("freerouting", "freerouting_profiles"),
            ("pcbrouter", "pcbrouter_profiles"),
        ):
            started = timing.get(f"{router_key}_started_at")
            finished = timing.get(f"{router_key}_finished_at")
            if started is None or finished is None:
                continue
            router_elapsed_sec = (finished - started).total_seconds()
            print(
                f"external_candidate_collection_{label}_elapsed_sec = {router_elapsed_sec:.3f}",
                flush=True,
            )

    def _external_ripup_previews(self) -> list[object]:
        # Keep backend previews as one stream for the GUI and selector setup.
        previews: list[object] = []
        if self._freerouting_run is not None:
            previews.extend(self._freerouting_run.ripup_previews)
        if self._pcb_router_run is not None:
            previews.extend(self._pcb_router_run.ripup_previews)
        return previews

    def _external_final_boards(self) -> list[BoardData]:
        # Feed every requested backend final board to the selector; exact dedup
        # happens later in candidate append without changing the Gurobi model.
        boards: list[BoardData] = []
        if self._freerouting_run is not None:
            boards.extend(self._freerouting_run.candidate_routed_boards)
        if self._pcb_router_run is not None:
            boards.extend(self._pcb_router_run.candidate_routed_boards or (self._pcb_router_run.routed_board,))
        return boards

    def _external_original_candidate_boards(self) -> list[BoardData]:
        # Preserve shifted routed input geometry as candidates without letting
        # those tracks/vias expand the selector board bounds.
        boards: list[BoardData] = []
        if self._freerouting_run is not None and self._freerouting_run.original_candidate_board is not None:
            boards.append(self._freerouting_run.original_candidate_board)
        if self._pcb_router_run is not None and self._pcb_router_run.original_candidate_board is not None:
            boards.append(self._pcb_router_run.original_candidate_board)
        return boards

    def _freerouting_payload_paths(self) -> list[Path]:
        if self._freerouting_run is None:
            return []
        return list(self._freerouting_run.candidate_ripup_payload_paths)

    def _pcbrouter_payload_paths(self) -> list[Path]:
        if self._pcb_router_run is None or self._pcb_router_run.ripup_payload_path is None:
            return []
        return list(self._pcb_router_run.candidate_ripup_payload_paths or (self._pcb_router_run.ripup_payload_path,))

    def _external_final_route_net_ids(self, selector_board: BoardData) -> set[int]:
        """Return selector-board net ids that have copper in external final boards.

        A net can be fully routed by an external router without ever appearing
        in a ripup payload. Such nets still need selector candidates from final
        boards; the normal candidate append path will still apply angle/Y,
        pad coverage, and collision validation before accepting them.
        """
        selector_net_by_name = {
            str(name): int(net_id)
            for net_id, name in selector_board.nets.items()
            if int(net_id) > 0 and str(name)
        }
        final_route_net_ids: set[int] = set()
        for candidate_board in self._external_final_boards():
            board_route_net_ids = {
                int(getattr(track, "net_id", 0) or 0)
                for track in candidate_board.tracks
            } | {
                int(getattr(via, "net_id", 0) or 0)
                for via in candidate_board.vias
            }
            for route_net_id in board_route_net_ids:
                if route_net_id <= 0:
                    continue
                route_net_name = str(candidate_board.nets.get(route_net_id, ""))
                selector_net_id = selector_net_by_name.get(route_net_name)
                if selector_net_id is not None:
                    final_route_net_ids.add(selector_net_id)
        return final_route_net_ids

    def _external_final_route_previews(self, selector_board: BoardData, net_ids: set[int]) -> list[object]:
        """Build display-only previews for final-board candidates on selected nets.

        This does not bypass selector validation. It only lets the lower-left
        canvas show final-route candidates for nets that have no ripup
        occurrence, so users can compare them against the faint reference route.
        """
        if not net_ids:
            return []
        selector_net_names = {
            int(net_id): str(selector_board.nets.get(net_id, ""))
            for net_id in net_ids
        }
        previews: list[object] = []
        seen: set[tuple[int, tuple[tuple[object, ...], ...], tuple[tuple[object, ...], ...]]] = set()
        for candidate_board in self._external_final_boards():
            source = f"final:{Path(candidate_board.path).stem}"
            source_net_by_name = {
                str(name): int(net_id)
                for net_id, name in candidate_board.nets.items()
                if int(net_id) > 0 and str(name)
            }
            for selector_net_id, net_name in selector_net_names.items():
                source_net_id = source_net_by_name.get(net_name)
                if source_net_id is None:
                    continue
                segments = [
                    track
                    for track in candidate_board.tracks
                    if int(getattr(track, "net_id", 0) or 0) == source_net_id
                ]
                vias = [
                    via
                    for via in candidate_board.vias
                    if int(getattr(via, "net_id", 0) or 0) == source_net_id
                ]
                if not segments and not vias:
                    continue
                segment_key = tuple(
                    sorted(
                        (
                            str(getattr(segment, "layer", "")),
                            round(float(segment.start[0]), 6),
                            round(float(segment.start[1]), 6),
                            round(float(segment.end[0]), 6),
                            round(float(segment.end[1]), 6),
                            round(float(getattr(segment, "width", 0.0)), 6),
                        )
                        for segment in segments
                    )
                )
                via_key = tuple(
                    sorted(
                        (
                            round(float(via.center[0]), 6),
                            round(float(via.center[1]), 6),
                            round(float(getattr(via, "diameter", 0.0)), 6),
                            tuple(getattr(via, "layers", ())),
                        )
                        for via in vias
                    )
                )
                preview_key = (selector_net_id, segment_key, via_key)
                if preview_key in seen:
                    continue
                seen.add(preview_key)
                previews.append(
                    SimpleNamespace(
                        net_id=selector_net_id,
                        net_name=net_name,
                        segments=segments,
                        vias=vias,
                        truncated=False,
                        occurrence_index=0,
                        source_net_id=source_net_id,
                        source=source,
                        preview_kind="candidate",
                    )
                )
        return previews

    def _show_external_backend_ripped_routes(self) -> None:
        previews = self._external_ripup_previews()
        if self._board is None:
            self.route_preview.show_message("External router result is not available")
            return
        selector_board = self._original_board or self._board
        display_offset = self._selector_to_display_offset_mm()
        display_previews = self._translate_route_previews_for_display(previews, display_offset)
        if display_offset != (0.0, 0.0):
            print(
                "external_preview_translation_mm = "
                f"dx={display_offset[0]:.6f} dy={display_offset[1]:.6f}",
                flush=True,
            )

        ripped_net_ids = {item.net_id for item in previews}
        visible_preview_net_ids = {
            item.net_id
            for item in previews
            if getattr(item, "segments", None) or getattr(item, "vias", None)
        }
        final_route_net_ids = self._external_final_route_net_ids(selector_board)
        selector_net_ids = set(ripped_net_ids) | set(final_route_net_ids)
        if not selector_net_ids:
            self.route_preview.show_message("External router result is not available")
            return
        # Some router payload entries exist only as empty occurrences after an
        # angle/Y rejection. Show their final-board route preview so the lower
        # pane still has visible geometry to compare against the reference path.
        added_final_only_net_ids = sorted(final_route_net_ids - visible_preview_net_ids)
        final_only_previews = self._external_final_route_previews(
            selector_board,
            set(added_final_only_net_ids),
        )
        display_previews.extend(self._translate_route_previews_for_display(final_only_previews, display_offset))
        overlay_net_ids = []
        for item in previews:
            if item.occurrence_index > 0:
                overlay_net_ids.append(f"{item.net_id}@E{item.occurrence_index}")
            else:
                overlay_net_ids.append(str(item.net_id))
        for item in final_only_previews:
            overlay_net_ids.append(f"{item.net_id}@final")
        segment_count = sum(len(item.segments) for item in previews)
        via_count = sum(len(item.vias) for item in previews)
        final_only_segment_count = sum(len(item.segments) for item in final_only_previews)
        final_only_via_count = sum(len(item.vias) for item in final_only_previews)
        truncated_count = sum(1 for item in previews if item.truncated)
        truncated_labels = []
        for item in previews:
            if not item.truncated:
                continue
            if item.occurrence_index > 0:
                truncated_labels.append(f"{item.net_id}@E{item.occurrence_index}")
            else:
                truncated_labels.append(str(item.net_id))
        self.trace_panel.show_ripped_nets(self._board, selector_net_ids)
        self.route_preview.show_routes(display_previews, self._board, selector_net_ids)
        print(
            "external_ripped_occurrences = "
            + ", ".join(overlay_net_ids),
            flush=True,
        )
        print(
            "external_selector_final_route_net_ids = "
            + (", ".join(str(net_id) for net_id in sorted(final_route_net_ids)) if final_route_net_ids else "(none)"),
            flush=True,
        )
        print(
            "external_selector_visible_preview_net_ids = "
            + (", ".join(str(net_id) for net_id in sorted(visible_preview_net_ids)) if visible_preview_net_ids else "(none)"),
            flush=True,
        )
        print(
            "external_selector_final_only_net_ids = "
            + (", ".join(str(net_id) for net_id in added_final_only_net_ids) if added_final_only_net_ids else "(none)"),
            flush=True,
        )
        print(
            "external_selector_final_only_preview_count = "
            f"{len(final_only_previews)} segments={final_only_segment_count} vias={final_only_via_count}",
            flush=True,
        )
        print(
            f"external_truncated_occurrence_count = {truncated_count}",
            flush=True,
        )
        print(
            "external_truncated_occurrences = "
            + (", ".join(truncated_labels) if truncated_labels else "(none)"),
            flush=True,
        )
        outcome = build_freerouting_external_selector_outcome(
            selector_board,
            selector_net_ids,
            self.trace_panel.grid_steps_per_mm,
            final_board=None,
            final_boards=self._external_final_boards(),
            original_candidate_boards=self._external_original_candidate_boards(),
            freerouting_payload_paths=self._freerouting_payload_paths(),
            pcbrouter_payload_paths=self._pcbrouter_payload_paths(),
            left_top_reference_board=self._board,
            backend_label="external",
        )
        if outcome.result:
            self._candidate_outcome = RerouteOutcome(
                ok=outcome.ok,
                message=outcome.message,
                result=outcome.result,
                elapsed_seconds=outcome.elapsed_seconds,
                candidate_export_path=outcome.candidate_export_path,
                selector_external_only=True,
                left_top_reference_board=outcome.left_top_reference_board,
            )
        else:
            self._candidate_outcome = None
        self._candidate_ripped_net_ids = set(selector_net_ids) if outcome.result else set()
        self._external_selector_net_ids = set(selector_net_ids) if outcome.result else set()
        self._freerouting_candidates_ready = bool(outcome.result)
        if outcome.elapsed_seconds is not None:
            print(
                f"freerouting_selector_candidate_runtime_sec = {outcome.elapsed_seconds:.3f}",
                flush=True,
            )
        print(
            "external_selector_candidate_status = "
            + ("ready" if self._freerouting_candidates_ready else "failed")
            + f"  # {outcome.message}",
            flush=True,
        )
        suffix = f" {truncated_count} net previews were truncated." if truncated_count else ""
        self.statusBar().showMessage(
            f"Showing external ripped routes for {len(ripped_net_ids)} unique nets "
            f"across {len(previews)} occurrences, "
            f"{segment_count + final_only_segment_count} segments, "
            f"{via_count + final_only_via_count} vias; "
            f"{len(selector_net_ids)} selector nets.{suffix}"
        )
        self._update_ripup_buttons()

    def _build_selected_preview_results(self, route_results, selection) -> list[object]:
        display_offset = self._selector_to_display_offset_mm()
        selected_by_net: dict[int, list[int]] = {}
        for net_selection in selection.selections:
            if not net_selection.selected_candidate_indices:
                continue
            selected_by_net[int(net_selection.net_id)] = [
                int(index) for index in net_selection.selected_candidate_indices
            ]

        preview_results: list[object] = []
        for route_result in route_results:
            net_id = int(getattr(route_result, "net_id", 0))
            candidate_preview_items = list(getattr(route_result, "candidate_preview_items", []))
            candidate_grid = list(getattr(route_result, "candidate_paths_grid", []))
            candidate_mm = list(getattr(route_result, "candidate_paths_mm", []))
            selected_indices = selected_by_net.get(net_id)
            if not selected_indices:
                continue
            for selected_index in selected_indices:
                if selected_index < 0:
                    continue
                if selected_index < len(candidate_preview_items):
                    preview_results.append(
                        self._translate_route_preview_for_display(candidate_preview_items[selected_index], display_offset)
                    )
                    continue
                if selected_index >= len(candidate_grid) or selected_index >= len(candidate_mm):
                    continue
                preview_results.append(
                    self._translate_route_preview_for_display(
                        SimpleNamespace(
                            net_id=net_id,
                            candidate_paths_grid=[candidate_grid[selected_index]],
                            candidate_paths_mm=[candidate_mm[selected_index]],
                        ),
                        display_offset,
                    )
                )
        return preview_results

    def _selector_to_display_offset_mm(self) -> tuple[float, float]:
        """Return the coordinate offset from selector-board space to display-board space.

        External routers may run on a sibling `.unrouted` board while the GUI
        shows a routed reference board. Candidate generation must stay in the
        selector board's coordinate frame, but preview overlays need this pure
        display translation so pads and route geometry line up visually.
        """
        if self._original_board is None or self._board is None:
            return (0.0, 0.0)
        if self._original_board is self._board or self._original_board.path == self._board.path:
            return (0.0, 0.0)
        try:
            return _board_translation_from_common_pads(self._original_board, self._board)
        except ValueError as exc:
            print(f"external_preview_translation_status = unavailable reason={exc}", flush=True)
            return (0.0, 0.0)

    def _translate_route_previews_for_display(self, previews: list[object], offset: tuple[float, float]) -> list[object]:
        """Translate a list of preview objects into the currently displayed board frame."""
        if offset == (0.0, 0.0):
            return previews
        return [self._translate_route_preview_for_display(preview, offset) for preview in previews]

    def _translate_route_preview_for_display(self, preview: object, offset: tuple[float, float]) -> object:
        """Translate preview geometry without mutating selector-owned candidate data."""
        dx, dy = offset
        if dx == 0.0 and dy == 0.0:
            return preview

        translated = SimpleNamespace(**getattr(preview, "__dict__", {}))
        if hasattr(preview, "net_id"):
            translated.net_id = getattr(preview, "net_id")
        if hasattr(preview, "net_name"):
            translated.net_name = getattr(preview, "net_name")
        if hasattr(preview, "truncated"):
            translated.truncated = getattr(preview, "truncated")
        if hasattr(preview, "occurrence_index"):
            translated.occurrence_index = getattr(preview, "occurrence_index")
        if hasattr(preview, "source_net_id"):
            translated.source_net_id = getattr(preview, "source_net_id")
        if hasattr(preview, "source"):
            translated.source = getattr(preview, "source")
        if hasattr(preview, "preview_kind"):
            translated.preview_kind = getattr(preview, "preview_kind")

        if hasattr(preview, "segments"):
            translated.segments = [
                _translated_preview_segment(segment, dx, dy)
                for segment in list(getattr(preview, "segments", []))
            ]
        if hasattr(preview, "vias"):
            translated.vias = [
                _translated_preview_via(via, dx, dy)
                for via in list(getattr(preview, "vias", []))
            ]
        if hasattr(preview, "candidate_paths_mm"):
            translated.candidate_paths_mm = [
                [_translated_preview_point(point, dx, dy) for point in path]
                for path in list(getattr(preview, "candidate_paths_mm", []))
            ]
        if hasattr(preview, "path_mm"):
            translated.path_mm = [
                _translated_preview_point(point, dx, dy)
                for point in list(getattr(preview, "path_mm", []))
            ]
        return translated

    def _set_layers(self, layers: list[str]) -> None:
        self.layer_combo.blockSignals(True)
        self.layer_combo.clear()
        self.layer_combo.addItems(layers or ["F.Cu"])
        self.layer_combo.blockSignals(False)
        if layers:
            self.canvas.set_current_layer(layers[0])

    def _estimate_failed_nets(self, board: BoardData) -> list[int]:
        pad_counts: dict[int, int] = {}
        for footprint in board.footprints:
            for pad in footprint.pads:
                if pad.net_id is None or pad.net_id <= 0:
                    continue
                pad_counts[pad.net_id] = pad_counts.get(pad.net_id, 0) + 1

        nets_with_copper: set[int] = set()
        for track in board.tracks:
            if track.net_id > 0:
                nets_with_copper.add(track.net_id)
        for via in board.vias:
            if via.net_id > 0:
                nets_with_copper.add(via.net_id)

        return sorted(
            net_id
            for net_id, count in pad_counts.items()
            if count >= 2 and net_id not in nets_with_copper
        )


class TracePanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(260)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            "QFrame { background: #f8f9fb; }"
            "QLabel { color: #20242b; font-size: 13px; }"
            "QLabel#Title { font-size: 18px; font-weight: 700; }"
        )

        layout = QVBoxLayout(self)
        title = QLabel("Trace")
        title.setObjectName("Title")
        layout.addWidget(title)

        form = QFormLayout()
        self.file_label = QLabel("-")
        self.net_id_label = QLabel("-")
        self.net_name_label = QLabel("-")
        self.layer_label = QLabel("-")
        self.segment_count_label = QLabel("-")
        self.width_label = QLabel("-")
        self.grid_steps_spin = QSpinBox()
        self.grid_steps_spin.setRange(1, 100)
        self.grid_steps_spin.setValue(10)
        self.grid_steps_spin.setSuffix(" grid/mm")
        self.selected_list = QListWidget()
        self.selected_list.setMinimumHeight(120)
        self.selected_list.setMaximumHeight(180)
        self.selected_list.setStyleSheet(
            "QListWidget { background: #ffffff; border: 1px solid #c8ced8; }"
            "QListWidget::item { padding: 3px 4px; }"
        )
        self.ripped_list = QListWidget()
        self.ripped_list.setMinimumHeight(120)
        self.ripped_list.setMaximumHeight(180)
        self.ripped_list.setStyleSheet(
            "QListWidget { background: #fff8f0; border: 1px solid #d7b99a; }"
            "QListWidget::item { padding: 3px 4px; }"
        )

        form.addRow("File", self.file_label)
        form.addRow("Selected nets", self.selected_list)
        form.addRow("Ripped-up nets", self.ripped_list)
        form.addRow("Net ID", self.net_id_label)
        form.addRow("Net name", self.net_name_label)
        form.addRow("Clicked layer", self.layer_label)
        form.addRow("Net trace count", self.segment_count_label)
        form.addRow("Clicked width", self.width_label)
        form.addRow("Grid density", self.grid_steps_spin)
        layout.addLayout(form)
        layout.addStretch(1)

    @property
    def grid_steps_per_mm(self) -> float:
        return float(self.grid_steps_spin.value())

    def set_grid_density_minimum(self, minimum: int) -> None:
        current = self.grid_steps_spin.value()
        self.grid_steps_spin.setMinimum(max(1, minimum))
        self.grid_steps_spin.setValue(max(current, self.grid_steps_spin.minimum()))

    def show_board(self, board: BoardData) -> None:
        self.file_label.setText(board.path.name)
        self.net_id_label.setText("-")
        self.net_name_label.setText("Click a trace")
        self.layer_label.setText("-")
        self.segment_count_label.setText(str(len(board.tracks)))
        self.width_label.setText("-")
        self.selected_list.clear()
        self.ripped_list.clear()
        self.selected_list.addItem("No nets selected")
        self.ripped_list.addItem("No nets ripped up")

    def show_trace(self, board: BoardData, track: TrackSegment) -> None:
        count = sum(1 for candidate in board.tracks if candidate.net_id == track.net_id)
        self.net_id_label.setText(str(track.net_id))
        self.net_name_label.setText(track.net_name)
        self.layer_label.setText(track.layer)
        self.segment_count_label.setText(str(count))
        self.width_label.setText(f"{track.width:g} mm")

    def show_selection(self, board: BoardData, selected_net_ids: set[int]) -> None:
        self.selected_list.clear()
        if not selected_net_ids:
            self.selected_list.addItem("No nets selected")
            return

        for net_id in sorted(selected_net_ids):
            self.selected_list.addItem(f"{net_id}: {board.nets.get(net_id, f'Net {net_id}')}")

    def show_ripped_nets(self, board: BoardData, ripped_net_ids: set[int]) -> None:
        self.ripped_list.clear()
        if not ripped_net_ids:
            self.ripped_list.addItem("No nets ripped up")
            return

        for net_id in sorted(ripped_net_ids):
            self.ripped_list.addItem(f"{net_id}: {board.nets.get(net_id, f'Net {net_id}')}")


def _board_translation_from_common_pads(source_board: BoardData, target_board: BoardData) -> tuple[float, float]:
    """Calculate a constant source-to-target translation from matching pad centers."""
    source_pads = _pad_centers_by_reference(source_board)
    target_pads = _pad_centers_by_reference(target_board)
    common_keys = sorted(set(source_pads) & set(target_pads))
    if len(common_keys) < 2:
        raise ValueError("too_few_common_pads")

    offsets = [
        (
            target_pads[key][0] - source_pads[key][0],
            target_pads[key][1] - source_pads[key][1],
        )
        for key in common_keys
    ]
    dx = _median(value[0] for value in offsets)
    dy = _median(value[1] for value in offsets)
    max_deviation = max(max(abs(item_dx - dx), abs(item_dy - dy)) for item_dx, item_dy in offsets)
    if max_deviation > 0.01:
        raise ValueError(f"not_simple_translation max_deviation_mm={max_deviation:.6f}")
    return dx, dy


def _pad_centers_by_reference(board: BoardData) -> dict[tuple[str, str], tuple[float, float]]:
    """Index pad centers by footprint reference and pad name for board-frame matching."""
    centers: dict[tuple[str, str], tuple[float, float]] = {}
    for footprint in board.footprints:
        for pad in footprint.pads:
            centers[(footprint.reference, pad.name)] = pad.center
    return centers


def _median(values: object) -> float:
    """Return the median value from a finite numeric iterable."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("empty_values")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _format_wall_timestamp(value: datetime) -> str:
    """Format a wall-clock timestamp with seconds and timezone offset."""
    return value.isoformat(timespec="seconds")


def _translated_preview_segment(segment: object, dx: float, dy: float) -> SimpleNamespace:
    """Return a display-only segment translated by the board-frame offset."""
    return SimpleNamespace(
        layer=str(getattr(segment, "layer", "F.Cu")),
        start=_translated_xy_tuple(getattr(segment, "start", (0.0, 0.0)), dx, dy),
        end=_translated_xy_tuple(getattr(segment, "end", (0.0, 0.0)), dx, dy),
        width=float(getattr(segment, "width", 0.2)),
    )


def _translated_preview_via(via: object, dx: float, dy: float) -> SimpleNamespace:
    """Return a display-only via translated by the board-frame offset."""
    return SimpleNamespace(
        center=_translated_xy_tuple(getattr(via, "center", (0.0, 0.0)), dx, dy),
        diameter=float(getattr(via, "diameter", 0.6)),
        start_layer=str(getattr(via, "start_layer", "F.Cu")),
        end_layer=str(getattr(via, "end_layer", "B.Cu")),
    )


def _translated_preview_point(point: object, dx: float, dy: float) -> SimpleNamespace:
    """Return a display-only path point translated by the board-frame offset."""
    if isinstance(point, (tuple, list)) and len(point) >= 2:
        x = float(point[0])
        y = float(point[1])
    else:
        x = float(getattr(point, "x", 0.0))
        y = float(getattr(point, "y", 0.0))
    return SimpleNamespace(
        x=x + dx,
        y=y + dy,
    )


def _translated_xy_tuple(value: object, dx: float, dy: float) -> tuple[float, float]:
    """Translate a two-value coordinate tuple by the board-frame offset."""
    x, y = value
    return (float(x) + dx, float(y) + dy)

