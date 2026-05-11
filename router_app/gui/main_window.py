from __future__ import annotations

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
from router_app.reroute_engine import (
    RerouteOutcome,
    build_freerouting_external_selector_outcome,
    minimum_grid_steps_per_mm,
    run_dijkstra_reroute_test,
    select_reroute_candidates,
)
from router_app.gui.pcb_canvas import PcbCanvas, RoutePreviewCanvas


class MainWindow(QMainWindow):
    def __init__(self, initial_file: str | None = None, freerouting_full: bool = False):
        super().__init__()
        self._board: BoardData | None = None
        self._original_board: BoardData | None = None
        self._candidate_outcome = None
        self._candidate_ripped_net_ids: set[int] = set()
        self._freerouting_full_enabled = freerouting_full
        self._freerouting_run: FreeroutingRunResult | None = None
        self._freerouting_candidates_ready = False

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
        left_column.addWidget(self.route_preview)
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
        if self._freerouting_full_enabled:
            self._open_board_with_freerouting(file_name)
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

    def _open_board_with_freerouting(self, file_name: str) -> None:
        try:
            result = run_freerouting_full(file_name)
        except Exception as exc:
            QMessageBox.critical(self, "freerouting-full failed", str(exc))
            return

        self._freerouting_run = result
        self._original_board = result.original_board
        self._board = result.routed_board
        self._reset_candidate_generation_state()
        self.canvas.load_board(result.routed_board)
        self._set_layers(result.routed_board.copper_layers)
        self.trace_panel.show_board(result.routed_board)
        self.trace_panel.set_grid_density_minimum(minimum_grid_steps_per_mm(result.routed_board))
        self._update_ripup_buttons()

        pad_count = sum(len(footprint.pads) for footprint in result.routed_board.footprints)
        self.statusBar().showMessage(
            f"Loaded {result.routed_board.path.name} via {result.routed_board.backend}: "
            f"{len(result.routed_board.tracks)} traces, {len(result.routed_board.vias)} vias, "
            f"{len(result.routed_board.footprints)} components, {pad_count} pads, "
            f"{len(result.routed_board.two_pin_net_ids)} two-pin nets, "
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
        if self._freerouting_full_enabled:
            self.rip_up_button.setEnabled(False)
            self.undo_rip_up_button.setEnabled(False)
            self.generate_candidates_button.setEnabled(bool(self._freerouting_run and self._freerouting_run.ripup_previews))
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
        if self._freerouting_full_enabled:
            self._show_freerouting_ripped_routes()
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
        if self._freerouting_full_enabled:
            if self._board is None or self._freerouting_run is None:
                self.statusBar().showMessage("freerouting-full result is not available.")
                return
            if self._candidate_outcome is None or not self._candidate_outcome.result:
                self.statusBar().showMessage("Generate candidates first, then reroute.")
                return

            selection = select_reroute_candidates(self._candidate_outcome, max_paths_per_net=1, prefer_gurobi=True)
            if not selection.ok:
                self.statusBar().showMessage(f"Path selection failed: {selection.message}")
                return
            if str(selection.solver).lower() != "gurobi":
                self.statusBar().showMessage(
                    f"Reroute skipped: solver={selection.solver} (not gurobi). {selection.message}"
                )
                return

            ripped_net_ids = {item.net_id for item in self._freerouting_run.ripup_previews}
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

        selection = select_reroute_candidates(self._candidate_outcome, max_paths_per_net=1, prefer_gurobi=True)
        if not selection.ok:
            self.statusBar().showMessage(f"Path selection failed: {selection.message}")
            return
        if str(selection.solver).lower() != "gurobi":
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
        self._freerouting_candidates_ready = False
        if self._freerouting_full_enabled:
            self.route_preview.show_message("Generate candidates to show freerouting ripped routes")
        else:
            self.route_preview.show_message("Generate candidates first")

    def _show_freerouting_ripped_routes(self) -> None:
        if self._board is None or self._freerouting_run is None:
            self.route_preview.show_message("freerouting-full result is not available")
            return
        if not self._freerouting_run.ripup_previews:
            self.route_preview.show_message("freerouting did not report ripped routes")
            self.statusBar().showMessage("freerouting completed without ripped-route preview data.")
            return

        ripped_net_ids = {item.net_id for item in self._freerouting_run.ripup_previews}
        overlay_net_ids = []
        for item in self._freerouting_run.ripup_previews:
            if item.occurrence_index > 0:
                overlay_net_ids.append(f"{item.net_id}@E{item.occurrence_index}")
            else:
                overlay_net_ids.append(str(item.net_id))
        segment_count = sum(len(item.segments) for item in self._freerouting_run.ripup_previews)
        via_count = sum(len(item.vias) for item in self._freerouting_run.ripup_previews)
        truncated_count = sum(1 for item in self._freerouting_run.ripup_previews if item.truncated)
        truncated_labels = []
        for item in self._freerouting_run.ripup_previews:
            if not item.truncated:
                continue
            if item.occurrence_index > 0:
                truncated_labels.append(f"{item.net_id}@E{item.occurrence_index}")
            else:
                truncated_labels.append(str(item.net_id))
        self.trace_panel.show_ripped_nets(self._board, ripped_net_ids)
        self.route_preview.show_routes(self._freerouting_run.ripup_previews, self._board, ripped_net_ids)
        print(
            "freerouting_ripped_occurrences = "
            + ", ".join(overlay_net_ids),
            flush=True,
        )
        print(
            f"freerouting_truncated_occurrence_count = {truncated_count}",
            flush=True,
        )
        print(
            "freerouting_truncated_occurrences = "
            + (", ".join(truncated_labels) if truncated_labels else "(none)"),
            flush=True,
        )
        selector_board = self._original_board or self._board
        outcome = build_freerouting_external_selector_outcome(
            selector_board,
            ripped_net_ids,
            self.trace_panel.grid_steps_per_mm,
        )
        if outcome.result:
            self._candidate_outcome = RerouteOutcome(
                ok=outcome.ok,
                message=outcome.message,
                result=outcome.result,
                elapsed_seconds=outcome.elapsed_seconds,
                candidate_export_path=outcome.candidate_export_path,
                selector_external_only=True,
            )
        else:
            self._candidate_outcome = None
        self._candidate_ripped_net_ids = set(ripped_net_ids) if outcome.result else set()
        self._freerouting_candidates_ready = bool(outcome.result)
        if outcome.elapsed_seconds is not None:
            print(
                f"freerouting_selector_candidate_runtime_sec = {outcome.elapsed_seconds:.3f}",
                flush=True,
            )
        print(
            "freerouting_selector_candidate_status = "
            + ("ready" if self._freerouting_candidates_ready else "failed")
            + f"  # {outcome.message}",
            flush=True,
        )
        suffix = f" {truncated_count} net previews were truncated." if truncated_count else ""
        self.statusBar().showMessage(
            f"Showing freerouting ripped routes for {len(ripped_net_ids)} unique nets "
            f"across {len(self._freerouting_run.ripup_previews)} occurrences, "
            f"{segment_count} segments, {via_count} vias.{suffix}"
        )
        self._update_ripup_buttons()

    def _build_selected_preview_results(self, route_results, selection) -> list[object]:
        selected_by_net: dict[int, int] = {}
        for net_selection in selection.selections:
            if not net_selection.selected_candidate_indices:
                continue
            selected_by_net[int(net_selection.net_id)] = int(net_selection.selected_candidate_indices[0])

        preview_results: list[object] = []
        for route_result in route_results:
            net_id = int(getattr(route_result, "net_id", 0))
            candidate_grid = list(getattr(route_result, "candidate_paths_grid", []))
            candidate_mm = list(getattr(route_result, "candidate_paths_mm", []))
            selected_index = selected_by_net.get(net_id)
            if selected_index is None or selected_index < 0:
                continue
            if selected_index >= len(candidate_grid) or selected_index >= len(candidate_mm):
                continue
            preview_results.append(
                SimpleNamespace(
                    net_id=net_id,
                    candidate_paths_grid=[candidate_grid[selected_index]],
                    candidate_paths_mm=[candidate_mm[selected_index]],
                )
            )
        return preview_results

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

