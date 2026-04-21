from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt
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

from router_app.kicad_parser import BoardData, TrackSegment, load_board
from router_app.reroute_engine import minimum_grid_steps_per_mm, run_dijkstra_reroute_test
from router_app.gui.pcb_canvas import PcbCanvas, RoutePreviewCanvas


class MainWindow(QMainWindow):
    def __init__(self, initial_file: str | None = None):
        super().__init__()
        self._board: BoardData | None = None

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
        try:
            board = load_board(file_name)
        except Exception as exc:
            QMessageBox.critical(self, "Could not open board", str(exc))
            return

        self._board = board
        self.canvas.load_board(board)
        self._set_layers(board.copper_layers)
        self.trace_panel.show_board(board)
        self.trace_panel.set_grid_density_minimum(minimum_grid_steps_per_mm(board))
        self._update_ripup_buttons()
        pad_count = sum(len(footprint.pads) for footprint in board.footprints)
        self.statusBar().showMessage(
            f"Loaded {board.path.name} via {board.backend}: {len(board.tracks)} traces, "
            f"{len(board.vias)} vias, {len(board.footprints)} components, {pad_count} pads, "
            f"{len(board.two_pin_net_ids)} two-pin nets"
        )

    def _show_trace(self, track: TrackSegment) -> None:
        if self._board is not None:
            self.trace_panel.show_trace(self._board, track)

    def _show_selected_nets(self, selected_net_ids: set[int]) -> None:
        if self._board is not None:
            self.trace_panel.show_selection(self._board, selected_net_ids)

    def _rip_up_selected(self) -> None:
        if self.canvas.rip_up_selected() and self._board is not None:
            self.trace_panel.show_selection(self._board, set())
            self.trace_panel.show_ripped_nets(self._board, self.canvas.ripped_net_ids)
            self._show_status_counts("Rip-up applied")

    def _undo_rip_up(self) -> None:
        if self.canvas.undo_rip_up():
            if self._board is not None:
                self.trace_panel.show_ripped_nets(self._board, self.canvas.ripped_net_ids)
            self._show_status_counts("Rip-up undone")

    def _update_ripup_buttons(self, *_args) -> None:
        self.rip_up_button.setEnabled(bool(self.canvas.selected_net_ids))
        self.undo_rip_up_button.setEnabled(self.canvas.can_undo_rip_up)
        self.reroute_button.setEnabled(bool(self.canvas.ripped_net_ids))

    def _show_ripped_nets(self, ripped_net_ids: set[int]) -> None:
        if self._board is not None:
            self.trace_panel.show_ripped_nets(self._board, ripped_net_ids)
        self._update_ripup_buttons()

    def _reroute_selected_removed_nets(self) -> None:
        if self._board is None:
            return
        outcome = run_dijkstra_reroute_test(
            self._board,
            self.canvas.ripped_net_ids,
            self.trace_panel.grid_steps_per_mm,
        )
        self.statusBar().showMessage(outcome.message)
        if outcome.result:
            self.route_preview.show_routes(outcome.result, self._board, self.canvas.ripped_net_ids)
        else:
            self.route_preview.show_message(outcome.message)

    def _show_status_counts(self, prefix: str) -> None:
        if self._board is None:
            return
        self.statusBar().showMessage(
            f"{prefix}: {len(self.canvas.selected_net_ids)} selected nets, "
            f"{len(self._board.tracks)} original traces"
        )

    def _set_layers(self, layers: list[str]) -> None:
        self.layer_combo.blockSignals(True)
        self.layer_combo.clear()
        self.layer_combo.addItems(layers or ["F.Cu"])
        self.layer_combo.blockSignals(False)
        if layers:
            self.canvas.set_current_layer(layers[0])


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
