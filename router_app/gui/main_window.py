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
    QMainWindow,
    QMessageBox,
    QSplitter,
    QToolBar,
    QVBoxLayout,
)

from router_app.kicad_parser import BoardData, TrackSegment, load_board
from router_app.gui.pcb_canvas import PcbCanvas


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
        left_column.addWidget(self.canvas)

        lower_placeholder = QLabel("Routing workspace")
        lower_placeholder.setAlignment(Qt.AlignCenter)
        lower_placeholder.setMinimumHeight(220)
        lower_placeholder.setStyleSheet("color: #8d96a5; background: #101318;")
        left_column.addWidget(lower_placeholder)
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
        pad_count = sum(len(footprint.pads) for footprint in board.footprints)
        self.statusBar().showMessage(
            f"Loaded {board.path.name} via {board.backend}: {len(board.tracks)} traces, "
            f"{len(board.footprints)} components, {pad_count} pads, "
            f"{len(board.two_pin_net_ids)} two-pin nets"
        )

    def _show_trace(self, track: TrackSegment) -> None:
        if self._board is not None:
            self.trace_panel.show_trace(self._board, track)

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

        form.addRow("File", self.file_label)
        form.addRow("Net ID", self.net_id_label)
        form.addRow("Net name", self.net_name_label)
        form.addRow("Clicked layer", self.layer_label)
        form.addRow("Net trace count", self.segment_count_label)
        form.addRow("Clicked width", self.width_label)
        layout.addLayout(form)
        layout.addStretch(1)

    def show_board(self, board: BoardData) -> None:
        self.file_label.setText(board.path.name)
        self.net_id_label.setText("-")
        self.net_name_label.setText("Click a trace")
        self.layer_label.setText("-")
        self.segment_count_label.setText(str(len(board.tracks)))
        self.width_label.setText("-")

    def show_trace(self, board: BoardData, track: TrackSegment) -> None:
        count = sum(1 for candidate in board.tracks if candidate.net_id == track.net_id)
        self.net_id_label.setText(str(track.net_id))
        self.net_name_label.setText(track.net_name)
        self.layer_label.setText(track.layer)
        self.segment_count_label.setText(str(count))
        self.width_label.setText(f"{track.width:g} mm")
