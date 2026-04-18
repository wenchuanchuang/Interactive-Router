from __future__ import annotations

from typing import Callable

from PyQt5.QtCore import QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsView,
)

from router_app.kicad_parser import BoardData, Footprint, Pad, TrackSegment


class TrackItem(QGraphicsLineItem):
    def __init__(self, track: TrackSegment, scale: float, on_clicked: Callable[[TrackSegment], None]):
        x1, y1 = track.start
        x2, y2 = track.end
        super().__init__(x1 * scale, y1 * scale, x2 * scale, y2 * scale)
        self.track = track
        self._scale = scale
        self._on_clicked = on_clicked
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.setZValue(1)
        self.apply_style(False, False, False)

    def apply_style(self, highlighted: bool, dimmed: bool, emphasized: bool) -> None:
        color = _layer_color(self.track.layer)
        if highlighted:
            color = _layer_highlight_color(self.track.layer)
        elif emphasized:
            color = _two_pin_color(self.track.layer)
        elif dimmed:
            color.setAlpha(55)

        visual_width = max(self.track.width * self._scale, 2.0)
        if emphasized and not highlighted:
            visual_width = max(visual_width + 2.5, 5.0)
        if highlighted:
            visual_width = max(visual_width + 4.5, 8.0)

        pen = QPen(color, visual_width, _layer_pen_style(self.track.layer), Qt.RoundCap, Qt.RoundJoin)
        self.setPen(pen)
        self.setZValue(_layer_z_value(self.track.layer) + (20 if highlighted else 0))

    def mousePressEvent(self, event) -> None:
        self._on_clicked(self.track)
        super().mousePressEvent(event)

    def hoverEnterEvent(self, event) -> None:
        self.setCursor(Qt.PointingHandCursor)
        super().hoverEnterEvent(event)


class PcbCanvas(QGraphicsView):
    traceSelected = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self._board: BoardData | None = None
        self._track_items: list[TrackItem] = []
        self._component_items: list[tuple[QGraphicsItem, tuple[str, ...]]] = []
        self._selected_net_id: int | None = None
        self._current_layer = "F.Cu"
        self._show_all_layers = True
        self._show_components = True
        self._emphasize_two_pin_nets = True
        self._scale = 22.0

        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setBackgroundBrush(QBrush(QColor("#15181d")))
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)

    def load_board(self, board: BoardData) -> None:
        self._board = board
        self._selected_net_id = None
        layers = board.copper_layers
        if layers:
            self._current_layer = layers[0]
        self._track_items.clear()
        self._component_items.clear()
        self._scene.clear()

        for footprint in board.footprints:
            self._add_footprint(footprint)

        for track in board.tracks:
            item = TrackItem(track, self._scale, self._select_track)
            self._track_items.append(item)
            self._scene.addItem(item)

        self._refresh_visibility()
        self._refresh_highlight()
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-60, -60, 60, 60))
        self.fit_board()

    def set_current_layer(self, layer: str) -> None:
        self._current_layer = layer
        self._refresh_visibility()

    def set_show_all_layers(self, enabled: bool) -> None:
        self._show_all_layers = enabled
        self._refresh_visibility()

    def set_show_components(self, enabled: bool) -> None:
        self._show_components = enabled
        self._refresh_visibility()

    def set_emphasize_two_pin_nets(self, enabled: bool) -> None:
        self._emphasize_two_pin_nets = enabled
        self._refresh_highlight()

    def fit_board(self) -> None:
        rect = self._scene.itemsBoundingRect()
        if not rect.isNull():
            self.fitInView(rect.adjusted(-30, -30, 30, 30), Qt.KeepAspectRatio)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._board is not None:
            self.fit_board()

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def _select_track(self, track: TrackSegment) -> None:
        self._selected_net_id = track.net_id
        self._refresh_highlight()
        self.traceSelected.emit(track)

    def _refresh_highlight(self) -> None:
        two_pin_nets = self._board.two_pin_net_ids if self._board is not None else set()
        for item in self._track_items:
            highlighted = item.track.net_id == self._selected_net_id
            dimmed = self._selected_net_id is not None and not highlighted
            emphasized = self._emphasize_two_pin_nets and item.track.net_id in two_pin_nets
            item.apply_style(highlighted, dimmed, emphasized)

    def _refresh_visibility(self) -> None:
        for item in self._track_items:
            item.setVisible(self._show_all_layers or _same_layer(item.track.layer, self._current_layer))

        for item, layers in self._component_items:
            item.setVisible(
                self._show_components
                and (self._show_all_layers or _layers_include(layers, self._current_layer))
            )

    def _add_footprint(self, footprint: Footprint) -> None:
        pad_rects: list[QRectF] = []
        footprint_layers: set[str] = set()
        for pad in footprint.pads:
            item = self._pad_item(pad)
            footprint_layers.update(pad.layers)
            pad_rects.append(item.rect() if isinstance(item, QGraphicsRectItem) else item.rect())
            self._component_items.append((item, pad.layers))
            self._scene.addItem(item)

        if pad_rects:
            outline_rect = pad_rects[0]
            for rect in pad_rects[1:]:
                outline_rect = outline_rect.united(rect)
        else:
            x, y = footprint.position
            outline_rect = QRectF(x * self._scale - 14, y * self._scale - 14, 28, 28)

        if pad_rects or footprint.reference:
            outline = QGraphicsRectItem(outline_rect.adjusted(-8, -8, 8, 8))
            outline_pen = QPen(QColor("#f0f3f8"), 1.4, Qt.DashLine)
            outline_pen.setCosmetic(True)
            outline.setPen(outline_pen)
            fill = QColor("#eef1f7")
            fill.setAlpha(24)
            outline.setBrush(QBrush(fill))
            outline.setZValue(1.5)
            display_layers = tuple(footprint_layers) or _component_layers_from_footprint(footprint.layer)
            self._component_items.append((outline, display_layers))
            self._scene.addItem(outline)

        x, y = footprint.position
        label = QGraphicsTextItem(footprint.reference)
        label.setDefaultTextColor(QColor("#ffffff"))
        label.setScale(0.75)
        label.setPos(x * self._scale + 6, y * self._scale + 6)
        label.setZValue(12)
        display_layers = tuple(footprint_layers) or _component_layers_from_footprint(footprint.layer)
        self._component_items.append((label, display_layers))
        self._scene.addItem(label)

    def _pad_item(self, pad: Pad) -> QGraphicsItem:
        x, y = pad.center
        width, height = pad.size
        rect = QRectF(
            (x - width / 2) * self._scale,
            (y - height / 2) * self._scale,
            width * self._scale,
            height * self._scale,
        )
        if pad.shape in {"circle", "oval"}:
            item = QGraphicsEllipseItem(rect)
        else:
            item = QGraphicsRectItem(rect)
        item.setPen(QPen(QColor("#f4f5f7"), 1.0))
        item.setBrush(QBrush(_pad_color(pad.layers)))
        item.setZValue(2)
        return item


def _layer_color(layer: str) -> QColor:
    if layer == "F.Cu":
        return QColor("#f15a3b")
    if layer == "B.Cu":
        return QColor("#22b7ff")
    if layer.startswith("In") and layer.endswith(".Cu"):
        return QColor("#40c777")
    if layer.endswith(".Cu"):
        return QColor("#d8bd3f")
    return QColor("#c2c7d0")


def _layer_highlight_color(layer: str) -> QColor:
    if layer == "F.Cu":
        return QColor("#ffd21f")
    if layer == "B.Cu":
        return QColor("#00f0ff")
    if layer.startswith("In") and layer.endswith(".Cu"):
        return QColor("#7dff8f")
    if layer.endswith(".Cu"):
        return QColor("#ff9f43")
    return QColor("#c2c7d0")


def _two_pin_color(layer: str) -> QColor:
    if layer == "F.Cu":
        return QColor("#ff9b2f")
    if layer == "B.Cu":
        return QColor("#8ff7ff")
    if layer.startswith("In") and layer.endswith(".Cu"):
        return QColor("#baff6d")
    return QColor("#ffffff")


def _layer_pen_style(layer: str) -> Qt.PenStyle:
    if layer == "B.Cu":
        return Qt.DashLine
    if layer.startswith("In") and layer.endswith(".Cu"):
        return Qt.DotLine
    return Qt.SolidLine


def _layer_z_value(layer: str) -> float:
    if layer == "F.Cu":
        return 6
    if layer == "B.Cu":
        return 5
    if layer.startswith("In") and layer.endswith(".Cu"):
        return 4
    return 3


def _pad_color(layers: tuple[str, ...]) -> QColor:
    if "F.Cu" in layers:
        color = QColor("#9b5b57")
    elif "B.Cu" in layers:
        color = QColor("#4d6f9d")
    else:
        color = QColor("#757b84")
    color.setAlpha(150)
    return color


def _layers_include(layers: tuple[str, ...], current_layer: str) -> bool:
    return "*.Cu" in layers or current_layer in layers


def _same_layer(layer: str, current_layer: str) -> bool:
    return layer == current_layer


def _component_layers_from_footprint(layer: str) -> tuple[str, ...]:
    if layer.startswith("B."):
        return ("B.Cu",)
    if layer.startswith("F."):
        return ("F.Cu",)
    return ("F.Cu", "B.Cu")
