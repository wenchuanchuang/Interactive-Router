from __future__ import annotations

from math import cos, sin
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

from router_app.kicad_parser import BoardData, Footprint, Pad, TrackSegment, Via


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


class ViaItem(QGraphicsEllipseItem):
    def __init__(self, via: Via, scale: float, on_clicked: Callable[[int], None]):
        radius = max(via.diameter * scale * 0.5, 3.0)
        x, y = via.center
        super().__init__(x * scale - radius, y * scale - radius, radius * 2.0, radius * 2.0)
        self.via = via
        self._on_clicked = on_clicked
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.setZValue(8)
        self.apply_style(False, False)

    def apply_style(self, highlighted: bool, dimmed: bool) -> None:
        if highlighted:
            pen = QPen(QColor("#ffffff"), 2.4)
            brush = QBrush(QColor("#ffd21f"))
            self.setZValue(28)
        else:
            color = QColor("#dfe6ef")
            if dimmed:
                color.setAlpha(70)
            pen = QPen(color, 1.4)
            brush_color = QColor("#2d343d")
            brush_color.setAlpha(210 if not dimmed else 80)
            brush = QBrush(brush_color)
            self.setZValue(8)
        pen.setCosmetic(True)
        self.setPen(pen)
        self.setBrush(brush)

    def mousePressEvent(self, event) -> None:
        self._on_clicked(self.via.net_id)
        super().mousePressEvent(event)

    def hoverEnterEvent(self, event) -> None:
        self.setCursor(Qt.PointingHandCursor)
        super().hoverEnterEvent(event)


class PcbCanvas(QGraphicsView):
    traceSelected = pyqtSignal(object)
    netSelectionChanged = pyqtSignal(object)
    ripupStateChanged = pyqtSignal(int)
    rippedNetsChanged = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self._board: BoardData | None = None
        self._track_items: list[TrackItem] = []
        self._via_items: list[ViaItem] = []
        self._component_items: list[tuple[QGraphicsItem, tuple[str, ...]]] = []
        self._selected_net_ids: set[int] = set()
        self._ripped_net_ids: set[int] = set()
        self._ripup_history: list[set[int]] = []
        self._current_layer = "F.Cu"
        self._show_all_layers = True
        self._show_components = True
        self._emphasize_two_pin_nets = True
        self._scale = 22.0
        self._zoom_level = 1.0

        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setBackgroundBrush(QBrush(QColor("#15181d")))
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)

    def load_board(self, board: BoardData) -> None:
        self._board = board
        self._selected_net_ids.clear()
        self._ripped_net_ids.clear()
        self._ripup_history.clear()
        layers = board.copper_layers
        if layers:
            self._current_layer = layers[0]
        self._track_items.clear()
        self._via_items.clear()
        self._component_items.clear()
        self._scene.clear()

        for footprint in board.footprints:
            self._add_footprint(footprint)

        for track in board.tracks:
            item = TrackItem(track, self._scale, self._select_track)
            self._track_items.append(item)
            self._scene.addItem(item)

        for via in board.vias:
            item = ViaItem(via, self._scale, self._toggle_net_selection)
            self._via_items.append(item)
            self._scene.addItem(item)

        self._refresh_visibility()
        self._refresh_highlight()
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-60, -60, 60, 60))
        self.fit_board()
        self.netSelectionChanged.emit(set(self._selected_net_ids))
        self.ripupStateChanged.emit(len(self._ripup_history))
        self.rippedNetsChanged.emit(set(self._ripped_net_ids))

    @property
    def selected_net_ids(self) -> set[int]:
        return set(self._selected_net_ids)

    @property
    def ripped_net_ids(self) -> set[int]:
        return set(self._ripped_net_ids)

    @property
    def can_undo_rip_up(self) -> bool:
        return bool(self._ripup_history)

    def rip_up_selected(self) -> bool:
        if not self._selected_net_ids:
            return False

        ripped_now = set(self._selected_net_ids)
        self._ripped_net_ids.update(ripped_now)
        self._ripup_history.append(ripped_now)
        self._selected_net_ids.clear()
        self._refresh_visibility()
        self._refresh_highlight()
        self.netSelectionChanged.emit(set(self._selected_net_ids))
        self.ripupStateChanged.emit(len(self._ripup_history))
        self.rippedNetsChanged.emit(set(self._ripped_net_ids))
        return True

    def undo_rip_up(self) -> bool:
        if not self._ripup_history:
            return False

        restored = self._ripup_history.pop()
        self._ripped_net_ids.difference_update(restored)
        self._refresh_visibility()
        self._refresh_highlight()
        self.ripupStateChanged.emit(len(self._ripup_history))
        self.rippedNetsChanged.emit(set(self._ripped_net_ids))
        return True

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
            self._zoom_level = 1.0
            self.fitInView(rect.adjusted(-30, -30, 30, 30), Qt.KeepAspectRatio)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._board is not None:
            self.fit_board()

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            delta = event.pixelDelta().y()
        if delta == 0:
            event.ignore()
            return

        factor = 1.0015 ** delta
        next_zoom = self._zoom_level * factor
        if next_zoom < 0.08 or next_zoom > 40.0:
            event.accept()
            return

        self._zoom_level = next_zoom
        self.scale(factor, factor)
        event.accept()

    def _select_track(self, track: TrackSegment) -> None:
        self._toggle_net_selection(track.net_id)
        self.traceSelected.emit(track)

    def _toggle_net_selection(self, net_id: int) -> None:
        if net_id in self._ripped_net_ids:
            return
        if net_id in self._selected_net_ids:
            self._selected_net_ids.remove(net_id)
        else:
            self._selected_net_ids.add(net_id)
        self._refresh_highlight()
        self.netSelectionChanged.emit(set(self._selected_net_ids))

    def _refresh_highlight(self) -> None:
        two_pin_nets = self._board.two_pin_net_ids if self._board is not None else set()
        for item in self._track_items:
            highlighted = item.track.net_id in self._selected_net_ids
            dimmed = bool(self._selected_net_ids) and not highlighted
            emphasized = self._emphasize_two_pin_nets and item.track.net_id in two_pin_nets
            item.apply_style(highlighted, dimmed, emphasized)
        for item in self._via_items:
            highlighted = item.via.net_id in self._selected_net_ids
            dimmed = bool(self._selected_net_ids) and not highlighted
            item.apply_style(highlighted, dimmed)

    def _refresh_visibility(self) -> None:
        for item in self._track_items:
            item.setVisible(
                item.track.net_id not in self._ripped_net_ids
                and (self._show_all_layers or _same_layer(item.track.layer, self._current_layer))
            )
        for item in self._via_items:
            item.setVisible(item.via.net_id not in self._ripped_net_ids)

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
            pad_rects.append(item.mapToScene(item.boundingRect()).boundingRect())
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
        rect = QRectF(-width * self._scale / 2, -height * self._scale / 2, width * self._scale, height * self._scale)
        if pad.shape in {"circle", "oval"}:
            item = QGraphicsEllipseItem(rect)
        else:
            item = QGraphicsRectItem(rect)
        item.setPos(x * self._scale, y * self._scale)
        item.setRotation(pad.rotation_degrees)
        item.setPen(QPen(QColor("#f4f5f7"), 1.0))
        item.setBrush(QBrush(_pad_color(pad.layers)))
        item.setZValue(2)
        return item


class RoutePreviewCanvas(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self._zoom_level = 1.0
        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setBackgroundBrush(QBrush(QColor("#101318")))
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)

    def show_message(self, message: str) -> None:
        self._scene.clear()
        text = QGraphicsTextItem(message)
        text.setDefaultTextColor(QColor("#8d96a5"))
        text.setPos(16, 16)
        self._scene.addItem(text)
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-16, -16, 120, 80))
        self._zoom_level = 1.0

    def show_route(
        self,
        route_result,
        board: BoardData | None = None,
        ripped_net_ids: set[int] | None = None,
    ) -> None:
        self.show_routes([route_result], board, ripped_net_ids)

    def show_routes(
        self,
        route_results,
        board: BoardData | None = None,
        ripped_net_ids: set[int] | None = None,
    ) -> None:
        self._scene.clear()
        route_paths = _preview_route_paths(route_results)
        if not route_paths:
            self.show_message("No route path")
            return

        ripped_net_ids = ripped_net_ids or set()
        scale = 22.0
        all_points = [point for _, _, points in route_paths for point in points]
        min_x, min_y = _preview_origin(all_points, board)

        if board is not None:
            self._draw_preview_board(board, ripped_net_ids, scale, min_x, min_y)

        halo_pen = QPen(QColor("#111318"), 9.0, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        for route_result, candidate_index, points in route_paths:
            candidate_alpha = 255 if candidate_index == 0 else 120
            for index, (start, end) in enumerate(zip(points, points[1:])):
                x1, y1 = _preview_point(start, scale, min_x, min_y)
                x2, y2 = _preview_point(end, scale, min_x, min_y)
                self._scene.addLine(x1, y1, x2, y2, halo_pen).setZValue(40)
                layer = _route_layer_for_segment(route_result, board, index)
                route_color = _layer_highlight_color(layer)
                route_color.setAlpha(candidate_alpha)
                route_pen = QPen(
                    route_color,
                    5.0 if candidate_index == 0 else 3.0,
                    _layer_pen_style(layer),
                    Qt.RoundCap,
                    Qt.RoundJoin,
                )
                self._scene.addLine(x1, y1, x2, y2, route_pen).setZValue(_layer_z_value(layer) + 41)
                if _route_changes_layer(route_result, index):
                    self._draw_route_layer_change_marker(x2, y2, layer)

        net_ids = ", ".join(str(int(getattr(result, "net_id", 0))) for result in route_results)
        label = QGraphicsTextItem(f"Nets {net_ids} | {len(route_paths)} candidates")
        label.setDefaultTextColor(QColor("#ffffff"))
        label.setPos(8, 8)
        label.setZValue(10)
        self._scene.addItem(label)
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-40, -40, 40, 40))
        self._zoom_level = 1.0
        self.fitInView(self._scene.itemsBoundingRect().adjusted(-30, -30, 30, 30), Qt.KeepAspectRatio)

    def _draw_route_layer_change_marker(self, x: float, y: float, layer: str) -> None:
        marker = QGraphicsEllipseItem(x - 6, y - 6, 12, 12)
        marker.setBrush(QBrush(_layer_highlight_color(layer)))
        pen = QPen(QColor("#ffffff"), 1.6)
        pen.setCosmetic(True)
        marker.setPen(pen)
        marker.setZValue(60)
        self._scene.addItem(marker)

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            delta = event.pixelDelta().y()
        if delta == 0:
            event.ignore()
            return

        factor = 1.0015 ** delta
        next_zoom = self._zoom_level * factor
        if next_zoom < 0.08 or next_zoom > 40.0:
            event.accept()
            return

        self._zoom_level = next_zoom
        self.scale(factor, factor)
        event.accept()

    def _draw_preview_board(
        self,
        board: BoardData,
        ripped_net_ids: set[int],
        scale: float,
        min_x: float,
        min_y: float,
    ) -> None:
        for footprint in board.footprints:
            for pad in footprint.pads:
                x, y = pad.center
                width, height = pad.size
                scaled_width = max(width * scale, 2.0)
                scaled_height = max(height * scale, 2.0)
                rect = QRectF(-scaled_width / 2, -scaled_height / 2, scaled_width, scaled_height)
                item = QGraphicsEllipseItem(rect) if pad.shape in {"circle", "oval"} else QGraphicsRectItem(rect)
                item.setPos((x - min_x) * scale, (y - min_y) * scale)
                item.setRotation(pad.rotation_degrees)
                pad_color = _pad_color(pad.layers)
                pad_color.setAlpha(65)
                item.setBrush(QBrush(pad_color))
                pen = QPen(QColor("#c7ced8"), 0.8)
                pen.setCosmetic(True)
                item.setPen(pen)
                item.setZValue(1)
                self._scene.addItem(item)

        for track in board.tracks:
            if track.net_id in ripped_net_ids:
                continue
            x1 = (track.start[0] - min_x) * scale
            y1 = (track.start[1] - min_y) * scale
            x2 = (track.end[0] - min_x) * scale
            y2 = (track.end[1] - min_y) * scale
            color = _layer_color(track.layer)
            color.setAlpha(95)
            pen = QPen(
                color,
                max(track.width * scale, 1.6),
                _layer_pen_style(track.layer),
                Qt.RoundCap,
                Qt.RoundJoin,
            )
            item = self._scene.addLine(x1, y1, x2, y2, pen)
            item.setZValue(_layer_z_value(track.layer))

        for via in board.vias:
            if via.net_id in ripped_net_ids:
                continue
            radius = max(via.diameter * scale * 0.5, 2.2)
            x = (via.center[0] - min_x) * scale
            y = (via.center[1] - min_y) * scale
            item = QGraphicsEllipseItem(x - radius, y - radius, radius * 2, radius * 2)
            item.setBrush(QBrush(QColor("#2d343d")))
            pen = QPen(QColor("#dfe6ef"), 1.0)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setZValue(8)
            self._scene.addItem(item)


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


def _preview_origin(points: list[object], board: BoardData | None) -> tuple[float, float]:
    xs = [_point_x(point) for point in points]
    ys = [_point_y(point) for point in points]
    if board is not None:
        for track in board.tracks:
            xs.extend([track.start[0], track.end[0]])
            ys.extend([track.start[1], track.end[1]])
        for footprint in board.footprints:
            for pad in footprint.pads:
                min_x, min_y, max_x, max_y = _rotated_pad_bounds(pad)
                xs.extend([min_x, max_x])
                ys.extend([min_y, max_y])
        for via in board.vias:
            radius = via.diameter * 0.5
            xs.extend([via.center[0] - radius, via.center[0] + radius])
            ys.extend([via.center[1] - radius, via.center[1] + radius])
    return min(xs), min(ys)


def _preview_point(point: object, scale: float, min_x: float, min_y: float) -> tuple[float, float]:
    return (_point_x(point) - min_x) * scale, (_point_y(point) - min_y) * scale


def _rotated_pad_bounds(pad: Pad) -> tuple[float, float, float, float]:
    x, y = pad.center
    half_x = pad.size[0] * 0.5
    half_y = pad.size[1] * 0.5
    angle = pad.rotation_degrees * 3.141592653589793 / 180.0
    cos_a = abs(cos(angle))
    sin_a = abs(sin(angle))
    extent_x = cos_a * half_x + sin_a * half_y
    extent_y = sin_a * half_x + cos_a * half_y
    return (x - extent_x, y - extent_y, x + extent_x, y + extent_y)


def _preview_route_paths(route_results) -> list[tuple[object, int, list[object]]]:
    route_paths: list[tuple[object, int, list[object]]] = []
    for route_result in route_results:
        candidate_paths = list(getattr(route_result, "candidate_paths_mm", []))
        if candidate_paths:
            for index, path in enumerate(candidate_paths):
                points = list(path)
                if len(points) >= 2:
                    route_paths.append((route_result, index, points))
        else:
            points = list(getattr(route_result, "path_mm", []))
            if len(points) >= 2:
                route_paths.append((route_result, 0, points))
    return route_paths


def _route_layer_for_segment(route_result: object, board: BoardData | None, index: int) -> str:
    layers = board.copper_layers if board is not None else []
    path_grid = list(getattr(route_result, "path_grid", []))
    if layers and index < len(path_grid):
        z = int(getattr(path_grid[index], "z", 0))
        if 0 <= z < len(layers):
            return layers[z]

    if board is not None:
        net_id = int(getattr(route_result, "net_id", 0))
        for track in board.tracks:
            if track.net_id == net_id:
                return track.layer
    return "F.Cu"


def _route_changes_layer(route_result: object, index: int) -> bool:
    path_grid = list(getattr(route_result, "path_grid", []))
    if index + 1 >= len(path_grid):
        return False
    return int(getattr(path_grid[index], "z", 0)) != int(getattr(path_grid[index + 1], "z", 0))


def _point_x(point: object) -> float:
    return float(getattr(point, "x"))


def _point_y(point: object) -> float:
    return float(getattr(point, "y"))


def _same_layer(layer: str, current_layer: str) -> bool:
    return layer == current_layer


def _component_layers_from_footprint(layer: str) -> tuple[str, ...]:
    if layer.startswith("B."):
        return ("B.Cu",)
    if layer.startswith("F."):
        return ("F.Cu",)
    return ("F.Cu", "B.Cu")
