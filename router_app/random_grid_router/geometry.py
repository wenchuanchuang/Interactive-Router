from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
import re

from router_app.kicad_parser import BoardData, Pad


@dataclass(frozen=True)
class PadInfo:
    """Simplified pad geometry used by the grid router."""

    pad: Pad
    footprint_ref: str
    layers: tuple[str, ...]
    bbox: tuple[float, float, float, float]


DIRS: tuple[tuple[int, int], ...] = (
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
)


def collect_pad_infos(board: BoardData) -> list[PadInfo]:
    """Collect pads with rotated bbox approximations for routing keepouts."""
    infos: list[PadInfo] = []
    for footprint in board.footprints:
        for pad in footprint.pads:
            layers = tuple(_normalize_layer(layer) for layer in pad.layers)
            infos.append(
                PadInfo(
                    pad=pad,
                    footprint_ref=footprint.reference,
                    layers=layers,
                    bbox=_rotated_pad_bbox(pad),
                )
            )
    return infos


def edge_or_pad_bounds(board: BoardData, pad_infos: list[PadInfo]) -> tuple[float, float, float, float]:
    """Use Edge.Cuts bbox when present, otherwise fall back to pad bbox."""
    edge_bounds = _edge_cut_bounds(board.path)
    if edge_bounds is not None:
        return edge_bounds
    if not pad_infos:
        return (0.0, 0.0, 1.0, 1.0)
    min_x = min(info.bbox[0] for info in pad_infos)
    min_y = min(info.bbox[1] for info in pad_infos)
    max_x = max(info.bbox[2] for info in pad_infos)
    max_y = max(info.bbox[3] for info in pad_infos)
    return (min_x, min_y, max_x, max_y)


def point_in_expanded_bbox(x: float, y: float, bbox: tuple[float, float, float, float], expand: float) -> bool:
    """Return whether a point lies inside an expanded axis-aligned bbox."""
    min_x, min_y, max_x, max_y = bbox
    return min_x - expand <= x <= max_x + expand and min_y - expand <= y <= max_y + expand


def direction_delta(first: int, second: int) -> int:
    """Return circular direction-bin distance on the 8-neighbor ring."""
    delta = abs(int(first) - int(second))
    return min(delta, 8 - delta)


def _normalize_layer(layer: str) -> str:
    """Normalize legacy layer aliases used in old KiCad boards."""
    if layer == "Top":
        return "F.Cu"
    if layer == "Bottom":
        return "B.Cu"
    return str(layer)


def _rotated_pad_bbox(pad: Pad) -> tuple[float, float, float, float]:
    """Approximate a pad as its rotated axis-aligned bounding box."""
    cx, cy = pad.center
    width, height = pad.size
    angle = radians(float(pad.rotation_degrees or 0.0))
    c = cos(angle)
    s = sin(angle)
    corners = []
    for dx in (-width * 0.5, width * 0.5):
        for dy in (-height * 0.5, height * 0.5):
            corners.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    return (min(xs), min(ys), max(xs), max(ys))


def _edge_cut_bounds(path: Path) -> tuple[float, float, float, float] | None:
    """Parse a simple Edge.Cuts bounding box from KiCad board text."""
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return None
    points: list[tuple[float, float]] = []
    for item in _top_level_items(text):
        if "Edge.Cuts" not in item:
            continue
        for match in re.finditer(r"\(\s*(?:start|end|center|mid|at|xy)\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)", item):
            points.append((float(match.group(1)), float(match.group(2))))
    if len(points) < 2:
        return None
    return (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )


def _top_level_items(text: str) -> list[str]:
    """Return top-level KiCad S-expression items without a full parser."""
    items: list[str] = []
    pattern = re.compile(r"(?m)^\s{2}\(")
    for match in pattern.finditer(text):
        start = match.start()
        depth = 0
        in_string = False
        escaped = False
        for index in range(start, len(text)):
            char = text[index]
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
                continue
            if char == '"':
                in_string = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    items.append(text[start:index + 1])
                    break
    return items
