from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, radians, sin
from pathlib import Path
import re
from typing import Any

from router_app.kicad_parser import BoardData, Footprint, Pad
from router_app.kicad_parser import (
    _canonical_layer,
    _child,
    _child_value,
    _number,
    _parse_sexpr,
    _top_level_nodes,
    _transform_point,
)

_CONTACT_BAND_TOLERANCE_MM = 0.001


@dataclass(frozen=True)
class SolderJumperBridge:
    """Normally-closed solder-jumper bridge geometry parsed from a KiCad board."""

    footprint_ref: str
    footprint_name: str
    description: str
    layer: str
    start: tuple[float, float]
    end: tuple[float, float]
    width: float
    endpoint_pads: tuple[Pad, Pad]

    @property
    def endpoint_net_ids(self) -> frozenset[int]:
        """Return the endpoint net ids that may legally touch this bridge inside pads."""
        return frozenset(int(pad.net_id) for pad in self.endpoint_pads if pad.net_id is not None and int(pad.net_id) > 0)

    @property
    def endpoint_net_names(self) -> frozenset[str]:
        """Return endpoint net names for matching localized KiCad DRC item text."""
        return frozenset(pad.net_name for pad in self.endpoint_pads if pad.net_name)

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        """Return the bridge line bounding box expanded by half the copper width."""
        radius = self.width * 0.5
        return (
            min(self.start[0], self.end[0]) - radius,
            min(self.start[1], self.end[1]) - radius,
            max(self.start[0], self.end[0]) + radius,
            max(self.start[1], self.end[1]) + radius,
        )


@dataclass(frozen=True)
class DrcTrackGeometry:
    """Track geometry looked up from KiCad DRC item UUIDs."""

    start: tuple[float, float]
    end: tuple[float, float]
    width: float
    layer: str
    net_name: str | None = None


def detect_solder_jumper_bridges(board: BoardData) -> list[SolderJumperBridge]:
    """Detect intentionally shorted solder-jumper copper bridges on a board."""
    try:
        text = Path(board.path).read_text(encoding="utf-8", errors="ignore")
        sexpr = _parse_sexpr(text)
    except Exception:
        return []

    footprints_by_ref = {footprint.reference: footprint for footprint in board.footprints}
    bridges: list[SolderJumperBridge] = []
    for node in list(_top_level_nodes(sexpr, "footprint")) + list(_top_level_nodes(sexpr, "module")):
        if len(node) < 2:
            continue
        footprint_name = str(node[1])
        description = str(_child_value(node, "descr") or "")
        if not _is_normally_closed_solder_jumper(footprint_name, description):
            continue

        ref = _footprint_reference_from_node(node)
        footprint = footprints_by_ref.get(ref or "")
        if footprint is None:
            continue
        raw_pad_net_names = _pad_net_names_from_node(node)
        endpoint_pads = tuple(_pad_with_fallback_net_name(pad, raw_pad_net_names) for pad in footprint.pads if _pad_has_electrical_net(pad, raw_pad_net_names))
        if len(endpoint_pads) < 2:
            continue

        at = _child(node, "at")
        fp_x = float(_number(at[1]) or 0.0) if at is not None and len(at) >= 3 else 0.0
        fp_y = float(_number(at[2]) or 0.0) if at is not None and len(at) >= 3 else 0.0
        fp_angle = float(_number(at[3]) or 0.0) if at is not None and len(at) >= 4 else 0.0
        for child in node[1:]:
            if not isinstance(child, list) or not child or child[0] != "fp_line":
                continue
            line = _parse_bridge_fp_line(child, (fp_x, fp_y), fp_angle)
            if line is None:
                continue
            layer, start, end, width = line
            if not _is_copper_layer(layer):
                continue
            pad_pair = _matching_endpoint_pad_pair(endpoint_pads, start, end, width)
            if pad_pair is None:
                continue
            bridges.append(
                SolderJumperBridge(
                    footprint_ref=footprint.reference,
                    footprint_name=footprint_name,
                    description=description,
                    layer=layer,
                    start=start,
                    end=end,
                    width=width,
                    endpoint_pads=pad_pair,
                )
            )
    return bridges


def point_inside_pad_copper(x: float, y: float, pad: Pad, tolerance: float = 0.02) -> bool:
    """Return whether a point lies in a pad's copper body using local pad geometry."""
    local_x, local_y = _point_to_pad_local(x, y, pad)
    width = max(float(pad.size[0]), 1e-9)
    height = max(float(pad.size[1]), 1e-9)
    half_w = width * 0.5 + tolerance
    half_h = height * 0.5 + tolerance
    shape = str(pad.shape or "").lower()
    if shape in {"circle", "oval"}:
        rx = half_w
        ry = half_h
        return (local_x / rx) ** 2 + (local_y / ry) ** 2 <= 1.0
    if shape == "roundrect":
        # Round-rect pads are conservatively approximated by the union of the
        # center rectangle and corner circles, so bbox-only false positives are avoided.
        corner = min(width, height) * 0.25 + tolerance
        inner_w = max(0.0, half_w - corner)
        inner_h = max(0.0, half_h - corner)
        ax = abs(local_x)
        ay = abs(local_y)
        if ax <= inner_w and ay <= half_h:
            return True
        if ay <= inner_h and ax <= half_w:
            return True
        return hypot(max(0.0, ax - inner_w), max(0.0, ay - inner_h)) <= corner
    return abs(local_x) <= half_w and abs(local_y) <= half_h


def bridge_point_allowed_for_net(
    bridge: SolderJumperBridge,
    net_id: int | None,
    x: float,
    y: float,
    tolerance: float = 0.02,
) -> bool:
    """Return whether a net may touch an SJ bridge at a point inside its own SJ pad."""
    if net_id is None:
        return False
    for pad in bridge.endpoint_pads:
        if pad.net_id is None or int(pad.net_id) != int(net_id):
            continue
        if point_inside_pad_copper(x, y, pad, tolerance=tolerance):
            return True
    return False


def violation_is_allowed_sj_bridge_contact(
    violation: dict[str, Any],
    bridges: list[SolderJumperBridge],
    uuid_to_candidate: dict[str, tuple[int, int]] | None = None,
    track_geometries: dict[str, DrcTrackGeometry] | None = None,
) -> bool:
    """Return whether a KiCad DRC violation is only an intentional SJ pad contact."""
    violation_type = str(violation.get("type", "")).lower()
    if violation_type not in {"clearance", "shorting_items"}:
        return False
    required_clearance = _clearance_requirement_from_violation(violation) if violation_type == "clearance" else 0.0
    # Shorting violations represent actual copper overlap, so they do not get
    # clearance expansion. Both shorting and clearance checks keep a tiny
    # numerical tolerance to absorb coordinate rounding without widening the SJ
    # pad contact region by a visible amount.
    distance_tolerance = _CONTACT_BAND_TOLERANCE_MM
    items = list(violation.get("items", []))
    if len(items) < 2:
        return False
    for bridge in bridges:
        bridge_items = [item for item in items if _item_mentions_bridge(item, bridge)]
        if not bridge_items:
            continue
        other_items = [item for item in items if item not in bridge_items]
        if not other_items:
            continue
        if all(_item_is_allowed_bridge_point(item, bridge, uuid_to_candidate) for item in bridge_items) and all(
            _item_is_allowed_endpoint_contact(
                item,
                bridge,
                uuid_to_candidate,
                track_geometries,
                required_clearance,
                distance_tolerance,
            )
            for item in other_items
        ):
            return True
    return False


def filter_allowed_sj_bridge_violations(
    board: BoardData,
    violations: list[dict[str, Any]],
    uuid_to_candidate: dict[str, tuple[int, int]] | None = None,
) -> tuple[list[dict[str, Any]], int]:
    """Remove intentional SJ bridge contacts from KiCad DRC violation lists."""
    bridges = detect_solder_jumper_bridges(board)
    if not bridges:
        return violations, 0
    track_geometries = _track_geometries_by_uuid(board.path)
    kept: list[dict[str, Any]] = []
    ignored = 0
    for violation in violations:
        if violation_is_allowed_sj_bridge_contact(violation, bridges, uuid_to_candidate, track_geometries):
            ignored += 1
        else:
            kept.append(violation)
    return kept, ignored


def distance_point_to_segment(x: float, y: float, start: tuple[float, float], end: tuple[float, float]) -> float:
    """Return Euclidean distance from a point to a line segment."""
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    denom = dx * dx + dy * dy
    if denom <= 1e-18:
        return hypot(x - sx, y - sy)
    t = max(0.0, min(1.0, ((x - sx) * dx + (y - sy) * dy) / denom))
    return hypot(x - (sx + t * dx), y - (sy + t * dy))


def closest_points_between_segments(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Return closest points on two 2D segments and their distance."""
    candidates = [
        (_closest_point_on_segment(first_start, second_start, second_end), first_start),
        (_closest_point_on_segment(first_end, second_start, second_end), first_end),
        (second_start, _closest_point_on_segment(second_start, first_start, first_end)),
        (second_end, _closest_point_on_segment(second_end, first_start, first_end)),
    ]
    if _segments_intersect(first_start, first_end, second_start, second_end):
        point = _line_intersection_or_midpoint(first_start, first_end, second_start, second_end)
        return point, point, 0.0
    best_second, best_first = min(
        candidates,
        key=lambda pair: hypot(pair[0][0] - pair[1][0], pair[0][1] - pair[1][1]),
    )
    distance = hypot(best_first[0] - best_second[0], best_first[1] - best_second[1])
    return best_first, best_second, distance


def _closest_point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    """Project a point to the nearest point on a segment."""
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    denom = dx * dx + dy * dy
    if denom <= 1e-18:
        return start
    t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / denom))
    return (sx + t * dx, sy + t * dy)


def _segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    """Return whether two closed 2D segments intersect."""
    def orient(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> bool:
        return (
            min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
            and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
        )

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True
    return (
        abs(o1) <= 1e-9
        and on_segment(a, c, b)
        or abs(o2) <= 1e-9
        and on_segment(a, d, b)
        or abs(o3) <= 1e-9
        and on_segment(c, a, d)
        or abs(o4) <= 1e-9
        and on_segment(c, b, d)
    )


def _line_intersection_or_midpoint(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, float]:
    """Return the line intersection, falling back to the midpoint of nearest endpoints."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    dx, dy = d
    denom = (ax - bx) * (cy - dy) - (ay - by) * (cx - dx)
    if abs(denom) <= 1e-12:
        points = [a, b, c, d]
        return (sum(point[0] for point in points) * 0.25, sum(point[1] for point in points) * 0.25)
    px = ((ax * by - ay * bx) * (cx - dx) - (ax - bx) * (cx * dy - cy * dx)) / denom
    py = ((ax * by - ay * bx) * (cy - dy) - (ay - by) * (cx * dy - cy * dx)) / denom
    return (px, py)


def _parse_bridge_fp_line(
    node: list[Any],
    footprint_position: tuple[float, float],
    footprint_angle: float,
) -> tuple[str, tuple[float, float], tuple[float, float], float] | None:
    start_node = _child(node, "start")
    end_node = _child(node, "end")
    if start_node is None or end_node is None or len(start_node) < 3 or len(end_node) < 3:
        return None
    start = _transform_point((float(_number(start_node[1]) or 0.0), float(_number(start_node[2]) or 0.0)), footprint_position, footprint_angle)
    end = _transform_point((float(_number(end_node[1]) or 0.0), float(_number(end_node[2]) or 0.0)), footprint_position, footprint_angle)
    layer = _canonical_layer(_child_value(node, "layer") or "")
    width = _fp_line_width(node)
    return layer, start, end, width


def _fp_line_width(node: list[Any]) -> float:
    """Read legacy fp_line width or KiCad 6+ stroke width."""
    direct_width = _number(_child_value(node, "width"), None)
    if direct_width is not None:
        return float(direct_width)
    stroke = _child(node, "stroke")
    if stroke is not None:
        stroke_width = _number(_child_value(stroke, "width"), None)
        if stroke_width is not None:
            return float(stroke_width)
    return 0.0


def _matching_endpoint_pad_pair(
    pads: tuple[Pad, ...],
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
) -> tuple[Pad, Pad] | None:
    # The bridge must touch or nearly touch two netted endpoint pads; this
    # prevents arbitrary no-net footprint art from being treated as a jumper.
    tolerance = max(0.08, width * 0.5 + 0.03)
    matches = [
        pad
        for pad in pads
        if point_inside_pad_copper(start[0], start[1], pad, tolerance=tolerance)
        or point_inside_pad_copper(end[0], end[1], pad, tolerance=tolerance)
        or distance_point_to_segment(pad.center[0], pad.center[1], start, end) <= max(pad.size) * 0.5 + tolerance
    ]
    unique: list[Pad] = []
    for pad in matches:
        if all(existing.name != pad.name or existing.center != pad.center for existing in unique):
            unique.append(pad)
    if len(unique) != 2:
        return None
    if _pad_net_identity(unique[0]) == _pad_net_identity(unique[1]):
        return None
    return (unique[0], unique[1])


def _pad_net_names_from_node(footprint_node: list[Any]) -> dict[str, str]:
    """Read KiCad 6+ pad net names when no numeric board net table is present."""
    names: dict[str, str] = {}
    for child in footprint_node[1:]:
        if not isinstance(child, list) or len(child) < 2 or child[0] != "pad":
            continue
        raw_net = _child_value(child, "net")
        if raw_net is None:
            continue
        net_name = str(raw_net)
        if net_name and net_name != "<no net>":
            names[str(child[1])] = net_name
    return names


def _pad_has_electrical_net(pad: Pad, raw_pad_net_names: dict[str, str]) -> bool:
    """Return whether a pad has either a numeric net id or a KiCad net name."""
    if pad.net_id is not None and int(pad.net_id) > 0:
        return True
    return bool(raw_pad_net_names.get(pad.name))


def _pad_with_fallback_net_name(pad: Pad, raw_pad_net_names: dict[str, str]) -> Pad:
    """Attach a raw KiCad net name to pads parsed from generated boards without net ids."""
    fallback = raw_pad_net_names.get(pad.name)
    if not fallback or pad.net_name:
        return pad
    return Pad(
        name=pad.name,
        center=pad.center,
        size=pad.size,
        shape=pad.shape,
        layers=pad.layers,
        net_id=pad.net_id,
        net_name=fallback,
        rotation_degrees=pad.rotation_degrees,
    )


def _pad_net_identity(pad: Pad) -> tuple[str, str | int | None]:
    """Return a stable electrical identity for pads with ids or name-only nets."""
    if pad.net_id is not None and int(pad.net_id) > 0:
        return ("id", int(pad.net_id))
    return ("name", pad.net_name or None)


def _item_mentions_bridge(item: dict[str, Any], bridge: SolderJumperBridge) -> bool:
    description = str(item.get("description", ""))
    if bridge.footprint_ref not in description:
        return False
    if "<no net>" not in description and "no net" not in description.lower():
        return False
    pos = item.get("pos") or {}
    x = pos.get("x")
    y = pos.get("y")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return True
    return distance_point_to_segment(float(x), float(y), bridge.start, bridge.end) <= bridge.width * 0.5 + 0.08


def _item_is_allowed_bridge_point(
    item: dict[str, Any],
    bridge: SolderJumperBridge,
    uuid_to_candidate: dict[str, tuple[int, int]] | None,
) -> bool:
    pos = item.get("pos") or {}
    x = pos.get("x")
    y = pos.get("y")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return False
    return any(point_inside_pad_copper(float(x), float(y), pad, tolerance=0.04) for pad in bridge.endpoint_pads)


def _item_is_allowed_endpoint_contact(
    item: dict[str, Any],
    bridge: SolderJumperBridge,
    uuid_to_candidate: dict[str, tuple[int, int]] | None,
    track_geometries: dict[str, DrcTrackGeometry] | None = None,
    required_clearance: float = 0.0,
    distance_tolerance: float = _CONTACT_BAND_TOLERANCE_MM,
) -> bool:
    pos = item.get("pos") or {}
    x = pos.get("x")
    y = pos.get("y")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return False
    net_name = _item_net_name(item)
    if net_name and _point_allowed_for_endpoint_net_name(bridge, net_name, float(x), float(y), tolerance=0.04):
        return True

    # KiCad reports a representative item position for tracks, not always the
    # actual short/contact point. Resolve the track UUID back to segment
    # geometry and test the closest route/bridge points against SJ endpoint pads.
    track = _track_geometry_for_item(item, track_geometries)
    if track is not None:
        route_net_name = net_name or track.net_name
        if route_net_name and _track_contact_allowed_for_endpoint_net_name(
            bridge,
            route_net_name,
            track,
            required_clearance,
            distance_tolerance,
        ):
            return True

    # Numeric net ids are only a fallback. Selector candidate ids may differ
    # from KiCad board net ids, so name-based matching above must get priority.
    net_id = _candidate_net_id(item, uuid_to_candidate)
    if net_id is not None:
        return bridge_point_allowed_for_net(bridge, net_id, float(x), float(y), tolerance=0.04)
    return False


def _point_allowed_for_endpoint_net_name(
    bridge: SolderJumperBridge,
    net_name: str,
    x: float,
    y: float,
    tolerance: float,
) -> bool:
    """Return whether a named net point is inside its own SJ endpoint pad."""
    if net_name not in bridge.endpoint_net_names:
        return False
    return any(
        pad.net_name == net_name and point_inside_pad_copper(x, y, pad, tolerance=tolerance)
        for pad in bridge.endpoint_pads
    )


def _track_contact_allowed_for_endpoint_net_name(
    bridge: SolderJumperBridge,
    net_name: str,
    track: DrcTrackGeometry,
    required_clearance: float = 0.0,
    distance_tolerance: float = _CONTACT_BAND_TOLERANCE_MM,
) -> bool:
    """Return whether the actual track/bridge violation region stays inside the net's SJ pad."""
    if net_name not in bridge.endpoint_net_names:
        return False
    for pad in bridge.endpoint_pads:
        if pad.net_name != net_name:
            continue
        if _track_bridge_violation_region_inside_pad(bridge, pad, track, required_clearance, distance_tolerance):
            return True
    return False


def _track_bridge_violation_region_inside_pad(
    bridge: SolderJumperBridge,
    pad: Pad,
    track: DrcTrackGeometry,
    required_clearance: float,
    distance_tolerance: float,
) -> bool:
    """Return whether every sampled near/overlap point stays inside the endpoint pad."""
    allowed_distance = (
        track.width * 0.5
        + bridge.width * 0.5
        + max(0.0, required_clearance)
        + max(0.0, distance_tolerance)
    )
    route_near_points = _segment_points_near_segment(track.start, track.end, bridge.start, bridge.end, allowed_distance)
    bridge_near_points = _segment_points_near_segment(bridge.start, bridge.end, track.start, track.end, allowed_distance)
    if not route_near_points or not bridge_near_points:
        return False
    return all(point_inside_pad_copper(x, y, pad, tolerance=0.04) for x, y in route_near_points) and all(
        point_inside_pad_copper(x, y, pad, tolerance=0.04) for x, y in bridge_near_points
    )


def _segment_points_near_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    other_start: tuple[float, float],
    other_end: tuple[float, float],
    allowed_distance: float,
) -> list[tuple[float, float]]:
    """Sample the portion of one segment that lies inside a distance band around another."""
    length = hypot(end[0] - start[0], end[1] - start[1])
    sample_count = max(1, int(length / 0.025) + 1)
    near_points: list[tuple[float, float]] = []
    for index in range(sample_count + 1):
        t = index / sample_count
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        if distance_point_to_segment(x, y, other_start, other_end) <= allowed_distance:
            near_points.append((x, y))
    return near_points


def _clearance_requirement_from_violation(violation: dict[str, Any]) -> float:
    """Parse KiCad's required clearance from a localized DRC description."""
    description = str(violation.get("description", ""))
    values = [float(value) for value in re.findall(r"([0-9]+(?:\.[0-9]+)?)\s*mm", description)]
    if values:
        return values[0]
    return 0.2


def _track_geometry_for_item(
    item: dict[str, Any],
    track_geometries: dict[str, DrcTrackGeometry] | None,
) -> DrcTrackGeometry | None:
    """Return parsed track geometry for a KiCad DRC item UUID."""
    if not track_geometries:
        return None
    uuid = str(item.get("uuid", "")).lower()
    if not uuid:
        return None
    return track_geometries.get(uuid)


def _track_geometries_by_uuid(board_path: Path) -> dict[str, DrcTrackGeometry]:
    """Parse KiCad segment geometry by UUID for DRC feedback contact checks."""
    try:
        text = Path(board_path).read_text(encoding="utf-8", errors="ignore")
        sexpr = _parse_sexpr(text)
    except Exception:
        return {}

    net_names = _net_names_from_sexpr(sexpr)
    tracks: dict[str, DrcTrackGeometry] = {}
    for node in _top_level_nodes(sexpr, "segment"):
        if not isinstance(node, list):
            continue
        uuid = str(_child_value(node, "uuid") or "").lower()
        if not uuid:
            continue
        start = _point_child(node, "start")
        end = _point_child(node, "end")
        if start is None or end is None:
            continue
        width = float(_number(_child_value(node, "width"), 0.0) or 0.0)
        layer = _canonical_layer(str(_child_value(node, "layer") or ""))
        net_id = _number(_child_value(node, "net"), None)
        net_name = net_names.get(int(net_id)) if net_id is not None else None
        tracks[uuid] = DrcTrackGeometry(start=start, end=end, width=width, layer=layer, net_name=net_name)
    return tracks


def _net_names_from_sexpr(sexpr: list[Any]) -> dict[int, str]:
    """Read KiCad board net table entries from an s-expression tree."""
    names: dict[int, str] = {}
    for node in _top_level_nodes(sexpr, "net"):
        if not isinstance(node, list) or len(node) < 3:
            continue
        net_id = _number(node[1], None)
        if net_id is None:
            continue
        names[int(net_id)] = str(node[2])
    return names


def _point_child(node: list[Any], name: str) -> tuple[float, float] | None:
    """Read a two-number point child such as `(start x y)` or `(end x y)`."""
    child = _child(node, name)
    if child is None or len(child) < 3:
        return None
    x = _number(child[1], None)
    y = _number(child[2], None)
    if x is None or y is None:
        return None
    return (float(x), float(y))


def _candidate_net_id(item: dict[str, Any], uuid_to_candidate: dict[str, tuple[int, int]] | None) -> int | None:
    if not uuid_to_candidate:
        return None
    uuid = str(item.get("uuid", "")).lower()
    candidate = uuid_to_candidate.get(uuid)
    if candidate is None:
        return None
    return int(candidate[0])


def _item_net_name(item: dict[str, Any]) -> str | None:
    description = str(item.get("description", ""))
    match = re.search(r"\[([^\]]+)\]", description)
    if not match:
        return None
    value = match.group(1).strip()
    if value == "<no net>":
        return None
    return value


def _footprint_reference_from_node(node: list[Any]) -> str | None:
    for child in node[1:]:
        if not isinstance(child, list) or len(child) < 3:
            continue
        if child[0] in {"property", "fp_text"} and child[1] in {"Reference", "reference"}:
            return str(child[2])
    return None


def _is_normally_closed_solder_jumper(name: str, description: str) -> bool:
    text = f"{name} {description}".upper()
    if "SJ_2S-TRACE" in text:
        return True
    if "NETTIE" in text or "NET-TIE" in text or "NET TIE" in text:
        return True
    if "SOLDERJUMPER" in text or "SOLDER JUMPER" in text:
        return "BRIDGED" in text or "SHORTED" in text or "TRACE" in text
    return False


def _is_copper_layer(layer: str) -> bool:
    normalized = _canonical_layer(layer)
    return normalized in {"F.Cu", "B.Cu", "Top", "Bottom"} or normalized.startswith("In") or normalized.startswith("Route")


def _point_to_pad_local(x: float, y: float, pad: Pad) -> tuple[float, float]:
    angle = radians(float(pad.rotation_degrees or 0.0))
    dx = float(x) - float(pad.center[0])
    dy = float(y) - float(pad.center[1])
    c = cos(angle)
    s = sin(angle)
    return (dx * c - dy * s, dx * s + dy * c)
