from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from pathlib import Path
from time import perf_counter
import sys
from typing import Any

from router_app.kicad_parser import BoardData


@dataclass(frozen=True)
class RerouteOutcome:
    ok: bool
    message: str
    result: Any | None = None
    elapsed_seconds: float | None = None


def minimum_grid_steps_per_mm(board: BoardData) -> int:
    previous_pitch = (_min_trace_width(board) + _min_clearance(board)) * 0.5
    if previous_pitch <= 0:
        return 1
    return max(1, ceil(1.0 / previous_pitch))


def run_dijkstra_reroute_test(
    board: BoardData,
    ripped_net_ids: set[int],
    grid_steps_per_mm: float = 10.0,
) -> RerouteOutcome:
    if not ripped_net_ids:
        return RerouteOutcome(False, "Rip up at least one net before rerouting.")

    started_at = perf_counter()
    try:
        router_core = _import_router_core()
    except ImportError as exc:
        return RerouteOutcome(
            False,
            "router_core is not built yet. Build the C++ pybind module before rerouting.",
            exc,
            perf_counter() - started_at,
        )

    results = []
    failures: list[str] = []
    all_ripped_net_ids = sorted(ripped_net_ids)
    for net_id in all_ripped_net_ids:
        ordered_ripped_net_ids = [net_id] + [other for other in all_ripped_net_ids if other != net_id]
        result = _run_single_dijkstra_reroute_test(router_core, board, ordered_ripped_net_ids, grid_steps_per_mm)
        if result.found:
            results.append(result)
            continue

        reason = getattr(result, "failure_reason", "") or "No path was found."
        terminal_sizes = getattr(result, "terminal_group_sizes", [])
        if terminal_sizes:
            reason = f"{reason} terminal vertices per pad: {list(terminal_sizes)}"
        reason = f"{reason} {_net_debug_summary(board, result.net_id)}"
        failures.append(f"net {result.net_id}: {reason}")

    if failures:
        return RerouteOutcome(
            False,
            f"Rerouted {len(results)}/{len(all_ripped_net_ids)} nets. Failed: {' | '.join(failures)}",
            results,
            perf_counter() - started_at,
        )

    total_candidates = sum(len(getattr(result, "candidate_paths_mm", [])) or 1 for result in results)
    total_vertices = sum(
        sum(len(path) for path in getattr(result, "candidate_paths_mm", [])) or len(result.path_mm)
        for result in results
    )
    return RerouteOutcome(
        True,
        f"Rerouted {len(results)} nets: {total_candidates} candidate paths, {total_vertices} total grid vertices.",
        results,
        perf_counter() - started_at,
    )


def _run_single_dijkstra_reroute_test(
    router_core: Any,
    board: BoardData,
    ordered_ripped_net_ids: list[int],
    grid_steps_per_mm: float = 10.0,
) -> Any:
    request = router_core.RouteRequest()
    request.layers = board.copper_layers or ["F.Cu"]
    request.ripped_net_ids = ordered_ripped_net_ids
    request.min_x, request.min_y, request.max_x, request.max_y = _board_bounds(board)
    request.min_trace_width = _min_trace_width(board)
    request.min_clearance = _min_clearance(board)
    request.generated_via_diameter = _generated_via_diameter(board, ordered_ripped_net_ids[0])
    request.grid_steps_per_mm = grid_steps_per_mm

    track_items = []
    for track in board.tracks:
        item = router_core.TrackGeometry()
        item.start = router_core.Point2D(track.start[0], track.start[1])
        item.end = router_core.Point2D(track.end[0], track.end[1])
        item.width = track.width
        item.clearance = _clearance_for_net(board, track.net_id)
        item.net_id = track.net_id
        item.layer = track.layer
        track_items.append(item)
    request.tracks = track_items

    pad_items = []
    for footprint in board.footprints:
        for pad in footprint.pads:
            item = router_core.PadGeometry()
            item.center = router_core.Point2D(pad.center[0], pad.center[1])
            item.footprint_center = router_core.Point2D(footprint.position[0], footprint.position[1])
            item.size_x = pad.size[0]
            item.size_y = pad.size[1]
            item.rotation_degrees = pad.rotation_degrees
            item.shape = pad.shape
            item.net_id = pad.net_id or 0
            item.layers = list(pad.layers)
            pad_items.append(item)
    request.pads = pad_items

    via_items = []
    for via in board.vias:
        item = router_core.ViaGeometry()
        item.center = router_core.Point2D(via.center[0], via.center[1])
        item.diameter = via.diameter
        item.net_id = via.net_id
        via_items.append(item)
    request.vias = via_items

    return router_core.run_dijkstra_test(request)


def _import_router_core():
    root = Path(__file__).resolve().parents[1]
    for candidate in [
        root,
        root / "build",
        root / "build-linux",
        root / "build" / "Release",
        root / "build" / "Debug",
    ]:
        if str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
    import router_core

    return router_core


def _net_debug_summary(board: BoardData, net_id: int) -> str:
    pads = [pad for footprint in board.footprints for pad in footprint.pads if pad.net_id == net_id]
    tracks = [track for track in board.tracks if track.net_id == net_id]
    vias = [via for via in board.vias if via.net_id == net_id]
    pad_bits = [
        f"{pad.name}@({pad.center[0]:.4f},{pad.center[1]:.4f}) size=({pad.size[0]:.4f},{pad.size[1]:.4f})"
        for pad in pads[:4]
    ]
    return (
        f"[Python sees {len(pads)} pads, {len(tracks)} tracks, {len(vias)} vias. "
        f"Pads: {'; '.join(pad_bits)}]"
    )


def _board_bounds(board: BoardData) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for track in board.tracks:
        xs.extend([track.start[0], track.end[0]])
        ys.extend([track.start[1], track.end[1]])
    for footprint in board.footprints:
        for pad in footprint.pads:
            half_x = pad.size[0] * 0.5
            half_y = pad.size[1] * 0.5
            xs.extend([pad.center[0] - half_x, pad.center[0] + half_x])
            ys.extend([pad.center[1] - half_y, pad.center[1] + half_y])
    for via in board.vias:
        radius = via.diameter * 0.5
        xs.extend([via.center[0] - radius, via.center[0] + radius])
        ys.extend([via.center[1] - radius, via.center[1] + radius])
    if not xs or not ys:
        return (0.0, 0.0, 10.0, 10.0)
    return (min(xs), min(ys), max(xs), max(ys))


def _min_trace_width(board: BoardData) -> float:
    widths = [track.width for track in board.tracks if track.width > 0]
    if widths:
        return min(widths)
    if board.design_rules and board.design_rules.get("track_width"):
        return board.design_rules["track_width"]
    return 0.2


def _min_clearance(board: BoardData) -> float:
    if board.design_rules and board.design_rules.get("min_clearance"):
        return board.design_rules["min_clearance"]
    return 0.2


def _clearance_for_net(board: BoardData, net_id: int) -> float:
    if board.net_clearances and net_id in board.net_clearances:
        return board.net_clearances[net_id]
    return _min_clearance(board)


def _generated_via_diameter(board: BoardData, net_id: int) -> float:
    trace_width = _trace_width_for_net(board, net_id)
    if board.design_rules and board.design_rules.get("via_diameter"):
        return max(float(board.design_rules["via_diameter"]), trace_width)

    net_diameters = [via.diameter for via in board.vias if via.net_id == net_id and via.diameter > 0]
    if net_diameters:
        counts: dict[float, int] = {}
        for diameter in net_diameters:
            counts[diameter] = counts.get(diameter, 0) + 1
        chosen_via_diameter = max(counts.items(), key=lambda item: (item[1], item[0]))[0]
        return max(chosen_via_diameter, trace_width)

    diameters = [via.diameter for via in board.vias if via.diameter > 0]
    if diameters:
        counts: dict[float, int] = {}
        for diameter in diameters:
            counts[diameter] = counts.get(diameter, 0) + 1
        chosen_via_diameter = max(counts.items(), key=lambda item: (item[1], item[0]))[0]
        return max(chosen_via_diameter, trace_width)
    return max(0.6, trace_width)


def _trace_width_for_net(board: BoardData, net_id: int) -> float:
    widths = [track.width for track in board.tracks if track.net_id == net_id and track.width > 0]
    if widths:
        return max(widths)
    return _min_trace_width(board)
