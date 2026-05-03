from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime
from math import ceil
import math
from math import cos, sin
from pathlib import Path
from time import perf_counter
import sys
from types import SimpleNamespace
from typing import Any

from router_app.kicad_parser import BoardData, load_board
from router_app.path_selector import SelectionResult
from router_app.path_selector.types import NetSelection as PyNetSelection


@dataclass(frozen=True)
class RerouteOutcome:
    ok: bool
    message: str
    result: Any | None = None
    elapsed_seconds: float | None = None
    candidate_export_path: Path | None = None


def select_reroute_candidates(
    outcome: RerouteOutcome,
    max_paths_per_net: int = 1,
    prefer_gurobi: bool = True,
) -> SelectionResult:
    if not outcome.result or not isinstance(outcome.result, list):
        return SelectionResult(
            ok=False,
            selections=[],
            solver="none",
            message="No reroute results available for selection.",
        )
    try:
        router_core = _import_router_core()
    except ImportError as exc:
        return SelectionResult(
            ok=False,
            selections=[],
            solver="none",
            message=f"router_core import failed: {exc}",
        )

    request = router_core.SelectionRequest()
    request.max_paths_per_net = max_paths_per_net
    request.prefer_gurobi = prefer_gurobi
    request.allow_fallback = True

    exported_boundary_by_net = _load_exported_boundary_vertices(outcome.candidate_export_path)
    selector_board = _load_selector_board(outcome.candidate_export_path)
    nets = []
    for result in outcome.result:
        item = router_core.NetCandidateSet()
        item.net_id = int(getattr(result, "net_id", 0))
        item.candidate_paths_grid = list(getattr(result, "candidate_paths_grid", []))
        item.candidate_paths_mm = list(getattr(result, "candidate_paths_mm", []))
        boundary_payload = exported_boundary_by_net.get(item.net_id, [])
        (
            item.candidate_boundary_vertices,
            item.candidate_terminal_coords,
            item.candidate_terminal_groups,
        ) = _selector_boundary_payload_from_export(
            router_core,
            selector_board,
            result,
            item.net_id,
            item.candidate_paths_grid,
            boundary_payload,
        )
        nets.append(item)
    request.nets = nets

    cpp_result = router_core.select_candidate_paths(request)
    selections = [
        PyNetSelection(
            net_id=int(selection.net_id),
            selected_candidate_indices=[int(i) for i in selection.selected_candidate_indices],
            solver=str(selection.solver),
            objective=float(selection.objective) if bool(selection.has_objective) else None,
        )
        for selection in cpp_result.selections
    ]
    return SelectionResult(
        ok=bool(cpp_result.ok),
        selections=selections,
        solver=str(cpp_result.solver),
        message=str(cpp_result.message),
    )


def _load_exported_boundary_vertices(export_path: Path | None) -> dict[int, list[list[dict[str, Any]]]]:
    if export_path is None or not export_path.exists():
        return {}
    try:
        payload = json.loads(export_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    results = payload.get("results", [])
    mapping: dict[int, list[list[dict[str, Any]]]] = {}
    for item in results:
        try:
            net_id = int(item.get("net_id", 0))
        except Exception:
            continue
        boundary_vertices = item.get("candidate_boundary_vertices", [])
        if isinstance(boundary_vertices, list):
            mapping[net_id] = boundary_vertices
    return mapping


def _load_selector_board(export_path: Path | None) -> BoardData | None:
    if export_path is None or not export_path.exists():
        return None
    try:
        payload = json.loads(export_path.read_text(encoding="utf-8"))
        board_path = payload.get("board", {}).get("path")
        if not board_path:
            return None
        return load_board(board_path)
    except Exception:
        return None


def _selector_boundary_payload_from_export(
    router_core: Any,
    board: BoardData | None,
    result: Any,
    net_id: int,
    candidate_paths_grid: list[list[Any]],
    boundary_payload: list[list[dict[str, Any]]],
) -> tuple[list[list[Any]], list[list[Any]], list[list[list[Any]]]]:
    boundary_vertices: list[list[Any]] = []
    terminal_coords: list[list[Any]] = []
    terminal_groups: list[list[list[Any]]] = []
    pad_boundary_cache: dict[tuple[Any, ...], list[tuple[int, int, int]]] = {}
    path_count = len(candidate_paths_grid)
    for index in range(path_count):
        path = candidate_paths_grid[index]
        raw_vertices = boundary_payload[index] if index < len(boundary_payload) else []

        vertices_for_path: list[Any] = []
        start_anchor: Any | None = None
        goal_anchor: Any | None = None
        seen: set[tuple[int, int, int]] = set()
        for vertex in raw_vertices:
            if not isinstance(vertex, dict):
                continue
            x = int(vertex.get("x", 0))
            y = int(vertex.get("y", 0))
            z = int(vertex.get("z", 0))
            key = (x, y, z)
            if key in seen:
                continue
            seen.add(key)
            point = router_core.GridPoint(x, y, z)
            vertices_for_path.append(point)
            anchor = str(vertex.get("anchor", ""))
            if anchor == "start":
                start_anchor = point
            elif anchor == "goal":
                goal_anchor = point

        if not path:
            boundary_vertices.append(vertices_for_path)
            terminal_coords.append([])
            terminal_groups.append([])
            continue

        if start_anchor is None:
            p0 = path[0]
            start_anchor = router_core.GridPoint(int(getattr(p0, "x", 0)), int(getattr(p0, "y", 0)), int(getattr(p0, "z", 0)))
        if goal_anchor is None:
            p1 = path[-1]
            goal_anchor = router_core.GridPoint(int(getattr(p1, "x", 0)), int(getattr(p1, "y", 0)), int(getattr(p1, "z", 0)))

        if (start_anchor.x, start_anchor.y, start_anchor.z) not in seen:
            vertices_for_path.append(start_anchor)
            seen.add((start_anchor.x, start_anchor.y, start_anchor.z))
        if (goal_anchor.x, goal_anchor.y, goal_anchor.z) not in seen:
            vertices_for_path.append(goal_anchor)
            seen.add((goal_anchor.x, goal_anchor.y, goal_anchor.z))

        groups_for_path: list[list[Any]] = []
        if board is not None:
            start_tuple = (start_anchor.x, start_anchor.y, start_anchor.z)
            goal_tuple = (goal_anchor.x, goal_anchor.y, goal_anchor.z)
            start_pad = _closest_net_pad_for_vertex(board, result, net_id, start_tuple)
            goal_pad = _closest_net_pad_for_vertex(board, result, net_id, goal_tuple)

            def _group_from_pad(pad: Any | None, anchor: tuple[int, int, int]) -> list[Any]:
                if pad is None:
                    return [router_core.GridPoint(anchor[0], anchor[1], anchor[2])]
                key = _pad_cache_key(pad)
                cached = pad_boundary_cache.get(key)
                if cached is None:
                    cached = sorted(_pad_boundary_vertices_for_selector(board, result, pad))
                    pad_boundary_cache[key] = cached
                group = [router_core.GridPoint(x, y, z) for (x, y, z) in cached]
                if anchor not in {(v.x, v.y, v.z) for v in group}:
                    group.append(router_core.GridPoint(anchor[0], anchor[1], anchor[2]))
                return group

            groups_for_path.append(_group_from_pad(start_pad, start_tuple))
            groups_for_path.append(_group_from_pad(goal_pad, goal_tuple))
        else:
            groups_for_path.append([start_anchor])
            groups_for_path.append([goal_anchor])

        boundary_vertices.append(vertices_for_path)
        terminal_coords.append([start_anchor, goal_anchor])
        terminal_groups.append(groups_for_path)
    return boundary_vertices, terminal_coords, terminal_groups


def _pad_cache_key(pad: Any) -> tuple[Any, ...]:
    return (
        str(getattr(pad, "shape", "")).lower(),
        round(float(getattr(pad, "center", (0.0, 0.0))[0]), 6),
        round(float(getattr(pad, "center", (0.0, 0.0))[1]), 6),
        round(float(getattr(pad, "size", (0.0, 0.0))[0]), 6),
        round(float(getattr(pad, "size", (0.0, 0.0))[1]), 6),
        round(float(getattr(pad, "rotation_degrees", 0.0)), 6),
        tuple(getattr(pad, "layers", [])),
    )


def _pad_boundary_vertices_for_selector(
    board: BoardData,
    result: Any,
    pad: Any,
) -> set[tuple[int, int, int]]:
    pitch = float(getattr(result, "grid_pitch", 0.0))
    if pitch <= 0.0:
        return set()
    nx = int(getattr(result, "nx", 0))
    ny = int(getattr(result, "ny", 0))
    if nx <= 0 or ny <= 0:
        return set()
    ox = float(getattr(result, "origin_x", 0.0))
    oy = float(getattr(result, "origin_y", 0.0))
    cx = float(pad.center[0])
    cy = float(pad.center[1])
    half_x = float(pad.size[0]) * 0.5
    half_y = float(pad.size[1]) * 0.5
    radius = math.hypot(half_x, half_y) + pitch * 1.5
    gx0 = max(0, int(math.floor((cx - radius - ox) / pitch)) - 1)
    gx1 = min(nx - 1, int(math.ceil((cx + radius - ox) / pitch)) + 1)
    gy0 = max(0, int(math.floor((cy - radius - oy) / pitch)) - 1)
    gy1 = min(ny - 1, int(math.ceil((cy + radius - oy) / pitch)) + 1)

    candidate_layers = []
    for z, layer_name in enumerate(board.copper_layers):
        if "*.Cu" in pad.layers or layer_name in pad.layers:
            candidate_layers.append(z)
    if not candidate_layers:
        return set()

    inside: set[tuple[int, int, int]] = set()
    for z in candidate_layers:
        for x in range(gx0, gx1 + 1):
            for y in range(gy0, gy1 + 1):
                vertex = (x, y, z)
                if _vertex_inside_pad_clearance(board, result, vertex, pad, 0.0):
                    inside.add(vertex)

    if not inside:
        return set()

    boundary: set[tuple[int, int, int]] = set()
    neighbors = (
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    )
    for x, y, z in inside:
        is_boundary = False
        for dx, dy in neighbors:
            neighbor = (x + dx, y + dy, z)
            if neighbor not in inside:
                is_boundary = True
                break
        if is_boundary:
            boundary.add((x, y, z))
    return boundary


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
    multi_net_mode = len(all_ripped_net_ids) > 1
    for net_id in all_ripped_net_ids:
        if multi_net_mode:
            single_net_result = _run_single_dijkstra_reroute_test(
                router_core,
                board,
                [net_id],
                grid_steps_per_mm,
            )
            ordered_ripped_net_ids = [net_id] + [other for other in all_ripped_net_ids if other != net_id]
            multi_net_result = _run_single_dijkstra_reroute_test(
                router_core,
                board,
                ordered_ripped_net_ids,
                grid_steps_per_mm,
            )
            result = _merge_two_phase_route_results(single_net_result, multi_net_result, per_phase_limit=1000)
        else:
            result = _run_single_dijkstra_reroute_test(router_core, board, [net_id], grid_steps_per_mm)
        if result.found:
            results.append(result)
            continue

        reason = getattr(result, "failure_reason", "") or "No path was found."
        terminal_sizes = getattr(result, "terminal_group_sizes", [])
        if terminal_sizes:
            reason = f"{reason} terminal vertices per pad: {list(terminal_sizes)}"
        reason = f"{reason} {_net_debug_summary(board, result.net_id)}"
        failures.append(f"net {result.net_id}: {reason}")

    candidate_export_path = _export_candidate_path_manifest(board, results, all_ripped_net_ids, grid_steps_per_mm)

    if failures:
        return RerouteOutcome(
            False,
            f"Rerouted {len(results)}/{len(all_ripped_net_ids)} nets. Failed: {' | '.join(failures)}",
            results,
            perf_counter() - started_at,
            candidate_export_path,
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
        candidate_export_path,
    )


def _merge_two_phase_route_results(
    phase1: Any,
    phase2: Any,
    per_phase_limit: int = 1000,
) -> Any:
    def _path_key(path: list[Any]) -> tuple[tuple[int, int, int], ...]:
        return tuple(
            (
                int(getattr(point, "x", 0)),
                int(getattr(point, "y", 0)),
                int(getattr(point, "z", 0)),
            )
            for point in path
        )

    def _limited_pairs(result: Any) -> list[tuple[list[Any], list[Any]]]:
        grid_paths = list(getattr(result, "candidate_paths_grid", []))
        mm_paths = list(getattr(result, "candidate_paths_mm", []))
        pairs: list[tuple[list[Any], list[Any]]] = []
        count = min(len(grid_paths), len(mm_paths), per_phase_limit)
        for index in range(count):
            pairs.append((list(grid_paths[index]), list(mm_paths[index])))
        return pairs

    merged_grid: list[list[Any]] = []
    merged_mm: list[list[Any]] = []
    seen: set[tuple[tuple[int, int, int], ...]] = set()
    for source in (phase1, phase2):
        for grid_path, mm_path in _limited_pairs(source):
            if len(grid_path) < 2:
                continue
            key = _path_key(grid_path)
            if key in seen:
                continue
            seen.add(key)
            merged_grid.append(grid_path)
            merged_mm.append(mm_path)

    base = phase2 if bool(getattr(phase2, "found", False)) else phase1
    found = bool(merged_grid)
    failure_reason = ""
    if not found:
        reason1 = str(getattr(phase1, "failure_reason", ""))
        reason2 = str(getattr(phase2, "failure_reason", ""))
        if reason1 and reason2 and reason1 != reason2:
            failure_reason = f"[single-net] {reason1} | [multi-net] {reason2}"
        else:
            failure_reason = reason2 or reason1

    path_grid = merged_grid[0] if merged_grid else list(getattr(base, "path_grid", []))
    path_mm = merged_mm[0] if merged_mm else list(getattr(base, "path_mm", []))
    return SimpleNamespace(
        net_id=int(getattr(base, "net_id", 0)),
        found=found,
        failure_reason=failure_reason,
        grid_pitch=float(getattr(base, "grid_pitch", 0.0)),
        origin_x=float(getattr(base, "origin_x", 0.0)),
        origin_y=float(getattr(base, "origin_y", 0.0)),
        nx=int(getattr(base, "nx", 0)),
        ny=int(getattr(base, "ny", 0)),
        nz=int(getattr(base, "nz", 0)),
        terminal_group_sizes=list(getattr(base, "terminal_group_sizes", [])),
        start_vertices=list(getattr(base, "start_vertices", [])),
        goal_vertices=list(getattr(base, "goal_vertices", [])),
        candidate_paths_grid=merged_grid,
        candidate_paths_mm=merged_mm,
        path_grid=path_grid,
        path_mm=path_mm,
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


def _export_candidate_path_manifest(
    board: BoardData,
    results: list[Any],
    ripped_net_ids: list[int],
    grid_steps_per_mm: float,
) -> Path | None:
    if not results:
        return None

    root = Path(__file__).resolve().parents[1]
    export_dir = root / "out" / "candidate_path_exports"
    export_dir.mkdir(parents=True, exist_ok=True)

    board_stem = board.path.stem or "board"
    net_label = "-".join(str(net_id) for net_id in ripped_net_ids) or "none"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    export_path = export_dir / f"{board_stem}__nets_{net_label}__{timestamp}.json"

    payload = {
        "schema_version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "board": {
            "path": str(board.path),
            "backend": board.backend,
            "copper_layers": list(board.copper_layers),
        },
        "ripped_net_ids": list(ripped_net_ids),
        "grid_steps_per_mm": float(grid_steps_per_mm),
        "results": [
            _serialize_route_result(board, result)
            for result in results
        ],
    }
    export_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return export_path


def _serialize_route_result(board: BoardData, result: Any) -> dict[str, Any]:
    net_id = int(getattr(result, "net_id", 0))
    candidate_paths_grid = list(getattr(result, "candidate_paths_grid", []))
    candidate_paths_mm = list(getattr(result, "candidate_paths_mm", []))
    clearance = _clearance_for_net(board, net_id)
    trace_width = _trace_width_for_net(board, net_id)
    generated_via_diameter = _generated_via_diameter(board, net_id)
    candidate_boundary_vertices = [
        _serialize_boundary_vertex_set(
            board,
            result,
            path,
            trace_width,
            clearance,
            generated_via_diameter,
        )
        for path in candidate_paths_grid
    ]

    return {
        "net_id": net_id,
        "net_name": board.nets.get(net_id, ""),
        "found": bool(getattr(result, "found", False)),
        "failure_reason": str(getattr(result, "failure_reason", "")),
        "trace_width_mm": float(trace_width),
        "clearance_mm": float(clearance),
        "generated_via_diameter_mm": float(generated_via_diameter),
        "grid": {
            "pitch_mm": float(getattr(result, "grid_pitch", 0.0)),
            "origin_x_mm": float(getattr(result, "origin_x", 0.0)),
            "origin_y_mm": float(getattr(result, "origin_y", 0.0)),
            "nx": int(getattr(result, "nx", 0)),
            "ny": int(getattr(result, "ny", 0)),
            "nz": int(getattr(result, "nz", 0)),
        },
        "terminal_group_sizes": [int(value) for value in getattr(result, "terminal_group_sizes", [])],
        "start_vertices": [_serialize_grid_point(point, board) for point in getattr(result, "start_vertices", [])],
        "goal_vertices": [_serialize_grid_point(point, board) for point in getattr(result, "goal_vertices", [])],
        "candidate_segment_counts": [max(0, len(path) - 1) for path in candidate_paths_grid],
        "candidate_boundary_vertices": candidate_boundary_vertices,
        "candidate_boundary_vertex_counts": [len(vertices) for vertices in candidate_boundary_vertices],
        "candidate_paths_grid": [
            [_serialize_grid_point(point, board) for point in path]
            for path in candidate_paths_grid
        ],
        "candidate_paths_physical": [
            _serialize_physical_path(board, grid_path, mm_path)
            for grid_path, mm_path in zip(candidate_paths_grid, candidate_paths_mm)
        ],
        "selected_path_grid": [_serialize_grid_point(point, board) for point in getattr(result, "path_grid", [])],
        "selected_path_physical": _serialize_physical_path(
            board,
            list(getattr(result, "path_grid", [])),
            list(getattr(result, "path_mm", [])),
        ),
    }


def _serialize_boundary_vertex_set(
    board: BoardData,
    result: Any,
    path_grid: list[Any],
    trace_width: float,
    clearance: float,
    via_diameter: float,
) -> list[dict[str, Any]]:
    if not path_grid:
        return []

    net_id = int(getattr(result, "net_id", 0))
    start_anchor = _grid_point_to_vertex_tuple(path_grid[0])
    goal_anchor = _grid_point_to_vertex_tuple(path_grid[-1])
    start_pad = _closest_net_pad_for_vertex(board, result, net_id, start_anchor)
    goal_pad = _closest_net_pad_for_vertex(board, result, net_id, goal_anchor)

    occupied = _path_occupied_vertex_shell(
        result,
        path_grid,
        trace_width * 0.5 + clearance,
        via_diameter * 0.5 + clearance,
    )
    filtered: set[tuple[int, int, int]] = set()
    for vertex in occupied:
        if vertex == start_anchor or vertex == goal_anchor:
            filtered.add(vertex)
            continue
        if _vertex_inside_pad_clearance(board, result, vertex, start_pad, clearance):
            continue
        if _vertex_inside_pad_clearance(board, result, vertex, goal_pad, clearance):
            continue
        filtered.add(vertex)

    filtered.add(start_anchor)
    filtered.add(goal_anchor)

    serialized: list[dict[str, Any]] = []
    serialized.append(_serialize_grid_vertex_tuple(board, start_anchor, anchor="start"))
    if goal_anchor != start_anchor:
        serialized.append(_serialize_grid_vertex_tuple(board, goal_anchor, anchor="goal"))
    for vertex in sorted(filtered):
        if vertex == start_anchor or vertex == goal_anchor:
            continue
        serialized.append(_serialize_grid_vertex_tuple(board, vertex))
    return serialized


def _serialize_grid_point(point: Any, board: BoardData) -> dict[str, Any]:
    layers = board.copper_layers
    z = int(getattr(point, "z", 0))
    layer = layers[z] if 0 <= z < len(layers) else None
    return {
        "x": int(getattr(point, "x", 0)),
        "y": int(getattr(point, "y", 0)),
        "z": z,
        "layer": layer,
    }


def _serialize_physical_path(board: BoardData, grid_path: list[Any], mm_path: list[Any]) -> list[dict[str, Any]]:
    layers = board.copper_layers
    serialized: list[dict[str, Any]] = []
    count = min(len(grid_path), len(mm_path))
    for index in range(count):
        grid_point = grid_path[index]
        point = mm_path[index]
        z = int(getattr(grid_point, "z", 0))
        layer = layers[z] if 0 <= z < len(layers) else None
        serialized.append(
            {
                "x_mm": float(getattr(point, "x", 0.0)),
                "y_mm": float(getattr(point, "y", 0.0)),
                "z": z,
                "layer": layer,
            }
        )
    return serialized


def _serialize_grid_vertex_tuple(
    board: BoardData,
    vertex: tuple[int, int, int],
    anchor: str | None = None,
) -> dict[str, Any]:
    layers = board.copper_layers
    x, y, z = vertex
    layer = layers[z] if 0 <= z < len(layers) else None
    payload: dict[str, Any] = {"x": x, "y": y, "z": z, "layer": layer}
    if anchor is not None:
        payload["anchor"] = anchor
    return payload


def _grid_point_to_vertex_tuple(point: Any) -> tuple[int, int, int]:
    return (
        int(getattr(point, "x", 0)),
        int(getattr(point, "y", 0)),
        int(getattr(point, "z", 0)),
    )


def _closest_net_pad_for_vertex(
    board: BoardData,
    result: Any,
    net_id: int,
    vertex: tuple[int, int, int],
) -> Any | None:
    layer_name = _layer_name_for_z(board, vertex[2])
    if layer_name is None:
        return None
    vx_mm, vy_mm = _grid_vertex_to_mm(result, vertex)
    best_pad = None
    best_dist2 = float("inf")
    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            if "*.Cu" not in pad.layers and layer_name not in pad.layers:
                continue
            dx = vx_mm - float(pad.center[0])
            dy = vy_mm - float(pad.center[1])
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_pad = pad
    return best_pad


def _layer_name_for_z(board: BoardData, z: int) -> str | None:
    layers = board.copper_layers
    if 0 <= z < len(layers):
        return layers[z]
    return None


def _grid_vertex_to_mm(result: Any, vertex: tuple[int, int, int]) -> tuple[float, float]:
    pitch = float(getattr(result, "grid_pitch", 0.0))
    ox = float(getattr(result, "origin_x", 0.0))
    oy = float(getattr(result, "origin_y", 0.0))
    return ox + vertex[0] * pitch, oy + vertex[1] * pitch


def _vertex_inside_pad_clearance(
    board: BoardData,
    result: Any,
    vertex: tuple[int, int, int],
    pad: Any | None,
    clearance: float,
) -> bool:
    if pad is None:
        return False
    layer_name = _layer_name_for_z(board, vertex[2])
    if layer_name is None:
        return False
    if "*.Cu" not in pad.layers and layer_name not in pad.layers:
        return False
    x_mm, y_mm = _grid_vertex_to_mm(result, vertex)
    return _point_inside_expanded_pad(x_mm, y_mm, pad, clearance)


def _point_inside_expanded_pad(x_mm: float, y_mm: float, pad: Any, clearance: float) -> bool:
    cx = float(pad.center[0])
    cy = float(pad.center[1])
    angle = float(getattr(pad, "rotation_degrees", 0.0)) * math.pi / 180.0
    dx = x_mm - cx
    dy = y_mm - cy
    local_x = dx * cos(angle) + dy * sin(angle)
    local_y = -dx * sin(angle) + dy * cos(angle)

    shape = str(getattr(pad, "shape", "")).lower()
    half_x = float(pad.size[0]) * 0.5 + clearance
    half_y = float(pad.size[1]) * 0.5 + clearance
    if half_x <= 0.0 or half_y <= 0.0:
        return False
    if shape in {"circle", "oval", "ellipse"}:
        vx = local_x / half_x
        vy = local_y / half_y
        return vx * vx + vy * vy <= 1.0 + 1e-9
    return abs(local_x) <= half_x + 1e-9 and abs(local_y) <= half_y + 1e-9


def _path_occupied_vertex_shell(
    result: Any,
    path_grid: list[Any],
    planar_radius_mm: float,
    via_radius_mm: float,
) -> set[tuple[int, int, int]]:
    occupied = _path_occupied_vertices(result, path_grid, planar_radius_mm, via_radius_mm)
    if not occupied:
        return set()

    nx = int(getattr(result, "nx", 0))
    ny = int(getattr(result, "ny", 0))
    boundary: set[tuple[int, int, int]] = set()
    for x, y, z in occupied:
        for dx, dy in (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ):
            nxp = x + dx
            nyp = y + dy
            if nxp < 0 or nxp >= nx or nyp < 0 or nyp >= ny or (nxp, nyp, z) not in occupied:
                boundary.add((x, y, z))
                break
    return boundary


def _path_occupied_vertices(
    result: Any,
    path_grid: list[Any],
    planar_radius_mm: float,
    via_radius_mm: float,
) -> set[tuple[int, int, int]]:
    if len(path_grid) < 2:
        return set()

    pitch = float(getattr(result, "grid_pitch", 0.0))
    origin_x = float(getattr(result, "origin_x", 0.0))
    origin_y = float(getattr(result, "origin_y", 0.0))
    nx = int(getattr(result, "nx", 0))
    ny = int(getattr(result, "ny", 0))

    if pitch <= 0 or nx <= 0 or ny <= 0:
        return set()

    occupied: set[tuple[int, int, int]] = set()

    def grid_to_mm(x: int, y: int) -> tuple[float, float]:
        return origin_x + x * pitch, origin_y + y * pitch

    for start, end in zip(path_grid, path_grid[1:]):
        x1 = int(getattr(start, "x", 0))
        y1 = int(getattr(start, "y", 0))
        z1 = int(getattr(start, "z", 0))
        x2 = int(getattr(end, "x", 0))
        y2 = int(getattr(end, "y", 0))
        z2 = int(getattr(end, "z", 0))

        if z1 == z2:
            sx_mm, sy_mm = grid_to_mm(x1, y1)
            ex_mm, ey_mm = grid_to_mm(x2, y2)
            _mark_segment_vertices(
                occupied,
                sx_mm,
                sy_mm,
                ex_mm,
                ey_mm,
                z1,
                planar_radius_mm,
                pitch,
                origin_x,
                origin_y,
                nx,
                ny,
            )
        elif x1 == x2 and y1 == y2:
            cx_mm, cy_mm = grid_to_mm(x1, y1)
            for z in range(min(z1, z2), max(z1, z2) + 1):
                _mark_circle_vertices(
                    occupied,
                    cx_mm,
                    cy_mm,
                    z,
                    via_radius_mm,
                    pitch,
                    origin_x,
                    origin_y,
                    nx,
                    ny,
                )
    return occupied


def _mark_segment_vertices(
    occupied: set[tuple[int, int, int]],
    sx_mm: float,
    sy_mm: float,
    ex_mm: float,
    ey_mm: float,
    z: int,
    radius_mm: float,
    pitch_mm: float,
    origin_x_mm: float,
    origin_y_mm: float,
    nx: int,
    ny: int,
) -> None:
    min_x = max(0, int(math.floor((min(sx_mm, ex_mm) - radius_mm - origin_x_mm) / pitch_mm)))
    max_x = min(nx - 1, int(math.ceil((max(sx_mm, ex_mm) + radius_mm - origin_x_mm) / pitch_mm)))
    min_y = max(0, int(math.floor((min(sy_mm, ey_mm) - radius_mm - origin_y_mm) / pitch_mm)))
    max_y = min(ny - 1, int(math.ceil((max(sy_mm, ey_mm) + radius_mm - origin_y_mm) / pitch_mm)))
    for y in range(min_y, max_y + 1):
        py = origin_y_mm + y * pitch_mm
        for x in range(min_x, max_x + 1):
            px = origin_x_mm + x * pitch_mm
            if _distance_point_to_segment(px, py, sx_mm, sy_mm, ex_mm, ey_mm) <= radius_mm + 1e-9:
                occupied.add((x, y, z))


def _mark_circle_vertices(
    occupied: set[tuple[int, int, int]],
    cx_mm: float,
    cy_mm: float,
    z: int,
    radius_mm: float,
    pitch_mm: float,
    origin_x_mm: float,
    origin_y_mm: float,
    nx: int,
    ny: int,
) -> None:
    min_x = max(0, int(math.floor((cx_mm - radius_mm - origin_x_mm) / pitch_mm)))
    max_x = min(nx - 1, int(math.ceil((cx_mm + radius_mm - origin_x_mm) / pitch_mm)))
    min_y = max(0, int(math.floor((cy_mm - radius_mm - origin_y_mm) / pitch_mm)))
    max_y = min(ny - 1, int(math.ceil((cy_mm + radius_mm - origin_y_mm) / pitch_mm)))
    radius_sq = radius_mm * radius_mm
    for y in range(min_y, max_y + 1):
        py = origin_y_mm + y * pitch_mm
        for x in range(min_x, max_x + 1):
            px = origin_x_mm + x * pitch_mm
            dx = px - cx_mm
            dy = py - cy_mm
            if dx * dx + dy * dy <= radius_sq + 1e-9:
                occupied.add((x, y, z))


def _distance_point_to_segment(
    px: float,
    py: float,
    sx: float,
    sy: float,
    ex: float,
    ey: float,
) -> float:
    dx = ex - sx
    dy = ey - sy
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return math.hypot(px - sx, py - sy)
    t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    closest_x = sx + t * dx
    closest_y = sy + t * dy
    return math.hypot(px - closest_x, py - closest_y)
