from __future__ import annotations

import json
import os
from collections import deque
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

_DLL_DIR_HANDLES: list[Any] = []


@dataclass(frozen=True)
class RerouteOutcome:
    ok: bool
    message: str
    result: Any | None = None
    elapsed_seconds: float | None = None
    candidate_export_path: Path | None = None
    selector_external_only: bool = False


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
    max_ripped_clearance = _max_ripped_clearance(
        selector_board,
        [int(getattr(result, "net_id", 0)) for result in outcome.result],
    ) if selector_board is not None else 0.0
    freerouting_occurrences_by_net = _load_freerouting_occurrences_by_net(selector_board)
    nets = []
    for result in outcome.result:
        item = router_core.NetCandidateSet()
        item.net_id = int(getattr(result, "net_id", 0))
        candidate_paths_grid: list[list[Any]]
        candidate_paths_mm: list[list[Any]]
        candidate_via_counts: list[int]
        candidate_cover_vertices: list[list[Any]]
        if outcome.selector_external_only:
            candidate_paths_grid = []
            candidate_paths_mm = []
            candidate_via_counts = []
            candidate_cover_vertices = []
        else:
            candidate_paths_grid = list(getattr(result, "candidate_paths_grid", []))
            candidate_paths_mm = list(getattr(result, "candidate_paths_mm", []))
            candidate_via_counts = list(getattr(result, "candidate_via_counts", []))
            candidate_cover_vertices = list(getattr(result, "candidate_cover_vertices", []))
        boundary_payload = list(exported_boundary_by_net.get(item.net_id, []))
        candidate_preview_items: list[Any] = []
        external_summary = {"original": 0, "final": 0, "freerouting": 0, "vertices": 0}
        if selector_board is not None:
            _append_external_candidates_for_selector(
                router_core=router_core,
                board=selector_board,
                result=result,
                net_id=item.net_id,
                max_ripped_clearance=max_ripped_clearance,
                freerouting_occurrences=freerouting_occurrences_by_net.get(item.net_id, []),
                candidate_paths_grid=candidate_paths_grid,
                candidate_paths_mm=candidate_paths_mm,
                candidate_via_counts=candidate_via_counts,
                candidate_cover_vertices=candidate_cover_vertices,
                boundary_payload=boundary_payload,
                candidate_preview_items=candidate_preview_items,
                summary=external_summary,
            )
        item.candidate_paths_grid = candidate_paths_grid
        item.candidate_paths_mm = candidate_paths_mm
        item.candidate_via_counts = candidate_via_counts
        item.candidate_cover_vertices = candidate_cover_vertices
        # Keep Python-side results aligned with selector candidates so GUI preview
        # can render the exact candidate index chosen by Gurobi.
        try:
            setattr(result, "candidate_paths_grid", candidate_paths_grid)
            setattr(result, "candidate_paths_mm", candidate_paths_mm)
            setattr(result, "candidate_via_counts", candidate_via_counts)
            setattr(result, "candidate_cover_vertices", candidate_cover_vertices)
            setattr(result, "candidate_preview_items", candidate_preview_items)
        except Exception:
            pass
        (
            item.candidate_boundary_vertices,
            item.candidate_terminal_coords,
            item.candidate_terminal_groups,
        ) = _selector_boundary_payload_from_export(
            router_core,
            selector_board,
            result,
            item.net_id,
            candidate_paths_grid,
            boundary_payload,
        )
        item.candidate_terminal_groups = _normalize_terminal_groups_for_net(
            router_core,
            item.candidate_terminal_groups,
            item.candidate_terminal_coords,
        )
        print(
            "selector_external_candidates "
            f"net={item.net_id} "
            f"original={external_summary['original']} "
            f"final={external_summary['final']} "
            f"freerouting={external_summary['freerouting']} "
            f"boundary_vertices={external_summary['vertices']}",
            flush=True,
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
    result_by_net = {
        int(getattr(result, "net_id", 0)): result
        for result in outcome.result
    }
    for selection in selections:
        if not selection.selected_candidate_indices:
            print(
                f"selector_selected_candidate net={selection.net_id} selected=(none)",
                flush=True,
            )
            continue
        selected_index = int(selection.selected_candidate_indices[0])
        route_result = result_by_net.get(int(selection.net_id))
        preview_items = list(getattr(route_result, "candidate_preview_items", [])) if route_result is not None else []
        via_counts = list(getattr(route_result, "candidate_via_counts", [])) if route_result is not None else []
        preview = preview_items[selected_index] if 0 <= selected_index < len(preview_items) else None
        via_count = via_counts[selected_index] if 0 <= selected_index < len(via_counts) else None
        source = str(getattr(preview, "source", "unknown")) if preview is not None else "unknown"
        occurrence_index = int(getattr(preview, "occurrence_index", 0)) if preview is not None else 0
        extra = f" occurrence_index={occurrence_index}" if occurrence_index > 0 else ""
        via_text = str(via_count) if via_count is not None else "unknown"
        print(
            "selector_selected_candidate "
            f"net={selection.net_id} "
            f"index={selected_index} "
            f"source={source}"
            f"{extra} "
            f"via_count={via_text}",
            flush=True,
        )
    return SelectionResult(
        ok=bool(cpp_result.ok),
        selections=selections,
        solver=str(cpp_result.solver),
        message=str(cpp_result.message),
    )


def _normalize_terminal_groups_for_net(
    router_core: Any,
    candidate_terminal_groups: list[list[list[Any]]],
    candidate_terminal_coords: list[list[Any]],
) -> list[list[list[Any]]]:
    if not candidate_terminal_groups:
        return candidate_terminal_groups

    net_group_unions: list[set[tuple[int, int, int]]] = []
    for groups in candidate_terminal_groups:
        for gi, group in enumerate(groups):
            while len(net_group_unions) <= gi:
                net_group_unions.append(set())
            for point in group:
                net_group_unions[gi].add((
                    int(getattr(point, "x", 0)),
                    int(getattr(point, "y", 0)),
                    int(getattr(point, "z", 0)),
                ))

    if not net_group_unions:
        return candidate_terminal_groups

    # Keep path anchors reachable by terminal constraints even if they were not
    # part of a path's boundary-derived group due to rasterization aliasing.
    for terminals in candidate_terminal_coords:
        for gi, point in enumerate(terminals):
            while len(net_group_unions) <= gi:
                net_group_unions.append(set())
            net_group_unions[gi].add((
                int(getattr(point, "x", 0)),
                int(getattr(point, "y", 0)),
                int(getattr(point, "z", 0)),
            ))

    canonical_groups: list[list[Any]] = []
    for group_vertices in net_group_unions:
        canonical_groups.append([
            router_core.GridPoint(x, y, z)
            for (x, y, z) in sorted(group_vertices)
        ])

    normalized: list[list[list[Any]]] = []
    for _ in candidate_terminal_groups:
        normalized.append([
            list(group)
            for group in canonical_groups
        ])
    return normalized


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
    net_pads = _all_net_pads(board, net_id) if board is not None else []
    path_count = max(len(candidate_paths_grid), len(boundary_payload))
    for index in range(path_count):
        path = candidate_paths_grid[index] if index < len(candidate_paths_grid) else []
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

        if not path and not vertices_for_path:
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
        terminals_for_path: list[Any] = []
        if board is not None:
            start_tuple = (start_anchor.x, start_anchor.y, start_anchor.z)
            goal_tuple = (goal_anchor.x, goal_anchor.y, goal_anchor.z)

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

            if net_pads:
                for pad in net_pads:
                    layer_hint = next(
                        (
                            layer
                            for layer in getattr(pad, "layers", ())
                            if layer in {"F.Cu", "B.Cu"} or layer.startswith("In")
                        ),
                        "F.Cu",
                    )
                    pad_anchor = _mm_to_grid_vertex(
                        result,
                        float(pad.center[0]),
                        float(pad.center[1]),
                        _layer_index_from_name(board, layer_hint) or 0,
                    )
                    group = _group_from_pad(pad, pad_anchor)
                    groups_for_path.append(group)
                    terminals_for_path.append(group[0] if group else router_core.GridPoint(*pad_anchor))
            elif start_anchor is not None and goal_anchor is not None:
                start_pad = _closest_net_pad_for_vertex(board, result, net_id, start_tuple)
                goal_pad = _closest_net_pad_for_vertex(board, result, net_id, goal_tuple)
                start_group = _group_from_pad(start_pad, start_tuple)
                goal_group = _group_from_pad(goal_pad, goal_tuple)
                groups_for_path.append(start_group)
                groups_for_path.append(goal_group)
                terminals_for_path.append(start_group[0] if start_group else start_anchor)
                terminals_for_path.append(goal_group[0] if goal_group else goal_anchor)
        else:
            groups_for_path.append([start_anchor])
            groups_for_path.append([goal_anchor])
            terminals_for_path.append(start_anchor)
            terminals_for_path.append(goal_anchor)

        boundary_vertices.append(vertices_for_path)
        terminal_coords.append(terminals_for_path)
        terminal_groups.append(groups_for_path)
    return boundary_vertices, terminal_coords, terminal_groups


def _append_external_candidates_for_selector(
    router_core: Any,
    board: BoardData,
    result: Any,
    net_id: int,
    max_ripped_clearance: float,
    freerouting_occurrences: list[dict[str, Any]],
    candidate_paths_grid: list[list[Any]],
    candidate_paths_mm: list[list[Any]],
    candidate_via_counts: list[int],
    candidate_cover_vertices: list[list[Any]],
    boundary_payload: list[list[dict[str, Any]]],
    candidate_preview_items: list[Any],
    summary: dict[str, int],
) -> None:
    existing_keys: set[tuple[tuple[int, int, int], ...]] = set()
    for raw_vertices in boundary_payload:
        key = _boundary_payload_key(raw_vertices)
        if key:
            existing_keys.add(key)

    anchors = _default_anchors_for_result(result)
    start_anchor: tuple[int, int, int] | None = anchors[0] if anchors is not None else None
    goal_anchor: tuple[int, int, int] | None = anchors[1] if anchors is not None else None

    final_board = getattr(result, "final_candidate_board", None)
    try:
        final_net_id = int(getattr(result, "final_candidate_net_id", 0))
    except Exception:
        final_net_id = 0

    if final_board is not None and final_net_id > 0:
        final_segments, final_vias = _collect_original_route_primitives(
            board=final_board,
            result=result,
            net_id=final_net_id,
            max_ripped_clearance=max_ripped_clearance,
        )
        final_raw = _build_boundary_payload_from_primitives(
            board=final_board,
            result=result,
            net_id=final_net_id,
            start_anchor=start_anchor,
            goal_anchor=goal_anchor,
            segments=final_segments,
            vias=final_vias,
        )
        final_pad_reason = _external_candidate_pad_coverage_reason(
            board=board,
            result=result,
            net_id=net_id,
            segments=final_segments,
            vias=final_vias,
            explicit_graph=None,
        )
        if final_pad_reason is not None:
            print(
                f"selector_external_rejected_missing_pads net={net_id} source=final reason={final_pad_reason}",
                flush=True,
            )
            for line in _external_candidate_pad_debug_lines(
                board=board,
                result=result,
                net_id=net_id,
                segments=final_segments,
                vias=final_vias,
                explicit_graph=None,
                source_label="final",
                reason=final_pad_reason,
            ):
                print(line, flush=True)
        elif _append_external_payload_candidate(
            router_core=router_core,
            board=board,
            result=result,
            start_anchor=start_anchor,
            goal_anchor=goal_anchor,
            raw_vertices=final_raw,
            candidate_paths_grid=candidate_paths_grid,
            candidate_paths_mm=candidate_paths_mm,
            candidate_via_counts=candidate_via_counts,
            candidate_cover_vertices=candidate_cover_vertices,
            boundary_payload=boundary_payload,
            candidate_preview_items=candidate_preview_items,
            existing_keys=existing_keys,
            segments=final_segments,
            vias=final_vias,
            preview_item=_candidate_preview_item(
                net_id,
                final_segments,
                final_vias,
                0,
                "final",
            ),
        ):
            summary["final"] += 1
            summary["vertices"] += len(final_raw)
        else:
            print(
                f"selector_external_missing_boundary net={net_id} source=final",
                flush=True,
            )

    original_segments, original_vias = _collect_original_route_primitives(
        board=board,
        result=result,
        net_id=net_id,
        max_ripped_clearance=max_ripped_clearance,
    )
    original_raw = _build_boundary_payload_from_primitives(
        board=board,
        result=result,
        net_id=net_id,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        segments=original_segments,
        vias=original_vias,
    )
    original_pad_reason = _external_candidate_pad_coverage_reason(
        board=board,
        result=result,
        net_id=net_id,
        segments=original_segments,
        vias=original_vias,
        explicit_graph=None,
    )
    if original_pad_reason is not None:
        print(
            f"selector_external_rejected_missing_pads net={net_id} source=original reason={original_pad_reason}",
            flush=True,
        )
        for line in _external_candidate_pad_debug_lines(
            board=board,
            result=result,
            net_id=net_id,
            segments=original_segments,
            vias=original_vias,
            explicit_graph=None,
            source_label="original",
            reason=original_pad_reason,
        ):
            print(line, flush=True)
    elif _append_external_payload_candidate(
        router_core=router_core,
        board=board,
        result=result,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        raw_vertices=original_raw,
        candidate_paths_grid=candidate_paths_grid,
        candidate_paths_mm=candidate_paths_mm,
        candidate_via_counts=candidate_via_counts,
        candidate_cover_vertices=candidate_cover_vertices,
        boundary_payload=boundary_payload,
        candidate_preview_items=candidate_preview_items,
        existing_keys=existing_keys,
        segments=original_segments,
        vias=original_vias,
        preview_item=_candidate_preview_item(
            net_id,
            original_segments,
            original_vias,
            0,
            "original",
        ),
    ):
        summary["original"] += 1
        summary["vertices"] += len(original_raw)
    else:
        print(
            f"selector_external_missing_boundary net={net_id} source=original",
            flush=True,
        )

    for occurrence in freerouting_occurrences:
        occurrence_index = int(occurrence.get("event_index", 0))
        occ_segments, occ_vias = _collect_freerouting_occurrence_primitives(
            board=board,
            result=result,
            max_ripped_clearance=max_ripped_clearance,
            occurrence=occurrence,
        )
        occ_graph = _collect_freerouting_occurrence_graph(
            board=board,
            result=result,
            occurrence=occurrence,
        )
        occ_raw = _build_boundary_payload_from_primitives(
            board=board,
            result=result,
            net_id=net_id,
            start_anchor=start_anchor,
            goal_anchor=goal_anchor,
            segments=occ_segments,
            vias=occ_vias,
        )
        occ_pad_reason = _external_candidate_pad_coverage_reason(
            board=board,
            result=result,
            net_id=net_id,
            segments=occ_segments,
            vias=occ_vias,
            explicit_graph=occ_graph,
        )
        if occ_pad_reason is not None:
            print(
                "selector_external_rejected_missing_pads "
                f"net={net_id} source=freerouting occurrence_index={occurrence_index} reason={occ_pad_reason}",
                flush=True,
            )
            for line in _external_candidate_pad_debug_lines(
                board=board,
                result=result,
                net_id=net_id,
                segments=occ_segments,
                vias=occ_vias,
                explicit_graph=occ_graph,
                source_label=f"freerouting occurrence_index={occurrence_index}",
                reason=occ_pad_reason,
            ):
                print(line, flush=True)
        elif _append_external_payload_candidate(
            router_core=router_core,
            board=board,
            result=result,
            start_anchor=start_anchor,
            goal_anchor=goal_anchor,
            raw_vertices=occ_raw,
            candidate_paths_grid=candidate_paths_grid,
            candidate_paths_mm=candidate_paths_mm,
            candidate_via_counts=candidate_via_counts,
            candidate_cover_vertices=candidate_cover_vertices,
            boundary_payload=boundary_payload,
            candidate_preview_items=candidate_preview_items,
            existing_keys=existing_keys,
            segments=occ_segments,
            vias=occ_vias,
            explicit_graph=occ_graph,
            preview_item=_candidate_preview_item(
                net_id,
                occ_segments,
                occ_vias,
                occurrence_index,
                "freerouting",
            ),
        ):
            summary["freerouting"] += 1
            summary["vertices"] += len(occ_raw)
        else:
            print(
                f"selector_external_missing_boundary net={net_id} source=freerouting occurrence_index={occurrence_index}",
                flush=True,
            )


def _append_external_payload_candidate(
    router_core: Any,
    board: BoardData,
    result: Any,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    raw_vertices: list[dict[str, Any]],
    candidate_paths_grid: list[list[Any]],
    candidate_paths_mm: list[list[Any]],
    candidate_via_counts: list[int],
    candidate_cover_vertices: list[list[Any]],
    boundary_payload: list[list[dict[str, Any]]],
    candidate_preview_items: list[Any],
    existing_keys: set[tuple[tuple[int, int, int], ...]],
    segments: list[dict[str, Any]] | None = None,
    vias: list[dict[str, Any]] | None = None,
    explicit_graph: dict[str, Any] | None = None,
    preview_item: Any | None = None,
) -> bool:
    graph = _build_external_grid_graph(
        result=result,
        segments=segments or [],
        vias=vias or [],
        explicit_graph=explicit_graph,
    )
    if not graph and not raw_vertices:
        return False
    key = _boundary_payload_key(raw_vertices)
    if not key or key in existing_keys:
        return False
    existing_keys.add(key)
    boundary_payload.append(raw_vertices)
    candidate_via_counts.append(len(vias or []))
    cover_vertices = _primitive_occupied_vertices(result, segments or [], vias or [])
    candidate_cover_vertices.append([
        router_core.GridPoint(x, y, z)
        for (x, y, z) in sorted(cover_vertices)
    ])
    candidate_preview_items.append(
        preview_item
        if preview_item is not None
        else SimpleNamespace(net_id=int(getattr(result, "net_id", 0)))
    )
    return True


def _candidate_preview_item(
    net_id: int,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    occurrence_index: int,
    source: str,
) -> Any:
    return SimpleNamespace(
        net_id=int(net_id),
        occurrence_index=int(occurrence_index),
        source=str(source),
        preview_kind="candidate",
        segments=[
            SimpleNamespace(
                start=tuple(segment.get("start", (0.0, 0.0))),
                end=tuple(segment.get("end", (0.0, 0.0))),
                width=float(segment.get("width_mm", segment.get("width", 0.2))),
                layer=str(segment.get("layer", "F.Cu")),
            )
            for segment in segments
        ],
        vias=[
            SimpleNamespace(
                center=tuple(via.get("center", (0.0, 0.0))),
                diameter=float(via.get("diameter_mm", via.get("diameter", 0.6))),
                start_layer=str(via.get("start_layer", "F.Cu")),
                end_layer=str(via.get("end_layer", "B.Cu")),
            )
            for via in vias
        ],
    )


def _default_anchors_for_result(result: Any) -> tuple[tuple[int, int, int], tuple[int, int, int]] | None:
    start_vertices = list(getattr(result, "start_vertices", []))
    goal_vertices = list(getattr(result, "goal_vertices", []))
    if not start_vertices or not goal_vertices:
        return None
    return (
        _grid_point_to_vertex_tuple(start_vertices[0]),
        _grid_point_to_vertex_tuple(goal_vertices[0]),
    )


def _build_original_route_boundary_payload(
    board: BoardData,
    result: Any,
    net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    max_ripped_clearance: float,
) -> list[dict[str, Any]]:
    segments, vias = _collect_original_route_primitives(
        board=board,
        result=result,
        net_id=net_id,
        max_ripped_clearance=max_ripped_clearance,
    )
    return _build_boundary_payload_from_primitives(
        board=board,
        result=result,
        net_id=net_id,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        segments=segments,
        vias=vias,
    )


def _build_freerouting_occurrence_boundary_payload(
    board: BoardData,
    result: Any,
    net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    max_ripped_clearance: float,
    occurrence: dict[str, Any],
) -> list[dict[str, Any]]:
    segments, vias = _collect_freerouting_occurrence_primitives(
        board=board,
        result=result,
        max_ripped_clearance=max_ripped_clearance,
        occurrence=occurrence,
    )
    return _build_boundary_payload_from_primitives(
        board=board,
        result=result,
        net_id=net_id,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        segments=segments,
        vias=vias,
    )


def _collect_original_route_primitives(
    board: BoardData,
    result: Any,
    net_id: int,
    max_ripped_clearance: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    boundary_clearance = max(0.0, max_ripped_clearance) * 0.5
    segments: list[dict[str, Any]] = []
    for track in board.tracks:
        if int(track.net_id) != net_id:
            continue
        z = _layer_index_from_name(board, track.layer)
        if z is None:
            continue
        segments.append(
            {
                "start": (float(track.start[0]), float(track.start[1])),
                "end": (float(track.end[0]), float(track.end[1])),
                "layer": str(track.layer),
                "width_mm": float(track.width),
                "z": z,
                "radius_mm": float(track.width) * 0.5 + boundary_clearance,
            }
        )

    default_z_end = max(0, int(getattr(result, "nz", 0)) - 1)
    vias: list[dict[str, Any]] = []
    for via in board.vias:
        if int(via.net_id) != net_id:
            continue
        vias.append(
            {
                "center": (float(via.center[0]), float(via.center[1])),
                "diameter_mm": float(via.diameter),
                "start_layer": "F.Cu",
                "end_layer": "B.Cu",
                "radius_mm": float(via.diameter) * 0.5 + boundary_clearance,
                "z_start": 0,
                "z_end": default_z_end,
            }
        )
    return segments, vias


def _collect_freerouting_occurrence_primitives(
    board: BoardData,
    result: Any,
    max_ripped_clearance: float,
    occurrence: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    boundary_clearance = max(0.0, max_ripped_clearance) * 0.5
    segments: list[dict[str, Any]] = []
    for segment in occurrence.get("segments", []):
        z = _layer_index_from_name(board, str(segment.get("layer", "")))
        if z is None:
            continue
        sx = float(segment.get("start", (0.0, 0.0))[0])
        sy = float(segment.get("start", (0.0, 0.0))[1])
        ex = float(segment.get("end", (0.0, 0.0))[0])
        ey = float(segment.get("end", (0.0, 0.0))[1])
        width = float(segment.get("width_mm", 0.2))
        segments.append(
            {
                "start": (sx, sy),
                "end": (ex, ey),
                "layer": str(segment.get("layer", "")),
                "width_mm": width,
                "z": z,
                "radius_mm": width * 0.5 + boundary_clearance,
            }
        )

    default_z_end = max(0, int(getattr(result, "nz", 0)) - 1)
    vias: list[dict[str, Any]] = []
    for via in occurrence.get("vias", []):
        sx = _layer_index_from_name(board, str(via.get("start_layer", "")))
        ex = _layer_index_from_name(board, str(via.get("end_layer", "")))
        if sx is None and ex is None:
            z_start = 0
            z_end = default_z_end
        else:
            z_values = [z for z in (sx, ex) if z is not None]
            z_start = min(z_values)
            z_end = max(z_values)
        center = via.get("center", (0.0, 0.0))
        diameter = float(via.get("diameter_mm", 0.6))
        vias.append(
            {
                "center": (float(center[0]), float(center[1])),
                "diameter_mm": diameter,
                "start_layer": str(via.get("start_layer", "F.Cu")),
                "end_layer": str(via.get("end_layer", "B.Cu")),
                "radius_mm": diameter * 0.5 + boundary_clearance,
                "z_start": z_start,
                "z_end": z_end,
            }
        )
    return segments, vias


def _collect_freerouting_occurrence_graph(
    board: BoardData,
    result: Any,
    occurrence: dict[str, Any],
) -> dict[str, Any] | None:
    raw_graph = occurrence.get("graph")
    if not isinstance(raw_graph, dict):
        return None
    raw_nodes = raw_graph.get("nodes", [])
    raw_edges = raw_graph.get("edges", [])
    if not isinstance(raw_nodes, list) or not isinstance(raw_edges, list):
        return None

    nodes: list[dict[str, Any]] = []
    for node in raw_nodes:
        if not isinstance(node, dict):
            continue
        try:
            node_id = int(node.get("id", -1))
        except Exception:
            continue
        point = node.get("point", {})
        if not isinstance(point, dict):
            continue
        z = _layer_index_from_name(board, str(node.get("layer", "")))
        if z is None:
            continue
        vertex = _mm_to_grid_vertex(
            result,
            float(point.get("x_mm", 0.0)),
            -float(point.get("y_mm", 0.0)),
            z,
        )
        nodes.append({"id": node_id, "vertex": vertex})

    edges: list[dict[str, int]] = []
    for edge in raw_edges:
        if not isinstance(edge, dict):
            continue
        try:
            from_id = int(edge.get("from", -1))
            to_id = int(edge.get("to", -1))
        except Exception:
            continue
        edges.append({"from": from_id, "to": to_id})

    if not nodes or not edges:
        return None
    return {"nodes": nodes, "edges": edges}


def _build_boundary_payload_from_primitives(
    board: BoardData,
    result: Any,
    net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    shell = _primitive_occupied_vertex_shell(result, segments, vias)
    filtered = _filter_boundary_vertices_with_pads(
        board=board,
        result=result,
        net_id=net_id,
        vertices=shell,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        clearance=_clearance_for_net(board, net_id),
    )
    return _serialize_boundary_vertices_with_anchors(board, filtered, start_anchor, goal_anchor)


def _build_external_centerline_grid_path(
    result: Any,
    start_anchor: tuple[int, int, int],
    goal_anchor: tuple[int, int, int],
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None = None,
) -> list[tuple[int, int, int]]:
    graph = _build_external_grid_graph(
        result=result,
        segments=segments,
        vias=vias,
        explicit_graph=explicit_graph,
    )
    if not graph:
        return []
    start_vertex = _nearest_graph_vertex(start_anchor, graph)
    goal_vertex = _nearest_graph_vertex(goal_anchor, graph)
    if start_vertex is None or goal_vertex is None:
        return []
    return _shortest_grid_path(graph, start_vertex, goal_vertex)


def _graph_traversal_grid_path(
    graph: dict[tuple[int, int, int], set[tuple[int, int, int]]],
) -> list[tuple[int, int, int]]:
    if not graph:
        return []
    remaining = set(graph.keys())
    ordered_components: list[list[tuple[int, int, int]]] = []
    while remaining:
        start = min(remaining)
        stack = [start]
        seen: set[tuple[int, int, int]] = set()
        component_order: list[tuple[int, int, int]] = []
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            remaining.discard(current)
            component_order.append(current)
            neighbors = sorted(graph.get(current, set()), reverse=True)
            stack.extend(neighbors)
        if component_order:
            ordered_components.append(component_order)
    path: list[tuple[int, int, int]] = []
    for component in ordered_components:
        if path and path[-1] == component[0]:
            path.extend(component[1:])
        else:
            path.extend(component)
    return path


def _build_external_grid_graph(
    result: Any,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None = None,
) -> dict[tuple[int, int, int], set[tuple[int, int, int]]]:
    graph: dict[tuple[int, int, int], set[tuple[int, int, int]]] = {}

    def add_edge(a: tuple[int, int, int], b: tuple[int, int, int]) -> None:
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)

    if explicit_graph:
        vertices_by_id: dict[int, tuple[int, int, int]] = {}
        for node in explicit_graph.get("nodes", []):
            if not isinstance(node, dict):
                continue
            try:
                node_id = int(node.get("id", -1))
            except Exception:
                continue
            vertex = node.get("vertex")
            if not isinstance(vertex, (tuple, list)) or len(vertex) != 3:
                continue
            v = (int(vertex[0]), int(vertex[1]), int(vertex[2]))
            vertices_by_id[node_id] = v
            graph.setdefault(v, set())
        for edge in explicit_graph.get("edges", []):
            if not isinstance(edge, dict):
                continue
            try:
                from_id = int(edge.get("from", -1))
                to_id = int(edge.get("to", -1))
            except Exception:
                continue
            a = vertices_by_id.get(from_id)
            b = vertices_by_id.get(to_id)
            if a is None or b is None:
                continue
            for p0, p1 in zip(_expand_graph_edge(a, b), _expand_graph_edge(a, b)[1:]):
                add_edge(p0, p1)

    if graph:
        return graph

    for segment in segments:
        z = int(segment.get("z", -1))
        if z < 0:
            continue
        start = segment.get("start", (0.0, 0.0))
        end = segment.get("end", (0.0, 0.0))
        line = _rasterize_grid_line_2d(
            _mm_to_grid_vertex(result, float(start[0]), float(start[1]), z),
            _mm_to_grid_vertex(result, float(end[0]), float(end[1]), z),
        )
        if not line:
            continue
        graph.setdefault(line[0], set())
        for a, b in zip(line, line[1:]):
            add_edge(a, b)

    for via in vias:
        center = via.get("center", (0.0, 0.0))
        z_start = int(via.get("z_start", 0))
        z_end = int(via.get("z_end", 0))
        step = 1 if z_end >= z_start else -1
        column = [
            _mm_to_grid_vertex(result, float(center[0]), float(center[1]), z)
            for z in range(z_start, z_end + step, step)
        ]
        if not column:
            continue
        graph.setdefault(column[0], set())
        for a, b in zip(column, column[1:]):
            add_edge(a, b)
    return graph


def _expand_graph_edge(
    start: tuple[int, int, int],
    end: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    if start[2] == end[2]:
        return _rasterize_grid_line_2d(start, end)
    if start[0] == end[0] and start[1] == end[1]:
        step = 1 if end[2] >= start[2] else -1
        return [(start[0], start[1], z) for z in range(start[2], end[2] + step, step)]
    return [start, end]


def _external_candidate_pad_coverage_reason(
    board: BoardData,
    result: Any,
    net_id: int,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None,
) -> str | None:
    graph = _build_external_grid_graph(
        result=result,
        segments=segments,
        vias=vias,
        explicit_graph=explicit_graph,
    )
    if not graph:
        return "empty_graph"

    component_by_vertex: dict[tuple[int, int, int], int] = {}
    component_index = 0
    for start in graph.keys():
        if start in component_by_vertex:
            continue
        queue: deque[tuple[int, int, int]] = deque([start])
        component_by_vertex[start] = component_index
        while queue:
            current = queue.popleft()
            for nxt in graph.get(current, set()):
                if nxt in component_by_vertex:
                    continue
                component_by_vertex[nxt] = component_index
                queue.append(nxt)
        component_index += 1

    pad_components: list[int] = []
    unmatched_pads = 0
    total_pads = 0
    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            total_pads += 1
            matched_components = {
                component_by_vertex[vertex]
                for vertex in graph.keys()
                if _vertex_inside_pad_clearance(board, result, vertex, pad, 0.0)
            }
            if not matched_components:
                unmatched_pads += 1
                continue
            pad_components.extend(sorted(matched_components))

    if total_pads <= 0:
        return None
    if unmatched_pads > 0:
        return f"unmatched_pads={unmatched_pads}/{total_pads}"
    unique_components = sorted(set(pad_components))
    if len(unique_components) > 1:
        return f"pad_components={unique_components}"
    return None


def _external_candidate_pad_debug_lines(
    board: BoardData,
    result: Any,
    net_id: int,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None,
    source_label: str,
    reason: str,
) -> list[str]:
    graph = _build_external_grid_graph(
        result=result,
        segments=segments,
        vias=vias,
        explicit_graph=explicit_graph,
    )
    vertices = sorted(graph.keys())
    lines = [
        (
            "selector_external_pad_debug "
            f"net={net_id} source={source_label} reason={reason} "
            f"graph_vertices={len(vertices)} graph_edges={sum(len(v) for v in graph.values()) // 2}"
        )
    ]
    if not vertices:
        return lines

    mm_vertices = [
        (vertex, *_grid_vertex_to_mm(result, vertex))
        for vertex in vertices
    ]
    xs = [x_mm for _, x_mm, _ in mm_vertices]
    ys = [y_mm for _, _, y_mm in mm_vertices]
    lines.append(
        "selector_external_graph_bounds "
        f"net={net_id} source={source_label} "
        f"x_mm=[{min(xs):.4f},{max(xs):.4f}] "
        f"y_mm=[{min(ys):.4f},{max(ys):.4f}]"
    )

    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            matched_vertices = [
                vertex
                for vertex in vertices
                if _vertex_inside_pad_clearance(board, result, vertex, pad, 0.0)
            ]
            nearest = sorted(
                mm_vertices,
                key=lambda item: (
                    (item[1] - float(pad.center[0])) ** 2 + (item[2] - float(pad.center[1])) ** 2
                ) ** 0.5,
            )
            nearest_lines: list[str] = []
            for vertex, x_mm, y_mm in nearest[: min(8, len(nearest))]:
                dist = ((x_mm - float(pad.center[0])) ** 2 + (y_mm - float(pad.center[1])) ** 2) ** 0.5
                nearest_lines.append(
                    f"{vertex}@({x_mm:.4f},{y_mm:.4f}) d_mm={dist:.4f}"
                )
            layer_hint = next(
                (
                    layer
                    for layer in pad.layers
                    if layer in {"F.Cu", "B.Cu"} or layer.startswith("In")
                ),
                pad.layers[0] if pad.layers else "?",
            )
            pad_grid = _mm_to_grid_vertex(
                result,
                float(pad.center[0]),
                float(pad.center[1]),
                _layer_index_from_name(board, layer_hint) or 0,
            )
            lines.append(
                "selector_external_pad_detail "
                f"net={net_id} source={source_label} "
                f"pad={footprint.reference}:{pad.name} "
                f"center_mm=({float(pad.center[0]):.4f},{float(pad.center[1]):.4f}) "
                f"size_mm=({float(pad.size[0]):.4f},{float(pad.size[1]):.4f}) "
                f"rotation_deg={float(getattr(pad, 'rotation_degrees', 0.0)):.4f} "
                f"layer_hint={layer_hint} "
                f"center_grid={pad_grid} "
                f"matched_vertices={len(matched_vertices)}"
            )
            if nearest_lines:
                lines.append(
                    "selector_external_pad_nearest "
                    f"net={net_id} source={source_label} "
                    f"pad={footprint.reference}:{pad.name} "
                    + " | ".join(nearest_lines)
                )

    dump_limit = 120
    for index, (vertex, x_mm, y_mm) in enumerate(mm_vertices[:dump_limit]):
        lines.append(
            "selector_external_graph_vertex "
            f"net={net_id} source={source_label} "
            f"index={index} grid={vertex} mm=({x_mm:.4f},{y_mm:.4f})"
        )
    if len(mm_vertices) > dump_limit:
        lines.append(
            "selector_external_graph_vertex "
            f"net={net_id} source={source_label} omitted={len(mm_vertices) - dump_limit}"
        )
    return lines


def _mm_to_grid_vertex(result: Any, x_mm: float, y_mm: float, z: int) -> tuple[int, int, int]:
    pitch = float(getattr(result, "grid_pitch", 0.0))
    ox = float(getattr(result, "origin_x", 0.0))
    oy = float(getattr(result, "origin_y", 0.0))
    nx = int(getattr(result, "nx", 0))
    ny = int(getattr(result, "ny", 0))
    nz = int(getattr(result, "nz", 0))
    if pitch <= 0.0 or nx <= 0 or ny <= 0 or nz <= 0:
        return 0, 0, 0
    x = int(round((x_mm - ox) / pitch))
    y = int(round((y_mm - oy) / pitch))
    x = max(0, min(nx - 1, x))
    y = max(0, min(ny - 1, y))
    z = max(0, min(nz - 1, int(z)))
    return x, y, z


def _rasterize_grid_line_2d(
    start: tuple[int, int, int],
    end: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    x0, y0, z0 = start
    x1, y1, z1 = end
    if z0 != z1:
        return [start, end]

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x = x0
    y = y0
    points: list[tuple[int, int, int]] = []
    while True:
        points.append((x, y, z0))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


def _connect_anchor_to_graph(
    result: Any,
    graph: dict[tuple[int, int, int], set[tuple[int, int, int]]],
    anchor: tuple[int, int, int],
) -> None:
    if anchor in graph:
        return
    graph.setdefault(anchor, set())
    if not graph:
        return
    candidates = [vertex for vertex in graph.keys() if vertex != anchor]
    if not candidates:
        return

    ax, ay, az = anchor
    nearest = min(
        candidates,
        key=lambda vertex: (
            abs(vertex[0] - ax) + abs(vertex[1] - ay) + abs(vertex[2] - az),
            abs(vertex[2] - az),
        ),
    )

    def add_edge(a: tuple[int, int, int], b: tuple[int, int, int]) -> None:
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)

    target_xy_same_z = (nearest[0], nearest[1], az)
    for a, b in zip(_rasterize_grid_line_2d(anchor, target_xy_same_z), _rasterize_grid_line_2d(anchor, target_xy_same_z)[1:]):
        add_edge(a, b)
    if target_xy_same_z != nearest:
        z0 = target_xy_same_z[2]
        z1 = nearest[2]
        step = 1 if z1 >= z0 else -1
        prev = target_xy_same_z
        for z in range(z0 + step, z1 + step, step):
            curr = (target_xy_same_z[0], target_xy_same_z[1], z)
            add_edge(prev, curr)
            prev = curr


def _nearest_graph_vertex(
    anchor: tuple[int, int, int],
    graph: dict[tuple[int, int, int], set[tuple[int, int, int]]],
) -> tuple[int, int, int] | None:
    if not graph:
        return None
    ax, ay, az = anchor
    return min(
        graph.keys(),
        key=lambda vertex: (
            abs(vertex[0] - ax) + abs(vertex[1] - ay) + abs(vertex[2] - az),
            abs(vertex[2] - az),
        ),
    )


def _shortest_grid_path(
    graph: dict[tuple[int, int, int], set[tuple[int, int, int]]],
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    if start == goal:
        return [start]
    if start not in graph or goal not in graph:
        return []

    queue: deque[tuple[int, int, int]] = deque([start])
    prev: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for nxt in graph.get(current, set()):
            if nxt in prev:
                continue
            prev[nxt] = current
            queue.append(nxt)
    if goal not in prev:
        return []

    path: list[tuple[int, int, int]] = []
    node: tuple[int, int, int] | None = goal
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return path


def _fallback_external_path(
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    if start == goal:
        return [start]
    path: list[tuple[int, int, int]] = []
    same_z_goal_xy = (goal[0], goal[1], start[2])
    line = _rasterize_grid_line_2d(start, same_z_goal_xy)
    path.extend(line)
    if same_z_goal_xy != goal:
        step = 1 if goal[2] >= start[2] else -1
        for z in range(start[2] + step, goal[2] + step, step):
            point = (goal[0], goal[1], z)
            if not path or path[-1] != point:
                path.append(point)
    return path


def _filter_boundary_vertices_with_pads(
    board: BoardData,
    result: Any,
    net_id: int,
    vertices: set[tuple[int, int, int]],
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    clearance: float,
) -> set[tuple[int, int, int]]:
    anchor_vertices = {
        anchor for anchor in (start_anchor, goal_anchor)
        if anchor is not None
    }
    pads_to_filter = _all_net_pads(board, net_id)
    filtered: set[tuple[int, int, int]] = set()
    for vertex in vertices:
        if vertex in anchor_vertices:
            filtered.add(vertex)
            continue
        if any(
            _vertex_inside_pad_clearance(board, result, vertex, pad, clearance)
            for pad in pads_to_filter
        ):
            continue
        filtered.add(vertex)
    filtered.update(anchor_vertices)
    return filtered


def _serialize_boundary_vertices_with_anchors(
    board: BoardData,
    vertices: set[tuple[int, int, int]],
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
) -> list[dict[str, Any]]:
    serialized: list[dict[str, Any]] = []
    if start_anchor is not None:
        serialized.append(_serialize_grid_vertex_tuple(board, start_anchor, anchor="start"))
    if goal_anchor is not None and goal_anchor != start_anchor:
        serialized.append(_serialize_grid_vertex_tuple(board, goal_anchor, anchor="goal"))
    for vertex in sorted(vertices):
        if vertex == start_anchor or vertex == goal_anchor:
            continue
        serialized.append(_serialize_grid_vertex_tuple(board, vertex))
    return serialized


def _primitive_occupied_vertices(
    result: Any,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
) -> set[tuple[int, int, int]]:
    pitch = float(getattr(result, "grid_pitch", 0.0))
    origin_x = float(getattr(result, "origin_x", 0.0))
    origin_y = float(getattr(result, "origin_y", 0.0))
    nx = int(getattr(result, "nx", 0))
    ny = int(getattr(result, "ny", 0))
    nz = int(getattr(result, "nz", 0))
    if pitch <= 0.0 or nx <= 0 or ny <= 0 or nz <= 0:
        return set()

    occupied: set[tuple[int, int, int]] = set()
    for segment in segments:
        z = int(segment.get("z", -1))
        if z < 0 or z >= nz:
            continue
        start = segment.get("start", (0.0, 0.0))
        end = segment.get("end", (0.0, 0.0))
        radius_mm = float(segment.get("radius_mm", 0.0))
        _mark_segment_vertices(
            occupied,
            float(start[0]),
            float(start[1]),
            float(end[0]),
            float(end[1]),
            z,
            radius_mm,
            pitch,
            origin_x,
            origin_y,
            nx,
            ny,
        )

    for via in vias:
        center = via.get("center", (0.0, 0.0))
        radius_mm = float(via.get("radius_mm", 0.0))
        z_start = max(0, int(via.get("z_start", 0)))
        z_end = min(nz - 1, int(via.get("z_end", nz - 1)))
        if z_start > z_end:
            z_start, z_end = z_end, z_start
        for z in range(z_start, z_end + 1):
            _mark_circle_vertices(
                occupied,
                float(center[0]),
                float(center[1]),
                z,
                radius_mm,
                pitch,
                origin_x,
                origin_y,
                nx,
                ny,
            )

    return occupied


def _primitive_occupied_vertex_shell(
    result: Any,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
) -> set[tuple[int, int, int]]:
    occupied = _primitive_occupied_vertices(result, segments, vias)
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


def _boundary_payload_key(raw_vertices: list[dict[str, Any]]) -> tuple[tuple[int, int, int], ...]:
    points = []
    for item in raw_vertices:
        if not isinstance(item, dict):
            continue
        points.append((int(item.get("x", 0)), int(item.get("y", 0)), int(item.get("z", 0))))
    if not points:
        return tuple()
    return tuple(sorted(set(points)))


def _layer_index_from_name(board: BoardData, layer: str) -> int | None:
    normalized = str(layer)
    if normalized == "Top":
        normalized = "F.Cu"
    elif normalized == "Bottom":
        normalized = "B.Cu"
    for index, name in enumerate(board.copper_layers):
        if name == normalized:
            return index
    return None


def _load_freerouting_occurrences_by_net(board: BoardData | None) -> dict[int, list[dict[str, Any]]]:
    if board is None:
        return {}

    payload_path = None
    for candidate in _guess_freerouting_payload_paths(board.path):
        if candidate.exists():
            payload_path = candidate
            break
    if payload_path is None:
        return {}

    try:
        payload = json.loads(payload_path.read_text(encoding="utf-8"))
    except Exception:
        return {}

    mapping: dict[int, list[dict[str, Any]]] = {}
    nets = payload.get("nets", []) if isinstance(payload, dict) else []
    for net in nets:
        if not isinstance(net, dict):
            continue
        try:
            net_id = int(net.get("net_id", 0))
        except Exception:
            continue
        occurrences = net.get("occurrences", [])
        if not isinstance(occurrences, list) or not occurrences:
            occurrences = [net]
        parsed: list[dict[str, Any]] = []
        for occurrence in occurrences:
            if not isinstance(occurrence, dict):
                continue
            segs = []
            for segment in occurrence.get("segments", []):
                if not isinstance(segment, dict):
                    continue
                start = segment.get("start", {})
                end = segment.get("end", {})
                segs.append(
                    {
                        "layer": str(segment.get("layer", "F.Cu")),
                        "start": (
                            float(start.get("x_mm", 0.0)),
                            -float(start.get("y_mm", 0.0)),
                        ),
                        "end": (
                            float(end.get("x_mm", 0.0)),
                            -float(end.get("y_mm", 0.0)),
                        ),
                        "width_mm": float(segment.get("width_mm", 0.2)),
                    }
                )
            vias = []
            for via in occurrence.get("vias", []):
                if not isinstance(via, dict):
                    continue
                center = via.get("center", {})
                vias.append(
                    {
                        "center": (
                            float(center.get("x_mm", 0.0)),
                            -float(center.get("y_mm", 0.0)),
                        ),
                        "diameter_mm": float(via.get("diameter_mm", 0.6)),
                        "start_layer": str(via.get("start_layer", "F.Cu")),
                        "end_layer": str(via.get("end_layer", "B.Cu")),
                    }
                )
            if segs or vias:
                parsed.append(
                    {
                        "segments": segs,
                        "vias": vias,
                        "event_index": int(occurrence.get("event_index", 0)),
                        "source_net_id": occurrence.get("source_net_id"),
                        "graph": occurrence.get("graph"),
                    }
                )
        if parsed:
            mapping[net_id] = parsed
    return mapping


def _guess_freerouting_payload_paths(board_path: Path) -> list[Path]:
    app_root = Path(__file__).resolve().parents[1]
    work_dir = app_root / "out" / "freerouting_full"
    stems = [board_path.stem]
    if board_path.stem.endswith(".freerouting.routed"):
        stems.append(board_path.stem[: -len(".freerouting.routed")])
    guesses = [board_path.parent / "freerouting_ripped_routes.json"]
    for stem in stems:
        guesses.append(work_dir / stem / "freerouting_ripped_routes.json")
    return guesses


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


def _matching_net_id_by_name(board: BoardData | None, net_name: str) -> int:
    if board is None or not net_name:
        return 0
    matches = [int(net_id) for net_id, name in board.nets.items() if name == net_name]
    if len(matches) == 1:
        return matches[0]
    return 0


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


def build_freerouting_external_selector_outcome(
    board: BoardData,
    ripped_net_ids: set[int],
    grid_steps_per_mm: float = 10.0,
    final_board: BoardData | None = None,
) -> RerouteOutcome:
    if not ripped_net_ids:
        return RerouteOutcome(False, "No ripped nets were provided for freerouting selector input.")

    layers = board.copper_layers or ["F.Cu"]
    nz = len(layers)
    pitch = 1.0 / (grid_steps_per_mm if grid_steps_per_mm > 0.0 else 10.0)
    min_x, min_y, max_x, max_y = _board_bounds(board)
    margin = pitch * 2.0
    origin_x = min_x - margin
    origin_y = min_y - margin
    nx = max(3, int(math.ceil((max_x - min_x + margin * 2.0) / pitch)) + 1)
    ny = max(3, int(math.ceil((max_y - min_y + margin * 2.0) / pitch)) + 1)

    results: list[Any] = []
    skipped: list[int] = []
    for net_id in sorted(ripped_net_ids):
        net_pads = _all_net_pads(board, net_id)
        net_name = board.nets.get(net_id, "")
        final_net_id = _matching_net_id_by_name(final_board, net_name)
        anchors = _choose_external_selector_anchors(board, net_id)
        if anchors is None:
            skipped.append(net_id)
            continue
        start_mm, goal_mm = anchors
        start_grid = _mm_anchor_to_grid(start_mm, pitch, origin_x, origin_y, nx, ny, nz)
        goal_grid = _mm_anchor_to_grid(goal_mm, pitch, origin_x, origin_y, nx, ny, nz)
        start_point = SimpleNamespace(x=start_grid[0], y=start_grid[1], z=start_grid[2])
        goal_point = SimpleNamespace(x=goal_grid[0], y=goal_grid[1], z=goal_grid[2])
        results.append(
            SimpleNamespace(
                net_id=int(net_id),
                found=True,
                failure_reason="",
                grid_pitch=float(pitch),
                origin_x=float(origin_x),
                origin_y=float(origin_y),
                nx=int(nx),
                ny=int(ny),
                nz=int(nz),
                terminal_group_sizes=[1 for _ in net_pads] or [1, 1],
                net_pads=list(net_pads),
                final_candidate_board=final_board,
                final_candidate_net_id=int(final_net_id),
                start_vertices=[start_point],
                goal_vertices=[goal_point],
                candidate_paths_grid=[],
                candidate_paths_mm=[],
                path_grid=[start_point, goal_point],
                path_mm=[
                    SimpleNamespace(x=float(start_mm[0]), y=float(start_mm[1])),
                    SimpleNamespace(x=float(goal_mm[0]), y=float(goal_mm[1])),
                ],
            )
        )

    if not results:
        return RerouteOutcome(
            False,
            "No usable net anchors were found for freerouting external-only reroute.",
            None,
            None,
            None,
            True,
        )

    export_path = _write_selector_stub_manifest(
        board=board,
        net_ids=[int(getattr(item, "net_id", 0)) for item in results],
        grid_steps_per_mm=grid_steps_per_mm,
    )
    message = (
        f"Freerouting external-only candidates prepared for {len(results)} nets."
        + (f" Skipped nets: {skipped}." if skipped else "")
    )
    return RerouteOutcome(
        True,
        message,
        results,
        None,
        export_path,
        True,
    )


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


def _choose_external_selector_anchors(
    board: BoardData,
    net_id: int,
) -> tuple[tuple[float, float, int], tuple[float, float, int]] | None:
    points: list[tuple[float, float, int]] = []
    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            z = 0
            for layer in pad.layers:
                layer_index = _layer_index_from_name(board, layer)
                if layer_index is not None:
                    z = layer_index
                    break
            points.append((float(pad.center[0]), float(pad.center[1]), int(z)))

    for track in board.tracks:
        if int(track.net_id) != net_id:
            continue
        z = _layer_index_from_name(board, track.layer)
        if z is None:
            z = 0
        points.append((float(track.start[0]), float(track.start[1]), int(z)))
        points.append((float(track.end[0]), float(track.end[1]), int(z)))

    for via in board.vias:
        if int(via.net_id) != net_id:
            continue
        points.append((float(via.center[0]), float(via.center[1]), 0))
        points.append((float(via.center[0]), float(via.center[1]), max(0, len(board.copper_layers or ["F.Cu"]) - 1)))

    dedup: list[tuple[float, float, int]] = []
    seen: set[tuple[float, float, int]] = set()
    for px, py, pz in points:
        key = (round(px, 6), round(py, 6), int(pz))
        if key in seen:
            continue
        seen.add(key)
        dedup.append((px, py, pz))

    if not dedup:
        return None
    if len(dedup) == 1:
        return dedup[0], dedup[0]

    best_pair = (dedup[0], dedup[1])
    best_dist2 = -1.0
    for i in range(len(dedup)):
        for j in range(i + 1, len(dedup)):
            dx = dedup[i][0] - dedup[j][0]
            dy = dedup[i][1] - dedup[j][1]
            dist2 = dx * dx + dy * dy
            if dist2 > best_dist2:
                best_dist2 = dist2
                best_pair = (dedup[i], dedup[j])
    return best_pair


def _mm_anchor_to_grid(
    point: tuple[float, float, int],
    pitch: float,
    origin_x: float,
    origin_y: float,
    nx: int,
    ny: int,
    nz: int,
) -> tuple[int, int, int]:
    x = int(round((float(point[0]) - origin_x) / pitch))
    y = int(round((float(point[1]) - origin_y) / pitch))
    z = int(point[2])
    x = max(0, min(nx - 1, x))
    y = max(0, min(ny - 1, y))
    z = max(0, min(nz - 1, z))
    return x, y, z


def _write_selector_stub_manifest(
    board: BoardData,
    net_ids: list[int],
    grid_steps_per_mm: float,
) -> Path:
    root = Path(__file__).resolve().parents[1]
    export_dir = root / "out" / "candidate_path_exports"
    export_dir.mkdir(parents=True, exist_ok=True)
    board_stem = board.path.stem or "board"
    net_label = "-".join(str(net_id) for net_id in net_ids) or "none"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    export_path = export_dir / f"{board_stem}__freerouting_external_only__nets_{net_label}__{timestamp}.json"

    payload = {
        "schema_version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "board": {
            "path": str(board.path),
            "backend": board.backend,
            "copper_layers": list(board.copper_layers),
        },
        "ripped_net_ids": list(net_ids),
        "grid_steps_per_mm": float(grid_steps_per_mm),
        "results": [
            {"net_id": int(net_id), "candidate_boundary_vertices": []}
            for net_id in net_ids
        ],
    }
    export_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return export_path


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
    if hasattr(os, "add_dll_directory"):
        dll_candidates: list[Path] = []
        gurobi_home = os.environ.get("GUROBI_HOME")
        if gurobi_home:
            dll_candidates.append(Path(gurobi_home) / "bin")
        default_gurobi_bin = Path(r"C:\gurobi1103\win64\bin")
        if default_gurobi_bin.exists():
            dll_candidates.append(default_gurobi_bin)
        for dll_dir in dll_candidates:
            try:
                handle = os.add_dll_directory(str(dll_dir))
                _DLL_DIR_HANDLES.append(handle)
            except OSError:
                continue

    candidates = [
        root / "build" / "Release",
        root / "build" / "Debug",
        root / "build",
        root / "build-kicad" / "Release",
        root / "build-kicad" / "Debug",
        root / "build-kicad",
        root / "build-linux",
        root,
    ]
    for candidate in reversed(candidates):
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
            _serialize_route_result(
                board,
                result,
                _max_ripped_clearance(board, ripped_net_ids),
            )
            for result in results
        ],
    }
    export_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return export_path


def _serialize_route_result(
    board: BoardData,
    result: Any,
    max_ripped_clearance: float,
) -> dict[str, Any]:
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
            max_ripped_clearance,
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
    max_ripped_clearance: float,
) -> list[dict[str, Any]]:
    if not path_grid:
        return []

    net_id = int(getattr(result, "net_id", 0))
    start_anchor = _grid_point_to_vertex_tuple(path_grid[0])
    goal_anchor = _grid_point_to_vertex_tuple(path_grid[-1])
    start_pad = _closest_net_pad_for_vertex(board, result, net_id, start_anchor)
    goal_pad = _closest_net_pad_for_vertex(board, result, net_id, goal_anchor)

    boundary_clearance = max(0.0, max_ripped_clearance) * 0.5
    occupied = _path_occupied_vertex_shell(
        result,
        path_grid,
        trace_width * 0.5 + boundary_clearance,
        via_diameter * 0.5 + boundary_clearance,
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


def _max_ripped_clearance(board: BoardData, ripped_net_ids: list[int]) -> float:
    if not ripped_net_ids:
        return _min_clearance(board)
    values = [_clearance_for_net(board, int(net_id)) for net_id in ripped_net_ids]
    if not values:
        return _min_clearance(board)
    return max(values)


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


def _all_net_pads(board: BoardData | None, net_id: int) -> list[Any]:
    if board is None:
        return []
    pads: list[Any] = []
    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) == net_id:
                pads.append(pad)
    pads.sort(key=lambda pad: (float(pad.center[0]), float(pad.center[1]), str(getattr(pad, "name", ""))))
    return pads


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
