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
    left_top_reference_board: BoardData | None = None


def select_reroute_candidates(
    outcome: RerouteOutcome,
    max_paths_per_net: int = 1,
    prefer_gurobi: bool = True,
) -> SelectionResult:
    selector_started_at = perf_counter()
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

    load_started_at = perf_counter()
    exported_boundary_by_net = _load_exported_boundary_vertices(outcome.candidate_export_path)
    selector_board = _load_selector_board(outcome.candidate_export_path)
    freerouting_payload_paths = _load_external_payload_paths(outcome.candidate_export_path)
    max_ripped_clearance = _max_ripped_clearance(
        selector_board,
        [int(getattr(result, "net_id", 0)) for result in outcome.result],
    ) if selector_board is not None else 0.0
    freerouting_occurrences_by_net = _load_freerouting_occurrences_by_net(selector_board, freerouting_payload_paths)
    load_elapsed = perf_counter() - load_started_at

    candidate_prep_total = 0.0
    external_append_total = 0.0
    boundary_payload_total = 0.0
    terminal_normalize_total = 0.0
    external_geometry_cache: dict[tuple[Any, ...], tuple[list[dict[str, Any]], list[Any], str | None]] = {}
    zero_valid_net_labels: list[str] = []
    zero_valid_net_details: list[str] = []
    nets = []
    for result in outcome.result:
        net_started_at = perf_counter()
        item = router_core.NetCandidateSet()
        item.net_id = int(getattr(result, "net_id", 0))
        candidate_paths_grid: list[list[Any]]
        candidate_paths_mm: list[list[Any]]
        candidate_via_counts: list[int]
        candidate_bend_counts: list[int]
        candidate_lengths_mm: list[float]
        candidate_cover_vertices: list[list[Any]]
        if outcome.selector_external_only:
            candidate_paths_grid = []
            candidate_paths_mm = []
            candidate_via_counts = []
            candidate_bend_counts = []
            candidate_lengths_mm = []
            candidate_cover_vertices = []
        else:
            candidate_paths_grid = list(getattr(result, "candidate_paths_grid", []))
            candidate_paths_mm = list(getattr(result, "candidate_paths_mm", []))
            candidate_via_counts = list(getattr(result, "candidate_via_counts", []))
            candidate_bend_counts = list(getattr(result, "candidate_bend_counts", []))
            candidate_lengths_mm = list(getattr(result, "candidate_lengths_mm", []))
            candidate_cover_vertices = list(getattr(result, "candidate_cover_vertices", []))
        boundary_payload = list(exported_boundary_by_net.get(item.net_id, []))
        candidate_preview_items: list[Any] = []
        external_summary: dict[str, Any] = {
            "original": 0,
            "final": 0,
            "freerouting": 0,
            "pcbrouter": 0,
            "vertices": 0,
            "rejections": [],
            "missing_boundary": [],
        }
        precomputed_pad_groups: list[list[Any]] | None = None
        if selector_board is not None:
            precomputed_pad_groups = _cpp_pad_boundary_groups_for_net(
                router_core,
                selector_board,
                result,
                item.net_id,
            )
        if selector_board is not None:
            append_started_at = perf_counter()
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
                geometry_cache=external_geometry_cache,
            )
            append_elapsed = perf_counter() - append_started_at
        else:
            append_elapsed = 0.0
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
        boundary_started_at = perf_counter()
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
            precomputed_pad_groups=precomputed_pad_groups,
        )
        boundary_elapsed = perf_counter() - boundary_started_at
        normalize_started_at = perf_counter()
        item.candidate_terminal_groups = _normalize_terminal_groups_for_net(
            router_core,
            item.candidate_terminal_groups,
            item.candidate_terminal_coords,
        )
        candidate_lengths_mm = _candidate_lengths_for_selector(
            candidate_paths_mm,
            candidate_preview_items,
            len(item.candidate_boundary_vertices),
        )
        candidate_bend_counts = _candidate_bend_counts_for_selector(
            router_core=router_core,
            board=selector_board,
            result=result,
            candidate_paths_grid=candidate_paths_grid,
            candidate_preview_items=candidate_preview_items,
            candidate_terminal_groups=item.candidate_terminal_groups,
            candidate_count=len(item.candidate_boundary_vertices),
        )
        item.candidate_lengths_mm = candidate_lengths_mm
        item.candidate_bend_counts = candidate_bend_counts
        try:
            setattr(result, "candidate_lengths_mm", candidate_lengths_mm)
            setattr(result, "candidate_bend_counts", candidate_bend_counts)
        except Exception:
            pass
        normalize_elapsed = perf_counter() - normalize_started_at
        net_elapsed = perf_counter() - net_started_at
        candidate_prep_total += net_elapsed
        external_append_total += append_elapsed
        boundary_payload_total += boundary_elapsed
        terminal_normalize_total += normalize_elapsed
        print(
            "selector_external_candidates "
            f"net={item.net_id} "
            f"original={external_summary['original']} "
            f"final={external_summary['final']} "
            f"freerouting={external_summary['freerouting']} "
            f"pcbrouter={external_summary['pcbrouter']} "
            f"boundary_vertices={external_summary['vertices']}",
            flush=True,
        )
        print(
            "selector_timing_net "
            f"net={item.net_id} "
            f"append_sec={append_elapsed:.3f} "
            f"boundary_sec={boundary_elapsed:.3f} "
            f"normalize_sec={normalize_elapsed:.3f} "
            f"total_sec={net_elapsed:.3f}",
            flush=True,
        )
        valid_candidate_count = _valid_selector_candidate_count(
            item.candidate_paths_grid,
            item.candidate_boundary_vertices,
            item.candidate_terminal_coords,
            item.candidate_terminal_groups,
        )
        if valid_candidate_count <= 0:
            rejection_text = ", ".join(external_summary.get("rejections", [])) or "(none)"
            missing_boundary_text = ", ".join(external_summary.get("missing_boundary", [])) or "(none)"
            net_name = selector_board.nets.get(item.net_id, "") if selector_board is not None else ""
            zero_valid_line = (
                "selector_zero_candidate_net "
                f"net={item.net_id} "
                f"net_name={net_name!r} "
                f"candidate_slots={max(_candidate_slot_count(item), 0)} "
                f"valid_candidates=0 "
                f"original={external_summary['original']} "
                f"final={external_summary['final']} "
                f"freerouting={external_summary['freerouting']} "
                f"pcbrouter={external_summary['pcbrouter']} "
                f"rejections={rejection_text} "
                f"missing_boundary={missing_boundary_text}"
            )
            print(zero_valid_line, flush=True)
            zero_valid_net_details.append(zero_valid_line)
            zero_valid_net_labels.append(f"{item.net_id}:{net_name}" if net_name else str(item.net_id))
        nets.append(item)
    request.nets = nets

    final_witness_lines = _final_witness_diagnostics(
        outcome.result,
        nets,
    )
    for line in final_witness_lines:
        print(line, flush=True)

    gurobi_started_at = perf_counter()
    cpp_result = router_core.select_candidate_paths(request)
    gurobi_elapsed = perf_counter() - gurobi_started_at
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
    gurobi_solution_selected = bool(cpp_result.ok) and str(cpp_result.solver).lower().startswith("gurobi")
    selected_source_counts = {
        "freerouting": 0,
        "pcbrouter": 0,
        "other": 0,
    }
    for selection in selections:
        if not selection.selected_candidate_indices:
            print(
                f"selector_selected_candidate net={selection.net_id} selected=(none)",
                flush=True,
            )
            continue
        route_result = result_by_net.get(int(selection.net_id))
        preview_items = list(getattr(route_result, "candidate_preview_items", [])) if route_result is not None else []
        via_counts = list(getattr(route_result, "candidate_via_counts", [])) if route_result is not None else []
        bend_counts = list(getattr(route_result, "candidate_bend_counts", [])) if route_result is not None else []
        for raw_index in selection.selected_candidate_indices:
            selected_index = int(raw_index)
            preview = preview_items[selected_index] if 0 <= selected_index < len(preview_items) else None
            via_count = via_counts[selected_index] if 0 <= selected_index < len(via_counts) else None
            bend_count = bend_counts[selected_index] if 0 <= selected_index < len(bend_counts) else None
            source = str(getattr(preview, "source", "unknown")) if preview is not None else "unknown"
            if gurobi_solution_selected:
                # Count selected candidates by external router family for terminal diagnostics.
                selected_source_counts[_selected_external_router_source_bucket(source)] += 1
            source_net_id = int(getattr(preview, "source_net_id", selection.net_id)) if preview is not None else int(selection.net_id)
            occurrence_index = int(getattr(preview, "occurrence_index", 0)) if preview is not None else 0
            extra = f" occurrence_index={occurrence_index}" if occurrence_index > 0 else ""
            source_net_extra = f" source_net_id={source_net_id}"
            via_text = str(via_count) if via_count is not None else "unknown"
            bend_text = str(bend_count) if bend_count is not None else "unknown"
            print(
                "selector_selected_candidate "
                f"net={selection.net_id} "
                f"index={selected_index} "
                f"source={source}"
                f"{extra}"
                f"{source_net_extra} "
                f"via_count={via_text} "
                f"bend_count={bend_text}",
                flush=True,
            )
    if gurobi_solution_selected:
        print(
            "selector_selected_source_counts = "
            f"freerouting:{selected_source_counts['freerouting']}, "
            f"pcbrouter:{selected_source_counts['pcbrouter']}, "
            f"other:{selected_source_counts['other']}",
            flush=True,
        )
        for line in _selected_topology_diagnostics(selector_board, outcome.result, selections):
            print(line, flush=True)

    # Report the board currently shown in the top-left canvas when the GUI
    # supplies one. This keeps selector statistics comparable to the visible
    # reference board without changing the candidate set or the Gurobi model.
    left_top_board = outcome.left_top_reference_board or _freerouting_final_board_from_results(outcome.result)
    if left_top_board is not None:
        left_top_wire_length_mm = sum(
            math.hypot(track.end[0] - track.start[0], track.end[1] - track.start[1])
            for track in left_top_board.tracks
        )
        left_top_via_count = len(left_top_board.vias)
        print(
            f"selector_left_top_total_via_count = {left_top_via_count}  # 左上顯示 board 總 via 數",
            flush=True,
        )
        print(
            f"selector_left_top_total_wire_length_mm = {left_top_wire_length_mm:.3f}  # 左上顯示 board 總線長(mm)",
            flush=True,
        )

    selected_via_count, selected_wire_length_mm = _selected_solution_stats(outcome.result, selections)
    print(
        f"selector_selected_total_via_count = {selected_via_count}  # Gurobi 選出結果總 via 數",
        flush=True,
    )
    print(
        f"selector_selected_total_wire_length_mm = {selected_wire_length_mm:.3f}  # Gurobi 選出結果總線長(mm)",
        flush=True,
    )
    total_elapsed = perf_counter() - selector_started_at
    print(
        "selector_timing_summary "
        f"load_sec={load_elapsed:.3f} "
        f"candidate_prep_sec={candidate_prep_total:.3f} "
        f"append_sec={external_append_total:.3f} "
        f"boundary_sec={boundary_payload_total:.3f} "
        f"normalize_sec={terminal_normalize_total:.3f} "
        f"gurobi_sec={gurobi_elapsed:.3f} "
        f"total_sec={total_elapsed:.3f}",
        flush=True,
    )
    if zero_valid_net_labels:
        print(
            "selector_zero_candidate_nets = " + ", ".join(zero_valid_net_labels),
            flush=True,
        )
        for detail in zero_valid_net_details:
            print(detail, flush=True)
    return SelectionResult(
        ok=bool(cpp_result.ok),
        selections=selections,
        solver=str(cpp_result.solver),
        message=str(cpp_result.message),
    )


def _candidate_slot_count(item: Any) -> int:
    return max(
        len(getattr(item, "candidate_paths_grid", [])),
        len(getattr(item, "candidate_paths_mm", [])),
        len(getattr(item, "candidate_via_counts", [])),
        len(getattr(item, "candidate_bend_counts", [])),
        len(getattr(item, "candidate_lengths_mm", [])),
        len(getattr(item, "candidate_boundary_vertices", [])),
        len(getattr(item, "candidate_cover_vertices", [])),
        len(getattr(item, "candidate_terminal_coords", [])),
        len(getattr(item, "candidate_terminal_groups", [])),
    )


def _selected_external_router_source_bucket(source: str) -> str:
    """Return the external router family represented by a selected candidate source label."""
    normalized = str(source).lower()
    if "freerouting" in normalized:
        return "freerouting"
    if "pcbrouter" in normalized:
        return "pcbrouter"
    return "other"


class _TopologyUnionFind:
    """Track electrical connected components for selected candidate geometry."""

    def __init__(self) -> None:
        self.parent: dict[Any, Any] = {}

    def add(self, node: Any) -> None:
        """Register a graph node without changing any existing component."""
        if node not in self.parent:
            self.parent[node] = node

    def find(self, node: Any) -> Any:
        """Return a stable representative for the node component."""
        self.add(node)
        parent = self.parent[node]
        if parent != node:
            self.parent[node] = self.find(parent)
        return self.parent[node]

    def union(self, left: Any, right: Any) -> None:
        """Merge two nodes into the same electrical component."""
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def _selected_topology_diagnostics(
    board: BoardData | None,
    results: list[Any] | None,
    selections: list[PyNetSelection],
) -> list[str]:
    """Report post-selection net topology without changing the Gurobi model.

    The check builds an electrical graph from the selected preview geometry and
    then groups pads by connected component. Power-like nets receive a heuristic
    source/outlet role check; ordinary signal nets are reported as role-unknown
    because KiCad does not normally encode driver/receiver semantics.
    """
    if board is None or not results or not selections:
        return []

    result_by_net = {int(getattr(result, "net_id", 0)): result for result in results}
    checked_nets = 0
    split_pin_group_nets = 0
    source_exit_warning_nets = 0
    unknown_role_nets = 0
    padless_component_total = 0
    detail_lines: list[str] = []

    for selection in selections:
        net_id = int(selection.net_id)
        route_result = result_by_net.get(net_id)
        if route_result is None or not selection.selected_candidate_indices:
            continue
        preview_items = list(getattr(route_result, "candidate_preview_items", []))
        selected_previews = [
            preview_items[int(index)]
            for index in selection.selected_candidate_indices
            if 0 <= int(index) < len(preview_items)
        ]
        if not selected_previews:
            continue

        pad_entries = _topology_pad_entries(board, net_id)
        # Skip small nets: two- and three-pin nets cannot express the multi-source
        # topology ambiguity this diagnostic is meant to catch.
        if len(pad_entries) <= 3:
            continue

        checked_nets += 1
        net_name = board.nets.get(net_id, str(getattr(selected_previews[0], "net_name", "")))
        components, padless_components = _selected_net_pin_components(
            board,
            net_id,
            pad_entries,
            selected_previews,
        )
        padless_component_total += padless_components
        pin_components = [component for component in components if component["pads"]]
        if len(pin_components) < 2 and padless_components <= 0:
            continue

        split_pin_group_nets += 1 if len(pin_components) >= 2 else 0
        policy = _net_topology_role_policy(net_name)
        status = "ok"
        if policy == "signal_unknown":
            status = "unknown_signal_roles"
            unknown_role_nets += 1
        else:
            role_warning = any(
                not component["source_hints"] or not component["outlet_hints"]
                for component in pin_components
            )
            if role_warning:
                status = "missing_source_or_outlet_hint"
                source_exit_warning_nets += 1

        details = "; ".join(
            _format_topology_component(index, component)
            for index, component in enumerate(pin_components[:6])
        )
        if len(pin_components) > 6:
            details += f"; omitted_components={len(pin_components) - 6}"
        if padless_components:
            details = (details + "; " if details else "") + f"padless_copper_components={padless_components}"
        detail_lines.append(
            "selector_topology_net "
            f"net={net_id} "
            f"net_name={net_name!r} "
            f"pin_count={len(pad_entries)} "
            f"pin_groups={len(pin_components)} "
            f"source_policy={policy} "
            f"status={status} "
            f"components={details or '(none)'}"
        )

    lines = [
        "selector_topology_summary "
        f"checked_nets={checked_nets} "
        f"split_pin_group_nets={split_pin_group_nets} "
        f"source_exit_warning_nets={source_exit_warning_nets} "
        f"unknown_role_nets={unknown_role_nets} "
        f"padless_copper_components={padless_component_total}"
    ]
    lines.extend(detail_lines[:40])
    if len(detail_lines) > 40:
        lines.append(f"selector_topology_omitted_net_count = {len(detail_lines) - 40}")
    return lines


def _selected_net_pin_components(
    board: BoardData,
    net_id: int,
    pad_entries: list[tuple[str, str, str, Any]],
    selected_previews: list[Any],
) -> tuple[list[dict[str, Any]], int]:
    """Build selected-route components and list pads contained by each one."""
    union_find = _TopologyUnionFind()
    segment_entries: list[dict[str, Any]] = []
    via_entries: list[dict[str, Any]] = []

    for label, _reference, _pad_name, _pad in pad_entries:
        union_find.add(("pad", label))

    for preview_index, preview in enumerate(selected_previews):
        for segment_index, segment in enumerate(list(getattr(preview, "segments", []) or [])):
            start = tuple(getattr(segment, "start", (0.0, 0.0)))
            end = tuple(getattr(segment, "end", (0.0, 0.0)))
            if len(start) < 2 or len(end) < 2:
                continue
            layer_index = _layer_index_from_name(board, str(getattr(segment, "layer", "F.Cu") or "F.Cu"))
            if layer_index is None:
                continue
            start_node = ("xy", layer_index, *_topology_xy_key(float(start[0]), float(start[1])))
            end_node = ("xy", layer_index, *_topology_xy_key(float(end[0]), float(end[1])))
            union_find.union(start_node, end_node)
            segment_entries.append(
                {
                    "start": (float(start[0]), float(start[1])),
                    "end": (float(end[0]), float(end[1])),
                    "layer": layer_index,
                    "width": float(getattr(segment, "width", 0.0) or 0.0),
                    "node": start_node,
                    "preview_index": preview_index,
                    "segment_index": segment_index,
                }
            )

        for via in list(getattr(preview, "vias", []) or []):
            center = tuple(getattr(via, "center", (0.0, 0.0)))
            if len(center) < 2:
                continue
            start_layer = _layer_index_from_name(board, str(getattr(via, "start_layer", "F.Cu") or "F.Cu"))
            end_layer = _layer_index_from_name(board, str(getattr(via, "end_layer", "B.Cu") or "B.Cu"))
            if start_layer is None or end_layer is None:
                continue
            low_layer, high_layer = sorted((start_layer, end_layer))
            previous_node = None
            via_nodes = []
            for layer_index in range(low_layer, high_layer + 1):
                node = ("xy", layer_index, *_topology_xy_key(float(center[0]), float(center[1])))
                union_find.add(node)
                via_nodes.append((layer_index, node))
                if previous_node is not None:
                    union_find.union(previous_node, node)
                previous_node = node
            via_entries.append(
                {
                    "center": (float(center[0]), float(center[1])),
                    "diameter": float(getattr(via, "diameter", 0.0) or 0.0),
                    "nodes": via_nodes,
                }
            )

    _connect_overlapping_topology_segments(union_find, segment_entries)
    _connect_vias_to_topology_segments(union_find, via_entries, segment_entries)
    _connect_pads_to_topology_geometry(union_find, board, pad_entries, segment_entries, via_entries)

    components_by_root: dict[Any, dict[str, Any]] = {}
    for label, reference, pad_name, pad in pad_entries:
        root = union_find.find(("pad", label))
        component = components_by_root.setdefault(root, {"pads": [], "source_hints": [], "outlet_hints": []})
        component["pads"].append(label)
        if _pad_has_source_hint(board.nets.get(net_id, ""), reference, pad_name):
            component["source_hints"].append(label)
        else:
            component["outlet_hints"].append(label)

    copper_roots = {
        union_find.find(entry["node"])
        for entry in segment_entries
    }
    for via_entry in via_entries:
        for _layer_index, node in via_entry["nodes"]:
            copper_roots.add(union_find.find(node))
    pad_roots = set(components_by_root)
    padless_components = len(copper_roots - pad_roots)

    components = list(components_by_root.values())
    components.sort(key=lambda component: (-len(component["pads"]), component["pads"]))
    return components, padless_components


def _connect_overlapping_topology_segments(
    union_find: _TopologyUnionFind,
    segment_entries: list[dict[str, Any]],
) -> None:
    """Connect same-layer selected segments that physically touch or cross."""
    for index, left in enumerate(segment_entries):
        for right in segment_entries[index + 1:]:
            if int(left["layer"]) != int(right["layer"]):
                continue
            if _segments_intersect_or_touch(left["start"], left["end"], right["start"], right["end"]):
                union_find.union(left["node"], right["node"])


def _connect_vias_to_topology_segments(
    union_find: _TopologyUnionFind,
    via_entries: list[dict[str, Any]],
    segment_entries: list[dict[str, Any]],
) -> None:
    """Connect vias to selected segments that pass through their center."""
    for via_entry in via_entries:
        center = via_entry["center"]
        radius = max(float(via_entry["diameter"]) * 0.5, 0.001)
        node_by_layer = {int(layer): node for layer, node in via_entry["nodes"]}
        for segment in segment_entries:
            layer = int(segment["layer"])
            via_node = node_by_layer.get(layer)
            if via_node is None:
                continue
            if _point_to_segment_distance(center, segment["start"], segment["end"]) <= radius + 1e-6:
                union_find.union(via_node, segment["node"])


def _connect_pads_to_topology_geometry(
    union_find: _TopologyUnionFind,
    board: BoardData,
    pad_entries: list[tuple[str, str, str, Any]],
    segment_entries: list[dict[str, Any]],
    via_entries: list[dict[str, Any]],
) -> None:
    """Attach pad nodes to any selected same-net route geometry touching them."""
    for label, _reference, _pad_name, pad in pad_entries:
        pad_node = ("pad", label)
        for segment in segment_entries:
            layer_name = _layer_name_for_z(board, int(segment["layer"]))
            if layer_name is None or not _pad_has_selector_layer(board, pad, layer_name):
                continue
            if _segment_intersects_pad_body(segment["start"], segment["end"], pad):
                union_find.union(pad_node, segment["node"])
        for via_entry in via_entries:
            for layer_index, via_node in via_entry["nodes"]:
                layer_name = _layer_name_for_z(board, int(layer_index))
                if layer_name is None or not _pad_has_selector_layer(board, pad, layer_name):
                    continue
                if _point_inside_expanded_pad(via_entry["center"][0], via_entry["center"][1], pad, 0.0):
                    union_find.union(pad_node, via_node)


def _topology_pad_entries(board: BoardData, net_id: int) -> list[tuple[str, str, str, Any]]:
    """Return labeled pads for one net, preserving footprint references."""
    entries: list[tuple[str, str, str, Any]] = []
    for footprint in board.footprints:
        reference = str(getattr(footprint, "reference", "") or "")
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            pad_name = str(getattr(pad, "name", "") or "")
            entries.append((f"{reference}:{pad_name}", reference, pad_name, pad))
    entries.sort(key=lambda entry: entry[0])
    return entries


def _format_topology_component(index: int, component: dict[str, Any]) -> str:
    """Format a compact pin-component summary for terminal output."""
    pads = list(component.get("pads", []))
    sources = list(component.get("source_hints", []))
    outlets = list(component.get("outlet_hints", []))
    pad_text = ",".join(pads[:8]) + ("..." if len(pads) > 8 else "")
    return (
        f"component={index}:pins={len(pads)}:"
        f"source_hints={len(sources)}:"
        f"outlet_hints={len(outlets)}:"
        f"pads=[{pad_text}]"
    )


def _net_topology_role_policy(net_name: str) -> str:
    """Classify whether a net has a useful source/outlet heuristic."""
    normalized = str(net_name).upper().replace("/", "")
    if any(token in normalized for token in ("GND", "GROUND", "VSS", "AGND", "DGND")):
        return "ground_heuristic"
    power_tokens = ("VCC", "VDD", "VIN", "VBAT", "VBUS", "3V3", "3.3V", "5V", "+3", "+5")
    if any(token in normalized for token in power_tokens):
        return "power_heuristic"
    return "signal_unknown"


def _pad_has_source_hint(net_name: str, reference: str, pad_name: str) -> bool:
    """Guess whether a pad is a source/root for power-style topology reports."""
    policy = _net_topology_role_policy(net_name)
    if policy == "signal_unknown":
        return False
    ref = reference.upper()
    pad = pad_name.upper()
    source_prefixes = ("J", "JP", "P", "CN", "CON", "USB", "B", "BT", "BAT")
    if ref.startswith(source_prefixes):
        return True
    if policy == "ground_heuristic" and any(token in pad for token in ("GND", "VSS", "GROUND")):
        return True
    if policy == "power_heuristic" and any(token in pad for token in ("VIN", "VBUS", "BAT", "VOUT", "OUT")):
        return True
    return False


def _topology_xy_key(x_mm: float, y_mm: float) -> tuple[int, int]:
    """Quantize millimeter coordinates for exact endpoint-style topology keys."""
    scale = 10000.0
    return int(round(x_mm * scale)), int(round(y_mm * scale))


def _segment_intersects_pad_body(
    start: tuple[float, float],
    end: tuple[float, float],
    pad: Any,
) -> bool:
    """Return true when a segment touches the physical pad shape."""
    local_start = _pad_local_point(start[0], start[1], pad)
    local_end = _pad_local_point(end[0], end[1], pad)
    half_x = float(pad.size[0]) * 0.5
    half_y = float(pad.size[1]) * 0.5
    if half_x <= 0.0 or half_y <= 0.0:
        return False
    shape = str(getattr(pad, "shape", "")).lower()
    if shape in {"circle", "oval", "ellipse"}:
        scaled_start = (local_start[0] / half_x, local_start[1] / half_y)
        scaled_end = (local_end[0] / half_x, local_end[1] / half_y)
        return _point_to_segment_distance((0.0, 0.0), scaled_start, scaled_end) <= 1.0 + 1e-9
    if _point_inside_rect(local_start, half_x, half_y) or _point_inside_rect(local_end, half_x, half_y):
        return True
    corners = [
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ]
    return any(
        _segments_intersect_or_touch(local_start, local_end, corners[index], corners[(index + 1) % 4])
        for index in range(4)
    )


def _pad_local_point(x_mm: float, y_mm: float, pad: Any) -> tuple[float, float]:
    """Convert a board-space point into the pad's rotated local frame."""
    cx = float(pad.center[0])
    cy = float(pad.center[1])
    angle = float(getattr(pad, "rotation_degrees", 0.0)) * math.pi / 180.0
    dx = x_mm - cx
    dy = y_mm - cy
    return dx * cos(angle) + dy * sin(angle), -dx * sin(angle) + dy * cos(angle)


def _point_inside_rect(point: tuple[float, float], half_x: float, half_y: float) -> bool:
    """Check an axis-aligned local point against a rectangle."""
    return abs(point[0]) <= half_x + 1e-9 and abs(point[1]) <= half_y + 1e-9


def _segments_intersect_or_touch(
    a_start: tuple[float, float],
    a_end: tuple[float, float],
    b_start: tuple[float, float],
    b_end: tuple[float, float],
) -> bool:
    """Return true when two 2D segments intersect or touch."""
    def orientation(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> bool:
        return (
            min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
            and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
        )

    o1 = orientation(a_start, a_end, b_start)
    o2 = orientation(a_start, a_end, b_end)
    o3 = orientation(b_start, b_end, a_start)
    o4 = orientation(b_start, b_end, a_end)
    if o1 * o2 < -1e-12 and o3 * o4 < -1e-12:
        return True
    return (
        abs(o1) <= 1e-9 and on_segment(a_start, b_start, a_end)
        or abs(o2) <= 1e-9 and on_segment(a_start, b_end, a_end)
        or abs(o3) <= 1e-9 and on_segment(b_start, a_start, b_end)
        or abs(o4) <= 1e-9 and on_segment(b_start, a_end, b_end)
    )


def _point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Measure the shortest XY distance from a point to a segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
    projection = (start[0] + t * dx, start[1] + t * dy)
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def _candidate_lengths_for_selector(
    candidate_paths_mm: list[list[Any]],
    candidate_preview_items: list[Any],
    candidate_count: int,
) -> list[float]:
    """Return one physical wire-length coefficient for each selector candidate."""
    lengths: list[float] = []
    external_offset = len(candidate_paths_mm)
    for candidate_idx in range(candidate_count):
        if candidate_idx < len(candidate_paths_mm):
            lengths.append(_path_length_mm(candidate_paths_mm[candidate_idx]))
            continue
        preview_idx = candidate_idx - external_offset
        preview = candidate_preview_items[preview_idx] if 0 <= preview_idx < len(candidate_preview_items) else None
        lengths.append(_preview_wire_length_mm(preview))
    return lengths


def _candidate_bend_counts_for_selector(
    router_core: Any,
    board: BoardData | None,
    result: Any,
    candidate_paths_grid: list[list[Any]],
    candidate_preview_items: list[Any],
    candidate_terminal_groups: list[list[list[Any]]],
    candidate_count: int,
) -> list[int]:
    """Return pad-aware bend-count coefficients for each selector candidate.

    Bends inside terminal pad areas are ignored because those local access
    wiggles are pad-contact artifacts rather than meaningful routing turns.
    """
    bend_counts: list[int] = []
    external_offset = len(candidate_paths_grid)
    for candidate_idx in range(candidate_count):
        terminal_groups = (
            candidate_terminal_groups[candidate_idx]
            if candidate_idx < len(candidate_terminal_groups)
            else []
        )
        pad_boxes = _terminal_group_grid_boxes(terminal_groups)
        if candidate_idx < len(candidate_paths_grid) and candidate_paths_grid[candidate_idx]:
            bend_counts.append(_grid_path_bend_count_outside_pads(candidate_paths_grid[candidate_idx], pad_boxes))
            continue
        preview_idx = candidate_idx - external_offset
        preview = candidate_preview_items[preview_idx] if 0 <= preview_idx < len(candidate_preview_items) else None
        bend_counts.append(_preview_bend_count_outside_pads(router_core, board, result, preview, pad_boxes))
    return bend_counts


def _path_length_mm(path_mm: list[Any]) -> float:
    """Measure a polyline in millimeters using only its XY distance."""
    total = 0.0
    for start, end in zip(path_mm, path_mm[1:]):
        sx = float(getattr(start, "x", 0.0))
        sy = float(getattr(start, "y", 0.0))
        ex = float(getattr(end, "x", 0.0))
        ey = float(getattr(end, "y", 0.0))
        total += math.hypot(ex - sx, ey - sy)
    return total


def _preview_wire_length_mm(preview: Any | None) -> float:
    """Measure external-router preview segments in millimeters."""
    total = 0.0
    for segment in list(getattr(preview, "segments", []) or []):
        start = tuple(getattr(segment, "start", (0.0, 0.0)))
        end = tuple(getattr(segment, "end", (0.0, 0.0)))
        if len(start) < 2 or len(end) < 2:
            continue
        total += math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
    return total


def _terminal_group_grid_boxes(
    terminal_groups: list[list[Any]],
) -> list[tuple[int, int, int, int, set[int]]]:
    """Build coarse grid-space pad boxes from terminal group vertices."""
    boxes: list[tuple[int, int, int, int, set[int]]] = []
    for group in terminal_groups:
        vertices = [_grid_point_to_vertex_tuple(point) for point in group]
        if not vertices:
            continue
        xs = [vertex[0] for vertex in vertices]
        ys = [vertex[1] for vertex in vertices]
        zs = {vertex[2] for vertex in vertices}
        boxes.append((min(xs), max(xs), min(ys), max(ys), zs))
    return boxes


def _grid_vertex_inside_pad_box(
    vertex: tuple[int, int, int],
    pad_boxes: list[tuple[int, int, int, int, set[int]]],
) -> bool:
    """Return true when a grid vertex lies inside a terminal pad box."""
    x, y, z = vertex
    for min_x, max_x, min_y, max_y, layers in pad_boxes:
        if z in layers and min_x <= x <= max_x and min_y <= y <= max_y:
            return True
    return False


def _normalized_grid_axis(
    start: tuple[int, int, int],
    end: tuple[int, int, int],
) -> tuple[int, int] | None:
    """Return an orientation-only XY direction for bend comparisons."""
    dx = int(end[0]) - int(start[0])
    dy = int(end[1]) - int(start[1])
    if dx == 0 and dy == 0:
        return None
    divisor = math.gcd(abs(dx), abs(dy)) or 1
    nx = dx // divisor
    ny = dy // divisor
    if nx < 0 or (nx == 0 and ny < 0):
        nx = -nx
        ny = -ny
    return nx, ny


def _grid_path_bend_count_outside_pads(
    path_grid: list[Any],
    pad_boxes: list[tuple[int, int, int, int, set[int]]],
) -> int:
    """Count ordered path bends while resetting direction inside terminal pads."""
    vertices: list[tuple[int, int, int]] = []
    for point in path_grid:
        vertex = _grid_point_to_vertex_tuple(point)
        if not vertices or vertices[-1] != vertex:
            vertices.append(vertex)

    bend_count = 0
    previous_axis: tuple[int, int] | None = None
    for start, end in zip(vertices, vertices[1:]):
        axis = _normalized_grid_axis(start, end)
        if axis is None:
            continue
        if _grid_vertex_inside_pad_box(start, pad_boxes) or _grid_vertex_inside_pad_box(end, pad_boxes):
            previous_axis = None if _grid_vertex_inside_pad_box(end, pad_boxes) else axis
            continue
        if previous_axis is not None and previous_axis != axis:
            bend_count += 1
        previous_axis = axis
    return bend_count


def _preview_bend_count_outside_pads(
    router_core: Any,
    board: BoardData | None,
    result: Any,
    preview: Any | None,
    pad_boxes: list[tuple[int, int, int, int, set[int]]],
) -> int:
    """Count bends in an unordered external-router segment graph."""
    if board is None or preview is None:
        return 0
    adjacency: dict[tuple[int, int, int], set[tuple[int, int, int]]] = {}
    for segment in list(getattr(preview, "segments", []) or []):
        layer_name = str(getattr(segment, "layer", "F.Cu") or "F.Cu")
        layer_index = _layer_index_from_name(board, layer_name) or 0
        start = tuple(getattr(segment, "start", (0.0, 0.0)))
        end = tuple(getattr(segment, "end", (0.0, 0.0)))
        if len(start) < 2 or len(end) < 2:
            continue
        start_vertex = _mm_to_grid_vertex(result, float(start[0]), float(start[1]), layer_index)
        end_vertex = _mm_to_grid_vertex(result, float(end[0]), float(end[1]), layer_index)
        if start_vertex == end_vertex:
            continue
        adjacency.setdefault(start_vertex, set()).add(end_vertex)
        adjacency.setdefault(end_vertex, set()).add(start_vertex)

    bend_count = 0
    for vertex, neighbors in adjacency.items():
        if _grid_vertex_inside_pad_box(vertex, pad_boxes):
            continue
        axes = {
            axis
            for neighbor in neighbors
            for axis in [_normalized_grid_axis(vertex, neighbor)]
            if axis is not None
        }
        if len(axes) > 1:
            bend_count += len(axes) - 1
    return bend_count


def _freerouting_final_board_from_results(results: list[Any] | None) -> BoardData | None:
    if not results:
        return None
    for result in results:
        board = getattr(result, "final_candidate_board", None)
        if isinstance(board, BoardData):
            return board
    return None


def _selected_solution_stats(
    results: list[Any] | None,
    selections: list[PyNetSelection],
) -> tuple[int, float]:
    if not results or not selections:
        return 0, 0.0

    result_by_net = {
        int(getattr(result, "net_id", 0)): result
        for result in results
    }
    total_via_count = 0
    total_wire_length_mm = 0.0
    for selection in selections:
        route_result = result_by_net.get(int(selection.net_id))
        if route_result is None:
            continue
        preview_items = list(getattr(route_result, "candidate_preview_items", []))
        for raw_index in selection.selected_candidate_indices:
            selected_index = int(raw_index)
            if not (0 <= selected_index < len(preview_items)):
                continue
            preview = preview_items[selected_index]
            total_via_count += len(getattr(preview, "vias", []) or [])
            for segment in getattr(preview, "segments", []) or []:
                start = getattr(segment, "start", None)
                end = getattr(segment, "end", None)
                if (
                    not isinstance(start, tuple)
                    or not isinstance(end, tuple)
                    or len(start) < 2
                    or len(end) < 2
                ):
                    continue
                total_wire_length_mm += math.hypot(
                    float(end[0]) - float(start[0]),
                    float(end[1]) - float(start[1]),
                )
    return total_via_count, total_wire_length_mm


def _grid_point_key(point: Any) -> tuple[int, int, int]:
    return (
        int(getattr(point, "x", 0)),
        int(getattr(point, "y", 0)),
        int(getattr(point, "z", 0)),
    )


def _candidate_group_signature(
    cover_vertices: list[Any],
    canonical_terminal_groups: list[list[Any]],
) -> list[int]:
    cover_set = {_grid_point_key(point) for point in cover_vertices}
    signature: list[int] = []
    for group_index, terminal_group in enumerate(canonical_terminal_groups):
        hit = any(_grid_point_key(point) in cover_set for point in terminal_group)
        if hit:
            signature.append(group_index)
    return signature


def _final_witness_diagnostics(results: list[Any] | None, nets: list[Any]) -> list[str]:
    if not results or not nets:
        return []

    lines: list[str] = []
    result_by_net = {
        int(getattr(result, "net_id", 0)): result
        for result in results
    }

    selected_final_by_net: dict[int, int] = {}
    missing_final_nets: list[int] = []

    canonical_groups_by_net: dict[int, list[list[Any]]] = {}
    terminal_vertices = set()

    for item in nets:
        net_id = int(getattr(item, "net_id", 0))
        route_result = result_by_net.get(net_id)
        preview_items = list(getattr(route_result, "candidate_preview_items", [])) if route_result is not None else []
        final_index = None
        for idx, preview in enumerate(preview_items):
            if str(getattr(preview, "source", "")).startswith("final"):
                final_index = idx
                break
        if final_index is None:
            missing_final_nets.append(net_id)
        else:
            selected_final_by_net[net_id] = final_index

        canonical_groups = list(getattr(item, "candidate_terminal_groups", []))
        if canonical_groups:
            canonical_groups_by_net[net_id] = canonical_groups[0]
            for group in canonical_groups[0]:
                for point in group:
                    terminal_vertices.add(_grid_point_key(point))
        else:
            canonical_groups_by_net[net_id] = []

    lines.append(
        "selector_final_witness_status "
        f"nets_with_final={len(selected_final_by_net)} "
        f"nets_missing_final={len(missing_final_nets)}"
    )
    if missing_final_nets:
        lines.append(
            "selector_final_witness_missing_final_nets = "
            + ", ".join(str(net_id) for net_id in missing_final_nets)
        )

    terminal_violations: list[str] = []
    capacity_violations: list[str] = []

    occupied_by_net: dict[int, set[tuple[int, int, int]]] = {}

    for item in nets:
        net_id = int(getattr(item, "net_id", 0))
        final_index = selected_final_by_net.get(net_id)
        if final_index is None:
            continue

        cover_vertices_all = list(getattr(item, "candidate_cover_vertices", []))
        boundary_vertices_all = list(getattr(item, "candidate_boundary_vertices", []))
        terminal_groups_all = list(getattr(item, "candidate_terminal_groups", []))

        if not (0 <= final_index < len(cover_vertices_all)):
            terminal_violations.append(f"net={net_id}:final_index_out_of_range")
            continue

        cover_vertices = cover_vertices_all[final_index]
        occupied_vertices = (
            boundary_vertices_all[final_index]
            if 0 <= final_index < len(boundary_vertices_all)
            else []
        )
        canonical_terminal_groups = canonical_groups_by_net.get(net_id, [])
        hit_signature = _candidate_group_signature(cover_vertices, canonical_terminal_groups)

        multi_group = False
        if terminal_groups_all:
            signatures: set[tuple[int, ...]] = set()
            for candidate_index, groups in enumerate(terminal_groups_all):
                if candidate_index >= len(cover_vertices_all):
                    continue
                signature = tuple(
                    _candidate_group_signature(cover_vertices_all[candidate_index], canonical_terminal_groups)
                )
                if signature:
                    signatures.add(signature)
            multi_group = len(signatures) >= 2

        for gi in range(len(canonical_terminal_groups)):
            hit = gi in hit_signature
            if multi_group:
                if not hit:
                    terminal_violations.append(f"net={net_id}:terminal_group={gi}:requires_>=1")
            else:
                if not hit:
                    terminal_violations.append(f"net={net_id}:terminal_group={gi}:requires_==1")

        occupied_by_net[net_id] = {_grid_point_key(point) for point in occupied_vertices}

    ordered_net_ids = sorted(occupied_by_net.keys())
    for index, net_a in enumerate(ordered_net_ids):
        occupied_a = occupied_by_net[net_a]
        for net_b in ordered_net_ids[index + 1:]:
            shared = {
                vertex for vertex in occupied_a.intersection(occupied_by_net[net_b])
                if vertex not in terminal_vertices
            }
            if shared:
                capacity_violations.append(
                    f"nets={net_a},{net_b}:shared_nonterminal_vertices={len(shared)}"
                )

    lines.append(
        "selector_final_witness_summary "
        f"terminal_violations={len(terminal_violations)} "
        f"capacity_violations={len(capacity_violations)}"
    )
    if terminal_violations:
        lines.append(
            "selector_final_witness_terminal_violations = "
            + ", ".join(terminal_violations[:20])
            + (" ..." if len(terminal_violations) > 20 else "")
        )
    if capacity_violations:
        lines.append(
            "selector_final_witness_capacity_violations = "
            + ", ".join(capacity_violations[:20])
            + (" ..." if len(capacity_violations) > 20 else "")
        )
    return lines


def _valid_selector_candidate_count(
    candidate_paths_grid: list[list[Any]],
    candidate_boundary_vertices: list[list[Any]],
    candidate_terminal_coords: list[list[Any]],
    candidate_terminal_groups: list[list[list[Any]]],
) -> int:
    valid_count = 0
    candidate_count = max(
        len(candidate_paths_grid),
        len(candidate_boundary_vertices),
        len(candidate_terminal_coords),
        len(candidate_terminal_groups),
    )
    for candidate_idx in range(candidate_count):
        path_grid = candidate_paths_grid[candidate_idx] if candidate_idx < len(candidate_paths_grid) else []
        occupied_vertices = (
            candidate_boundary_vertices[candidate_idx]
            if candidate_idx < len(candidate_boundary_vertices) and candidate_boundary_vertices[candidate_idx]
            else path_grid
        )
        terminal_coords = list(candidate_terminal_coords[candidate_idx]) if candidate_idx < len(candidate_terminal_coords) else []
        terminal_groups = list(candidate_terminal_groups[candidate_idx]) if candidate_idx < len(candidate_terminal_groups) else []
        if not terminal_groups:
            terminal_groups = [[point] for point in terminal_coords]
        if not terminal_coords and terminal_groups:
            terminal_coords = [group[0] for group in terminal_groups if group]
        if occupied_vertices and terminal_coords:
            valid_count += 1
    return valid_count


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
    precomputed_pad_groups: list[list[Any]] | None = None,
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
                pad_index = net_pads.index(pad) if pad in net_pads else -1
                if precomputed_pad_groups is not None and 0 <= pad_index < len(precomputed_pad_groups):
                    group = [
                        router_core.GridPoint(int(getattr(point, "x", 0)), int(getattr(point, "y", 0)), int(getattr(point, "z", 0)))
                        for point in precomputed_pad_groups[pad_index]
                    ]
                else:
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
                            if layer in {"F.Cu", "B.Cu"} or layer.startswith("In") or layer.startswith("Route")
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
    summary: dict[str, Any],
    geometry_cache: dict[tuple[Any, ...], tuple[list[dict[str, Any]], list[Any], str | None]],
) -> None:
    # Break append time into smaller buckets so we can pinpoint the remaining hotspot.
    # This separates primitive collection, graph building, C++ raster work,
    # pad-coverage checks, and the final selector candidate insertion step.
    collect_total = 0.0
    raster_total = 0.0
    graph_total = 0.0
    pad_reason_total = 0.0
    candidate_append_total = 0.0
    cache_total = 0.0
    occurrence_total = 0.0
    occurrence_count = 0

    existing_keys: set[tuple[tuple[int, int, int], ...]] = set()
    for raw_vertices in boundary_payload:
        key = _boundary_payload_key(raw_vertices)
        if key:
            existing_keys.add(key)

    anchors = _default_anchors_for_result(result)
    start_anchor: tuple[int, int, int] | None = anchors[0] if anchors is not None else None
    goal_anchor: tuple[int, int, int] | None = anchors[1] if anchors is not None else None

    final_profiles = list(getattr(result, "final_candidate_profiles", []))
    if not final_profiles:
        final_board = getattr(result, "final_candidate_board", None)
        try:
            final_net_id = int(getattr(result, "final_candidate_net_id", 0))
        except Exception:
            final_net_id = 0
        if final_board is not None and final_net_id > 0:
            final_profiles = [
                SimpleNamespace(board=final_board, net_id=final_net_id, source="final")
            ]

    for final_profile in final_profiles:
        final_board = getattr(final_profile, "board", None)
        try:
            final_net_id = int(getattr(final_profile, "net_id", 0))
        except Exception:
            final_net_id = 0
        final_source = str(getattr(final_profile, "source", "final") or "final")
        if final_board is None or final_net_id <= 0:
            continue
        collect_started_at = perf_counter()
        final_segments, final_vias = _collect_original_route_primitives(
            board=final_board,
            result=result,
            net_id=final_net_id,
            max_ripped_clearance=max_ripped_clearance,
            layer_index_board=board,
        )
        collect_total += perf_counter() - collect_started_at
        cache_started_at = perf_counter()
        (
            final_raw,
            final_cover_vertices,
            final_pad_reason,
            cache_hit,
            raster_elapsed,
            pad_reason_elapsed,
        ) = _cached_cpp_external_candidate_analysis(
            router_core,
            final_board,
            board,
            result,
            net_id,
            final_net_id,
            start_anchor,
            goal_anchor,
            final_segments,
            final_vias,
            None,
            geometry_cache,
        )
        cache_total += perf_counter() - cache_started_at
        raster_total += raster_elapsed
        pad_reason_total += pad_reason_elapsed
        summary["geometry_cache_hits"] = int(summary.get("geometry_cache_hits", 0)) + int(cache_hit)
        summary["geometry_cache_misses"] = int(summary.get("geometry_cache_misses", 0)) + int(not cache_hit)
        if final_pad_reason is not None:
            summary.setdefault("rejections", []).append(f"final:{final_source}:{final_pad_reason}")
            print(
                f"selector_external_rejected_missing_pads net={net_id} source=final profile={final_source} reason={final_pad_reason}",
                flush=True,
            )
            for line in _external_candidate_pad_debug_lines(
                board=board,
                result=result,
                net_id=net_id,
                segments=final_segments,
                vias=final_vias,
                explicit_graph=None,
                source_label=f"final profile={final_source}",
                reason=final_pad_reason,
            ):
                print(line, flush=True)
        else:
            candidate_append_started_at = perf_counter()
            appended = _append_external_payload_candidate(
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
                cover_vertices_points=final_cover_vertices,
                preview_item=_candidate_preview_item(
                    net_id,
                    final_segments,
                    final_vias,
                    0,
                    f"final:{final_source}",
                    net_id,
                ),
            )
            candidate_append_total += perf_counter() - candidate_append_started_at
            if appended:
                summary["final"] += 1
                summary["vertices"] += len(final_raw)
            else:
                summary.setdefault("missing_boundary", []).append(f"final:{final_source}")
                print(
                    f"selector_external_missing_boundary net={net_id} source=final profile={final_source}",
                    flush=True,
                )

    original_profiles = list(getattr(result, "original_candidate_profiles", []))
    if not original_profiles:
        original_profiles = [SimpleNamespace(board=board, net_id=net_id, source="original")]

    for original_profile in original_profiles:
        original_board = getattr(original_profile, "board", board)
        try:
            original_net_id = int(getattr(original_profile, "net_id", net_id))
        except Exception:
            original_net_id = net_id
        original_source = str(getattr(original_profile, "source", "original") or "original")
        collect_started_at = perf_counter()
        original_segments, original_vias = _collect_original_route_primitives(
            board=original_board,
            result=result,
            net_id=original_net_id,
            max_ripped_clearance=max_ripped_clearance,
            layer_index_board=board,
        )
        collect_total += perf_counter() - collect_started_at
        cache_started_at = perf_counter()
        (
            original_raw,
            original_cover_vertices,
            original_pad_reason,
            cache_hit,
            raster_elapsed,
            pad_reason_elapsed,
        ) = _cached_cpp_external_candidate_analysis(
            router_core,
            original_board,
            board,
            result,
            net_id,
            original_net_id,
            start_anchor,
            goal_anchor,
            original_segments,
            original_vias,
            None,
            geometry_cache,
        )
        cache_total += perf_counter() - cache_started_at
        raster_total += raster_elapsed
        pad_reason_total += pad_reason_elapsed
        summary["geometry_cache_hits"] = int(summary.get("geometry_cache_hits", 0)) + int(cache_hit)
        summary["geometry_cache_misses"] = int(summary.get("geometry_cache_misses", 0)) + int(not cache_hit)
        if original_pad_reason is not None:
            summary.setdefault("rejections", []).append(f"original:{original_source}:{original_pad_reason}")
            print(
                f"selector_external_rejected_missing_pads net={net_id} source=original profile={original_source} reason={original_pad_reason}",
                flush=True,
            )
            for line in _external_candidate_pad_debug_lines(
                board=board,
                result=result,
                net_id=net_id,
                segments=original_segments,
                vias=original_vias,
                explicit_graph=None,
                source_label=f"original profile={original_source}",
                reason=original_pad_reason,
            ):
                print(line, flush=True)
        else:
            candidate_append_started_at = perf_counter()
            appended = _append_external_payload_candidate(
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
                cover_vertices_points=original_cover_vertices,
                preview_item=_candidate_preview_item(
                    net_id,
                    original_segments,
                    original_vias,
                    0,
                    f"original:{original_source}",
                    original_net_id,
                ),
            )
            candidate_append_total += perf_counter() - candidate_append_started_at
            if appended:
                summary["original"] += 1
                summary["vertices"] += len(original_raw)
            else:
                summary.setdefault("missing_boundary", []).append(f"original:{original_source}")
                print(
                    f"selector_external_missing_boundary net={net_id} source=original profile={original_source}",
                    flush=True,
                )

    occurrence_entries: list[Any] = []
    for occurrence in freerouting_occurrences:
        occurrence_started_at = perf_counter()
        occurrence_count += 1
        occurrence_index = int(occurrence.get("event_index", 0))
        source_backend = str(occurrence.get("source_backend", "freerouting") or "freerouting")
        source_profile = str(occurrence.get("source_profile", "selected") or "selected")
        source_label = source_backend if source_profile == "selected" else f"{source_backend}:{source_profile}"
        collect_started_at = perf_counter()
        occ_segments, occ_vias = _collect_freerouting_occurrence_primitives(
            board=board,
            result=result,
            max_ripped_clearance=max_ripped_clearance,
            occurrence=occurrence,
        )
        collect_total += perf_counter() - collect_started_at
        graph_started_at = perf_counter()
        occ_graph = _collect_freerouting_occurrence_graph(
            board=board,
            result=result,
            occurrence=occurrence,
        )
        graph_total += perf_counter() - graph_started_at
        occurrence_entries.append(
            SimpleNamespace(
                source_board=board,
                selector_net_id=net_id,
                source_net_id=net_id,
                preview_source_net_id=int(occurrence.get("source_net_id", net_id) or net_id),
                start_anchor=start_anchor,
                goal_anchor=goal_anchor,
                segments=occ_segments,
                vias=occ_vias,
                explicit_graph=occ_graph,
                occurrence_index=occurrence_index,
                source_label=source_label,
                occurrence_started_at=occurrence_started_at,
            )
        )
        occurrence_total += perf_counter() - occurrence_started_at

    cache_started_at = perf_counter()
    occurrence_analyses = _batch_cpp_external_candidate_analysis(
        router_core=router_core,
        selector_board=board,
        result=result,
        entries=occurrence_entries,
        geometry_cache=geometry_cache,
    )
    cache_total += perf_counter() - cache_started_at

    for occurrence_entry, occurrence_analysis in zip(occurrence_entries, occurrence_analyses):
        (
            occ_raw,
            occ_cover_vertices,
            occ_pad_reason,
            cache_hit,
            raster_elapsed,
            pad_reason_elapsed,
        ) = occurrence_analysis
        occurrence_index = int(getattr(occurrence_entry, "occurrence_index", 0))
        source_label = str(getattr(occurrence_entry, "source_label", "freerouting"))
        occ_segments = list(getattr(occurrence_entry, "segments", []))
        occ_vias = list(getattr(occurrence_entry, "vias", []))
        occ_graph = getattr(occurrence_entry, "explicit_graph", None)
        raster_total += raster_elapsed
        pad_reason_total += pad_reason_elapsed
        summary["geometry_cache_hits"] = int(summary.get("geometry_cache_hits", 0)) + int(cache_hit)
        summary["geometry_cache_misses"] = int(summary.get("geometry_cache_misses", 0)) + int(not cache_hit)
        if occ_pad_reason is not None:
            summary.setdefault("rejections", []).append(
                f"{source_label}@E{occurrence_index}:{occ_pad_reason}"
            )
            print(
                "selector_external_rejected_missing_pads "
                f"net={net_id} source={source_label} occurrence_index={occurrence_index} reason={occ_pad_reason}",
                flush=True,
            )
            for line in _external_candidate_pad_debug_lines(
                board=board,
                result=result,
                net_id=net_id,
                segments=occ_segments,
                vias=occ_vias,
                explicit_graph=occ_graph,
                source_label=f"{source_label} occurrence_index={occurrence_index}",
                    reason=occ_pad_reason,
                ):
                    print(line, flush=True)
        else:
            candidate_append_started_at = perf_counter()
            appended = _append_external_payload_candidate(
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
                cover_vertices_points=occ_cover_vertices,
                preview_item=_candidate_preview_item(
                    net_id,
                    occ_segments,
                    occ_vias,
                    occurrence_index,
                    source_label,
                    int(getattr(occurrence_entry, "preview_source_net_id", net_id) or net_id),
                ),
            )
            candidate_append_total += perf_counter() - candidate_append_started_at
            if appended:
                if source_backend == "pcbrouter":
                    summary["pcbrouter"] += 1
                else:
                    summary["freerouting"] += 1
                summary["vertices"] += len(occ_raw)
            else:
                summary.setdefault("missing_boundary", []).append(f"{source_label}@E{occurrence_index}")
                print(
                    f"selector_external_missing_boundary net={net_id} source={source_label} occurrence_index={occurrence_index}",
                    flush=True,
                )

    print(
        "selector_timing_append_breakdown "
        f"net={net_id} "
        f"collect_sec={collect_total:.3f} "
        f"graph_sec={graph_total:.3f} "
        f"raster_sec={raster_total:.3f} "
        f"pad_reason_sec={pad_reason_total:.3f} "
        f"candidate_append_sec={candidate_append_total:.3f} "
        f"geometry_cache_sec={cache_total:.3f} "
        f"geometry_cache_hits={int(summary.get('geometry_cache_hits', 0))} "
        f"geometry_cache_misses={int(summary.get('geometry_cache_misses', 0))} "
        f"occurrence_count={occurrence_count} "
        f"occurrence_total_sec={occurrence_total:.3f}",
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
    cover_vertices_points: list[Any] | None = None,
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
    if cover_vertices_points is not None:
        candidate_cover_vertices.append(list(cover_vertices_points))
    else:
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
    source_net_id: int | None = None,
) -> Any:
    return SimpleNamespace(
        net_id=int(net_id),
        occurrence_index=int(occurrence_index),
        source=str(source),
        source_net_id=int(source_net_id if source_net_id is not None else net_id),
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
    layer_index_board: BoardData | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    # Track/via geometry may come from a board that uses different copper-layer
    # names than the selector board. Cross-board mapping is based only on copper
    # stack order: source layer index k maps to target layer index k.
    mapping_board = layer_index_board if layer_index_board is not None else board
    boundary_clearance = max(0.0, max_ripped_clearance) * 0.5
    segments: list[dict[str, Any]] = []
    for track in board.tracks:
        if int(track.net_id) != net_id:
            continue
        z = _map_layer_index_by_stack_order(board, mapping_board, track.layer)
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
        center = (float(via.center[0]), float(via.center[1]))
        diameter = float(via.diameter)
        radius = diameter * 0.5 + boundary_clearance
        z_values = _active_via_layer_indices(
            source_board=board,
            mapping_board=mapping_board,
            net_id=net_id,
            center=center,
            radius_mm=max(radius, float(getattr(result, "grid_pitch", 0.0))),
            segments=segments,
        )
        if not z_values:
            start_z = _map_layer_index_by_stack_order(board, mapping_board, getattr(via, "start_layer", "F.Cu"))
            end_z = _map_layer_index_by_stack_order(board, mapping_board, getattr(via, "end_layer", "B.Cu"))
            z_values = [z for z in (start_z, end_z) if z is not None]
        if not z_values:
            z_values = [0, default_z_end]
        z_start = min(z_values)
        z_end = max(z_values)
        vias.append(
            {
                "center": center,
                "diameter_mm": diameter,
                "start_layer": _layer_name_for_z(mapping_board, z_start) or getattr(via, "start_layer", "F.Cu"),
                "end_layer": _layer_name_for_z(mapping_board, z_end) or getattr(via, "end_layer", "B.Cu"),
                "radius_mm": radius,
                "z_start": z_start,
                "z_end": z_end,
            }
        )
    return segments, vias


def _active_via_layer_indices(
    source_board: BoardData,
    mapping_board: BoardData,
    net_id: int,
    center: tuple[float, float],
    radius_mm: float,
    segments: list[dict[str, Any]],
) -> list[int]:
    """Infer which selector layers a via actually connects in this candidate."""
    active_layers: set[int] = set()
    cx, cy = center
    tolerance = max(radius_mm, 0.0) + 1e-6
    for segment in segments:
        start = segment.get("start", (0.0, 0.0))
        end = segment.get("end", (0.0, 0.0))
        if _distance_point_to_segment_mm(cx, cy, float(start[0]), float(start[1]), float(end[0]), float(end[1])) <= tolerance:
            active_layers.add(int(segment.get("z", 0)))
    for pad in _all_net_pads(source_board, net_id):
        if not _point_inside_expanded_pad(cx, cy, pad, radius_mm):
            continue
        for layer in getattr(pad, "layers", ()):
            z = _map_layer_index_by_stack_order(source_board, mapping_board, str(layer))
            if z is not None:
                active_layers.add(z)
    return sorted(active_layers)


def _distance_point_to_segment_mm(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    """Measure the shortest 2D distance from a point to a segment."""
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


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


def _cpp_raster_request(
    router_core: Any,
    board: BoardData,
    result: Any,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    pads: list[Any],
    pad_clearance: float,
    anchor_vertices: list[tuple[int, int, int]],
    explicit_graph: dict[str, Any] | None = None,
) -> Any:
    request = router_core.RasterRequest()
    request.grid_pitch = float(getattr(result, "grid_pitch", 0.0))
    request.origin_x = float(getattr(result, "origin_x", 0.0))
    request.origin_y = float(getattr(result, "origin_y", 0.0))
    request.nx = int(getattr(result, "nx", 0))
    request.ny = int(getattr(result, "ny", 0))
    request.nz = int(getattr(result, "nz", 0))
    request.pad_clearance = float(pad_clearance)
    request.anchor_vertices = [
        router_core.GridPoint(int(x), int(y), int(z))
        for (x, y, z) in anchor_vertices
    ]

    raster_segments: list[Any] = []
    for segment in segments:
        item = router_core.RasterSegment()
        start = segment.get("start", (0.0, 0.0))
        end = segment.get("end", (0.0, 0.0))
        item.start = router_core.Point2D(float(start[0]), float(start[1]))
        item.end = router_core.Point2D(float(end[0]), float(end[1]))
        item.radius_mm = float(segment.get("radius_mm", 0.0))
        item.z = int(segment.get("z", 0))
        raster_segments.append(item)
    request.segments = raster_segments

    raster_vias: list[Any] = []
    for via in vias:
        item = router_core.RasterVia()
        center = via.get("center", (0.0, 0.0))
        item.center = router_core.Point2D(float(center[0]), float(center[1]))
        item.radius_mm = float(via.get("radius_mm", 0.0))
        item.z_start = int(via.get("z_start", 0))
        item.z_end = int(via.get("z_end", 0))
        raster_vias.append(item)
    request.vias = raster_vias

    raster_pads: list[Any] = []
    for pad in pads:
        item = router_core.RasterPad()
        item.center = router_core.Point2D(float(pad.center[0]), float(pad.center[1]))
        item.size_x = float(pad.size[0])
        item.size_y = float(pad.size[1])
        item.rotation_degrees = float(getattr(pad, "rotation_degrees", 0.0))
        item.shape = str(getattr(pad, "shape", ""))
        item.candidate_layers = [
            z
            for z, layer_name in enumerate(board.copper_layers)
            if _pad_has_selector_layer(board, pad, layer_name)
        ]
        raster_pads.append(item)
    request.pads = raster_pads

    if explicit_graph:
        request.explicit_graph_nodes = []
        for node in explicit_graph.get("nodes", []):
            if not isinstance(node, dict):
                continue
            vertex = node.get("vertex")
            if not isinstance(vertex, (tuple, list)) or len(vertex) != 3:
                continue
            item = router_core.RasterGraphNode()
            item.id = int(node.get("id", 0))
            item.vertex = router_core.GridPoint(int(vertex[0]), int(vertex[1]), int(vertex[2]))
            request.explicit_graph_nodes.append(item)

        request.explicit_graph_edges = []
        for edge in explicit_graph.get("edges", []):
            if not isinstance(edge, dict):
                continue
            item = router_core.RasterGraphEdge()
            item.from_id = int(edge.get("from", 0))
            item.to_id = int(edge.get("to", 0))
            request.explicit_graph_edges.append(item)
    return request


def _cpp_raster_candidate_geometry(
    router_core: Any,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None = None,
) -> Any:
    """Convert one candidate's variable geometry for C++ batch analysis."""
    candidate = router_core.RasterCandidateGeometry()
    raster_segments: list[Any] = []
    for segment in segments:
        item = router_core.RasterSegment()
        start = segment.get("start", (0.0, 0.0))
        end = segment.get("end", (0.0, 0.0))
        item.start = router_core.Point2D(float(start[0]), float(start[1]))
        item.end = router_core.Point2D(float(end[0]), float(end[1]))
        item.radius_mm = float(segment.get("radius_mm", 0.0))
        item.z = int(segment.get("z", 0))
        raster_segments.append(item)
    candidate.segments = raster_segments

    raster_vias: list[Any] = []
    for via in vias:
        item = router_core.RasterVia()
        center = via.get("center", (0.0, 0.0))
        item.center = router_core.Point2D(float(center[0]), float(center[1]))
        item.radius_mm = float(via.get("radius_mm", 0.0))
        item.z_start = int(via.get("z_start", 0))
        item.z_end = int(via.get("z_end", 0))
        raster_vias.append(item)
    candidate.vias = raster_vias

    if explicit_graph:
        graph_node_ids: list[int] = []
        graph_node_x: list[int] = []
        graph_node_y: list[int] = []
        graph_node_z: list[int] = []
        for node in explicit_graph.get("nodes", []):
            if not isinstance(node, dict):
                continue
            vertex = node.get("vertex")
            if not isinstance(vertex, (tuple, list)) or len(vertex) != 3:
                continue
            graph_node_ids.append(int(node.get("id", 0)))
            graph_node_x.append(int(vertex[0]))
            graph_node_y.append(int(vertex[1]))
            graph_node_z.append(int(vertex[2]))
        candidate.graph_node_ids = graph_node_ids
        candidate.graph_node_x = graph_node_x
        candidate.graph_node_y = graph_node_y
        candidate.graph_node_z = graph_node_z

        graph_edge_from: list[int] = []
        graph_edge_to: list[int] = []
        for edge in explicit_graph.get("edges", []):
            if not isinstance(edge, dict):
                continue
            graph_edge_from.append(int(edge.get("from", 0)))
            graph_edge_to.append(int(edge.get("to", 0)))
        candidate.graph_edge_from = graph_edge_from
        candidate.graph_edge_to = graph_edge_to
    return candidate


def _candidate_geometry_cache_key(
    selector_board: BoardData,
    source_board: BoardData,
    result: Any,
    selector_net_id: int,
    source_net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None,
) -> tuple[Any, ...]:
    """Build an exact key for selector-visible candidate geometry analysis."""
    return (
        str(getattr(selector_board, "path", "")),
        tuple(selector_board.copper_layers),
        str(getattr(source_board, "path", "")),
        int(selector_net_id),
        int(source_net_id),
        float(getattr(result, "grid_pitch", 0.0)),
        float(getattr(result, "origin_x", 0.0)),
        float(getattr(result, "origin_y", 0.0)),
        int(getattr(result, "nx", 0)),
        int(getattr(result, "ny", 0)),
        int(getattr(result, "nz", 0)),
        tuple(start_anchor) if start_anchor is not None else None,
        tuple(goal_anchor) if goal_anchor is not None else None,
        float(_clearance_for_net(selector_board, selector_net_id)),
        _primitive_geometry_key(segments, vias),
        _explicit_graph_key(explicit_graph),
    )


def _primitive_geometry_key(
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
) -> tuple[Any, ...]:
    """Encode segment/via primitives without approximating their geometry."""
    segment_key = tuple(
        (
            tuple(segment.get("start", (0.0, 0.0))),
            tuple(segment.get("end", (0.0, 0.0))),
            str(segment.get("layer", "")),
            float(segment.get("width_mm", 0.0)),
            int(segment.get("z", 0)),
            float(segment.get("radius_mm", 0.0)),
        )
        for segment in segments
    )
    via_key = tuple(
        (
            tuple(via.get("center", (0.0, 0.0))),
            float(via.get("diameter_mm", 0.0)),
            str(via.get("start_layer", "")),
            str(via.get("end_layer", "")),
            float(via.get("radius_mm", 0.0)),
            int(via.get("z_start", 0)),
            int(via.get("z_end", 0)),
        )
        for via in vias
    )
    return segment_key, via_key


def _explicit_graph_key(explicit_graph: dict[str, Any] | None) -> tuple[Any, ...] | None:
    """Encode explicit graph topology exactly for raster-analysis caching."""
    if not explicit_graph:
        return None
    nodes = tuple(
        (
            int(node.get("id", 0)),
            tuple(node.get("vertex", (0, 0, 0))),
        )
        for node in explicit_graph.get("nodes", [])
        if isinstance(node, dict)
    )
    edges = tuple(
        (
            int(edge.get("from", 0)),
            int(edge.get("to", 0)),
        )
        for edge in explicit_graph.get("edges", [])
        if isinstance(edge, dict)
    )
    return nodes, edges


def _cached_cpp_external_candidate_analysis(
    router_core: Any,
    source_board: BoardData,
    selector_board: BoardData,
    result: Any,
    selector_net_id: int,
    source_net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None,
    geometry_cache: dict[tuple[Any, ...], tuple[list[dict[str, Any]], list[Any], str | None]],
) -> tuple[list[dict[str, Any]], list[Any], str | None, bool, float, float]:
    """Reuse exact raster and pad-coverage results for identical candidates."""
    key = _candidate_geometry_cache_key(
        selector_board=selector_board,
        source_board=source_board,
        result=result,
        selector_net_id=selector_net_id,
        source_net_id=source_net_id,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        segments=segments,
        vias=vias,
        explicit_graph=explicit_graph,
    )
    cached = geometry_cache.get(key)
    if cached is not None:
        raw_vertices, cover_vertices, pad_reason = cached
        return list(raw_vertices), list(cover_vertices), pad_reason, True, 0.0, 0.0

    raster_started_at = perf_counter()
    raw_vertices, cover_vertices = _cpp_rasterize_candidate_geometry(
        router_core,
        source_board,
        result,
        source_net_id,
        start_anchor,
        goal_anchor,
        segments,
        vias,
        explicit_graph,
    )
    raster_elapsed = perf_counter() - raster_started_at

    pad_reason_started_at = perf_counter()
    pad_reason = _cpp_external_candidate_pad_coverage_reason(
        board=selector_board,
        router_core=router_core,
        result=result,
        net_id=selector_net_id,
        segments=segments,
        vias=vias,
        explicit_graph=explicit_graph,
    )
    pad_reason_elapsed = perf_counter() - pad_reason_started_at

    geometry_cache[key] = (list(raw_vertices), list(cover_vertices), pad_reason)
    return raw_vertices, cover_vertices, pad_reason, False, raster_elapsed, pad_reason_elapsed


def _batch_cpp_external_candidate_analysis(
    router_core: Any,
    selector_board: BoardData,
    result: Any,
    entries: list[Any],
    geometry_cache: dict[tuple[Any, ...], tuple[list[dict[str, Any]], list[Any], str | None]],
) -> list[tuple[list[dict[str, Any]], list[Any], str | None, bool, float, float]]:
    """Analyze many external candidates in one C++ call, with safe fallback."""
    if not entries:
        return []
    try:
        anchors = [
            anchor
            for anchor in (
                getattr(entries[0], "start_anchor", None),
                getattr(entries[0], "goal_anchor", None),
            )
            if anchor is not None
        ]
        first_source_board = getattr(entries[0], "source_board", selector_board)
        first_source_net_id = int(getattr(entries[0], "source_net_id", getattr(entries[0], "selector_net_id", 0)))
        first_selector_net_id = int(getattr(entries[0], "selector_net_id", first_source_net_id))
        same_base = all(
            getattr(entry, "source_board", selector_board) is first_source_board
            and int(getattr(entry, "source_net_id", getattr(entry, "selector_net_id", 0))) == first_source_net_id
            and int(getattr(entry, "selector_net_id", first_selector_net_id)) == first_selector_net_id
            and getattr(entry, "start_anchor", None) == getattr(entries[0], "start_anchor", None)
            and getattr(entry, "goal_anchor", None) == getattr(entries[0], "goal_anchor", None)
            for entry in entries
        )

        if same_base and hasattr(router_core, "RasterCandidateBatchRequest"):
            batch_request = router_core.RasterCandidateBatchRequest()
            batch_request.raster_base = _cpp_raster_request(
                router_core,
                first_source_board,
                result,
                [],
                [],
                _all_net_pads(first_source_board, first_source_net_id),
                _clearance_for_net(first_source_board, first_source_net_id),
                anchors,
            )
            batch_request.coverage_base = _cpp_raster_request(
                router_core,
                selector_board,
                result,
                [],
                [],
                _all_net_pads(selector_board, first_selector_net_id),
                0.0,
                [],
            )
            batch_request.candidates = [
                _cpp_raster_candidate_geometry(
                    router_core,
                    list(getattr(entry, "segments", [])),
                    list(getattr(entry, "vias", [])),
                    getattr(entry, "explicit_graph", None),
                )
                for entry in entries
            ]
            batch_started_at = perf_counter()
            batch_results = router_core.analyze_selector_geometry_candidate_batch(batch_request)
        else:
            requests = []
            for entry in entries:
                pair = router_core.RasterAnalysisPairRequest()
                source_board = getattr(entry, "source_board", selector_board)
                source_net_id = int(getattr(entry, "source_net_id", getattr(entry, "selector_net_id", 0)))
                selector_net_id = int(getattr(entry, "selector_net_id", source_net_id))
                pair.raster_request = _cpp_raster_request(
                    router_core,
                    source_board,
                    result,
                    list(getattr(entry, "segments", [])),
                    list(getattr(entry, "vias", [])),
                    _all_net_pads(source_board, source_net_id),
                    _clearance_for_net(source_board, source_net_id),
                    anchors,
                    getattr(entry, "explicit_graph", None),
                )
                pair.coverage_request = _cpp_raster_request(
                    router_core,
                    selector_board,
                    result,
                    list(getattr(entry, "segments", [])),
                    list(getattr(entry, "vias", [])),
                    _all_net_pads(selector_board, selector_net_id),
                    0.0,
                    [],
                    getattr(entry, "explicit_graph", None),
                )
                requests.append(pair)
            batch_started_at = perf_counter()
            batch_results = router_core.analyze_selector_geometry_batch(requests)

        batch_elapsed = perf_counter() - batch_started_at
        elapsed_per_entry = batch_elapsed / max(1, len(entries))
        analyzed = []
        for entry, analysis in zip(entries, batch_results):
            boundary_set = {
                (int(point.x), int(point.y), int(point.z))
                for point in getattr(getattr(analysis, "raster", None), "boundary_vertices", [])
            }
            raw_vertices = _serialize_boundary_vertices_with_anchors(
                getattr(entry, "source_board", selector_board),
                boundary_set,
                getattr(entry, "start_anchor", None),
                getattr(entry, "goal_anchor", None),
            )
            cover_vertices = [
                router_core.GridPoint(int(point.x), int(point.y), int(point.z))
                for point in getattr(getattr(analysis, "raster", None), "occupied_vertices", [])
            ]
            pad_reason = _pad_coverage_reason_from_analysis(getattr(analysis, "coverage", None))
            analyzed.append(
                (
                    raw_vertices,
                    cover_vertices,
                    pad_reason,
                    bool(getattr(analysis, "cache_hit", False)),
                    0.0 if bool(getattr(analysis, "cache_hit", False)) else elapsed_per_entry,
                    0.0,
                )
            )
        return analyzed
    except Exception as exc:
        print(f"selector_batch_geometry_fallback reason={type(exc).__name__}:{exc}", flush=True)
        analyzed = []
        for entry in entries:
            analyzed.append(
                _cached_cpp_external_candidate_analysis(
                    router_core=router_core,
                    source_board=getattr(entry, "source_board", selector_board),
                    selector_board=selector_board,
                    result=result,
                    selector_net_id=int(getattr(entry, "selector_net_id", 0)),
                    source_net_id=int(getattr(entry, "source_net_id", getattr(entry, "selector_net_id", 0))),
                    start_anchor=getattr(entry, "start_anchor", None),
                    goal_anchor=getattr(entry, "goal_anchor", None),
                    segments=list(getattr(entry, "segments", [])),
                    vias=list(getattr(entry, "vias", [])),
                    explicit_graph=getattr(entry, "explicit_graph", None),
                    geometry_cache=geometry_cache,
                )
            )
        return analyzed


def _cpp_rasterize_candidate_geometry(
    router_core: Any,
    board: BoardData,
    result: Any,
    net_id: int,
    start_anchor: tuple[int, int, int] | None,
    goal_anchor: tuple[int, int, int] | None,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], list[Any]]:
    try:
        pads = _all_net_pads(board, net_id)
        anchors = [
            anchor for anchor in (start_anchor, goal_anchor)
            if anchor is not None
        ]
        request = _cpp_raster_request(
            router_core,
            board,
            result,
            segments,
            vias,
            pads,
            _clearance_for_net(board, net_id),
            anchors,
            explicit_graph,
        )
        raster = router_core.rasterize_selector_geometry(request)
        boundary_set = {
            (int(point.x), int(point.y), int(point.z))
            for point in getattr(raster, "boundary_vertices", [])
        }
        raw_vertices = _serialize_boundary_vertices_with_anchors(board, boundary_set, start_anchor, goal_anchor)
        cover_vertices = [
            router_core.GridPoint(int(point.x), int(point.y), int(point.z))
            for point in getattr(raster, "occupied_vertices", [])
        ]
        return raw_vertices, cover_vertices
    except Exception:
        raw_vertices = _build_boundary_payload_from_primitives(
            board=board,
            result=result,
            net_id=net_id,
            start_anchor=start_anchor,
            goal_anchor=goal_anchor,
            segments=segments,
            vias=vias,
        )
        cover_vertices = [
            router_core.GridPoint(x, y, z)
            for (x, y, z) in sorted(_primitive_occupied_vertices(result, segments, vias))
        ]
        return raw_vertices, cover_vertices


def _cpp_external_candidate_pad_coverage_reason(
    router_core: Any,
    board: BoardData,
    result: Any,
    net_id: int,
    segments: list[dict[str, Any]],
    vias: list[dict[str, Any]],
    explicit_graph: dict[str, Any] | None,
) -> str | None:
    try:
        pads = _all_net_pads(board, net_id)
        request = _cpp_raster_request(
            router_core,
            board,
            result,
            segments,
            vias,
            pads,
            0.0,
            [],
            explicit_graph,
        )
        analysis = router_core.analyze_pad_coverage(request)
        return _pad_coverage_reason_from_analysis(analysis)
    except Exception:
        return _external_candidate_pad_coverage_reason(
            board=board,
            result=result,
            net_id=net_id,
            segments=segments,
            vias=vias,
            explicit_graph=explicit_graph,
        )


def _pad_coverage_reason_from_analysis(analysis: Any) -> str | None:
    """Convert C++ pad-coverage analysis into the selector rejection reason."""
    if not bool(getattr(analysis, "has_graph", False)):
        return "empty_graph"
    padless_components = int(getattr(analysis, "padless_components", 0))
    if padless_components > 0:
        return f"padless_components={padless_components}"
    dangling_endpoints = int(getattr(analysis, "dangling_endpoints", 0))
    if dangling_endpoints > 0:
        return f"nonpad_endpoints={dangling_endpoints}"
    return None


def _cpp_pad_boundary_groups_for_net(
    router_core: Any,
    board: BoardData,
    result: Any,
    net_id: int,
) -> list[list[Any]]:
    try:
        pads = _all_net_pads(board, net_id)
        if not pads:
            return []
        request = _cpp_raster_request(
            router_core,
            board,
            result,
            [],
            [],
            pads,
            0.0,
            [],
        )
        groups = router_core.build_pad_boundary_groups(request)
        return [
            [
                router_core.GridPoint(int(point.x), int(point.y), int(point.z))
                for point in group
            ]
            for group in groups
        ]
    except Exception:
        groups: list[list[Any]] = []
        for pad in _all_net_pads(board, net_id):
            vertices = sorted(_pad_boundary_vertices_for_selector(board, result, pad))
            groups.append([router_core.GridPoint(x, y, z) for (x, y, z) in vertices])
        return groups


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
    component_has_pad: list[bool] = [False] * component_index
    vertex_hits_pad: dict[tuple[int, int, int], bool] = {vertex: False for vertex in graph.keys()}
    for footprint in board.footprints:
        for pad in footprint.pads:
            if (pad.net_id or 0) != net_id:
                continue
            matched_components = {
                component_by_vertex[vertex]
                for vertex in graph.keys()
                if _vertex_inside_pad_clearance(board, result, vertex, pad, 0.0)
            }
            for vertex in graph.keys():
                if _vertex_inside_pad_clearance(board, result, vertex, pad, 0.0):
                    vertex_hits_pad[vertex] = True
            for component_id in matched_components:
                if 0 <= component_id < len(component_has_pad):
                    component_has_pad[component_id] = True
            pad_components.extend(sorted(matched_components))

    padless_components = sum(1 for has_pad in component_has_pad if not has_pad)
    if padless_components > 0:
        return f"padless_components={padless_components}"

    dangling_endpoints = 0
    for vertex, neighbors in graph.items():
        if vertex_hits_pad.get(vertex, False):
            continue
        if len(neighbors) <= 1:
            dangling_endpoints += 1
    if dangling_endpoints > 0:
        return f"nonpad_endpoints={dangling_endpoints}"
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
    """Return a layer's local copper stack index inside one board."""
    normalized = _resolve_board_layer_name(board, layer)
    for index, name in enumerate(board.copper_layers):
        if name == normalized:
            return index
    return None


def _map_layer_index_by_stack_order(
    source_board: BoardData,
    target_board: BoardData,
    source_layer: str,
) -> int | None:
    """Map a source-board layer to the target board using only stack order."""
    source_layers = list(source_board.copper_layers)
    target_layers = list(target_board.copper_layers)
    if not source_layers or not target_layers:
        return None
    if len(source_layers) != len(target_layers):
        return None
    source_index = _layer_index_from_name(source_board, source_layer)
    if source_index is None or source_index >= len(target_layers):
        return None
    return source_index


def _resolve_board_layer_name(board: BoardData, layer: str) -> str:
    """Resolve a layer label into this board's local copper stack label."""
    normalized = str(layer)
    if normalized == "Top":
        normalized = "F.Cu"
    elif normalized == "Bottom":
        normalized = "B.Cu"
    aliases = getattr(board, "layer_aliases", None) or {}
    return str(aliases.get(normalized, aliases.get(str(layer), normalized)))


def _load_freerouting_occurrences_by_net(
    board: BoardData | None,
    payload_paths: list[Path] | None = None,
) -> dict[int, list[dict[str, Any]]]:
    if board is None:
        return {}

    candidate_paths = list(payload_paths or [])
    if not candidate_paths:
        for candidate in _guess_freerouting_payload_paths(board.path):
            if candidate.exists():
                candidate_paths.append(candidate)
                break
    if not candidate_paths:
        return {}

    mapping: dict[int, list[dict[str, Any]]] = {}
    for payload_path in candidate_paths:
        try:
            payload = json.loads(Path(payload_path).read_text(encoding="utf-8"))
        except Exception:
            continue
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
            source_backend = _external_payload_backend(payload, payload_path)
            source_profile = _freerouting_payload_profile(payload_path) if source_backend == "freerouting" else "selected"
            parsed = []
            for occurrence in occurrences:
                parsed_occurrence = _parse_freerouting_occurrence_for_selector(occurrence, net_id)
                if parsed_occurrence is None:
                    continue
                parsed_occurrence["source_backend"] = source_backend
                parsed_occurrence["source_profile"] = source_profile
                parsed.append(parsed_occurrence)
            for parsed_occurrence in parsed:
                # The payload parent net can be the net that caused the ripup
                # event, while source_net_id identifies the actual routed
                # component owner. Route each occurrence to its real net bucket.
                effective_net_id = int(parsed_occurrence.get("source_net_id", net_id) or net_id)
                if effective_net_id <= 0:
                    effective_net_id = net_id
                mapping.setdefault(effective_net_id, []).append(parsed_occurrence)
    return mapping


def _external_payload_backend(payload: dict[str, Any], payload_path: Path) -> str:
    """Return the backend label used for selector debug output."""
    backend = str(payload.get("backend", "") or payload.get("generator", "")).lower()
    if "pcbrouter" in backend or "pcb_router" in backend:
        return "pcbrouter"
    name = Path(payload_path).name.lower()
    if "pcbrouter" in name or "pcb_router" in name:
        return "pcbrouter"
    return "freerouting"


def _freerouting_payload_profile(payload_path: Path) -> str:
    """Infer the freerouting profile name from a payload filename."""
    name = Path(payload_path).name.lower()
    if "aggressive" in name:
        return "aggressive"
    if "stable" in name:
        return "stable"
    return "selected"


def _parse_freerouting_occurrence_for_selector(
    occurrence: Any,
    parent_net_id: int,
) -> dict[str, Any] | None:
    """Convert one freerouting payload occurrence into selector primitives."""
    if not isinstance(occurrence, dict):
        return None
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
    if not segs and not vias:
        return None
    try:
        source_net_id = int(occurrence.get("source_net_id", parent_net_id) or parent_net_id)
    except Exception:
        source_net_id = int(parent_net_id)
    return {
        "segments": segs,
        "vias": vias,
        "event_index": int(occurrence.get("event_index", 0)),
        "source_net_id": source_net_id,
        "payload_parent_net_id": int(parent_net_id),
        "graph": occurrence.get("graph"),
    }


def _load_external_payload_paths(candidate_export_path: Path | None) -> list[Path]:
    """Read all backend payload files recorded in a selector manifest."""
    if candidate_export_path is None or not Path(candidate_export_path).exists():
        return []
    try:
        payload = json.loads(Path(candidate_export_path).read_text(encoding="utf-8"))
    except Exception:
        return []
    if not isinstance(payload, dict):
        return []
    raw_paths: list[Any] = []
    for key in ("freerouting_payload_paths", "pcbrouter_payload_paths", "external_payload_paths"):
        # Backend payloads share the same occurrence geometry schema; source
        # metadata keeps freerouting and PcbRouter debug output separate.
        value = payload.get(key, [])
        if isinstance(value, list):
            raw_paths.extend(value)
    paths: list[Path] = []
    for raw_path in raw_paths:
        path = Path(str(raw_path))
        if path.exists():
            paths.append(path)
    return paths


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
        if _pad_has_selector_layer(board, pad, layer_name):
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
    final_boards: list[BoardData] | None = None,
    original_candidate_boards: list[BoardData] | None = None,
    freerouting_payload_paths: list[Path] | None = None,
    pcbrouter_payload_paths: list[Path] | None = None,
    external_payload_paths: list[Path] | None = None,
    left_top_reference_board: BoardData | None = None,
    backend_label: str = "external",
) -> RerouteOutcome:
    if not ripped_net_ids:
        return RerouteOutcome(False, f"No ripped nets were provided for {backend_label} selector input.")

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
        candidate_final_boards = list(final_boards or [])
        if final_board is not None and all(Path(getattr(item, "path", "")) != Path(final_board.path) for item in candidate_final_boards):
            candidate_final_boards.append(final_board)
        final_candidates = [
            SimpleNamespace(
                board=candidate_board,
                net_id=int(_matching_net_id_by_name(candidate_board, net_name)),
                source=str(Path(candidate_board.path).stem),
            )
            for candidate_board in candidate_final_boards
        ]
        # Keep translated routed input geometry as candidate sources only. They
        # must not be stored in `board.tracks`, because selector grid bounds are
        # computed from the selector board itself.
        original_candidates = [
            SimpleNamespace(
                board=candidate_board,
                net_id=int(_matching_net_id_by_name(candidate_board, net_name)),
                source=str(Path(candidate_board.path).stem),
            )
            for candidate_board in (original_candidate_boards or [])
        ]
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
                final_candidate_net_id=int(final_candidates[0].net_id if final_candidates else 0),
                final_candidate_profiles=final_candidates,
                original_candidate_profiles=original_candidates,
                freerouting_payload_paths=[str(path) for path in (freerouting_payload_paths or [])],
                pcbrouter_payload_paths=[str(path) for path in (pcbrouter_payload_paths or [])],
                external_payload_paths=[str(path) for path in (external_payload_paths or [])],
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
            f"No usable net anchors were found for {backend_label} external-only reroute.",
            None,
            None,
            None,
            True,
        )

    export_path = _write_selector_stub_manifest(
        board=board,
        net_ids=[int(getattr(item, "net_id", 0)) for item in results],
        grid_steps_per_mm=grid_steps_per_mm,
        freerouting_payload_paths=freerouting_payload_paths or [],
        pcbrouter_payload_paths=pcbrouter_payload_paths or [],
        external_payload_paths=external_payload_paths or [],
        backend_label=backend_label,
    )
    message = (
        f"{backend_label} external-only candidates prepared for {len(results)} nets."
        + (f" Skipped nets: {skipped}." if skipped else "")
    )
    return RerouteOutcome(
        True,
        message,
        results,
        None,
        export_path,
        True,
        left_top_reference_board,
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
    freerouting_payload_paths: list[Path] | None = None,
    pcbrouter_payload_paths: list[Path] | None = None,
    external_payload_paths: list[Path] | None = None,
    backend_label: str = "external",
) -> Path:
    root = Path(__file__).resolve().parents[1]
    export_dir = root / "out" / "candidate_path_exports"
    export_dir.mkdir(parents=True, exist_ok=True)
    board_stem = board.path.stem or "board"
    net_label = "-".join(str(net_id) for net_id in net_ids) or "none"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    safe_backend = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in backend_label.lower())
    export_path = export_dir / f"{board_stem}__{safe_backend}_external_only__nets_{net_label}__{timestamp}.json"

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
        "freerouting_payload_paths": [str(path) for path in (freerouting_payload_paths or [])],
        "pcbrouter_payload_paths": [str(path) for path in (pcbrouter_payload_paths or [])],
        "external_payload_paths": [str(path) for path in (external_payload_paths or [])],
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
            if not _pad_has_selector_layer(board, pad, layer_name):
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
    if not _pad_has_selector_layer(board, pad, layer_name):
        return False
    x_mm, y_mm = _grid_vertex_to_mm(result, vertex)
    return _point_inside_expanded_pad(x_mm, y_mm, pad, clearance)


def _pad_has_selector_layer(board: BoardData, pad: Any, layer_name: str) -> bool:
    """Check pad membership by comparing local copper stack indices."""
    pad_layers = tuple(getattr(pad, "layers", ()) or ())
    if "*.Cu" in pad_layers:
        return True
    target_index = _layer_index_from_name(board, layer_name)
    if target_index is None:
        return False
    return any(_layer_index_from_name(board, str(layer)) == target_index for layer in pad_layers)


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
