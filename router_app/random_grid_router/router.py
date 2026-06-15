from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
from math import ceil, hypot
import os
from pathlib import Path
import random
import sys
from typing import Any

from router_app.kicad_parser import BoardData
from router_app.sj_bridge import (
    bridge_point_allowed_for_net,
    detect_solder_jumper_bridges,
    distance_point_to_segment,
)

from .config import RouterConfig
from .geometry import (
    DIRS,
    PadInfo,
    collect_pad_infos,
    edge_or_pad_bounds,
    point_in_expanded_bbox,
)
from .payload import (
    RouteCandidate,
    candidate_key,
    dedupe_segments,
    dedupe_vias,
    merge_grid_paths,
)

_ROUTER_CORE = None
_ROUTER_CORE_IMPORT_ATTEMPTED = False
_DLL_DIR_HANDLES: list[object] = []


class RandomGridRouter:
    """Generate angle-safe grid-route candidates under fixed-geometry keepouts."""

    def __init__(self, board: BoardData, config: RouterConfig):
        self.board = board
        self.config = config
        self.layers = board.copper_layers or ["F.Cu", "B.Cu"]
        if len(self.layers) == 1:
            self.layers = [self.layers[0]]
        self.layer_to_z = {layer: index for index, layer in enumerate(self.layers)}
        self.pad_infos = collect_pad_infos(board)
        self.sj_bridges = detect_solder_jumper_bridges(board)
        self.edge_bounds = edge_or_pad_bounds(board, self.pad_infos)
        self.origin_x, self.origin_y, self.nx, self.ny = self._grid_bounds()
        self.xy_count = self.nx * self.ny
        self.vertex_count = self.xy_count * len(self.layers)
        self.net_pads: dict[int, list[PadInfo]] = {}
        for pad_info in self.pad_infos:
            if pad_info.pad.net_id is not None and int(pad_info.pad.net_id) > 0:
                self.net_pads.setdefault(int(pad_info.pad.net_id), []).append(pad_info)
        self._build_keepout_cache()

    def generate_payload(self) -> dict[str, Any]:
        """Route every net with at least two pads and return a freerouting-like payload."""
        net_ids = [net_id for net_id in sorted(self.net_pads) if len(self.net_pads[net_id]) >= 2]
        if self.config.parallel_workers > 1 and len(net_ids) > 1:
            nets, infeasible, candidate_total = self._generate_payload_parallel(net_ids)
        else:
            nets, infeasible, candidate_total = self._generate_payload_for_net_ids(net_ids)

        return {
            "schema_version": 1,
            "backend": "random_grid_router",
            "generator": "random_grid_router",
            "board_unit": "mm",
            "source_unit": "mm",
            "grid_pitch_mm": self.config.pitch_mm,
            "net_count": len(nets),
            "candidate_count": candidate_total,
            "infeasible_net_count": len(infeasible),
            "infeasible_nets": infeasible,
            "nets": nets,
        }

    def _generate_payload_parallel(self, net_ids: list[int]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
        """Generate independent net candidates in worker processes."""
        worker_count = max(1, min(int(self.config.parallel_workers), len(net_ids), os.cpu_count() or 1))
        chunks = [net_ids[index::worker_count] for index in range(worker_count)]
        chunks = [chunk for chunk in chunks if chunk]
        nets: list[dict[str, Any]] = []
        infeasible: list[dict[str, Any]] = []
        candidate_total = 0
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            futures = [
                executor.submit(_route_net_batch_worker, self.board, self.config, chunk)
                for chunk in chunks
            ]
            for future in as_completed(futures):
                chunk_nets, chunk_infeasible, chunk_total = future.result()
                nets.extend(chunk_nets)
                infeasible.extend(chunk_infeasible)
                candidate_total += int(chunk_total)
        nets.sort(key=lambda item: int(item.get("net_id", 0)))
        infeasible.sort(key=lambda item: int(item.get("net_id", 0)))
        return nets, infeasible, candidate_total

    def _generate_payload_for_net_ids(self, net_ids: list[int]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
        """Generate candidates for a fixed subset of net ids."""
        nets: list[dict[str, Any]] = []
        infeasible: list[dict[str, Any]] = []
        candidate_total = 0
        for net_id in sorted(net_ids):
            pads = self.net_pads.get(net_id, [])
            if len(pads) < 2:
                continue
            rng = random.Random(self.config.seed + int(net_id) * 1000003)
            net_name = self.board.nets.get(net_id, pads[0].pad.net_name)
            blocked_trace, blocked_via = self._blocked_sets_for_net(net_id)
            deterministic = self._route_tree_candidate(
                net_id=net_id,
                pads=pads,
                blocked_trace=blocked_trace,
                blocked_via=blocked_via,
                rng=rng,
                randomize=False,
                used_penalty=set(),
            )
            if deterministic is None:
                infeasible.append({"net_id": net_id, "net_name": net_name, "reason": "deterministic_no_legal_tree"})
                print(f"random_grid_router_infeasible net={net_id} net_name={net_name!r} reason=deterministic_no_legal_tree", flush=True)
                continue

            occurrences = [self._occurrence_dict(net_id, deterministic, 1, "deterministic")]
            used_penalty = set(deterministic.grid_path)
            seen_keys = {candidate_key(deterministic)}
            attempts = 0
            while len(occurrences) < 1 + self.config.random_per_net and attempts < self.config.max_attempts_per_net:
                attempts += 1
                candidate = self._route_tree_candidate(
                    net_id=net_id,
                    pads=pads,
                    blocked_trace=blocked_trace,
                    blocked_via=blocked_via,
                    rng=rng,
                    randomize=True,
                    used_penalty=used_penalty,
                )
                if candidate is None:
                    continue
                key = candidate_key(candidate)
                if key in seen_keys:
                    continue
                seen_keys.add(key)
                used_penalty.update(candidate.grid_path)
                occurrences.append(self._occurrence_dict(net_id, candidate, len(occurrences) + 1, "randomized"))

            candidate_total += len(occurrences)
            nets.append(
                {
                    "net_id": net_id,
                    "net_name": net_name,
                    "truncated": False,
                    "segments": [],
                    "vias": [],
                    "occurrences": occurrences,
                }
            )
        return nets, infeasible, candidate_total

    def _route_tree_candidate(
        self,
        net_id: int,
        pads: list[PadInfo],
        blocked_trace: bytearray,
        blocked_via: bytearray,
        rng: random.Random,
        randomize: bool,
        used_penalty: set[int],
    ) -> RouteCandidate | None:
        """Grow a route tree until all pads of one net are connected."""
        ordered_pads = list(pads)
        if randomize:
            rng.shuffle(ordered_pads)
        connected = [ordered_pads[0]]
        remaining = ordered_pads[1:]
        tree_vertices = set(self._terminal_vertices(ordered_pads[0]))
        all_paths: list[list[int]] = []

        while remaining:
            best: tuple[float, int, list[int]] | None = None
            for candidate_targets in self._target_candidate_batches(tree_vertices, remaining, rng, randomize):
                routed = self._search_path_to_any_pad(
                    sources=tree_vertices,
                    target_pads=candidate_targets,
                    blocked_trace=blocked_trace,
                    blocked_via=blocked_via,
                    rng=rng,
                    randomize=randomize,
                    used_penalty=used_penalty,
                )
                if routed is None:
                    continue
                target_index, path = routed
                score = self._path_score(path, randomize=randomize, rng=rng)
                if best is None or score < best[0]:
                    best = (score, target_index, path)
                break
            if best is None:
                return None
            _, target_index, path = best
            all_paths.append(path)
            for vertex in path:
                tree_vertices.add(vertex)
            connected.append(remaining.pop(target_index))

        merged_path = merge_grid_paths(all_paths)
        segments, vias = self._grid_paths_to_primitives(all_paths, net_id)
        if not segments and not vias:
            return None
        return RouteCandidate(segments=segments, vias=vias, grid_path=merged_path)

    def _target_candidate_batches(
        self,
        tree_vertices: set[int],
        remaining: list[PadInfo],
        rng: random.Random,
        randomize: bool,
    ) -> list[list[tuple[int, PadInfo]]]:
        """Return increasingly broad target batches, nearest pads first."""
        candidates = list(enumerate(remaining))
        scored = [
            (self._target_distance_score(tree_vertices, pad, rng, randomize), target_index, pad)
            for target_index, pad in candidates
        ]
        scored.sort(key=lambda item: item[0])
        ordered = [(target_index, pad) for _, target_index, pad in scored]
        limit = max(1, int(self.config.target_pad_limit))
        sizes = [min(limit, len(ordered))]
        while sizes[-1] < len(ordered):
            sizes.append(min(len(ordered), sizes[-1] * 2))
        batches: list[list[tuple[int, PadInfo]]] = []
        seen_sizes: set[int] = set()
        for size in sizes:
            if size in seen_sizes:
                continue
            seen_sizes.add(size)
            batch = list(ordered[:size])
            if randomize:
                rng.shuffle(batch)
            batches.append(batch)
        return batches

    def _target_distance_score(
        self,
        tree_vertices: set[int],
        pad_info: PadInfo,
        rng: random.Random,
        randomize: bool,
    ) -> float:
        """Estimate how close one target pad is to the current route tree."""
        px, py = pad_info.pad.center
        best = float("inf")
        for vertex in tree_vertices:
            x, y, _ = self._unpack_vertex(vertex)
            sx, sy = self._grid_to_xy(x, y)
            best = min(best, hypot(px - sx, py - sy))
        if randomize:
            best += rng.random() * self.config.pitch_mm * 8.0
        return best

    def _search_path_to_any_pad(
        self,
        sources: set[int],
        target_pads: list[tuple[int, PadInfo]],
        blocked_trace: bytearray,
        blocked_via: bytearray,
        rng: random.Random,
        randomize: bool,
        used_penalty: set[int],
    ) -> tuple[int, list[int]] | None:
        """Run multi-target A* from the same-net tree to the first reachable pad."""
        target_by_vertex: dict[int, int] = {}
        for target_index, target_pad in target_pads:
            for vertex in self._terminal_vertices(target_pad):
                target_by_vertex.setdefault(vertex, target_index)
        if not target_by_vertex:
            return None
        target_xy = [self._grid_to_xy(*self._unpack_vertex_xy(vertex)) for vertex in target_by_vertex]
        for source_batch in self._source_batches(sources, target_xy, rng, randomize):
            for bounds in self._search_windows(source_batch, set(target_by_vertex)):
                routed = self._search_path_to_targets_with_bounds(
                    sources=source_batch,
                    target_by_vertex=target_by_vertex,
                    target_xy=target_xy,
                    blocked_trace=blocked_trace,
                    blocked_via=blocked_via,
                    rng=rng,
                    randomize=randomize,
                    used_penalty=used_penalty,
                    bounds=bounds,
                )
                if routed is not None:
                    return routed
        return None

    def _source_batches(
        self,
        sources: set[int],
        target_xy: list[tuple[float, float]],
        rng: random.Random,
        randomize: bool,
    ) -> list[set[int]]:
        """Return nearest source frontiers first, with a full-tree fallback."""
        if not sources:
            return []
        limit = max(1, int(self.config.source_frontier_limit))
        scored = []
        for vertex in sources:
            score = self._heuristic(vertex, target_xy)
            if randomize:
                score += rng.random() * self.config.pitch_mm * 4.0
            scored.append((score, vertex))
        scored.sort(key=lambda item: item[0])
        ordered = [vertex for _, vertex in scored]
        sizes = [min(limit, len(ordered))]
        while sizes[-1] < len(ordered):
            sizes.append(min(len(ordered), sizes[-1] * 4))
        batches: list[set[int]] = []
        seen_sizes: set[int] = set()
        for size in sizes:
            if size in seen_sizes:
                continue
            seen_sizes.add(size)
            batches.append(set(ordered[:size]))
        return batches

    def _search_path_to_targets_with_bounds(
        self,
        sources: set[int],
        target_by_vertex: dict[int, int],
        target_xy: list[tuple[float, float]],
        blocked_trace: bytearray,
        blocked_via: bytearray,
        rng: random.Random,
        randomize: bool,
        used_penalty: set[int],
        bounds: tuple[int, int, int, int] | None,
    ) -> tuple[int, list[int]] | None:
        """Run the required C++ A* helper inside one optional bounding window."""
        return self._search_path_to_targets_with_native(
            sources=sources,
            target_by_vertex=target_by_vertex,
            blocked_trace=blocked_trace,
            blocked_via=blocked_via,
            randomize=randomize,
            used_penalty=used_penalty,
            bounds=bounds,
        )

    def _search_path_to_targets_with_native(
        self,
        sources: set[int],
        target_by_vertex: dict[int, int],
        blocked_trace: bytearray,
        blocked_via: bytearray,
        randomize: bool,
        used_penalty: set[int],
        bounds: tuple[int, int, int, int] | None,
    ) -> tuple[int, list[int]] | None:
        """Use the required C++ A* helper and fail loudly if it is unavailable."""
        router_core = _import_router_core_optional()
        if router_core is None or not hasattr(router_core, "random_grid_astar"):
            raise RuntimeError("router_core.random_grid_astar is required for random-grid routing. Rebuild router_core.")
        target_vertices = list(target_by_vertex)
        target_indices = [int(target_by_vertex[vertex]) for vertex in target_vertices]
        target_grid_xy = [self._unpack_vertex_xy(vertex) for vertex in target_vertices]
        bounds_list = list(bounds) if bounds is not None else []
        max_expanded = max(20000, self.nx * self.ny * max(1, len(self.layers)) // 2)
        try:
            target_index, path = router_core.random_grid_astar(
                self.nx,
                self.ny,
                len(self.layers),
                float(self.config.pitch_mm),
                list(sources),
                target_vertices,
                target_indices,
                target_grid_xy,
                blocked_trace,
                blocked_via,
                list(used_penalty) if randomize else [],
                bounds_list,
                bool(randomize),
                int(self.config.seed + len(sources) * 131 + len(target_vertices) * 17),
                int(max_expanded),
            )
        except Exception as exc:
            raise RuntimeError(f"router_core.random_grid_astar failed: {exc}") from exc
        if not path:
            return None
        return int(target_index), [int(vertex) for vertex in path]

    def _search_windows(
        self,
        sources: set[int],
        targets: set[int],
    ) -> list[tuple[int, int, int, int] | None]:
        """Return expanding local windows, then a full-board fallback."""
        if not sources or not targets:
            return [None]
        source_xy = [self._unpack_vertex_xy(vertex) for vertex in sources]
        target_xy = [self._unpack_vertex_xy(vertex) for vertex in targets]
        source_xs = [point[0] for point in source_xy]
        source_ys = [point[1] for point in source_xy]
        target_xs = [point[0] for point in target_xy]
        target_ys = [point[1] for point in target_xy]
        base = (
            min(min(source_xs), min(target_xs)),
            min(min(source_ys), min(target_ys)),
            max(max(source_xs), max(target_xs)),
            max(max(source_ys), max(target_ys)),
        )
        windows: list[tuple[int, int, int, int] | None] = []
        seen: set[tuple[int, int, int, int]] = set()
        window_mm = max(0.0, float(self.config.search_window_initial_mm))
        max_window_mm = max(window_mm, float(self.config.search_window_max_mm))
        growth = max(1.01, float(self.config.search_window_growth))
        while window_mm <= max_window_mm + 1e-9:
            margin = int(ceil(window_mm / self.config.pitch_mm))
            bounds = (
                max(0, base[0] - margin),
                max(0, base[1] - margin),
                min(self.nx - 1, base[2] + margin),
                min(self.ny - 1, base[3] + margin),
            )
            if bounds not in seen:
                seen.add(bounds)
                windows.append(bounds)
            if bounds == (0, 0, self.nx - 1, self.ny - 1):
                break
            window_mm *= growth
        windows.append(None)
        return windows

    def _in_search_bounds(self, x: int, y: int, bounds: tuple[int, int, int, int] | None) -> bool:
        """Return whether a grid point is inside the optional A* search window."""
        if bounds is None:
            return True
        min_x, min_y, max_x, max_y = bounds
        return min_x <= x <= max_x and min_y <= y <= max_y

    def _blocked_sets_for_net(self, net_id: int) -> tuple[bytearray, bytearray]:
        """Combine cached pad, board-edge, and SJ bridge keepouts for one routed net."""
        blocked_trace = bytearray(self._edge_trace_keepout)
        blocked_via = bytearray(self._edge_via_keepout)
        for pad_index, pad_net in enumerate(self._pad_net_ids):
            if pad_net == net_id:
                # Same-net pads are legal via attach targets, so do not apply
                # the pad via keepout for the net currently being routed.
                continue
            self._mark_ids(blocked_trace, self._pad_trace_keepouts[pad_index])
            self._mark_ids(blocked_via, self._pad_via_keepouts[pad_index])
        self._mark_sj_bridge_keepouts(blocked_trace, blocked_via, net_id)
        # Allow same-net terminal vertices to remain reachable even when a pad
        # sits inside the rectangular edge keepout approximation. Same-net via
        # attach is handled above by skipping that pad's via keepout.
        for pad_info in self.net_pads.get(net_id, []):
            for vertex in self._terminal_vertices(pad_info):
                blocked_trace[vertex] = 0
        return blocked_trace, blocked_via

    def _build_keepout_cache(self) -> None:
        """Rasterize reusable pad and edge keepouts once per router run."""
        trace_radius = self.config.trace_width_mm * 0.5
        via_radius = self.config.via_diameter_mm * 0.5
        self._edge_trace_keepout, self._edge_via_keepout = self._rasterize_edge_keepouts(trace_radius, via_radius)
        self._pad_net_ids: list[int] = []
        self._pad_trace_keepouts: list[list[int]] = []
        self._pad_via_keepouts: list[list[int]] = []
        for pad_info in self.pad_infos:
            pad_trace = bytearray(self.vertex_count)
            pad_via = bytearray(self.xy_count)
            self._block_pad(pad_trace, pad_via, pad_info, trace_radius, via_radius)
            self._pad_net_ids.append(int(pad_info.pad.net_id or 0))
            self._pad_trace_keepouts.append([index for index, blocked in enumerate(pad_trace) if blocked])
            self._pad_via_keepouts.append([index for index, blocked in enumerate(pad_via) if blocked])

    def _mark_sj_bridge_keepouts(self, blocked_trace: bytearray, blocked_via: bytearray, net_id: int) -> None:
        """Block normally-closed SJ bridge copper outside legal endpoint pad contact regions."""
        if not self.sj_bridges:
            return
        trace_radius = self.config.trace_width_mm * 0.5
        via_radius = self.config.via_diameter_mm * 0.5
        for bridge in self.sj_bridges:
            trace_expand = bridge.width * 0.5 + trace_radius + self.config.clearance_mm
            via_expand = bridge.width * 0.5 + via_radius + self.config.clearance_mm
            min_x, min_y, max_x, max_y = bridge.bbox
            gx0, gy0 = self._xy_to_grid(min_x - max(trace_expand, via_expand), min_y - max(trace_expand, via_expand))
            gx1, gy1 = self._xy_to_grid(max_x + max(trace_expand, via_expand), max_y + max(trace_expand, via_expand))
            trace_layers = self._bridge_z_layers(bridge.layer)
            for x in range(max(0, gx0), min(self.nx - 1, gx1) + 1):
                for y in range(max(0, gy0), min(self.ny - 1, gy1) + 1):
                    px, py = self._grid_to_xy(x, y)
                    if distance_point_to_segment(px, py, bridge.start, bridge.end) <= trace_expand + 1e-9:
                        # Same-net SJ routes may enter the bridge only at the actual endpoint pad copper.
                        if not bridge_point_allowed_for_net(bridge, net_id, px, py, tolerance=0.0):
                            for z in trace_layers:
                                blocked_trace[self._pack_vertex(x, y, z)] = 1
                    if distance_point_to_segment(px, py, bridge.start, bridge.end) <= via_expand + 1e-9:
                        # Vias are through-copper objects, so the bridge body blocks via centers on all layers.
                        if not bridge_point_allowed_for_net(bridge, net_id, px, py, tolerance=0.0):
                            blocked_via[self._pack_xy(x, y)] = 1

    def _rasterize_edge_keepouts(
        self,
        trace_radius: float,
        via_radius: float,
    ) -> tuple[bytearray, bytearray]:
        """Rasterize the rectangular board-edge keepout once for all nets."""
        blocked_trace = bytearray(self.vertex_count)
        blocked_via = bytearray(self.xy_count)
        min_x, min_y, max_x, max_y = self.edge_bounds
        for x in range(self.nx):
            for y in range(self.ny):
                px, py = self._grid_to_xy(x, y)
                edge_bad = (
                    px < min_x + self.config.edge_clearance_mm + trace_radius
                    or px > max_x - self.config.edge_clearance_mm - trace_radius
                    or py < min_y + self.config.edge_clearance_mm + trace_radius
                    or py > max_y - self.config.edge_clearance_mm - trace_radius
                )
                via_edge_bad = (
                    px < min_x + self.config.edge_clearance_mm + via_radius
                    or px > max_x - self.config.edge_clearance_mm - via_radius
                    or py < min_y + self.config.edge_clearance_mm + via_radius
                    or py > max_y - self.config.edge_clearance_mm - via_radius
                )
                if edge_bad:
                    for z in range(len(self.layers)):
                        blocked_trace[self._pack_vertex(x, y, z)] = 1
                if via_edge_bad:
                    blocked_via[self._pack_xy(x, y)] = 1
        return blocked_trace, blocked_via

    def _block_pad(
        self,
        blocked_trace: bytearray,
        blocked_via: bytearray,
        pad_info: PadInfo,
        trace_radius: float,
        via_radius: float,
    ) -> None:
        """Block grid points whose centers fall inside a pad clearance envelope."""
        min_x, min_y, max_x, max_y = pad_info.bbox
        trace_expand = self.config.clearance_mm + trace_radius
        tx0, ty0 = self._xy_to_grid(min_x - trace_expand, min_y - trace_expand)
        tx1, ty1 = self._xy_to_grid(max_x + trace_expand, max_y + trace_expand)
        pad_layers = self._pad_z_layers(pad_info)
        for x in range(max(0, tx0), min(self.nx - 1, tx1) + 1):
            for y in range(max(0, ty0), min(self.ny - 1, ty1) + 1):
                px, py = self._grid_to_xy(x, y)
                if point_in_expanded_bbox(px, py, pad_info.bbox, trace_expand):
                    for z in pad_layers:
                        blocked_trace[self._pack_vertex(x, y, z)] = 1
        self._block_pad_via(blocked_via, pad_info, via_radius)

    def _block_pad_via(
        self,
        blocked_via: bytearray,
        pad_info: PadInfo,
        via_radius: float,
    ) -> None:
        """Cache via-center keepouts for one pad; same-net pads are skipped per route."""
        min_x, min_y, max_x, max_y = pad_info.bbox
        via_expand = self.config.clearance_mm + via_radius
        vx0, vy0 = self._xy_to_grid(min_x - via_expand, min_y - via_expand)
        vx1, vy1 = self._xy_to_grid(max_x + via_expand, max_y + via_expand)
        for x in range(max(0, vx0), min(self.nx - 1, vx1) + 1):
            for y in range(max(0, vy0), min(self.ny - 1, vy1) + 1):
                px, py = self._grid_to_xy(x, y)
                if point_in_expanded_bbox(px, py, pad_info.bbox, via_expand):
                    blocked_via[self._pack_xy(x, y)] = 1

    def _terminal_vertices(self, pad_info: PadInfo) -> list[int]:
        """Return legal terminal grid vertices near one pad center."""
        cx, cy = pad_info.pad.center
        gx, gy = self._xy_to_grid(cx, cy)
        vertices: list[int] = []
        for z in self._pad_z_layers(pad_info):
            for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                x = gx + dx
                y = gy + dy
                if self._in_grid(x, y):
                    vertices.append(self._pack_vertex(x, y, z))
        return vertices

    def _pad_z_layers(self, pad_info: PadInfo) -> list[int]:
        """Map pad copper layers into local grid z indices."""
        layers = pad_info.layers
        if any(layer in {"*.Cu", "*.Mask"} for layer in layers):
            return list(range(len(self.layers)))
        mapped = [self.layer_to_z[layer] for layer in layers if layer in self.layer_to_z]
        if mapped:
            return sorted(set(mapped))
        if pad_info.pad.layers:
            return [0]
        return list(range(len(self.layers)))

    def _bridge_z_layers(self, layer: str) -> list[int]:
        """Map a footprint copper bridge layer into local grid z indices."""
        mapped = self.layer_to_z.get(layer)
        if mapped is not None:
            return [mapped]
        # Legacy boards sometimes spell copper layers as Top/Bottom before
        # normalization, so keep the fallback narrow before using every layer.
        aliases = {"Top": "F.Cu", "Bottom": "B.Cu"}
        alias = aliases.get(layer)
        if alias in self.layer_to_z:
            return [self.layer_to_z[alias]]
        return list(range(len(self.layers)))

    def _grid_paths_to_primitives(self, paths: list[list[int]], net_id: int) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Convert grid paths into KiCad-style segments and vias."""
        segments: list[dict[str, Any]] = []
        vias: list[dict[str, Any]] = []
        for path in paths:
            if len(path) < 2:
                continue
            run_start = path[0]
            prev = path[0]
            prev_dir: tuple[int, int, int] | None = None
            for current in path[1:]:
                px, py, pz = self._unpack_vertex(prev)
                cx, cy, cz = self._unpack_vertex(current)
                dx = cx - px
                dy = cy - py
                dz = cz - pz
                if dz != 0:
                    if run_start != prev:
                        segments.append(self._segment_dict(run_start, prev))
                    vias.append(self._via_dict(prev, current))
                    run_start = current
                    prev_dir = None
                else:
                    direction = (dx, dy, dz)
                    if prev_dir is not None and direction != prev_dir:
                        segments.append(self._segment_dict(run_start, prev))
                        run_start = prev
                    prev_dir = direction
                prev = current
            if run_start != prev:
                segments.append(self._segment_dict(run_start, prev))
        return dedupe_segments(segments), dedupe_vias(vias)

    def _segment_dict(self, start: int, end: int) -> dict[str, Any]:
        """Serialize one segment in payload source coordinates."""
        start_x, start_y, start_z = self._unpack_vertex(start)
        end_x, end_y, _ = self._unpack_vertex(end)
        sx, sy = self._grid_to_xy(start_x, start_y)
        ex, ey = self._grid_to_xy(end_x, end_y)
        return {
            "layer": self.layers[start_z],
            "start": {"x_mm": round(sx, 6), "y_mm": round(-sy, 6)},
            "end": {"x_mm": round(ex, 6), "y_mm": round(-ey, 6)},
            "width_mm": self.config.trace_width_mm,
        }

    def _via_dict(self, start: int, end: int) -> dict[str, Any]:
        """Serialize one via in payload source coordinates."""
        start_x, start_y, start_z = self._unpack_vertex(start)
        _, _, end_z = self._unpack_vertex(end)
        cx, cy = self._grid_to_xy(start_x, start_y)
        z0 = min(start_z, end_z)
        z1 = max(start_z, end_z)
        return {
            "center": {"x_mm": round(cx, 6), "y_mm": round(-cy, 6)},
            "diameter_mm": self.config.via_diameter_mm,
            "drill_mm": self.config.via_drill_mm,
            "via_type": "through",
            "start_layer": self.layers[z0],
            "end_layer": self.layers[z1],
        }

    def _occurrence_dict(self, net_id: int, candidate: RouteCandidate, event_index: int, mode: str) -> dict[str, Any]:
        """Serialize one candidate occurrence for selector ingestion."""
        return {
            "event_index": event_index,
            "source_net_id": net_id,
            "source": f"random_grid_router:{mode}",
            "truncated": False,
            "segments": candidate.segments,
            "vias": candidate.vias,
            "graph": {"mode": mode, "grid_pitch_mm": self.config.pitch_mm},
        }

    def _path_score(self, path: list[int], randomize: bool, rng: random.Random) -> float:
        """Score a path for deterministic target ordering."""
        length = 0.0
        vias = 0
        bends = 0
        prev_dir: tuple[int, int, int] | None = None
        for first, second in zip(path, path[1:]):
            fx, fy, fz = self._unpack_vertex(first)
            sx, sy, sz = self._unpack_vertex(second)
            dx = sx - fx
            dy = sy - fy
            dz = sz - fz
            if dz:
                vias += 1
                length += 8.0
                prev_dir = None
                continue
            direction = (dx, dy, dz)
            if prev_dir is not None and direction != prev_dir:
                bends += 1
            prev_dir = direction
            length += hypot(dx, dy) * self.config.pitch_mm
        jitter = rng.random() * 0.5 if randomize else 0.0
        return length + vias * 8.0 + bends * 0.3 + jitter

    def _heuristic(self, vertex: int, target_xy: list[tuple[float, float]]) -> float:
        """Return an admissible-ish Euclidean distance to the nearest target point."""
        vx, vy, _ = self._unpack_vertex(vertex)
        x, y = self._grid_to_xy(vx, vy)
        return min(hypot(x - tx, y - ty) for tx, ty in target_xy)

    def _grid_bounds(self) -> tuple[float, float, int, int]:
        """Build a rectangular grid around the board edge box."""
        min_x, min_y, max_x, max_y = self.edge_bounds
        margin = max(1.0, self.config.edge_clearance_mm + self.config.via_diameter_mm)
        origin_x = min_x - margin
        origin_y = min_y - margin
        nx = int(ceil((max_x - min_x + 2.0 * margin) / self.config.pitch_mm)) + 1
        ny = int(ceil((max_y - min_y + 2.0 * margin) / self.config.pitch_mm)) + 1
        return origin_x, origin_y, max(2, nx), max(2, ny)

    def _xy_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert board coordinates into integer grid coordinates."""
        return (
            int(round((float(x) - self.origin_x) / self.config.pitch_mm)),
            int(round((float(y) - self.origin_y) / self.config.pitch_mm)),
        )

    def _grid_to_xy(self, x: int, y: int) -> tuple[float, float]:
        """Convert integer grid coordinates into board coordinates."""
        return (
            self.origin_x + int(x) * self.config.pitch_mm,
            self.origin_y + int(y) * self.config.pitch_mm,
        )

    def _in_grid(self, x: int, y: int) -> bool:
        """Return whether a grid coordinate is inside the allocated grid."""
        return 0 <= x < self.nx and 0 <= y < self.ny

    def _pack_xy(self, x: int, y: int) -> int:
        """Pack one 2D grid coordinate into a dense integer id."""
        return int(y) * self.nx + int(x)

    def _pack_vertex(self, x: int, y: int, z: int) -> int:
        """Pack one 3D grid coordinate into a dense integer id."""
        return int(z) * self.xy_count + self._pack_xy(x, y)

    def _unpack_vertex(self, vertex: int) -> tuple[int, int, int]:
        """Unpack a dense vertex id into x, y, z grid coordinates."""
        vertex = int(vertex)
        z = vertex // self.xy_count
        xy = vertex - z * self.xy_count
        y = xy // self.nx
        x = xy - y * self.nx
        return x, y, z

    def _unpack_vertex_xy(self, vertex: int) -> tuple[int, int]:
        """Unpack only x and y from a dense vertex id."""
        vertex = int(vertex)
        xy = vertex % self.xy_count
        y = xy // self.nx
        x = xy - y * self.nx
        return x, y

    @staticmethod
    def _mark_ids(bitmap: bytearray, ids: list[int]) -> None:
        """Set cached keepout ids in a mutable bitmap."""
        for item_id in ids:
            bitmap[item_id] = 1


def _route_net_batch_worker(board: BoardData, config: RouterConfig, net_ids: list[int]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    """Worker entry point for process-level net parallelism."""
    router = RandomGridRouter(board, config)
    return router._generate_payload_for_net_ids(net_ids)


def _import_router_core_optional():
    """Import router_core from local build directories when it is available."""
    global _ROUTER_CORE, _ROUTER_CORE_IMPORT_ATTEMPTED
    if _ROUTER_CORE_IMPORT_ATTEMPTED:
        return _ROUTER_CORE
    _ROUTER_CORE_IMPORT_ATTEMPTED = True
    root = Path(__file__).resolve().parents[2]
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
                _DLL_DIR_HANDLES.append(os.add_dll_directory(str(dll_dir)))
            except OSError:
                continue
    candidates = [
        root / "build-random-grid-native" / "Release",
        root / "build-random-grid-native" / "Debug",
        root / "build-random-grid-native",
        root / "build" / "Release",
        root / "build" / "Debug",
        root / "build",
        root / "build-kicad" / "Release",
        root / "build-kicad" / "Debug",
        root / "build-kicad",
        root,
    ]
    for candidate in reversed(candidates):
        if str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
    try:
        import router_core
    except Exception:
        _ROUTER_CORE = None
    else:
        _ROUTER_CORE = router_core
    return _ROUTER_CORE
