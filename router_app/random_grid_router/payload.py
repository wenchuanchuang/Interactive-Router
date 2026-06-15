from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RouteCandidate:
    """One generated route occurrence in board coordinates."""

    segments: list[dict[str, Any]]
    vias: list[dict[str, Any]]
    grid_path: list[int]


def merge_grid_paths(paths: list[list[int]]) -> list[int]:
    """Flatten generated tree paths into a stable unique vertex list."""
    merged: list[int] = []
    seen: set[int] = set()
    for path in paths:
        for vertex in path:
            if vertex not in seen:
                seen.add(vertex)
                merged.append(vertex)
    return merged


def dedupe_segments(segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Remove duplicate serialized segments while preserving order."""
    seen: set[tuple[Any, ...]] = set()
    result = []
    for segment in segments:
        key = (
            segment["layer"],
            segment["start"]["x_mm"], segment["start"]["y_mm"],
            segment["end"]["x_mm"], segment["end"]["y_mm"],
            segment["width_mm"],
        )
        reverse_key = (
            segment["layer"],
            segment["end"]["x_mm"], segment["end"]["y_mm"],
            segment["start"]["x_mm"], segment["start"]["y_mm"],
            segment["width_mm"],
        )
        if key in seen or reverse_key in seen:
            continue
        seen.add(key)
        result.append(segment)
    return result


def dedupe_vias(vias: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Remove duplicate serialized vias while preserving order."""
    seen: set[tuple[Any, ...]] = set()
    result = []
    for via in vias:
        key = (
            via["center"]["x_mm"], via["center"]["y_mm"],
            via["diameter_mm"], via["drill_mm"],
            via["start_layer"], via["end_layer"],
        )
        if key in seen:
            continue
        seen.add(key)
        result.append(via)
    return result


def candidate_key(candidate: RouteCandidate) -> tuple[tuple[Any, ...], tuple[Any, ...]]:
    """Return a route signature used to avoid duplicate random candidates."""
    segments = tuple(
        sorted(
            (
                segment["layer"],
                segment["start"]["x_mm"], segment["start"]["y_mm"],
                segment["end"]["x_mm"], segment["end"]["y_mm"],
            )
            for segment in candidate.segments
        )
    )
    vias = tuple(
        sorted((via["center"]["x_mm"], via["center"]["y_mm"], via["start_layer"], via["end_layer"]) for via in candidate.vias)
    )
    return segments, vias
