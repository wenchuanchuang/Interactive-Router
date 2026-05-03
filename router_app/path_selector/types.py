from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NetCandidates:
    net_id: int
    candidate_paths_grid: list[list[Any]]
    candidate_paths_mm: list[list[Any]]


@dataclass(frozen=True)
class NetSelection:
    net_id: int
    selected_candidate_indices: list[int]
    solver: str
    objective: float | None = None

