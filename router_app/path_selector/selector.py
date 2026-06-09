from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .types import NetCandidates, NetSelection


@dataclass(frozen=True)
class SelectionConfig:
    max_paths_per_net: int = 1
    allow_fallback: bool = True


@dataclass(frozen=True)
class SelectionResult:
    ok: bool
    selections: list[NetSelection]
    solver: str
    message: str
    artifacts: dict[str, str] | None = None


def _path_length_mm(path_mm: list[Any]) -> float:
    total = 0.0
    if len(path_mm) < 2:
        return total
    for start, end in zip(path_mm, path_mm[1:]):
        sx = float(getattr(start, "x", 0.0))
        sy = float(getattr(start, "y", 0.0))
        ex = float(getattr(end, "x", 0.0))
        ey = float(getattr(end, "y", 0.0))
        dx = ex - sx
        dy = ey - sy
        total += (dx * dx + dy * dy) ** 0.5
    return total


def _build_input(results: list[Any]) -> list[NetCandidates]:
    nets: list[NetCandidates] = []
    for result in results:
        net_id = int(getattr(result, "net_id", 0))
        grid_paths = list(getattr(result, "candidate_paths_grid", []))
        mm_paths = list(getattr(result, "candidate_paths_mm", []))
        if not grid_paths and getattr(result, "path_grid", None):
            grid_paths = [list(getattr(result, "path_grid", []))]
            mm_paths = [list(getattr(result, "path_mm", []))]
        nets.append(
            NetCandidates(
                net_id=net_id,
                candidate_paths_grid=grid_paths,
                candidate_paths_mm=mm_paths,
            )
        )
    return nets


def _fallback_select_shortest(
    nets: list[NetCandidates],
    config: SelectionConfig,
) -> SelectionResult:
    selections: list[NetSelection] = []
    for net in nets:
        if not net.candidate_paths_grid:
            selections.append(
                NetSelection(
                    net_id=net.net_id,
                    selected_candidate_indices=[],
                    solver="fallback-shortest",
                    objective=None,
                )
            )
            continue
        lengths = [
            _path_length_mm(path_mm) if index < len(net.candidate_paths_mm) else 0.0
            for index, path_mm in enumerate(net.candidate_paths_mm)
        ]
        ranked = list(range(len(net.candidate_paths_grid)))
        ranked.sort(key=lambda i: lengths[i] if i < len(lengths) else 0.0)
        selected = ranked[: max(1, config.max_paths_per_net)]
        objective = sum(lengths[i] for i in selected if i < len(lengths))
        selections.append(
            NetSelection(
                net_id=net.net_id,
                selected_candidate_indices=selected,
                solver="fallback-shortest",
                objective=objective,
            )
        )
    return SelectionResult(
        ok=True,
        selections=selections,
        solver="fallback-shortest",
        message="Fallback selector chose shortest candidate path(s) per net.",
    )


def _gurobi_select(
    nets: list[NetCandidates],
    config: SelectionConfig,
) -> SelectionResult:
    # Stubbed model entry point: lets us wire integration first.
    # You can replace this block with the full ILP model from tmp.cpp logic.
    try:
        import gurobipy as gp  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("gurobipy is not available in this environment.") from exc

    _ = gp
    return _fallback_select_shortest(nets, config)


def select_paths(
    results: list[Any],
    config: SelectionConfig | None = None,
    prefer_gurobi: bool = True,
) -> SelectionResult:
    config = config or SelectionConfig()
    nets = _build_input(results)

    if prefer_gurobi:
        try:
            return _gurobi_select(nets, config)
        except Exception as exc:
            if not config.allow_fallback:
                return SelectionResult(
                    ok=False,
                    selections=[],
                    solver="gurobi",
                    message=f"Gurobi selection failed: {exc}",
                )

    return _fallback_select_shortest(nets, config)

