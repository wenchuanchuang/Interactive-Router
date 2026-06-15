from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

from router_app.kicad_parser import BoardData


@dataclass(frozen=True)
class RouterConfig:
    """Runtime knobs for random-grid candidate generation."""

    pitch_mm: float
    random_per_net: int
    max_attempts_per_net: int
    trace_width_mm: float
    via_diameter_mm: float
    via_drill_mm: float
    clearance_mm: float
    edge_clearance_mm: float
    seed: int
    target_pad_limit: int
    search_window_initial_mm: float
    search_window_max_mm: float
    search_window_growth: float
    source_frontier_limit: int
    parallel_workers: int


def router_config(board: BoardData) -> RouterConfig:
    """Read random-grid router settings from environment and board rules."""
    rules = board.design_rules or {}
    clearance = float(rules.get("min_clearance", 0.2) or 0.2)
    via_diameter = float(rules.get("via_diameter", rules.get("netclass_via_diameter", 0.6)) or 0.6)
    via_drill = float(rules.get("via_drill", rules.get("netclass_via_drill", max(0.1, via_diameter * 0.5))) or max(0.1, via_diameter * 0.5))
    random_per_net = _env_int("INTERACTIVE_ROUTER_RANDOM_GRID_RANDOM_PER_NET", 1)
    return RouterConfig(
        pitch_mm=_env_float("INTERACTIVE_ROUTER_RANDOM_GRID_PITCH_MM", 0.1),
        random_per_net=random_per_net,
        max_attempts_per_net=_env_int("INTERACTIVE_ROUTER_RANDOM_GRID_MAX_ATTEMPTS", max(2, random_per_net * 2)),
        trace_width_mm=float(rules.get("track_width", 0.2) or 0.2),
        via_diameter_mm=via_diameter,
        via_drill_mm=via_drill,
        clearance_mm=clearance,
        edge_clearance_mm=float(rules.get("copper_edge_clearance", rules.get("min_clearance", clearance)) or clearance),
        seed=_env_int("INTERACTIVE_ROUTER_RANDOM_GRID_SEED", 1),
        target_pad_limit=_env_int("INTERACTIVE_ROUTER_RANDOM_GRID_TARGET_PAD_LIMIT", 2),
        search_window_initial_mm=_env_float("INTERACTIVE_ROUTER_RANDOM_GRID_SEARCH_WINDOW_INITIAL_MM", 5.0),
        search_window_max_mm=_env_float("INTERACTIVE_ROUTER_RANDOM_GRID_SEARCH_WINDOW_MAX_MM", 40.0),
        search_window_growth=_env_float("INTERACTIVE_ROUTER_RANDOM_GRID_SEARCH_WINDOW_GROWTH", 2.0),
        source_frontier_limit=_env_int("INTERACTIVE_ROUTER_RANDOM_GRID_SOURCE_FRONTIER_LIMIT", 64),
        parallel_workers=_env_int("INTERACTIVE_ROUTER_RANDOM_GRID_PARALLEL_WORKERS", 1),
    )


def artifact_root(app_root: Path) -> Path:
    """Return the run artifact root shared with the other external routers."""
    raw_root = os.environ.get("INTERACTIVE_ROUTER_ARTIFACT_ROOT")
    if raw_root:
        return Path(raw_root).resolve()
    return app_root / "out"


def _env_int(name: str, default: int) -> int:
    """Parse an integer environment override."""
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    """Parse a float environment override."""
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default

