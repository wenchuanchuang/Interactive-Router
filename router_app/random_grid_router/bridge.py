from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from time import perf_counter

from router_app.freerouting_full.bridge import FreeroutingRipupPreview, _load_ripup_previews
from router_app.kicad_parser import BoardData, load_board

from .config import artifact_root, router_config
from .router import RandomGridRouter


@dataclass(frozen=True)
class RandomGridRunResult:
    """Candidate-only result for the randomized grid router."""

    original_board: BoardData
    ripup_previews: list[FreeroutingRipupPreview]
    work_dir: Path
    ripup_payload_path: Path
    candidate_ripup_payload_paths: tuple[Path, ...]
    unique_ripped_net_count: int
    candidate_count: int
    infeasible_net_count: int
    elapsed_seconds: float


def run_random_grid_router(board_path: str | Path) -> RandomGridRunResult:
    """Generate candidate-only randomized routes for every multi-pin net."""
    started_at = perf_counter()
    board_path = Path(board_path).resolve()
    board = load_board(board_path)
    app_root = Path(__file__).resolve().parents[2]
    work_dir = artifact_root(app_root) / "random_grid_router" / board_path.stem
    work_dir.mkdir(parents=True, exist_ok=True)

    config = router_config(board)
    router = RandomGridRouter(board, config)
    payload = router.generate_payload()
    payload_path = work_dir / "random_grid_router_ripped_routes.json"
    payload_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    previews = _load_ripup_previews(payload_path)
    elapsed = perf_counter() - started_at

    metadata = {
        "schema_version": 1,
        "board": str(board_path),
        "payload": str(payload_path),
        "grid_pitch_mm": config.pitch_mm,
        "random_per_net": config.random_per_net,
        "max_attempts_per_net": config.max_attempts_per_net,
        "target_pad_limit": config.target_pad_limit,
        "search_window_initial_mm": config.search_window_initial_mm,
        "search_window_max_mm": config.search_window_max_mm,
        "search_window_growth": config.search_window_growth,
        "source_frontier_limit": config.source_frontier_limit,
        "parallel_workers": config.parallel_workers,
        "candidate_count": int(payload.get("candidate_count", 0)),
        "infeasible_net_count": int(payload.get("infeasible_net_count", 0)),
        "elapsed_sec": elapsed,
    }
    (work_dir / "run_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(
        "random_grid_router_summary "
        f"nets={payload.get('net_count', 0)} candidates={payload.get('candidate_count', 0)} "
        f"infeasible={payload.get('infeasible_net_count', 0)} elapsed_sec={elapsed:.3f} "
        f"payload={payload_path.resolve()}",
        flush=True,
    )
    return RandomGridRunResult(
        original_board=board,
        ripup_previews=previews,
        work_dir=work_dir,
        ripup_payload_path=payload_path,
        candidate_ripup_payload_paths=(payload_path,),
        unique_ripped_net_count=len({preview.net_id for preview in previews}),
        candidate_count=int(payload.get("candidate_count", 0)),
        infeasible_net_count=int(payload.get("infeasible_net_count", 0)),
        elapsed_seconds=elapsed,
    )
