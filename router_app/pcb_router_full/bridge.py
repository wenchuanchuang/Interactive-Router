from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from time import perf_counter
from typing import Iterable

from router_app.kicad_parser import BoardData, TrackSegment, Via, load_board


PCBROUTER_GRID_SCALE = 10 # default is 10
PCBROUTER_PROFILE_SCHEMA = "interactive_router_pcbrouter_profiles_v1"
BoardCache = dict[Path, BoardData]


@dataclass(frozen=True)
class PcbRouterSegment:
    layer: str
    start: tuple[float, float]
    end: tuple[float, float]
    width: float


@dataclass(frozen=True)
class PcbRouterVia:
    center: tuple[float, float]
    diameter: float
    start_layer: str
    end_layer: str


@dataclass(frozen=True)
class PcbRouterRipupPreview:
    net_id: int
    net_name: str
    segments: list[PcbRouterSegment]
    vias: list[PcbRouterVia]
    truncated: bool = False
    occurrence_index: int = 0
    source_net_id: int | None = None
    backend: str = "pcbrouter"


@dataclass(frozen=True)
class PcbRouterProfile:
    name: str
    order_strategy: str = "natural"
    order_seed: int | None = None
    grid_scale: int = PCBROUTER_GRID_SCALE
    num_iterations: int | None = None
    enlarge_boundary: int | None = None
    layer_change_weight: float | None = None
    track_obstacle_weight: float | None = None
    track_obstacle_step_size: float | None = None
    via_obstacle_step_size: float | None = None


@dataclass(frozen=True)
class PcbRouterRunResult:
    original_board: BoardData
    original_candidate_board: BoardData | None
    routed_board: BoardData
    ripup_previews: list[PcbRouterRipupPreview]
    work_dir: Path
    routed_board_path: Path
    ripup_payload_path: Path | None
    candidate_routed_boards: tuple[BoardData, ...] = ()
    candidate_routed_board_paths: tuple[Path, ...] = ()
    candidate_ripup_payload_paths: tuple[Path, ...] = ()
    attempted_profiles: tuple[str, ...] = ()
    selected_profile: str = "natural"
    profile_reports: tuple[dict[str, object], ...] = ()
    profile_manifest_path: Path | None = None
    unique_ripped_net_count: int = 0
    ripup_event_count: int = 0
    stdout_log_path: Path | None = None
    stderr_log_path: Path | None = None
    failed_net_ids_estimated: tuple[int, ...] = ()


def run_pcb_router_full(board_path: str | Path, profile_manifest_path: str | Path | None = None) -> PcbRouterRunResult:
    """Run one or more PcbRouter profiles and collect every candidate artifact."""
    board_path = Path(board_path).resolve()
    board_cache: BoardCache = {}
    router_source_board_path = _canonical_router_source_board(board_path, board_cache)
    app_root = Path(__file__).resolve().parents[2]
    work_dir = app_root / "out" / "pcb_router_full" / board_path.stem
    work_dir.mkdir(parents=True, exist_ok=True)

    routed_board_path = work_dir / f"{board_path.stem}.pcbrouter.routed.kicad_pcb"
    ripup_payload_path = work_dir / "pcbrouter_ripped_routes.json"
    metadata_path = work_dir / "pcbrouter_profile_manifest.json"
    stdout_log_path = work_dir / "pcbrouter.selected.stdout.log"
    stderr_log_path = work_dir / "pcbrouter.selected.stderr.log"
    for artifact in (routed_board_path, ripup_payload_path, metadata_path, stdout_log_path, stderr_log_path):
        if artifact.exists():
            artifact.unlink()

    exe_path = _find_pcbrouter_executable(app_root)
    profiles = _load_pcbrouter_profiles(profile_manifest_path or os.environ.get("INTERACTIVE_ROUTER_REPLAY_PROFILE_MANIFEST"))
    max_workers = _profile_max_workers()
    print(
        "pcbrouter_profiles = "
        + ", ".join(profile.name for profile in profiles)
        + f"  # max_parallel={max_workers}",
        flush=True,
    )

    started_at = perf_counter()
    stage_results = _run_pcbrouter_profiles(
        profiles=profiles,
        board_stem=board_path.stem,
        router_source_board_path=router_source_board_path,
        work_dir=work_dir,
        exe_path=exe_path,
        max_workers=max_workers,
        final_repair_enabled=False,
    )
    elapsed_sec = perf_counter() - started_at
    if not stage_results:
        raise RuntimeError("PcbRouter did not produce any profile result.")

    raw_selected_stage = min(
        stage_results,
        key=lambda stage: (
            len(stage["failed_net_ids_estimated"]),
            len(stage["routed_board"].vias),
            sum(len(item.segments) for item in stage["ripup_previews"]),
        ),
    )
    repaired_started_at = perf_counter()
    try:
        selected_stage = _run_pcbrouter_profile_stage(
            profile=raw_selected_stage["profile"],
            board_stem=board_path.stem,
            router_source_board_path=router_source_board_path,
            profile_root=work_dir / "selected_repair_profile",
            exe_path=exe_path,
            final_repair_enabled=True,
        )
        selected_stage["selected_repair_elapsed_sec"] = perf_counter() - repaired_started_at
        stage_results.append(selected_stage)
    except Exception as exc:
        # Keep the raw selected profile if selected-only repair fails, so the
        # bridge still returns a reproducible router result instead of losing
        # all candidates from successful diversity profiles.
        print(f"pcbrouter_selected_repair_failed {raw_selected_stage['profile'].name} reason={exc}", flush=True)
        selected_stage = raw_selected_stage
    _copy_stage_artifact(Path(selected_stage["routed_board_path"]), routed_board_path)
    if selected_stage["ripup_payload_path"] is not None:
        _copy_stage_artifact(Path(selected_stage["ripup_payload_path"]), ripup_payload_path)
        final_payload_path: Path | None = ripup_payload_path
    else:
        final_payload_path = None
    _copy_stage_artifact(Path(selected_stage["stdout_log_path"]), stdout_log_path)
    _copy_stage_artifact(Path(selected_stage["stderr_log_path"]), stderr_log_path)

    original_board, original_candidate_board = _load_selector_original_board(
        input_board_path=board_path,
        router_source_board_path=router_source_board_path,
        board_cache=board_cache,
    )
    routed_board = selected_stage["routed_board"]
    candidate_routed_boards = tuple(stage["routed_board"] for stage in stage_results)
    candidate_routed_board_paths = tuple(Path(stage["routed_board_path"]) for stage in stage_results)
    candidate_ripup_payload_paths = tuple(
        Path(stage["ripup_payload_path"])
        for stage in stage_results
        if stage["ripup_payload_path"] is not None
    )
    ripup_previews = [
        preview
        for stage in stage_results
        for preview in stage["ripup_previews"]
    ]
    ripup_event_count = sum(int(stage["ripup_event_count"]) for stage in stage_results)
    failed_net_ids_estimated = tuple(selected_stage["failed_net_ids_estimated"])
    profile_reports = tuple(_stage_report_for_manifest(stage) for stage in stage_results)
    metadata = {
        "schema": PCBROUTER_PROFILE_SCHEMA,
        "board": str(board_path),
        "router_source_board": str(router_source_board_path),
        "selected_profile": selected_stage["profile"].name,
        "raw_selected_profile": raw_selected_stage["profile"].name,
        "non_selected_profile_repair": "skipped",
        "selected_profile_repair": bool(selected_stage is not raw_selected_stage),
        "max_parallel_profiles": max_workers,
        "elapsed_sec": elapsed_sec,
        "profiles": list(profile_reports),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"pcbrouter_profile_manifest = {metadata_path.resolve()}", flush=True)
    for report in profile_reports:
        print(
            "pcbrouter_profile_result "
            f"{report['name']} = failed_nets={len(report['failed_net_ids_estimated'])} "
            f"vias={report['via_count']} payload={report['ripup_payload_path']}",
            flush=True,
        )
        timing = report.get("timing", {})
        if isinstance(timing, dict):
            print(
                "pcbrouter_profile_timing "
                f"{report['name']} "
                f"prepare_sec={float(timing.get('prepare_sec', 0.0)):.3f} "
                f"subprocess_sec={float(timing.get('subprocess_sec', 0.0)):.3f} "
                f"artifact_collect_sec={float(timing.get('artifact_collect_sec', 0.0)):.3f} "
                f"parser_sec={float(timing.get('internal_parser_sec', 0.0)):.3f} "
                f"route_output_drc_sec={float(timing.get('internal_route_output_drc_sec', 0.0)):.3f} "
                f"repair_sec={float(timing.get('internal_repair_sec', 0.0)):.3f} "
                f"total_sec={float(timing.get('stage_total_sec', 0.0)):.3f}",
                flush=True,
            )

    return PcbRouterRunResult(
        original_board=original_board,
        original_candidate_board=original_candidate_board,
        routed_board=routed_board,
        ripup_previews=ripup_previews,
        work_dir=work_dir,
        routed_board_path=routed_board_path,
        ripup_payload_path=final_payload_path,
        candidate_routed_boards=candidate_routed_boards,
        candidate_routed_board_paths=candidate_routed_board_paths,
        candidate_ripup_payload_paths=candidate_ripup_payload_paths,
        attempted_profiles=tuple(profile.name for profile in profiles),
        selected_profile=str(selected_stage["profile"].name),
        profile_reports=profile_reports,
        profile_manifest_path=metadata_path.resolve(),
        unique_ripped_net_count=len({item.net_id for item in ripup_previews}),
        ripup_event_count=ripup_event_count,
        stdout_log_path=stdout_log_path.resolve(),
        stderr_log_path=stderr_log_path.resolve(),
        failed_net_ids_estimated=failed_net_ids_estimated,
    )


def _load_pcbrouter_profiles(manifest_path: str | Path | None = None) -> tuple[PcbRouterProfile, ...]:
    """Load reproducible PcbRouter profiles from a manifest or use diversity defaults."""
    if manifest_path:
        manifest_file = Path(manifest_path)
        if manifest_file.exists():
            manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
            raw_profiles = manifest.get("profiles", []) if isinstance(manifest, dict) else []
            profiles = [
                _pcbrouter_profile_from_mapping(raw)
                for raw in raw_profiles
                if isinstance(raw, dict) and str(raw.get("router", "pcbrouter")) == "pcbrouter"
            ]
            if profiles:
                return tuple(profiles)

    light_profiles = (
        PcbRouterProfile(name="natural", order_strategy="natural", order_seed=1470295829),
        PcbRouterProfile(name="reverse", order_strategy="reverse", order_seed=1470295829),
        PcbRouterProfile(name="random_seed_1", order_strategy="random", order_seed=1),
    )
    if os.environ.get("INTERACTIVE_ROUTER_PROFILE_SET", "diversity_light") != "diversity_full":
        return light_profiles
    return (
        *light_profiles,
        PcbRouterProfile(name="high_pin_first", order_strategy="high_pin_first", order_seed=1470295829),
        PcbRouterProfile(name="high_obstacle", order_strategy="natural", order_seed=1470295829, track_obstacle_weight=500.0),
    )


def _pcbrouter_profile_from_mapping(raw: dict[str, object]) -> PcbRouterProfile:
    """Convert a manifest profile object into a PcbRouterProfile."""
    params = raw.get("params", {})
    if not isinstance(params, dict):
        params = {}
    return PcbRouterProfile(
        name=str(raw.get("name") or raw.get("id") or "profile"),
        order_strategy=str(raw.get("order_strategy") or params.get("order_strategy") or "natural"),
        order_seed=_optional_int(raw.get("order_seed", params.get("order_seed"))),
        grid_scale=int(params.get("grid_scale", raw.get("grid_scale", PCBROUTER_GRID_SCALE))),
        num_iterations=_optional_int(params.get("num_iterations")),
        enlarge_boundary=_optional_int(params.get("enlarge_boundary")),
        layer_change_weight=_optional_float(params.get("layer_change_weight")),
        track_obstacle_weight=_optional_float(params.get("track_obstacle_weight")),
        track_obstacle_step_size=_optional_float(params.get("track_obstacle_step_size")),
        via_obstacle_step_size=_optional_float(params.get("via_obstacle_step_size")),
    )


def _optional_int(value: object) -> int | None:
    """Parse an optional integer from JSON/env-like values."""
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def _optional_float(value: object) -> float | None:
    """Parse an optional float from JSON/env-like values."""
    if value is None or value == "":
        return None
    try:
        return float(value)
    except Exception:
        return None


def _profile_max_workers() -> int:
    """Limit profile process parallelism so router runs stay reproducible and memory-safe."""
    try:
        value = int(os.environ.get("INTERACTIVE_ROUTER_PROFILE_MAX_PARALLEL", "2"))
    except ValueError:
        value = 2
    return max(1, min(value, 4))


def _run_pcbrouter_profiles(
    profiles: tuple[PcbRouterProfile, ...],
    board_stem: str,
    router_source_board_path: Path,
    work_dir: Path,
    exe_path: Path,
    max_workers: int,
    final_repair_enabled: bool,
) -> list[dict[str, object]]:
    """Run PcbRouter profiles in isolated work directories and return stage artifacts."""
    profile_root = work_dir / "profiles"
    if profile_root.exists():
        shutil.rmtree(profile_root)
    profile_root.mkdir(parents=True, exist_ok=True)

    results: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=min(max_workers, len(profiles))) as executor:
        future_to_profile = {
            executor.submit(
                _run_pcbrouter_profile_stage,
                profile=profile,
                board_stem=board_stem,
                router_source_board_path=router_source_board_path,
                profile_root=profile_root,
                exe_path=exe_path,
                final_repair_enabled=final_repair_enabled,
            ): profile
            for profile in profiles
        }
        for future in as_completed(future_to_profile):
            profile = future_to_profile[future]
            try:
                results.append(future.result())
            except Exception as exc:
                print(f"pcbrouter_profile_failed {profile.name} reason={exc}", flush=True)
    results.sort(key=lambda stage: profiles.index(stage["profile"]))
    return results


def _run_pcbrouter_profile_stage(
    profile: PcbRouterProfile,
    board_stem: str,
    router_source_board_path: Path,
    profile_root: Path,
    exe_path: Path,
    final_repair_enabled: bool,
) -> dict[str, object]:
    """Run one PcbRouter profile in an isolated directory with reproducible environment."""
    stage_started_at = perf_counter()
    prepare_started_at = perf_counter()
    stage_dir = profile_root / _safe_profile_name(profile.name)
    if stage_dir.exists():
        shutil.rmtree(stage_dir)
    output_dir = stage_dir / "output"
    log_dir = stage_dir / "log"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    pcbrouter_input_path = _prepare_pcbrouter_input_board(router_source_board_path, stage_dir / f"{board_stem}.kicad_pcb")
    stdout_log_path = stage_dir / "pcbrouter.stdout.log"
    stderr_log_path = stage_dir / "pcbrouter.stderr.log"
    command = _pcbrouter_command(exe_path, pcbrouter_input_path, profile)
    env = os.environ.copy()
    env["PCBROUTER_ORDER_STRATEGY"] = profile.order_strategy
    env["PCBROUTER_FINAL_REPAIR"] = "1" if final_repair_enabled else "0"
    # Forward the candidate-cleanup experiment explicitly. The PcbRouter side
    # treats "no_check" as an inert diagnostic mode and only performs cleanup
    # for concrete modes such as "y_only", "acute_only", or "1".
    env.pop("PCBROUTER_CANDIDATE_PAD_ENTRY_FIX", None)
    if _env_flag("INTERACTIVE_ROUTER_CANDIDATE_PAD_ENTRY_FIX"):
        env["PCBROUTER_CANDIDATE_PAD_ENTRY_FIX"] = os.environ["INTERACTIVE_ROUTER_CANDIDATE_PAD_ENTRY_FIX"]
    if profile.order_seed is not None:
        env["PCBROUTER_ORDER_SEED"] = str(profile.order_seed)
    prepare_sec = perf_counter() - prepare_started_at
    print(
        f"pcbrouter_profile_start {profile.name} order={profile.order_strategy} "
        f"seed={profile.order_seed} grid_scale={profile.grid_scale} "
        f"final_repair={'on' if final_repair_enabled else 'off'}",
        flush=True,
    )
    subprocess_started_at = perf_counter()
    with stdout_log_path.open("w", encoding="utf-8", errors="replace") as stdout_log, stderr_log_path.open(
        "w", encoding="utf-8", errors="replace"
    ) as stderr_log:
        process = subprocess.run(
            command,
            cwd=str(stage_dir),
            stdout=stdout_log,
            stderr=stderr_log,
            text=True,
            check=False,
            env=env,
        )
    subprocess_sec = perf_counter() - subprocess_started_at
    if process.returncode != 0:
        raise RuntimeError(
            "PcbRouter failed with exit code "
            f"{process.returncode}. stdout={stdout_log_path.resolve()} stderr={stderr_log_path.resolve()}"
        )
    artifact_started_at = perf_counter()
    produced_board = _find_pcbrouter_routed_board(output_dir, board_stem)
    if produced_board is None:
        raise RuntimeError(
            f"PcbRouter completed but no routed board was found under {output_dir.resolve()}. "
            f"stdout={stdout_log_path.resolve()} stderr={stderr_log_path.resolve()}"
        )
    routed_board_path = stage_dir / f"{board_stem}.{profile.name}.pcbrouter.routed.kicad_pcb"
    shutil.copyfile(produced_board, routed_board_path)
    produced_payload = output_dir / "pcbrouter_ripped_routes.json"
    ripup_payload_path: Path | None = None
    if produced_payload.exists():
        ripup_payload_path = stage_dir / f"{profile.name}.pcbrouter_ripped_routes.json"
        shutil.copyfile(produced_payload, ripup_payload_path)
    routed_board = load_board(routed_board_path)
    artifact_collect_sec = perf_counter() - artifact_started_at
    internal_timing = _parse_pcbrouter_stdout_timing(stdout_log_path)
    timing = {
        "prepare_sec": prepare_sec,
        "subprocess_sec": subprocess_sec,
        "artifact_collect_sec": artifact_collect_sec,
        "stage_total_sec": perf_counter() - stage_started_at,
        **internal_timing,
    }
    return {
        "profile": profile,
        "stage_dir": stage_dir,
        "routed_board": routed_board,
        "routed_board_path": routed_board_path.resolve(),
        "ripup_payload_path": ripup_payload_path.resolve() if ripup_payload_path is not None else None,
        "ripup_previews": _load_ripup_previews(ripup_payload_path, backend_label=f"pcbrouter:{profile.name}"),
        "ripup_event_count": _read_ripup_event_count(ripup_payload_path),
        "failed_net_ids_estimated": tuple(_estimate_failed_nets(routed_board)),
        "stdout_log_path": stdout_log_path.resolve(),
        "stderr_log_path": stderr_log_path.resolve(),
        "timing": timing,
        "final_repair_enabled": final_repair_enabled,
    }


def _pcbrouter_command(exe_path: Path, board_path: Path, profile: PcbRouterProfile) -> list[str]:
    """Build the positional PcbRouter command while preserving old defaults."""
    command = [str(exe_path), str(board_path), str(profile.grid_scale)]
    optional_values = [
        (profile.num_iterations, 5),
        (profile.enlarge_boundary, 0),
        (profile.layer_change_weight, 10.0),
        (profile.track_obstacle_weight, 50.0),
        (profile.track_obstacle_step_size, 0.0),
        (profile.via_obstacle_step_size, 0.0),
    ]
    while optional_values and optional_values[-1][0] is None:
        optional_values.pop()
    command.extend(str(value if value is not None else default) for value, default in optional_values)
    return command


def _safe_profile_name(name: str) -> str:
    """Make a profile name safe for local folder and artifact names."""
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    return safe or "profile"


def _copy_stage_artifact(source: Path, destination: Path) -> None:
    """Copy a selected profile artifact unless it already has the target path."""
    if source.resolve() == destination.resolve():
        return
    shutil.copyfile(source, destination)


def _stage_report_for_manifest(stage: dict[str, object]) -> dict[str, object]:
    """Serialize one PcbRouter profile stage for replay and debugging."""
    profile = stage["profile"]
    assert isinstance(profile, PcbRouterProfile)
    return {
        "router": "pcbrouter",
        "name": profile.name,
        "order_strategy": profile.order_strategy,
        "order_seed": profile.order_seed,
        "params": {
            "grid_scale": profile.grid_scale,
            "num_iterations": profile.num_iterations,
            "enlarge_boundary": profile.enlarge_boundary,
            "layer_change_weight": profile.layer_change_weight,
            "track_obstacle_weight": profile.track_obstacle_weight,
            "track_obstacle_step_size": profile.track_obstacle_step_size,
            "via_obstacle_step_size": profile.via_obstacle_step_size,
        },
        "routed_board_path": str(stage["routed_board_path"]),
        "ripup_payload_path": str(stage["ripup_payload_path"]) if stage["ripup_payload_path"] is not None else None,
        "stdout_log_path": str(stage["stdout_log_path"]),
        "stderr_log_path": str(stage["stderr_log_path"]),
        "via_count": len(stage["routed_board"].vias),
        "failed_net_ids_estimated": list(stage["failed_net_ids_estimated"]),
        "ripup_event_count": int(stage["ripup_event_count"]),
        "timing": stage.get("timing", {}),
        "final_repair_enabled": bool(stage.get("final_repair_enabled", True)),
    }


def _parse_pcbrouter_stdout_timing(stdout_log_path: Path) -> dict[str, float]:
    """Extract coarse PcbRouter phase timing from stdout without touching router code.

    The router already prints parser, GridBasedRouter, final total, and repair
    timing lines. This parser keeps the bridge-level timing reproducible and
    avoids changing the routing process itself.
    """
    timing = {
        "internal_parser_sec": 0.0,
        "internal_grid_router_sec": 0.0,
        "internal_final_real_sec": 0.0,
        "internal_repair_sec": 0.0,
        "internal_route_output_drc_sec": 0.0,
    }
    try:
        text = stdout_log_path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return timing

    for label, seconds in re.findall(
        r"-+\s*(.*?)\s+period time usage\s*-+\s*[\r\n]+\s*Real:([0-9.]+)s;",
        text,
        flags=re.IGNORECASE,
    ):
        normalized_label = label.strip().lower()
        value = float(seconds)
        if normalized_label == "parser":
            timing["internal_parser_sec"] = value
        elif normalized_label == "gridbasedrouter":
            timing["internal_grid_router_sec"] = value

    final_match = re.search(r"Final Real:([0-9.]+)s;", text)
    if final_match:
        timing["internal_final_real_sec"] = float(final_match.group(1))

    repair_match = re.search(r"pcbrouter_final_repair_timing_sec\s*=\s*([0-9.]+)", text)
    if repair_match:
        timing["internal_repair_sec"] = float(repair_match.group(1))

    if timing["internal_grid_router_sec"] > 0.0:
        timing["internal_route_output_drc_sec"] = max(
            0.0,
            timing["internal_grid_router_sec"] - timing["internal_repair_sec"],
        )
    elif timing["internal_final_real_sec"] > 0.0:
        timing["internal_route_output_drc_sec"] = max(
            0.0,
            timing["internal_final_real_sec"] - timing["internal_parser_sec"] - timing["internal_repair_sec"],
        )
    return timing


def _prepare_pcbrouter_input_board(board_path: Path, output_path: Path) -> Path:
    """Return a PcbRouter input board that always contains Edge.Cuts.

    PcbRouter builds its routing grid from Edge.Cuts. Some paper/result boards
    keep routed tracks but omit the board outline, so this preflight either
    copies a valid board as-is or creates a temporary copy with Edge.Cuts pulled
    from a matching sibling unrouted board. Sibling outlines are translated into
    the input board's coordinate frame so PcbRouter output stays aligned with
    Interactive-Router's displayed original board.
    """
    board_text = board_path.read_text(encoding="utf-8")
    if _extract_top_level_edge_cut_items(board_text):
        shutil.copyfile(board_path, output_path)
        return output_path

    sibling = _find_sibling_unrouted_board(board_path)
    if sibling is None:
        raise RuntimeError(
            "PcbRouter input board has no Edge.Cuts, and no matching sibling "
            f"unrouted board was found for outline recovery: {board_path}"
        )

    sibling_text = sibling.read_text(encoding="utf-8")
    edge_cut_items = _extract_top_level_edge_cut_items(sibling_text)
    if not edge_cut_items:
        raise RuntimeError(
            "PcbRouter input board has no Edge.Cuts, and the matching sibling "
            f"board also has no Edge.Cuts: input={board_path} sibling={sibling}"
        )

    dx, dy = _board_translation_from_common_pads(source_board=sibling, target_board=board_path)
    translated_edge_cut_items = [_translate_edge_cut_item(item, dx, dy) for item in edge_cut_items]
    output_path.write_text(_insert_top_level_items_before_board_end(board_text, translated_edge_cut_items), encoding="utf-8")
    print(f"pcbrouter_edge_cuts_source = {sibling.resolve()}", flush=True)
    print(f"pcbrouter_edge_cuts_translation_mm = dx={dx:.6f} dy={dy:.6f}", flush=True)
    print(f"pcbrouter_input_board = {output_path.resolve()}", flush=True)
    print("pcbrouter_input_strategy = translated_edge_cuts", flush=True)
    return output_path


def _canonical_router_source_board(board_path: Path, board_cache: BoardCache | None = None) -> Path:
    """Choose the board coordinate frame used by PcbRouter.

    If a routed reference board is only a translated copy of its sibling
    unrouted board, running PcbRouter on the sibling keeps the router input
    identical to a direct `.unrouted` run. The routed input's tracks/vias are
    translated back later and kept as original selector candidates.
    """
    sibling = _find_sibling_unrouted_board(board_path)
    if sibling is None:
        return board_path
    try:
        dx, dy = _board_translation_from_common_pads(source_board=sibling, target_board=board_path, board_cache=board_cache)
    except Exception:
        return board_path
    print(f"pcbrouter_router_source_board = {sibling.resolve()}", flush=True)
    print(f"pcbrouter_input_to_source_translation_mm = dx={dx:.6f} dy={dy:.6f}", flush=True)
    print("pcbrouter_router_input_strategy = sibling_unrouted_coordinate_frame", flush=True)
    return sibling


def _load_selector_original_board(
    input_board_path: Path,
    router_source_board_path: Path,
    board_cache: BoardCache | None = None,
) -> tuple[BoardData, BoardData | None]:
    """Load selector data in router coordinates while preserving input routes.

    Selector geometry must share the external router's coordinate frame. For a
    translated routed input, the selector board remains the sibling unrouted
    board so routed tracks do not change grid bounds. Existing tracks and vias
    are shifted back separately and exposed as original candidate geometry.
    """
    if input_board_path == router_source_board_path:
        return _load_board_cached(input_board_path, board_cache), None

    source_board = _load_board_cached(router_source_board_path, board_cache)
    input_board = _load_board_cached(input_board_path, board_cache)
    dx, dy = _board_translation_from_common_pads(
        source_board=router_source_board_path,
        target_board=input_board_path,
        board_cache=board_cache,
    )
    translated_tracks = [
        TrackSegment(
            start=(track.start[0] - dx, track.start[1] - dy),
            end=(track.end[0] - dx, track.end[1] - dy),
            width=track.width,
            layer=track.layer,
            net_id=track.net_id,
            net_name=track.net_name,
        )
        for track in input_board.tracks
    ]
    translated_vias = [
        Via(
            center=(via.center[0] - dx, via.center[1] - dy),
            diameter=via.diameter,
            net_id=via.net_id,
            net_name=via.net_name,
            start_layer=via.start_layer,
            end_layer=via.end_layer,
        )
        for via in input_board.vias
    ]
    print(
        "pcbrouter_original_candidate_translation_mm = "
        f"dx={-dx:.6f} dy={-dy:.6f} tracks={len(translated_tracks)} vias={len(translated_vias)}",
        flush=True,
    )
    translated_candidate_board = BoardData(
        path=router_source_board_path,
        nets=source_board.nets,
        tracks=translated_tracks,
        footprints=source_board.footprints,
        vias=translated_vias,
        backend=f"{source_board.backend}+translated-original",
        design_rules=source_board.design_rules,
        net_clearances=source_board.net_clearances,
        declared_copper_layers=source_board.declared_copper_layers,
        layer_aliases=source_board.layer_aliases,
    )
    return source_board, translated_candidate_board


def _board_translation_from_common_pads(
    source_board: Path,
    target_board: Path,
    board_cache: BoardCache | None = None,
) -> tuple[float, float]:
    """Calculate a constant source-to-target board translation from common pads.

    PcbRouter must route in the same coordinate frame as the displayed board.
    When a sibling board provides only the missing outline, common footprint/pad
    centers give a robust way to move that outline into the target board frame.
    """
    source_pads = _pad_centers_by_key(_load_board_cached(source_board, board_cache))
    target_pads = _pad_centers_by_key(_load_board_cached(target_board, board_cache))
    common_keys = sorted(set(source_pads) & set(target_pads))
    if len(common_keys) < 3:
        raise RuntimeError(
            "Could not recover Edge.Cuts from sibling board because too few "
            f"common pads were found: source={source_board} target={target_board}"
        )

    offsets = [
        (target_pads[key][0] - source_pads[key][0], target_pads[key][1] - source_pads[key][1])
        for key in common_keys
    ]
    dx = _median(value[0] for value in offsets)
    dy = _median(value[1] for value in offsets)
    max_deviation = max(max(abs(item_dx - dx), abs(item_dy - dy)) for item_dx, item_dy in offsets)
    if max_deviation > 0.01:
        raise RuntimeError(
            "Could not recover Edge.Cuts from sibling board because common pads "
            f"are not a simple translation: max_deviation_mm={max_deviation:.6f} "
            f"source={source_board} target={target_board}"
        )
    return dx, dy


def _load_board_cached(board_path: Path, board_cache: BoardCache | None = None) -> BoardData:
    """Load a board once per bridge run and reuse it for metadata-only reads."""
    if board_cache is None:
        return load_board(board_path)
    key = Path(board_path).resolve()
    cached = board_cache.get(key)
    if cached is None:
        cached = load_board(key)
        board_cache[key] = cached
    return cached


def _pad_centers_by_key(board: BoardData) -> dict[tuple[str, str], tuple[float, float]]:
    """Index pad centers by footprint reference and pad name."""
    centers: dict[tuple[str, str], tuple[float, float]] = {}
    for footprint in board.footprints:
        for pad in footprint.pads:
            centers[(footprint.reference, pad.name)] = pad.center
    return centers


def _median(values: Iterable[float]) -> float:
    """Return the median value without importing statistics in hot startup code."""
    ordered = sorted(values)
    if not ordered:
        raise RuntimeError("Cannot calculate a median from an empty value list.")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


_EDGE_CUT_POINT_RE = re.compile(
    r"\((start|end|center|mid|xy)\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)"
)


def _translate_edge_cut_item(item_text: str, dx: float, dy: float) -> str:
    """Translate coordinate-bearing Edge.Cuts forms by a board-frame offset."""

    def replace_point(match: re.Match[str]) -> str:
        symbol = match.group(1)
        x = float(match.group(2)) + dx
        y = float(match.group(3)) + dy
        return f"({symbol} {_format_mm(x)} {_format_mm(y)}"

    return _EDGE_CUT_POINT_RE.sub(replace_point, item_text)


def _format_mm(value: float) -> str:
    """Format KiCad millimeter coordinates with stable compact precision."""
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def _find_sibling_unrouted_board(board_path: Path) -> Path | None:
    """Find a likely unrouted board that shares the same board outline."""
    candidates: list[Path] = []
    stem = board_path.stem
    if stem.endswith(".routed"):
        candidates.append(board_path.with_name(stem[: -len(".routed")] + ".unrouted" + board_path.suffix))
    if ".routed." in stem:
        # Paper/result files may include extra suffixes after `.routed`; use
        # the prefix before that marker as the canonical unrouted sibling.
        candidates.append(board_path.with_name(stem.split(".routed.", 1)[0] + board_path.suffix))
    candidates.append(board_path.with_name(board_path.name.replace(".routed.", ".unrouted.")))

    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate.exists() and candidate != board_path:
            return candidate
    return None


def _extract_top_level_edge_cut_items(text: str) -> list[str]:
    """Collect top-level KiCad drawing items on the Edge.Cuts layer."""
    items: list[str] = []
    for start, end, item_text in _top_level_form_ranges(text):
        if _form_symbol(item_text) in {"gr_line", "gr_arc", "gr_circle", "gr_curve", "gr_poly", "gr_rect"}:
            if re.search(r"\(layer\s+(?:\"Edge\.Cuts\"|Edge\.Cuts)\)", item_text):
                # Preserve the original formatting and item geometry from the sibling board.
                items.append(text[start:end].rstrip())
    return items


def _insert_top_level_items_before_board_end(text: str, items: list[str]) -> str:
    """Insert top-level board items immediately before the closing board paren."""
    insert_at = len(text.rstrip()) - 1
    if insert_at < 0 or text[insert_at] != ")":
        raise RuntimeError("Could not find the closing KiCad board parenthesis for Edge.Cuts insertion.")
    insertion = "\n" + "\n".join(items) + "\n"
    return text[:insert_at] + insertion + text[insert_at:]


def _top_level_form_ranges(text: str) -> list[tuple[int, int, str]]:
    """Return ranges for forms directly under the `(kicad_pcb ...)` root."""
    ranges: list[tuple[int, int, str]] = []
    depth = 0
    index = 0
    while index < len(text):
        char = text[index]
        if char == '"':
            index = _skip_quoted_string(text, index + 1)
            continue
        if char == "(":
            if depth == 1:
                end = _matching_paren_end(text, index)
                if end is not None:
                    ranges.append((index, end, text[index:end]))
                    index = end
                    continue
            depth += 1
        elif char == ")":
            depth = max(0, depth - 1)
        index += 1
    return ranges


def _form_symbol(form_text: str) -> str:
    """Read the leading symbol from a KiCad S-expression form."""
    match = re.match(r"\s*\(\s*([^\s\)]+)", form_text)
    return match.group(1) if match else ""


def _skip_quoted_string(text: str, index: int) -> int:
    """Return the first index after a quoted KiCad string."""
    escaped = False
    while index < len(text):
        char = text[index]
        if char == '"' and not escaped:
            return index + 1
        escaped = char == "\\" and not escaped
        if char != "\\":
            escaped = False
        index += 1
    return len(text)


def _matching_paren_end(text: str, start: int) -> int | None:
    """Return the exclusive end offset of the S-expression at start."""
    depth = 0
    index = start
    while index < len(text):
        char = text[index]
        if char == '"':
            index = _skip_quoted_string(text, index + 1)
            continue
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                end = index + 1
                while end < len(text) and text[end] in " \t\r\n":
                    end += 1
                return end
        index += 1
    return None


def _find_pcbrouter_executable(app_root: Path) -> Path:
    """Resolve PcbRouter from PCBROUTER_EXE first, then repository defaults."""
    env_path = os.environ.get("PCBROUTER_EXE")
    candidates: list[Path] = []
    if env_path:
        candidates.append(Path(env_path))
    workspace_root = app_root.parent
    candidates.extend(
        [
            workspace_root / "PcbRouter" / "bin" / "pcbrouter.exe",
            workspace_root / "PcbRouter" / "bin" / "Release" / "pcbrouter.exe",
            workspace_root / "PcbRouter" / "bin" / "pcbrouter",
            workspace_root / "PcbRouter" / "build" / "Release" / "pcbrouter.exe",
            workspace_root / "PcbRouter" / "build" / "pcbrouter.exe",
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise RuntimeError(
        "PcbRouter executable was not found. Set PCBROUTER_EXE or build "
        f"{workspace_root / 'PcbRouter' / 'bin' / 'pcbrouter.exe'}."
    )


def _find_pcbrouter_routed_board(output_dir: Path, board_stem: str) -> Path | None:
    """Find the final PcbRouter board, preferring the after-post-processing output."""
    if not output_dir.exists():
        return None
    patterns = [
        f"output.afterPostProcessing.*.{board_stem}.kicad_pcb",
        f"output.bestSolutionWithMerging.*.{board_stem}.kicad_pcb",
        f"*.{board_stem}.kicad_pcb",
        "*.kicad_pcb",
    ]
    for pattern in patterns:
        matches = sorted(output_dir.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
        if matches:
            return matches[0]
    return None


def _load_ripup_previews(payload_path: Path | None, backend_label: str = "pcbrouter") -> list[PcbRouterRipupPreview]:
    """Load PcbRouter ripped route previews from its component occurrence payload."""
    if payload_path is None or not payload_path.exists():
        return []
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    previews: list[PcbRouterRipupPreview] = []
    for item in payload.get("nets", []):
        if not isinstance(item, dict):
            continue
        try:
            net_id = int(item.get("net_id", 0))
        except Exception:
            continue
        net_name = str(item.get("net_name", ""))
        for occurrence in item.get("occurrences", []):
            if not isinstance(occurrence, dict):
                continue
            segments = _parse_segments(occurrence.get("segments", []))
            vias = _parse_vias(occurrence.get("vias", []))
            if not segments and not vias:
                continue
            source_net_id = occurrence.get("source_net_id")
            try:
                source_net_id = int(source_net_id) if source_net_id is not None else None
            except Exception:
                source_net_id = None
            effective_net_id = source_net_id if source_net_id is not None and source_net_id > 0 else net_id
            previews.append(
                PcbRouterRipupPreview(
                    net_id=effective_net_id,
                    net_name=net_name,
                    segments=segments,
                    vias=vias,
                    truncated=bool(occurrence.get("truncated", False)),
                    occurrence_index=int(occurrence.get("event_index", 0)),
                    source_net_id=source_net_id,
                    backend=backend_label,
                )
            )
    previews.sort(key=lambda item: (item.occurrence_index, item.net_id))
    return previews


def _parse_segments(raw_segments: object) -> list[PcbRouterSegment]:
    if not isinstance(raw_segments, list):
        return []
    return [
        PcbRouterSegment(
            layer=str(segment.get("layer", "F.Cu")),
            start=(
                float(segment.get("start", {}).get("x_mm", 0.0)),
                -float(segment.get("start", {}).get("y_mm", 0.0)),
            ),
            end=(
                float(segment.get("end", {}).get("x_mm", 0.0)),
                -float(segment.get("end", {}).get("y_mm", 0.0)),
            ),
            width=float(segment.get("width_mm", 0.2)),
        )
        for segment in raw_segments
        if isinstance(segment, dict)
    ]


def _parse_vias(raw_vias: object) -> list[PcbRouterVia]:
    if not isinstance(raw_vias, list):
        return []
    return [
        PcbRouterVia(
            center=(
                float(via.get("center", {}).get("x_mm", 0.0)),
                -float(via.get("center", {}).get("y_mm", 0.0)),
            ),
            diameter=float(via.get("diameter_mm", 0.6)),
            start_layer=str(via.get("start_layer", "F.Cu")),
            end_layer=str(via.get("end_layer", "B.Cu")),
        )
        for via in raw_vias
        if isinstance(via, dict)
    ]


def _read_ripup_event_count(payload_path: Path | None) -> int:
    if payload_path is None or not payload_path.exists():
        return 0
    try:
        payload = json.loads(payload_path.read_text(encoding="utf-8"))
    except Exception:
        return 0
    try:
        return int(payload.get("ripup_event_count", 0))
    except Exception:
        return 0


def _env_flag(name: str) -> bool:
    """Return true when an environment switch is set to an enabled value."""
    value = os.environ.get(name, "")
    return value not in {"", "0", "false", "FALSE", "False"}


def _estimate_failed_nets(board: BoardData) -> list[int]:
    """Estimate failed nets from the final board using the existing no-route heuristic."""
    pads_by_net: dict[int, int] = {}
    routed_nets: set[int] = set()
    for footprint in board.footprints:
        for pad in footprint.pads:
            net_id = int(pad.net_id or 0)
            if net_id > 0:
                pads_by_net[net_id] = pads_by_net.get(net_id, 0) + 1
    for track in board.tracks:
        if int(track.net_id) > 0:
            routed_nets.add(int(track.net_id))
    for via in board.vias:
        if int(via.net_id) > 0:
            routed_nets.add(int(via.net_id))
    return sorted(net_id for net_id, pad_count in pads_by_net.items() if pad_count >= 2 and net_id not in routed_nets)
