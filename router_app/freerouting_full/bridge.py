from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import pcbnew

from router_app.kicad_parser import BoardData, load_board


@dataclass(frozen=True)
class FreeroutingSegment:
    layer: str
    start: tuple[float, float]
    end: tuple[float, float]
    width: float


@dataclass(frozen=True)
class FreeroutingVia:
    center: tuple[float, float]
    diameter: float
    start_layer: str
    end_layer: str


@dataclass(frozen=True)
class FreeroutingRipupPreview:
    net_id: int
    net_name: str
    segments: list[FreeroutingSegment]
    vias: list[FreeroutingVia]
    truncated: bool = False
    occurrence_index: int = 0
    source_net_id: int | None = None


@dataclass(frozen=True)
class FreeroutingRunResult:
    original_board: BoardData
    routed_board: BoardData
    ripup_previews: list[FreeroutingRipupPreview]
    work_dir: Path
    dsn_path: Path
    ses_path: Path
    routed_board_path: Path
    ripup_payload_path: Path
    unique_ripped_net_count: int = 0
    ripup_event_count: int = 0
    cumulative_ripped_net_count: int = 0
    cumulative_ripped_item_count: int = 0
    cumulative_ripped_segment_count: int = 0
    still_unconnected: bool = False
    unconnected_item_count: int | None = None
    unconnected_report: str = ""
    attempted_profiles: tuple[str, ...] = ()
    selected_profile: str = "stable"
    profile_reports: tuple[dict[str, object], ...] = ()
    candidate_routed_boards: tuple[BoardData, ...] = ()
    candidate_routed_board_paths: tuple[Path, ...] = ()
    candidate_ripup_payload_paths: tuple[Path, ...] = ()
    stdout_log_path: Path | None = None
    stderr_log_path: Path | None = None


def run_freerouting_full(board_path: str | Path) -> FreeroutingRunResult:
    board_path = Path(board_path).resolve()
    app_root = Path(__file__).resolve().parents[2]
    workspace_root = app_root.parent
    freerouting_root = workspace_root / "freerouting"
    jar_path = freerouting_root / "build" / "libs" / "freerouting-current-executable.jar"
    if not jar_path.exists():
        raise RuntimeError(f"freerouting executable jar was not found: {jar_path}")

    work_dir = app_root / "out" / "freerouting_full" / board_path.stem
    work_dir.mkdir(parents=True, exist_ok=True)

    dsn_path = work_dir / "freerouting.dsn"
    ses_path = work_dir / "freerouting.ses"
    routed_board_path = work_dir / f"{board_path.stem}.freerouting.routed.kicad_pcb"
    ripup_payload_path = work_dir / "freerouting_ripped_routes.json"
    metadata_path = work_dir / "run_metadata.json"
    stdout_log_path = work_dir / "freerouting.stdout.log"
    stderr_log_path = work_dir / "freerouting.stderr.log"
    for artifact in (
        dsn_path,
        ses_path,
        routed_board_path,
        ripup_payload_path,
        metadata_path,
        stdout_log_path,
        stderr_log_path,
    ):
        if artifact.exists():
            artifact.unlink()

    home_dir = app_root / ".freerouting-home"
    home_dir.mkdir(parents=True, exist_ok=True)
    profile_paths = _ensure_freerouting_profiles(home_dir)

    _export_dsn(board_path, dsn_path)
    _force_dsn_snap_angle_fortyfive(dsn_path)
    stable_stage = _run_freerouting_stage(
        profile_name="stable",
        profile_config_path=profile_paths["stable"],
        jar_path=jar_path,
        board_path=board_path,
        dsn_path=dsn_path,
        ripup_payload_path=work_dir / "freerouting.stable_ripped_routes.json",
        ses_path=work_dir / "freerouting.stable.ses",
        routed_board_path=work_dir / f"{board_path.stem}.freerouting.stable.routed.kicad_pcb",
        app_root=app_root,
        stdout_log_path=work_dir / "freerouting.stable.stdout.log",
        stderr_log_path=work_dir / "freerouting.stable.stderr.log",
    )
    attempted_profiles = ["stable"]
    selected_stage = stable_stage
    stage_results = [stable_stage]
    if bool(stable_stage["run_report"]["still_unconnected"]):
        aggressive_stage = _run_freerouting_stage(
            profile_name="aggressive",
            profile_config_path=profile_paths["aggressive"],
            jar_path=jar_path,
            board_path=board_path,
            dsn_path=dsn_path,
            ripup_payload_path=work_dir / "freerouting.aggressive_ripped_routes.json",
            ses_path=work_dir / "freerouting.aggressive.ses",
            routed_board_path=work_dir / f"{board_path.stem}.freerouting.aggressive.routed.kicad_pcb",
            app_root=app_root,
            stdout_log_path=work_dir / "freerouting.aggressive.stdout.log",
            stderr_log_path=work_dir / "freerouting.aggressive.stderr.log",
        )
        attempted_profiles.append("aggressive")
        stage_results.append(aggressive_stage)
        if _is_stage_better(aggressive_stage, stable_stage):
            selected_stage = aggressive_stage

    _copy_stage_artifact(selected_stage["ses_path"], ses_path)
    _copy_stage_artifact(selected_stage["routed_board_path"], routed_board_path)
    _copy_stage_artifact(selected_stage["ripup_payload_path"], ripup_payload_path)
    _copy_stage_artifact(selected_stage["stdout_log_path"], stdout_log_path)
    _copy_stage_artifact(selected_stage["stderr_log_path"], stderr_log_path)
    run_report = dict(selected_stage["run_report"])

    original_board = load_board(board_path)
    routed_board = load_board(routed_board_path)
    candidate_routed_boards = tuple(
        load_board(Path(stage["routed_board_path"]))
        for stage in stage_results
    )
    candidate_routed_board_paths = tuple(Path(stage["routed_board_path"]) for stage in stage_results)
    candidate_ripup_payload_paths = tuple(Path(stage["ripup_payload_path"]) for stage in stage_results)
    # Feed all attempted freerouting profiles into the selector. The selected
    # stage still drives the left-top routed board, but Gurobi can choose from
    # every path occurrence generated by stable/aggressive attempts.
    ripup_previews = [
        preview
        for payload_path in candidate_ripup_payload_paths
        for preview in _load_ripup_previews(payload_path)
    ]

    payloads: list[dict[str, object]] = []
    for payload_source_path in candidate_ripup_payload_paths:
        try:
            payload = json.loads(payload_source_path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                payloads.append(payload)
        except Exception:
            continue
    ripup_event_count = sum(int(payload.get("ripup_event_count", 0)) for payload in payloads)
    cumulative_ripped_item_count = sum(int(payload.get("cumulative_ripped_item_count", 0)) for payload in payloads)
    cumulative_ripped_segment_count = sum(int(payload.get("cumulative_ripped_segment_count", 0)) for payload in payloads)
    cumulative_ripped_net_count = 0
    unique_ripped_net_count = 0
    if payloads:
        unique_ripped_net_count = len({
            int(item.get("net_id", 0))
            for payload in payloads
            for item in payload.get("nets", [])
            if isinstance(item, dict)
        })
        for payload in payloads:
            for item in payload.get("nets", []):
                if not isinstance(item, dict):
                    continue
                try:
                    cumulative_ripped_net_count += int(item.get("ripup_count", 0))
                except Exception:
                    continue
    if unique_ripped_net_count <= 0:
        unique_ripped_net_count = len({item.net_id for item in ripup_previews})

    stage_reports = [
        {
            "profile": stable_stage["profile_name"],
            "still_unconnected": stable_stage["run_report"]["still_unconnected"],
            "unconnected_item_count": stable_stage["run_report"]["unconnected_item_count"],
            "stdout_log": str(stable_stage["stdout_log_path"]),
            "stderr_log": str(stable_stage["stderr_log_path"]),
        },
        *(
            [
                {
                    "profile": aggressive_stage["profile_name"],
                    "still_unconnected": aggressive_stage["run_report"]["still_unconnected"],
                    "unconnected_item_count": aggressive_stage["run_report"]["unconnected_item_count"],
                    "stdout_log": str(aggressive_stage["stdout_log_path"]),
                    "stderr_log": str(aggressive_stage["stderr_log_path"]),
                }
            ]
            if "aggressive_stage" in locals()
            else []
        ),
    ]

    metadata = {
        "board": str(board_path),
        "dsn": str(dsn_path),
        "ses": str(ses_path),
        "routed_board": str(routed_board_path),
        "ripup_payload": str(ripup_payload_path),
        "candidate_routed_boards": [str(path) for path in candidate_routed_board_paths],
        "candidate_ripup_payloads": [str(path) for path in candidate_ripup_payload_paths],
        "ripup_net_count": len(ripup_previews),
        "unique_ripped_net_count": unique_ripped_net_count,
        "ripup_segment_count": sum(len(item.segments) for item in ripup_previews),
        "ripup_via_count": sum(len(item.vias) for item in ripup_previews),
        "ripup_event_count": ripup_event_count,
        "cumulative_ripped_net_count": cumulative_ripped_net_count,
        "cumulative_ripped_item_count": cumulative_ripped_item_count,
        "cumulative_ripped_segment_count": cumulative_ripped_segment_count,
        "still_unconnected": run_report["still_unconnected"],
        "unconnected_item_count": run_report["unconnected_item_count"],
        "unconnected_report": run_report["unconnected_report"],
        "attempted_profiles": attempted_profiles,
        "selected_profile": selected_stage["profile_name"],
        "stage_reports": stage_reports,
        "stdout_log": str(stdout_log_path),
        "stderr_log": str(stderr_log_path),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    return FreeroutingRunResult(
        original_board=original_board,
        routed_board=routed_board,
        ripup_previews=ripup_previews,
        work_dir=work_dir,
        dsn_path=dsn_path,
        ses_path=ses_path,
        routed_board_path=routed_board_path,
        ripup_payload_path=ripup_payload_path,
        unique_ripped_net_count=unique_ripped_net_count,
        ripup_event_count=ripup_event_count,
        cumulative_ripped_net_count=cumulative_ripped_net_count,
        cumulative_ripped_item_count=cumulative_ripped_item_count,
        cumulative_ripped_segment_count=cumulative_ripped_segment_count,
        still_unconnected=bool(run_report["still_unconnected"]),
        unconnected_item_count=run_report["unconnected_item_count"],
        unconnected_report=str(run_report["unconnected_report"]),
        attempted_profiles=tuple(attempted_profiles),
        selected_profile=str(selected_stage["profile_name"]),
        profile_reports=tuple(stage_reports),
        candidate_routed_boards=candidate_routed_boards,
        candidate_routed_board_paths=candidate_routed_board_paths,
        candidate_ripup_payload_paths=candidate_ripup_payload_paths,
        stdout_log_path=stdout_log_path,
        stderr_log_path=stderr_log_path,
    )


def _export_dsn(board_path: Path, dsn_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    if board is None:
        raise RuntimeError(f"pcbnew failed to load board for DSN export: {board_path}")
    ok = pcbnew.ExportSpecctraDSN(board, str(dsn_path))
    if not ok or not dsn_path.exists():
        raise RuntimeError(f"pcbnew failed to export Specctra DSN: {dsn_path}")


def _force_dsn_snap_angle_fortyfive(dsn_path: Path) -> None:
    if not dsn_path.exists():
        raise RuntimeError(f"DSN file was not found for snap-angle patch: {dsn_path}")
    text = dsn_path.read_text(encoding="utf-8")

    snap_scope_pattern = re.compile(r"\(snap_angle\b[^)]*\)", re.IGNORECASE | re.DOTALL)
    patched_text, count = snap_scope_pattern.subn("(snap_angle fortyfive_degree)", text, count=1)
    if count == 0:
        structure_pattern = re.compile(r"(\(\s*structure\b)", re.IGNORECASE)
        patched_text, inserted = structure_pattern.subn(
            r"\1\n    (snap_angle fortyfive_degree)",
            text,
            count=1,
        )
        if inserted == 0:
            raise RuntimeError("Could not find '(structure ...)' scope in DSN to set snap_angle.")
    dsn_path.write_text(patched_text, encoding="utf-8")


def _import_ses(board_path: Path, ses_path: Path, routed_board_path: Path) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    if board is None:
        raise RuntimeError(f"pcbnew failed to load board for SES import: {board_path}")
    ok = pcbnew.ImportSpecctraSES(board, str(ses_path))
    if not ok:
        raise RuntimeError(f"pcbnew failed to import freerouting SES: {ses_path}")
    if not pcbnew.SaveBoard(str(routed_board_path), board):
        raise RuntimeError(f"pcbnew failed to save routed board: {routed_board_path}")


def _run_freerouting(
    jar_path: Path,
    dsn_path: Path,
    ses_path: Path,
    ripup_payload_path: Path,
    app_root: Path,
    profile_config_path: Path,
    stdout_log_path: Path,
    stderr_log_path: Path,
) -> dict[str, object]:
    java_exe = _resolve_java_executable()
    home_dir = app_root / ".freerouting-home"
    home_dir.mkdir(parents=True, exist_ok=True)
    active_config_path = home_dir / "freerouting.json"
    shutil.copyfile(profile_config_path, active_config_path)

    command = [
        str(java_exe),
        f"-Duser.home={home_dir}",
        f"-Dfreerouting.ripup.output={ripup_payload_path}",
        "-jar",
        str(jar_path),
        "--gui.enabled=false",
        "--api_server.authentication.enabled=false",
        "--feature_flags.multi_threading=false",
        f"--user_data_path={home_dir}",
        "-da",
        "-host",
        "Interactive-Router",
        "-de",
        str(dsn_path),
        "-do",
        str(ses_path),
    ]
    env = os.environ.copy()
    env["FREEROUTING__USER_DATA_PATH"] = str(home_dir)
    completed = subprocess.run(
        command,
        cwd=str(jar_path.parents[2]),
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )
    stdout_log_path.write_text(completed.stdout or "", encoding="utf-8")
    stderr_log_path.write_text(completed.stderr or "", encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(
            "freerouting failed.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    if not ses_path.exists():
        raise RuntimeError(f"freerouting completed without producing SES output: {ses_path}")
    return _analyze_freerouting_output(completed.stdout or "", completed.stderr or "")


def _run_freerouting_stage(
    profile_name: str,
    profile_config_path: Path,
    jar_path: Path,
    board_path: Path,
    dsn_path: Path,
    ripup_payload_path: Path,
    ses_path: Path,
    routed_board_path: Path,
    app_root: Path,
    stdout_log_path: Path,
    stderr_log_path: Path,
) -> dict[str, object]:
    for artifact in (ripup_payload_path, ses_path, routed_board_path, stdout_log_path, stderr_log_path):
        if artifact.exists():
            artifact.unlink()
    run_report = _run_freerouting(
        jar_path=jar_path,
        dsn_path=dsn_path,
        ses_path=ses_path,
        ripup_payload_path=ripup_payload_path,
        app_root=app_root,
        profile_config_path=profile_config_path,
        stdout_log_path=stdout_log_path,
        stderr_log_path=stderr_log_path,
    )
    _import_ses(board_path, ses_path, routed_board_path)
    return {
        "profile_name": profile_name,
        "run_report": run_report,
        "ripup_payload_path": ripup_payload_path,
        "ses_path": ses_path,
        "routed_board_path": routed_board_path,
        "stdout_log_path": stdout_log_path,
        "stderr_log_path": stderr_log_path,
    }


def _copy_stage_artifact(source: Path, destination: Path) -> None:
    if source.resolve() == destination.resolve():
        return
    shutil.copyfile(source, destination)


def _unconnected_count(stage_result: dict[str, object]) -> int:
    report = stage_result["run_report"]
    count = report.get("unconnected_item_count")
    if isinstance(count, int):
        return count
    return 0 if not report.get("still_unconnected") else 10**9


def _is_stage_better(candidate_stage: dict[str, object], baseline_stage: dict[str, object]) -> bool:
    candidate_count = _unconnected_count(candidate_stage)
    baseline_count = _unconnected_count(baseline_stage)
    return candidate_count < baseline_count


def _ensure_freerouting_profiles(home_dir: Path) -> dict[str, Path]:
    stable_path = home_dir / "freerouting.stable.json"
    aggressive_path = home_dir / "freerouting.aggressive.json"
    if not stable_path.exists():
        stable_path.write_text(json.dumps(_default_freerouting_profile(home_dir, "stable"), indent=2), encoding="utf-8")
    if not aggressive_path.exists():
        aggressive_path.write_text(json.dumps(_default_freerouting_profile(home_dir, "aggressive"), indent=2), encoding="utf-8")
    return {
        "stable": stable_path,
        "aggressive": aggressive_path,
    }


def _default_freerouting_profile(home_dir: Path, profile_name: str) -> dict[str, object]:
    base = {
        "profile": {
            "id": "b53e464f-9207-4f46-a274-87a45acad687",
            "email": "",
            "allow_telemetry": True,
            "allow_contact": True,
        },
        "gui": {
            "enabled": True,
            "input_directory": "",
            "dialog_confirmation_timeout": 5,
        },
        "router": {
            "algorithm": "freerouting-router",
            "job_timeout": "12:00:00",
            "max_passes": 9999,
            "stop_at_pass_minimum": 20,
            "stagnation_pass_limit": 40,
            "stagnation_score_threshold": 0.1,
            "optimizer": {},
            "scoring": {
                "default_preferred_direction_trace_cost": 1.0,
                "default_undesired_direction_trace_cost": 1.05,
                "via_costs": 20,
                "start_ripup_costs": 20,
            },
        },
        "drc": {
            "include_warnings": True,
            "include_errors": True,
        },
        "usage_and_diagnostic_data": {
            "disable_analytics": False,
        },
        "feature_flags": {
            "multi_threading": True,
            "inspection_mode": False,
            "other_menu": False,
            "save_jobs": False,
        },
        "api_server": {
            "enabled": False,
            "http_allowed": True,
            "endpoints": [
                "http://127.0.0.1:37864",
            ],
            "authentication": {
                "enabled": True,
                "providers": "",
                "google_sheets": {},
            },
            "cors_origins": "",
        },
        "statistics": {
            "start_time": "2026-05-13 08:58:14",
            "sessions_total": 0,
            "jobs_started": 0,
            "jobs_completed": 0,
        },
        "logging": {
            "console": {
                "enabled": True,
                "level": "INFO",
            },
            "file": {
                "enabled": False,
                "level": "INFO",
                "location": str(home_dir / "freerouting.log"),
            },
        },
        "version": "2.2.2",
    }
    if profile_name == "aggressive":
        router = base["router"]
        assert isinstance(router, dict)
        router["stagnation_pass_limit"] = 80
        router["stagnation_score_threshold"] = 0.05
        scoring = router["scoring"]
        assert isinstance(scoring, dict)
        scoring["default_undesired_direction_trace_cost"] = 1.01
        scoring["via_costs"] = 8
        scoring["start_ripup_costs"] = 10
    return base


def _analyze_freerouting_output(stdout_text: str, stderr_text: str) -> dict[str, object]:
    combined = "\n".join(part for part in (stdout_text, stderr_text) if part)
    still_unconnected = False
    unconnected_item_count: int | None = None

    still_unconnected_match = re.search(
        r"(\d+)\s+item(?:s)?\s+still\s+unconnected",
        combined,
        flags=re.IGNORECASE,
    )
    if still_unconnected_match:
        still_unconnected = True
        unconnected_item_count = int(still_unconnected_match.group(1))

    connections_not_found_match = re.search(
        r"autoroute\s+completed,\s*(\d+)\s+connections\s+not\s+found",
        combined,
        flags=re.IGNORECASE,
    )
    if connections_not_found_match:
        still_unconnected = int(connections_not_found_match.group(1)) > 0
        if unconnected_item_count is None:
            unconnected_item_count = int(connections_not_found_match.group(1))

    report = ""
    report_marker = "The following connections could not be routed"
    marker_index = combined.find(report_marker)
    if marker_index >= 0:
        report = combined[marker_index:].strip()
        next_section = re.search(r"\n(?:INFO|WARN|ERROR|DEBUG|TRACE)\b", report)
        if next_section:
            report = report[: next_section.start()].strip()

    return {
        "still_unconnected": still_unconnected,
        "unconnected_item_count": unconnected_item_count,
        "unconnected_report": report,
    }


def _resolve_java_executable() -> Path:
    candidates: list[Path] = []
    for root in (
        Path(r"C:\Program Files\Eclipse Adoptium"),
        Path(r"C:\Program Files\Java"),
    ):
        if not root.exists():
            continue
        preferred = sorted(root.glob("jdk-25*"), reverse=True)
        fallback = sorted(root.glob("jdk-*"), reverse=True)
        for child in [*preferred, *fallback]:
            candidates.append(child / "bin" / "java.exe")
            candidates.append(child / "bin" / "java")
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidates.append(Path(java_home) / "bin" / "java.exe")
        candidates.append(Path(java_home) / "bin" / "java")
    which_java = shutil.which("java")
    if which_java:
        candidates.append(Path(which_java))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise RuntimeError("Java executable was not found. Set JAVA_HOME or add java to PATH.")


def _load_ripup_previews(payload_path: Path) -> list[FreeroutingRipupPreview]:
    if not payload_path.exists():
        return []
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    previews: list[FreeroutingRipupPreview] = []
    for item in payload.get("nets", []):
        try:
            net_id = int(item.get("net_id", 0))
        except Exception:
            continue
        net_name = str(item.get("net_name", ""))
        occurrences = item.get("occurrences", [])
        if isinstance(occurrences, list) and occurrences:
            for occurrence in occurrences:
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
                previews.append(
                    FreeroutingRipupPreview(
                        net_id=net_id,
                        net_name=net_name,
                        segments=segments,
                        vias=vias,
                        truncated=bool(occurrence.get("truncated", False)),
                        occurrence_index=int(occurrence.get("event_index", 0)),
                        source_net_id=source_net_id,
                    )
                )
            continue

        segments = _parse_segments(item.get("segments", []))
        vias = _parse_vias(item.get("vias", []))
        if not segments and not vias:
            continue
        previews.append(
            FreeroutingRipupPreview(
                net_id=net_id,
                net_name=net_name,
                segments=segments,
                vias=vias,
                truncated=bool(item.get("truncated", False)),
            )
        )
    previews.sort(key=lambda item: (item.occurrence_index, item.net_id))
    return previews


def _parse_segments(raw_segments) -> list[FreeroutingSegment]:
    if not isinstance(raw_segments, list):
        return []
    return [
        FreeroutingSegment(
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


def _parse_vias(raw_vias) -> list[FreeroutingVia]:
    if not isinstance(raw_vias, list):
        return []
    return [
        FreeroutingVia(
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
