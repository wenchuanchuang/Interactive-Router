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
    for artifact in (dsn_path, ses_path, routed_board_path, ripup_payload_path, metadata_path):
        if artifact.exists():
            artifact.unlink()

    _export_dsn(board_path, dsn_path)
    _force_dsn_snap_angle_fortyfive(dsn_path)
    _run_freerouting(jar_path, dsn_path, ses_path, ripup_payload_path, app_root)
    _import_ses(board_path, ses_path, routed_board_path)

    original_board = load_board(board_path)
    routed_board = load_board(routed_board_path)
    ripup_previews = _load_ripup_previews(ripup_payload_path)

    payload = {}
    if ripup_payload_path.exists():
        try:
            payload = json.loads(ripup_payload_path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
    ripup_event_count = int(payload.get("ripup_event_count", 0)) if isinstance(payload, dict) else 0
    cumulative_ripped_item_count = int(payload.get("cumulative_ripped_item_count", 0)) if isinstance(payload, dict) else 0
    cumulative_ripped_segment_count = int(payload.get("cumulative_ripped_segment_count", 0)) if isinstance(payload, dict) else 0
    cumulative_ripped_net_count = 0
    unique_ripped_net_count = 0
    if isinstance(payload, dict):
        unique_ripped_net_count = len(payload.get("nets", []))
        for item in payload.get("nets", []):
            try:
                cumulative_ripped_net_count += int(item.get("ripup_count", 0))
            except Exception:
                continue
    if unique_ripped_net_count <= 0:
        unique_ripped_net_count = len({item.net_id for item in ripup_previews})

    metadata = {
        "board": str(board_path),
        "dsn": str(dsn_path),
        "ses": str(ses_path),
        "routed_board": str(routed_board_path),
        "ripup_payload": str(ripup_payload_path),
        "ripup_net_count": len(ripup_previews),
        "unique_ripped_net_count": unique_ripped_net_count,
        "ripup_segment_count": sum(len(item.segments) for item in ripup_previews),
        "ripup_via_count": sum(len(item.vias) for item in ripup_previews),
        "ripup_event_count": ripup_event_count,
        "cumulative_ripped_net_count": cumulative_ripped_net_count,
        "cumulative_ripped_item_count": cumulative_ripped_item_count,
        "cumulative_ripped_segment_count": cumulative_ripped_segment_count,
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
) -> None:
    java_exe = _resolve_java_executable()
    home_dir = app_root / ".freerouting-home"
    home_dir.mkdir(parents=True, exist_ok=True)

    command = [
        str(java_exe),
        f"-Duser.home={home_dir}",
        f"-Dfreerouting.ripup.output={ripup_payload_path}",
        "-jar",
        str(jar_path),
        "--gui.enabled=false",
        "--api_server.authentication.enabled=false",
        "--feature_flags.multi_threading=false",
        "-da",
        "-host",
        "Interactive-Router",
        "-de",
        str(dsn_path),
        "-do",
        str(ses_path),
    ]
    completed = subprocess.run(
        command,
        cwd=str(jar_path.parents[2]),
        check=False,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "freerouting failed.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    if not ses_path.exists():
        raise RuntimeError(f"freerouting completed without producing SES output: {ses_path}")


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
