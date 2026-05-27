from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Iterable

import pcbnew

from router_app.kicad_parser import BoardData, TrackSegment, Via, load_board


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
    original_candidate_board: BoardData | None
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
    router_source_board_path = _canonical_router_source_board(board_path)
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
    unrouted_input_board_path = work_dir / f"{board_path.stem}.freerouting.input_unrouted.kicad_pcb"
    routed_board_path = work_dir / f"{board_path.stem}.freerouting.routed.kicad_pcb"
    ripup_payload_path = work_dir / "freerouting_ripped_routes.json"
    metadata_path = work_dir / "run_metadata.json"
    stdout_log_path = work_dir / "freerouting.stdout.log"
    stderr_log_path = work_dir / "freerouting.stderr.log"
    for artifact in (
        dsn_path,
        ses_path,
        unrouted_input_board_path,
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

    _write_board_without_tracks_and_vias(router_source_board_path, unrouted_input_board_path)
    _export_dsn(unrouted_input_board_path, dsn_path)
    _normalize_dsn_pcb_name(dsn_path, router_source_board_path.stem)
    _force_dsn_snap_angle_fortyfive(dsn_path)
    stable_stage = _run_freerouting_stage(
        profile_name="stable",
        profile_config_path=profile_paths["stable"],
        jar_path=jar_path,
        board_path=unrouted_input_board_path,
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
            board_path=unrouted_input_board_path,
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

    original_board, original_candidate_board = _load_selector_original_board(
        input_board_path=board_path,
        router_source_board_path=router_source_board_path,
    )
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
        "router_source_board": str(router_source_board_path),
        "unrouted_input_board": str(unrouted_input_board_path),
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
        original_candidate_board=original_candidate_board,
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


def _normalize_dsn_pcb_name(dsn_path: Path, pcb_name: str) -> None:
    """Make DSN identity stable across equivalent temporary file paths.

    KiCad writes the output DSN path into the top-level `(pcb "...")` form.
    Two byte-identical input boards exported from different work directories
    would otherwise produce different DSN files, which can perturb downstream
    router metadata and deterministic comparisons.
    """
    text = dsn_path.read_text(encoding="utf-8")
    normalized = re.sub(r'\A\(pcb\s+"[^"]*"', f'(pcb "{pcb_name}"', text, count=1)
    if normalized != text:
        dsn_path.write_text(normalized, encoding="utf-8")


def _canonical_router_source_board(board_path: Path) -> Path:
    """Choose the board coordinate frame used by external routers.

    Routed reference boards can be simple translations of their sibling
    unrouted input. Running a heuristic router in the translated coordinate
    frame can change rounding and tie-breaking, so external routers use the
    sibling unrouted board when it is an exact translated match. The routed
    input geometry is still kept later as a shifted selector candidate.
    """
    sibling = _find_sibling_unrouted_board(board_path)
    if sibling is None:
        return board_path
    try:
        dx, dy = _board_translation_from_common_pads(source_board=sibling, target_board=board_path)
    except Exception:
        return board_path
    print(f"freerouting_router_source_board = {sibling.resolve()}", flush=True)
    print(f"freerouting_input_to_source_translation_mm = dx={dx:.6f} dy={dy:.6f}", flush=True)
    print("freerouting_router_input_strategy = sibling_unrouted_coordinate_frame", flush=True)
    return sibling


def _load_selector_original_board(input_board_path: Path, router_source_board_path: Path) -> tuple[BoardData, BoardData | None]:
    """Load selector baseline data and preserve shifted routed input geometry.

    The selector must live in the same coordinate frame as the external router
    output. When the user opened a translated routed reference board, keep the
    selector board itself equal to the sibling unrouted board so routed tracks do
    not change grid bounds. The shifted routed geometry is returned separately
    as an original candidate source.
    """
    if input_board_path == router_source_board_path:
        return load_board(input_board_path), None

    source_board = load_board(router_source_board_path)
    input_board = load_board(input_board_path)
    dx, dy = _board_translation_from_common_pads(source_board=router_source_board_path, target_board=input_board_path)
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
        "freerouting_original_candidate_translation_mm = "
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


def _write_board_without_tracks_and_vias(board_path: Path, output_path: Path) -> None:
    """Save a temporary board copy with all routed track/via items removed.

    The input board is still loaded later as the selector's original board, so
    existing routed geometry remains available as an "original" candidate. This
    temporary copy is only used as freerouting's DSN/SES base board, matching
    PcbRouter's behavior of routing from a cleared board. Some routed reference
    boards omit Edge.Cuts, so missing outlines are recovered from a matching
    sibling unrouted board and translated into this board's coordinate frame.
    """
    text = board_path.read_text(encoding="utf-8")
    text = _remove_top_level_board_items(text, {"segment", "via"})
    if not _extract_top_level_edge_cut_items(text):
        sibling = _find_sibling_unrouted_board(board_path)
        if sibling is None:
            raise RuntimeError(
                "freerouting input board has no Edge.Cuts, and no matching sibling "
                f"unrouted board was found for outline recovery: {board_path}"
            )
        sibling_text = sibling.read_text(encoding="utf-8")
        edge_cut_items = _extract_top_level_edge_cut_items(sibling_text)
        if not edge_cut_items:
            raise RuntimeError(
                "freerouting input board has no Edge.Cuts, and the matching sibling "
                f"board also has no Edge.Cuts: input={board_path} sibling={sibling}"
            )
        dx, dy = _board_translation_from_common_pads(source_board=sibling, target_board=board_path)
        translated_edge_cut_items = [_translate_edge_cut_item(item, dx, dy) for item in edge_cut_items]
        text = _insert_top_level_items_before_board_end(text, translated_edge_cut_items)
        print(f"freerouting_edge_cuts_source = {sibling.resolve()}", flush=True)
        print(f"freerouting_edge_cuts_translation_mm = dx={dx:.6f} dy={dy:.6f}", flush=True)
        print("freerouting_input_strategy = translated_edge_cuts", flush=True)
    # Keep the legacy general section consistent for tools that still read it.
    text = re.sub(r"(?m)(^\s*\(tracks\s+)\d+(\)\s*$)", r"\g<1>0\2", text, count=1)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text, encoding="utf-8")


def _find_sibling_unrouted_board(board_path: Path) -> Path | None:
    """Find a likely unrouted board next to a routed reference board."""
    candidates: list[Path] = []
    stem = board_path.stem
    if stem.endswith(".routed"):
        candidates.append(board_path.with_name(stem[: -len(".routed")] + ".unrouted" + board_path.suffix))
    if ".routed." in stem:
        # Paper/result files may be named like
        # `bm2.unrouted.routed.ours.bestSolutionWithMerging.kicad_pcb`;
        # the canonical external-router input is the prefix before `.routed`.
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


def _board_translation_from_common_pads(source_board: Path, target_board: Path) -> tuple[float, float]:
    """Calculate a constant source-to-target board translation from common pads."""
    source_pads = _pad_centers_by_key(load_board(source_board))
    target_pads = _pad_centers_by_key(load_board(target_board))
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


def _pad_centers_by_key(board: BoardData) -> dict[tuple[str, str], tuple[float, float]]:
    """Index pad centers by footprint reference and pad name."""
    centers: dict[tuple[str, str], tuple[float, float]] = {}
    for footprint in board.footprints:
        for pad in footprint.pads:
            centers[(footprint.reference, pad.name)] = pad.center
    return centers


def _median(values: Iterable[float]) -> float:
    """Return the median value for translation estimation."""
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


def _remove_top_level_board_items(text: str, item_names: set[str]) -> str:
    """Remove top-level KiCad board forms whose symbol is in item_names.

    Tracks and vias are top-level `(segment ...)` / `(via ...)` forms in KiCad
    PCB files. Footprint-local graphics remain untouched because they are nested
    deeper than the board root.
    """
    ranges: list[tuple[int, int]] = []
    depth = 0
    index = 0
    while index < len(text):
        char = text[index]
        if char == '"':
            index = _skip_quoted_string(text, index + 1)
            continue
        if char == "(":
            if depth == 1:
                symbol = _sexpr_symbol_at(text, index + 1)
                if symbol in item_names:
                    end = _matching_paren_end(text, index)
                    if end is not None:
                        ranges.append((index, end))
                        index = end
                        continue
            depth += 1
        elif char == ")":
            depth = max(0, depth - 1)
        index += 1

    if not ranges:
        return text
    pieces: list[str] = []
    previous = 0
    for start, end in ranges:
        pieces.append(text[previous:start])
        previous = end
    pieces.append(text[previous:])
    return "".join(pieces)


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


def _sexpr_symbol_at(text: str, index: int) -> str:
    """Read the first S-expression symbol after an opening parenthesis."""
    while index < len(text) and text[index].isspace():
        index += 1
    start = index
    while index < len(text) and not text[index].isspace() and text[index] not in "()":
        index += 1
    return text[start:index]


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


def _sanitize_ses_for_kicad_import(ses_path: Path) -> int:
    """Remove SES placement blocks that KiCad's importer rejects.

    Freerouting may emit an empty component-name block for mounting-hole style
    items, for example ``(component`` followed directly by ``(place @HOLE0 ...)``.
    KiCad's SES importer returns False on that block even though it does not
    carry routed copper.  Removing only these empty component blocks preserves
    all routed network data while allowing the imported board to be created.
    """
    text = ses_path.read_text(encoding="utf-8", errors="replace")
    empty_component_pattern = re.compile(
        r"(?ms)^[ \t]*\(component[ \t]*\r?\n"
        r"(?:[ \t]*\(place[^\r\n]*\)[ \t]*\r?\n)+"
        r"[ \t]*\)[ \t]*(?:\r?\n)?"
    )
    sanitized_text, removed_count = empty_component_pattern.subn("", text)
    if removed_count > 0:
        ses_path.write_text(sanitized_text, encoding="utf-8")
    return removed_count


def _import_ses(board_path: Path, ses_path: Path, routed_board_path: Path) -> None:
    removed_empty_components = _sanitize_ses_for_kicad_import(ses_path)
    if removed_empty_components:
        print(f"freerouting_ses_empty_component_blocks_removed = {removed_empty_components}")
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
                # Freerouting component exports can be stored under the net that
                # triggered the ripup, while the component itself belongs to
                # source_net_id. Use the component owner for selector membership.
                effective_net_id = source_net_id if source_net_id is not None and source_net_id > 0 else net_id
                previews.append(
                    FreeroutingRipupPreview(
                        net_id=effective_net_id,
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
