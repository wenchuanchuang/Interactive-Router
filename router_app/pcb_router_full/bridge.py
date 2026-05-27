from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Iterable

from router_app.kicad_parser import BoardData, TrackSegment, Via, load_board


PCBROUTER_GRID_SCALE = 10 # default is 10


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
class PcbRouterRunResult:
    original_board: BoardData
    original_candidate_board: BoardData | None
    routed_board: BoardData
    ripup_previews: list[PcbRouterRipupPreview]
    work_dir: Path
    routed_board_path: Path
    ripup_payload_path: Path | None
    unique_ripped_net_count: int = 0
    ripup_event_count: int = 0
    stdout_log_path: Path | None = None
    stderr_log_path: Path | None = None
    failed_net_ids_estimated: tuple[int, ...] = ()


def run_pcb_router_full(board_path: str | Path) -> PcbRouterRunResult:
    """Run PcbRouter as an Interactive-Router backend and collect its artifacts."""
    board_path = Path(board_path).resolve()
    router_source_board_path = _canonical_router_source_board(board_path)
    app_root = Path(__file__).resolve().parents[2]
    work_dir = app_root / "out" / "pcb_router_full" / board_path.stem
    work_dir.mkdir(parents=True, exist_ok=True)

    routed_board_path = work_dir / f"{board_path.stem}.pcbrouter.routed.kicad_pcb"
    ripup_payload_path = work_dir / "pcbrouter_ripped_routes.json"
    stdout_log_path = work_dir / "pcbrouter.stdout.log"
    stderr_log_path = work_dir / "pcbrouter.stderr.log"
    pcbrouter_input_path = work_dir / f"{board_path.stem}.kicad_pcb"
    for artifact in (routed_board_path, ripup_payload_path, stdout_log_path, stderr_log_path, pcbrouter_input_path):
        if artifact.exists():
            artifact.unlink()

    output_dir = work_dir / "output"
    if output_dir.exists():
        shutil.rmtree(output_dir)
    # PcbRouter writes its routed board and ripup payload to relative output/log folders.
    # Create them before launch because the Windows build reports an error instead of
    # creating missing directories in some code paths.
    output_dir.mkdir(parents=True, exist_ok=True)
    (work_dir / "log").mkdir(parents=True, exist_ok=True)

    pcbrouter_input_path = _prepare_pcbrouter_input_board(router_source_board_path, pcbrouter_input_path)
    exe_path = _find_pcbrouter_executable(app_root)
    # Pass the grid scale explicitly so PcbRouter uses 0.01 mm cells
    # (gridFactor = 1 / 100) without rebuilding the executable.
    pcbrouter_command = [str(exe_path), str(pcbrouter_input_path), str(PCBROUTER_GRID_SCALE)]
    print(f"pcbrouter_grid_scale = {PCBROUTER_GRID_SCALE}", flush=True)
    print(f"pcbrouter_grid_pitch_mm = {1.0 / PCBROUTER_GRID_SCALE:.5f}", flush=True)
    with stdout_log_path.open("w", encoding="utf-8", errors="replace") as stdout_log, stderr_log_path.open(
        "w", encoding="utf-8", errors="replace"
    ) as stderr_log:
        process = subprocess.run(
            pcbrouter_command,
            cwd=str(work_dir),
            stdout=stdout_log,
            stderr=stderr_log,
            text=True,
            check=False,
        )
    if process.returncode != 0:
        raise RuntimeError(
            "PcbRouter failed with exit code "
            f"{process.returncode}. stdout={stdout_log_path.resolve()} stderr={stderr_log_path.resolve()}"
        )

    produced_board = _find_pcbrouter_routed_board(output_dir, board_path.stem)
    if produced_board is None:
        raise RuntimeError(
            f"PcbRouter completed but no routed board was found under {output_dir.resolve()}. "
            f"stdout={stdout_log_path.resolve()} stderr={stderr_log_path.resolve()}"
        )
    shutil.copyfile(produced_board, routed_board_path)

    produced_payload = output_dir / "pcbrouter_ripped_routes.json"
    final_payload_path: Path | None = None
    if produced_payload.exists():
        shutil.copyfile(produced_payload, ripup_payload_path)
        final_payload_path = ripup_payload_path

    original_board, original_candidate_board = _load_selector_original_board(
        input_board_path=board_path,
        router_source_board_path=router_source_board_path,
    )
    routed_board = load_board(routed_board_path)
    ripup_previews = _load_ripup_previews(final_payload_path) if final_payload_path is not None else []
    ripup_event_count = _read_ripup_event_count(final_payload_path)
    failed_net_ids_estimated = tuple(_estimate_failed_nets(routed_board))

    return PcbRouterRunResult(
        original_board=original_board,
        original_candidate_board=original_candidate_board,
        routed_board=routed_board,
        ripup_previews=ripup_previews,
        work_dir=work_dir,
        routed_board_path=routed_board_path,
        ripup_payload_path=final_payload_path,
        unique_ripped_net_count=len({item.net_id for item in ripup_previews}),
        ripup_event_count=ripup_event_count,
        stdout_log_path=stdout_log_path.resolve(),
        stderr_log_path=stderr_log_path.resolve(),
        failed_net_ids_estimated=failed_net_ids_estimated,
    )


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


def _canonical_router_source_board(board_path: Path) -> Path:
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
        dx, dy = _board_translation_from_common_pads(source_board=sibling, target_board=board_path)
    except Exception:
        return board_path
    print(f"pcbrouter_router_source_board = {sibling.resolve()}", flush=True)
    print(f"pcbrouter_input_to_source_translation_mm = dx={dx:.6f} dy={dy:.6f}", flush=True)
    print("pcbrouter_router_input_strategy = sibling_unrouted_coordinate_frame", flush=True)
    return sibling


def _load_selector_original_board(input_board_path: Path, router_source_board_path: Path) -> tuple[BoardData, BoardData | None]:
    """Load selector data in router coordinates while preserving input routes.

    Selector geometry must share the external router's coordinate frame. For a
    translated routed input, the selector board remains the sibling unrouted
    board so routed tracks do not change grid bounds. Existing tracks and vias
    are shifted back separately and exposed as original candidate geometry.
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


def _board_translation_from_common_pads(source_board: Path, target_board: Path) -> tuple[float, float]:
    """Calculate a constant source-to-target board translation from common pads.

    PcbRouter must route in the same coordinate frame as the displayed board.
    When a sibling board provides only the missing outline, common footprint/pad
    centers give a robust way to move that outline into the target board frame.
    """
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


def _load_ripup_previews(payload_path: Path | None) -> list[PcbRouterRipupPreview]:
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
