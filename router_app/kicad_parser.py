from __future__ import annotations

import os
import re
import sys
import tempfile
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
from typing import Any, Iterable


@dataclass(frozen=True)
class TrackSegment:
    start: tuple[float, float]
    end: tuple[float, float]
    width: float
    layer: str
    net_id: int
    net_name: str


@dataclass(frozen=True)
class Pad:
    name: str
    center: tuple[float, float]
    size: tuple[float, float]
    shape: str
    layers: tuple[str, ...]
    net_id: int | None
    net_name: str


@dataclass(frozen=True)
class Footprint:
    name: str
    reference: str
    position: tuple[float, float]
    layer: str
    pads: list[Pad]


@dataclass(frozen=True)
class BoardData:
    path: Path
    nets: dict[int, str]
    tracks: list[TrackSegment]
    footprints: list[Footprint]
    backend: str = "sexpr"
    design_rules: dict[str, float] | None = None

    @property
    def copper_layers(self) -> list[str]:
        layers = {track.layer for track in self.tracks}
        if not layers:
            for footprint in self.footprints:
                if footprint.layer.endswith(".Cu"):
                    layers.add(footprint.layer)
                for pad in footprint.pads:
                    layers.update(
                        layer
                        for layer in pad.layers
                        if layer in {"F.Cu", "B.Cu"} or layer.startswith("In")
                    )
        return sorted(layers, key=_layer_sort_key)

    @property
    def two_pin_net_ids(self) -> set[int]:
        pad_counts: dict[int, int] = {}
        for footprint in self.footprints:
            for pad in footprint.pads:
                if pad.net_id is not None:
                    pad_counts[pad.net_id] = pad_counts.get(pad.net_id, 0) + 1
        return {net_id for net_id, count in pad_counts.items() if count == 2}


def load_board(path: str | Path) -> BoardData:
    try:
        return _load_board_with_pcbnew(path)
    except ImportError:
        return _load_board_with_sexpr(path)


def _load_board_with_pcbnew(path: str | Path) -> BoardData:
    pcbnew = _import_pcbnew()
    board_path = Path(path)
    load_path = _normalized_legacy_board_path(board_path) if _needs_normalized_pcbnew_load(board_path) else board_path
    try:
        board = pcbnew.LoadBoard(str(load_path))
    finally:
        if load_path != board_path:
            try:
                load_path.unlink()
            except OSError:
                pass
    nets = _pcbnew_nets(board)
    tracks = _pcbnew_tracks(board, pcbnew, nets)
    footprints = _pcbnew_footprints(board, pcbnew, nets)

    return BoardData(
        path=board_path,
        nets=nets,
        tracks=tracks,
        footprints=footprints,
        backend="pcbnew-normalized" if load_path != board_path else "pcbnew",
        design_rules=_pcbnew_design_rules(board, pcbnew),
    )


def _load_board_with_sexpr(path: str | Path) -> BoardData:
    board_path = Path(path)
    text = board_path.read_text(encoding="utf-8")
    sexpr = _parse_sexpr(text)
    nets: dict[int, str] = {}
    tracks: list[TrackSegment] = []
    footprints: list[Footprint] = []

    for node in _walk_lists(sexpr):
        if not node:
            continue

        kind = node[0]
        if kind == "net" and len(node) >= 3:
            net_id = _to_int(node[1])
            if net_id is not None:
                nets[net_id] = str(node[2])

        if kind == "segment":
            segment = _parse_segment(node, nets)
            if segment is not None:
                tracks.append(segment)

    for node in _top_level_nodes(sexpr, "footprint"):
        footprint = _parse_footprint(node, nets)
        if footprint is not None:
            footprints.append(footprint)
    for node in _top_level_nodes(sexpr, "module"):
        footprint = _parse_footprint(node, nets)
        if footprint is not None:
            footprints.append(footprint)

    return BoardData(path=board_path, nets=nets, tracks=tracks, footprints=footprints, backend="sexpr")


def _uses_legacy_layer_names(path: str | Path) -> bool:
    try:
        head = Path(path).read_text(encoding="utf-8", errors="ignore")[:12000]
    except OSError:
        return False
    return "(0 Top signal)" in head or "(31 Bottom signal)" in head


def _needs_normalized_pcbnew_load(path: str | Path) -> bool:
    try:
        text = Path(path).read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    return (
        "(0 Top signal)" in text[:12000]
        or "(31 Bottom signal)" in text[:12000]
        or _legacy_version_number(text) < 20200000
        or _has_non_copper_segments(text)
    )


def _normalized_legacy_board_path(path: Path) -> Path:
    text = path.read_text(encoding="utf-8", errors="ignore")
    replacements = {
        "(0 Top signal)": "(0 F.Cu signal)",
        "(31 Bottom signal)": "(31 B.Cu signal)",
        "(layer Top)": "(layer F.Cu)",
        "(layer Bottom)": "(layer B.Cu)",
        "(layers Top)": "(layers F.Cu)",
        "(layers Bottom)": "(layers B.Cu)",
        " Top)": " F.Cu)",
        " Bottom)": " B.Cu)",
        " Top ": " F.Cu ",
        " Bottom ": " B.Cu ",
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    text = _remove_legacy_zone_fill_setting(text)
    text = _remove_non_copper_segments(text)

    handle = tempfile.NamedTemporaryFile(
        "w",
        delete=False,
        encoding="utf-8",
        suffix=".kicad_pcb",
        prefix=f"{path.stem}.normalized.",
    )
    with handle:
        handle.write(text)
    return Path(handle.name)


def _legacy_version_number(text: str) -> int:
    match = re.search(r"\(version\s+(\d+)\)", text[:200])
    if not match:
        return 99999999
    return int(match.group(1))


def _has_non_copper_segments(text: str) -> bool:
    for segment in _top_level_segment_strings(text):
        layer = _segment_layer(segment)
        if layer is not None and not _is_copper_layer(layer):
            return True
    return False


def _remove_non_copper_segments(text: str) -> str:
    pieces: list[str] = []
    cursor = 0
    for start, end, segment in _top_level_segment_ranges(text):
        layer = _segment_layer(segment)
        if layer is not None and not _is_copper_layer(layer):
            pieces.append(text[cursor:start])
            cursor = end
    pieces.append(text[cursor:])
    return "".join(pieces)


def _top_level_segment_strings(text: str) -> Iterable[str]:
    for _, _, segment in _top_level_segment_ranges(text):
        yield segment


def _top_level_segment_ranges(text: str) -> Iterable[tuple[int, int, str]]:
    pattern = re.compile(r"(?m)^\s{2}\(segment\b")
    for match in pattern.finditer(text):
        start = match.start()
        depth = 0
        in_string = False
        escaped = False
        for index in range(start, len(text)):
            char = text[index]
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
                continue
            if char == '"':
                in_string = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    end = index + 1
                    yield start, end, text[start:end]
                    break


def _segment_layer(segment: str) -> str | None:
    match = re.search(r"\(layer\s+([^\s\)]+)\)", segment)
    return match.group(1) if match else None


def _is_copper_layer(layer: str) -> bool:
    return layer in {"F.Cu", "B.Cu", "Top", "Bottom"} or bool(re.fullmatch(r"In\d+\.Cu", layer))


def _remove_legacy_zone_fill_setting(text: str) -> str:
    return re.sub(r"(?m)^\s*\(zone_45_only\s+(yes|no)\)\s*\r?\n", "", text)


def _import_pcbnew():
    try:
        import pcbnew

        return pcbnew
    except ImportError as first_error:
        for candidate in _candidate_kicad_python_paths():
            if str(candidate) not in sys.path and candidate.exists():
                sys.path.insert(0, str(candidate))
            _add_dll_directory_if_needed(candidate)
        try:
            import pcbnew

            return pcbnew
        except ImportError:
            raise first_error


def _candidate_kicad_python_paths() -> Iterable[Path]:
    env_path = os.environ.get("KICAD_PCBNEW_PATH")
    if env_path:
        yield Path(env_path)

    local_app_data = os.environ.get("LOCALAPPDATA")
    candidate_roots = []
    if local_app_data:
        candidate_roots.append(Path(local_app_data) / "Programs" / "KiCad")
    candidate_roots.extend(
        [
            Path.home() / "AppData" / "Local" / "Programs" / "KiCad",
            Path("C:/Program Files/KiCad"),
            Path("C:/Program Files (x86)/KiCad"),
        ]
    )

    for root in candidate_roots:
        if not _path_exists(root):
            continue
        for version_dir in sorted(_safe_iterdir(root), reverse=True):
            yield version_dir / "bin"
            yield version_dir / "bin" / "Lib" / "site-packages"
            yield version_dir / "lib" / "python3" / "dist-packages"


def _add_dll_directory_if_needed(candidate: Path) -> None:
    if not hasattr(os, "add_dll_directory"):
        return

    possible_bins = [candidate]
    if candidate.name == "site-packages":
        possible_bins.append(candidate.parents[1])
    if candidate.name == "dist-packages":
        possible_bins.append(candidate.parents[2] / "bin")

    for path in possible_bins:
        if path.name.lower() != "bin" or not path.exists():
            continue
        try:
            os.add_dll_directory(str(path))
        except OSError:
            pass


def _path_exists(path: Path) -> bool:
    try:
        return path.exists()
    except OSError:
        return False


def _safe_iterdir(path: Path) -> list[Path]:
    try:
        return list(path.iterdir())
    except OSError:
        return []


def _pcbnew_nets(board: Any) -> dict[int, str]:
    nets: dict[int, str] = {}
    if hasattr(board, "GetNetsByNetcode"):
        for net_code, net_info in board.GetNetsByNetcode().items():
            nets[int(net_code)] = _call_or_default(net_info, "GetNetname", str(net_info))
    return nets


def _pcbnew_tracks(board: Any, pcbnew: Any, nets: dict[int, str]) -> list[TrackSegment]:
    tracks: list[TrackSegment] = []
    for item in board.GetTracks():
        if _pcbnew_is_via(item, pcbnew) or not hasattr(item, "GetEnd"):
            continue

        net_id = int(_call_or_default(item, "GetNetCode", 0))
        layer = _pcbnew_item_layer(board, item)
        tracks.append(
            TrackSegment(
                start=_pcbnew_point_to_mm(item.GetStart(), pcbnew),
                end=_pcbnew_point_to_mm(item.GetEnd(), pcbnew),
                width=_pcbnew_to_mm(item.GetWidth(), pcbnew),
                layer=layer,
                net_id=net_id,
                net_name=_call_or_default(item, "GetNetname", nets.get(net_id, f"Net {net_id}")),
            )
        )
    return tracks


def _pcbnew_footprints(board: Any, pcbnew: Any, nets: dict[int, str]) -> list[Footprint]:
    footprints: list[Footprint] = []
    for footprint in _pcbnew_board_footprints(board):
        layer = _pcbnew_item_layer(board, footprint)
        position = _pcbnew_point_to_mm(footprint.GetPosition(), pcbnew)
        pads = [
            _pcbnew_pad(board, pcbnew, pad, nets)
            for pad in _pcbnew_footprint_pads(footprint)
        ]
        footprints.append(
            Footprint(
                name=_call_or_default(footprint, "GetFPIDAsString", _call_or_default(footprint, "GetValue", "")),
                reference=_call_or_default(footprint, "GetReference", ""),
                position=position,
                layer=layer,
                pads=[pad for pad in pads if pad is not None],
            )
        )
    return footprints


def _pcbnew_pad(board: Any, pcbnew: Any, pad: Any, nets: dict[int, str]) -> Pad | None:
    net_id = int(_call_or_default(pad, "GetNetCode", 0))
    size = pad.GetSize()
    layers = _pcbnew_pad_layers(board, pad)
    return Pad(
        name=str(_call_or_default(pad, "GetName", _call_or_default(pad, "GetPadName", ""))),
        center=_pcbnew_point_to_mm(pad.GetPosition(), pcbnew),
        size=(_pcbnew_to_mm(_axis_value(size, "x"), pcbnew), _pcbnew_to_mm(_axis_value(size, "y"), pcbnew)),
        shape=_pcbnew_pad_shape_name(pad, pcbnew),
        layers=layers,
        net_id=net_id,
        net_name=_call_or_default(pad, "GetNetname", nets.get(net_id, f"Net {net_id}")),
    )


def _pcbnew_design_rules(board: Any, pcbnew: Any) -> dict[str, float]:
    settings = _call_or_default(board, "GetDesignSettings", None)
    if settings is None:
        return {}

    rules: dict[str, float] = {}
    for key, method_name in {
        "track_width": "GetCurrentTrackWidth",
        "via_diameter": "GetCurrentViaSize",
        "via_drill": "GetCurrentViaDrill",
        "min_clearance": "GetSmallestClearanceValue",
    }.items():
        value = _call_or_default(settings, method_name, None)
        if isinstance(value, (int, float)):
            rules[key] = _pcbnew_to_mm(value, pcbnew)
    return rules


def _pcbnew_board_footprints(board: Any) -> Iterable[Any]:
    if hasattr(board, "GetFootprints"):
        return board.GetFootprints()
    if hasattr(board, "GetModules"):
        return board.GetModules()
    return []


def _pcbnew_footprint_pads(footprint: Any) -> Iterable[Any]:
    if hasattr(footprint, "Pads"):
        return footprint.Pads()
    if hasattr(footprint, "GetPads"):
        return footprint.GetPads()
    return []


def _pcbnew_pad_layers(board: Any, pad: Any) -> tuple[str, ...]:
    layer_set = _call_or_default(pad, "GetLayerSet", None)
    if layer_set is not None and hasattr(layer_set, "Seq"):
        return tuple(_canonical_layer(board.GetLayerName(layer_id)) for layer_id in layer_set.Seq())

    layers: list[str] = []
    for layer_id in _pcbnew_copper_layer_ids(board):
        if bool(_call_or_default(pad, "IsOnLayer", False, layer_id)):
            layers.append(_canonical_layer(board.GetLayerName(layer_id)))
    return tuple(layers)


def _pcbnew_copper_layer_ids(board: Any) -> Iterable[int]:
    count = int(_call_or_default(board, "GetCopperLayerCount", 0))
    if count:
        return range(count)
    return range(32)


def _pcbnew_item_layer(board: Any, item: Any) -> str:
    layer_id = _call_or_default(item, "GetLayer", None)
    if layer_id is None:
        return "unknown"
    return _canonical_layer(board.GetLayerName(layer_id))


def _pcbnew_point_to_mm(point: Any, pcbnew: Any) -> tuple[float, float]:
    return (_pcbnew_to_mm(_axis_value(point, "x"), pcbnew), _pcbnew_to_mm(_axis_value(point, "y"), pcbnew))


def _pcbnew_to_mm(value: Any, pcbnew: Any) -> float:
    if hasattr(pcbnew, "ToMM"):
        return float(pcbnew.ToMM(value))
    return float(value) / 1_000_000.0


def _axis_value(value: Any, axis: str) -> int | float:
    upper = axis.upper()
    if hasattr(value, axis):
        return getattr(value, axis)
    if hasattr(value, upper):
        attr = getattr(value, upper)
        return attr() if callable(attr) else attr
    raise AttributeError(f"Object has no {axis}/{upper} coordinate")


def _pcbnew_is_via(item: Any, pcbnew: Any) -> bool:
    via_class = getattr(pcbnew, "PCB_VIA", None)
    if via_class is not None and isinstance(item, via_class):
        return True
    return "VIA" in item.__class__.__name__.upper()


def _pcbnew_pad_shape_name(pad: Any, pcbnew: Any) -> str:
    shape = _call_or_default(pad, "GetShape", None)
    mapping = {
        getattr(pcbnew, "PAD_SHAPE_CIRCLE", object()): "circle",
        getattr(pcbnew, "PAD_SHAPE_OVAL", object()): "oval",
        getattr(pcbnew, "PAD_SHAPE_RECT", object()): "rect",
        getattr(pcbnew, "PAD_SHAPE_ROUNDRECT", object()): "roundrect",
        getattr(pcbnew, "PAD_SHAPE_TRAPEZOID", object()): "trapezoid",
    }
    return mapping.get(shape, str(shape).lower())


def _call_or_default(obj: Any, method_name: str, default: Any, *args: Any) -> Any:
    if obj is None or not hasattr(obj, method_name):
        return default
    try:
        return getattr(obj, method_name)(*args)
    except TypeError:
        return default


def _parse_segment(node: list[Any], nets: dict[int, str]) -> TrackSegment | None:
    start = _pair(_child(node, "start"))
    end = _pair(_child(node, "end"))
    width = _number(_child_value(node, "width"), default=0.2)
    layer = _canonical_layer(_child_value(node, "layer") or "unknown")
    net_id = _to_int(_child_value(node, "net"))

    if start is None or end is None or net_id is None:
        return None

    return TrackSegment(
        start=start,
        end=end,
        width=width,
        layer=layer,
        net_id=net_id,
        net_name=nets.get(net_id, f"Net {net_id}"),
    )


def _parse_footprint(node: list[Any], nets: dict[int, str]) -> Footprint | None:
    if len(node) < 2:
        return None

    name = str(node[1])
    layer = _canonical_layer(_child_value(node, "layer") or "unknown")
    at = _child(node, "at")
    x = _number(at[1]) if at is not None and len(at) >= 3 else 0.0
    y = _number(at[2]) if at is not None and len(at) >= 3 else 0.0
    angle = _number(at[3]) if at is not None and len(at) >= 4 else 0.0
    reference = _footprint_reference(node) or name
    pads: list[Pad] = []

    for child in node[1:]:
        if isinstance(child, list) and child and child[0] == "pad":
            pad = _parse_pad(child, (x, y), angle, nets)
            if pad is not None:
                pads.append(pad)

    return Footprint(name=name, reference=reference, position=(x, y), layer=layer, pads=pads)


def _parse_pad(
    node: list[Any],
    footprint_position: tuple[float, float],
    footprint_angle: float,
    nets: dict[int, str],
) -> Pad | None:
    if len(node) < 4:
        return None

    name = str(node[1])
    shape = str(node[3])
    at = _child(node, "at")
    local_x = _number(at[1]) if at is not None and len(at) >= 3 else 0.0
    local_y = _number(at[2]) if at is not None and len(at) >= 3 else 0.0
    pad_angle = _number(at[3]) if at is not None and len(at) >= 4 else 0.0
    center = _transform_point((local_x, local_y), footprint_position, footprint_angle)
    size = _pair(_child(node, "size")) or (0.8, 0.8)
    layers_node = _child(node, "layers")
    layers = (
        tuple(_canonical_layer(layer) for layer in layers_node[1:])
        if layers_node is not None
        else tuple()
    )
    net_id = _to_int(_child_value(node, "net"))

    return Pad(
        name=name,
        center=center,
        size=size,
        shape=shape,
        layers=layers,
        net_id=net_id,
        net_name=nets.get(net_id, f"Net {net_id}") if net_id is not None else "",
    )


def _footprint_reference(node: list[Any]) -> str | None:
    for child in node[1:]:
        if not isinstance(child, list) or len(child) < 3:
            continue
        if child[0] in {"property", "fp_text"} and child[1] in {"Reference", "reference"}:
            return str(child[2])
    return None


def _transform_point(
    point: tuple[float, float],
    offset: tuple[float, float],
    angle_degrees: float,
) -> tuple[float, float]:
    angle = radians(angle_degrees)
    x, y = point
    return (
        offset[0] + x * cos(angle) - y * sin(angle),
        offset[1] + x * sin(angle) + y * cos(angle),
    )


def _tokenize(text: str) -> list[str]:
    tokens: list[str] = []
    i = 0
    while i < len(text):
        char = text[i]
        if char.isspace():
            i += 1
            continue
        if char in "()":
            tokens.append(char)
            i += 1
            continue
        if char == '"':
            i += 1
            value: list[str] = []
            while i < len(text):
                if text[i] == "\\" and i + 1 < len(text):
                    value.append(text[i + 1])
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                value.append(text[i])
                i += 1
            tokens.append("".join(value))
            continue

        start = i
        while i < len(text) and not text[i].isspace() and text[i] not in "()":
            i += 1
        tokens.append(text[start:i])
    return tokens


def _parse_sexpr(text: str) -> list[Any]:
    tokens = _tokenize(text)
    stack: list[list[Any]] = [[]]
    for token in tokens:
        if token == "(":
            child: list[Any] = []
            stack[-1].append(child)
            stack.append(child)
        elif token == ")":
            if len(stack) == 1:
                raise ValueError("Unexpected ')' in KiCad PCB file")
            stack.pop()
        else:
            stack[-1].append(token)
    if len(stack) != 1:
        raise ValueError("Unclosed '(' in KiCad PCB file")
    return stack[0]


def _walk_lists(node: Any) -> Iterable[list[Any]]:
    if isinstance(node, list):
        yield node
        for child in node:
            yield from _walk_lists(child)


def _top_level_nodes(sexpr: list[Any], name: str) -> Iterable[list[Any]]:
    roots = sexpr
    if len(sexpr) == 1 and isinstance(sexpr[0], list):
        roots = sexpr[0]
    for child in roots[1:]:
        if isinstance(child, list) and child and child[0] == name:
            yield child


def _child(node: list[Any], name: str) -> list[Any] | None:
    for child in node[1:]:
        if isinstance(child, list) and child and child[0] == name:
            return child
    return None


def _child_value(node: list[Any], name: str) -> Any:
    child = _child(node, name)
    if child is None or len(child) < 2:
        return None
    return child[1]


def _pair(node: list[Any] | None) -> tuple[float, float] | None:
    if node is None or len(node) < 3:
        return None
    return (_number(node[1]), _number(node[2]))


def _number(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _layer_sort_key(layer: str) -> tuple[int, str]:
    if layer == "F.Cu":
        return (0, layer)
    if layer == "B.Cu":
        return (99, layer)
    if layer.startswith("In") and layer.endswith(".Cu"):
        number = _to_int(layer[2:-3])
        return (number or 1, layer)
    return (50, layer)


def _canonical_layer(layer: Any) -> str:
    value = str(layer)
    old_to_new = {
        "Top": "F.Cu",
        "Front": "F.Cu",
        "Bottom": "B.Cu",
        "Back": "B.Cu",
        "*.Cu": "*.Cu",
    }
    return old_to_new.get(value, value)
