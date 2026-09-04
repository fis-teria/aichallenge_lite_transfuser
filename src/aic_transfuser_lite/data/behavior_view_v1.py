from __future__ import annotations

from bisect import bisect_right
from dataclasses import asdict, dataclass
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Iterable, Sequence

from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_ONTOLOGY_V1,
    BEHAVIOR_SIDE_NAMES_V1,
    BehaviorClassV1,
    BehaviorSideV1,
)

from .storage_v3 import validate_complete_dataset


BEHAVIOR_VIEW_FORMAT_V1 = "aic_behavior_view_v1"
BEHAVIOR_LABEL_SOURCES_V1 = frozenset(
    {"mpc_expert_autoware_log", "recovery_reference_phase"}
)
_LOG_TIME = re.compile(r"\[INFO\]\s+\[([0-9]+(?:\.[0-9]+)?)\]")
_FIELD = re.compile(r"(?:^|\s)([a-zA-Z][a-zA-Z0-9_]*)=([^\s]+)")
_REQUIRED_FIELDS = frozenset(
    {"loop_seq", "avoid_state", "avoid_vehicle", "brake_vehicle", "avoid_candidate", "recovery_state"}
)


@dataclass(frozen=True)
class BagStateEventV1:
    timestamp_ns: int
    state: str


@dataclass(frozen=True)
class BehaviorDiagnosticV1:
    wall_time_sec: float
    sim_time_ns: int
    loop_seq: int
    behavior_class: int
    behavior_side: int
    side_valid: bool
    authority: str
    target_vehicle: str | None


@dataclass(frozen=True)
class BehaviorAnnotationV1:
    sample_id: str
    run_id: str
    grid_stamp_ns: int
    behavior_class: int
    behavior_label: str
    behavior_side: int
    behavior_side_label: str
    behavior_valid: bool
    behavior_side_valid: bool
    quality: float
    source_stamp_ns: int | None
    source_age_ms: float | None
    source: str | None
    authority: str | None
    target_vehicle: str | None
    invalid_reason: str | None


def parse_speed_diagnostics_v1(
    lines: Iterable[str], *, wall_to_sim_offset_sec: float
) -> tuple[tuple[BehaviorDiagnosticV1, ...], int]:
    """Parse compatible speed.diag rows and reject older partial formats."""

    diagnostics: list[BehaviorDiagnosticV1] = []
    rejected = 0
    for line in lines:
        if "[speed.diag]" not in line:
            continue
        time_match = _LOG_TIME.search(line)
        fields = dict(_FIELD.findall(line))
        if time_match is None or not _REQUIRED_FIELDS.issubset(fields):
            rejected += 1
            continue
        try:
            wall_time = float(time_match.group(1))
            loop_seq = int(fields["loop_seq"])
            behavior, side, side_valid, target = _classify_fields(fields)
            sim_sec = wall_time - wall_to_sim_offset_sec
            if not math.isfinite(sim_sec) or sim_sec < 0.0:
                raise ValueError("invalid aligned simulation time")
        except (TypeError, ValueError):
            rejected += 1
            continue
        diagnostics.append(
            BehaviorDiagnosticV1(
                wall_time_sec=wall_time,
                sim_time_ns=int(round(sim_sec * 1e9)),
                loop_seq=loop_seq,
                behavior_class=int(behavior),
                behavior_side=int(side),
                side_valid=side_valid,
                authority=fields.get("control_authority", fields.get("steering_mode", "UNKNOWN")),
                target_vehicle=target,
            )
        )
    diagnostics.sort(key=lambda item: (item.sim_time_ns, item.loop_seq))
    deduplicated = {item.sim_time_ns: item for item in diagnostics}
    return tuple(deduplicated[key] for key in sorted(deduplicated)), rejected


def align_behavior_annotations_v1(
    sample_rows: Sequence[dict[str, str]],
    diagnostics: Sequence[BehaviorDiagnosticV1],
    *,
    run_id: str,
    active_interval_ns: tuple[int, int],
    max_gap_ms: float = 500.0,
) -> tuple[BehaviorAnnotationV1, ...]:
    """Attach only labels bracketed by two compatible, unchanged diagnostics."""

    if max_gap_ms <= 0.0 or not math.isfinite(max_gap_ms):
        raise ValueError("max_gap_ms must be finite and positive")
    start_ns, end_ns = active_interval_ns
    if start_ns < 0 or end_ns <= start_ns:
        raise ValueError("active behavior interval must satisfy 0 <= start < end")
    stamps = [item.sim_time_ns for item in diagnostics]
    output = []
    for row in sample_rows:
        if row["run_id"] != run_id:
            continue
        sample_id = row["sample_id"]
        stamp = int(row["grid_stamp_ns"])
        invalid_reason: str | None = None
        previous = following = None
        if stamp < start_ns or stamp > end_ns:
            invalid_reason = "outside_active_interval"
        else:
            right = bisect_right(stamps, stamp)
            if right == 0 or right >= len(diagnostics):
                invalid_reason = "diagnostic_not_bracketed"
            else:
                previous, following = diagnostics[right - 1], diagnostics[right]
                gap_ms = (following.sim_time_ns - previous.sim_time_ns) / 1e6
                if gap_ms > max_gap_ms:
                    invalid_reason = "diagnostic_gap"
                elif (
                    previous.behavior_class != following.behavior_class
                    or previous.behavior_side != following.behavior_side
                    or previous.side_valid != following.side_valid
                ):
                    invalid_reason = "transition_uncertain"
        if invalid_reason is not None or previous is None or following is None:
            output.append(_invalid_annotation(sample_id, run_id, stamp, invalid_reason or "unknown"))
            continue
        source_age_ms = (stamp - previous.sim_time_ns) / 1e6
        span_ms = (following.sim_time_ns - previous.sim_time_ns) / 1e6
        quality = max(0.0, min(1.0, 1.0 - source_age_ms / max(max_gap_ms, span_ms)))
        output.append(
            BehaviorAnnotationV1(
                sample_id=sample_id,
                run_id=run_id,
                grid_stamp_ns=stamp,
                behavior_class=previous.behavior_class,
                behavior_label=BEHAVIOR_CLASS_NAMES_V1[previous.behavior_class],
                behavior_side=previous.behavior_side,
                behavior_side_label=BEHAVIOR_SIDE_NAMES_V1[previous.behavior_side],
                behavior_valid=True,
                behavior_side_valid=previous.side_valid,
                quality=quality,
                source_stamp_ns=previous.sim_time_ns,
                source_age_ms=source_age_ms,
                source="mpc_expert_autoware_log",
                authority=previous.authority,
                target_vehicle=previous.target_vehicle,
                invalid_reason=None,
            )
        )
    return tuple(output)


def read_awsim_state_events_v1(bag_directory: str | Path) -> tuple[BagStateEventV1, ...]:
    from rosbags.highlevel import AnyReader

    bag = Path(bag_directory)
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"rosbag2 metadata not found: {bag / 'metadata.yaml'}")
    result = []
    with AnyReader([bag]) as reader:
        connections = [item for item in reader.connections if item.topic == "/awsim/state"]
        if len(connections) != 1 or connections[0].msgtype != "std_msgs/msg/String":
            raise ValueError("bag requires exactly one /awsim/state std_msgs/msg/String stream")
        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            result.append(BagStateEventV1(int(timestamp_ns), str(message.data)))
    if not result:
        raise ValueError("bag /awsim/state stream is empty")
    return tuple(result)


def behavior_alignment_anchors_v1(
    log_lines: Sequence[str], state_events: Sequence[BagStateEventV1]
) -> tuple[float, tuple[int, int]]:
    grounded_wall = None
    for line in log_lines:
        if "start condition met" in line and "/awsim/state == Grounded" in line:
            match = _LOG_TIME.search(line)
            if match is not None:
                grounded_wall = float(match.group(1))
                break
    grounded = next((item for item in state_events if item.state == "Grounded"), None)
    if grounded_wall is None or grounded is None:
        raise ValueError("Grounded wall/simulation alignment anchor is unavailable")
    ready_index = next((index for index, item in enumerate(state_events) if item.state == "Ready"), None)
    if ready_index is None:
        raise ValueError("/awsim/state has no Ready event")
    race_start = next((item for item in state_events[ready_index + 1 :] if item.state == "Start"), None)
    if race_start is None:
        raise ValueError("/awsim/state has no Start after Ready")
    race_finish = next(
        (item for item in state_events if item.state == "Finish" and item.timestamp_ns > race_start.timestamp_ns),
        None,
    )
    if race_finish is None:
        raise ValueError("/awsim/state has no Finish after race Start")
    offset = grounded_wall - grounded.timestamp_ns / 1e9
    return offset, (race_start.timestamp_ns, race_finish.timestamp_ns)


def build_behavior_view_v1(
    *,
    dataset_root: str | Path,
    run_sources: Sequence[tuple[str, str | Path, str | Path]],
    output_root: str | Path,
    max_gap_ms: float = 500.0,
) -> dict[str, object]:
    root = Path(dataset_root)
    dataset_manifest = validate_complete_dataset(root)
    output = Path(output_root)
    if output.exists():
        raise FileExistsError(f"behavior view output already exists: {output}")
    with (root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        sample_rows = list(csv.DictReader(stream))
    known_runs = {row["run_id"] for row in sample_rows}
    seen: set[str] = set()
    annotations: list[BehaviorAnnotationV1] = []
    sources = []
    for run_id, log_path_raw, bag_path_raw in run_sources:
        if run_id in seen:
            raise ValueError(f"duplicate behavior run source: {run_id}")
        if run_id not in known_runs:
            raise ValueError(f"behavior run {run_id!r} is absent from Dataset V3")
        seen.add(run_id)
        log_path, bag_path = Path(log_path_raw), Path(bag_path_raw)
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        states = read_awsim_state_events_v1(bag_path)
        offset, interval = behavior_alignment_anchors_v1(lines, states)
        diagnostics, rejected = parse_speed_diagnostics_v1(lines, wall_to_sim_offset_sec=offset)
        if len(diagnostics) < 2:
            raise ValueError(f"run {run_id!r} has fewer than two compatible speed diagnostics")
        run_annotations = align_behavior_annotations_v1(
            sample_rows, diagnostics, run_id=run_id, active_interval_ns=interval, max_gap_ms=max_gap_ms
        )
        annotations.extend(run_annotations)
        sources.append(
            {
                "run_id": run_id,
                "autoware_log": str(log_path.resolve()),
                "autoware_log_sha256": _sha256(log_path),
                "bag_directory": str(bag_path.resolve()),
                "bag_metadata_sha256": _sha256(bag_path / "metadata.yaml"),
                "bag_storage_sha256": _directory_sha256(bag_path),
                "wall_to_sim_offset_sec": offset,
                "active_interval_ns": list(interval),
                "compatible_diagnostic_count": len(diagnostics),
                "rejected_diagnostic_count": rejected,
            }
        )
    output.mkdir(parents=True)
    labels_path = output / "behavior_labels.csv"
    with labels_path.open("w", newline="", encoding="utf-8") as stream:
        fields = list(BehaviorAnnotationV1.__dataclass_fields__)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in annotations:
            writer.writerow(asdict(item))
    payload: dict[str, object] = {
        "format": BEHAVIOR_VIEW_FORMAT_V1,
        "ontology": BEHAVIOR_ONTOLOGY_V1,
        "class_names": list(BEHAVIOR_CLASS_NAMES_V1),
        "side_names": list(BEHAVIOR_SIDE_NAMES_V1),
        "dataset_manifest_sha256": dataset_manifest["manifest_sha256"],
        "max_gap_ms": max_gap_ms,
        "labels_sha256": _sha256(labels_path),
        "sources": sources,
        "sample_count": len(annotations),
        "valid_behavior_count": sum(item.behavior_valid for item in annotations),
        "valid_side_count": sum(item.behavior_side_valid for item in annotations),
    }
    manifest_text = json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    (output / "manifest.json").write_text(manifest_text, encoding="utf-8")
    return payload


def load_behavior_view_v1(
    behavior_view_root: str | Path, *, dataset_manifest_sha256: str
) -> dict[str, dict[str, str]]:
    root = Path(behavior_view_root)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    expected = {
        "format": BEHAVIOR_VIEW_FORMAT_V1,
        "ontology": BEHAVIOR_ONTOLOGY_V1,
        "class_names": list(BEHAVIOR_CLASS_NAMES_V1),
        "side_names": list(BEHAVIOR_SIDE_NAMES_V1),
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise ValueError(f"behavior view {key} mismatch")
    if manifest.get("dataset_manifest_sha256") != dataset_manifest_sha256:
        raise ValueError("behavior view targets a different Dataset V3 manifest")
    labels_path = root / "behavior_labels.csv"
    if _sha256(labels_path) != manifest.get("labels_sha256"):
        raise ValueError("behavior labels SHA-256 mismatch")
    with labels_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(BehaviorAnnotationV1.__dataclass_fields__):
            raise ValueError("behavior labels CSV fields mismatch")
        rows = list(reader)
    for row in rows:
        _validate_behavior_row_v1(row)
    by_sample = {row["sample_id"]: row for row in rows}
    if len(by_sample) != len(rows):
        raise ValueError("behavior view contains duplicate sample IDs")
    expected_counts = {
        "sample_count": len(rows),
        "valid_behavior_count": sum(_strict_bool(row["behavior_valid"]) for row in rows),
        "valid_side_count": sum(_strict_bool(row["behavior_side_valid"]) for row in rows),
    }
    if any(manifest.get(key) != value for key, value in expected_counts.items()):
        raise ValueError("behavior view manifest counts mismatch")
    return by_sample


def merge_behavior_views_v1(
    *,
    dataset_root: str | Path,
    source_view_roots: Sequence[str | Path],
    output_root: str | Path,
) -> dict[str, object]:
    """Merge disjoint Behavior V1 views and retarget them to one Dataset V3."""

    if not source_view_roots:
        raise ValueError("at least one behavior view is required")
    dataset = Path(dataset_root)
    dataset_manifest = validate_complete_dataset(dataset)
    with (dataset / "samples.csv").open(newline="", encoding="utf-8") as stream:
        sample_rows = list(csv.DictReader(stream))
    expected_ids = {row["sample_id"] for row in sample_rows}
    if len(expected_ids) != len(sample_rows):
        raise ValueError("target Dataset V3 contains duplicate sample IDs")
    merged: dict[str, dict[str, str]] = {}
    sources = []
    for source_raw in source_view_roots:
        source = Path(source_raw)
        manifest_path = source / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        source_dataset_sha = str(manifest.get("dataset_manifest_sha256", ""))
        rows = load_behavior_view_v1(
            source,
            dataset_manifest_sha256=source_dataset_sha,
        )
        overlap = set(merged).intersection(rows)
        if overlap:
            raise ValueError(f"behavior views overlap sample IDs: {sorted(overlap)[:3]}")
        merged.update(rows)
        sources.append(
            {
                "view": str(source.resolve()),
                "manifest_sha256": _sha256(manifest_path),
                "dataset_manifest_sha256": source_dataset_sha,
                "sample_count": len(rows),
            }
        )
    missing = expected_ids - set(merged)
    extra = set(merged) - expected_ids
    if missing or extra:
        raise ValueError(
            f"merged behavior coverage mismatch: missing={len(missing)}, extra={len(extra)}"
        )
    output = Path(output_root)
    if output.exists():
        raise FileExistsError(f"merged behavior view output already exists: {output}")
    output.mkdir(parents=True)
    labels_path = output / "behavior_labels.csv"
    fields = list(BehaviorAnnotationV1.__dataclass_fields__)
    with labels_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample_id in sorted(merged):
            writer.writerow(merged[sample_id])
    payload: dict[str, object] = {
        "format": BEHAVIOR_VIEW_FORMAT_V1,
        "ontology": BEHAVIOR_ONTOLOGY_V1,
        "class_names": list(BEHAVIOR_CLASS_NAMES_V1),
        "side_names": list(BEHAVIOR_SIDE_NAMES_V1),
        "dataset_manifest_sha256": dataset_manifest["manifest_sha256"],
        "labels_sha256": _sha256(labels_path),
        "sources": sources,
        "sample_count": len(merged),
        "valid_behavior_count": sum(
            _strict_bool(row["behavior_valid"]) for row in merged.values()
        ),
        "valid_side_count": sum(
            _strict_bool(row["behavior_side_valid"]) for row in merged.values()
        ),
    }
    (output / "manifest.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return payload


def _classify_fields(
    fields: dict[str, str]
) -> tuple[BehaviorClassV1, BehaviorSideV1, bool, str | None]:
    recovery = fields["recovery_state"]
    avoid = fields["avoid_state"]
    brake = fields["brake_vehicle"]
    if recovery != "MONITORING":
        behavior = BehaviorClassV1.RECOVERY
        target = None
    elif avoid == "AVOIDING":
        behavior = BehaviorClassV1.FORWARD_AVOID
        target = _vehicle(fields["avoid_vehicle"])
    elif avoid == "RETURNING":
        behavior = BehaviorClassV1.FORWARD_RETURN
        target = _vehicle(fields["avoid_vehicle"])
    elif brake != "none":
        behavior = BehaviorClassV1.FORWARD_FOLLOW
        target = _vehicle(brake)
    elif avoid == "IDLE":
        behavior = BehaviorClassV1.FORWARD_NORMAL
        target = None
    else:
        raise ValueError(f"unknown avoid state: {avoid}")
    candidate = fields["avoid_candidate"]
    side = (
        BehaviorSideV1.LEFT
        if candidate.startswith("LEFT_")
        else BehaviorSideV1.RIGHT
        if candidate.startswith("RIGHT_")
        else BehaviorSideV1.NONE
    )
    directional = behavior in (BehaviorClassV1.FORWARD_AVOID, BehaviorClassV1.FORWARD_RETURN)
    side_valid = not directional or side is not BehaviorSideV1.NONE
    if not directional:
        side = BehaviorSideV1.NONE
    return behavior, side, side_valid, target


def _vehicle(value: str) -> str | None:
    return None if value == "none" else value


def _invalid_annotation(sample_id: str, run_id: str, stamp: int, reason: str) -> BehaviorAnnotationV1:
    return BehaviorAnnotationV1(
        sample_id, run_id, stamp, -1, "UNKNOWN", -1, "UNKNOWN", False, False,
        0.0, None, None, None, None, None, reason,
    )


def _validate_behavior_row_v1(row: dict[str, str]) -> None:
    if not row["sample_id"].strip() or not row["run_id"].strip():
        raise ValueError("behavior row sample_id and run_id must be non-empty")
    behavior_valid = _strict_bool(row["behavior_valid"])
    side_valid = _strict_bool(row["behavior_side_valid"])
    if side_valid and not behavior_valid:
        raise ValueError("behavior side cannot be valid when behavior is invalid")
    behavior = int(row["behavior_class"])
    side = int(row["behavior_side"])
    if behavior_valid:
        if behavior not in range(len(BEHAVIOR_CLASS_NAMES_V1)):
            raise ValueError("valid behavior class is outside ontology")
        if row["behavior_label"] != BEHAVIOR_CLASS_NAMES_V1[behavior]:
            raise ValueError("behavior class/label mismatch")
        if row["source"] not in BEHAVIOR_LABEL_SOURCES_V1:
            raise ValueError("valid behavior row has unexpected source")
        if not row["source_stamp_ns"] or not row["source_age_ms"]:
            raise ValueError("valid behavior row requires source timestamp and age")
        int(row["source_stamp_ns"])
        source_age_ms = float(row["source_age_ms"])
        if not math.isfinite(source_age_ms) or source_age_ms < 0.0:
            raise ValueError("behavior source age must be finite and non-negative")
    elif behavior != -1 or row["behavior_label"] != "UNKNOWN":
        raise ValueError("masked behavior row must use UNKNOWN/-1")
    if side_valid:
        if side not in range(len(BEHAVIOR_SIDE_NAMES_V1)):
            raise ValueError("valid behavior side is outside ontology")
        if row["behavior_side_label"] != BEHAVIOR_SIDE_NAMES_V1[side]:
            raise ValueError("behavior side id/label mismatch")
    elif side != -1 or row["behavior_side_label"] != "UNKNOWN":
        raise ValueError("masked behavior side row must use UNKNOWN/-1")
    quality = float(row["quality"])
    if not math.isfinite(quality) or not 0.0 <= quality <= 1.0:
        raise ValueError("behavior label quality must be within [0,1]")
    int(row["grid_stamp_ns"])


def _strict_bool(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError(f"invalid behavior boolean: {value!r}")
    return normalized == "true"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _directory_sha256(path: Path) -> str:
    """Hash bag storage contents and relative names for reproducible provenance."""

    files = sorted(item for item in path.rglob("*") if item.is_file())
    if not files:
        raise ValueError(f"bag directory contains no files: {path}")
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(_sha256(item)))
    return digest.hexdigest()
