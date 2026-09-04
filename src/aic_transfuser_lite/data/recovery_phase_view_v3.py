from __future__ import annotations

from bisect import bisect_left
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from .collection_reference_v3 import (
    RoutePointV3,
    load_route_reference_v3,
    project_to_route_v3,
)
from .behavior_view_v1 import (
    BEHAVIOR_VIEW_FORMAT_V1,
    BehaviorAnnotationV1,
)
from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_ONTOLOGY_V1,
    BEHAVIOR_SIDE_NAMES_V1,
    BehaviorClassV1,
    BehaviorSideV1,
)
from .recovery_reference_v3 import load_mpc_reference_v3


RECOVERY_PHASE_VIEW_FORMAT = "aic_recovery_phase_view_v1"
_PHASE_FIELDS = (
    "segment_id",
    "phase",
    "side",
    "offset_m",
    "geometry",
    "start_point_id",
    "end_point_id",
    "start_s_m",
    "end_s_m",
    "training_eligible",
)


@dataclass(frozen=True)
class RecoveryPhaseIntervalV3:
    segment_id: str
    phase: str
    side: str
    offset_m: float
    geometry: str
    start_point_id: int
    end_point_id: int
    start_s_m: float
    end_s_m: float
    training_eligible: bool

    @property
    def signed_offset_m(self) -> float:
        return self.offset_m if self.side == "left" else -self.offset_m

    def validate(self) -> None:
        if not self.segment_id or self.phase not in {"approach", "hold", "recovery"}:
            raise ValueError(f"invalid recovery phase identity: {self}")
        if self.side not in {"left", "right"} or not self.geometry:
            raise ValueError(f"invalid recovery phase side/geometry: {self}")
        if self.start_point_id < 0 or self.end_point_id < self.start_point_id:
            raise ValueError(f"invalid recovery phase point interval: {self}")
        values = (self.offset_m, self.start_s_m, self.end_s_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"non-finite recovery phase interval: {self}")
        if self.offset_m <= 0.0 or self.end_s_m < self.start_s_m:
            raise ValueError(f"invalid recovery phase SI interval: {self}")


@dataclass(frozen=True)
class RecoveryPhaseLabelV3:
    sample_id: str
    run_id: str
    grid_stamp_ns: int
    phase: str
    segment_id: str
    side: str
    geometry: str
    requested_signed_offset_m: float
    generated_point_id: int
    pose_source_stamp_ns: int
    pose_delta_ms: float
    generated_lateral_error_m: float
    base_lateral_offset_m: float
    training_eligible: bool


def load_recovery_phase_intervals_v3(
    path: str | Path,
) -> tuple[RecoveryPhaseIntervalV3, ...]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != _PHASE_FIELDS:
            raise ValueError(
                f"recovery phase interval fields must be {_PHASE_FIELDS}, got {reader.fieldnames}"
            )
        intervals = tuple(
            RecoveryPhaseIntervalV3(
                segment_id=row["segment_id"],
                phase=row["phase"],
                side=row["side"],
                offset_m=float(row["offset_m"]),
                geometry=row["geometry"],
                start_point_id=int(row["start_point_id"]),
                end_point_id=int(row["end_point_id"]),
                start_s_m=float(row["start_s_m"]),
                end_s_m=float(row["end_s_m"]),
                training_eligible=_strict_bool(row["training_eligible"]),
            )
            for row in reader
        )
    if not intervals:
        raise ValueError("recovery phase intervals are empty")
    for interval in intervals:
        interval.validate()
    occupied: set[int] = set()
    for interval in intervals:
        points = set(range(interval.start_point_id, interval.end_point_id + 1))
        if occupied.intersection(points):
            raise ValueError("recovery phase point intervals overlap")
        occupied.update(points)
    return intervals


def classify_recovery_phase_v3(
    point_id: int,
    intervals: Sequence[RecoveryPhaseIntervalV3],
) -> RecoveryPhaseIntervalV3 | None:
    matches = [
        interval
        for interval in intervals
        if interval.start_point_id <= point_id <= interval.end_point_id
    ]
    if len(matches) > 1:
        raise ValueError(f"generated point {point_id} matches multiple recovery phases")
    return matches[0] if matches else None


def build_recovery_phase_labels_v3(
    sample_rows: Sequence[Mapping[str, str]],
    poses: Sequence[Any],
    *,
    run_id: str,
    generated_reference_path: str | Path,
    base_reference_path: str | Path,
    intervals_path: str | Path,
    max_pose_delta_ms: float = 50.0,
) -> tuple[RecoveryPhaseLabelV3, ...]:
    if not run_id:
        raise ValueError("run_id must be non-empty")
    if not math.isfinite(max_pose_delta_ms) or max_pose_delta_ms <= 0.0:
        raise ValueError("max_pose_delta_ms must be finite and positive")
    ordered_poses = tuple(sorted(poses, key=lambda item: int(item.timestamp_ns)))
    if not ordered_poses:
        raise ValueError(f"run {run_id!r} has no poses")
    pose_stamps = [int(item.timestamp_ns) for item in ordered_poses]
    if len(set(pose_stamps)) != len(pose_stamps):
        raise ValueError(f"run {run_id!r} has duplicate pose timestamps")
    generated = _mpc_route_points(generated_reference_path)
    base = load_route_reference_v3(base_reference_path)
    intervals = load_recovery_phase_intervals_v3(intervals_path)
    labels: list[RecoveryPhaseLabelV3] = []
    for row in sample_rows:
        if row["run_id"] != run_id:
            continue
        grid_stamp_ns = int(row["grid_stamp_ns"])
        pose = _nearest_pose(grid_stamp_ns, ordered_poses, pose_stamps)
        pose_delta_ms = abs(int(pose.timestamp_ns) - grid_stamp_ns) / 1e6
        if pose_delta_ms > max_pose_delta_ms:
            raise ValueError(
                f"sample {row['sample_id']} nearest pose delta {pose_delta_ms:.6f} ms "
                f"exceeds {max_pose_delta_ms:.6f} ms"
            )
        generated_projection = project_to_route_v3(
            float(pose.x_world_m),
            float(pose.y_world_m),
            float(pose.yaw_world_rad),
            generated,
        )
        base_projection = project_to_route_v3(
            float(pose.x_world_m),
            float(pose.y_world_m),
            float(pose.yaw_world_rad),
            base,
        )
        interval = classify_recovery_phase_v3(generated_projection.point_id, intervals)
        labels.append(
            RecoveryPhaseLabelV3(
                sample_id=row["sample_id"],
                run_id=run_id,
                grid_stamp_ns=grid_stamp_ns,
                phase="baseline" if interval is None else interval.phase,
                segment_id="none" if interval is None else interval.segment_id,
                side="none" if interval is None else interval.side,
                geometry="unknown" if interval is None else interval.geometry,
                requested_signed_offset_m=(
                    0.0 if interval is None else interval.signed_offset_m
                ),
                generated_point_id=generated_projection.point_id,
                pose_source_stamp_ns=int(pose.timestamp_ns),
                pose_delta_ms=pose_delta_ms,
                generated_lateral_error_m=generated_projection.lateral_offset_m,
                base_lateral_offset_m=base_projection.lateral_offset_m,
                training_eligible=False if interval is None else interval.training_eligible,
            )
        )
    if not labels:
        raise ValueError(f"Dataset V3 has no samples for run {run_id!r}")
    return tuple(labels)


def write_recovery_phase_view_v3(
    output_root: str | Path,
    *,
    dataset_manifest_sha256: str,
    labels: Sequence[RecoveryPhaseLabelV3],
    source_records: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    if len(dataset_manifest_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in dataset_manifest_sha256
    ):
        raise ValueError("dataset_manifest_sha256 must be lowercase SHA-256")
    output = Path(output_root)
    if output.exists():
        raise FileExistsError(f"recovery phase view output already exists: {output}")
    if not labels:
        raise ValueError("recovery phase view requires labels")
    sample_ids = [label.sample_id for label in labels]
    if len(set(sample_ids)) != len(sample_ids):
        raise ValueError("recovery phase view contains duplicate sample_id values")
    output.mkdir(parents=True)
    labels_path = output / "phase_labels.csv"
    with labels_path.open("w", newline="", encoding="utf-8") as stream:
        fields = list(RecoveryPhaseLabelV3.__dataclass_fields__)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for label in labels:
            writer.writerow(asdict(label))
    by_phase: dict[str, int] = {}
    by_run: dict[str, int] = {}
    for label in labels:
        by_phase[label.phase] = by_phase.get(label.phase, 0) + 1
        by_run[label.run_id] = by_run.get(label.run_id, 0) + 1
    payload: dict[str, Any] = {
        "format": RECOVERY_PHASE_VIEW_FORMAT,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "labels_sha256": _sha256(labels_path),
        "sample_count": len(labels),
        "eligible_sample_count": sum(
            int(label.training_eligible) for label in labels
        ),
        "phase_counts": dict(sorted(by_phase.items())),
        "run_counts": dict(sorted(by_run.items())),
        "pose_delta_ms": {
            "max": max(label.pose_delta_ms for label in labels),
            "mean": sum(label.pose_delta_ms for label in labels) / len(labels),
        },
        "sources": list(source_records),
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return payload


def load_recovery_phase_view_v3(
    root: str | Path,
    *,
    dataset_manifest_sha256: str,
) -> tuple[RecoveryPhaseLabelV3, ...]:
    source = Path(root)
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("format") != RECOVERY_PHASE_VIEW_FORMAT:
        raise ValueError("recovery phase view format mismatch")
    if manifest.get("dataset_manifest_sha256") != dataset_manifest_sha256:
        raise ValueError("recovery phase view targets a different Dataset V3 manifest")
    labels_path = source / "phase_labels.csv"
    if _sha256(labels_path) != manifest.get("labels_sha256"):
        raise ValueError("recovery phase labels SHA-256 mismatch")
    with labels_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = list(RecoveryPhaseLabelV3.__dataclass_fields__)
        if reader.fieldnames != fields:
            raise ValueError("recovery phase labels CSV fields mismatch")
        labels = tuple(
            RecoveryPhaseLabelV3(
                sample_id=row["sample_id"],
                run_id=row["run_id"],
                grid_stamp_ns=int(row["grid_stamp_ns"]),
                phase=row["phase"],
                segment_id=row["segment_id"],
                side=row["side"],
                geometry=row["geometry"],
                requested_signed_offset_m=float(row["requested_signed_offset_m"]),
                generated_point_id=int(row["generated_point_id"]),
                pose_source_stamp_ns=int(row["pose_source_stamp_ns"]),
                pose_delta_ms=float(row["pose_delta_ms"]),
                generated_lateral_error_m=float(row["generated_lateral_error_m"]),
                base_lateral_offset_m=float(row["base_lateral_offset_m"]),
                training_eligible=_strict_bool(row["training_eligible"]),
            )
            for row in reader
        )
    if len(labels) != manifest.get("sample_count"):
        raise ValueError("recovery phase view sample count mismatch")
    if len({label.sample_id for label in labels}) != len(labels):
        raise ValueError("recovery phase view contains duplicate sample_id values")
    return labels


def write_recovery_behavior_view_v1(
    output_root: str | Path,
    *,
    dataset_manifest_sha256: str,
    phase_view_manifest_sha256: str,
    labels: Sequence[RecoveryPhaseLabelV3],
) -> dict[str, Any]:
    """Write behavior labels without conflating lateral return with safety recovery.

    These runs contain no dynamic obstacle decision. Every reference-tracking
    phase is therefore FORWARD_NORMAL; the lateral excursion remains encoded
    only by the authoritative future trajectory target.
    """

    output = Path(output_root)
    if output.exists():
        raise FileExistsError(f"recovery behavior view output already exists: {output}")
    if not labels:
        raise ValueError("recovery behavior view requires labels")
    if len({label.sample_id for label in labels}) != len(labels):
        raise ValueError("recovery behavior view contains duplicate sample IDs")
    annotations = tuple(
        BehaviorAnnotationV1(
            sample_id=label.sample_id,
            run_id=label.run_id,
            grid_stamp_ns=label.grid_stamp_ns,
            behavior_class=int(BehaviorClassV1.FORWARD_NORMAL),
            behavior_label=BehaviorClassV1.FORWARD_NORMAL.name,
            behavior_side=int(BehaviorSideV1.NONE),
            behavior_side_label=BehaviorSideV1.NONE.name,
            behavior_valid=True,
            behavior_side_valid=True,
            quality=1.0,
            source_stamp_ns=label.pose_source_stamp_ns,
            source_age_ms=label.pose_delta_ms,
            source="recovery_reference_phase",
            authority="teacher_reference_tracking",
            target_vehicle=None,
            invalid_reason=None,
        )
        for label in labels
    )
    output.mkdir(parents=True)
    labels_path = output / "behavior_labels.csv"
    with labels_path.open("w", newline="", encoding="utf-8") as stream:
        fields = list(BehaviorAnnotationV1.__dataclass_fields__)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for annotation in annotations:
            writer.writerow(asdict(annotation))
    payload: dict[str, Any] = {
        "format": BEHAVIOR_VIEW_FORMAT_V1,
        "ontology": BEHAVIOR_ONTOLOGY_V1,
        "class_names": list(BEHAVIOR_CLASS_NAMES_V1),
        "side_names": list(BEHAVIOR_SIDE_NAMES_V1),
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "phase_view_manifest_sha256": phase_view_manifest_sha256,
        "labels_sha256": _sha256(labels_path),
        "sources": [{"source": "recovery_reference_phase"}],
        "sample_count": len(annotations),
        "valid_behavior_count": len(annotations),
        "valid_side_count": len(annotations),
    }
    (output / "manifest.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return payload


def _mpc_route_points(path: str | Path) -> tuple[RoutePointV3, ...]:
    return tuple(
        RoutePointV3(index, point.x_m, point.y_m, point.psi_rad)
        for index, point in enumerate(load_mpc_reference_v3(path))
    )


def _nearest_pose(
    timestamp_ns: int,
    poses: Sequence[Any],
    stamps: Sequence[int],
) -> Any:
    position = bisect_left(stamps, timestamp_ns)
    candidates = [index for index in (position - 1, position) if 0 <= index < len(poses)]
    if not candidates:
        raise ValueError(f"no pose candidate for timestamp {timestamp_ns}")
    return poses[min(candidates, key=lambda index: abs(stamps[index] - timestamp_ns))]


def _strict_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
