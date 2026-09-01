from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Any, Mapping

import numpy as np


DATASET_SCHEMA_VERSION_V3 = "aic_canonical_dataset_v3"
STORAGE_BACKEND_V3 = "csv_npy_jpeg"


class MissingReason(str, Enum):
    NOT_MISSING = "not_missing"
    NOT_RECORDED = "not_recorded"
    OUTSIDE_TOLERANCE = "outside_tolerance"
    INVALID_VALUE = "invalid_value"
    NOT_APPLICABLE = "not_applicable"
    UNKNOWN = "unknown"


class LabelProvenance(str, Enum):
    MEASURED_POSE = "measured_pose"
    MEASURED_VELOCITY = "measured_velocity"
    NOMINAL_COMMAND = "nominal_command"
    FINAL_EXECUTED_COMMAND = "final_executed_command"
    OFFLINE_TEACHER = "offline_teacher"
    MANUAL_ANNOTATION = "manual_annotation"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class OptionalNumericV3:
    """One optional SI-unit scalar with explicit missingness.

    A valid value must be finite and use ``NOT_MISSING``. An invalid value must
    be NaN and include a concrete missing reason. Zero is never a missing-value
    sentinel.
    """

    value: float
    valid: bool
    missing_reason: MissingReason

    def validate(self, *, field_name: str) -> None:
        finite = math.isfinite(float(self.value))
        if self.valid:
            if not finite:
                raise ValueError(f"{field_name} is valid but non-finite")
            if self.missing_reason is not MissingReason.NOT_MISSING:
                raise ValueError(
                    f"{field_name} is valid but has missing reason {self.missing_reason.value}"
                )
        else:
            if finite:
                raise ValueError(
                    f"{field_name} is invalid but has a finite value; use NaN"
                )
            if self.missing_reason is MissingReason.NOT_MISSING:
                raise ValueError(f"{field_name} is invalid without a missing reason")


@dataclass(frozen=True)
class AssetReferenceV3:
    """Reference to an asset sampled for one observation.

    ``source_stamp_ns`` is the original message timestamp and
    ``source_age_ms`` is signed ``(grid_stamp-source_stamp)`` in milliseconds.
    """

    path: str | None
    valid: bool
    source_stamp_ns: int | None
    source_age_ms: float | None
    missing_reason: MissingReason

    def validate(self, *, field_name: str) -> None:
        if self.valid:
            if not self.path:
                raise ValueError(f"{field_name} is valid but has no asset path")
            if self.source_stamp_ns is None or self.source_stamp_ns < 0:
                raise ValueError(f"{field_name} has an invalid source timestamp")
            if self.source_age_ms is None or not math.isfinite(self.source_age_ms):
                raise ValueError(f"{field_name} has an invalid source age")
            if self.missing_reason is not MissingReason.NOT_MISSING:
                raise ValueError(f"{field_name} is valid but has a missing reason")
        else:
            if self.path is not None:
                raise ValueError(f"{field_name} is invalid but has an asset path")
            if self.missing_reason is MissingReason.NOT_MISSING:
                raise ValueError(f"{field_name} is invalid without a missing reason")


@dataclass(frozen=True)
class LidarReferenceV3(AssetReferenceV3):
    valid_path: str | None = None
    points: int | None = None
    angle_min_rad: float | None = None
    angle_increment_rad: float | None = None
    range_min_m: float | None = None
    range_max_m: float | None = None
    frame_id: str | None = None

    def validate(self, *, field_name: str = "lidar") -> None:
        super().validate(field_name=field_name)
        if not self.valid:
            optional_values = (
                self.valid_path,
                self.points,
                self.angle_min_rad,
                self.angle_increment_rad,
                self.range_min_m,
                self.range_max_m,
                self.frame_id,
            )
            if any(value is not None for value in optional_values):
                raise ValueError(f"{field_name} is invalid but contains geometry")
            return
        numeric = (
            self.angle_min_rad,
            self.angle_increment_rad,
            self.range_min_m,
            self.range_max_m,
        )
        if not self.valid_path or self.points is None or self.points < 2:
            raise ValueError(f"{field_name} requires validity path and beam count")
        if not self.frame_id or any(
            value is None or not math.isfinite(float(value)) for value in numeric
        ):
            raise ValueError(f"{field_name} geometry must be explicit and finite")
        assert self.angle_increment_rad is not None
        assert self.range_min_m is not None and self.range_max_m is not None
        if self.angle_increment_rad <= 0.0:
            raise ValueError(f"{field_name} angle increment must be positive")
        if self.range_max_m <= self.range_min_m:
            raise ValueError(f"{field_name} range maximum must exceed minimum")


@dataclass(frozen=True)
class EgoStateV3:
    longitudinal_speed_mps: OptionalNumericV3
    lateral_speed_mps: OptionalNumericV3
    yaw_rate_rps: OptionalNumericV3
    actual_steering_rad: OptionalNumericV3
    gear: str
    gear_valid: bool

    def validate(self) -> None:
        for name in (
            "longitudinal_speed_mps",
            "lateral_speed_mps",
            "yaw_rate_rps",
            "actual_steering_rad",
        ):
            getattr(self, name).validate(field_name=f"ego_state.{name}")
        if self.gear_valid and self.gear == "UNKNOWN":
            raise ValueError("valid ego_state.gear may not be UNKNOWN")
        if not self.gear_valid and self.gear != "UNKNOWN":
            raise ValueError("invalid ego_state.gear must be UNKNOWN")


@dataclass(frozen=True)
class CommandStateV3:
    steering_rad: OptionalNumericV3
    speed_mps: OptionalNumericV3
    acceleration_mps2: OptionalNumericV3
    source_stamp_ns: int | None
    source_age_ms: float | None

    def validate(self, *, field_name: str) -> None:
        for name in ("steering_rad", "speed_mps", "acceleration_mps2"):
            getattr(self, name).validate(field_name=f"{field_name}.{name}")
        any_valid = any(
            getattr(self, name).valid
            for name in ("steering_rad", "speed_mps", "acceleration_mps2")
        )
        if any_valid:
            if self.source_stamp_ns is None or self.source_stamp_ns < 0:
                raise ValueError(f"{field_name} valid values require a source timestamp")
            if self.source_age_ms is None or not math.isfinite(self.source_age_ms):
                raise ValueError(f"{field_name} valid values require a finite source age")


@dataclass(frozen=True)
class DenseFutureStateV3:
    """Dense future ego state in ``base_link@t_obs``.

    Every array has shape ``[N]``. Time is seconds, position is metres, yaw is
    radians, speed is m/s, and yaw rate is rad/s. Invalid rows contain NaN in
    every numeric field and ``valid=False``.
    """

    relative_time_sec: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    yaw_rad: np.ndarray
    longitudinal_speed_mps: np.ndarray
    lateral_speed_mps: np.ndarray
    yaw_rate_rps: np.ndarray
    valid: np.ndarray

    def validate(self) -> None:
        arrays = {
            "relative_time_sec": np.asarray(self.relative_time_sec),
            "x_m": np.asarray(self.x_m),
            "y_m": np.asarray(self.y_m),
            "yaw_rad": np.asarray(self.yaw_rad),
            "longitudinal_speed_mps": np.asarray(self.longitudinal_speed_mps),
            "lateral_speed_mps": np.asarray(self.lateral_speed_mps),
            "yaw_rate_rps": np.asarray(self.yaw_rate_rps),
            "valid": np.asarray(self.valid),
        }
        shapes = {name: value.shape for name, value in arrays.items()}
        if any(value.ndim != 1 for value in arrays.values()) or len(set(shapes.values())) != 1:
            raise ValueError(f"Dense future arrays must share shape [N], got {shapes}")
        if arrays["relative_time_sec"].size == 0:
            raise ValueError("Dense future state must contain at least one step")
        if arrays["valid"].dtype != np.bool_:
            raise ValueError("Dense future valid mask must have boolean dtype")
        times = arrays["relative_time_sec"].astype(np.float64, copy=False)
        if not np.all(np.isfinite(times)) or np.any(times <= 0.0) or np.any(np.diff(times) <= 0.0):
            raise ValueError("Dense future times must be finite, positive, and increasing")
        valid = arrays["valid"]
        for name, values in arrays.items():
            if name in {"relative_time_sec", "valid"}:
                continue
            numeric = values.astype(np.float64, copy=False)
            if np.any(~np.isfinite(numeric[valid])):
                raise ValueError(f"Dense future {name} contains non-finite valid values")
            if np.any(~np.isnan(numeric[~valid])):
                raise ValueError(f"Dense future {name} must use NaN for invalid steps")


@dataclass(frozen=True)
class LabelEvidenceV3:
    valid: bool
    provenance: LabelProvenance
    quality: float
    source_stamp_ns: int | None
    source_age_ms: float | None

    def validate(self, *, label_name: str) -> None:
        if not math.isfinite(self.quality) or not 0.0 <= self.quality <= 1.0:
            raise ValueError(f"{label_name} quality must be within [0,1]")
        if self.valid:
            if self.provenance is LabelProvenance.UNKNOWN:
                raise ValueError(f"valid {label_name} requires known provenance")
            if self.source_stamp_ns is None or self.source_stamp_ns < 0:
                raise ValueError(f"valid {label_name} requires a source timestamp")
            if self.source_age_ms is None or not math.isfinite(self.source_age_ms):
                raise ValueError(f"valid {label_name} requires a finite source age")


@dataclass(frozen=True)
class SampleQualityV3:
    camera_delta_ms: float | None
    lidar_delta_ms: float | None
    max_state_endpoint_delta_ms: float | None
    accepted: bool
    rejection_reasons: tuple[str, ...] = ()

    def validate(self) -> None:
        for name in (
            "camera_delta_ms",
            "lidar_delta_ms",
            "max_state_endpoint_delta_ms",
        ):
            value = getattr(self, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f"quality.{name} must be finite or null")
        if self.accepted and self.rejection_reasons:
            raise ValueError("accepted samples may not have rejection reasons")
        if not self.accepted and not self.rejection_reasons:
            raise ValueError("rejected samples require at least one reason")


@dataclass(frozen=True)
class SampleProvenanceV3:
    labels: Mapping[str, LabelEvidenceV3]

    def validate(self) -> None:
        if not self.labels:
            raise ValueError("sample provenance must describe at least one label")
        for name, evidence in self.labels.items():
            if not name:
                raise ValueError("sample provenance label names must be non-empty")
            evidence.validate(label_name=name)


@dataclass(frozen=True)
class CanonicalSampleV3:
    sample_id: str
    run_id: str
    scenario_id: str
    segment_id: str
    grid_stamp_ns: int
    camera: AssetReferenceV3
    lidar: LidarReferenceV3
    ego_state: EgoStateV3
    nominal_command: CommandStateV3
    final_command: CommandStateV3
    future_state: DenseFutureStateV3 | None
    quality: SampleQualityV3
    provenance: SampleProvenanceV3

    def validate(self) -> None:
        expected_id = make_sample_id(self.run_id, self.segment_id, self.grid_stamp_ns)
        if self.sample_id != expected_id:
            raise ValueError(f"sample_id must be deterministic: expected {expected_id!r}")
        if not self.scenario_id:
            raise ValueError("scenario_id must be non-empty")
        self.camera.validate(field_name="camera")
        self.lidar.validate()
        self.ego_state.validate()
        self.nominal_command.validate(field_name="nominal_command")
        self.final_command.validate(field_name="final_command")
        if self.future_state is not None:
            self.future_state.validate()
        self.quality.validate()
        self.provenance.validate()


@dataclass(frozen=True)
class RunRecordV3:
    run_id: str
    scenario_id: str
    segment_id: str
    source_uri: str
    source_hash: str
    topic_profile_id: str
    start_stamp_ns: int
    end_stamp_ns: int
    capabilities: tuple[str, ...]
    conversion_status: str
    rejection_reasons: tuple[str, ...] = ()

    def validate(self) -> None:
        for name in ("run_id", "scenario_id", "segment_id", "source_uri", "topic_profile_id"):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")
        if len(self.source_hash) != 64 or any(c not in "0123456789abcdef" for c in self.source_hash):
            raise ValueError("source_hash must be a lowercase SHA-256 digest")
        if self.start_stamp_ns < 0 or self.end_stamp_ns < self.start_stamp_ns:
            raise ValueError("run timestamps must satisfy 0 <= start <= end")
        if len(set(self.capabilities)) != len(self.capabilities):
            raise ValueError("run capabilities must be unique")
        if self.conversion_status not in {"complete", "partial", "rejected"}:
            raise ValueError("unknown conversion status")
        if self.conversion_status == "complete" and self.rejection_reasons:
            raise ValueError("complete runs may not have rejection reasons")
        if self.conversion_status == "rejected" and not self.rejection_reasons:
            raise ValueError("rejected runs require a reason")


@dataclass(frozen=True)
class DatasetManifestV3:
    dataset_id: str
    topic_profile_id: str
    runs: tuple[RunRecordV3, ...]
    schema_version: str = DATASET_SCHEMA_VERSION_V3
    storage_backend: str = STORAGE_BACKEND_V3
    coordinate_frame: str = "base_link@t_obs"
    distance_unit: str = "m"
    angle_unit: str = "rad"
    time_unit: str = "s"

    def validate(self) -> None:
        if self.schema_version != DATASET_SCHEMA_VERSION_V3:
            raise ValueError(f"unsupported Dataset V3 schema version: {self.schema_version!r}")
        if self.storage_backend != STORAGE_BACKEND_V3:
            raise ValueError(f"unsupported Dataset V3 storage backend: {self.storage_backend!r}")
        if not self.dataset_id or not self.topic_profile_id:
            raise ValueError("dataset_id and topic_profile_id must be non-empty")
        expected_units = ("base_link@t_obs", "m", "rad", "s")
        actual_units = (
            self.coordinate_frame,
            self.distance_unit,
            self.angle_unit,
            self.time_unit,
        )
        if actual_units != expected_units:
            raise ValueError(f"Dataset V3 SI/frame contract mismatch: {actual_units!r}")
        identities: set[tuple[str, str]] = set()
        for run in self.runs:
            run.validate()
            identity = (run.run_id, run.segment_id)
            if identity in identities:
                raise ValueError(f"duplicate run segment in manifest: {identity!r}")
            identities.add(identity)

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)


def make_sample_id(run_id: str, segment_id: str, grid_stamp_ns: int) -> str:
    """Return ``<run_id>__<segment_id>__<grid_stamp_ns>`` deterministically."""

    if not run_id or not segment_id:
        raise ValueError("run_id and segment_id must be non-empty")
    if int(grid_stamp_ns) < 0:
        raise ValueError("grid_stamp_ns must be non-negative")
    return f"{run_id}__{segment_id}__{int(grid_stamp_ns)}"
