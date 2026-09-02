from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from .canonical_schema_v3 import (
    AssetReferenceV3,
    CanonicalSampleV3,
    CommandStateV3,
    DatasetManifestV3,
    DenseFutureStateV3,
    EgoStateV3,
    LabelEvidenceV3,
    LabelProvenance,
    LidarReferenceV3,
    MissingReason,
    OptionalNumericV3,
    RunRecordV3,
    SampleProvenanceV3,
    SampleQualityV3,
    make_sample_id,
)
from .clock_segments import ClockEpoch
from .mcap_converter_v2 import (
    InterpolationTiming,
    RunStreams,
    TimedImage,
    TimedLidar,
    TimedPose,
    TimedVelocity,
    select_regular_grid,
)
from .storage_v3 import CsvNpyJpegBackend, StorageSummary
from .synchronization_v3 import (
    IndexedTimedValues,
    TimedValue,
    angle_interpolate,
    causal_previous,
    linear_interpolate,
    nearest,
)


CONVERTER_CONFIG_FORMAT_V3 = "aic_dataset_v3_converter_config_v1"


@dataclass(frozen=True)
class DatasetV3ConverterConfig:
    sample_rate_hz: float
    camera_tolerance_ms: float
    lidar_tolerance_ms: float
    interpolation_tolerance_ms: float
    command_max_age_ms: float
    future_step_sec: float
    future_horizon_sec: float
    expected_lidar_points: int | None
    jpeg_quality: int
    require_full_future: bool
    format_version: str = CONVERTER_CONFIG_FORMAT_V3

    def validate(self) -> None:
        if self.format_version != CONVERTER_CONFIG_FORMAT_V3:
            raise ValueError(f"unsupported V3 converter config: {self.format_version!r}")
        positive = (
            self.sample_rate_hz,
            self.camera_tolerance_ms,
            self.lidar_tolerance_ms,
            self.interpolation_tolerance_ms,
            self.command_max_age_ms,
            self.future_step_sec,
            self.future_horizon_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("converter rates, tolerances, and horizons must be finite and positive")
        ratio = self.future_horizon_sec / self.future_step_sec
        if not math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("future_horizon_sec must be an integer multiple of future_step_sec")
        if not math.isclose(self.future_step_sec, 0.1, abs_tol=1e-9):
            raise ValueError("initial Dataset V3 future_step_sec must be 0.1 s")
        if not math.isclose(self.future_horizon_sec, 3.0, abs_tol=1e-9):
            raise ValueError("initial Dataset V3 future_horizon_sec must be 3.0 s")
        if self.expected_lidar_points is not None and self.expected_lidar_points < 2:
            raise ValueError("expected_lidar_points must be null or at least two")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be within [1,100]")

    @property
    def future_times_sec(self) -> np.ndarray:
        count = int(round(self.future_horizon_sec / self.future_step_sec))
        return np.arange(1, count + 1, dtype=np.float64) * self.future_step_sec


@dataclass(frozen=True)
class PreparedCanonicalSampleV3:
    sample: CanonicalSampleV3
    image_rgb: np.ndarray
    lidar_ranges_m: np.ndarray
    lidar_valid: np.ndarray


@dataclass(frozen=True)
class PreparedRunV3:
    run: RunRecordV3
    samples: tuple[PreparedCanonicalSampleV3, ...]


@dataclass(frozen=True)
class _RunStreamIndexesV3:
    poses: IndexedTimedValues[TimedPose]
    lidars: IndexedTimedValues[TimedLidar]
    longitudinal_velocity: IndexedTimedValues[float]
    lateral_velocity: IndexedTimedValues[float]
    yaw_rate: IndexedTimedValues[float]
    actual_steering: IndexedTimedValues[float]
    nominal_commands: IndexedTimedValues[Any]
    final_commands: IndexedTimedValues[Any]
    gears: IndexedTimedValues[int]


def _index_run_streams(streams: RunStreams) -> _RunStreamIndexesV3:
    """Build immutable timestamp indexes once for all per-sample lookups."""

    return _RunStreamIndexesV3(
        poses=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, item) for item in streams.poses)
        ),
        lidars=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, item) for item in streams.lidars)
        ),
        longitudinal_velocity=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, float(item.longitudinal_mps)) for item in streams.velocities)
        ),
        lateral_velocity=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, float(item.lateral_mps)) for item in streams.velocities)
        ),
        yaw_rate=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, float(item.yaw_rate_rps)) for item in streams.velocities)
        ),
        actual_steering=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, float(item.steering_rad)) for item in streams.actual_steering)
        ),
        nominal_commands=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, item) for item in streams.nominal_commands)
        ),
        final_commands=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, item) for item in streams.final_commands)
        ),
        gears=IndexedTimedValues.from_values(
            tuple(TimedValue(item.timestamp_ns, int(item.gear)) for item in streams.gears)
        ),
    )


def _interpolate_pose_indexed(
    poses: IndexedTimedValues[TimedPose], target_ns: int, *, tolerance_ms: float
) -> tuple[TimedPose, InterpolationTiming]:
    if not poses:
        raise ValueError("Cannot interpolate an empty stream")
    position = bisect_left(poses.stamps_ns, target_ns)
    if position < len(poses) and poses.stamps_ns[position] == target_ns:
        left_index = right_index = position
        alpha = 0.0
        timing = InterpolationTiming(0, 0, 0)
    else:
        left_index, right_index = position - 1, position
        if left_index < 0 or right_index >= len(poses):
            raise ValueError("Target is outside the interpolation stream")
        before_delta = poses.stamps_ns[left_index] - target_ns
        after_delta = poses.stamps_ns[right_index] - target_ns
        tolerance_ns = int(round(tolerance_ms * 1e6))
        if abs(before_delta) > tolerance_ns or abs(after_delta) > tolerance_ns:
            raise ValueError("Interpolation endpoints exceed tolerance")
        alpha = (target_ns - poses.stamps_ns[left_index]) / (
            poses.stamps_ns[right_index] - poses.stamps_ns[left_index]
        )
        timing = InterpolationTiming(
            before_delta,
            after_delta,
            max(abs(before_delta), abs(after_delta)),
        )
    left = poses[left_index].value
    right = poses[right_index].value
    if left.frame_id != right.frame_id or left.child_frame_id != right.child_frame_id:
        raise ValueError("Pose frame changed across interpolation endpoints")
    yaw_delta = math.atan2(
        math.sin(right.yaw_world_rad - left.yaw_world_rad),
        math.cos(right.yaw_world_rad - left.yaw_world_rad),
    )
    yaw = left.yaw_world_rad + alpha * yaw_delta
    return (
        TimedPose(
            timestamp_ns=int(target_ns),
            x_world_m=float(left.x_world_m + alpha * (right.x_world_m - left.x_world_m)),
            y_world_m=float(left.y_world_m + alpha * (right.y_world_m - left.y_world_m)),
            yaw_world_rad=math.atan2(math.sin(yaw), math.cos(yaw)),
            frame_id=left.frame_id,
            child_frame_id=left.child_frame_id,
            timestamp_source="interpolated",
        ),
        timing,
    )


def load_dataset_v3_converter_config(path: str | Path) -> DatasetV3ConverterConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("Dataset V3 converter config root must be a mapping")
    expected = {
        "format_version",
        "sample_rate_hz",
        "camera_tolerance_ms",
        "lidar_tolerance_ms",
        "interpolation_tolerance_ms",
        "command_max_age_ms",
        "future_step_sec",
        "future_horizon_sec",
        "expected_lidar_points",
        "jpeg_quality",
        "require_full_future",
    }
    unknown = sorted(set(raw).difference(expected))
    missing = sorted(expected.difference(raw))
    if unknown or missing:
        raise ValueError(f"Dataset V3 converter config fields missing={missing}, unknown={unknown}")
    if not isinstance(raw["require_full_future"], bool):
        raise ValueError("require_full_future must be boolean")
    config = DatasetV3ConverterConfig(
        format_version=str(raw["format_version"]),
        sample_rate_hz=float(raw["sample_rate_hz"]),
        camera_tolerance_ms=float(raw["camera_tolerance_ms"]),
        lidar_tolerance_ms=float(raw["lidar_tolerance_ms"]),
        interpolation_tolerance_ms=float(raw["interpolation_tolerance_ms"]),
        command_max_age_ms=float(raw["command_max_age_ms"]),
        future_step_sec=float(raw["future_step_sec"]),
        future_horizon_sec=float(raw["future_horizon_sec"]),
        expected_lidar_points=(
            None
            if raw["expected_lidar_points"] is None
            else int(raw["expected_lidar_points"])
        ),
        jpeg_quality=int(raw["jpeg_quality"]),
        require_full_future=raw["require_full_future"],
    )
    config.validate()
    return config


def convert_decoded_run_v3(
    streams: RunStreams,
    *,
    run_id: str,
    scenario_id: str,
    source_uri: str,
    source_hash: str,
    topic_profile_id: str,
    epochs: Sequence[ClockEpoch],
    config: DatasetV3ConverterConfig,
) -> PreparedRunV3:
    """Convert already-decoded Rosbag streams into model-neutral V3 records.

    Images are ``uint8 [H,W,3]``. Native LiDAR ranges and masks are ``float32
    [P]`` and ``uint8 [P]``. Dense future arrays are ``[30]`` in SI units and
    never use samples outside the observation clock epoch.
    """

    config.validate()
    _validate_identity(run_id, scenario_id, source_uri, source_hash, topic_profile_id)
    geometry = _validate_native_lidar_geometry(
        streams.lidars, expected_points=config.expected_lidar_points
    )
    indexes = _index_run_streams(streams)
    _validate_epoch_ranges(epochs)
    prepared: list[PreparedCanonicalSampleV3] = []
    for epoch in epochs:
        epoch_images = [
            item
            for item in streams.images
            if epoch.first_sim_stamp_ns <= item.timestamp_ns <= epoch.last_sim_stamp_ns
        ]
        if not epoch_images:
            continue
        matches = select_regular_grid(
            [item.timestamp_ns for item in epoch_images],
            sample_rate_hz=config.sample_rate_hz,
            tolerance_ms=config.camera_tolerance_ms,
            origin_ns=epoch_images[0].timestamp_ns,
        )
        for match in matches:
            image = epoch_images[match.source_index]
            converted = _convert_observation(
                indexes=indexes,
                image=image,
                grid_stamp_ns=match.target_timestamp_ns,
                camera_delta_ns=match.delta_ns,
                run_id=run_id,
                scenario_id=scenario_id,
                segment_id=epoch.epoch_id,
                epoch=epoch,
                geometry=geometry,
                config=config,
            )
            if converted is not None:
                prepared.append(converted)
    if not prepared:
        raise RuntimeError(f"Run {run_id!r} produced no valid Dataset V3 samples")
    first_stamp = min(item.sample.grid_stamp_ns for item in prepared)
    last_stamp = max(item.sample.grid_stamp_ns for item in prepared)
    capabilities = ["camera_input", "lidar_input", "trajectory_label", "v1_compatibility"]
    if streams.actual_steering:
        capabilities.append("actual_steering")
    if streams.nominal_commands or streams.final_commands:
        capabilities.append("full_control_label")
    run = RunRecordV3(
        run_id=run_id,
        scenario_id=scenario_id,
        segment_id="multi" if len(epochs) > 1 else epochs[0].epoch_id,
        source_uri=source_uri,
        source_hash=source_hash,
        topic_profile_id=topic_profile_id,
        start_stamp_ns=first_stamp,
        end_stamp_ns=last_stamp,
        capabilities=tuple(capabilities),
        conversion_status="complete",
    )
    run.validate()
    return PreparedRunV3(run=run, samples=tuple(prepared))


def write_prepared_dataset_v3(
    output_root: str | Path,
    *,
    dataset_id: str,
    topic_profile_id: str,
    runs: Sequence[PreparedRunV3],
    jpeg_quality: int,
) -> StorageSummary:
    """Persist prepared runs through the atomic initial V3 backend."""

    if not runs:
        raise ValueError("at least one prepared run is required")
    manifest = DatasetManifestV3(
        dataset_id=dataset_id,
        topic_profile_id=topic_profile_id,
        runs=tuple(run.run for run in runs),
    )
    with CsvNpyJpegBackend(output_root, manifest) as backend:
        for prepared_run in runs:
            backend.write_run(prepared_run.run)
            for item in prepared_run.samples:
                sample = item.sample
                backend.write_image(
                    Path(sample.run_id) / f"{sample.sample_id}.jpg",
                    item.image_rgb,
                    quality=jpeg_quality,
                )
                backend.write_array(
                    "lidar", Path(sample.run_id) / f"{sample.sample_id}.npy", item.lidar_ranges_m
                )
                backend.write_array(
                    "lidar",
                    Path(sample.run_id) / f"{sample.sample_id}.valid.npy",
                    item.lidar_valid,
                )
                assert sample.future_state is not None
                trajectory = np.stack(
                    [
                        sample.future_state.relative_time_sec,
                        sample.future_state.x_m,
                        sample.future_state.y_m,
                        sample.future_state.yaw_rad,
                        sample.future_state.longitudinal_speed_mps,
                        sample.future_state.lateral_speed_mps,
                        sample.future_state.yaw_rate_rps,
                        sample.future_state.valid.astype(np.float32),
                    ],
                    axis=1,
                ).astype(np.float32)
                trajectory_path = backend.write_array(
                    "trajectories",
                    Path(sample.run_id) / f"{sample.sample_id}.npy",
                    trajectory,
                )
                backend.write_sample(_sample_storage_row(sample, trajectory_path))
        return backend.finalize()


def _convert_observation(
    *,
    indexes: _RunStreamIndexesV3,
    image: TimedImage,
    grid_stamp_ns: int,
    camera_delta_ns: int,
    run_id: str,
    scenario_id: str,
    segment_id: str,
    epoch: ClockEpoch,
    geometry: dict[str, Any],
    config: DatasetV3ConverterConfig,
) -> PreparedCanonicalSampleV3 | None:
    lidar_sync = nearest(
        indexes.lidars,
        target_ns=image.timestamp_ns,
        tolerance_ns=int(round(config.lidar_tolerance_ms * 1e6)),
    )
    if not lidar_sync.valid or lidar_sync.value is None:
        return None
    lidar = lidar_sync.value
    try:
        pose, pose_timing = _interpolate_pose_indexed(
            indexes.poses,
            image.timestamp_ns,
            tolerance_ms=config.interpolation_tolerance_ms,
        )
        velocity, velocity_delta_ms = _velocity_at(
            indexes, image.timestamp_ns, config.interpolation_tolerance_ms
        )
    except ValueError:
        return None
    actual_steering = _actual_steering_at(
        indexes.actual_steering, image.timestamp_ns, config.interpolation_tolerance_ms
    )
    nominal = _command_at(
        indexes.nominal_commands, image.timestamp_ns, config.command_max_age_ms
    )
    final = _command_at(
        indexes.final_commands, image.timestamp_ns, config.command_max_age_ms
    )
    gear = causal_previous(
        indexes.gears,
        target_ns=image.timestamp_ns,
        max_age_ns=int(round(config.command_max_age_ms * 1e6)),
    )
    future = _dense_future_state(
        indexes,
        observation=pose,
        epoch=epoch,
        config=config,
    )
    if config.require_full_future and not bool(future.valid.all()):
        return None
    sample_id = make_sample_id(run_id, segment_id, grid_stamp_ns)
    image_path = f"images/{run_id}/{sample_id}.jpg"
    lidar_path = f"lidar/{run_id}/{sample_id}.npy"
    lidar_valid_path = f"lidar/{run_id}/{sample_id}.valid.npy"
    sanitized, valid_mask = _sanitize_lidar(lidar)
    future_valid = bool(future.valid.all())
    sample = CanonicalSampleV3(
        sample_id=sample_id,
        run_id=run_id,
        scenario_id=scenario_id,
        segment_id=segment_id,
        grid_stamp_ns=grid_stamp_ns,
        camera=AssetReferenceV3(
            image_path,
            True,
            image.timestamp_ns,
            (grid_stamp_ns - image.timestamp_ns) / 1e6,
            MissingReason.NOT_MISSING,
        ),
        lidar=LidarReferenceV3(
            path=lidar_path,
            valid=True,
            source_stamp_ns=lidar.timestamp_ns,
            source_age_ms=(grid_stamp_ns - lidar.timestamp_ns) / 1e6,
            missing_reason=MissingReason.NOT_MISSING,
            valid_path=lidar_valid_path,
            points=int(geometry["points"]),
            angle_min_rad=float(geometry["angle_min_rad"]),
            angle_increment_rad=float(geometry["angle_increment_rad"]),
            range_min_m=float(geometry["range_min_m"]),
            range_max_m=float(geometry["range_max_m"]),
            frame_id=str(geometry["frame_id"]),
        ),
        ego_state=EgoStateV3(
            longitudinal_speed_mps=_present(velocity.longitudinal_mps),
            lateral_speed_mps=_present(velocity.lateral_mps),
            yaw_rate_rps=_present(velocity.yaw_rate_rps),
            actual_steering_rad=actual_steering,
            gear=str(gear.value) if gear.valid else "UNKNOWN",
            gear_valid=gear.valid,
        ),
        nominal_command=nominal,
        final_command=final,
        future_state=future,
        quality=SampleQualityV3(
            camera_delta_ms=camera_delta_ns / 1e6,
            lidar_delta_ms=(lidar.timestamp_ns - image.timestamp_ns) / 1e6,
            max_state_endpoint_delta_ms=max(
                pose_timing.max_endpoint_delta_ns / 1e6, velocity_delta_ms
            ),
            accepted=True,
        ),
        provenance=SampleProvenanceV3(
            labels={
                "future_state": LabelEvidenceV3(
                    valid=future_valid,
                    provenance=(
                        LabelProvenance.MEASURED_POSE
                        if future_valid
                        else LabelProvenance.UNKNOWN
                    ),
                    quality=float(future.valid.mean()),
                    source_stamp_ns=(
                        image.timestamp_ns
                        + int(round(config.future_horizon_sec * 1e9))
                        if future_valid
                        else None
                    ),
                    source_age_ms=(-config.future_horizon_sec * 1000.0 if future_valid else None),
                )
            }
        ),
    )
    sample.validate()
    return PreparedCanonicalSampleV3(
        sample=sample,
        image_rgb=np.ascontiguousarray(image.image_rgb, dtype=np.uint8),
        lidar_ranges_m=sanitized,
        lidar_valid=valid_mask,
    )


def _dense_future_state(
    indexes: _RunStreamIndexesV3,
    *,
    observation: TimedPose,
    epoch: ClockEpoch,
    config: DatasetV3ConverterConfig,
) -> DenseFutureStateV3:
    times = config.future_times_sec
    fields = [np.full(times.shape, np.nan, dtype=np.float32) for _ in range(6)]
    valid = np.zeros(times.shape, dtype=np.bool_)
    cosine = math.cos(observation.yaw_world_rad)
    sine = math.sin(observation.yaw_world_rad)
    for index, relative_sec in enumerate(times):
        target_ns = observation.timestamp_ns + int(round(float(relative_sec) * 1e9))
        if target_ns > epoch.last_sim_stamp_ns:
            continue
        try:
            future_pose, _ = _interpolate_pose_indexed(
                indexes.poses,
                target_ns,
                tolerance_ms=config.interpolation_tolerance_ms,
            )
            future_velocity, _ = _velocity_at(
                indexes, target_ns, config.interpolation_tolerance_ms
            )
        except ValueError:
            continue
        dx = future_pose.x_world_m - observation.x_world_m
        dy = future_pose.y_world_m - observation.y_world_m
        fields[0][index] = cosine * dx + sine * dy
        fields[1][index] = -sine * dx + cosine * dy
        fields[2][index] = _wrap_angle(future_pose.yaw_world_rad - observation.yaw_world_rad)
        fields[3][index] = future_velocity.longitudinal_mps
        fields[4][index] = future_velocity.lateral_mps
        fields[5][index] = future_velocity.yaw_rate_rps
        valid[index] = True
    result = DenseFutureStateV3(
        relative_time_sec=times.astype(np.float32),
        x_m=fields[0],
        y_m=fields[1],
        yaw_rad=fields[2],
        longitudinal_speed_mps=fields[3],
        lateral_speed_mps=fields[4],
        yaw_rate_rps=fields[5],
        valid=valid,
    )
    result.validate()
    return result


def _velocity_at(
    indexes: _RunStreamIndexesV3, target_ns: int, tolerance_ms: float
) -> tuple[TimedVelocity, float]:
    tolerance_ns = int(round(tolerance_ms * 1e6))
    components = []
    for name, stream in (
        ("longitudinal_mps", indexes.longitudinal_velocity),
        ("lateral_mps", indexes.lateral_velocity),
        ("yaw_rate_rps", indexes.yaw_rate),
    ):
        result = linear_interpolate(
            stream,
            target_ns=target_ns,
            tolerance_ns=tolerance_ns,
        )
        if not result.valid or result.value is None:
            raise ValueError(f"velocity {name} unavailable: {result.reason}")
        components.append((float(result.value), result))
    endpoint_delta_ms = max(
        max(abs(stamp - target_ns) for stamp in result.source_stamps_ns)
        for _, result in components
    ) / 1e6
    return (
        TimedVelocity(target_ns, components[0][0], components[1][0], components[2][0]),
        endpoint_delta_ms,
    )


def _actual_steering_at(
    values: IndexedTimedValues[float], target_ns: int, tolerance_ms: float
) -> OptionalNumericV3:
    if not values:
        return _missing(MissingReason.NOT_RECORDED)
    result = angle_interpolate(
        values,
        target_ns=target_ns,
        tolerance_ns=int(round(tolerance_ms * 1e6)),
    )
    return _present(float(result.value)) if result.valid and result.value is not None else _missing(MissingReason.OUTSIDE_TOLERANCE)


def _command_at(
    values: IndexedTimedValues[Any], target_ns: int, max_age_ms: float
) -> CommandStateV3:
    result = causal_previous(
        values,
        target_ns=target_ns,
        max_age_ns=int(round(max_age_ms * 1e6)),
    )
    if not result.valid or result.value is None:
        reason = MissingReason.NOT_RECORDED if not values else MissingReason.OUTSIDE_TOLERANCE
        return CommandStateV3(_missing(reason), _missing(reason), _missing(reason), None, None)
    value = result.value
    return CommandStateV3(
        _present(value.steering_rad),
        _present(value.speed_mps),
        _present(value.acceleration_mps2),
        value.timestamp_ns,
        float(result.age_ns or 0) / 1e6,
    )


def _validate_native_lidar_geometry(
    lidars: Sequence[TimedLidar], *, expected_points: int | None
) -> dict[str, Any]:
    if not lidars:
        raise ValueError("LiDAR stream is empty")
    first = lidars[0]
    points = int(np.asarray(first.ranges_m).size)
    if points < 2 or (expected_points is not None and points != expected_points):
        raise ValueError("native LiDAR beam count violates the configured contract")
    for item in lidars:
        same = (
            np.asarray(item.ranges_m).shape == (points,)
            and math.isclose(item.angle_min_rad, first.angle_min_rad, abs_tol=1e-7)
            and math.isclose(item.angle_increment_rad, first.angle_increment_rad, abs_tol=1e-9)
            and math.isclose(item.range_min_m, first.range_min_m, abs_tol=1e-7)
            and math.isclose(item.range_max_m, first.range_max_m, abs_tol=1e-7)
            and item.frame_id == first.frame_id
        )
        if not same:
            raise ValueError("LiDAR geometry changed within one run")
    return {
        "points": points,
        "angle_min_rad": first.angle_min_rad,
        "angle_increment_rad": first.angle_increment_rad,
        "range_min_m": first.range_min_m,
        "range_max_m": first.range_max_m,
        "frame_id": first.frame_id,
    }


def _sanitize_lidar(item: TimedLidar) -> tuple[np.ndarray, np.ndarray]:
    ranges = np.asarray(item.ranges_m, dtype=np.float32)
    valid = (
        np.isfinite(ranges)
        & (ranges >= float(item.range_min_m))
        & (ranges <= float(item.range_max_m))
    )
    return (
        np.where(valid, ranges, float(item.range_max_m)).astype(np.float32),
        valid.astype(np.uint8),
    )


def _sample_storage_row(sample: CanonicalSampleV3, trajectory_path: str) -> dict[str, Any]:
    assert sample.future_state is not None
    return {
        "sample_id": sample.sample_id,
        "run_id": sample.run_id,
        "scenario_id": sample.scenario_id,
        "segment_id": sample.segment_id,
        "grid_stamp_ns": sample.grid_stamp_ns,
        "image_path": sample.camera.path,
        "lidar_path": sample.lidar.path,
        "lidar_valid_path": sample.lidar.valid_path,
        "trajectory_path": trajectory_path,
        "future_valid_count": int(sample.future_state.valid.sum()),
        "future_step_count": int(sample.future_state.valid.size),
        "camera_delta_ms": sample.quality.camera_delta_ms,
        "lidar_delta_ms": sample.quality.lidar_delta_ms,
        "max_state_endpoint_delta_ms": sample.quality.max_state_endpoint_delta_ms,
        "velocity_longitudinal_mps": sample.ego_state.longitudinal_speed_mps.value,
        "velocity_lateral_mps": sample.ego_state.lateral_speed_mps.value,
        "yaw_rate_rps": sample.ego_state.yaw_rate_rps.value,
        "actual_steering_rad": sample.ego_state.actual_steering_rad.value,
        "actual_steering_valid": sample.ego_state.actual_steering_rad.valid,
        "nominal_command_age_ms": sample.nominal_command.source_age_ms,
        "final_command_age_ms": sample.final_command.source_age_ms,
        "nominal_command": json.dumps(_command_json(sample.nominal_command), allow_nan=True),
        "final_command": json.dumps(_command_json(sample.final_command), allow_nan=True),
        "label_provenance": "measured_pose",
    }


def _command_json(command: CommandStateV3) -> dict[str, Any]:
    return {
        "steering_rad": command.steering_rad.value,
        "speed_mps": command.speed_mps.value,
        "acceleration_mps2": command.acceleration_mps2.value,
        "valid": command.steering_rad.valid and command.speed_mps.valid and command.acceleration_mps2.valid,
        "source_stamp_ns": command.source_stamp_ns,
        "source_age_ms": command.source_age_ms,
    }


def _present(value: float) -> OptionalNumericV3:
    if not math.isfinite(float(value)):
        raise ValueError("present canonical numeric values must be finite")
    return OptionalNumericV3(float(value), True, MissingReason.NOT_MISSING)


def _missing(reason: MissingReason) -> OptionalNumericV3:
    if reason is MissingReason.NOT_MISSING:
        raise ValueError("missing value requires a missing reason")
    return OptionalNumericV3(float("nan"), False, reason)


def _validate_identity(*values: str) -> None:
    if any(not value for value in values[:-2]):
        raise ValueError("run, scenario, source, and topic profile identifiers must be non-empty")
    source_hash = values[-2]
    if len(source_hash) != 64 or any(c not in "0123456789abcdef" for c in source_hash):
        raise ValueError("source_hash must be a lowercase SHA-256 digest")
    if not values[-1]:
        raise ValueError("topic_profile_id must be non-empty")


def _validate_epoch_ranges(epochs: Sequence[ClockEpoch]) -> None:
    if not epochs:
        raise ValueError("at least one clock epoch is required")
    for epoch in epochs:
        if epoch.first_sim_stamp_ns > epoch.last_sim_stamp_ns:
            raise ValueError("clock epoch has inverted simulation timestamps")
    if any(
        right.first_sim_stamp_ns <= left.last_sim_stamp_ns
        for left, right in zip(epochs, epochs[1:])
    ):
        raise ValueError("clock epochs must be ordered and non-overlapping")


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
