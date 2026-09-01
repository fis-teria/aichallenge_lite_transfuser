from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass, replace
import hashlib
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import pandas as pd
from PIL import Image
import yaml

from .delay_estimation import (
    DelayConsistencyAssessment,
    DelayEstimationConfig,
    DelayFitResult,
    assess_delay_consistency,
    estimate_combined_yaw_delay,
    estimate_steering_delay,
)
from .mcap_converter import assign_run_splits, discover_bag_directories, message_image_to_rgb
from .schema import DATASET_FORMAT_VERSION_V2, validate_v2_row
from .topic_contract_v2 import DATASET_V2_TOPICS, TOPIC_BY_NAME, TOPIC_BY_ROLE


@dataclass(frozen=True)
class V2ConverterConfig:
    """Dataset v2 conversion contract with explicit SI units."""

    sample_rate_hz: float = 10.0
    camera_tolerance_ms: float = 40.0
    lidar_tolerance_ms: float = 30.0
    interpolation_tolerance_ms: float = 50.0
    command_max_age_ms: float = 50.0
    waypoint_times_sec: tuple[float, ...] = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
    target_speed_offset_sec: float = 0.5
    wheelbase_m: float = 1.087
    expected_lidar_points: int | None = None
    jpeg_quality: int = 90
    delay_sample_rate_hz: float = 50.0
    delay_config: DelayEstimationConfig = DelayEstimationConfig()

    def validate(self) -> None:
        if not 9.8 <= self.sample_rate_hz <= 10.2:
            raise ValueError("Dataset v2 sample_rate_hz must be in [9.8, 10.2]")
        for name, value in (
            ("camera_tolerance_ms", self.camera_tolerance_ms),
            ("lidar_tolerance_ms", self.lidar_tolerance_ms),
            ("interpolation_tolerance_ms", self.interpolation_tolerance_ms),
            ("command_max_age_ms", self.command_max_age_ms),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.lidar_tolerance_ms > 30.0:
            raise ValueError("lidar_tolerance_ms must not exceed the 30 ms gate")
        if not self.waypoint_times_sec:
            raise ValueError("waypoint_times_sec must not be empty")
        if any(value <= 0.0 for value in self.waypoint_times_sec):
            raise ValueError("waypoint_times_sec must contain positive values")
        if any(
            right <= left
            for left, right in zip(self.waypoint_times_sec, self.waypoint_times_sec[1:])
        ):
            raise ValueError("waypoint_times_sec must be strictly increasing")
        if self.target_speed_offset_sec <= 0.0:
            raise ValueError("target_speed_offset_sec must be positive")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.expected_lidar_points is not None and self.expected_lidar_points <= 1:
            raise ValueError("expected_lidar_points must be greater than one")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")
        if self.delay_sample_rate_hz <= 0.0:
            raise ValueError("delay_sample_rate_hz must be positive")
        self.delay_config.validate()


@dataclass(frozen=True)
class GridMatch:
    target_timestamp_ns: int
    source_index: int
    source_timestamp_ns: int
    delta_ns: int


@dataclass(frozen=True)
class InterpolationTiming:
    before_delta_ns: int
    after_delta_ns: int
    max_endpoint_delta_ns: int


@dataclass(frozen=True)
class TimedImage:
    timestamp_ns: int
    image_rgb: np.ndarray
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedLidar:
    timestamp_ns: int
    ranges_m: np.ndarray
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    frame_id: str = ""
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedPose:
    timestamp_ns: int
    x_world_m: float
    y_world_m: float
    yaw_world_rad: float
    frame_id: str
    child_frame_id: str
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedVelocity:
    timestamp_ns: int
    longitudinal_mps: float
    lateral_mps: float
    yaw_rate_rps: float
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedSteering:
    timestamp_ns: int
    steering_rad: float
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedCommand:
    timestamp_ns: int
    speed_mps: float
    acceleration_mps2: float
    steering_rad: float
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class TimedGear:
    timestamp_ns: int
    gear: int
    bag_timestamp_ns: int = 0
    timestamp_source: str = "bag"


@dataclass(frozen=True)
class RunStreams:
    images: tuple[TimedImage, ...]
    lidars: tuple[TimedLidar, ...]
    poses: tuple[TimedPose, ...]
    velocities: tuple[TimedVelocity, ...]
    actual_steering: tuple[TimedSteering, ...]
    nominal_commands: tuple[TimedCommand, ...]
    final_commands: tuple[TimedCommand, ...]
    gears: tuple[TimedGear, ...]
    topic_types: Mapping[str, str]
    timestamp_fallback_counts: Mapping[str, int]


@dataclass(frozen=True)
class PreparedSample:
    row: dict[str, Any]
    image_rgb: np.ndarray
    lidar_ranges_m: np.ndarray
    lidar_valid: np.ndarray


@dataclass(frozen=True)
class PreparedRunResult:
    run_id: str
    scenario_id: str
    samples: tuple[PreparedSample, ...]
    input_counts: dict[str, int]
    dropped_counts: dict[str, int]
    lidar_geometry: dict[str, Any]
    quality_metrics: dict[str, Any]
    delay_calibration: DelayFitResult | None = None


def _strict_timestamps(items: Sequence[Any], name: str) -> list[int]:
    timestamps = [int(item.timestamp_ns) for item in items]
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{name} timestamps must be strictly increasing")
    return timestamps


def select_regular_grid(
    timestamps_ns: Sequence[int],
    *,
    sample_rate_hz: float,
    tolerance_ms: float,
    origin_ns: int | None = None,
    max_samples: int | None = None,
) -> list[GridMatch]:
    """Assign unique camera frames to an absolute regular time grid."""

    timestamps = [int(value) for value in timestamps_ns]
    if not timestamps:
        return []
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError("Camera timestamps must be strictly increasing")
    if sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    if tolerance_ms <= 0.0:
        raise ValueError("tolerance_ms must be positive")
    if max_samples is not None and max_samples <= 0:
        raise ValueError("max_samples must be positive")
    interval_ns = int(round(1e9 / sample_rate_hz))
    tolerance_ns = int(round(tolerance_ms * 1e6))
    target_ns = timestamps[0] if origin_ns is None else int(origin_ns)
    used: set[int] = set()
    matches: list[GridMatch] = []
    while target_ns <= timestamps[-1] + tolerance_ns:
        position = bisect_left(timestamps, target_ns)
        candidates = [
            index
            for index in (position - 1, position)
            if 0 <= index < len(timestamps) and index not in used
        ]
        if candidates:
            index = min(
                candidates,
                key=lambda candidate: (abs(timestamps[candidate] - target_ns), candidate),
            )
            delta_ns = timestamps[index] - target_ns
            if abs(delta_ns) <= tolerance_ns:
                matches.append(
                    GridMatch(target_ns, index, timestamps[index], int(delta_ns))
                )
                used.add(index)
                if max_samples is not None and len(matches) >= max_samples:
                    break
        target_ns += interval_ns
    return matches


def _interpolation_indices(
    timestamps_ns: Sequence[int], target_ns: int, tolerance_ms: float
) -> tuple[int, int, float, InterpolationTiming]:
    if not timestamps_ns:
        raise ValueError("Cannot interpolate an empty stream")
    tolerance_ns = int(round(tolerance_ms * 1e6))
    position = bisect_left(timestamps_ns, target_ns)
    if position < len(timestamps_ns) and timestamps_ns[position] == target_ns:
        timing = InterpolationTiming(0, 0, 0)
        return position, position, 0.0, timing
    before = position - 1
    after = position
    if before < 0 or after >= len(timestamps_ns):
        raise ValueError("Target is outside the interpolation stream")
    before_delta = int(timestamps_ns[before] - target_ns)
    after_delta = int(timestamps_ns[after] - target_ns)
    if abs(before_delta) > tolerance_ns or abs(after_delta) > tolerance_ns:
        raise ValueError("Interpolation endpoints exceed tolerance")
    denominator = timestamps_ns[after] - timestamps_ns[before]
    alpha = (target_ns - timestamps_ns[before]) / denominator
    timing = InterpolationTiming(
        before_delta_ns=before_delta,
        after_delta_ns=after_delta,
        max_endpoint_delta_ns=max(abs(before_delta), abs(after_delta)),
    )
    return before, after, float(alpha), timing


def _interpolate_angle(left_rad: float, right_rad: float, alpha: float) -> float:
    delta = math.atan2(math.sin(right_rad - left_rad), math.cos(right_rad - left_rad))
    return math.atan2(
        math.sin(left_rad + alpha * delta), math.cos(left_rad + alpha * delta)
    )


def interpolate_pose(
    poses: Sequence[TimedPose], target_ns: int, *, tolerance_ms: float
) -> tuple[TimedPose, InterpolationTiming]:
    timestamps = [item.timestamp_ns for item in poses]
    left_index, right_index, alpha, timing = _interpolation_indices(
        timestamps, target_ns, tolerance_ms
    )
    left = poses[left_index]
    right = poses[right_index]
    if left.frame_id != right.frame_id or left.child_frame_id != right.child_frame_id:
        raise ValueError("Pose frame changed across interpolation endpoints")
    return (
        TimedPose(
            timestamp_ns=int(target_ns),
            x_world_m=float(left.x_world_m + alpha * (right.x_world_m - left.x_world_m)),
            y_world_m=float(left.y_world_m + alpha * (right.y_world_m - left.y_world_m)),
            yaw_world_rad=_interpolate_angle(left.yaw_world_rad, right.yaw_world_rad, alpha),
            frame_id=left.frame_id,
            child_frame_id=left.child_frame_id,
            timestamp_source="interpolated",
        ),
        timing,
    )


def _interpolate_velocity(
    velocities: Sequence[TimedVelocity], target_ns: int, tolerance_ms: float
) -> tuple[TimedVelocity, InterpolationTiming]:
    timestamps = [item.timestamp_ns for item in velocities]
    left_index, right_index, alpha, timing = _interpolation_indices(
        timestamps, target_ns, tolerance_ms
    )
    left = velocities[left_index]
    right = velocities[right_index]
    return (
        TimedVelocity(
            timestamp_ns=int(target_ns),
            longitudinal_mps=float(
                left.longitudinal_mps
                + alpha * (right.longitudinal_mps - left.longitudinal_mps)
            ),
            lateral_mps=float(
                left.lateral_mps + alpha * (right.lateral_mps - left.lateral_mps)
            ),
            yaw_rate_rps=float(
                left.yaw_rate_rps + alpha * (right.yaw_rate_rps - left.yaw_rate_rps)
            ),
            timestamp_source="interpolated",
        ),
        timing,
    )


def _interpolate_steering(
    values: Sequence[TimedSteering], target_ns: int, tolerance_ms: float
) -> tuple[TimedSteering, InterpolationTiming]:
    timestamps = [item.timestamp_ns for item in values]
    left_index, right_index, alpha, timing = _interpolation_indices(
        timestamps, target_ns, tolerance_ms
    )
    left = values[left_index]
    right = values[right_index]
    return (
        TimedSteering(
            timestamp_ns=int(target_ns),
            steering_rad=float(
                left.steering_rad + alpha * (right.steering_rad - left.steering_rad)
            ),
            timestamp_source="interpolated",
        ),
        timing,
    )


def future_waypoints_from_pose(
    poses: Sequence[TimedPose],
    *,
    observation_ns: int,
    horizons_sec: Sequence[float],
    tolerance_ms: float,
) -> np.ndarray:
    """Create measured future positions in the observation ego SE(2) frame."""

    if not horizons_sec or any(float(value) <= 0.0 for value in horizons_sec):
        raise ValueError("horizons_sec must contain positive values")
    observation, _ = interpolate_pose(poses, observation_ns, tolerance_ms=tolerance_ms)
    cosine = math.cos(observation.yaw_world_rad)
    sine = math.sin(observation.yaw_world_rad)
    waypoints: list[tuple[float, float]] = []
    for horizon_sec in horizons_sec:
        future_ns = observation_ns + int(round(float(horizon_sec) * 1e9))
        future, _ = interpolate_pose(poses, future_ns, tolerance_ms=tolerance_ms)
        delta_x = future.x_world_m - observation.x_world_m
        delta_y = future.y_world_m - observation.y_world_m
        x_ego = cosine * delta_x + sine * delta_y
        y_ego = -sine * delta_x + cosine * delta_y
        waypoints.append((x_ego, y_ego))
    result = np.asarray(waypoints, dtype=np.float32)
    if result.shape != (len(horizons_sec), 2):
        raise AssertionError(f"Waypoint shape mismatch: {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError("Measured waypoints contain NaN or infinity")
    return result


def _nearest_index(
    timestamps_ns: Sequence[int], target_ns: int, tolerance_ms: float
) -> tuple[int, int] | None:
    if not timestamps_ns:
        return None
    position = bisect_left(timestamps_ns, target_ns)
    candidates = [
        index for index in (position - 1, position) if 0 <= index < len(timestamps_ns)
    ]
    index = min(candidates, key=lambda item: abs(timestamps_ns[item] - target_ns))
    delta = int(timestamps_ns[index] - target_ns)
    return (index, delta) if abs(delta) <= int(round(tolerance_ms * 1e6)) else None


def _previous_item(
    items: Sequence[Any], target_ns: int, max_age_ms: float
) -> tuple[Any, int] | None:
    timestamps = [item.timestamp_ns for item in items]
    index = bisect_right(timestamps, target_ns) - 1
    if index < 0:
        return None
    age_ns = int(target_ns - timestamps[index])
    if age_ns > int(round(max_age_ms * 1e6)):
        return None
    return items[index], age_ns


def _validate_lidar_geometry(
    lidars: Sequence[TimedLidar], expected_points: int | None
) -> dict[str, Any]:
    if not lidars:
        raise ValueError("LiDAR stream is empty")
    first = lidars[0]
    source_points = int(np.asarray(first.ranges_m).size)
    if source_points <= 1:
        raise ValueError("LaserScan must contain more than one beam")
    if expected_points is not None and source_points != expected_points:
        raise ValueError(
            f"Native LiDAR points={source_points} do not match expected={expected_points}"
        )
    for item in lidars:
        values = np.asarray(item.ranges_m)
        same = (
            values.ndim == 1
            and values.size == source_points
            and math.isclose(item.angle_min_rad, first.angle_min_rad, abs_tol=1e-7)
            and math.isclose(
                item.angle_increment_rad, first.angle_increment_rad, abs_tol=1e-9
            )
            and math.isclose(item.range_min_m, first.range_min_m, abs_tol=1e-7)
            and math.isclose(item.range_max_m, first.range_max_m, abs_tol=1e-7)
            and item.frame_id == first.frame_id
        )
        if not same:
            raise ValueError("LiDAR geometry changed within one run")
    return {
        "source_points": source_points,
        "saved_points": source_points,
        "angle_min_rad": first.angle_min_rad,
        "angle_increment_rad": first.angle_increment_rad,
        "angle_max_rad": first.angle_min_rad
        + first.angle_increment_rad * (source_points - 1),
        "range_min_m": first.range_min_m,
        "range_max_m": first.range_max_m,
        "frame_id": first.frame_id,
        "resampling": "none_native_beam_order",
        "invalid_replacement": "range_max_m_with_separate_valid_mask",
    }


def _sanitize_native_lidar(item: TimedLidar) -> tuple[np.ndarray, np.ndarray]:
    ranges = np.asarray(item.ranges_m, dtype=np.float32)
    valid = (
        np.isfinite(ranges)
        & (ranges >= float(item.range_min_m))
        & (ranges <= float(item.range_max_m))
    )
    sanitized = np.where(valid, ranges, float(item.range_max_m)).astype(np.float32)
    return sanitized, valid.astype(np.uint8)


def _effective_rate_hz(samples: Sequence[PreparedSample]) -> float:
    if len(samples) < 2:
        return 0.0
    first = int(samples[0].row["grid_timestamp_ns"])
    last = int(samples[-1].row["grid_timestamp_ns"])
    duration_sec = (last - first) / 1e9
    return 0.0 if duration_sec <= 0.0 else (len(samples) - 1) / duration_sec


def prepare_run_samples(
    streams: RunStreams,
    *,
    run_id: str,
    scenario_id: str,
    config: V2ConverterConfig,
    max_samples: int | None = None,
) -> PreparedRunResult:
    """Synchronize already decoded streams and build measured-label samples."""

    config.validate()
    for name, items in (
        ("camera", streams.images),
        ("lidar", streams.lidars),
        ("pose", streams.poses),
        ("velocity", streams.velocities),
        ("nominal_command", streams.nominal_commands),
        ("final_command", streams.final_commands),
        ("gear", streams.gears),
    ):
        if not items:
            raise ValueError(f"Required {name} stream is empty")
        _strict_timestamps(items, name)
    if streams.actual_steering:
        _strict_timestamps(streams.actual_steering, "actual_steering")
    geometry = _validate_lidar_geometry(
        streams.lidars, config.expected_lidar_points
    )
    camera_times = [item.timestamp_ns for item in streams.images]
    lidar_times = [item.timestamp_ns for item in streams.lidars]
    grid = select_regular_grid(
        camera_times,
        sample_rate_hz=config.sample_rate_hz,
        tolerance_ms=config.camera_tolerance_ms,
        origin_ns=camera_times[0],
        max_samples=max_samples,
    )
    dropped = {
        "lidar_sync": 0,
        "pose_velocity_sync": 0,
        "future_horizon": 0,
        "command_age": 0,
        "gear_age": 0,
        "invalid_numeric": 0,
    }
    samples: list[PreparedSample] = []
    lidar_skews_ms: list[float] = []
    max_horizon_sec = max(max(config.waypoint_times_sec), config.target_speed_offset_sec)
    usable_end_ns = min(
        streams.poses[-1].timestamp_ns,
        streams.velocities[-1].timestamp_ns,
    ) - int(round(max_horizon_sec * 1e9))
    eligible_count = sum(
        1 for match in grid if streams.images[match.source_index].timestamp_ns <= usable_end_ns
    )
    for match in grid:
        image = streams.images[match.source_index]
        observation_ns = image.timestamp_ns
        if observation_ns > usable_end_ns:
            dropped["future_horizon"] += 1
            continue
        lidar_match = _nearest_index(
            lidar_times, observation_ns, config.lidar_tolerance_ms
        )
        if lidar_match is None:
            dropped["lidar_sync"] += 1
            continue
        try:
            pose, pose_timing = interpolate_pose(
                streams.poses,
                observation_ns,
                tolerance_ms=config.interpolation_tolerance_ms,
            )
            velocity, velocity_timing = _interpolate_velocity(
                streams.velocities,
                observation_ns,
                config.interpolation_tolerance_ms,
            )
            target_velocity, _ = _interpolate_velocity(
                streams.velocities,
                observation_ns + int(round(config.target_speed_offset_sec * 1e9)),
                config.interpolation_tolerance_ms,
            )
            waypoints = future_waypoints_from_pose(
                streams.poses,
                observation_ns=observation_ns,
                horizons_sec=config.waypoint_times_sec,
                tolerance_ms=config.interpolation_tolerance_ms,
            )
        except ValueError:
            dropped["pose_velocity_sync"] += 1
            continue
        nominal_match = _previous_item(
            streams.nominal_commands, observation_ns, config.command_max_age_ms
        )
        final_match = _previous_item(
            streams.final_commands, observation_ns, config.command_max_age_ms
        )
        if nominal_match is None or final_match is None:
            dropped["command_age"] += 1
            continue
        gear_match = _previous_item(streams.gears, observation_ns, config.command_max_age_ms)
        if gear_match is None:
            dropped["gear_age"] += 1
            continue
        nominal, nominal_age_ns = nominal_match
        final, final_age_ns = final_match
        gear, _ = gear_match
        steering_timing_ms = float("nan")
        actual_steering_rad = float("nan")
        actual_valid = 0
        if streams.actual_steering:
            try:
                steering, steering_timing = _interpolate_steering(
                    streams.actual_steering,
                    observation_ns,
                    config.interpolation_tolerance_ms,
                )
                actual_steering_rad = steering.steering_rad
                steering_timing_ms = steering_timing.max_endpoint_delta_ns / 1e6
                actual_valid = 1
            except ValueError:
                pass
        numeric = (
            velocity.longitudinal_mps,
            velocity.lateral_mps,
            velocity.yaw_rate_rps,
            target_velocity.longitudinal_mps,
            nominal.steering_rad,
            nominal.acceleration_mps2,
            final.steering_rad,
        )
        if not all(math.isfinite(value) for value in numeric):
            dropped["invalid_numeric"] += 1
            continue
        lidar = streams.lidars[lidar_match[0]]
        lidar_ranges, lidar_valid = _sanitize_native_lidar(lidar)
        sample_id = f"{run_id}_{len(samples):06d}"
        image_rel = Path("images") / run_id / f"{sample_id}.jpg"
        lidar_rel = Path("lidar") / run_id / f"{sample_id}.npy"
        valid_rel = Path("lidar_valid") / run_id / f"{sample_id}.npy"
        lidar_dt_ms = lidar_match[1] / 1e6
        quality_terms = (
            abs(match.delta_ns) / (config.camera_tolerance_ms * 1e6),
            abs(lidar_match[1]) / (config.lidar_tolerance_ms * 1e6),
            pose_timing.max_endpoint_delta_ns
            / (config.interpolation_tolerance_ms * 1e6),
            velocity_timing.max_endpoint_delta_ns
            / (config.interpolation_tolerance_ms * 1e6),
            nominal_age_ns / (config.command_max_age_ms * 1e6),
            final_age_ns / (config.command_max_age_ms * 1e6),
        )
        row: dict[str, Any] = {
            "sample_id": sample_id,
            "run_id": run_id,
            "scenario_id": scenario_id,
            "timestamp_ns": observation_ns,
            "grid_timestamp_ns": match.target_timestamp_ns,
            "image_path": image_rel.as_posix(),
            "lidar_path": lidar_rel.as_posix(),
            "lidar_valid_path": valid_rel.as_posix(),
            "velocity_longitudinal_mps": velocity.longitudinal_mps,
            "velocity_lateral_mps": velocity.lateral_mps,
            "yaw_rate_rps": velocity.yaw_rate_rps,
            "gear": int(gear.gear),
            "actual_steering_rad": actual_steering_rad,
            "actual_steering_valid": actual_valid,
            "nominal_command_steering_rad": nominal.steering_rad,
            "nominal_command_speed_mps": nominal.speed_mps,
            "nominal_command_acceleration_mps2": nominal.acceleration_mps2,
            "final_command_steering_rad": final.steering_rad,
            "final_command_speed_mps": final.speed_mps,
            "final_command_acceleration_mps2": final.acceleration_mps2,
            "target_speed_mps": target_velocity.longitudinal_mps,
            "teacher_command_steering_rad": nominal.steering_rad,
            "teacher_command_acceleration_mps2": nominal.acceleration_mps2,
            "collision": float("nan"),
            "offtrack": float("nan"),
            "recovery_flag": float("nan"),
            "quality_score": max(0.0, 1.0 - max(quality_terms)),
            "label_provenance": "measured_pose",
            "pose_frame_id": pose.frame_id,
            "pose_child_frame_id": pose.child_frame_id,
            "pose_x_world_m": pose.x_world_m,
            "pose_y_world_m": pose.y_world_m,
            "pose_yaw_world_rad": pose.yaw_world_rad,
            "lidar_points": int(lidar_ranges.size),
            "camera_dt_ms": match.delta_ns / 1e6,
            "lidar_dt_ms": lidar_dt_ms,
            "pose_dt_ms": pose_timing.max_endpoint_delta_ns / 1e6,
            "velocity_dt_ms": velocity_timing.max_endpoint_delta_ns / 1e6,
            "steering_dt_ms": steering_timing_ms,
            "nominal_command_age_ms": nominal_age_ns / 1e6,
            "final_command_age_ms": final_age_ns / 1e6,
        }
        for index, (x_m, y_m) in enumerate(waypoints):
            row[f"wp_{index}_x"] = float(x_m)
            row[f"wp_{index}_y"] = float(y_m)
        validate_v2_row(row, num_waypoints=len(config.waypoint_times_sec))
        samples.append(
            PreparedSample(
                row=row,
                image_rgb=np.ascontiguousarray(image.image_rgb, dtype=np.uint8),
                lidar_ranges_m=lidar_ranges,
                lidar_valid=lidar_valid,
            )
        )
        lidar_skews_ms.append(abs(lidar_dt_ms))
    effective_rate = _effective_rate_hz(samples)
    gap_count = sum(
        1
        for left, right in zip(samples, samples[1:])
        if int(right.row["grid_timestamp_ns"]) - int(left.row["grid_timestamp_ns"])
        >= 200_000_000
    )
    quality_metrics = {
        "effective_sample_rate_hz": effective_rate,
        "camera_lidar_p95_skew_ms": (
            float(np.percentile(lidar_skews_ms, 95)) if lidar_skews_ms else None
        ),
        "pose_velocity_missing_fraction": (
            dropped["pose_velocity_sync"] / eligible_count if eligible_count else 1.0
        ),
        "grid_candidate_count": len(grid),
        "eligible_candidate_count": eligible_count,
        "output_sample_count": len(samples),
        "gap_at_least_200ms_count": gap_count,
        "actual_steering_valid_fraction": (
            sum(int(sample.row["actual_steering_valid"]) for sample in samples)
            / len(samples)
            if samples
            else 0.0
        ),
        "waypoint_provenance": "measured_pose",
    }
    return PreparedRunResult(
        run_id=run_id,
        scenario_id=scenario_id,
        samples=tuple(samples),
        input_counts={
            "camera": len(streams.images),
            "regular_grid_camera": len(grid),
            "lidar": len(streams.lidars),
            "pose": len(streams.poses),
            "velocity": len(streams.velocities),
            "actual_steering": len(streams.actual_steering),
            "nominal_command": len(streams.nominal_commands),
            "final_command": len(streams.final_commands),
            "gear": len(streams.gears),
            "output": len(samples),
        },
        dropped_counts=dropped,
        lidar_geometry=geometry,
        quality_metrics=quality_metrics,
    )


def _nested(message: Any, path: str) -> Any:
    value = message
    for part in path.split("."):
        value = getattr(value, part)
    return value


def _stamp_ns(message: Any, bag_timestamp_ns: int) -> tuple[int, str]:
    for path in ("header.stamp", "stamp"):
        try:
            stamp = _nested(message, path)
            value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if value > 0:
                return value, path
        except (AttributeError, TypeError, ValueError):
            continue
    return int(bag_timestamp_ns), "bag_timestamp_fallback"


def _yaw_from_quaternion(quaternion: Any) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("Odometry orientation quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _deduplicate_sorted(items: Sequence[Any]) -> tuple[Any, ...]:
    by_timestamp = {int(item.timestamp_ns): item for item in items}
    return tuple(by_timestamp[key] for key in sorted(by_timestamp))


def read_run_messages_v2(bag_dir: Path) -> RunStreams:
    """Deserialize all Dataset v2 streams from one rosbag2 MCAP directory."""

    from rosbags.highlevel import AnyReader

    if not (bag_dir / "metadata.yaml").is_file():
        raise FileNotFoundError(f"rosbag2 metadata not found: {bag_dir / 'metadata.yaml'}")
    buckets: dict[str, list[Any]] = {contract.role: [] for contract in DATASET_V2_TOPICS}
    topic_types: dict[str, str] = {}
    fallback_counts: dict[str, int] = {contract.role: 0 for contract in DATASET_V2_TOPICS}
    with AnyReader([bag_dir]) as reader:
        available = {connection.topic: connection.msgtype for connection in reader.connections}
        required = {
            contract.name
            for contract in DATASET_V2_TOPICS
            if contract.required_for_conversion
        }
        missing = sorted(required.difference(available))
        if missing:
            raise ValueError(f"Run {bag_dir.name} is missing required topics: {missing}")
        for contract in DATASET_V2_TOPICS:
            if contract.name in available and available[contract.name] != contract.message_type:
                raise ValueError(
                    f"Topic {contract.name} type={available[contract.name]!r}, "
                    f"expected={contract.message_type!r}"
                )
        connections = [
            connection for connection in reader.connections if connection.topic in TOPIC_BY_NAME
        ]
        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            contract = TOPIC_BY_NAME[connection.topic]
            topic_types[connection.topic] = connection.msgtype
            message = reader.deserialize(rawdata, connection.msgtype)
            timestamp_ns, timestamp_source = _stamp_ns(message, int(bag_timestamp_ns))
            if timestamp_source == "bag_timestamp_fallback":
                fallback_counts[contract.role] += 1
            if contract.role == "camera":
                item = TimedImage(
                    timestamp_ns,
                    message_image_to_rgb(message),
                    int(bag_timestamp_ns),
                    timestamp_source,
                )
            elif contract.role == "lidar":
                item = TimedLidar(
                    timestamp_ns=timestamp_ns,
                    ranges_m=np.asarray(message.ranges, dtype=np.float32),
                    angle_min_rad=float(message.angle_min),
                    angle_increment_rad=float(message.angle_increment),
                    range_min_m=float(message.range_min),
                    range_max_m=float(message.range_max),
                    frame_id=str(message.header.frame_id),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif contract.role == "pose":
                item = TimedPose(
                    timestamp_ns=timestamp_ns,
                    x_world_m=float(message.pose.pose.position.x),
                    y_world_m=float(message.pose.pose.position.y),
                    yaw_world_rad=_yaw_from_quaternion(message.pose.pose.orientation),
                    frame_id=str(message.header.frame_id),
                    child_frame_id=str(message.child_frame_id),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif contract.role == "velocity":
                item = TimedVelocity(
                    timestamp_ns=timestamp_ns,
                    longitudinal_mps=float(message.longitudinal_velocity),
                    lateral_mps=float(message.lateral_velocity),
                    yaw_rate_rps=float(message.heading_rate),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif contract.role == "actual_steering":
                item = TimedSteering(
                    timestamp_ns=timestamp_ns,
                    steering_rad=float(message.steering_tire_angle),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif contract.role == "gear":
                item = TimedGear(
                    timestamp_ns=timestamp_ns,
                    gear=int(message.report),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            else:
                item = TimedCommand(
                    timestamp_ns=timestamp_ns,
                    speed_mps=float(message.longitudinal.speed),
                    acceleration_mps2=float(message.longitudinal.acceleration),
                    steering_rad=float(message.lateral.steering_tire_angle),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            buckets[contract.role].append(item)
    return RunStreams(
        images=_deduplicate_sorted(buckets["camera"]),
        lidars=_deduplicate_sorted(buckets["lidar"]),
        poses=_deduplicate_sorted(buckets["pose"]),
        velocities=_deduplicate_sorted(buckets["velocity"]),
        actual_steering=_deduplicate_sorted(buckets["actual_steering"]),
        nominal_commands=_deduplicate_sorted(buckets["nominal_command"]),
        final_commands=_deduplicate_sorted(buckets["final_command"]),
        gears=_deduplicate_sorted(buckets["gear"]),
        topic_types=topic_types,
        timestamp_fallback_counts=fallback_counts,
    )


def _invalid_delay_result(reason: str, total_samples: int = 0) -> DelayFitResult:
    return DelayFitResult(
        method=reason,
        delay_sec=0.0,
        time_constant_sec=None,
        objective=0.0,
        steering_nrmse=None,
        yaw_rate_nrmse=0.0,
        correlation_peak=0.0,
        dynamic_sample_count=0,
        total_sample_count=total_samples,
        individual_valid=False,
        validity_reasons=(reason,),
    )


def _hold_coverage_mask(
    source_timestamps_ns: np.ndarray,
    grid_ns: np.ndarray,
    max_age_ns: int,
) -> tuple[np.ndarray, np.ndarray]:
    indices = np.searchsorted(source_timestamps_ns, grid_ns, side="right") - 1
    clipped = np.clip(indices, 0, len(source_timestamps_ns) - 1)
    ages = grid_ns - source_timestamps_ns[clipped]
    valid = (indices >= 0) & (ages >= 0) & (ages <= max_age_ns)
    return clipped, valid


def _linear_coverage_mask(
    source_timestamps_ns: np.ndarray,
    grid_ns: np.ndarray,
    tolerance_ns: int,
) -> np.ndarray:
    positions = np.searchsorted(source_timestamps_ns, grid_ns, side="left")
    exact = (positions < len(source_timestamps_ns)) & (
        source_timestamps_ns[np.clip(positions, 0, len(source_timestamps_ns) - 1)]
        == grid_ns
    )
    before = positions - 1
    after = positions
    bracketed = (before >= 0) & (after < len(source_timestamps_ns))
    before_clipped = np.clip(before, 0, len(source_timestamps_ns) - 1)
    after_clipped = np.clip(after, 0, len(source_timestamps_ns) - 1)
    before_delta = grid_ns - source_timestamps_ns[before_clipped]
    after_delta = source_timestamps_ns[after_clipped] - grid_ns
    return exact | (
        bracketed
        & (before_delta >= 0)
        & (after_delta >= 0)
        & (before_delta <= tolerance_ns)
        & (after_delta <= tolerance_ns)
    )


def _longest_true_block(mask: np.ndarray) -> slice | None:
    best_start = 0
    best_stop = 0
    current_start: int | None = None
    for index, value in enumerate(mask):
        if bool(value) and current_start is None:
            current_start = index
        if current_start is not None and (not bool(value) or index == len(mask) - 1):
            stop = index if not bool(value) else index + 1
            if stop - current_start > best_stop - best_start:
                best_start, best_stop = current_start, stop
            current_start = None
    return None if best_stop <= best_start else slice(best_start, best_stop)


def calibrate_run_delay(streams: RunStreams, config: V2ConverterConfig) -> DelayFitResult:
    required = (streams.final_commands, streams.velocities)
    if any(len(values) < 3 for values in required):
        return _invalid_delay_result("insufficient_streams")
    starts = [streams.final_commands[0].timestamp_ns, streams.velocities[0].timestamp_ns]
    ends = [streams.final_commands[-1].timestamp_ns, streams.velocities[-1].timestamp_ns]
    if streams.actual_steering:
        starts.append(streams.actual_steering[0].timestamp_ns)
        ends.append(streams.actual_steering[-1].timestamp_ns)
    start_ns = max(starts)
    end_ns = min(ends)
    step_ns = int(round(1e9 / config.delay_sample_rate_hz))
    if end_ns - start_ns < step_ns * 3:
        return _invalid_delay_result("insufficient_overlap")
    grid_ns = np.arange(start_ns, end_ns + 1, step_ns, dtype=np.int64)
    command_times = np.asarray(
        [item.timestamp_ns for item in streams.final_commands], dtype=np.int64
    )
    velocity_times = np.asarray(
        [item.timestamp_ns for item in streams.velocities], dtype=np.int64
    )
    command_indices, coverage = _hold_coverage_mask(
        command_times,
        grid_ns,
        int(round(config.command_max_age_ms * 1e6)),
    )
    tolerance_ns = int(round(config.interpolation_tolerance_ms * 1e6))
    coverage &= _linear_coverage_mask(velocity_times, grid_ns, tolerance_ns)
    steering_times: np.ndarray | None = None
    if streams.actual_steering:
        steering_times = np.asarray(
            [item.timestamp_ns for item in streams.actual_steering], dtype=np.int64
        )
        coverage &= _linear_coverage_mask(steering_times, grid_ns, tolerance_ns)
    block = _longest_true_block(coverage)
    minimum_fit_samples = max(3, config.delay_config.minimum_dynamic_samples)
    if block is None or block.stop - block.start < minimum_fit_samples:
        return _invalid_delay_result("insufficient_contiguous_coverage")
    grid_ns = grid_ns[block]
    command_indices = command_indices[block]
    timestamps_sec = (grid_ns - grid_ns[0]).astype(np.float64) / 1e9
    command = np.asarray(
        [streams.final_commands[index].steering_rad for index in command_indices],
        dtype=np.float64,
    )
    speed = np.interp(
        grid_ns,
        velocity_times,
        [item.longitudinal_mps for item in streams.velocities],
    )
    yaw = np.interp(
        grid_ns,
        velocity_times,
        [item.yaw_rate_rps for item in streams.velocities],
    )
    if streams.actual_steering:
        assert steering_times is not None
        actual = np.interp(
            grid_ns,
            steering_times,
            [item.steering_rad for item in streams.actual_steering],
        )
        return estimate_steering_delay(
            timestamps_sec,
            command,
            actual,
            speed,
            yaw,
            wheelbase_m=config.wheelbase_m,
            config=config.delay_config,
        )
    return estimate_combined_yaw_delay(
        timestamps_sec,
        command,
        speed,
        yaw,
        wheelbase_m=config.wheelbase_m,
        config=config.delay_config,
    )


def _scenario_id(bag_dir: Path) -> str:
    candidates = (
        bag_dir / "recording_manifest.yaml",
        bag_dir.parent / f"{bag_dir.name}.recording_manifest.yaml",
    )
    for path in candidates:
        if path.is_file():
            payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            value = payload.get("scenario_id")
            if value:
                return str(value)
    return bag_dir.name


def _safe_id(value: str) -> str:
    result = "".join(character if character.isalnum() or character in "-_" else "_" for character in value)
    if not result:
        raise ValueError(f"Cannot derive a safe id from {value!r}")
    return result


def _write_prepared_run(output_root: Path, result: PreparedRunResult, jpeg_quality: int) -> None:
    for sample in result.samples:
        image_path = output_root / sample.row["image_path"]
        lidar_path = output_root / sample.row["lidar_path"]
        valid_path = output_root / sample.row["lidar_valid_path"]
        image_path.parent.mkdir(parents=True, exist_ok=True)
        lidar_path.parent.mkdir(parents=True, exist_ok=True)
        valid_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(sample.image_rgb, mode="RGB").save(image_path, quality=jpeg_quality)
        np.save(lidar_path, sample.lidar_ranges_m, allow_pickle=False)
        np.save(valid_path, sample.lidar_valid, allow_pickle=False)


def convert_dataset_v2(
    input_root: Path,
    output_root: Path,
    config: V2ConverterConfig,
    *,
    max_runs: int | None = None,
    max_samples_per_run: int | None = None,
    val_ratio: float = 0.2,
    test_ratio: float = 0.2,
    split_seed: int = 42,
    vehicle_config_provenance: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Convert new multi-topic bags without touching the format-v1 dataset."""

    config.validate()
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    bag_dirs = discover_bag_directories(input_root)
    if max_runs is not None:
        if max_runs <= 0:
            raise ValueError("max_runs must be positive")
        bag_dirs = bag_dirs[:max_runs]
    results: list[PreparedRunResult] = []
    topic_types: dict[str, str] = {}
    fallback_by_run: dict[str, Mapping[str, int]] = {}
    for bag_dir in bag_dirs:
        run_id = _safe_id(bag_dir.name)
        streams = read_run_messages_v2(bag_dir)
        result = prepare_run_samples(
            streams,
            run_id=run_id,
            scenario_id=_scenario_id(bag_dir),
            config=config,
            max_samples=max_samples_per_run,
        )
        if not result.samples:
            raise RuntimeError(f"Run {run_id} produced no valid Dataset v2 samples")
        result = replace(result, delay_calibration=calibrate_run_delay(streams, config))
        if results and result.lidar_geometry != results[0].lidar_geometry:
            raise ValueError("Native LiDAR geometry differs across runs")
        _write_prepared_run(output_root, result, config.jpeg_quality)
        results.append(result)
        topic_types.update(streams.topic_types)
        fallback_by_run[run_id] = streams.timestamp_fallback_counts
    all_samples = [sample for result in results for sample in result.samples]
    if not all_samples:
        raise RuntimeError("Conversion produced no valid samples")
    split_mapping = assign_run_splits(
        [result.run_id for result in results],
        val_ratio=val_ratio,
        test_ratio=test_ratio,
        seed=split_seed,
    )
    rows: list[dict[str, Any]] = []
    for sample in all_samples:
        row = dict(sample.row)
        row["split"] = split_mapping[row["run_id"]]
        rows.append(row)
    frame = pd.DataFrame(rows).sort_values(["run_id", "timestamp_ns"])
    frame.to_csv(output_root / "index.csv", index=False)
    for split in ("train", "val", "test"):
        frame[frame["split"] == split].to_csv(
            output_root / f"{split}_index.csv", index=False
        )
    delay_results = [
        result.delay_calibration
        for result in results
        if result.delay_calibration is not None
    ]
    delay_consistency: DelayConsistencyAssessment = assess_delay_consistency(
        delay_results, minimum_runs=5, max_deviation_sec=0.10
    )
    run_rates = [result.quality_metrics["effective_sample_rate_hz"] for result in results]
    lidar_skews = [abs(float(row["lidar_dt_ms"])) for row in rows]
    eligible_total = sum(
        int(result.quality_metrics["eligible_candidate_count"]) for result in results
    )
    missing_total = sum(result.dropped_counts["pose_velocity_sync"] for result in results)
    missing_fraction = missing_total / eligible_total if eligible_total else 1.0
    dataset_gates = {
        "minimum_five_runs": len(results) >= 5,
        "effective_sample_rate_9_8_to_10_2_hz": all(
            9.8 <= float(value) <= 10.2 for value in run_rates
        ),
        "camera_lidar_p95_skew_below_30_ms": float(np.percentile(lidar_skews, 95)) < 30.0,
        "pose_velocity_missing_fraction_below_0_01": missing_fraction < 0.01,
        "measured_pose_waypoints": all(
            row["label_provenance"] == "measured_pose" for row in rows
        ),
        "run_split_overlap_absent": bool(
            set(frame["run_id"]) == set(split_mapping)
            and (frame.groupby("run_id")["split"].nunique() == 1).all()
        ),
        "delay_metadata_valid": delay_consistency.dataset_valid,
    }
    metadata: dict[str, Any] = {
        "format_version": DATASET_FORMAT_VERSION_V2,
        "source_root": str(input_root.resolve()),
        "topics": {
            contract.role: {
                "name": contract.name,
                "expected_type": contract.message_type,
                "recorded_type": topic_types.get(contract.name),
                "required_for_conversion": contract.required_for_conversion,
            }
            for contract in DATASET_V2_TOPICS
        },
        "converter": asdict(config),
        "vehicle_config_provenance": dict(vehicle_config_provenance or {}),
        "coordinate_contract": {
            "waypoint_frame": "observation_ego",
            "x_axis": "forward",
            "y_axis": "left",
            "pose_interpolation": "linear_xy_shortest_arc_yaw",
            "distance_unit": "m",
            "angle_unit": "rad",
            "time_unit": "s",
        },
        "input_provenance": {
            "camera": "sensor_msgs/Image header stamp",
            "lidar": "native sensor_msgs/LaserScan header stamp and beam order",
            "pose": "nav_msgs/Odometry measured/global pose",
            "velocity": "autoware_auto_vehicle_msgs/VelocityReport measured state",
            "ego_model_input": "measured longitudinal velocity only",
        },
        "teacher_debug_only_provenance": {
            "future_waypoints": "measured future pose transformed into observation ego frame",
            "target_speed": f"measured longitudinal velocity at observation+{config.target_speed_offset_sec}s",
            "teacher_command": "nominal command retained for debug only; excluded from model inputs/loss",
            "stop_label": "absent; no intentional-stop annotations",
        },
        "timestamp_fallback_counts": fallback_by_run,
        "lidar_geometry": results[0].lidar_geometry,
        "split_policy": {
            "unit": "run_id",
            "seed": split_seed,
            "val_ratio": val_ratio,
            "test_ratio": test_ratio,
            "assignments": split_mapping,
        },
        "rows": int(len(frame)),
        "split_rows": {
            split: int((frame["split"] == split).sum())
            for split in ("train", "val", "test")
        },
        "dataset_quality": {
            "run_effective_sample_rate_hz": run_rates,
            "camera_lidar_p95_skew_ms": float(np.percentile(lidar_skews, 95)),
            "pose_velocity_missing_fraction": missing_fraction,
            "gates": dataset_gates,
            "all_gates_pass": all(dataset_gates.values()),
        },
        "delay_consistency": delay_consistency.to_dict(),
        "runs": [
            {
                "run_id": result.run_id,
                "scenario_id": result.scenario_id,
                "input_counts": result.input_counts,
                "dropped_counts": result.dropped_counts,
                "quality_metrics": result.quality_metrics,
                "delay_calibration": (
                    result.delay_calibration.to_dict()
                    if result.delay_calibration is not None
                    else None
                ),
            }
            for result in results
        ],
    }
    (output_root / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return metadata


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
