from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class TimedScalarV3:
    time_sec: float
    value: float


@dataclass(frozen=True)
class TimedPlanDiagnosticV3:
    time_sec: float
    preflight_ready: bool
    commanded_speed_mps: float
    acceleration_mps2: float
    controller_state: str
    fault_reason: str | None
    stop_required: bool
    decision_reasons: tuple[str, ...]
    preflight_reasons: tuple[str, ...]


@dataclass(frozen=True)
class TimedPose2DV3:
    """Planar pose in the map frame at a ROS timestamp (m, rad, s)."""

    time_sec: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class TimedTrajectoryPredictionV3:
    """Trajectory Head output expressed in the observation-time ego frame."""

    observation_time_sec: float
    waypoint_times_sec: tuple[float, ...]
    trajectory_xy_m: tuple[tuple[float, float], ...]


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = (len(ordered) - 1) * percentile
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _interpolate_pose(
    poses: Sequence[TimedPose2DV3], target_time_sec: float
) -> TimedPose2DV3 | None:
    if not poses or target_time_sec < poses[0].time_sec or target_time_sec > poses[-1].time_sec:
        return None
    for index, right in enumerate(poses):
        if right.time_sec < target_time_sec:
            continue
        if index == 0 or math.isclose(right.time_sec, target_time_sec, abs_tol=1e-9):
            return right
        left = poses[index - 1]
        width = right.time_sec - left.time_sec
        if width <= 0.0:
            return right
        fraction = (target_time_sec - left.time_sec) / width
        yaw_delta = math.atan2(
            math.sin(right.yaw_rad - left.yaw_rad),
            math.cos(right.yaw_rad - left.yaw_rad),
        )
        return TimedPose2DV3(
            time_sec=target_time_sec,
            x_m=left.x_m + fraction * (right.x_m - left.x_m),
            y_m=left.y_m + fraction * (right.y_m - left.y_m),
            yaw_rad=left.yaw_rad + fraction * yaw_delta,
        )
    return None


def summarize_trajectory_tracking_v3(
    *,
    predictions: Sequence[TimedTrajectoryPredictionV3],
    poses: Sequence[TimedPose2DV3],
    horizon_sec: float = 1.0,
) -> dict[str, object]:
    """Compare a fixed-horizon Trajectory Head point with future odometry.

    Predicted points and the measured displacement are both expressed in the
    ego frame at ``observation_time_sec``.  Records without bracketing odometry
    or without the requested horizon are excluded and reported as coverage.
    """

    if not math.isfinite(horizon_sec) or horizon_sec <= 0.0:
        raise ValueError("tracking horizon_sec must be finite and positive")
    previous_pose_time = -math.inf
    for pose in poses:
        values = (pose.time_sec, pose.x_m, pose.y_m, pose.yaw_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("tracking poses must be finite")
        if pose.time_sec < previous_pose_time:
            raise ValueError("tracking poses must be time ordered")
        previous_pose_time = pose.time_sec

    longitudinal_errors: list[float] = []
    lateral_errors: list[float] = []
    euclidean_errors: list[float] = []
    for prediction in predictions:
        if not math.isfinite(prediction.observation_time_sec):
            raise ValueError("prediction observation time must be finite")
        times = prediction.waypoint_times_sec
        points = prediction.trajectory_xy_m
        if len(times) != len(points) or len(times) < 2:
            raise ValueError("prediction times and points must have equal length >= 2")
        if any(not math.isfinite(value) for value in times):
            raise ValueError("prediction waypoint times must be finite")
        if any(right <= left for left, right in zip(times, times[1:])):
            raise ValueError("prediction waypoint times must be strictly increasing")
        if any(
            len(point) != 2 or not all(math.isfinite(value) for value in point)
            for point in points
        ):
            raise ValueError("prediction trajectory points must be finite xy pairs")
        if horizon_sec < times[0] or horizon_sec > times[-1]:
            continue
        right_index = next(index for index, value in enumerate(times) if value >= horizon_sec)
        if right_index == 0:
            predicted_x, predicted_y = points[0]
        else:
            left_time = times[right_index - 1]
            right_time = times[right_index]
            fraction = (horizon_sec - left_time) / (right_time - left_time)
            predicted_x = points[right_index - 1][0] + fraction * (
                points[right_index][0] - points[right_index - 1][0]
            )
            predicted_y = points[right_index - 1][1] + fraction * (
                points[right_index][1] - points[right_index - 1][1]
            )
        start = _interpolate_pose(poses, prediction.observation_time_sec)
        end = _interpolate_pose(poses, prediction.observation_time_sec + horizon_sec)
        if start is None or end is None:
            continue
        delta_x = end.x_m - start.x_m
        delta_y = end.y_m - start.y_m
        cosine = math.cos(start.yaw_rad)
        sine = math.sin(start.yaw_rad)
        actual_x = cosine * delta_x + sine * delta_y
        actual_y = -sine * delta_x + cosine * delta_y
        longitudinal = actual_x - predicted_x
        lateral = actual_y - predicted_y
        longitudinal_errors.append(abs(longitudinal))
        lateral_errors.append(abs(lateral))
        euclidean_errors.append(math.hypot(longitudinal, lateral))

    total = len(predictions)
    matched = len(euclidean_errors)
    return {
        "tracking_horizon_sec": horizon_sec,
        "tracking_prediction_count": total,
        "tracking_matched_count": matched,
        "tracking_coverage_ratio": None if total == 0 else matched / total,
        "tracking_longitudinal_abs_error_p50_m": _percentile(longitudinal_errors, 0.50),
        "tracking_longitudinal_abs_error_p95_m": _percentile(longitudinal_errors, 0.95),
        "tracking_lateral_abs_error_p50_m": _percentile(lateral_errors, 0.50),
        "tracking_lateral_abs_error_p95_m": _percentile(lateral_errors, 0.95),
        "tracking_euclidean_error_p50_m": _percentile(euclidean_errors, 0.50),
        "tracking_euclidean_error_p95_m": _percentile(euclidean_errors, 0.95),
    }


def _validate_timed_scalars(samples: Sequence[TimedScalarV3]) -> None:
    previous = -math.inf
    for sample in samples:
        if not math.isfinite(sample.time_sec) or not math.isfinite(sample.value):
            raise ValueError("timed scalar samples must be finite")
        if sample.time_sec < previous:
            raise ValueError("timed scalar samples must be time ordered")
        previous = sample.time_sec


def summarize_m3_interval_v3(
    *,
    arm_start_sec: float,
    arm_end_sec: float,
    velocity_mps: Sequence[TimedScalarV3],
    yaw_rate_rps: Sequence[TimedScalarV3],
    plans: Sequence[TimedPlanDiagnosticV3],
    safety_reasons: Iterable[str],
    displacement_m: float | None,
    collision_topic_present: bool,
    collision_true_count: int,
    speed_cap_mps: float,
    speed_tolerance_mps: float = 0.10,
    launch_threshold_mps: float = 0.10,
    launch_timeout_sec: float = 3.0,
    turn_rate_threshold_rps: float = 0.05,
) -> dict[str, object]:
    """Summarize one officially armed M3 interval in SI units.

    Collision absence is only accepted when the collision topic was present.
    A silent observer without a matched publisher is reported as unverified.
    """

    finite_parameters = (
        arm_start_sec,
        arm_end_sec,
        speed_cap_mps,
        speed_tolerance_mps,
        launch_threshold_mps,
        launch_timeout_sec,
        turn_rate_threshold_rps,
    )
    if not all(math.isfinite(value) for value in finite_parameters):
        raise ValueError("M3 interval parameters must be finite")
    if arm_end_sec <= arm_start_sec:
        raise ValueError("arm_end_sec must be greater than arm_start_sec")
    if speed_cap_mps <= 0.0 or speed_tolerance_mps < 0.:
        raise ValueError("speed cap must be positive and tolerance non-negative")
    if launch_threshold_mps <= 0.0 or launch_timeout_sec <= 0.0:
        raise ValueError("launch threshold and timeout must be positive")
    if turn_rate_threshold_rps <= 0.0:
        raise ValueError("turn_rate_threshold_rps must be positive")
    if collision_true_count < 0:
        raise ValueError("collision_true_count must be non-negative")
    if displacement_m is not None and (
        not math.isfinite(displacement_m) or displacement_m < 0.0
    ):
        raise ValueError("displacement_m must be finite and non-negative")
    _validate_timed_scalars(velocity_mps)
    _validate_timed_scalars(yaw_rate_rps)

    velocities = [
        sample for sample in velocity_mps
        if arm_start_sec <= sample.time_sec <= arm_end_sec
    ]
    yaw_rates = [
        sample.value for sample in yaw_rate_rps
        if arm_start_sec <= sample.time_sec <= arm_end_sec
    ]
    interval_plans = [
        sample for sample in plans
        if arm_start_sec <= sample.time_sec <= arm_end_sec
    ]
    if not velocities:
        raise ValueError("armed interval contains no velocity samples")
    if not interval_plans:
        raise ValueError("armed interval contains no plan diagnostics")

    launch_sample = next(
        (sample for sample in velocities if sample.value >= launch_threshold_mps),
        None,
    )
    launch_latency_sec = (
        None if launch_sample is None else launch_sample.time_sec - arm_start_sec
    )
    max_speed_mps = max(sample.value for sample in velocities)
    preflight_ready_ratio = sum(
        sample.preflight_ready for sample in interval_plans
    ) / len(interval_plans)
    fault_counts = Counter(
        sample.fault_reason for sample in interval_plans if sample.fault_reason
    )
    controller_state_counts = Counter(
        sample.controller_state for sample in interval_plans
    )
    decision_reason_counts = Counter(
        reason for sample in interval_plans for reason in sample.decision_reasons
    )
    preflight_reason_counts = Counter(
        reason for sample in interval_plans for reason in sample.preflight_reasons
    )
    safety_counts = Counter(str(reason) for reason in safety_reasons)
    safety_total = sum(safety_counts.values())
    safety_normal_ratio = (
        None if safety_total == 0 else safety_counts.get("normal", 0) / safety_total
    )
    straight_samples = sum(
        abs(rate) < turn_rate_threshold_rps for rate in yaw_rates
    )
    left_samples = sum(rate >= turn_rate_threshold_rps for rate in yaw_rates)
    right_samples = sum(rate <= -turn_rate_threshold_rps for rate in yaw_rates)
    collision_clear: bool | None = (
        collision_true_count == 0 if collision_topic_present else None
    )

    return {
        "duration_sec": arm_end_sec - arm_start_sec,
        "launch_latency_sec": launch_latency_sec,
        "launch_pass": (
            launch_latency_sec is not None
            and launch_latency_sec <= launch_timeout_sec
        ),
        "maximum_speed_mps": max_speed_mps,
        "final_speed_mps": velocities[-1].value,
        "speed_cap_pass": max_speed_mps <= speed_cap_mps + speed_tolerance_mps,
        "preflight_ready_ratio": preflight_ready_ratio,
        "controller_fault_counts": dict(sorted(fault_counts.items())),
        "controller_state_counts": dict(sorted(controller_state_counts.items())),
        "stop_required_count": sum(sample.stop_required for sample in interval_plans),
        "decision_reason_counts": dict(sorted(decision_reason_counts.items())),
        "preflight_reason_counts": dict(sorted(preflight_reason_counts.items())),
        "zero_commanded_speed_count": sum(
            sample.commanded_speed_mps <= 1e-6 for sample in interval_plans
        ),
        "maximum_commanded_speed_mps": max(
            sample.commanded_speed_mps for sample in interval_plans
        ),
        "minimum_acceleration_mps2": min(
            sample.acceleration_mps2 for sample in interval_plans
        ),
        "maximum_acceleration_mps2": max(
            sample.acceleration_mps2 for sample in interval_plans
        ),
        "safety_reason_counts": dict(sorted(safety_counts.items())),
        "safety_normal_ratio": safety_normal_ratio,
        "straight_sample_count": straight_samples,
        "left_turn_sample_count": left_samples,
        "right_turn_sample_count": right_samples,
        "displacement_m": displacement_m,
        "collision_topic_present": collision_topic_present,
        "collision_true_count": collision_true_count,
        "collision_clear": collision_clear,
    }
