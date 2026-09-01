from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .waypoint_controller import ControlCommand


@dataclass(frozen=True)
class DelayAwareControllerConfig:
    waypoint_times_sec: tuple[float, ...]
    estimated_delay_sec: float = 0.0
    base_preview_sec: float = 0.35
    min_preview_sec: float = 0.5
    max_preview_sec: float = 1.2
    wheelbase_m: float = 1.087
    max_steer_rad: float = 0.6
    min_accel_mps2: float = -4.0
    max_accel_mps2: float = 2.0
    speed_kp: float = 1.0
    max_steering_rate_radps: float = 0.0
    control_period_sec: float = 0.1
    small_yaw_rate_rps: float = 1e-4
    minimum_target_x_m: float = 1e-3

    def __post_init__(self) -> None:
        times = tuple(float(value) for value in self.waypoint_times_sec)
        if not times or not all(math.isfinite(value) and value > 0.0 for value in times):
            raise ValueError("waypoint_times_sec must contain finite positive values")
        if any(right <= left for left, right in zip(times, times[1:])):
            raise ValueError("waypoint_times_sec must be strictly increasing")
        if not math.isfinite(self.estimated_delay_sec) or self.estimated_delay_sec < 0.0:
            raise ValueError("estimated_delay_sec must be finite and non-negative")
        if not (
            math.isfinite(self.base_preview_sec)
            and math.isfinite(self.min_preview_sec)
            and math.isfinite(self.max_preview_sec)
            and 0.0 < self.min_preview_sec <= self.max_preview_sec
        ):
            raise ValueError("preview bounds must be finite, positive, and ordered")
        if not math.isfinite(self.wheelbase_m) or self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be finite and positive")
        if not math.isfinite(self.max_steer_rad) or self.max_steer_rad <= 0.0:
            raise ValueError("max_steer_rad must be finite and positive")
        if not self.min_accel_mps2 < self.max_accel_mps2:
            raise ValueError("acceleration limits must be ordered")
        if not math.isfinite(self.speed_kp) or self.speed_kp <= 0.0:
            raise ValueError("speed_kp must be finite and positive")
        if not math.isfinite(self.max_steering_rate_radps) or self.max_steering_rate_radps < 0.0:
            raise ValueError("max_steering_rate_radps must be finite and non-negative")
        if not math.isfinite(self.control_period_sec) or self.control_period_sec <= 0.0:
            raise ValueError("control_period_sec must be finite and positive")


@dataclass(frozen=True)
class DelayAwareControlResult:
    command: ControlCommand
    commanded_speed_mps: float
    delay_sec: float
    preview_time_sec: float
    preview_target_xy_m: np.ndarray
    projected_waypoints_m: np.ndarray
    curvature_per_m: float
    unlimited_steering_rad: float
    steering_rate_limited: bool


def _finite_waypoints(waypoints: np.ndarray) -> np.ndarray:
    points = np.asarray(waypoints, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError(f"Expected waypoints [N,2], got {points.shape}")
    if not np.isfinite(points).all():
        raise ValueError("waypoints must be finite")
    return points


def project_waypoints_to_future_ego(
    waypoints: np.ndarray,
    *,
    speed_mps: float,
    yaw_rate_rps: float,
    delay_sec: float,
    small_yaw_rate_rps: float = 1e-4,
) -> np.ndarray:
    """Express current-ego waypoints in the ego frame after ``delay_sec``."""

    points = _finite_waypoints(waypoints)
    values = (speed_mps, yaw_rate_rps, delay_sec, small_yaw_rate_rps)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("projection inputs must be finite")
    if delay_sec < 0.0:
        raise ValueError("delay_sec must be non-negative")
    if small_yaw_rate_rps <= 0.0:
        raise ValueError("small_yaw_rate_rps must be positive")
    if delay_sec == 0.0:
        return points.copy()

    delta_yaw = float(yaw_rate_rps) * float(delay_sec)
    if abs(float(yaw_rate_rps)) < float(small_yaw_rate_rps):
        x_delay = float(speed_mps) * float(delay_sec)
        y_delay = 0.0
    else:
        radius = float(speed_mps) / float(yaw_rate_rps)
        x_delay = radius * math.sin(delta_yaw)
        y_delay = radius * (1.0 - math.cos(delta_yaw))
    translated = points.astype(np.float64) - np.asarray([x_delay, y_delay])
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    future_x = cosine * translated[:, 0] + sine * translated[:, 1]
    future_y = -sine * translated[:, 0] + cosine * translated[:, 1]
    return np.stack((future_x, future_y), axis=1).astype(np.float32)


def interpolate_waypoint(
    waypoints: np.ndarray,
    waypoint_times_sec: Sequence[float],
    preview_time_sec: float,
) -> np.ndarray:
    points = _finite_waypoints(waypoints)
    times = np.asarray(tuple(waypoint_times_sec), dtype=np.float64)
    if times.shape != (len(points),):
        raise ValueError("waypoint time count must equal waypoint count")
    if not np.isfinite(times).all() or np.any(times <= 0.0) or np.any(np.diff(times) <= 0.0):
        raise ValueError("waypoint times must be finite, positive, and strictly increasing")
    preview = float(preview_time_sec)
    if not math.isfinite(preview):
        raise ValueError("preview_time_sec must be finite")
    target = np.asarray(
        [
            np.interp(preview, times, points[:, axis])
            for axis in range(2)
        ],
        dtype=np.float32,
    )
    return target


def control_from_waypoints_delay_aware(
    waypoints: np.ndarray,
    *,
    target_speed_mps: float,
    current_longitudinal_speed_mps: float,
    yaw_rate_rps: float,
    actual_steering_rad: float,
    config: DelayAwareControllerConfig,
    delay_override_sec: float | None = None,
) -> DelayAwareControlResult:
    numeric = (
        target_speed_mps,
        current_longitudinal_speed_mps,
        yaw_rate_rps,
        actual_steering_rad,
    )
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError("control inputs must be finite")
    delay = config.estimated_delay_sec if delay_override_sec is None else float(delay_override_sec)
    if not math.isfinite(delay) or delay < 0.0:
        raise ValueError("delay must be finite and non-negative")
    projected = project_waypoints_to_future_ego(
        waypoints,
        speed_mps=current_longitudinal_speed_mps,
        yaw_rate_rps=yaw_rate_rps,
        delay_sec=delay,
        small_yaw_rate_rps=config.small_yaw_rate_rps,
    )
    preview = float(
        np.clip(
            config.base_preview_sec + delay,
            config.min_preview_sec,
            config.max_preview_sec,
        )
    )
    target = interpolate_waypoint(projected, config.waypoint_times_sec, preview)
    x, y = float(target[0]), float(target[1])
    if x <= config.minimum_target_x_m:
        raise ValueError(f"preview target must remain ahead of ego, got x={x}")
    lookahead_sq = x * x + y * y
    if not math.isfinite(lookahead_sq) or lookahead_sq <= 1e-8:
        raise ValueError("preview target lookahead is invalid")
    curvature = 2.0 * y / lookahead_sq
    unlimited = math.atan(config.wheelbase_m * curvature)
    steering = float(np.clip(unlimited, -config.max_steer_rad, config.max_steer_rad))
    rate_limited = False
    if config.max_steering_rate_radps > 0.0:
        maximum_delta = config.max_steering_rate_radps * config.control_period_sec
        limited = float(
            np.clip(
                steering,
                actual_steering_rad - maximum_delta,
                actual_steering_rad + maximum_delta,
            )
        )
        rate_limited = not math.isclose(limited, steering, rel_tol=0.0, abs_tol=1e-12)
        steering = limited

    commanded_speed = max(float(target_speed_mps), 0.0)
    acceleration = config.speed_kp * (
        commanded_speed - float(current_longitudinal_speed_mps)
    )
    acceleration = float(
        np.clip(acceleration, config.min_accel_mps2, config.max_accel_mps2)
    )
    return DelayAwareControlResult(
        command=ControlCommand(steering, acceleration),
        commanded_speed_mps=commanded_speed,
        delay_sec=delay,
        preview_time_sec=preview,
        preview_target_xy_m=target,
        projected_waypoints_m=projected,
        curvature_per_m=curvature,
        unlimited_steering_rad=unlimited,
        steering_rate_limited=rate_limited,
    )
