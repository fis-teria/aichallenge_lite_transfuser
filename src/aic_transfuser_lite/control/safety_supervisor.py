from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np

from .waypoint_controller import ControlCommand


@dataclass(frozen=True)
class SafetyConfig:
    front_angle_deg: float = 15.0
    latency_sec: float = 0.2
    brake_accel_mps2: float = 3.0
    margin_m: float = 0.5
    enable_model_stop: bool = False
    stop_prob_threshold: float = 0.6
    confidence_threshold: float = 0.3
    max_steer_rad: float = 0.6
    max_accel_mps2: float = 2.0
    min_accel_mps2: float = -4.0
    camera_timeout_sec: float = 0.3
    lidar_timeout_sec: float = 0.2
    ego_timeout_sec: float = 0.2


@dataclass(frozen=True)
class SensorStamps:
    camera_sec: float
    lidar_sec: float
    ego_sec: float


@dataclass(frozen=True)
class SafetyDecision:
    command: ControlCommand
    overridden: bool
    reason: str
    front_distance_m: float
    stopping_distance_m: float


def stopping_distance(
    speed_mps: float,
    latency_sec: float,
    brake_accel_mps2: float,
    margin_m: float,
) -> float:
    speed = max(float(speed_mps), 0.0)
    decel = abs(float(brake_accel_mps2))
    if decel <= 0.0:
        raise ValueError("brake_accel_mps2 must be non-zero")
    return speed * latency_sec + speed * speed / (2.0 * decel) + margin_m


def min_front_distance(
    ranges_m: np.ndarray,
    angle_min_rad: float,
    angle_increment_rad: float,
    front_angle_deg: float,
) -> float:
    ranges = np.asarray(ranges_m, dtype=np.float32)
    if ranges.ndim != 1 or ranges.size == 0:
        raise ValueError(f"Expected non-empty ranges [P], got {ranges.shape}")
    angles = angle_min_rad + np.arange(ranges.size) * angle_increment_rad
    half = math.radians(front_angle_deg)
    mask = (np.abs(angles) <= half) & np.isfinite(ranges) & (ranges > 0.0)
    if not np.any(mask):
        return float("inf")
    return float(np.min(ranges[mask]))


def _is_stale(now_sec: float, stamp_sec: float, timeout_sec: float) -> bool:
    return not math.isfinite(stamp_sec) or now_sec - stamp_sec > timeout_sec


def apply_safety(
    nominal: ControlCommand,
    *,
    speed_mps: float,
    lidar_ranges_m: np.ndarray,
    angle_min_rad: float,
    angle_increment_rad: float,
    stop_probability: float | None,
    confidence: float | None,
    stamps: SensorStamps,
    now_sec: float | None = None,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    now = time.time() if now_sec is None else float(now_sec)
    front = min_front_distance(
        lidar_ranges_m,
        angle_min_rad,
        angle_increment_rad,
        config.front_angle_deg,
    )
    d_stop = stopping_distance(
        speed_mps,
        config.latency_sec,
        config.brake_accel_mps2,
        config.margin_m,
    )
    brake = ControlCommand(steering_rad=0.0, acceleration_mps2=config.min_accel_mps2)

    if _is_stale(now, stamps.camera_sec, config.camera_timeout_sec):
        return SafetyDecision(brake, True, "camera_timeout", front, d_stop)
    if _is_stale(now, stamps.lidar_sec, config.lidar_timeout_sec):
        return SafetyDecision(brake, True, "lidar_timeout", front, d_stop)
    if _is_stale(now, stamps.ego_sec, config.ego_timeout_sec):
        return SafetyDecision(brake, True, "ego_timeout", front, d_stop)

    values = [nominal.steering_rad, nominal.acceleration_mps2, speed_mps]
    if config.enable_model_stop:
        if stop_probability is None:
            return SafetyDecision(brake, True, "model_stop_missing", front, d_stop)
        values.append(stop_probability)
    if confidence is not None:
        values.append(confidence)
    if not all(math.isfinite(float(value)) for value in values):
        return SafetyDecision(brake, True, "non_finite_input", front, d_stop)

    if front < d_stop:
        return SafetyDecision(brake, True, "front_obstacle_inside_stopping_distance", front, d_stop)
    if (
        config.enable_model_stop
        and stop_probability is not None
        and stop_probability >= config.stop_prob_threshold
    ):
        return SafetyDecision(brake, True, "model_stop", front, d_stop)

    steering = float(np.clip(nominal.steering_rad, -config.max_steer_rad, config.max_steer_rad))
    acceleration = float(
        np.clip(nominal.acceleration_mps2, config.min_accel_mps2, config.max_accel_mps2)
    )
    overridden = steering != nominal.steering_rad or acceleration != nominal.acceleration_mps2
    reason = "command_clamped" if overridden else "normal"

    if confidence is not None and confidence < config.confidence_threshold:
        acceleration = min(acceleration, 0.0)
        overridden = True
        reason = "low_confidence_deceleration"

    return SafetyDecision(
        ControlCommand(steering_rad=steering, acceleration_mps2=acceleration),
        overridden,
        reason,
        front,
        d_stop,
    )
