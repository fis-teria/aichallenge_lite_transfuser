from __future__ import annotations

from dataclasses import dataclass
import math

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
    future_tolerance_sec: float = 0.001
    max_speed_mps: float = 12.0
    speed_limit_guard_margin_mps: float = 0.1
    max_command_validity_sec: float = 0.15
    nominal_timeout_sec: float = 0.3


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


@dataclass(frozen=True)
class CommandEnvelope:
    speed_mps: float
    source_observation_stamp_sec: float
    generated_stamp_sec: float
    valid_until_sec: float


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
        return float("nan")
    return float(np.min(ranges[mask]))


def _is_stale(now_sec: float, stamp_sec: float, timeout_sec: float) -> bool:
    return not math.isfinite(stamp_sec) or now_sec - stamp_sec > timeout_sec


def clamp_command_envelope(
    *,
    proposed_speed_mps: float,
    source_observation_stamp_sec: float,
    generated_stamp_sec: float,
    requested_valid_until_sec: float,
    now_sec: float,
    config: SafetyConfig = SafetyConfig(),
) -> CommandEnvelope:
    """Clamp speed/deadline and reject future or already-late proposals."""
    values = (
        proposed_speed_mps,
        source_observation_stamp_sec,
        generated_stamp_sec,
        requested_valid_until_sec,
        now_sec,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("command envelope values must be finite")
    if source_observation_stamp_sec > now_sec + config.future_tolerance_sec:
        raise ValueError("source_observation_future_timestamp")
    if generated_stamp_sec > now_sec + config.future_tolerance_sec:
        raise ValueError("model_generated_future_timestamp")
    if generated_stamp_sec < source_observation_stamp_sec:
        raise ValueError("model generation precedes source observation")
    deadline = min(
        requested_valid_until_sec,
        generated_stamp_sec + config.max_command_validity_sec,
    )
    if deadline <= now_sec:
        raise TimeoutError("command_deadline_missed")
    return CommandEnvelope(
        speed_mps=float(np.clip(proposed_speed_mps, 0.0, config.max_speed_mps)),
        source_observation_stamp_sec=float(source_observation_stamp_sec),
        generated_stamp_sec=float(generated_stamp_sec),
        valid_until_sec=float(deadline),
    )


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
    if now_sec is None:
        raise ValueError("strict safety path requires explicit now_sec")
    now = float(now_sec)
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

    for role, stamp in (
        ("camera", stamps.camera_sec),
        ("lidar", stamps.lidar_sec),
        ("ego", stamps.ego_sec),
    ):
        if math.isfinite(stamp) and stamp > now + config.future_tolerance_sec:
            return SafetyDecision(brake, True, f"{role}_future_timestamp", front, d_stop)

    if _is_stale(now, stamps.camera_sec, config.camera_timeout_sec):
        return SafetyDecision(brake, True, "camera_timeout", front, d_stop)
    if _is_stale(now, stamps.lidar_sec, config.lidar_timeout_sec):
        return SafetyDecision(brake, True, "lidar_timeout", front, d_stop)
    if _is_stale(now, stamps.ego_sec, config.ego_timeout_sec):
        return SafetyDecision(brake, True, "ego_timeout", front, d_stop)
    if not math.isfinite(front):
        return SafetyDecision(brake, True, "lidar_no_valid_front_beams", front, d_stop)

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
    guard_margin = float(config.speed_limit_guard_margin_mps)
    if not math.isfinite(guard_margin) or guard_margin < 0.0:
        raise ValueError("speed_limit_guard_margin_mps must be finite and non-negative")
    if speed_mps > config.max_speed_mps:
        return SafetyDecision(
            ControlCommand(
                steering_rad=steering,
                acceleration_mps2=config.min_accel_mps2,
            ),
            True,
            "speed_limit_exceeded",
            front,
            d_stop,
        )
    guard_speed_mps = max(config.max_speed_mps - guard_margin, 0.0)
    if nominal.acceleration_mps2 > 0.0 and speed_mps >= guard_speed_mps:
        return SafetyDecision(
            ControlCommand(
                steering_rad=steering,
                acceleration_mps2=config.min_accel_mps2,
            ),
            True,
            "speed_limit_guard",
            front,
            d_stop,
        )
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
