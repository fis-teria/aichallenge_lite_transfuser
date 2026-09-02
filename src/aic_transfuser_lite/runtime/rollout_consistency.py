from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np

from aic_transfuser_lite.data.calibration.artifact import CalibrationArtifact
from aic_transfuser_lite.data.calibration.longitudinal import LongitudinalModeFit

from .control_projection import ProjectedControlSequence


LongitudinalMode = Literal["drive", "brake"]


@dataclass(frozen=True)
class RolloutInitialState:
    """Measured actuator state at ego-frame origin, in SI units."""

    actual_steering_rad: float
    actual_acceleration_mps2: float
    longitudinal_mode: LongitudinalMode = "drive"


@dataclass(frozen=True)
class ActuatorBicycleRollout:
    """Predicted ego path and actuator state at each future command step."""

    trajectory_xy_m: np.ndarray
    heading_rad: np.ndarray
    speed_mps: np.ndarray
    actual_steering_rad: np.ndarray
    actual_acceleration_mps2: np.ndarray
    longitudinal_modes: tuple[LongitudinalMode, ...]


@dataclass(frozen=True)
class ConsistencyThresholds:
    max_position_error_m: float
    max_lateral_error_m: float
    max_heading_error_rad: float
    max_speed_error_mps: float
    max_endpoint_error_m: float

    def validate(self) -> None:
        values = (
            self.max_position_error_m,
            self.max_lateral_error_m,
            self.max_heading_error_rad,
            self.max_speed_error_mps,
            self.max_endpoint_error_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("consistency thresholds must be finite and positive")


@dataclass(frozen=True)
class ConsistencyMetrics:
    mean_position_error_m: float
    max_position_error_m: float
    max_lateral_error_m: float
    max_heading_error_rad: float
    max_speed_error_mps: float
    endpoint_error_m: float
    consistent: bool
    reasons: tuple[str, ...]


def _validate_fit(fit: LongitudinalModeFit) -> None:
    values = (
        fit.pure_delay_sec,
        fit.time_constant_sec,
        fit.gain,
        fit.bias_mps2,
        *fit.valid_speed_range_mps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{fit.mode} calibration contains non-finite values")
    if fit.pure_delay_sec < 0.0 or fit.time_constant_sec <= 0.0 or fit.gain <= 0.0:
        raise ValueError(f"{fit.mode} calibration dynamics are invalid")
    if fit.valid_speed_range_mps[0] > fit.valid_speed_range_mps[1]:
        raise ValueError(f"{fit.mode} calibration speed range is invalid")
    if not fit.individually_valid:
        reasons = ",".join(fit.validity_reasons) or "quality_gate"
        raise ValueError(f"{fit.mode} calibration is not individually valid:{reasons}")


def _validate_calibration(calibration: CalibrationArtifact) -> None:
    steering = calibration.steering
    steering_values = (
        steering.pure_delay_sec,
        steering.time_constant_sec,
        steering.gain,
        steering.bias_rad,
        *steering.valid_speed_range_mps,
    )
    if not all(math.isfinite(value) for value in steering_values):
        raise ValueError("steering calibration contains non-finite values")
    if (
        steering.pure_delay_sec < 0.0
        or steering.time_constant_sec <= 0.0
        or steering.gain <= 0.0
        or steering.valid_speed_range_mps[0] > steering.valid_speed_range_mps[1]
    ):
        raise ValueError("steering calibration dynamics are invalid")
    if not steering.individually_valid:
        reasons = ",".join(steering.validity_reasons) or "quality_gate"
        raise ValueError(f"steering calibration is not individually valid:{reasons}")
    _validate_fit(calibration.drive)
    _validate_fit(calibration.brake)


def _delayed_command(
    values: np.ndarray,
    index: int,
    *,
    delay_sec: float,
    dt_sec: float,
    initial_value: float,
) -> float:
    source_position = index - delay_sec / dt_sec
    if source_position < 0.0:
        return initial_value
    lower = int(math.floor(source_position))
    upper = min(lower + 1, index)
    fraction = source_position - lower
    return float(values[lower] * (1.0 - fraction) + values[upper] * fraction)


def _first_order_step(
    current: float, target: float, *, dt_sec: float, time_constant_sec: float
) -> float:
    alpha = 1.0 - math.exp(-dt_sec / time_constant_sec)
    return current + alpha * (target - current)


def _require_in_range(name: str, value: float, interval: tuple[float, float]) -> None:
    if not interval[0] <= value <= interval[1]:
        raise ValueError(
            f"{name} {value:.6f} is outside calibration applicability "
            f"[{interval[0]:.6f},{interval[1]:.6f}]"
        )


def rollout_actuator_bicycle(
    sequence: ProjectedControlSequence,
    *,
    calibration: CalibrationArtifact,
    wheelbase_m: float,
    initial: RolloutInitialState,
    mode_hysteresis_mps2: float = 0.1,
) -> ActuatorBicycleRollout:
    """Roll out bounded ``[H,3]`` commands through calibrated actuators and bicycle."""

    _validate_calibration(calibration)
    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be finite and positive")
    if not math.isfinite(mode_hysteresis_mps2) or mode_hysteresis_mps2 < 0.0:
        raise ValueError("mode_hysteresis_mps2 must be finite and non-negative")
    if initial.longitudinal_mode not in ("drive", "brake"):
        raise ValueError("initial longitudinal mode must be drive or brake")
    if not math.isfinite(initial.actual_steering_rad) or not math.isfinite(
        initial.actual_acceleration_mps2
    ):
        raise ValueError("initial actuator state must be finite")
    commands = np.asarray(sequence.commands, dtype=np.float64)
    if commands.ndim != 2 or commands.shape[0] < 1 or commands.shape[1] != 3:
        raise ValueError(f"projected commands must be [H,3], got {commands.shape}")
    if not np.isfinite(commands).all():
        raise ValueError("projected commands must be finite")
    if not math.isfinite(sequence.dt_sec) or sequence.dt_sec <= 0.0:
        raise ValueError("projected command dt_sec must be finite and positive")

    horizon = commands.shape[0]
    xy = np.empty((horizon, 2), dtype=np.float64)
    headings = np.empty(horizon, dtype=np.float64)
    speeds = np.empty(horizon, dtype=np.float64)
    steering_actual = np.empty(horizon, dtype=np.float64)
    acceleration_actual = np.empty(horizon, dtype=np.float64)
    modes: list[LongitudinalMode] = []

    dt_sec = sequence.dt_sec
    steering_commands = commands[:, 0]
    acceleration_commands = commands[:, 2]
    x_m = 0.0
    y_m = 0.0
    yaw_rad = 0.0
    speed_mps = sequence.initial_state.speed_mps
    if not math.isfinite(speed_mps) or speed_mps < 0.0:
        raise ValueError("initial longitudinal speed must be finite and non-negative")
    actual_steering = initial.actual_steering_rad
    actual_acceleration = initial.actual_acceleration_mps2
    mode = initial.longitudinal_mode

    for index in range(horizon):
        command_speed = float(commands[index, 1])
        _require_in_range(
            "steering speed",
            command_speed,
            calibration.steering.valid_speed_range_mps,
        )
        command_acceleration = float(acceleration_commands[index])
        if command_acceleration > mode_hysteresis_mps2:
            mode = "drive"
        elif command_acceleration < -mode_hysteresis_mps2:
            mode = "brake"
        fit = calibration.drive if mode == "drive" else calibration.brake
        _require_in_range(f"{mode} speed", command_speed, fit.valid_speed_range_mps)

        x_m += dt_sec * speed_mps * math.cos(yaw_rad)
        y_m += dt_sec * speed_mps * math.sin(yaw_rad)
        yaw_rad += dt_sec * speed_mps / wheelbase_m * math.tan(actual_steering)
        speed_mps = max(0.0, speed_mps + dt_sec * actual_acceleration)

        delayed_steering = _delayed_command(
            steering_commands,
            index,
            delay_sec=calibration.steering.pure_delay_sec,
            dt_sec=dt_sec,
            initial_value=sequence.initial_state.steering_rad,
        )
        steering_target = (
            calibration.steering.gain * delayed_steering
            + calibration.steering.bias_rad
        )
        actual_steering = _first_order_step(
            actual_steering,
            steering_target,
            dt_sec=dt_sec,
            time_constant_sec=calibration.steering.time_constant_sec,
        )
        delayed_acceleration = _delayed_command(
            acceleration_commands,
            index,
            delay_sec=fit.pure_delay_sec,
            dt_sec=dt_sec,
            initial_value=sequence.initial_state.acceleration_mps2,
        )
        acceleration_target = fit.gain * delayed_acceleration + fit.bias_mps2
        actual_acceleration = _first_order_step(
            actual_acceleration,
            acceleration_target,
            dt_sec=dt_sec,
            time_constant_sec=fit.time_constant_sec,
        )

        xy[index] = (x_m, y_m)
        headings[index] = yaw_rad
        speeds[index] = speed_mps
        steering_actual[index] = actual_steering
        acceleration_actual[index] = actual_acceleration
        modes.append(mode)
    return ActuatorBicycleRollout(
        trajectory_xy_m=xy,
        heading_rad=headings,
        speed_mps=speeds,
        actual_steering_rad=steering_actual,
        actual_acceleration_mps2=acceleration_actual,
        longitudinal_modes=tuple(modes),
    )


def _reference_headings(trajectory_xy_m: np.ndarray) -> np.ndarray:
    previous = np.vstack((np.zeros((1, 2), dtype=np.float64), trajectory_xy_m[:-1]))
    differences = trajectory_xy_m - previous
    headings = np.zeros(trajectory_xy_m.shape[0], dtype=np.float64)
    last_heading = 0.0
    for index, difference in enumerate(differences):
        if float(np.linalg.norm(difference)) > 1e-9:
            last_heading = math.atan2(float(difference[1]), float(difference[0]))
        headings[index] = last_heading
    return headings


def _wrapped_angle_difference(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(left - right), np.cos(left - right))


def evaluate_rollout_consistency(
    model_trajectory_xy_m: np.ndarray,
    model_speed_profile_mps: np.ndarray,
    rollout: ActuatorBicycleRollout,
    *,
    thresholds: ConsistencyThresholds,
) -> ConsistencyMetrics:
    """Compare the same selected model trajectory with its control rollout."""

    thresholds.validate()
    trajectory = np.asarray(model_trajectory_xy_m, dtype=np.float64)
    model_speeds = np.asarray(model_speed_profile_mps, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[0] < 2 or trajectory.shape[1] != 2:
        raise ValueError(f"model trajectory must be [N,2] with N>=2, got {trajectory.shape}")
    if model_speeds.shape != (trajectory.shape[0],):
        raise ValueError("model speed profile must be [N] matching trajectory")
    expected_vector_shape = (trajectory.shape[0],)
    if rollout.trajectory_xy_m.shape != trajectory.shape:
        raise ValueError("rollout trajectory shape does not match model trajectory")
    if rollout.heading_rad.shape != expected_vector_shape or rollout.speed_mps.shape != expected_vector_shape:
        raise ValueError("rollout heading/speed shape does not match model trajectory")
    arrays = (
        trajectory,
        model_speeds,
        rollout.trajectory_xy_m,
        rollout.heading_rad,
        rollout.speed_mps,
    )
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError("consistency inputs must be finite")
    if bool((model_speeds < 0.0).any()) or bool((rollout.speed_mps < 0.0).any()):
        raise ValueError("consistency speeds must be non-negative")

    reference_heading = _reference_headings(trajectory)
    displacement = rollout.trajectory_xy_m - trajectory
    position_error = np.linalg.norm(displacement, axis=1)
    lateral_error = np.abs(
        -np.sin(reference_heading) * displacement[:, 0]
        + np.cos(reference_heading) * displacement[:, 1]
    )
    heading_error = np.abs(
        _wrapped_angle_difference(rollout.heading_rad, reference_heading)
    )
    speed_error = np.abs(rollout.speed_mps - model_speeds)
    endpoint_error = float(position_error[-1])
    metrics = {
        "max_position_error_m": float(np.max(position_error)),
        "max_lateral_error_m": float(np.max(lateral_error)),
        "max_heading_error_rad": float(np.max(heading_error)),
        "max_speed_error_mps": float(np.max(speed_error)),
        "endpoint_error_m": endpoint_error,
    }
    gates = (
        ("max_position_error_m", thresholds.max_position_error_m),
        ("max_lateral_error_m", thresholds.max_lateral_error_m),
        ("max_heading_error_rad", thresholds.max_heading_error_rad),
        ("max_speed_error_mps", thresholds.max_speed_error_mps),
        ("endpoint_error_m", thresholds.max_endpoint_error_m),
    )
    reasons = tuple(
        f"{name}>{limit:.6f}"
        for name, limit in gates
        if metrics[name] > limit
    )
    return ConsistencyMetrics(
        mean_position_error_m=float(np.mean(position_error)),
        max_position_error_m=metrics["max_position_error_m"],
        max_lateral_error_m=metrics["max_lateral_error_m"],
        max_heading_error_rad=metrics["max_heading_error_rad"],
        max_speed_error_mps=metrics["max_speed_error_mps"],
        endpoint_error_m=metrics["endpoint_error_m"],
        consistent=not reasons,
        reasons=reasons,
    )
