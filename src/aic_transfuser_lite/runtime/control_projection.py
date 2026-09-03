from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PreviousControlState:
    """Measured command state in rad, m/s, and m/s^2."""

    steering_rad: float
    speed_mps: float
    acceleration_mps2: float


@dataclass(frozen=True)
class ControlLimits:
    """Authoritative SI-unit bounds required by the full-control decoder."""

    max_abs_steering_rad: float
    max_steering_rate_radps: float
    min_acceleration_mps2: float
    max_acceleration_mps2: float
    min_jerk_mps3: float
    max_jerk_mps3: float
    max_speed_mps: float
    dt_sec: float
    authoritative: bool
    source: str

    def validate_for_full_control(self) -> None:
        values = (
            self.max_abs_steering_rad,
            self.max_steering_rate_radps,
            self.min_acceleration_mps2,
            self.max_acceleration_mps2,
            self.min_jerk_mps3,
            self.max_jerk_mps3,
            self.max_speed_mps,
            self.dt_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("control limits must be finite")
        if not self.authoritative or not self.source.strip():
            raise ValueError("authoritative control limits are required")
        if self.max_abs_steering_rad <= 0.0:
            raise ValueError("max_abs_steering_rad must be positive")
        if self.max_steering_rate_radps <= 0.0:
            raise ValueError("max_steering_rate_radps must be positive")
        if self.min_acceleration_mps2 >= 0.0 or self.max_acceleration_mps2 <= 0.0:
            raise ValueError("acceleration limits must straddle zero")
        if self.min_acceleration_mps2 >= self.max_acceleration_mps2:
            raise ValueError("acceleration limits are not ordered")
        if self.min_jerk_mps3 >= 0.0 or self.max_jerk_mps3 <= 0.0:
            raise ValueError("jerk limits must straddle zero")
        if self.min_jerk_mps3 >= self.max_jerk_mps3:
            raise ValueError("jerk limits are not ordered")
        if self.max_speed_mps <= 0.0 or self.dt_sec <= 0.0:
            raise ValueError("speed and time-step limits must be positive")


@dataclass(frozen=True)
class ProjectionTiming:
    """Source-observation and command-lifetime contract, in seconds."""

    observation_stamp_sec: float
    now_sec: float
    valid_for_sec: float
    max_observation_age_sec: float
    future_tolerance_sec: float = 0.001

    def valid_until_sec(self) -> float:
        values = (
            self.observation_stamp_sec,
            self.now_sec,
            self.valid_for_sec,
            self.max_observation_age_sec,
            self.future_tolerance_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("projection timing must be finite")
        if self.observation_stamp_sec <= 0.0 or self.now_sec <= 0.0:
            raise ValueError("projection timestamps must be positive")
        if self.valid_for_sec <= 0.0 or self.max_observation_age_sec <= 0.0:
            raise ValueError("projection lifetimes must be positive")
        if self.future_tolerance_sec < 0.0:
            raise ValueError("future_tolerance_sec must be non-negative")
        if self.observation_stamp_sec > self.now_sec + self.future_tolerance_sec:
            raise ValueError("projection observation timestamp is in the future")
        age_sec = self.now_sec - self.observation_stamp_sec
        if age_sec > self.max_observation_age_sec:
            raise ValueError("projection observation timestamp is stale")
        valid_until = self.observation_stamp_sec + self.valid_for_sec
        if valid_until < self.now_sec:
            raise ValueError("projected command is already expired")
        return valid_until


@dataclass(frozen=True)
class ProjectedControlSequence:
    """Bounded ``[H,3]`` steering, speed, acceleration command sequence."""

    commands: np.ndarray
    steering_rate_radps: np.ndarray
    jerk_mps3: np.ndarray
    source_stamp_sec: float
    valid_until_sec: float
    limits_source: str
    dt_sec: float
    initial_state: PreviousControlState


def normalize_measured_speed_for_projection(
    speed_mps: float, *, stationary_noise_tolerance_mps: float = 1e-4
) -> float:
    """Clamp only numerical negative noise at standstill to zero in m/s."""

    if not math.isfinite(speed_mps) or not math.isfinite(
        stationary_noise_tolerance_mps
    ):
        raise ValueError("measured speed and stationary tolerance must be finite")
    if stationary_noise_tolerance_mps < 0.0:
        raise ValueError("stationary speed tolerance must be non-negative")
    if speed_mps < -stationary_noise_tolerance_mps:
        raise ValueError("measured reverse speed exceeds stationary noise tolerance")
    return max(0.0, speed_mps)


def apply_stopped_launch_acceleration_floor(
    commands: np.ndarray,
    *,
    previous: PreviousControlState,
    limits: ControlLimits,
    stopped_speed_threshold_mps: float,
    minimum_commanded_speed_mps: float,
    acceleration_floor_mps2: float,
) -> tuple[np.ndarray, bool]:
    """Apply an explicit one-shot launch floor before authoritative projection."""

    limits.validate_for_full_control()
    _validate_previous(previous, limits)
    values = np.asarray(commands, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 3:
        raise ValueError(f"model control sequence must be [H,3], got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("model control sequence must be finite")
    parameters = (
        stopped_speed_threshold_mps,
        minimum_commanded_speed_mps,
        acceleration_floor_mps2,
    )
    if not all(math.isfinite(value) for value in parameters):
        raise ValueError("launch-assist parameters must be finite")
    if stopped_speed_threshold_mps < 0.0:
        raise ValueError("launch-assist stopped speed must be non-negative")
    if not 0.0 < minimum_commanded_speed_mps <= limits.max_speed_mps:
        raise ValueError("launch-assist commanded speed must be within control limits")
    if not 0.0 < acceleration_floor_mps2 <= limits.max_acceleration_mps2:
        raise ValueError("launch-assist acceleration floor must be within control limits")
    if (
        previous.speed_mps > stopped_speed_threshold_mps
        or values[0, 1] < minimum_commanded_speed_mps
    ):
        return values.copy(), False
    adjusted = values.copy()
    adjusted[:, 2] = np.maximum(adjusted[:, 2], acceleration_floor_mps2)
    return adjusted, bool(np.any(adjusted[:, 2] != values[:, 2]))


def _validate_previous(previous: PreviousControlState, limits: ControlLimits) -> None:
    values = (
        previous.steering_rad,
        previous.speed_mps,
        previous.acceleration_mps2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("previous control state must be finite")
    tolerance = 1e-9
    if abs(previous.steering_rad) > limits.max_abs_steering_rad + tolerance:
        raise ValueError("previous steering exceeds authoritative limit")
    if not -tolerance <= previous.speed_mps <= limits.max_speed_mps + tolerance:
        raise ValueError("previous speed exceeds authoritative limit")
    if not (
        limits.min_acceleration_mps2 - tolerance
        <= previous.acceleration_mps2
        <= limits.max_acceleration_mps2 + tolerance
    ):
        raise ValueError("previous acceleration exceeds authoritative limit")


def _asymmetric_bounded_tanh(
    raw: np.ndarray, *, minimum: float, maximum: float
) -> np.ndarray:
    scale = np.where(raw >= 0.0, maximum, -minimum)
    return np.tanh(raw) * scale


def project_control_sequence(
    raw_steering_rate_and_jerk: np.ndarray,
    *,
    previous: PreviousControlState,
    limits: ControlLimits,
    timing: ProjectionTiming,
) -> ProjectedControlSequence:
    """Integrate finite raw ``[H,2]`` steering-rate/jerk into bounded SI commands."""

    limits.validate_for_full_control()
    _validate_previous(previous, limits)
    valid_until_sec = timing.valid_until_sec()
    raw = np.asarray(raw_steering_rate_and_jerk, dtype=np.float64)
    if raw.ndim != 2 or raw.shape[0] < 1 or raw.shape[1] != 2:
        raise ValueError(f"raw control sequence must be [H,2], got {raw.shape}")
    if not np.isfinite(raw).all():
        raise ValueError("raw control sequence must be finite")

    requested_steering_rate = (
        np.tanh(raw[:, 0]) * limits.max_steering_rate_radps
    )
    requested_jerk = _asymmetric_bounded_tanh(
        raw[:, 1], minimum=limits.min_jerk_mps3, maximum=limits.max_jerk_mps3
    )
    commands = np.empty((raw.shape[0], 3), dtype=np.float64)
    applied_steering_rate = np.empty(raw.shape[0], dtype=np.float64)
    applied_jerk = np.empty(raw.shape[0], dtype=np.float64)
    steering = previous.steering_rad
    speed = previous.speed_mps
    acceleration = previous.acceleration_mps2
    for index in range(raw.shape[0]):
        next_steering = float(
            np.clip(
                steering + limits.dt_sec * requested_steering_rate[index],
                -limits.max_abs_steering_rad,
                limits.max_abs_steering_rad,
            )
        )
        next_acceleration = float(
            np.clip(
                acceleration + limits.dt_sec * requested_jerk[index],
                limits.min_acceleration_mps2,
                limits.max_acceleration_mps2,
            )
        )
        next_speed = float(
            np.clip(
                speed + limits.dt_sec * next_acceleration,
                0.0,
                limits.max_speed_mps,
            )
        )
        applied_steering_rate[index] = (next_steering - steering) / limits.dt_sec
        applied_jerk[index] = (next_acceleration - acceleration) / limits.dt_sec
        commands[index] = (next_steering, next_speed, next_acceleration)
        steering, speed, acceleration = (
            next_steering,
            next_speed,
            next_acceleration,
        )
    return ProjectedControlSequence(
        commands=commands,
        steering_rate_radps=applied_steering_rate,
        jerk_mps3=applied_jerk,
        source_stamp_sec=timing.observation_stamp_sec,
        valid_until_sec=valid_until_sec,
        limits_source=limits.source,
        dt_sec=limits.dt_sec,
        initial_state=previous,
    )


def validate_model_control_sequence(
    commands: np.ndarray,
    *,
    previous: PreviousControlState,
    limits: ControlLimits,
    timing: ProjectionTiming,
) -> ProjectedControlSequence:
    """Revalidate a model-decoded physical ``[H,3]`` sequence fail-closed."""

    limits.validate_for_full_control()
    _validate_previous(previous, limits)
    valid_until_sec = timing.valid_until_sec()
    values = np.asarray(commands, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 3:
        raise ValueError(f"model control sequence must be [H,3], got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("model control sequence must be finite")
    tolerance = 1e-8
    if bool((np.abs(values[:, 0]) > limits.max_abs_steering_rad + tolerance).any()):
        raise ValueError("model steering exceeds authoritative limit")
    if bool(((values[:, 1] < -tolerance) | (values[:, 1] > limits.max_speed_mps + tolerance)).any()):
        raise ValueError("model speed exceeds authoritative limit")
    if bool(
        (
            (values[:, 2] < limits.min_acceleration_mps2 - tolerance)
            | (values[:, 2] > limits.max_acceleration_mps2 + tolerance)
        ).any()
    ):
        raise ValueError("model acceleration exceeds authoritative limit")
    steering = np.concatenate(([previous.steering_rad], values[:, 0]))
    acceleration = np.concatenate(([previous.acceleration_mps2], values[:, 2]))
    rates = np.diff(steering) / limits.dt_sec
    jerk = np.diff(acceleration) / limits.dt_sec
    if bool((np.abs(rates) > limits.max_steering_rate_radps + tolerance).any()):
        raise ValueError("model steering rate exceeds authoritative limit")
    if bool(
        (
            (jerk < limits.min_jerk_mps3 - tolerance)
            | (jerk > limits.max_jerk_mps3 + tolerance)
        ).any()
    ):
        raise ValueError("model jerk exceeds authoritative limit")
    return ProjectedControlSequence(
        commands=values.copy(),
        steering_rate_radps=rates,
        jerk_mps3=jerk,
        source_stamp_sec=timing.observation_stamp_sec,
        valid_until_sec=valid_until_sec,
        limits_source=limits.source,
        dt_sec=limits.dt_sec,
        initial_state=previous,
    )


def project_model_control_sequence(
    commands: np.ndarray,
    *,
    previous: PreviousControlState,
    limits: ControlLimits,
    timing: ProjectionTiming,
) -> ProjectedControlSequence:
    """Apply authoritative absolute/rate bounds to physical model proposals."""

    limits.validate_for_full_control()
    _validate_previous(previous, limits)
    valid_until_sec = timing.valid_until_sec()
    requested = np.asarray(commands, dtype=np.float64)
    if requested.ndim != 2 or requested.shape[0] < 1 or requested.shape[1] != 3:
        raise ValueError(f"model control sequence must be [H,3], got {requested.shape}")
    if not np.isfinite(requested).all():
        raise ValueError("model control sequence must be finite")
    projected = np.empty_like(requested)
    rates = np.empty(requested.shape[0], dtype=np.float64)
    jerk = np.empty(requested.shape[0], dtype=np.float64)
    steering = previous.steering_rad
    acceleration = previous.acceleration_mps2
    for index, proposal in enumerate(requested):
        target_steering = float(
            np.clip(proposal[0], -limits.max_abs_steering_rad, limits.max_abs_steering_rad)
        )
        steering_delta = float(
            np.clip(
                target_steering - steering,
                -limits.max_steering_rate_radps * limits.dt_sec,
                limits.max_steering_rate_radps * limits.dt_sec,
            )
        )
        target_acceleration = float(
            np.clip(proposal[2], limits.min_acceleration_mps2, limits.max_acceleration_mps2)
        )
        acceleration_delta = float(
            np.clip(
                target_acceleration - acceleration,
                limits.min_jerk_mps3 * limits.dt_sec,
                limits.max_jerk_mps3 * limits.dt_sec,
            )
        )
        steering += steering_delta
        acceleration += acceleration_delta
        projected[index] = (
            steering,
            float(np.clip(proposal[1], 0.0, limits.max_speed_mps)),
            acceleration,
        )
        rates[index] = steering_delta / limits.dt_sec
        jerk[index] = acceleration_delta / limits.dt_sec
    return ProjectedControlSequence(
        commands=projected,
        steering_rate_radps=rates,
        jerk_mps3=jerk,
        source_stamp_sec=timing.observation_stamp_sec,
        valid_until_sec=valid_until_sec,
        limits_source=limits.source,
        dt_sec=limits.dt_sec,
        initial_state=previous,
    )
