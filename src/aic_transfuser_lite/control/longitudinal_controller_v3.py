from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class LongitudinalStateV3(str, Enum):
    STOPPED = "stopped"
    LAUNCHING = "launching"
    MOVING = "moving"
    BLOCKED = "blocked"
    RESPONSE_FAULT = "response_fault"


@dataclass(frozen=True)
class LongitudinalControllerConfigV3:
    control_dt_sec: float = 0.1
    reference_horizon_sec: float = 0.5
    speed_kp: float = 0.8
    speed_ki: float = 0.3
    integral_limit_mps_sec: float = 1.0
    acceleration_gain: float = 1.0
    acceleration_bias_mps2: float = 0.0
    min_acceleration_mps2: float = -4.0
    max_acceleration_mps2: float = 2.0
    min_jerk_mps3: float = -8.0
    max_jerk_mps3: float = 4.0
    stopped_speed_mps: float = 0.05
    moving_speed_mps: float = 0.15
    launch_min_reference_speed_mps: float = 0.2
    launch_acceleration_floor_mps2: float = 0.5
    launch_timeout_sec: float = 3.0
    response_timeout_sec: float = 1.0
    minimum_launch_speed_delta_mps: float = 0.02

    def validate(self) -> None:
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError("longitudinal controller config must be finite")
        if self.control_dt_sec <= 0.0 or self.reference_horizon_sec <= 0.0:
            raise ValueError("controller time values must be positive")
        if self.speed_kp < 0.0 or self.speed_ki < 0.0:
            raise ValueError("PI gains must be non-negative")
        if self.integral_limit_mps_sec <= 0.0:
            raise ValueError("integral limit must be positive")
        if self.acceleration_gain <= 0.0:
            raise ValueError("acceleration gain must be positive")
        if not self.min_acceleration_mps2 < 0.0 < self.max_acceleration_mps2:
            raise ValueError("acceleration limits must straddle zero")
        if not self.min_jerk_mps3 < 0.0 < self.max_jerk_mps3:
            raise ValueError("jerk limits must straddle zero")
        if not 0.0 <= self.stopped_speed_mps < self.moving_speed_mps:
            raise ValueError("moving speed must exceed stopped speed")
        if self.launch_min_reference_speed_mps <= 0.0:
            raise ValueError("launch reference threshold must be positive")
        if not 0.0 < self.launch_acceleration_floor_mps2 <= self.max_acceleration_mps2:
            raise ValueError("launch acceleration floor is outside acceleration limits")
        if not 0.0 < self.response_timeout_sec <= self.launch_timeout_sec:
            raise ValueError("response timeout must be within launch timeout")
        if self.minimum_launch_speed_delta_mps <= 0.0:
            raise ValueError("minimum launch speed delta must be positive")


@dataclass(frozen=True)
class LongitudinalControlResultV3:
    acceleration_mps2: float
    reference_acceleration_mps2: float
    feedforward_acceleration_mps2: float
    feedback_acceleration_mps2: float
    integral_speed_error_mps_sec: float
    state: LongitudinalStateV3
    saturated: bool
    jerk_limited: bool
    launch_elapsed_sec: float
    fault_reason: str | None = None


class LongitudinalControllerV3:
    """Stateful feedforward + PI controller with bounded launch monitoring."""

    def __init__(self, config: LongitudinalControllerConfigV3) -> None:
        config.validate()
        self.config = config
        self.integral_speed_error_mps_sec = 0.0
        self.previous_acceleration_mps2 = 0.0
        self.state = LongitudinalStateV3.STOPPED
        self.launch_elapsed_sec = 0.0
        self.launch_initial_speed_mps = 0.0
        self.fault_reason: str | None = None

    def reset(self) -> None:
        self.integral_speed_error_mps_sec = 0.0
        self.previous_acceleration_mps2 = 0.0
        self.state = LongitudinalStateV3.STOPPED
        self.launch_elapsed_sec = 0.0
        self.launch_initial_speed_mps = 0.0
        self.fault_reason = None

    def step(
        self,
        *,
        executable_speed_mps: float,
        measured_speed_mps: float,
        drive_preflight_ready: bool,
        stop_requested: bool = False,
    ) -> LongitudinalControlResultV3:
        cfg = self.config
        values = (executable_speed_mps, measured_speed_mps)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("longitudinal controller inputs must be finite")
        if executable_speed_mps < 0.0 or measured_speed_mps < 0.0:
            raise ValueError("longitudinal speeds must be non-negative")

        if stop_requested or not drive_preflight_ready:
            self.integral_speed_error_mps_sec = 0.0
            self.launch_elapsed_sec = 0.0
            self.state = (
                LongitudinalStateV3.STOPPED
                if stop_requested
                else LongitudinalStateV3.BLOCKED
            )
            return self._bounded_result(
                proposed_acceleration_mps2=cfg.min_acceleration_mps2,
                reference_acceleration_mps2=cfg.min_acceleration_mps2,
                feedforward_acceleration_mps2=cfg.min_acceleration_mps2,
                feedback_acceleration_mps2=0.0,
            )

        if self.fault_reason is not None:
            self.state = LongitudinalStateV3.RESPONSE_FAULT
            return self._bounded_result(
                proposed_acceleration_mps2=cfg.min_acceleration_mps2,
                reference_acceleration_mps2=cfg.min_acceleration_mps2,
                feedforward_acceleration_mps2=cfg.min_acceleration_mps2,
                feedback_acceleration_mps2=0.0,
            )

        launch_requested = (
            measured_speed_mps <= cfg.stopped_speed_mps
            and executable_speed_mps >= cfg.launch_min_reference_speed_mps
        )
        if self.state in {LongitudinalStateV3.STOPPED, LongitudinalStateV3.BLOCKED}:
            if launch_requested:
                self.state = LongitudinalStateV3.LAUNCHING
                self.launch_elapsed_sec = 0.0
                self.launch_initial_speed_mps = measured_speed_mps
        elif self.state is LongitudinalStateV3.LAUNCHING:
            if measured_speed_mps >= cfg.moving_speed_mps:
                self.state = LongitudinalStateV3.MOVING
                self.launch_elapsed_sec = 0.0
            else:
                self.launch_elapsed_sec += cfg.control_dt_sec
                launch_delta = measured_speed_mps - self.launch_initial_speed_mps
                if (
                    self.launch_elapsed_sec >= cfg.response_timeout_sec
                    and launch_delta < cfg.minimum_launch_speed_delta_mps
                ):
                    self.fault_reason = "launch_response_missing"
                    self.state = LongitudinalStateV3.RESPONSE_FAULT
                elif self.launch_elapsed_sec >= cfg.launch_timeout_sec:
                    self.fault_reason = "launch_timeout"
                    self.state = LongitudinalStateV3.RESPONSE_FAULT
        elif (
            self.state is LongitudinalStateV3.MOVING
            and measured_speed_mps <= cfg.stopped_speed_mps
        ):
            self.state = LongitudinalStateV3.STOPPED

        if self.fault_reason is not None:
            return self._bounded_result(
                proposed_acceleration_mps2=cfg.min_acceleration_mps2,
                reference_acceleration_mps2=cfg.min_acceleration_mps2,
                feedforward_acceleration_mps2=cfg.min_acceleration_mps2,
                feedback_acceleration_mps2=0.0,
            )

        speed_error = executable_speed_mps - measured_speed_mps
        reference_acceleration = speed_error / cfg.reference_horizon_sec
        feedforward = (
            reference_acceleration - cfg.acceleration_bias_mps2
        ) / cfg.acceleration_gain
        candidate_integral = max(
            -cfg.integral_limit_mps_sec,
            min(
                cfg.integral_limit_mps_sec,
                self.integral_speed_error_mps_sec
                + speed_error * cfg.control_dt_sec,
            ),
        )
        feedback = cfg.speed_kp * speed_error + cfg.speed_ki * candidate_integral
        proposal = feedforward + feedback
        if self.state is LongitudinalStateV3.LAUNCHING:
            proposal = max(proposal, cfg.launch_acceleration_floor_mps2)
        saturated_proposal = max(
            cfg.min_acceleration_mps2,
            min(cfg.max_acceleration_mps2, proposal),
        )
        saturated = not math.isclose(proposal, saturated_proposal, abs_tol=1e-12)
        if not saturated or (
            saturated_proposal >= cfg.max_acceleration_mps2 and speed_error < 0.0
        ) or (
            saturated_proposal <= cfg.min_acceleration_mps2 and speed_error > 0.0
        ):
            self.integral_speed_error_mps_sec = candidate_integral
        feedback = (
            cfg.speed_kp * speed_error
            + cfg.speed_ki * self.integral_speed_error_mps_sec
        )
        proposal = feedforward + feedback
        if self.state is LongitudinalStateV3.LAUNCHING:
            proposal = max(proposal, cfg.launch_acceleration_floor_mps2)
        return self._bounded_result(
            proposed_acceleration_mps2=proposal,
            reference_acceleration_mps2=reference_acceleration,
            feedforward_acceleration_mps2=feedforward,
            feedback_acceleration_mps2=feedback,
        )

    def _bounded_result(
        self,
        *,
        proposed_acceleration_mps2: float,
        reference_acceleration_mps2: float,
        feedforward_acceleration_mps2: float,
        feedback_acceleration_mps2: float,
    ) -> LongitudinalControlResultV3:
        cfg = self.config
        saturated_value = max(
            cfg.min_acceleration_mps2,
            min(cfg.max_acceleration_mps2, proposed_acceleration_mps2),
        )
        saturated = not math.isclose(
            saturated_value, proposed_acceleration_mps2, abs_tol=1e-12
        )
        minimum = self.previous_acceleration_mps2 + cfg.min_jerk_mps3 * cfg.control_dt_sec
        maximum = self.previous_acceleration_mps2 + cfg.max_jerk_mps3 * cfg.control_dt_sec
        bounded = max(minimum, min(maximum, saturated_value))
        bounded = max(cfg.min_acceleration_mps2, min(cfg.max_acceleration_mps2, bounded))
        jerk_limited = not math.isclose(bounded, saturated_value, abs_tol=1e-12)
        self.previous_acceleration_mps2 = bounded
        return LongitudinalControlResultV3(
            acceleration_mps2=float(bounded),
            reference_acceleration_mps2=float(reference_acceleration_mps2),
            feedforward_acceleration_mps2=float(feedforward_acceleration_mps2),
            feedback_acceleration_mps2=float(feedback_acceleration_mps2),
            integral_speed_error_mps_sec=float(
                self.integral_speed_error_mps_sec
            ),
            state=self.state,
            saturated=saturated,
            jerk_limited=jerk_limited,
            launch_elapsed_sec=float(self.launch_elapsed_sec),
            fault_reason=self.fault_reason,
        )
