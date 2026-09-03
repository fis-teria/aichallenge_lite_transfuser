from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from .delay_aware_controller import (
    DelayAwareControllerConfig,
    DelayAwareControlResult,
    control_from_waypoints_delay_aware,
)
from .executable_reference import ExecutableReferenceV3
from .longitudinal_controller_v3 import (
    LongitudinalControllerV3,
    LongitudinalControlResultV3,
)
from .waypoint_controller import ControlCommand


@dataclass(frozen=True)
class TrajectoryAuthoritativeControlV3:
    """Nominal command derived only from one executable Plan reference."""

    control: DelayAwareControlResult
    reference_id: str
    longitudinal: LongitudinalControlResultV3 | None = None
    authority: str = "trajectory_authoritative"


@dataclass(frozen=True)
class FailClosedStopControlV3:
    """Bounded STOP proposal that still requires the external Safety Supervisor."""

    command: ControlCommand
    commanded_speed_mps: float = 0.0
    authority: str = "trajectory_authoritative_stop"


def control_from_executable_reference_v3(
    reference: ExecutableReferenceV3,
    *,
    current_longitudinal_speed_mps: float,
    yaw_rate_rps: float,
    actual_steering_rad: float,
    config: DelayAwareControllerConfig,
    longitudinal_controller: LongitudinalControllerV3 | None = None,
    drive_preflight_ready: bool = True,
) -> TrajectoryAuthoritativeControlV3:
    """Track the cap-applied, retimed reference; raw Plan timing is never used."""

    reference.validate()
    controller_config = replace(
        config,
        waypoint_times_sec=tuple(
            float(value) for value in reference.time_from_observation_sec
        ),
    )
    preview_time_sec = float(
        np.clip(
            controller_config.base_preview_sec
            + controller_config.estimated_delay_sec,
            controller_config.min_preview_sec,
            controller_config.max_preview_sec,
        )
    )
    target_speed_mps = float(
        np.interp(
            preview_time_sec,
            reference.time_from_observation_sec,
            reference.speed_mps,
        )
    )
    control = control_from_waypoints_delay_aware(
        reference.trajectory_xy_m,
        target_speed_mps=target_speed_mps,
        current_longitudinal_speed_mps=current_longitudinal_speed_mps,
        yaw_rate_rps=yaw_rate_rps,
        actual_steering_rad=actual_steering_rad,
        config=controller_config,
    )
    longitudinal = None
    if longitudinal_controller is not None:
        longitudinal = longitudinal_controller.step(
            executable_speed_mps=target_speed_mps,
            measured_speed_mps=current_longitudinal_speed_mps,
            drive_preflight_ready=drive_preflight_ready,
        )
        control = replace(
            control,
            command=ControlCommand(
                control.command.steering_rad,
                longitudinal.acceleration_mps2,
            ),
        )
    return TrajectoryAuthoritativeControlV3(
        control=control,
        reference_id=reference.reference_id,
        longitudinal=longitudinal,
    )


def fail_closed_stop_control_v3(
    *,
    actual_steering_rad: float,
    max_abs_steering_rad: float,
    braking_acceleration_mps2: float,
) -> FailClosedStopControlV3:
    """Build a finite zero-speed brake proposal for an invalid/stopping Plan."""

    values = (
        actual_steering_rad,
        max_abs_steering_rad,
        braking_acceleration_mps2,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("fail-closed STOP inputs must be finite")
    if max_abs_steering_rad <= 0.0:
        raise ValueError("max_abs_steering_rad must be positive")
    if braking_acceleration_mps2 >= 0.0:
        raise ValueError("braking_acceleration_mps2 must be negative")
    steering = float(
        np.clip(actual_steering_rad, -max_abs_steering_rad, max_abs_steering_rad)
    )
    return FailClosedStopControlV3(
        command=ControlCommand(steering, float(braking_acceleration_mps2))
    )
