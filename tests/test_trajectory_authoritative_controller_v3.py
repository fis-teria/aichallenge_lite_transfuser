from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
)
from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
    build_executable_reference_v3,
)
from aic_transfuser_lite.control.trajectory_authoritative_controller import (
    control_from_executable_reference_v3,
    fail_closed_stop_control_v3,
)
from aic_transfuser_lite.control.longitudinal_controller_v3 import (
    LongitudinalControllerConfigV3,
    LongitudinalControllerV3,
    LongitudinalStateV3,
)


def _reference():
    decision = build_executable_reference_v3(
        AuthoritativePlanV3(
            trajectory_xy_m=np.asarray([[0.1, 0.0], [0.2, 0.0], [0.3, 0.0]]),
            speed_profile_mps=np.asarray([2.0, 2.0, 2.0]),
            waypoint_times_sec=np.asarray([0.1, 0.2, 0.3]),
            observation_stamp_sec=1.0,
        ),
        current_speed_mps=0.5,
        config=ExecutableReferenceConfigV3(
            odd_speed_cap_mps=0.75,
            max_lateral_acceleration_mps2=1.0,
        ),
    )
    assert decision.reference is not None
    return decision.reference


def test_controller_tracks_retimed_capped_reference() -> None:
    reference = _reference()
    result = control_from_executable_reference_v3(
        reference,
        current_longitudinal_speed_mps=0.5,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(
            waypoint_times_sec=(0.1, 0.2, 0.3),
            min_preview_sec=0.1,
            max_preview_sec=1.0,
        ),
    )

    assert result.authority == "trajectory_authoritative"
    assert result.reference_id == reference.reference_id
    assert result.control.commanded_speed_mps == pytest.approx(0.75)
    assert result.control.command.acceleration_mps2 == pytest.approx(0.25)


def test_invalid_plan_stop_never_becomes_direct_model_control() -> None:
    result = fail_closed_stop_control_v3(
        actual_steering_rad=0.8,
        max_abs_steering_rad=0.6,
        braking_acceleration_mps2=-2.0,
    )
    assert result.authority == "trajectory_authoritative_stop"
    assert result.commanded_speed_mps == 0.0
    assert result.command.steering_rad == 0.6
    assert result.command.acceleration_mps2 == -2.0

    with pytest.raises(ValueError, match="negative"):
        fail_closed_stop_control_v3(
            actual_steering_rad=0.0,
            max_abs_steering_rad=0.6,
            braking_acceleration_mps2=0.0,
        )


def test_authoritative_controller_uses_stateful_longitudinal_output() -> None:
    reference = _reference()
    longitudinal = LongitudinalControllerV3(
        LongitudinalControllerConfigV3(response_timeout_sec=2.0)
    )
    result = control_from_executable_reference_v3(
        reference,
        current_longitudinal_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(
            waypoint_times_sec=(0.1, 0.2, 0.3),
            min_preview_sec=0.1,
            max_preview_sec=1.0,
        ),
        longitudinal_controller=longitudinal,
        drive_preflight_ready=True,
    )
    assert result.longitudinal is not None
    assert result.longitudinal.state is LongitudinalStateV3.LAUNCHING
    assert result.control.command.acceleration_mps2 == pytest.approx(0.4)
