from __future__ import annotations

import pytest

from aic_transfuser_lite.control.longitudinal_controller_v3 import (
    LongitudinalControllerConfigV3,
    LongitudinalControllerV3,
    LongitudinalStateV3,
)


def _controller(**overrides) -> LongitudinalControllerV3:
    values = {**LongitudinalControllerConfigV3().__dict__, **overrides}
    return LongitudinalControllerV3(LongitudinalControllerConfigV3(**values))


def test_launch_ramps_through_deadzone_with_jerk_limit() -> None:
    controller = _controller(response_timeout_sec=2.0)
    first = controller.step(
        executable_speed_mps=0.75,
        measured_speed_mps=0.0,
        drive_preflight_ready=True,
    )
    second = controller.step(
        executable_speed_mps=0.75,
        measured_speed_mps=0.01,
        drive_preflight_ready=True,
    )
    assert first.state is LongitudinalStateV3.LAUNCHING
    assert first.acceleration_mps2 == pytest.approx(0.4)
    assert first.jerk_limited
    assert second.acceleration_mps2 >= 0.5


def test_preflight_block_resets_integral_and_brakes_with_jerk_bound() -> None:
    controller = _controller()
    controller.step(
        executable_speed_mps=0.75,
        measured_speed_mps=0.2,
        drive_preflight_ready=True,
    )
    blocked = controller.step(
        executable_speed_mps=0.75,
        measured_speed_mps=0.2,
        drive_preflight_ready=False,
    )
    assert blocked.state is LongitudinalStateV3.BLOCKED
    assert blocked.integral_speed_error_mps_sec == 0.0
    assert blocked.acceleration_mps2 >= -4.0
    assert blocked.jerk_limited


def test_anti_windup_does_not_integrate_further_into_positive_saturation() -> None:
    controller = _controller(max_acceleration_mps2=0.5)
    result = None
    for _ in range(20):
        result = controller.step(
            executable_speed_mps=10.0,
            measured_speed_mps=1.0,
            drive_preflight_ready=True,
        )
    assert result is not None
    assert result.saturated
    assert result.integral_speed_error_mps_sec == 0.0


def test_missing_launch_response_latches_fault_and_requests_braking() -> None:
    controller = _controller(response_timeout_sec=0.3, launch_timeout_sec=1.0)
    result = None
    for _ in range(5):
        result = controller.step(
            executable_speed_mps=0.75,
            measured_speed_mps=0.0,
            drive_preflight_ready=True,
        )
    assert result is not None
    assert result.state is LongitudinalStateV3.RESPONSE_FAULT
    assert result.fault_reason == "launch_response_missing"
    assert result.acceleration_mps2 < 0.0


@pytest.mark.parametrize(
    "overrides",
    [
        {"acceleration_gain": 0.0},
        {"stopped_speed_mps": 0.2, "moving_speed_mps": 0.1},
        {"response_timeout_sec": 4.0, "launch_timeout_sec": 3.0},
        {"launch_acceleration_floor_mps2": 3.0},
    ],
)
def test_invalid_longitudinal_config_is_rejected(overrides) -> None:
    with pytest.raises(ValueError):
        _controller(**overrides)
