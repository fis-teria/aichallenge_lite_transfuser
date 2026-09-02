from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
)
from aic_transfuser_lite.control.shadow_trajectory_controller import (
    shadow_control_from_trajectory_speed_profile,
)


TIMES = tuple((index + 1) * 0.1 for index in range(15))


def _straight() -> np.ndarray:
    return np.stack(
        (np.linspace(0.5, 7.5, 15), np.zeros(15)), axis=1
    ).astype(np.float32)


def test_shadow_controller_uses_same_preview_for_trajectory_and_speed() -> None:
    speeds = np.linspace(1.0, 15.0, 15, dtype=np.float32)
    result = shadow_control_from_trajectory_speed_profile(
        _straight(),
        speeds,
        current_longitudinal_speed_mps=2.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(
            waypoint_times_sec=TIMES,
            estimated_delay_sec=0.2,
            min_preview_sec=0.1,
            max_preview_sec=1.5,
        ),
    )

    assert result.control.preview_time_sec == pytest.approx(0.55)
    assert result.target_speed_mps == pytest.approx(5.5)
    assert result.control.commanded_speed_mps == pytest.approx(5.5)
    assert result.control.command.steering_rad == pytest.approx(0.0)
    assert result.calibration_status == "unverified"
    assert not result.nominal_control_eligible


@pytest.mark.parametrize(
    ("points", "speeds", "message"),
    [
        (np.zeros((15, 3)), np.zeros(15), r"\[N,2\]"),
        (_straight(), np.zeros(14), "matching trajectory"),
        (_straight(), np.full(15, np.nan), "must be finite"),
        (_straight(), -np.ones(15), "must be non-negative"),
    ],
)
def test_shadow_controller_rejects_invalid_shapes_and_speeds(
    points: np.ndarray,
    speeds: np.ndarray,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        shadow_control_from_trajectory_speed_profile(
            points,
            speeds,
            current_longitudinal_speed_mps=0.0,
            yaw_rate_rps=0.0,
            actual_steering_rad=0.0,
            config=DelayAwareControllerConfig(waypoint_times_sec=TIMES),
        )


def test_shadow_controller_rejects_time_count_and_behind_preview() -> None:
    with pytest.raises(ValueError, match="time count"):
        shadow_control_from_trajectory_speed_profile(
            _straight(),
            np.ones(15),
            current_longitudinal_speed_mps=0.0,
            yaw_rate_rps=0.0,
            actual_steering_rad=0.0,
            config=DelayAwareControllerConfig(waypoint_times_sec=TIMES[:-1]),
        )
    with pytest.raises(ValueError, match="ahead"):
        shadow_control_from_trajectory_speed_profile(
            -_straight(),
            np.ones(15),
            current_longitudinal_speed_mps=0.0,
            yaw_rate_rps=0.0,
            actual_steering_rad=0.0,
            config=DelayAwareControllerConfig(waypoint_times_sec=TIMES),
        )
