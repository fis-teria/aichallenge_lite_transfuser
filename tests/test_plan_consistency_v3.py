from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.control.executable_reference import AuthoritativePlanV3
from aic_transfuser_lite.runtime.plan_consistency import evaluate_plan_consistency_v3


def _plan(trajectory: np.ndarray, speeds: np.ndarray) -> AuthoritativePlanV3:
    count = len(trajectory)
    return AuthoritativePlanV3(
        trajectory_xy_m=np.asarray(trajectory, dtype=np.float64),
        speed_profile_mps=np.asarray(speeds, dtype=np.float64),
        waypoint_times_sec=np.arange(1, count + 1, dtype=np.float64) * 0.1,
        observation_stamp_sec=1.0,
    )


def test_matching_geometry_and_trapezoidal_speed_has_zero_e_plan() -> None:
    metrics = evaluate_plan_consistency_v3(
        _plan(
            np.asarray([[0.1, 0.0], [0.2, 0.0], [0.3, 0.0]]),
            np.ones(3),
        ),
        current_speed_mps=1.0,
    )
    np.testing.assert_allclose(metrics.segment_length_m, 0.1)
    np.testing.assert_allclose(metrics.geometric_speed_mps, 1.0)
    np.testing.assert_allclose(metrics.trapezoidal_speed_mps, 1.0)
    np.testing.assert_allclose(metrics.speed_residual_mps, 0.0, atol=1e-15)
    assert metrics.mean_absolute_error_mps == pytest.approx(0.0, abs=1e-15)
    assert metrics.max_absolute_error_mps == pytest.approx(0.0, abs=1e-15)
    assert metrics.normalized_huber_mean == pytest.approx(0.0, abs=1e-15)


def test_e_plan_reports_scale_normalized_huber_and_absolute_error() -> None:
    metrics = evaluate_plan_consistency_v3(
        _plan(np.asarray([[1.0, 0.0], [2.0, 0.0]]), np.ones(2)),
        current_speed_mps=1.0,
        speed_scale_mps=2.0,
        huber_delta=1.0,
    )
    np.testing.assert_allclose(metrics.geometric_speed_mps, 10.0)
    np.testing.assert_allclose(metrics.trapezoidal_speed_mps, 1.0)
    np.testing.assert_allclose(metrics.speed_residual_mps, 9.0)
    assert metrics.mean_absolute_error_mps == pytest.approx(9.0)
    assert metrics.max_absolute_error_mps == pytest.approx(9.0)
    assert metrics.normalized_huber_mean == pytest.approx(4.0)


def test_curve_uses_polyline_segment_speed_not_origin_to_endpoint_speed() -> None:
    metrics = evaluate_plan_consistency_v3(
        _plan(
            np.asarray([[0.1, 0.0], [0.1, 0.1], [0.0, 0.1]]),
            np.ones(3),
        ),
        current_speed_mps=1.0,
    )
    np.testing.assert_allclose(metrics.segment_length_m, 0.1)
    np.testing.assert_allclose(metrics.geometric_speed_mps, 1.0)


@pytest.mark.parametrize(
    ("current_speed", "speed_scale", "huber_delta", "message"),
    [
        (-0.1, 1.0, 1.0, "current_speed_mps"),
        (0.0, 0.0, 1.0, "speed_scale_mps"),
        (0.0, 1.0, 0.0, "huber_delta"),
    ],
)
def test_invalid_metric_parameters_are_rejected(
    current_speed: float,
    speed_scale: float,
    huber_delta: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        evaluate_plan_consistency_v3(
            _plan(np.asarray([[0.1, 0.0], [0.2, 0.0]]), np.ones(2)),
            current_speed_mps=current_speed,
            speed_scale_mps=speed_scale,
            huber_delta=huber_delta,
        )


def test_non_monotonic_waypoint_time_is_rejected() -> None:
    plan = AuthoritativePlanV3(
        trajectory_xy_m=np.asarray([[0.1, 0.0], [0.2, 0.0]]),
        speed_profile_mps=np.ones(2),
        waypoint_times_sec=np.asarray([0.1, 0.1]),
        observation_stamp_sec=1.0,
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        evaluate_plan_consistency_v3(plan, current_speed_mps=1.0)
