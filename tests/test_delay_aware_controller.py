from __future__ import annotations

import math

import numpy as np
import pytest

from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
    control_from_waypoints_delay_aware,
    interpolate_waypoint,
    project_waypoints_to_future_ego,
)


WAYPOINT_TIMES = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)


def straight_waypoints() -> np.ndarray:
    return np.asarray([[1.0 + index, 0.0] for index in range(6)], dtype=np.float32)


def test_tau_zero_projection_is_identity_and_nominal_straight_control_matches() -> None:
    waypoints = straight_waypoints()
    projected = project_waypoints_to_future_ego(
        waypoints, speed_mps=4.0, yaw_rate_rps=0.2, delay_sec=0.0
    )
    np.testing.assert_allclose(projected, waypoints)
    result = control_from_waypoints_delay_aware(
        waypoints,
        target_speed_mps=5.0,
        current_longitudinal_speed_mps=4.0,
        yaw_rate_rps=0.2,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(waypoint_times_sec=WAYPOINT_TIMES),
    )
    assert result.delay_sec == 0.0
    assert result.preview_time_sec == pytest.approx(0.5)
    np.testing.assert_allclose(result.preview_target_xy_m, [1.0, 0.0])
    assert result.command.steering_rad == pytest.approx(0.0)
    assert result.command.acceleration_mps2 == pytest.approx(1.0)
    assert result.commanded_speed_mps == pytest.approx(5.0)


def test_straight_and_turning_state_projection_match_kinematic_equations() -> None:
    points = np.asarray([[3.0, 1.0]], dtype=np.float32)
    straight = project_waypoints_to_future_ego(
        points, speed_mps=2.0, yaw_rate_rps=0.0, delay_sec=0.5
    )
    np.testing.assert_allclose(straight, [[2.0, 1.0]], atol=1e-6)

    speed = 2.0
    yaw_rate = 1.0
    delay = 0.5
    yaw = yaw_rate * delay
    radius = speed / yaw_rate
    dx = radius * math.sin(yaw)
    dy = radius * (1.0 - math.cos(yaw))
    expected_x = math.cos(yaw) * (3.0 - dx) + math.sin(yaw) * (1.0 - dy)
    expected_y = -math.sin(yaw) * (3.0 - dx) + math.cos(yaw) * (1.0 - dy)
    turning = project_waypoints_to_future_ego(
        points, speed_mps=speed, yaw_rate_rps=yaw_rate, delay_sec=delay
    )
    np.testing.assert_allclose(turning, [[expected_x, expected_y]], atol=1e-6)


def test_preview_interpolation_is_continuous_and_right_corner_steering_is_negative() -> None:
    waypoints = np.asarray(
        [[1.0, -0.1], [2.0, -0.4], [3.0, -0.9], [4.0, -1.4], [5.0, -1.9], [6.0, -2.4]],
        dtype=np.float32,
    )
    target_055 = interpolate_waypoint(waypoints, WAYPOINT_TIMES, 0.55)
    target_056 = interpolate_waypoint(waypoints, WAYPOINT_TIMES, 0.56)
    np.testing.assert_allclose(target_055, [1.1, -0.13], atol=1e-6)
    assert np.linalg.norm(target_056 - target_055) < 0.03
    result = control_from_waypoints_delay_aware(
        waypoints,
        target_speed_mps=4.0,
        current_longitudinal_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(
            waypoint_times_sec=WAYPOINT_TIMES,
            estimated_delay_sec=0.2,
        ),
    )
    assert result.preview_time_sec == pytest.approx(0.55)
    assert result.command.steering_rad < 0.0


def test_optional_rate_limit_is_anchored_to_measured_actual_steering() -> None:
    waypoints = np.asarray([[1.0, 1.0]] * 6, dtype=np.float32)
    result = control_from_waypoints_delay_aware(
        waypoints,
        target_speed_mps=2.0,
        current_longitudinal_speed_mps=1.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=-0.1,
        config=DelayAwareControllerConfig(
            waypoint_times_sec=WAYPOINT_TIMES,
            max_steering_rate_radps=1.0,
            control_period_sec=0.1,
        ),
    )
    assert result.unlimited_steering_rad > 0.0
    assert result.command.steering_rad == pytest.approx(0.0, abs=1e-7)
    assert result.steering_rate_limited


def test_invalid_or_behind_preview_target_fails_closed() -> None:
    with pytest.raises(ValueError, match="ahead"):
        control_from_waypoints_delay_aware(
            np.asarray([[-1.0, 0.0]] * 6, dtype=np.float32),
            target_speed_mps=1.0,
            current_longitudinal_speed_mps=1.0,
            yaw_rate_rps=0.0,
            actual_steering_rad=0.0,
            config=DelayAwareControllerConfig(waypoint_times_sec=WAYPOINT_TIMES),
        )
    with pytest.raises(ValueError, match="finite"):
        project_waypoints_to_future_ego(
            straight_waypoints(),
            speed_mps=float("nan"),
            yaw_rate_rps=0.0,
            delay_sec=0.1,
        )


def test_negative_model_target_speed_is_clamped_before_commanding() -> None:
    result = control_from_waypoints_delay_aware(
        straight_waypoints(),
        target_speed_mps=-2.0,
        current_longitudinal_speed_mps=1.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=DelayAwareControllerConfig(waypoint_times_sec=WAYPOINT_TIMES),
    )
    assert result.commanded_speed_mps == 0.0
    assert result.command.acceleration_mps2 == pytest.approx(-1.0)
