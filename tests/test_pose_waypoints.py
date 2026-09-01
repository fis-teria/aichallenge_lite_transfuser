from __future__ import annotations

import math

import numpy as np
import pytest

from aic_transfuser_lite.data.mcap_converter_v2 import (
    TimedPose,
    future_waypoints_from_pose,
    interpolate_pose,
)


def _pose(time_sec: float, x_m: float, y_m: float, yaw_rad: float) -> TimedPose:
    return TimedPose(
        timestamp_ns=int(round(time_sec * 1e9)),
        x_world_m=x_m,
        y_world_m=y_m,
        yaw_world_rad=yaw_rad,
        frame_id="map",
        child_frame_id="base_link",
    )


def test_measured_pose_waypoints_straight_in_observation_ego_frame() -> None:
    poses = [_pose(t, 2.0 * t, 0.0, 0.0) for t in np.arange(0.0, 3.1, 0.1)]

    waypoints = future_waypoints_from_pose(
        poses,
        observation_ns=0,
        horizons_sec=(0.5, 1.0, 1.5),
        tolerance_ms=60.0,
    )

    assert waypoints.shape == (3, 2)
    np.testing.assert_allclose(waypoints[:, 0], [1.0, 2.0, 3.0], atol=1e-6)
    np.testing.assert_allclose(waypoints[:, 1], 0.0, atol=1e-6)


@pytest.mark.parametrize("turn_sign", [1.0, -1.0])
def test_measured_pose_waypoints_preserve_left_right_sign(turn_sign: float) -> None:
    poses = [
        _pose(t, t, turn_sign * 0.25 * t * t, turn_sign * 0.2 * t)
        for t in np.arange(0.0, 3.1, 0.1)
    ]

    waypoints = future_waypoints_from_pose(
        poses,
        observation_ns=0,
        horizons_sec=(0.5, 1.0, 1.5),
        tolerance_ms=60.0,
    )

    assert np.all(waypoints[:, 0] > 0.0)
    assert np.all(np.sign(waypoints[:, 1]) == turn_sign)


def test_pose_interpolation_unwraps_yaw_across_pi() -> None:
    poses = [
        _pose(0.0, 0.0, 0.0, math.radians(179.0)),
        _pose(1.0, 1.0, 0.0, math.radians(-179.0)),
    ]

    interpolated, timing = interpolate_pose(poses, 500_000_000, tolerance_ms=600.0)

    assert abs(abs(interpolated.yaw_world_rad) - math.pi) < math.radians(0.1)
    assert interpolated.x_world_m == pytest.approx(0.5)
    assert timing.max_endpoint_delta_ns == 500_000_000


def test_waypoint_transform_uses_observation_orientation() -> None:
    poses = [
        _pose(0.0, 10.0, 20.0, math.pi / 2.0),
        _pose(0.5, 10.0, 21.0, math.pi / 2.0),
        _pose(1.0, 10.0, 22.0, math.pi / 2.0),
    ]

    waypoints = future_waypoints_from_pose(
        poses,
        observation_ns=0,
        horizons_sec=(0.5, 1.0),
        tolerance_ms=600.0,
    )

    np.testing.assert_allclose(waypoints, [[1.0, 0.0], [2.0, 0.0]], atol=1e-6)
