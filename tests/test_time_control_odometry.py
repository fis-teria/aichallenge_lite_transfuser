"""Relative control geometry and input failures without GNSS, IMU or ROS."""
from __future__ import annotations

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.runtime.time_control_odometry import TimeControlOdometry, CONTROL_ODOMETRY_FRAME
from aic_transfuser_lite.control.time_reference_v1 import TimePlan
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose, time_trial_control
from aic_transfuser_lite.control.turning_scan_guard import scan_pose_in_rear
from aic_transfuser_lite.control.vehicle_motion_v1 import WHEELBASE_M, AWSIM_20KMH_POLICY, body_curvature_for_tire


def feed(odom, t, speed=.5, steer=0., epoch='0'):
    odom.add_speed(t, speed)
    odom.add_steering(t, steer)
    return odom.drain(t, epoch)


def test_straight_units_origin_and_exact_duplicates():
    odom = TimeControlOdometry(.001)
    poses = [feed(odom, t)[0][0] for t in range(0, 1_000_000_001, 50_000_000)]
    assert poses[0].x_m == poses[0].y_m == poses[0].yaw_rad == 0.
    assert poses[-1].x_m == pytest.approx(.5)
    assert poses[-1].y_m == poses[-1].yaw_rad == 0.
    assert poses[-1].world_frame == CONTROL_ODOMETRY_FRAME
    assert poses[-1].body_frame == 'base_link'
    assert feed(odom, 1_000_000_000) == []


@pytest.mark.parametrize('steer', [-.25, .25])
def test_turn_and_rear_to_base_transform(steer):
    offset = .001
    odom = TimeControlOdometry(offset)
    for t in range(0, 1_000_000_001, 50_000_000):
        pose, trace = feed(odom, t, steer=steer)[0]
    k = math.tan(steer)/WHEELBASE_M
    angle = .5*k
    assert pose.yaw_rad == pytest.approx(angle)
    assert pose.x_m == pytest.approx(math.sin(angle)/k + offset*(1-math.cos(angle)))
    assert pose.y_m == pytest.approx((1-math.cos(angle))/k-offset*math.sin(angle))
    assert trace['estimated_yaw_rate_radps'] == pytest.approx(.5*k)


def test_response_policy_and_speed_change_are_integrated_at_capture_time():
    odom = TimeControlOdometry(0., AWSIM_20KMH_POLICY)
    feed(odom, 0, speed=2., steer=.2)
    pose, trace = feed(odom, 100_000_000, speed=3., steer=.2)[0]
    a = 2*body_curvature_for_tire(.2, 2., AWSIM_20KMH_POLICY)
    b = 3*body_curvature_for_tire(.2, 3., AWSIM_20KMH_POLICY)
    assert pose.yaw_rad == pytest.approx(.05*(a+b))
    assert trace['estimated_yaw_rate_radps'] == pytest.approx(b)


def test_async_join_interpolates_steering_without_extrapolation():
    odom = TimeControlOdometry(0.)
    odom.add_steering(0, 0.)
    odom.add_speed(50_000_000, .5)
    assert odom.drain(50_000_000, '0') == []
    odom.add_steering(100_000_000, .2)
    pose, trace = odom.drain(100_000_000, '0')[0]
    assert pose.stamp_ns == 50_000_000
    assert trace['steering_rad'] == pytest.approx(.1)
    assert trace['steering_stamps_ns'] == [0, 100_000_000]


def test_missing_steering_latches_fault_and_reset_starts_new_origin():
    odom = TimeControlOdometry(0.)
    odom.add_speed(0, .5)
    with pytest.raises(ValueError, match='JOIN_TIMEOUT'):
        odom.drain(150_000_001, '0')
    with pytest.raises(ValueError, match='JOIN_TIMEOUT'):
        odom.add_steering(150_000_001, 0.)
    odom.reset()
    pose = feed(odom, 0, epoch='1')[0][0]
    assert pose.epoch == '1' and pose.x_m == 0.


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -.1, 10.])
def test_bad_speed_is_rejected(bad):
    with pytest.raises(ValueError):
        feed(TimeControlOdometry(0.), 0, speed=bad)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -.501, .501])
def test_bad_steering_is_rejected(bad):
    with pytest.raises(ValueError):
        feed(TimeControlOdometry(0.), 0, steer=bad)


@pytest.mark.parametrize('stamp', [-1, 0., True])
def test_invalid_stamp_type_or_value(stamp):
    with pytest.raises(ValueError, match='STAMP'):
        feed(TimeControlOdometry(0.), stamp)


def test_gap_future_conflict_and_reordered_input_fail():
    odom = TimeControlOdometry(0.)
    feed(odom, 0)
    with pytest.raises(ValueError, match='GAP'):
        feed(odom, 200_000_000)
    odom = TimeControlOdometry(0.)
    feed(odom, 100_000_000)
    with pytest.raises(ValueError, match='OUT_OF_ORDER'):
        feed(odom, 50_000_000)
    odom = TimeControlOdometry(0.)
    feed(odom, 0)
    with pytest.raises(ValueError, match='CONFLICTING'):
        feed(odom, 0, speed=.6)
    odom = TimeControlOdometry(0.)
    odom.add_steering(100_000_000, 0.)
    odom.add_speed(100_000_000, .5)
    with pytest.raises(ValueError, match='FUTURE'):
        odom.drain(0, '0')


def test_local_pose_drives_unchanged_pp_and_scan_alignment():
    odom = TimeControlOdometry(.001)
    poses = [feed(odom, t, speed=.1)[0][0] for t in range(0, 300_000_001, 50_000_000)]
    observation = interpolate_body_pose(poses, 75_000_000)
    plan = TimePlan('local', observation, np.column_stack((np.arange(1, 31)*.02, np.zeros(30))))
    result = time_trial_control(plan, poses[-1], speed_mps=.1, rear_axle_offset_m=(.001, 0.))
    assert result['acceleration_mps2'] > 0
    assert result['steer_rad'] == 0.
    scan_pose = interpolate_body_pose(poses, 175_000_000)
    assert scan_pose_in_rear(scan_pose, poses[-1], .001) == pytest.approx((1.65-.0125-.001, 0., 0.))


def test_control_and_launch_have_no_external_pose_or_imu_subscriptions():
    root = Path(__file__).resolve().parents[1]
    runtime = root/'ros2_ws/src/aic_e2e_runtime'
    controller = (runtime/'aic_e2e_runtime/time_trial_controller_node.py').read_text()
    strings = [n.value for n in ast.walk(ast.parse(controller)) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert not any(s.startswith(('/localization/', '/sensing/imu/', '/sensing/gnss/', '/tf')) for s in strings)
    assert '--pose-source' not in strings
    assert 'time_control_odometry' in controller
    assert '--pose-source' not in (runtime/'launch/time_path_awsim.launch.py').read_text()
    assert '--pose-source' not in (root/'tools/run_time_path_trial_nodes.py').read_text()
