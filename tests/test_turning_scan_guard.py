from dataclasses import replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.control.long_sim_tracking_v4 import check_scan
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan, scan_pose_in_rear, stopping_sweep


ANGLES = np.linspace(-math.pi/2, math.pi/2, 751)


def scan_with_hit(x_m, y_m):
    """Synthetic obstacle in current rear axle coordinates, metres."""
    ray = int(np.argmin(abs(ANGLES-np.arctan2(y_m, x_m-1.649))))
    r = np.full(len(ANGLES), np.inf)
    r[ray] = np.hypot(x_m-1.649, y_m)
    return r


def check(r=None, measured=0., issued=0., **kwargs):
    return check_turning_scan(np.full(751, np.inf) if r is None else r, ANGLES[0], ANGLES[1]-ANGLES[0], 0., 25.,
        speed_mps=kwargs.pop('speed_mps', 1.2381234169), measured_steer_rad=measured,
        issued_steer_rad=issued, scan_in_current_rear=kwargs.pop('scan_in_current_rear', (1.649, 0., 0.)), **kwargs)


def test_clear_straight_and_mirrored_corners():
    for sign in (-1, 1):
        assert check(measured=sign*.3, issued=sign*.3)['checked_rays'] == 751
    result = check()
    assert result['policy'] == 'STEERING_INTERVAL_SWEEP_V1'
    assert result['full_body_free_space_verified'] is False


def test_turn_clears_outer_corner_but_straight_and_steering_lag_reject():
    r = scan_with_hit(3.77, .839)
    with pytest.raises(ValueError, match='CORRIDOR_OCCUPIED'):
        check_scan(r, ANGLES[0], ANGLES[1]-ANGLES[0], 0., 25., 1.25)
    assert check(r, measured=-.12, issued=-.12)['minimum_ray_margin_m'] > 0
    with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
        check(r, measured=0., issued=-.12)
    with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
        check(r, measured=-.12, issued=-.12, previous_steer_rad=0.)
    with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
        check(r)


def test_turning_into_inner_obstacle_and_front_obstacle_still_reject():
    for sign in (-1, 1):
        with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
            check(scan_with_hit(3., sign*1.2), measured=sign*.3, issued=sign*.3)
        with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
            check(scan_with_hit(2., 0.), measured=sign*.3, issued=sign*.3)


@pytest.mark.parametrize('bad', [np.nan, -np.inf, -.1, 25.1])
def test_unknown_rays_never_become_free(bad):
    r = np.full(751, np.inf); r[300] = bad
    with pytest.raises(ValueError, match='SCAN_UNKNOWN'):
        check(r, measured=-.1, issued=-.1)


@pytest.mark.parametrize('bad', [np.zeros((751, 1)), np.zeros(10)])
def test_bad_shapes(bad):
    with pytest.raises(ValueError, match='SCAN_CONTRACT'):
        check(bad)


def test_fov_range_exhaustion_and_pose_misalignment_reject():
    with pytest.raises(ValueError, match='SCAN_COVERAGE'):
        check_turning_scan(np.full(751, np.inf), -.5, .001, 0., 25., speed_mps=1.,
            measured_steer_rad=0., issued_steer_rad=0., scan_in_current_rear=(1.649, 0., 0.))
    with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
        check(np.full(751, 1.))
    with pytest.raises(ValueError, match='SCAN_POSE_ALIGNMENT'):
        check(scan_in_current_rear=(0., 0., 0.))


@pytest.mark.parametrize('speed,actual,issued', [(2., 0., 0.), (-.1, 0., 0.), (1., .51, 0.), (1., 0., -.51), (1., np.nan, 0.)])
def test_vehicle_limits(speed, actual, issued):
    with pytest.raises(ValueError, match='SWEEP_VEHICLE_STATE'):
        stopping_sweep(speed, actual, issued)


def test_capture_pose_is_not_relabelled_and_units_are_metres():
    captured = TimedBodyPose(100_000_000, 'sim', '0', 'map', 'base_link', 10., 20., math.pi/2)
    current = replace(captured, stamp_ns=200_000_000, y_m=20.1)
    np.testing.assert_allclose(scan_pose_in_rear(captured, current, .001), [1.548999976158142, 0., 0.], atol=1e-9)
    with pytest.raises(ValueError, match='SCAN_POSE_IDENTITY_OR_AGE'):
        scan_pose_in_rear(captured, replace(current, epoch='1'), .001)
    with pytest.raises(ValueError, match='SCAN_POSE_IDENTITY_OR_AGE'):
        scan_pose_in_rear(captured, replace(current, stamp_ns=300_000_000), .001)


def test_varying_steering_trajectory_is_contained_by_sampled_interval_tube():
    # Compare an independently integrated alternating curvature trajectory
    # against the conservative tube, including a front-left body corner.
    actual, issued = -.3, -.05
    poses, inflate = stopping_sweep(1.3, actual, issued)
    kmin, kmax = np.tan([actual, issued])/1.087
    travel = .4+1.3*.5+1.3**2/2.
    distances = np.linspace(0., travel, len(poses))
    point = np.zeros(2); yaw = 0.
    for i in range(1, len(poses)):
        ds = (distances[i]-distances[i-1])/40.
        for j in range(40):
            k = kmin if (i+j)%3 else kmax
            point += ds*np.array([np.cos(yaw+k*ds/2), np.sin(yaw+k*ds/2)])
            yaw += k*ds
        corner = point+np.array([1.984*np.cos(yaw)-.85*np.sin(yaw), 1.984*np.sin(yaw)+.85*np.cos(yaw)])
        reference = poses[i, :2]+np.array([1.984*np.cos(poses[i, 2])-.85*np.sin(poses[i, 2]),
                                         1.984*np.sin(poses[i, 2])+.85*np.cos(poses[i, 2])])
        assert np.linalg.norm(corner-reference) <= inflate[i]+1e-8
