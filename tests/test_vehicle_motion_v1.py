import json
import math
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.vehicle_motion_v1 import (
    IDEAL_POLICY, AWSIM_POLICY, WHEELBASE_M, MAX_CURVATURE_PER_M, COM_FORWARD_OF_REAR_M,
    body_curvature_for_tire, physical_tire_for_curvature, effective_response_length, stopping_motion,
)
from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.awsim_steering import CALIBRATED_POLICY, command_steering
from tools.evaluate_time_awsim_trial import replay_recorded_control


@pytest.mark.parametrize('speed', [0., .1, .2, .8, 1.3, 6/3.6])
@pytest.mark.parametrize('curvature', [-.23, 0., .23])
def test_nominal_forward_inverse_units_sign_and_static_length(speed, curvature):
    for policy in (IDEAL_POLICY, AWSIM_POLICY):
        tire = physical_tire_for_curvature(curvature, speed, policy)
        assert body_curvature_for_tire(tire, speed, policy) == pytest.approx(curvature)
    old = physical_tire_for_curvature(curvature, speed)
    assert old == math.atan(curvature*1.087)
    new = physical_tire_for_curvature(curvature, speed, AWSIM_POLICY)
    assert abs(new) >= abs(old)
    assert effective_response_length(0., AWSIM_POLICY) == WHEELBASE_M


@pytest.mark.parametrize('speed', [-.1, 2., float('nan'), float('inf')])
def test_speed_domain_rejects(speed):
    with pytest.raises(ValueError, match='MODEL_SPEED'):
        effective_response_length(speed, AWSIM_POLICY)


def test_unknown_policy_and_nonfinite_shape_values_reject():
    with pytest.raises(ValueError, match='MODEL_POLICY'):
        effective_response_length(1., 'unverified')
    with pytest.raises(ValueError, match='MODEL_CURVATURE'):
        physical_tire_for_curvature(float('nan'), 1.)
    for angle in (.51, float('nan')):
        with pytest.raises(ValueError, match='MODEL_TIRE'):
            body_curvature_for_tire(angle, 1.)


@pytest.mark.parametrize('sign', [-1, 1])
def test_braking_bounds_cover_every_speed_parameter_angle_and_measured_body_yaw(sign):
    speed = 1.3
    angles = sign*np.array([.11, .15, .09])
    # Body transient is outside the angle-derived interval; it must be retained.
    yaw = sign*.04
    result = stopping_motion(speed, *angles, policy=AWSIM_POLICY, heading_rate_radps=yaw,
                             reported_lateral_mps=COM_FORWARD_OF_REAR_M*yaw)
    low, high = result['curvature_interval_per_m']
    assert low <= yaw/speed <= high
    for v in np.linspace(0., speed, 11):
        for gradient in np.linspace(0., .2, 11):
            for angle in np.linspace(min(angles), max(angles), 11):
                k = math.tan(angle)/(1.087+gradient*v*v)
                assert low-1e-14 <= k <= high+1e-14
    assert result['rear_lateral_mps'] == 0.
    assert result['lateral_displacement_bound_m'] == pytest.approx(.03*(.5+speed))
    assert result['static_wheelbase_m'] == 1.087


@pytest.mark.parametrize('speed', [0., .1, .199])
def test_low_speed_never_divides_by_near_zero_or_assumes_zero_yaw(speed):
    r = stopping_motion(speed, .1, .1, policy=AWSIM_POLICY, heading_rate_radps=.01,
                        reported_lateral_mps=COM_FORWARD_OF_REAR_M*.01)
    assert r['curvature_interval_per_m'] == [-MAX_CURVATURE_PER_M, MAX_CURVATURE_PER_M]
    assert r['measured_curvature_per_m'] is None


@pytest.mark.parametrize('yaw,lateral,reason', [(None, 0., 'MEASUREMENT_REQUIRED'),
    (0., None, 'MEASUREMENT_REQUIRED'), (float('nan'), 0., 'YAW_RATE_INVALID'),
    (180., 0., 'YAW_RATE_INVALID'), (.8, 0., 'YAW_RATE_INVALID'),
    (0., float('inf'), 'REAR_LATERAL_INVALID'), (0., .03001, 'REAR_LATERAL_INVALID')])
def test_bad_measured_motion_rejects_instead_of_clamping(yaw, lateral, reason):
    with pytest.raises(ValueError, match=reason):
        stopping_motion(1.3, .1, .1, policy=AWSIM_POLICY, heading_rate_radps=yaw, reported_lateral_mps=lateral)


def test_pinned_config_requires_same_scene_and_steering_and_support_model():
    c = json.loads((Path(__file__).parents[1]/'configs/control/time_path_vehicle_model_5kmh_20260913.json').read_text())
    assert validate_trial_config(c) == 'fixed_5kmh'
    for key, value in [('obstacle_policy', 'straight_v1'), ('speed_policy', 'source_capped_0p25'),
                       ('geometry', {**c['geometry'], 'wheelbase_m': 1.2}),
                       ('geometry', {**c['geometry'], 'scene_sha256': '0'*64})]:
        with pytest.raises(ValueError, match='VEHICLE_MODEL_ASSET_OR_POLICY_CONTRACT'):
            validate_trial_config({**c, key: value})


@pytest.mark.parametrize('direction', [-1., 1.])
def test_original_pp_points_and_speed_preserved_with_shared_inverse_and_replay(direction):
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    k = direction*.13
    s = np.arange(1,31)*.15
    xy = np.column_stack((np.sin(k*s)/k, (1-np.cos(k*s))/k))
    plan = TimePlan('test', pose, xy.copy())
    options = dict(speed_mps=1.3, rear_axle_offset_m=(0., 0.), speed_policy='fixed_5kmh', lookahead_policy='stopping_preview_v1')
    old = time_trial_control(plan, pose, **options)
    new = time_trial_control(plan, pose, **options, vehicle_model_policy=AWSIM_POLICY)
    np.testing.assert_array_equal(new['reference_xy_rear_m'], old['reference_xy_rear_m'])
    np.testing.assert_array_equal(plan.xy_m, xy)
    assert new['lookahead_rear_m'] == old['lookahead_rear_m']
    assert new['target_speed_mps'] == 5/3.6
    assert abs(new['steer_rad']) > abs(old['steer_rad'])
    assert body_curvature_for_tire(new['steer_rad'], 1.3, AWSIM_POLICY) == pytest.approx(k, abs=2e-8)
    mapping = command_steering(new['steer_rad'], new['steer_rad']/.6, .05, policy=CALIBRATED_POLICY)
    yaw = 1.3*k
    lateral = COM_FORWARD_OF_REAR_M*yaw
    motion = stopping_motion(1.3, new['steer_rad'], mapping['issued_tire_target_rad'],
        mapping['previous_tire_target_rad'], policy=AWSIM_POLICY, heading_rate_radps=yaw, reported_lateral_mps=lateral)
    details = {**new, 'current_pose': pose.__dict__, 'observation_pose': pose.__dict__,
               'steering_actuator': mapping, 'obstacle_guard': {'vehicle_motion': motion}}
    command = {'plan_id': 'test', 'reason': 'TIME_PATH_TRACKING', 'speed_mps': 1.3,
        'steer_rad': mapping['issued_input_rad'], 'measured_steer_rad': new['steer_rad'], 'details': details,
        'motion_observation': {'frame': 'base_link', 'heading_rate_radps': yaw, 'reported_lateral_mps': lateral}}
    args = dict(speed_policy='fixed_5kmh', obstacle_policy='steering_support_v2',
                steering_policy=CALIBRATED_POLICY, lookahead_policy='stopping_preview_v1', vehicle_model_policy=AWSIM_POLICY)
    saved_plan = {'plan_id': 'test', 'raw_xy_m': xy.tolist()}
    assert replay_recorded_control([command], [saved_plan], 0., **args)['vehicle_motion_matched'] == 1
    bad = deepcopy(command); bad['details']['obstacle_guard']['vehicle_motion']['curvature_interval_per_m'][0] += .01
    with pytest.raises(ValueError, match='stopping motion differs'):
        replay_recorded_control([bad], [saved_plan], 0., **args)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_independent_braking_vehicle_with_lateral_motion_is_contained(direction):
    v0 = 1.3
    motion = stopping_motion(v0, direction*.12, direction*.2, direction*.08, policy=AWSIM_POLICY,
                            heading_rate_radps=direction*.1, reported_lateral_mps=direction*.0175)
    kmin, kmax = motion['curvature_interval_per_m']
    travel = .4+v0*.5+v0*v0/2
    n, h, _ = curvature_support_envelope(kmin, kmax, travel, .004, np.array([1.649,0.]),
                                         lateral_padding_m=motion['lateral_displacement_bound_m'])
    corners = np.array([[-.510,-.85],[1.984,-.85],[1.984,.85],[-.510,.85]])
    # Independent time integration, with braking, variable physical steering,
    # K and signed lateral speed. It does not call the nominal conversion.
    p = np.zeros(2); yaw = 0.; dt = .0005
    for t in np.arange(0., .5+v0, dt):
        v = max(0., v0-max(0.,t-.5))
        angle = direction*(.08+.12*(.5+.5*math.sin(13*t)))
        gradient = .1+.1*math.sin(7*t)
        omega = v*math.tan(angle)/(1.087+gradient*v*v)
        lateral = .03*math.cos(11*t)
        mid = yaw+omega*dt/2
        p += dt*np.array([v*math.cos(mid)-lateral*math.sin(mid), v*math.sin(mid)+lateral*math.cos(mid)])
        yaw += omega*dt
        c, s = math.cos(yaw), math.sin(yaw)
        assert np.max((corners@np.array([[c,s],[-s,c]])+p)@n.T-h) <= 1e-8


def test_recorded_side_clearance_rejects_with_motion_uncertainty_preserved():
    f = json.loads((Path(__file__).parent/'fixtures/time_path/turn16_side_margin_rejection.json').read_text())
    scan = f['scan']
    args = (np.array(scan['ranges'],float), scan['angle_min'], scan['angle_increment'], scan['range_min'], scan['range_max'])
    options = {k:f[k] for k in ('speed_mps','measured_steer_rad','issued_steer_rad','previous_steer_rad','scan_in_current_rear')}
    options['envelope_policy'] = 'curvature_support_v2'
    old = check_turning_scan(*args, **options)
    assert old['minimum_ray_margin_m'] == pytest.approx(.12044124271057566)
    # The old margin omitted sideways travel. A tiny new rejection margin is
    # still a rejection; do not drop physical uncertainty to pass this scan.
    with pytest.raises(ValueError, match='STOPPING_SWEEP_OCCUPIED'):
        check_turning_scan(*args, **options, vehicle_model_policy=AWSIM_POLICY,
            heading_rate_radps=f['motion_observation']['heading_rate_radps'],
            reported_lateral_mps=f['motion_observation']['reported_lateral_mps'])
