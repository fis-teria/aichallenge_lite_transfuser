"""Analytic time/distance alignment and preserved independent stop monitoring."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_speed_v1 import preview_speed_limit
from aic_transfuser_lite.control.curvature_support_v2 import stopping_envelope_parameters
from aic_transfuser_lite.control.time_preview_v1 import (
    TIME_LOOKAHEAD_POLICY, time_preview_distance, time_horizon_speed_cap,
)
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_20KMH_POLICY, stopping_motion
from tools.evaluate_time_awsim_trial import replay_recorded_control


POLICY = 'curvature_time_preview_20kmh_v1'
OPTIONS = dict(speed_policy=POLICY, lookahead_policy=TIME_LOOKAHEAD_POLICY,
               vehicle_model_policy=AWSIM_20KMH_POLICY)


def curve(speed: float, curvature: float = 0.) -> np.ndarray:
    arc = np.arange(1, 31) * .1 * speed
    return (np.column_stack([np.sin(curvature*arc)/curvature, (1-np.cos(curvature*arc))/curvature])
            if curvature else np.column_stack([arc, np.zeros(30)]))


@pytest.mark.parametrize('kmh', [5., 10., 15., 20.])
def test_linear_lookahead_and_inverse_fit_three_seconds_of_constant_motion(kmh: float) -> None:
    v = kmh/3.6
    assert time_preview_distance(v) == pytest.approx(max(1., .4+1.5*v))
    cap = time_horizon_speed_cap(3*v, reserve_m=.5, extra_delay_s=.3)
    assert cap > v
    assert time_preview_distance(cap)+.5+.3*cap == pytest.approx(3*v)


@pytest.mark.parametrize('distance', [0., .9, 1.49, 1.5, 1.55, 2., 8., 20.])
def test_inverse_respects_minimum_distance_branch_and_reserve(distance: float) -> None:
    cap = time_horizon_speed_cap(distance, reserve_m=.5, extra_delay_s=.3)
    if distance < 1.5:
        assert cap == 0.
    else:
        assert time_preview_distance(cap)+.5+.3*cap == pytest.approx(distance)
        assert time_preview_distance(cap+1e-5)+.5+.3*(cap+1e-5) > distance


@pytest.mark.parametrize('invalid', [-.01, float('nan'), float('inf')])
def test_invalid_units_and_nonfinite_inputs_fail(invalid: float) -> None:
    with pytest.raises(ValueError, match='TIME_PREVIEW_SPEED'):
        time_preview_distance(invalid)
    with pytest.raises(ValueError, match='TIME_PREVIEW_HORIZON'):
        time_horizon_speed_cap(invalid, reserve_m=.5, extra_delay_s=.3)


@pytest.mark.parametrize('age', [0., .2, .5])
def test_twenty_kmh_original_timed_path_reaches_pp_without_extrapolation(age: float) -> None:
    v = 20/3.6
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    current = TimedBodyPose(round(age*1e9), 'sim', '0', 'map', 'base_link', age*v, 0., 0.)
    xy = curve(v); before = xy.copy(); plan = TimePlan('straight', pose, xy)
    result = time_trial_control(plan, current, speed_mps=v, rear_axle_offset_m=(0., 0.), **OPTIONS)
    assert result['target_speed_mps'] == pytest.approx(v)
    assert result['steer_rad'] == 0.
    assert result['minimum_preview_distance_m'] == pytest.approx(8.733333333333333)
    assert result['selected_lookahead_distance_m'] >= result['minimum_preview_distance_m']
    assert 0 < result['lookahead_selection']['remaining_s'] <= 3-age
    assert result['longitudinal_preview']['endpoint_distance_m'] == pytest.approx((3-age)*v)
    assert result['longitudinal_preview']['horizon_contract']['stopping_distance_used_for_pp'] is False
    np.testing.assert_array_equal(xy, before)
    np.testing.assert_array_equal(plan.xy_m, before)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_time_policy_still_slows_for_curvature_and_keeps_tire_feasibility(sign: float) -> None:
    v = 20/3.6; curvature = sign*.08
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    result = time_trial_control(TimePlan('bend', pose, curve(v, curvature)), pose,
        speed_mps=v, rear_axle_offset_m=(0., 0.), **OPTIONS)
    assert result['target_speed_mps'] <= math.sqrt(1/abs(curvature))+1e-6
    assert result['acceleration_mps2'] < 0
    assert abs(result['steer_rad']) <= .3


def test_time_preview_never_shortens_to_fit_an_inadequate_prediction() -> None:
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    with pytest.raises(ValueError, match='STEERING_FEASIBLE_LOOKAHEAD_MISSING'):
        time_trial_control(TimePlan('too-short', pose, curve(2.)), pose,
            speed_mps=20/3.6, rear_axle_offset_m=(0., 0.), **OPTIONS)
    with pytest.raises(ValueError, match='TRIAL_SPEED_CONTRACT'):
        time_trial_control(TimePlan('overspeed', pose, curve(6.)), pose,
            speed_mps=21/3.6+.001, rear_axle_offset_m=(0., 0.), **OPTIONS)


def test_braking_envelope_remains_quadratic_and_independent() -> None:
    v = 20/3.6
    motion = stopping_motion(v, 0., 0., policy=AWSIM_20KMH_POLICY,
        heading_rate_radps=0., reported_lateral_mps=0.)
    travel, _, _ = stopping_envelope_parameters(v, motion)
    assert travel == pytest.approx(.4+.5*v+v*v/2)
    assert travel == pytest.approx(18.609876543209875)
    assert travel > 2*time_preview_distance(v)


def test_longitudinal_time_cap_slows_before_linear_lookahead_exhaustion() -> None:
    v = 3.
    points = np.column_stack([np.linspace(0., 5.4, 31), np.zeros(31)])
    result = preview_speed_limit(points, measured_speed_mps=v, cruise_ceiling_mps=20/3.6,
        tracking_curvature_per_m=0., policy=POLICY)
    assert time_preview_distance(v) < 5.4
    assert result['target_speed_mps'] < v and result['acceleration_mps2'] < 0
    assert result['limiting_reason'] == 'prediction_horizon'


@pytest.mark.parametrize('kmh', [15, 20])
def test_explicit_configs_retain_scene_and_log_only_scope(kmh: int) -> None:
    root = Path(__file__).parents[1]/'configs/control'
    old = json.loads((root/'time_path_curvature_logonly_20260917.json').read_bytes())
    new = json.loads((root/f'time_path_timepreview_{kmh}kmh_20260917.json').read_bytes())
    policy = f'curvature_time_preview_{kmh}kmh_v1'
    assert new == {**old, 'speed_policy': policy, 'lookahead_policy': TIME_LOOKAHEAD_POLICY,
        'vehicle_model_policy': f'awsim_understeer_{kmh}kmh_trial_v1',
        'speed_cap_mps':kmh/3.6, 'overspeed_limit_mps':(kmh+1)/3.6}
    assert validate_trial_config(new) == policy
    for key, value in [('host','other'), ('diagnostic_only',False), ('maximum_diagnostic_trials',2),
            ('lookahead_policy','stopping_preview_extended_v1'), ('speed_cap_mps',25/3.6),
            ('vehicle_model_policy','awsim_understeer_v1')]:
        with pytest.raises(ValueError):
            validate_trial_config({**new, key:value})


def test_replay_detects_time_preview_and_horizon_contract_tampering() -> None:
    v=20/3.6
    pose=TimedBodyPose(0,'sim','0','map','base_link',0.,0.,0.)
    plan=TimePlan('replay-time',pose,curve(v))
    details=time_trial_control(plan,pose,speed_mps=v,rear_axle_offset_m=(0.,0.),**OPTIONS)
    details.update(current_pose=pose.__dict__,observation_pose=pose.__dict__)
    command=dict(plan_id=plan.plan_id,reason='TIME_PATH_TRACKING',speed_mps=v,sim_ns=0,
        target_speed_mps=details['target_speed_mps'],acceleration_mps2=details['acceleration_mps2'],details=details)
    plans=[dict(plan_id=plan.plan_id,raw_xy_m=plan.xy_m.tolist())]
    assert replay_recorded_control([command],plans,0.,**OPTIONS)['matched_commands']==1
    for key in ('minimum_preview_distance_m','selected_lookahead_distance_m'):
        bad=deepcopy(command);bad['details'][key]+=.1
        with pytest.raises(ValueError,match='recorded time preview differs'):
            replay_recorded_control([bad],plans,0.,**OPTIONS)
    bad=deepcopy(command)
    bad['details']['longitudinal_preview']['horizon_contract']['preview_time_s']=3.
    with pytest.raises(ValueError,match='recorded longitudinal preview differs'):
        replay_recorded_control([bad],plans,0.,**OPTIONS)
