"""Higher targets retain the 1 m diagnostic, actual-speed PP, and old bounds."""
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_support_v2 import ONE_METRE_STOPPING_TRAVEL
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, trial_speed_limits, validate_trial_config
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.vehicle_motion_v1 import (
    AWSIM_POLICY, AWSIM_10KMH_POLICY, AWSIM_15KMH_POLICY, effective_response_length,
)
from aic_transfuser_lite.evaluation.time_speed_ladder_v1 import assess_speed_step


def test_fifteen_profile_changes_only_speed_and_motion_domain() -> None:
    root = Path(__file__).parents[1]/'configs/control'
    old = json.loads((root/'time_path_launch_10kmh_stop1m_20260917.json').read_bytes())
    new = json.loads((root/'time_path_launch_15kmh_stop1m_20260917.json').read_bytes())
    assert new == {**old, 'speed_policy':'fixed_15kmh', 'vehicle_model_policy':AWSIM_15KMH_POLICY,
                  'speed_cap_mps':15/3.6, 'overspeed_limit_mps':16/3.6}
    assert validate_trial_config(new) == 'fixed_15kmh'
    assert trial_speed_limits('fixed_15kmh') == (15/3.6, 16/3.6)
    for key, value in [('host','other'), ('record_vehicle_motion',False), ('speed_policy','fixed_10kmh'),
            ('vehicle_model_policy',AWSIM_10KMH_POLICY), ('stopping_distance_policy','measured_speed_v1'),
            ('stopping_distance_policy','awsim_cap_5kmh_diagnostic_v1'), ('diagnostic_only',False),
            ('lookahead_policy','fixed_1m_v1'), ('overspeed_limit_mps',20/3.6)]:
        with pytest.raises(ValueError):
            validate_trial_config({**new,key:value})


@pytest.mark.parametrize('curvature', [-.03, 0., .03])
def test_full_speed_synthetic_path_reaches_unchanged_pp_and_one_metre_guard(curvature: float) -> None:
    speed = 15/3.6
    pose = TimedBodyPose(0,'sim','0','map','base_link',0.,0.,0.)
    d = np.arange(1,31)*.1*4.8
    xy = np.column_stack([np.sin(curvature*d)/curvature, (1-np.cos(curvature*d))/curvature]) if curvature else np.column_stack([d,np.zeros(30)])
    before = xy.copy(); plan = TimePlan('fifteen',pose,xy)
    result = time_trial_control(plan,pose,speed_mps=speed,rear_axle_offset_m=(0.,0.),
        speed_policy='fixed_15kmh',vehicle_model_policy=AWSIM_15KMH_POLICY,lookahead_policy='stopping_preview_extended_v1')
    assert result['target_speed_mps'] == speed
    assert result['minimum_preview_distance_m'] == pytest.approx(11.163888888888891)
    assert result['selected_lookahead_distance_m'] >= result['minimum_preview_distance_m']
    assert abs(result['steer_rad']) <= .3
    np.testing.assert_array_equal(xy,before)
    guard = check_turning_scan(np.full(750,np.inf),-np.pi,2*np.pi/750,.1,25.,
        speed_mps=speed,measured_steer_rad=0.,issued_steer_rad=0.,previous_steer_rad=0.,
        scan_in_current_rear=(1.649,0.,0.),heading_rate_radps=0.,reported_lateral_mps=0.,
        vehicle_model_policy=AWSIM_15KMH_POLICY,envelope_policy='curvature_support_v2',
        stopping_distance_policy=ONE_METRE_STOPPING_TRAVEL)
    assert guard['stopping_travel_m'] == 1.
    assert guard['vehicle_motion']['calibrated_at_trial_speed'] is False
    assert guard['vehicle_motion']['nominal_response_length_m'] == pytest.approx(1.087+.045*speed**2)


def test_fifteen_does_not_bypass_short_prediction_or_old_speed_domains() -> None:
    for policy, limit in [(AWSIM_POLICY,6), (AWSIM_10KMH_POLICY,11), (AWSIM_15KMH_POLICY,16)]:
        with pytest.raises(ValueError,match='VEHICLE_MODEL_SPEED'):
            effective_response_length(limit/3.6+.0001,policy)
    pose = TimedBodyPose(0,'sim','0','map','base_link',0.,0.,0.)
    plan = TimePlan('short',pose,np.column_stack([np.arange(1,31)*.25,np.zeros(30)]))
    with pytest.raises(ValueError,match='STEERING_FEASIBLE_LOOKAHEAD_MISSING'):
        time_trial_control(plan,pose,speed_mps=15/3.6,rear_axle_offset_m=(0.,0.),speed_policy='fixed_15kmh',
                           vehicle_model_policy=AWSIM_15KMH_POLICY,lookahead_policy='stopping_preview_extended_v1')
    for speed_policy,model in [('fixed_15kmh',AWSIM_10KMH_POLICY),('fixed_10kmh',AWSIM_15KMH_POLICY),('fixed_5kmh',AWSIM_15KMH_POLICY)]:
        with pytest.raises(ValueError,match='TRIAL_SPEED_MODEL_CONTRACT'):
            time_trial_control(plan,pose,speed_mps=0.,rear_axle_offset_m=(0.,0.),speed_policy=speed_policy,vehicle_model_policy=model)


@pytest.mark.parametrize('complete,kmh,status,next_target', [
    (True,14.5,'PASS',20), (True,13.5,'PASS',20), (True,11.5,'PASS',20),
    (False,14.5,'LAP_NOT_COMPLETED',None), (True,0.,'PASS',20)])
def test_step_acceptance_uses_observed_speed_and_completion(complete: bool, kmh: float, status: str, next_target: int | None) -> None:
    result = assess_speed_step(15.,complete,np.array([0.]*5+[kmh/3.6]*100))
    assert result['status'] == status and result['next_target_kmh'] == next_target
    assert result['target_speed_reached'] == (kmh >= 13.5)


@pytest.mark.parametrize('samples', [np.array([[1.]]),np.array([float('nan')]),np.array([float('inf')]),np.array([-.031])])
def test_step_rejects_bad_measurement_shape_and_values(samples: np.ndarray) -> None:
    with pytest.raises(ValueError,match='SPEED_STEP_INPUT'):
        assess_speed_step(15.,True,samples)
