"""AWSIM-only proximity stop suppression retains malformed-input guards."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from aic_transfuser_lite.control.awsim_steering import CALIBRATED_POLICY, command_steering
from aic_transfuser_lite.control.curvature_speed_v1 import ADAPTIVE_SPEED_POLICY
from aic_transfuser_lite.control.curvature_support_v2 import SCAN_LOG_ONLY_POLICY, ONE_METRE_STOPPING_TRAVEL
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_15KMH_POLICY
from tools.evaluate_time_awsim_trial import replay_recorded_control


def options() -> dict:
    return dict(speed_mps=2., measured_steer_rad=0., issued_steer_rad=0., previous_steer_rad=0.,
        scan_in_current_rear=(1.649,0.,0.), envelope_policy='curvature_support_v2',
        vehicle_model_policy=AWSIM_15KMH_POLICY, heading_rate_radps=0., reported_lateral_mps=0.,
        stopping_distance_policy=ONE_METRE_STOPPING_TRAVEL, occupancy_policy=SCAN_LOG_ONLY_POLICY)


def test_recorded_rejection_is_reported_without_raising() -> None:
    data=json.loads((Path(__file__).parent/'fixtures/time_path/curvature_margin_scan.json').read_bytes())
    scan=data['scan']
    kwargs={**options(), **{k:data[k] for k in ('speed_mps','measured_steer_rad','issued_steer_rad',
        'previous_steer_rad','scan_in_current_rear','heading_rate_radps','reported_lateral_mps')}}
    args=(np.asarray(scan['ranges'],float),scan['angle_min'],scan['angle_increment'],scan['range_min'],scan['range_max'])
    result=check_turning_scan(*args,**kwargs)
    assert result['proximity_stop_enforced'] is False
    assert result['would_stop_reason']=='STOPPING_SWEEP_OCCUPIED' and result['occupied_ray_count']==2
    assert result['minimum_ray_margin_m']==pytest.approx(data['expected'][0]['minimum_ray_margin_m'])
    assert result['stopping_travel_m']==1.
    with pytest.raises(ValueError,match='STOPPING_SWEEP_OCCUPIED'):
        check_turning_scan(*args,**{**kwargs,'occupancy_policy':'stop_v1'})


@pytest.mark.parametrize('invalid,reason',[(float('nan'),'SCAN_UNKNOWN'),(-float('inf'),'SCAN_UNKNOWN'),(.01,'SCAN_UNKNOWN')])
def test_bad_scan_still_rejects(invalid: float, reason: str) -> None:
    ranges=np.full(750,np.inf);ranges[375]=invalid
    with pytest.raises(ValueError,match=reason):
        check_turning_scan(ranges,-np.pi,2*np.pi/750,.1,25.,**options())


def test_motion_checks_and_scope_are_not_disabled() -> None:
    for changes in [dict(heading_rate_radps=float('nan')),dict(reported_lateral_mps=.2),
                    dict(occupancy_policy='disabled'),dict(vehicle_model_policy='awsim_understeer_v1'),
                    dict(stopping_distance_policy='measured_speed_v1'),dict(envelope_policy='isotropic_v1')]:
        with pytest.raises(ValueError):
            check_turning_scan(np.full(750,np.inf),-np.pi,2*np.pi/750,.1,25.,**{**options(),**changes})
    result=check_turning_scan(np.full(750,np.inf),-np.pi,2*np.pi/750,.1,25.,**options())
    assert result['would_stop_reason'] is None and result['occupied_ray_count']==0


def test_only_explicit_bounded_simulator_configuration_can_disable_stop() -> None:
    root=Path(__file__).parents[1]/'configs/control'
    old=json.loads((root/'time_path_curvature_preview_20260917.json').read_bytes())
    new=json.loads((root/'time_path_curvature_logonly_20260917.json').read_bytes())
    assert new=={**old,'scan_occupancy_policy':SCAN_LOG_ONLY_POLICY}
    assert validate_trial_config(new)==ADAPTIVE_SPEED_POLICY
    for key,value in [('host','other'),('maximum_diagnostic_trials',2),('diagnostic_only',False),
                      ('speed_policy','fixed_15kmh'),('obstacle_policy','straight_v1'),
                      ('stopping_distance_policy','measured_speed_v1'),('scan_occupancy_policy','disabled')]:
        with pytest.raises(ValueError):validate_trial_config({**new,key:value})


def test_replay_accepts_logged_proximity_hit_and_rejects_policy_tampering() -> None:
    pose=TimedBodyPose(0,'sim','0','map','base_link',0.,0.,0.)
    plan=TimePlan('observe',pose,np.column_stack([np.arange(1,31)*.3,np.zeros(30)]))
    result=time_trial_control(plan,pose,speed_mps=2.,rear_axle_offset_m=(0.,0.),
        speed_policy=ADAPTIVE_SPEED_POLICY,vehicle_model_policy=AWSIM_15KMH_POLICY,
        lookahead_policy='stopping_preview_extended_v1')
    ranges=np.full(750,np.inf);ranges[375]=1.
    guard=check_turning_scan(ranges,-np.pi,2*np.pi/750,.1,25.,**options())
    mapping=command_steering(result['steer_rad'],0.,.05,policy=CALIBRATED_POLICY)
    details={**result,'current_pose':pose.__dict__,'observation_pose':pose.__dict__,
             'steering_actuator':mapping,'obstacle_guard':guard}
    command=dict(plan_id='observe',sim_ns=0,reason='TIME_PATH_TRACKING',speed_mps=2.,
        steer_rad=mapping['issued_input_rad'],measured_steer_rad=0.,
        target_speed_mps=result['target_speed_mps'],acceleration_mps2=result['acceleration_mps2'],details=details,
        motion_observation=dict(frame='base_link',heading_rate_radps=0.,reported_lateral_mps=0.))
    kwargs=dict(speed_policy=ADAPTIVE_SPEED_POLICY,obstacle_policy='steering_support_v2',
        steering_policy=CALIBRATED_POLICY,lookahead_policy='stopping_preview_extended_v1',
        vehicle_model_policy=AWSIM_15KMH_POLICY,stopping_distance_policy=ONE_METRE_STOPPING_TRAVEL,
        scan_occupancy_policy=SCAN_LOG_ONLY_POLICY)
    plans=[dict(plan_id='observe',raw_xy_m=plan.xy_m.tolist())]
    assert replay_recorded_control([command],plans,0.,**kwargs)['status']=='PASS'
    for key,value in [('proximity_stop_enforced',True),('occupied_ray_count',0),('minimum_ray_margin_m',float('nan'))]:
        bad=deepcopy(command);bad['details']['obstacle_guard'][key]=value
        with pytest.raises(ValueError,match='recorded log-only scan policy differs'):
            replay_recorded_control([bad],plans,0.,**kwargs)
