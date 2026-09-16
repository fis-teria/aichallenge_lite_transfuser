"""Explicit simulator comparison: 10 km/h driving with a 5 km/h scan horizon."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_support_v2 import (
    ACTUAL_STOPPING_SPEED, FIVE_KMH_STOPPING_SPEED, stopping_envelope_parameters,
)
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_10KMH_POLICY, AWSIM_POLICY, stopping_motion
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin


def check(ranges: np.ndarray, speed: float, policy: str = FIVE_KMH_STOPPING_SPEED, **overrides: object) -> dict:
    kwargs = dict(speed_mps=speed, measured_steer_rad=0., issued_steer_rad=0., previous_steer_rad=0.,
        scan_in_current_rear=(1.649, 0., 0.), envelope_policy='curvature_support_v2',
        vehicle_model_policy=AWSIM_10KMH_POLICY, heading_rate_radps=0., reported_lateral_mps=0.,
        stopping_distance_policy=policy)
    return check_turning_scan(ranges, -np.pi, 2*np.pi/750, .1, 25., **{**kwargs, **overrides})


def test_scope_keeps_previous_ten_kmh_config_and_pp_settings() -> None:
    root = Path(__file__).parents[1]/'configs/control'
    old = json.loads((root/'time_path_launch_10kmh_20260917.json').read_bytes())
    new = json.loads((root/'time_path_launch_10kmh_stop5_20260917.json').read_bytes())
    assert new == {**old, 'stopping_distance_policy': FIVE_KMH_STOPPING_SPEED,
                  'diagnostic_only': True, 'maximum_diagnostic_trials': 1}
    assert validate_trial_config(new) == 'fixed_10kmh'
    assert validate_trial_config(old) == 'fixed_10kmh'
    for key, bad in [('diagnostic_only', False), ('maximum_diagnostic_trials', True),
                     ('maximum_diagnostic_trials', 2), ('host', 'other'),
                     ('execution_profile', 'bounded_10s'), ('obstacle_policy', 'steering_sweep_v1'),
                     ('vehicle_model_policy', AWSIM_POLICY), ('speed_policy', 'fixed_5kmh'),
                     ('diagnostic_clearance_profile', 'awsim_near_limit_v1'), ('stopping_distance_policy', 'typo')]:
        with pytest.raises(ValueError):
            validate_trial_config({**new, key: bad})


@pytest.mark.parametrize('speed', [0., .2, 5/3.6, 10/3.6, 11/3.6])
def test_cap_changes_horizon_but_keeps_actual_motion(speed: float) -> None:
    clear = np.full(750, np.inf)
    actual = check(clear, speed, ACTUAL_STOPPING_SPEED)
    diagnostic = check(clear, speed)
    capped = min(speed, 5/3.6)
    assert diagnostic['stopping_travel_m'] == pytest.approx(.4+.5*capped+capped*capped/2)
    assert diagnostic['vehicle_motion'] == actual['vehicle_motion']
    assert diagnostic['measured_speed_mps'] == speed
    assert diagnostic['envelope_speed_mps'] == capped
    assert diagnostic['lateral_padding_m'] == pytest.approx(.03*(.5+capped))
    assert diagnostic['actual_speed_stopping_envelope'] is False
    if speed <= 5/3.6:
        for key in actual:
            assert diagnostic[key] == actual[key]


def test_distant_hit_passes_only_diagnostic_and_near_hit_still_rejects() -> None:
    ranges = np.full(750, np.inf)
    ranges[375] = 5.4-1.649
    with pytest.raises(ValueError, match='STOPPING_SWEEP_OCCUPIED'):
        check(ranges, 10/3.6, ACTUAL_STOPPING_SPEED)
    assert check(ranges, 10/3.6)['minimum_ray_margin_m'] > 0
    ranges[375] = 3.-1.649
    with pytest.raises(ValueError, match='STOPPING_SWEEP_OCCUPIED'):
        check(ranges, 10/3.6)
    replay = scan_margin(dict(ranges=ranges, angle_min=-np.pi, angle_increment=2*np.pi/750,
        range_min=.1, range_max=25.), (1.649, 0., 0.), speed_mps=10/3.6,
        measured_rad=0., issued_rad=0., previous_rad=0., yaw_rate_radps=0., lateral_mps=0.,
        vehicle_model_policy=AWSIM_10KMH_POLICY, stopping_distance_policy=FIVE_KMH_STOPPING_SPEED)
    assert replay['reason'] == 'STOPPING_SWEEP_OCCUPIED' and replay['minimum_ray_margin_m'] < 0


@pytest.mark.parametrize('overrides', [dict(envelope_policy='isotropic_v1'),
    dict(vehicle_model_policy=AWSIM_POLICY), dict(clearance_profile='awsim_near_limit_v1'),
    dict(reported_lateral_mps=.030001), dict(speed_mps=11/3.6+.001), dict(speed_mps=float('nan'))])
def test_diagnostic_does_not_hide_invalid_actual_state(overrides: dict) -> None:
    with pytest.raises(ValueError):
        check(np.full(750, np.inf), 5/3.6, **overrides)


def test_recorded_diagnostic_envelope_is_verified_and_tampering_rejected() -> None:
    from aic_transfuser_lite.control.awsim_steering import CALIBRATED_POLICY, command_steering
    from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
    from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
    from tools.evaluate_time_awsim_trial import replay_recorded_control
    speed = 10/3.6
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    plan = TimePlan('capped', pose, np.column_stack([np.arange(1, 31)*.3, np.zeros(30)]))
    options = dict(speed_policy='fixed_10kmh', vehicle_model_policy=AWSIM_10KMH_POLICY,
                   lookahead_policy='stopping_preview_extended_v1')
    calculated = time_trial_control(plan, pose, speed_mps=speed, rear_axle_offset_m=(0., 0.), **options)
    assert calculated['minimum_preview_distance_m'] == pytest.approx(5.646913580246914)
    mapping = command_steering(0., 0., .05, policy=CALIBRATED_POLICY)
    guard = check(np.full(750, np.inf), speed)
    command = dict(plan_id=plan.plan_id, reason='TIME_PATH_TRACKING', speed_mps=speed,
        steer_rad=0., measured_steer_rad=0., motion_observation=dict(frame='base_link', heading_rate_radps=0., reported_lateral_mps=0.),
        details={**calculated, 'current_pose': pose.__dict__, 'observation_pose': pose.__dict__,
                 'steering_actuator': mapping, 'obstacle_guard': guard})
    saved = [dict(plan_id=plan.plan_id, raw_xy_m=plan.xy_m.tolist())]
    kwargs = dict(**options, obstacle_policy='steering_support_v2', steering_policy=CALIBRATED_POLICY,
                  stopping_distance_policy=FIVE_KMH_STOPPING_SPEED)
    assert replay_recorded_control([command], saved, 0., **kwargs)['matched_commands'] == 1
    for key, wrong in [('stopping_travel_m', 5.647), ('measured_speed_mps', 5/3.6), ('actual_speed_stopping_envelope', True)]:
        bad = deepcopy(command); bad['details']['obstacle_guard'][key] = wrong
        with pytest.raises(ValueError, match='diagnostic stopping envelope differs'):
            replay_recorded_control([bad], saved, 0., **kwargs)
