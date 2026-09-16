"""Physical preview limits, preserved PP admission, and deterministic replay."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_speed_v1 import (
    ADAPTIVE_SPEED_POLICY, CurvatureSpeedConfig, preview_speed_limit,
)
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_15KMH_POLICY
from tools.evaluate_time_awsim_trial import replay_recorded_control


def path(curvature: float = 0., length_m: float = 12.) -> np.ndarray:
    s = np.linspace(0., length_m, 31)
    return (np.column_stack((np.sin(curvature*s)/curvature, (1-np.cos(curvature*s))/curvature))
            if curvature else np.column_stack((s, np.zeros(31))))


def limit(points: np.ndarray, speed: float = 2., tracking: float = 0.) -> dict:
    return preview_speed_limit(points, measured_speed_mps=speed,
                               cruise_ceiling_mps=15/3.6, tracking_curvature_per_m=tracking)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_constant_radius_limits_lateral_acceleration_symmetrically(sign: float) -> None:
    radius = 5.
    result = limit(path(sign/radius, 9.), speed=3., tracking=sign/radius)
    assert result['target_speed_mps'] == pytest.approx(math.sqrt(radius), rel=.025)
    assert result['target_speed_mps']**2/radius <= 1.+1e-8
    assert result['acceleration_mps2'] == -1.


def test_bend_ahead_requires_braking_while_immediate_tracking_is_straight() -> None:
    s = np.linspace(0., 14., 31)
    curve_s = np.maximum(0., s-5.)
    points = np.column_stack((np.minimum(s, 5.)+5*np.sin(curve_s/5), 5*(1-np.cos(curve_s/5))))
    result = limit(points, speed=4., tracking=0.)
    assert result['limiting_reason'] == 'preview_curvature'
    assert 2.2 < result['target_speed_mps'] < 4.
    assert result['acceleration_mps2'] < 0
    assert result['limiting_curve_support_start_m'] > 2.
    # Independent braking-distance inequality using the logged local bend limit.
    needed = (result['target_speed_mps']**2-result['limiting_curve_local_speed_mps']**2)/(2*.7)
    assert needed <= result['limiting_curve_usable_distance_m']+1e-9


def test_horizon_slows_before_measured_speed_pp_runs_out_of_path() -> None:
    result = limit(path(0., 6.), speed=2.6)
    assert result['limiting_reason'] == 'prediction_horizon'
    assert .4+.5*2.6+2.6**2/2 < 6.  # Existing PP can still select a point now.
    assert result['target_speed_mps'] < 2.6
    assert result['acceleration_mps2'] < 0
    v = result['target_speed_mps']
    assert .4+.8*v+v*v/2 <= 6.-.5+1e-9


def test_exit_reaccelerates_to_ceiling_without_floor_overriding_corner() -> None:
    corner = limit(path(.2, 9.), speed=2.5, tracking=.2)
    straight = limit(path(0., 15.), speed=2.5)
    assert corner['acceleration_mps2'] < 0
    assert straight['target_speed_mps'] == 15/3.6
    assert straight['acceleration_mps2'] == .4
    steady = limit(path(0., 15.), speed=15/3.6)
    assert steady['acceleration_mps2'] == 0.
    assert limit(path(0., 15.), speed=16/3.6)['acceleration_mps2'] < 0


def test_measurement_does_not_mutate_path_and_accounts_for_progress() -> None:
    points = path(.12, 10.); before = points.copy()
    original = limit(points)
    shifted = points-np.array([1., 0.])
    moved = limit(shifted)
    assert moved['origin_projection_arc_m'] > .9
    assert moved['target_speed_mps'] <= original['target_speed_mps']+1e-9
    np.testing.assert_array_equal(points, before)
    # Duplicate first point does not create a curvature spike.
    duplicate = np.vstack((points[0], points[:-1]))
    assert math.isfinite(limit(duplicate)['target_speed_mps'])


@pytest.mark.parametrize('points', [np.empty((0, 2)), np.zeros((32, 2)), np.zeros((30, 3)),
                                  np.array([1., 2.]), np.full((31, 2), np.nan)])
def test_invalid_shape_and_nonfinite_values_fail_closed(points: np.ndarray) -> None:
    with pytest.raises(ValueError, match='CURVATURE_SPEED_INPUT'):
        limit(points)


@pytest.mark.parametrize('speed', [-.01, 16/3.6+.01, float('nan'), float('inf')])
def test_invalid_measured_speed_fails_closed(speed: float) -> None:
    with pytest.raises(ValueError, match='CURVATURE_SPEED_INPUT'):
        limit(path(), speed)


def test_unresolved_path_and_inconsistent_acceleration_limits_fail_closed() -> None:
    for points in (np.zeros((31, 2)), path(0., .8)):
        with pytest.raises(ValueError, match='CURVATURE_SPEED_PATH_UNRESOLVED'):
            limit(points)
    with pytest.raises(ValueError, match='CURVATURE_SPEED_CONFIG'):
        replace(CurvatureSpeedConfig(), planning_deceleration_mps2=1.1)
    with pytest.raises(ValueError, match='CURVATURE_SPEED_CONFIG'):
        replace(CurvatureSpeedConfig(), response_delay_s=float('nan'))


def control(speed: float = 2., curvature: float = .1, policy: str = ADAPTIVE_SPEED_POLICY) -> tuple:
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    plan = TimePlan('preview', pose, path(curvature, 9.)[1:])
    result = time_trial_control(plan, pose, speed_mps=speed, rear_axle_offset_m=(0., 0.),
        speed_policy=policy, vehicle_model_policy=AWSIM_15KMH_POLICY,
        lookahead_policy='stopping_preview_extended_v1')
    return pose, plan, result


def test_integration_preserves_measured_speed_pp_and_raw_reference() -> None:
    _, plan, new = control()
    _, _, old = control(policy='fixed_15kmh')
    assert new['steer_rad'] == old['steer_rad']
    assert new['minimum_preview_distance_m'] == old['minimum_preview_distance_m']
    assert new['lookahead_selection'] == old['lookahead_selection']
    assert new['target_speed_mps'] < old['target_speed_mps']
    assert new['reference_xy_rear_m'] == old['reference_xy_rear_m']
    assert new['acceleration_mps2'] == new['longitudinal_preview']['acceleration_mps2']
    np.testing.assert_array_equal(plan.xy_m, path(.1, 9.)[1:])
    with pytest.raises(ValueError, match='STEERING_FEASIBLE_LOOKAHEAD_MISSING'):
        control(speed=15/3.6)  # Lower target never falsifies actual-speed admission.


def test_replay_verifies_preview_limits_and_detects_tampering() -> None:
    pose, plan, details = control()
    details.update(current_pose=pose.__dict__, observation_pose=pose.__dict__)
    command = dict(plan_id=plan.plan_id, reason='TIME_PATH_TRACKING', speed_mps=2.,
        sim_ns=0, target_speed_mps=details['target_speed_mps'],
        acceleration_mps2=details['acceleration_mps2'], details=details)
    recorded = [dict(plan_id=plan.plan_id, raw_xy_m=plan.xy_m.tolist())]
    options = dict(speed_policy=ADAPTIVE_SPEED_POLICY, vehicle_model_policy=AWSIM_15KMH_POLICY,
                   lookahead_policy='stopping_preview_extended_v1')
    assert replay_recorded_control([command], recorded, 0., **options)['matched_commands'] == 1
    bad = deepcopy(command)
    bad['details']['longitudinal_preview']['limits_mps']['preview_curvature'] += .1
    with pytest.raises(ValueError, match='recorded longitudinal preview differs'):
        replay_recorded_control([bad], recorded, 0., **options)


def test_explicit_configuration_preserves_previous_monitor_and_model() -> None:
    root = Path(__file__).parents[1]/'configs/control'
    fixed = json.loads((root/'time_path_launch_15kmh_stop1m_20260917.json').read_bytes())
    adaptive = json.loads((root/'time_path_curvature_preview_20260917.json').read_bytes())
    assert adaptive == {**fixed, 'speed_policy': ADAPTIVE_SPEED_POLICY}
    assert validate_trial_config(adaptive) == ADAPTIVE_SPEED_POLICY
    for key, bad in [('vehicle_model_policy', 'awsim_understeer_v1'), ('host', 'other'),
                     ('lookahead_policy', 'fixed_1m_v1'), ('speed_cap_mps', 20/3.6)]:
        with pytest.raises(ValueError):
            validate_trial_config({**adaptive, key: bad})
