"""10 km/h is an explicit simulated trial; old speed domains stay closed."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from aic_transfuser_lite.control.awsim_steering import CALIBRATED_POLICY, command_steering
from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, trial_speed_limits, validate_trial_config
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.vehicle_motion_v1 import (
    AWSIM_POLICY, AWSIM_10KMH_POLICY, IDEAL_POLICY, MAX_CURVATURE_PER_M,
    COM_FORWARD_OF_REAR_M, effective_response_length, stopping_motion,
)
from tools.evaluate_time_awsim_trial import replay_recorded_control
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin


def test_ros_smoke_cli_resolves_pinned_source_without_pythonpath(tmp_path: Path) -> None:
    script = Path(__file__).parents[1]/'tools/check_time_ros_connection.py'
    environment = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
    result = subprocess.run([sys.executable, str(script), '--help'], cwd=tmp_path,
        env=environment, text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert 'fixed_10kmh' in result.stdout


def fixture(curvature: float = 0., source_speed: float = 3.) -> tuple[TimedBodyPose, TimePlan]:
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    distances = np.arange(1, 31)*.1*source_speed
    xy = (np.column_stack((np.sin(curvature*distances)/curvature, (1-np.cos(curvature*distances))/curvature))
          if curvature else np.column_stack((distances, np.zeros(30))))
    assert xy.shape == (30, 2)
    return pose, TimePlan('ten', pose, xy)


def control(speed: float, curvature: float = 0., source_speed: float = 3.) -> dict:
    pose, plan = fixture(curvature, source_speed)
    return time_trial_control(plan, pose, speed_mps=speed, rear_axle_offset_m=(0., 0.),
        speed_policy='fixed_10kmh', vehicle_model_policy=AWSIM_10KMH_POLICY,
        lookahead_policy='stopping_preview_extended_v1')


def test_new_trial_profile_is_explicit_and_old_domains_are_unchanged() -> None:
    assert trial_speed_limits('fixed_10kmh') == pytest.approx((10/3.6, 11/3.6))
    assert trial_speed_limits('fixed_5kmh') == pytest.approx((5/3.6, 6/3.6))
    for old in (IDEAL_POLICY, AWSIM_POLICY):
        with pytest.raises(ValueError, match='VEHICLE_MODEL_SPEED'):
            effective_response_length(2., old)
    assert effective_response_length(10/3.6, AWSIM_10KMH_POLICY) == pytest.approx(1.087+.045*(10/3.6)**2)
    for invalid in (-.01, 11/3.6+1e-6, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='VEHICLE_MODEL_SPEED'):
            effective_response_length(invalid, AWSIM_10KMH_POLICY)


def test_configuration_preserves_checkpoint_and_rejects_mixed_profiles() -> None:
    root = Path(__file__).parents[1]/'configs/control'
    old = json.loads((root/'time_path_launch_probe_20260917.json').read_text())
    new = json.loads((root/'time_path_launch_10kmh_20260917.json').read_text())
    assert new == {**old, 'speed_policy': 'fixed_10kmh', 'speed_cap_mps': 10/3.6,
                  'overspeed_limit_mps': 11/3.6, 'vehicle_model_policy': AWSIM_10KMH_POLICY}
    assert validate_trial_config(new) == 'fixed_10kmh'
    assert validate_trial_config(old) == 'fixed_5kmh'
    for key, invalid in [('host', 'other'), ('record_vehicle_motion', False), ('execution_profile', 'bounded_10s'),
                         ('vehicle_model_policy', AWSIM_POLICY), ('vehicle_model_policy', IDEAL_POLICY),
                         ('speed_cap_mps', 5/3.6), ('overspeed_limit_mps', 12/3.6),
                         ('obstacle_policy', 'straight_v1'), ('speed_policy', 'fixed_5kmh')]:
        with pytest.raises(ValueError):
            validate_trial_config({**new, key: invalid})


@pytest.mark.parametrize('source_speed', [.8, 3.])
def test_fixed_target_is_not_teacher_speed_and_raw_points_stay_unchanged(source_speed: float) -> None:
    pose, plan = fixture(source_speed=source_speed)
    before = plan.xy_m.copy()
    result = time_trial_control(plan, pose, speed_mps=0., rear_axle_offset_m=(0., 0.),
        speed_policy='fixed_10kmh', vehicle_model_policy=AWSIM_10KMH_POLICY)
    assert result['target_speed_mps'] == 10/3.6
    assert result['predicted_source_speed_mps'] == pytest.approx(source_speed)
    assert result['acceleration_mps2'] == 1.
    np.testing.assert_array_equal(before, plan.xy_m)


def test_ten_kmh_brakes_on_overshoot_and_rejects_short_stationary_and_stale_predictions() -> None:
    assert control(10.5/3.6)['acceleration_mps2'] < 0
    with pytest.raises(ValueError, match='TRIAL_SPEED_CONTRACT'):
        control(11/3.6+1e-6)
    with pytest.raises(ValueError, match='MOTION_UNRESOLVED'):
        control(0., source_speed=0.)
    with pytest.raises(ValueError, match='STEERING_FEASIBLE_LOOKAHEAD_MISSING'):
        control(10/3.6, source_speed=1.5)
    pose, plan = fixture()
    with pytest.raises(ValueError, match='STALE'):
        time_trial_control(plan, TimedBodyPose(600_000_000, 'sim', '0', 'map', 'base_link', 0., 0., 0.),
            speed_mps=10/3.6, rear_axle_offset_m=(0., 0.), speed_policy='fixed_10kmh', vehicle_model_policy=AWSIM_10KMH_POLICY)
    with pytest.raises(ValueError, match='TRIAL_SPEED_MODEL_CONTRACT'):
        time_trial_control(plan, pose, speed_mps=0., rear_axle_offset_m=(0., 0.), speed_policy='fixed_10kmh')


@pytest.mark.parametrize('curvature', [-.08, 0., .08])
def test_full_speed_preview_units_and_steering_limit(curvature: float) -> None:
    speed = 10/3.6
    result = control(speed, curvature)
    stopping = .4+speed*.5+speed*speed/2
    assert result['minimum_preview_distance_m'] == pytest.approx(5.646913580246914)
    assert stopping <= result['selected_lookahead_distance_m'] <= stopping+1.
    assert abs(result['steer_rad']) <= .3
    assert result['target_speed_mps'] == speed


def test_scan_stopping_reserve_grows_with_speed_without_relaxing_clearance() -> None:
    ranges = np.full(750, np.inf)
    ranges[375] = 5.4-1.649  # Obstacle forward of the old 5 km/h stopping body.
    scan = (ranges, -np.pi, 2*np.pi/750, .1, 25.)
    kwargs = dict(measured_steer_rad=0., issued_steer_rad=0., previous_steer_rad=0.,
        scan_in_current_rear=(1.649, 0., 0.), heading_rate_radps=0., reported_lateral_mps=0.,
        envelope_policy='curvature_support_v2')
    assert check_turning_scan(*scan, speed_mps=5/3.6, vehicle_model_policy=AWSIM_POLICY, **kwargs)['minimum_ray_margin_m'] > 0
    with pytest.raises(ValueError, match='STOPPING_SWEEP_OCCUPIED'):
        check_turning_scan(*scan, speed_mps=10/3.6, vehicle_model_policy=AWSIM_10KMH_POLICY, **kwargs)
    proof = scan_margin(dict(ranges=ranges.tolist(), angle_min=-np.pi, angle_increment=2*np.pi/750,
        range_min=.1, range_max=25.), (1.649, 0., 0.), speed_mps=10/3.6, measured_rad=0., issued_rad=0.,
        previous_rad=0., yaw_rate_radps=0., lateral_mps=0., vehicle_model_policy=AWSIM_10KMH_POLICY)
    assert proof['reason'] == 'STOPPING_SWEEP_OCCUPIED' and proof['minimum_ray_margin_m'] < 0


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('speed', [10/3.6, 11/3.6])
def test_independent_high_speed_braking_body_is_inside_support(sign: float, speed: float) -> None:
    motion = stopping_motion(speed, sign*.12, sign*.2, sign*.08, policy=AWSIM_10KMH_POLICY,
        heading_rate_radps=sign*.1, reported_lateral_mps=COM_FORWARD_OF_REAR_M*sign*.1)
    assert motion['calibrated_at_10kmh'] is False
    travel = .4+speed*.5+speed*speed/2
    normals, support, metadata = curvature_support_envelope(*motion['curvature_interval_per_m'],
        travel, .004, np.array([1.649, 0.]), lateral_padding_m=motion['lateral_displacement_bound_m'],
        vehicle_model_policy=AWSIM_10KMH_POLICY)
    assert normals.shape == (64, 2) and support.shape == (64,)
    assert metadata['stopping_travel_m'] == travel
    corners = np.array([[-.510, -.85], [1.984, -.85], [1.984, .85], [-.510, .85]])
    position = np.zeros(2); heading = 0.; dt = .002
    for t in np.arange(0., .5+speed, dt):
        v = max(0., speed-max(0., t-.5))
        tire = sign*(.08+.12*(.5+.5*math.sin(13*t)))
        gradient = .1+.1*math.sin(7*t)
        rate = v*math.tan(tire)/(1.087+gradient*v*v)
        lateral = .03*math.cos(11*t)
        mid = heading+rate*dt/2
        position += dt*np.array([v*math.cos(mid)-lateral*math.sin(mid), v*math.sin(mid)+lateral*math.cos(mid)])
        heading += rate*dt
        c, s = math.cos(heading), math.sin(heading)
        assert np.max((corners@np.array([[c, s], [-s, c]])+position)@normals.T-support) <= 1e-8


def test_heading_and_lateral_bounds_still_reject_outside_proof_domain() -> None:
    speed = 11/3.6
    travel = .4+speed*.5+speed*speed/2
    with pytest.raises(ValueError, match='SUPPORT_ENVELOPE_HEADING_DOMAIN'):
        curvature_support_envelope(-MAX_CURVATURE_PER_M, MAX_CURVATURE_PER_M, travel, .004,
            np.array([1.649, 0.]), vehicle_model_policy=AWSIM_10KMH_POLICY)
    with pytest.raises(ValueError, match='MOTION_REAR_LATERAL_INVALID'):
        stopping_motion(speed, 0., 0., policy=AWSIM_10KMH_POLICY, heading_rate_radps=0., reported_lateral_mps=.030001)


def test_high_speed_record_replay_checks_motion_model_and_detects_tampering() -> None:
    speed = 10/3.6; curvature = .05
    pose, plan = fixture(curvature)
    calculated = control(speed, curvature)
    tire = calculated['steer_rad']
    mapping = command_steering(tire, tire/.6, .05, policy=CALIBRATED_POLICY)
    yaw = speed*curvature
    motion = stopping_motion(speed, tire, mapping['issued_tire_target_rad'], mapping['previous_tire_target_rad'],
        policy=AWSIM_10KMH_POLICY, heading_rate_radps=yaw, reported_lateral_mps=COM_FORWARD_OF_REAR_M*yaw)
    details = {**calculated, 'current_pose': pose.__dict__, 'observation_pose': pose.__dict__,
               'steering_actuator': mapping, 'obstacle_guard': {'vehicle_motion': motion}}
    command = dict(plan_id=plan.plan_id, reason='TIME_PATH_TRACKING', speed_mps=speed,
        steer_rad=mapping['issued_input_rad'], measured_steer_rad=tire, details=details,
        motion_observation=dict(frame='base_link', heading_rate_radps=yaw, reported_lateral_mps=COM_FORWARD_OF_REAR_M*yaw))
    options = dict(speed_policy='fixed_10kmh', obstacle_policy='steering_support_v2', steering_policy=CALIBRATED_POLICY,
        lookahead_policy='stopping_preview_extended_v1', vehicle_model_policy=AWSIM_10KMH_POLICY)
    saved = [dict(plan_id=plan.plan_id, raw_xy_m=plan.xy_m.tolist())]
    assert replay_recorded_control([command], saved, 0., **options)['vehicle_motion_matched'] == 1
    bad = deepcopy(command)
    bad['details']['obstacle_guard']['vehicle_motion']['calibrated_at_10kmh'] = True
    with pytest.raises(ValueError, match='stopping motion differs'):
        replay_recorded_control([bad], saved, 0., **options)
