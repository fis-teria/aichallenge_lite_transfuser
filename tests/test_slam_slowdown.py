import json
import math
from pathlib import Path

import pytest

from aic_transfuser_lite.control.slam_slowdown import SlamSlowdown
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config


def packet(distance=None, stamp=1_000_000_000):
    return dict(event='OBSTACLES', run_id='test', frame='time_slam_map',
                gnss_imu_map_inputs=False, stamp_ns=stamp, plan_observation_ns=stamp,
                input_valid=True, path_valid=True, path_blocked=distance is not None,
                nearest_path_obstacle_m=distance)


def update(guard, observation, **kwargs):
    args = dict(run_id='test', source_valid=True, now_sim_ns=1_000_000_000,
                now_wall_ns=2_000_000_000, receipt_ns=2_000_000_000,
                speed_mps=3., target_mps=5., acceleration_mps2=.8, dt_s=.05)
    args.update(kwargs)
    return guard.update(observation, **args)


def test_clear_and_distant_preserve_command_but_near_obstacle_brakes():
    guard = SlamSlowdown()
    for distance in (None, 30.):
        result = update(guard, packet(distance))
        assert result['target_speed_mps'] == 5. and result['acceleration_mps2'] == .8
        assert not result['limited'] and not result['steering_modified']
    result = update(guard, packet(5.))
    assert 0 < result['target_speed_mps'] < 3.
    assert result['acceleration_mps2'] < 0
    result = update(guard, packet(2.8))
    assert result['target_speed_mps'] == 0 and result['acceleration_mps2'] == -1.


@pytest.mark.parametrize('change', [
    {'run_id':'other'}, {'frame':'map'}, {'gnss_imu_map_inputs':True}, {'input_valid':False},
    {'path_valid':False}, {'stamp_ns':float('nan')}, {'stamp_ns':1_000_000_001},
    {'plan_observation_ns':0}, {'path_blocked':None}, {'nearest_path_obstacle_m':float('inf')},
    {'path_blocked':False, 'nearest_path_obstacle_m':2.},
])
def test_invalid_observations_request_braking_without_nan_logs(change):
    observation = packet(5.); observation.update(change)
    result = update(SlamSlowdown(), observation)
    assert not result['valid'] and result['target_speed_mps'] == 0
    assert result['acceleration_mps2'] == -1.
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('kwargs', [
    {'source_valid':False}, {'receipt_ns':None}, {'receipt_ns':1_000_000_000},
    {'now_sim_ns':1_350_000_001}, {'receipt_ns':2_000_000_001},
])
def test_missing_stale_or_wrong_publisher_never_means_clear(kwargs):
    result = update(SlamSlowdown(), packet(), **kwargs)
    assert not result['valid'] and result['target_speed_mps'] == 0
    assert update(SlamSlowdown(), None)['target_speed_mps'] == 0


def test_release_is_bounded_and_cannot_raise_e2e_speed_or_acceleration():
    guard = SlamSlowdown(); update(guard, packet(2.))
    previous = 0.
    for _ in range(50):
        result = update(guard, packet(), target_mps=1., acceleration_mps2=-.4)
        assert previous <= result['target_speed_mps'] <= min(1., previous+.025+1e-12)
        assert result['acceleration_mps2'] <= -.4
        previous = result['target_speed_mps']
    assert previous == 1.
    assert update(guard, packet(), target_mps=.2)['target_speed_mps'] == .2


def test_latency_reduces_speed_limit_and_missing_command_contract_is_rejected():
    fresh = update(SlamSlowdown(), packet(8.))
    old = update(SlamSlowdown(), packet(8.), now_sim_ns=1_200_000_000)
    assert old['target_speed_mps'] < fresh['target_speed_mps']
    with pytest.raises(ValueError, match='COMMAND_CONTRACT'):
        update(SlamSlowdown(), packet(), speed_mps=float('nan'))


def test_stationary_object_closed_loop_stops_with_gap_and_resumes_after_removal():
    guard = SlamSlowdown(); speed = 3.; position = 0.; dt = .05
    # Ideal longitudinal plant, not a substitute for AWSIM braking measurement.
    for i in range(300):
        t = 1_000_000_000+i*50_000_000
        result = update(guard, packet(15.-position, t), now_sim_ns=t,
                        speed_mps=speed, target_mps=3., acceleration_mps2=min(1., 2*(3.-speed)))
        speed = max(0., speed+result['acceleration_mps2']*dt)
        position += speed*dt
        assert position+1.8 < 15.
    assert speed < .03 and 15.-position-1.8 >= .95
    for i in range(150):
        result = update(guard, packet(), speed_mps=speed, target_mps=3., acceleration_mps2=1.)
        speed = max(0., speed+result['acceleration_mps2']*dt)
    assert speed > 2.


def test_slowdown_config_is_explicit_and_typo_is_rejected():
    config = json.loads(Path('configs/control/time_path_slam_slowdown.json').read_text())
    validate_trial_config(config)
    config['slam_slowdown_policy'] = 'typo'
    with pytest.raises(ValueError, match='SLAM_SLOWDOWN_POLICY'):
        validate_trial_config(config)
