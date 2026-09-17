"""Config units, corner preview behavior, legacy compatibility and replay."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_dev_v1 import (
    DEV_SPEED_POLICY, TimeDevSpeeds, configure_dev_speeds, configured_dev_speeds,
)
from aic_transfuser_lite.control.curvature_speed_v1 import preview_speed_limit
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from tools.evaluate_time_awsim_trial import replay_recorded_control
from tools.time_dev_runner import make_command

ROOT = Path(__file__).resolve().parents[1]


def base():
    return json.loads((ROOT/'configs/control/time_path_dev.json').read_bytes())


@pytest.mark.parametrize('maximum,corner', [(20., 10.), (15., 8.), (12., 12.), (5., 3.)])
def test_configured_speed_units_and_guard(maximum, corner):
    original = base(); before = deepcopy(original)
    config = configure_dev_speeds(original, max_speed_kmh=maximum, corner_max_speed_kmh=corner)
    assert original == before
    assert validate_trial_config(config) == DEV_SPEED_POLICY
    speeds = configured_dev_speeds(config)
    assert config['speed_cap_mps'] == speeds.max_mps == maximum/3.6
    assert speeds.corner_mps == corner/3.6
    assert config['overspeed_limit_mps'] == (maximum+1)/3.6
    assert {k for k in config if config[k] != original[k]} <= {
        'speed_parameters', 'speed_cap_mps', 'overspeed_limit_mps'}


@pytest.mark.parametrize('maximum,corner', [(0, 1), (21, 10), (10, 11), (10, 0), (10, -1),
    (float('nan'), 10), (20, float('inf')), (True, .5), ('20', 10), (20, None)])
def test_invalid_speed_parameters_are_rejected(maximum, corner):
    with pytest.raises(ValueError, match='TIME_DEV_SPEEDS'):
        TimeDevSpeeds(maximum, corner)


def test_config_mismatch_cannot_silently_fall_back_to_old_ceiling():
    config = configure_dev_speeds(base(), max_speed_kmh=12., corner_max_speed_kmh=8.)
    for key in ('speed_cap_mps', 'overspeed_limit_mps'):
        bad = deepcopy(config); bad[key] += .1
        with pytest.raises(ValueError, match='TRIAL_CONFIG_MISMATCH'):
            validate_trial_config(bad)
    for values in (None, {}, {'max_speed_kmh': 12, 'corner_max_speed_kmh': 8, 'typo': 1}):
        with pytest.raises(ValueError, match='TIME_DEV_SPEED_PARAMETER_CONTRACT'):
            validate_trial_config({**config, 'speed_parameters': values})
    old = json.loads((ROOT/'configs/control/time_path_timepreview_20kmh_20260917.json').read_bytes())
    assert configured_dev_speeds(old) is None
    assert validate_trial_config(old) == DEV_SPEED_POLICY
    with pytest.raises(ValueError, match='TRIAL_CONFIG_MISMATCH'):
        validate_trial_config({**old, 'speed_cap_mps': 12/3.6})


def path(curvature=0., length=16.):
    s = np.linspace(0., length, 31)
    return (np.column_stack((np.sin(s*curvature)/curvature, (1-np.cos(s*curvature))/curvature))
            if curvature else np.column_stack((s, np.zeros_like(s))))


def limit(points, *, corner=10/3.6, speed=4., tracking=0.):
    return preview_speed_limit(points, measured_speed_mps=speed, cruise_ceiling_mps=20/3.6,
        tracking_curvature_per_m=tracking, policy=DEV_SPEED_POLICY, corner_max_speed_mps=corner)


@pytest.mark.parametrize('sign', [-1, 1])
def test_corner_cap_and_tighter_physics_with_both_turn_directions(sign):
    original = path(sign*.1, 12.); saved = original.copy()
    result = limit(original, tracking=sign*.1)
    assert result['target_speed_mps'] == pytest.approx(10/3.6)
    assert result['acceleration_mps2'] < 0
    assert result['limiting_reason'] in ('preview_corner', 'tracking_corner')
    np.testing.assert_array_equal(original, saved)
    tighter = limit(path(sign*.2, 8.), tracking=sign*.2)
    assert tighter['target_speed_mps'] <= np.sqrt(5.)+1e-9 < 10/3.6
    straight = limit(path(), speed=2.)
    assert straight['target_speed_mps'] == 20/3.6
    assert straight['acceleration_mps2'] > 0


def test_corner_ahead_brakes_before_entering_without_capping_distant_straight():
    s = np.linspace(0., 16., 31); bend = np.maximum(0., s-5.)
    xy = np.column_stack((np.minimum(s, 5.)+10*np.sin(bend/10), 10*(1-np.cos(bend/10))))
    result = limit(xy, speed=4.5)
    assert result['limiting_reason'] == 'preview_corner'
    assert 10/3.6 < result['target_speed_mps'] < 4.5
    corner = result['corner_speed_limit']
    assert corner['support_start_m'] > 2.
    needed = (result['target_speed_mps']**2-corner['local_speed_mps']**2)/(2*.7)
    assert needed <= corner['usable_distance_m']+1e-9


def test_transition_is_continuous_and_short_horizon_can_lower_corner_ceiling():
    targets = [limit(path(k, 14.), tracking=k)['target_speed_mps'] for k in (.024999, .025, .025001)]
    assert max(targets)-min(targets) < .001
    assert limit(path(0., 2.))['target_speed_mps'] < 10/3.6


@pytest.mark.parametrize('corner', [0., -1., 6., float('nan'), float('inf'), True])
def test_bad_corner_limit_is_rejected(corner):
    with pytest.raises(ValueError, match='CORNER_SPEED_INPUT'):
        limit(path(), corner=corner)


def test_speed_parameters_reach_control_and_deterministic_replay():
    pose = TimedBodyPose(0, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    plan = TimePlan('corner', pose, path(.08, 12.)[1:])
    options = dict(speed_policy=DEV_SPEED_POLICY, vehicle_model_policy='awsim_understeer_20kmh_trial_v1',
        lookahead_policy='velocity_time_preview_v1', speed_cap_mps=15/3.6, corner_max_speed_mps=8/3.6)
    detail = time_trial_control(plan, pose, speed_mps=2., rear_axle_offset_m=(0., 0.), **options)
    assert detail['target_speed_mps'] == pytest.approx(8/3.6)
    detail.update(current_pose=pose.__dict__, observation_pose=pose.__dict__)
    row = dict(plan_id=plan.plan_id, reason='TIME_PATH_TRACKING', speed_mps=2., sim_ns=0,
        target_speed_mps=detail['target_speed_mps'], acceleration_mps2=detail['acceleration_mps2'], details=detail)
    records = [dict(plan_id=plan.plan_id, raw_xy_m=plan.xy_m.tolist())]
    assert replay_recorded_control([row], records, 0., **options)['matched_commands'] == 1
    with pytest.raises(ValueError, match='recorded control differs'):
        replay_recorded_control([row], records, 0., **{**options, 'corner_max_speed_mps': 9/3.6})
    bad = deepcopy(row); bad['details']['longitudinal_preview']['corner_speed_limit']['max_speed_mps'] += 1
    with pytest.raises(ValueError, match='recorded longitudinal preview differs'):
        replay_recorded_control([bad], records, 0., **options)


def test_make_runner_uses_ros_launch_and_finite_supervision():
    command = make_command(source=Path('/space in path/source'), deployment=Path('/space in path'),
        run_id='codex-time-dev-test', display=':0', speeds=TimeDevSpeeds(12., 8.), record_video=True)
    assert command[:4] == ['timeout', '--signal=TERM', '--kill-after=10s', '710s']
    assert command[5] == '/space in path/source/tools/run_time_path_awsim_trial.py'
    assert command[-6:] == ['--ros-launch', '--max-speed-kmh', '12.0', '--corner-max-speed-kmh', '8.0', '--record-video']
    with pytest.raises(ValueError, match='INVALID_OWNED_RUN_ID'):
        make_command(source=ROOT, deployment=ROOT, run_id='../old-run', display=':0',
                     speeds=TimeDevSpeeds(), record_video=False)


def test_make_runner_forwards_two_npcs_without_extra_ego_controllers():
    command = make_command(source=ROOT, deployment=ROOT, run_id='codex-time-npc-test',
        display=':0', speeds=TimeDevSpeeds(), record_video=True, npcs=2)
    assert command[command.index('--npcs') + 1] == '2'
    assert '--recovery-side' not in command
    with pytest.raises(ValueError, match='NPC_COUNT'):
        make_command(source=ROOT, deployment=ROOT, run_id='codex-time-npc-test',
            display=':0', speeds=TimeDevSpeeds(), record_video=False, npcs=4)
