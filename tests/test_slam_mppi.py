"""Deterministic geometry/switching tests; these are not AWSIM driving proof."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from aic_transfuser_lite.control.slam_mppi import (
    POLICY, SPEED_MPS, AvoidancePlanner, ReferenceMppi, avoidance_command, transform,
)
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.control.time_dev_v1 import TimeDevSpeeds
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length
from tools.time_dev_runner import make_command


def scene(block=True):
    grid = np.zeros((140, 180), dtype=np.int8)
    # 36 x 28 m synthetic observed free area, origin (-8,-14) m.
    origin = np.array([-8., -14.])
    if block:
        grid[68:72, 68:72] = 100  # Small box near x=6 m, y=0 m.
    reference = np.column_stack((np.linspace(0., 12., 31), np.zeros(31)))
    return reference, grid, origin


def packet(blocked=True, stamp=1_000_000_000):
    return dict(event='OBSTACLES', run_id='test', frame='time_slam_map', gnss_imu_map_inputs=False,
        stamp_ns=stamp, plan_observation_ns=stamp, input_valid=True, path_valid=True,
        path_blocked=blocked, nearest_path_obstacle_m=6. if blocked else None,
        base_pose_xyyaw=[0., 0., 0.])


def command(observation, **overrides):
    args = dict(run_id='test', source_valid=True, now_sim_ns=1_000_000_000,
        now_wall_ns=2_000_000_000, receipt_ns=2_000_000_000, speed_mps=1., target_mps=SPEED_MPS,
        acceleration_mps2=.5, dt_s=.05, scan_wheel_pose=np.zeros(3), current_wheel_pose=np.zeros(3),
        vehicle_model_policy='awsim_understeer_v1')
    args.update(overrides)
    return avoidance_command(observation, **args)


def planned(block=True):
    ref, grid, origin = scene(block)
    p = packet(block)
    p['mppi'] = AvoidancePlanner().update(p, ref,
        SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2))
    return p


def test_box_bypass_rejoins_reference_and_keeps_side():
    ref, grid, origin = scene()
    solver = ReferenceMppi()
    path = np.asarray(solver.solve(ref, np.zeros(3), grid, origin, .2)['path_world_xy_m'])
    assert np.max(np.abs(path[:, 1])) > 1.5
    np.testing.assert_allclose(path[[0, -1]], ref[[0, -1]], atol=1e-8)
    # Independently check the path centers stay away from the box.
    assert np.min(np.linalg.norm(path-np.array([6., 0.]), axis=1)) > 1.5
    side = solver.side
    solver.solve(ref, np.zeros(3), grid, origin, .2)
    assert solver.side == side != 0


@pytest.mark.parametrize('value', [-1, 100])
def test_unknown_and_blocked_corridors_cannot_generate_a_path(value):
    ref, grid, origin = scene(False)
    grid[:, 65:75] = value
    with pytest.raises(ValueError, match='NO_FEASIBLE_PATH'):
        ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .2)


def test_coordinate_rotation_and_translation_preserve_avoidance():
    ref, grid, origin = scene()
    # Translate both occupancy origin and model path; heading stays unchanged.
    pose = np.array([37., -19., 0.])
    a = ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .2)
    b = ReferenceMppi().solve(transform(ref, pose), pose, grid, origin+pose[:2], .2)
    np.testing.assert_allclose(np.asarray(b['path_world_xy_m'])-pose[:2], a['path_world_xy_m'], atol=1e-8)
    p = planned(); first = command(p)
    wheel = np.array([10., 20., 1.2])
    shifted = command(p, scan_wheel_pose=wheel, current_wheel_pose=wheel)
    assert shifted['steer_rad'] == pytest.approx(first['steer_rad'])
    assert shifted['mode'] == 'AVOID'


def test_clear_road_preserves_nominal_and_blockage_switches_to_mppi():
    clear = command(planned(False))
    assert clear['mode'] == 'NOMINAL' and clear['steer_rad'] is None
    assert clear['acceleration_mps2'] == .5
    avoided = command(planned())
    assert avoided['mode'] == 'AVOID' and abs(avoided['steer_rad']) <= .3
    assert 0 < avoided['target_speed_mps'] <= SPEED_MPS


@pytest.mark.parametrize('override', [dict(source_valid=False), dict(receipt_ns=None),
    dict(now_sim_ns=1_400_000_000), dict(now_wall_ns=2_400_000_000),
    dict(scan_wheel_pose=None), dict(target_mps=0.), dict(speed_mps=2.)])
def test_bad_inputs_or_stop_request_brake(override):
    result = command(planned(), **override)
    assert result['mode'] == 'STOP' and result['target_speed_mps'] == 0.
    assert result['acceleration_mps2'] == -1.
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('field,value', [('stamp_ns', 2), ('frame', 'map'), ('mode', 'unknown'),
    ('path_world_xy_m', [[float('nan'), 0.]]), ('weighted_path_rechecked', False)])
def test_malformed_plan_is_rejected(field, value):
    p = planned(); p['mppi'][field] = value
    assert command(p)['mode'] == 'STOP'


@pytest.mark.parametrize('kind', ['shape', 'nan', 'resolution', 'short'])
def test_invalid_geometry_is_explicitly_rejected(kind):
    ref, grid, origin = scene(False)
    if kind == 'shape':
        ref = np.zeros((30, 3))
    elif kind == 'nan':
        ref[5, 0] = np.nan
    elif kind == 'short':
        ref = ref[:3]
    with pytest.raises(ValueError, match='MPPI_'):
        ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .1 if kind == 'resolution' else .2)


def test_legacy_replay_does_not_certify_modified_mppi_commands():
    from tools.evaluate_time_awsim_trial import replay_recorded_control
    replay = replay_recorded_control([{'details': {'slam_mppi': {'mode': 'AVOID'}}}], [], 0.)
    assert replay['status'] == 'UNSUPPORTED_SLAM_MPPI' and replay['matched_commands'] == 0


def test_release_needs_multiple_fresh_observations_and_clock_reset_rejected():
    ref, grid, origin = scene()
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    assert planner.update(packet(), ref, g)['mode'] == 'AVOID'
    g.values.fill(0)
    for i in range(1, 6):
        assert planner.update(packet(False, 1_000_000_000+i*100_000_000), ref, g)['mode'] == 'AVOID'
    assert planner.update(packet(False, 1_600_000_000), ref, g)['mode'] == 'NOMINAL'
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        planner.update(packet(False), ref, g)


def test_config_and_launcher_require_real_stop_guard_and_initial_speed():
    config = json.loads(Path('configs/control/time_path_slam_mppi.json').read_text())
    validate_trial_config(config)
    for key, value in [('scan_occupancy_policy', 'log_only_awsim_v1'),
                       ('stopping_distance_policy', 'awsim_cap_1m_diagnostic_v1'),
                       ('slam_mppi_policy', 'typo')]:
        bad = deepcopy(config); bad[key] = value
        with pytest.raises(ValueError, match='SLAM_MPPI'):
            validate_trial_config(bad)
    kwargs = dict(source=Path('/src'), deployment=Path('/runtime'), run_id='codex-time-mppi',
                  display=':0', speeds=TimeDevSpeeds(5., 5.), record_video=False, slam_mppi=True)
    args = make_command(**kwargs)
    assert '--slam-obstacles' in args and 'configs/control/time_path_slam_mppi.json' in args
    with pytest.raises(ValueError, match='STATIC_SINGLE_EGO'):
        make_command(**dict(kwargs, pp_vehicles=1))


def test_replanning_follows_around_box_and_rejoins_in_ideal_plant():
    _, grid, origin = scene()
    ref = np.column_stack((np.linspace(0., 25., 100), np.zeros(100)))
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner(); pose = np.zeros(3)
    largest_offset = 0.
    for i in range(240):
        stamp = 1_000_000_000+i*100_000_000
        p = packet(bool(pose[0] < 4.), stamp)
        p['base_pose_xyyaw'] = pose.tolist()
        p['mppi'] = planner.update(p, ref, g)
        out = command(p, now_sim_ns=stamp, now_wall_ns=stamp, receipt_ns=stamp,
                      scan_wheel_pose=pose, current_wheel_pose=pose, target_mps=1., acceleration_mps2=0.)
        assert out['mode'] != 'STOP', (i, pose, out['reason'])
        if out['mode'] == 'NOMINAL':
            assert pose[0] > 8. and abs(pose[1]) < .25 and largest_offset > 1.8
            break
        k = np.tan(out['steer_rad'])/effective_response_length(1., 'awsim_understeer_v1')
        pose[:2] += .1*np.array([np.cos(pose[2]+.05*k), np.sin(pose[2]+.05*k)])
        pose[2] += .1*k
        largest_offset = max(largest_offset, abs(pose[1]))
        # Check actual body rectangle against the box, independently of MPPI circles.
        corners = np.array([[5.6,-.4],[5.6,.4],[6.4,-.4],[6.4,.4]])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        body = (corners-pose[:2])@np.array([[c,-s],[s,c]])
        assert not np.any((body[:,0] >= -.6) & (body[:,0] <= 1.8) & (np.abs(body[:,1]) <= .7))
    else:
        pytest.fail('did not return to the reference within 24 simulated seconds')
