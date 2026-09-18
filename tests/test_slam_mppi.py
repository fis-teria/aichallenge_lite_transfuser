"""Deterministic geometry/switching tests; these are not AWSIM driving proof."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from aic_transfuser_lite.control.slam_mppi import (
    POLICY, SPEED_MPS, AvoidancePlanner, ReferenceMppi, avoidance_command, transform, mppi_nominal_control,
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


def test_box_bypass_allows_partial_terminal_and_keeps_side():
    ref, grid, origin = scene()
    solver = ReferenceMppi()
    path = np.asarray(solver.solve(ref, np.zeros(3), grid, origin, .2)['path_world_xy_m'])
    assert np.max(np.abs(path[:, 1])) > 1.5
    np.testing.assert_allclose(path[0], ref[0], atol=1e-8)
    assert path[-1, 0] == pytest.approx(ref[-1, 0])
    assert abs(path[-1, 1]) <= 2.5
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


def test_ros_entrypoints_compile_without_ros_installed():
    import ast
    for name in ('slam_obstacle_node.py', 'time_trial_controller_node.py'):
        path = Path('ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime')/name
        ast.parse(path.read_text(), filename=str(path))


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


def test_short_reference_can_end_beside_box_without_early_rejoin():
    ref, grid, origin = scene(False)
    ref[:, 0] = np.linspace(0., 3.806, len(ref))
    # Approximate first box01 failure: box ahead/right and short 3 s TimePath.
    grid[65:68, 71:74] = 100
    out = ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .2)
    assert out['terminal_offset_m'] > .2
    assert out['reference_horizon_m'] == pytest.approx(3.806)
    assert sum(d['feasible'] for d in out['candidate_diagnostics']) > 0


def test_partial_terminal_can_still_be_turning_in_narrow_asymmetric_corridor():
    ref, grid, origin = scene(False)
    ref[:, 0] = np.linspace(0., 3.806, len(ref))
    grid[79:, :] = 100  # Left wall at y=1.8 m.
    grid[:45, :] = 100  # Right wall at y=-5 m.
    grid[65:68, 71:74] = 100
    out = ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .2)
    path = np.asarray(out['path_world_xy_m'])
    final_delta = path[-1]-path[-2]
    assert path[-1, 1] < -.5
    assert np.arctan2(final_delta[1], final_delta[0]) < -.1


@pytest.mark.parametrize('scale', [.01, 0.])
def test_stopped_short_prediction_retains_spatial_reference_but_rechecks_grid(scale):
    ref, grid, origin = scene()
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    assert planner.update(packet(), ref, g)['mode'] == 'AVOID'
    short = ref*scale
    out = planner.update(packet(True, 1_100_000_000), short, g)
    assert out['mode'] == 'AVOID' and out['reference_retained']
    assert out['reference_horizon_m'] == pytest.approx(12.)
    g.values[:, 65:75] = 100
    assert planner.update(packet(True, 1_200_000_000), short, g)['reason'] == 'MPPI_NO_FEASIBLE_PATH'
    assert planner.update(packet(True, 1_300_000_000), None, g)['reason'] == 'MPPI_NO_REFERENCE'
    assert planner.update(packet(True, 11_100_000_000), short, g)['reason'] == 'MPPI_RETAINED_REFERENCE_EXPIRED'


def test_only_overlapping_fresh_prediction_can_extend_retained_reference():
    ref, grid, origin = scene()
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    planner.update(packet(), ref, g)
    planner.update(packet(True, 1_100_000_000), ref+[1., 0.], g)
    np.testing.assert_allclose(planner.reference_world[-1], [13., 0.])
    planner.update(packet(True, 1_200_000_000), ref+[10., 8.], g)
    np.testing.assert_allclose(planner.reference_world[-1], [13., 0.])
    with pytest.raises(ValueError, match='FINITE'):
        planner._reference(np.full((30, 2), np.nan), 1_300_000_000)


def test_reference_refresh_does_not_create_a_splice_curvature_spike():
    ref, grid, origin = scene(False)
    ref[:, 0] = np.linspace(0., 3.8, len(ref))
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    assert planner.update(packet(), ref, g)['mode'] == 'AVOID'
    updated = packet(True, 1_100_000_000)
    updated['base_pose_xyyaw'] = [.2, 0., 0.]
    out = planner.update(updated, ref+[.2, .015], g)
    assert out['mode'] == 'AVOID'
    path = np.asarray(out['path_world_xy_m'])
    segment = np.diff(path, axis=0)
    angle = np.unwrap(np.arctan2(segment[:, 1], segment[:, 0]))
    ds = np.linalg.norm(segment, axis=1)
    assert np.max(np.abs(np.diff(angle)/(.5*(ds[:-1]+ds[1:])))) <= np.tan(.3)/1.2


def test_fresh_prediction_from_turned_ego_refreshes_without_crossing_old_endpoint():
    ref, grid, origin = scene(False)
    ref[:, 0] = np.linspace(0., 3.8, len(ref))
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    planner.update(packet(), ref, g)
    turned = np.array([1.5, -.3, -.5])
    fresh = transform(ref, turned)
    assert np.linalg.norm(fresh[-1]-ref[-1]) > .5
    p = packet(True, 1_100_000_000); p['base_pose_xyyaw'] = turned.tolist()
    assert planner.update(p, fresh, g)['mode'] == 'AVOID'
    np.testing.assert_allclose(planner.reference_world, fresh)


def test_avoidance_does_not_release_just_because_turned_prediction_misses_box():
    ref, grid, origin = scene(False)
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner()
    first = packet(); first['surfaces'] = [dict(path_overlap=True, points_xy_m=[[6.,0.], [6.4,0.]])]
    assert planner.update(first, ref, g)['mode'] == 'AVOID'
    for i in range(1, 8):
        assert planner.update(packet(False, 1_000_000_000+i*100_000_000), ref, g)['mode'] == 'AVOID'
    passed = packet(False, 1_800_000_000); passed['base_pose_xyyaw'] = [7.5, 0., 0.]
    assert planner.update(passed, ref, g)['mode'] == 'NOMINAL'


def test_prediction_start_ahead_joins_actual_pose_and_short_remaining_path_is_usable():
    ref, grid, origin = scene(False)
    ref[:, 0] = np.linspace(.025, 2.5, len(ref))
    out = ReferenceMppi().solve(ref, np.zeros(3), grid, origin, .2)
    np.testing.assert_allclose(out['path_world_xy_m'][0], [0., 0.], atol=1e-9)
    assert out['path_world_xy_m'][-1][0] == pytest.approx(2.5)
    p = packet()
    p['mppi'] = dict(out, policy=POLICY, mode='AVOID', reason='MPPI_FEASIBLE',
                     stamp_ns=p['stamp_ns'], frame='time_slam_map')
    assert command(p, speed_mps=0.)['mode'] == 'AVOID'
    # Sensor-time path consumption still enforces actual stopping distance.
    assert command(p, current_wheel_pose=np.array([1.8,0.,0.]), speed_mps=1.3)['mode'] == 'STOP'


def test_short_nominal_fallback_requires_fresh_feasible_mppi_and_keeps_admission():
    from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
    pose = TimedBodyPose(1_000_000_000, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    plan = TimePlan('test', pose, np.column_stack((np.linspace(.01, .2, 30), np.zeros(30))))
    kwargs = dict(speed_mps=0., rear_axle_offset_m=(.001, 0.), speed_policy='fixed_5kmh',
                  lookahead_policy='stopping_preview_segment_v1', vehicle_model_policy='awsim_understeer_v1')
    nominal = mppi_nominal_control(plan, pose, **kwargs)
    reason = nominal['nominal_tracking_unavailable']
    assert command(planned(False), nominal_tracking_unavailable=reason)['mode'] == 'STOP'
    assert command(planned(), nominal_tracking_unavailable=reason)['mode'] == 'AVOID'
    assert command(planned(), nominal_tracking_unavailable=reason, source_valid=False)['mode'] == 'STOP'
    from dataclasses import replace
    with pytest.raises(ValueError, match='STALE'):
        mppi_nominal_control(plan, replace(pose, stamp_ns=1_600_000_000), **kwargs)
    with pytest.raises(ValueError, match='SPEED'):
        mppi_nominal_control(plan, pose, **dict(kwargs, speed_mps=2.))
    bad = plan.xy_m.copy(); bad[15] += [4., 0.]
    with pytest.raises(ValueError, match='DISCONTINUITY'):
        mppi_nominal_control(TimePlan('bad', pose, bad), pose, **kwargs)


def test_first_feasible_path_is_used_without_a_second_blockage_confirmation():
    p = planned()
    p.update(path_blocked=False, nearest_path_obstacle_m=None)
    assert command(p)['mode'] == 'AVOID'


def test_recorded_failure_has_a_physical_rollout_and_obstacle_updates_still_stop():
    fixture = Path(__file__).parent/'fixtures/slam_mppi/partial05_first_failure.npz'
    with np.load(fixture) as d:
        solver = ReferenceMppi()
        out = solver.solve(d['reference_world'], d['base_pose'], d['values'], d['origin_xy_m'], float(d['resolution_m']))
        assert out['trajectory_family'] == 'curvature_rollout'
        path = np.asarray(out['path_world_xy_m'])
        np.testing.assert_allclose(path[0], d['base_pose'][:2], atol=1e-9)
        segments = np.diff(path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        yaw = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        assert np.max(np.abs(np.diff(yaw)/(.5*(lengths[:-1]+lengths[1:])))) <= np.tan(.3)/1.2
        blocked = np.full_like(d['values'], -1)
        with pytest.raises(ValueError, match='NO_FEASIBLE_PATH'):
            solver.solve(d['reference_world'], d['base_pose'], blocked, d['origin_xy_m'], float(d['resolution_m']))


def test_short_rolling_timepath_continues_avoidance_and_rejoins_in_ideal_plant():
    _, grid, origin = scene(False)
    grid[79:, :] = 100; grid[:45, :] = 100; grid[65:68, 71:74] = 100
    g = SimpleNamespace(values=grid, origin=origin/.2, resolution_m=.2)
    planner = AvoidancePlanner(); pose = np.zeros(3); used_rollout = False
    for i in range(220):
        # A moving 3.8 m prediction, rather than the old 25 m ideal fixture.
        ref = np.column_stack((np.linspace(pose[0], pose[0]+3.8, 30), np.zeros(30)))
        stamp = 1_000_000_000+i*100_000_000
        p = packet(bool(pose[0] < 8.), stamp); p['base_pose_xyyaw'] = pose.tolist()
        p['mppi'] = planner.update(p, ref, g)
        used_rollout |= p['mppi'].get('trajectory_family') == 'curvature_rollout'
        out = command(p, now_sim_ns=stamp, now_wall_ns=stamp, receipt_ns=stamp,
                      scan_wheel_pose=pose, current_wheel_pose=pose, target_mps=1., acceleration_mps2=0.)
        assert out['mode'] != 'STOP', (i, pose, out['reason'])
        if out['mode'] == 'NOMINAL':
            assert used_rollout and pose[0] > 8. and abs(pose[1]) <= .25
            break
        k = np.tan(out['steer_rad'])/effective_response_length(1., 'awsim_understeer_v1')
        pose[:2] += .1*np.array([np.cos(pose[2]+.05*k), np.sin(pose[2]+.05*k)])
        pose[2] += .1*k
        body = np.array([[-.6,-.7],[-.6,.7],[1.8,-.7],[1.8,.7]])
        corners = transform(body, pose)
        assert corners[:, 1].min() > -5. and corners[:, 1].max() < 1.8
        box = np.array([[6.2,-1.], [6.8,-1.], [6.8,-.4], [6.2,-.4]])
        axes = np.array([[1.,0.], [0.,1.], [np.cos(pose[2]),np.sin(pose[2])],
                         [-np.sin(pose[2]),np.cos(pose[2])]])
        vehicle_projection, box_projection = corners@axes.T, box@axes.T
        separated = ((vehicle_projection.max(axis=0) < box_projection.min(axis=0))
                     | (box_projection.max(axis=0) < vehicle_projection.min(axis=0)))
        assert separated.any(), 'vehicle rectangle intersects the box'
    else:
        pytest.fail('short-horizon avoidance did not rejoin in 22 simulated seconds')
