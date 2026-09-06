"""Finite pure-array/synthetic tests. No checkpoints, ROS, assets or sensors."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from aic_transfuser_lite.runtime.spatial_input_v4 import SpatialInputV4, PassiveCommand, Stamp, GridObservation
from aic_transfuser_lite.runtime.spatial_sim_adapter_v4 import Sample, interpolate, align_observation, base_pose_from_gnss_imu
from aic_transfuser_lite.control.spatial_sim_guard_v4 import MotionEvidence, OperationLease, scan_coverage

ROOT = Path(__file__).parents[1]


def config():
    c = yaml.safe_load((ROOT/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    return dict(c, lidar_x_in_base_m=1.65, lidar_y_in_base_m=0.)


def sample(ns, value, frame='base_link', epoch='0', received=None):
    return Sample(ns, ns if received is None else received, value, frame, epoch)


def test_bracketing_and_availability_are_not_receipt_timestamps():
    a, b = sample(100, [0., 3.13], received=201), sample(200, [2., -3.13], received=205)
    value, proof = interpolate([a, b], 150, angle_columns=(1,))
    assert value[0] == 1. and abs(value[1]) > 3.1
    assert proof['source_stamps_ns'] == [100, 200] and proof['available_ns'] == 205
    with pytest.raises(ValueError, match='missing_bracket'):
        interpolate([a], 150)
    with pytest.raises(ValueError, match='MIXED_EPOCH'):
        interpolate([a, replace(b, epoch='1')], 150)


def test_sim_history_is_explicit_past_only_and_passive_unchanged():
    builder = SpatialInputV4(command_binding_known=True, command_policy='SIM_ONLY_POLICY_CHANGED')
    stamp = Stamp(100_000_000, 1, 1)
    cmd = PassiveCommand(stamp, .1, .2, .3, source='sim_sent')
    assert builder.add_command(cmd) == 'ACCEPTED'
    assert builder._command_for(stamp, stamp, 1)[0] is None
    future = replace(stamp, header_ns=200_000_000)
    selected, retained = builder._command_for(stamp, future, 1)
    assert selected[1] == 'sim_sent' and retained is cmd
    assert builder._command_for(stamp, future, 0)[0] is None
    with pytest.raises(ValueError): builder.add_command(replace(cmd, source='nominal'))
    with pytest.raises(ValueError): SpatialInputV4().add_command(cmd)


def test_true_warmup_padding_stays_false():
    b = SpatialInputV4(command_binding_known=True, command_policy='SIM_ONLY_POLICY_CHANGED')
    t = Stamp(100_000_000, 1, 1)
    obs = GridObservation(100_000_000, t, t, t, np.zeros((8, 8, 3), dtype=np.uint8), np.ones(750), (0., 0., 0., 0.))
    assert b.append(obs, 1) == 'ACCEPTED'
    batch, provenance = b.build(1)
    assert not batch.command_mask.any() and batch.image_mask.tolist() == [[False, False, False, True]]
    assert provenance['command_policy'] == 'SIM_ONLY_POLICY_CHANGED'
    assert b.append(replace(obs, grid_ns=200_000_000, camera=replace(t, header_ns=200_000_000),
                            ego_stamp=replace(t, header_ns=200_000_000)), 1) == 'ACCEPTED'
    with pytest.raises(ValueError, match='COMMAND_SOURCE_STALE'): b.build(1)


def test_sensor_geometry_and_clock_contract():
    ns = 1_000_000_000
    camera = sample(ns, np.zeros((8, 8, 3), dtype=np.uint8), 'camera_optical_link')
    scan = dict(ranges=np.ones(750), angle_min=-1.5666074752807617,
                angle_increment=.004188789986073971, range_min=0., range_max=25.)
    lidar = [sample(ns, scan, 'lidar')]
    v = [sample(ns-10_000_000, [0., 0., 0.]), sample(ns+10_000_000, [.2, 0., 0.])]
    d = [sample(s.ns, [0.]) for s in v]
    obs, _ = align_observation(camera, lidar, v, d, cutoff_ns=ns+20_000_000, grid_phase_ns=ns)
    assert obs.ego_stamp.header_ns == ns and obs.ego_si[0] == .1
    with pytest.raises(ValueError, match='UNSUPPORTED_LIDAR_GEOMETRY'):
        align_observation(camera, [replace(lidar[0], value=dict(scan, angle_min=-3.14))], v, d,
                          cutoff_ns=ns+20_000_000, grid_phase_ns=ns)
    with pytest.raises(ValueError, match='UNAVAILABLE'):
        align_observation(camera, lidar, v, d, cutoff_ns=ns, grid_phase_ns=ns)


def test_gnss_imu_extrinsics_do_not_assume_rear_or_heading_csv():
    # Sensor local yaw +90deg with base yaw 0; GNSS antenna is -.26m.
    p = base_pose_from_gnss_imu(np.array([10., 20.]), [0., 0., np.sqrt(.5), np.sqrt(.5)])
    assert np.allclose(p, [10.26, 20., 0.])
    with pytest.raises(ValueError): base_pose_from_gnss_imu(np.zeros(2), [0., 0., 0., 0.])


def test_full_footprint_is_unknown_behind_front_lidar():
    scan = dict(ranges=np.full(750, 25.), angle_min=-np.pi/2, angle_increment=np.pi/749,
                range_min=0., range_max=25.)
    proof = scan_coverage(np.zeros((1, 5)), scan, config())
    assert not proof['verified'] and proof['unknown_cells'] > 0
    assert proof['prior_footprint_assumed_free'] is False
    full = dict(scan, angle_min=-np.pi, angle_increment=2*np.pi/749)
    # Move the footprint safely forward of the scan to avoid enclosing scanner.
    z = np.array([[6., 0., 0., 0., 0.]])
    assert scan_coverage(z, full, config())['verified']
    full['ranges'][370:380] = np.nan
    assert not scan_coverage(z, full, config())['verified']


def result():
    return dict(epoch='0', input_id='i', observed_sim_ns=100, deadline_monotonic_ns=200,
                request={'operation_id':'u'}, solver_accepted=True, motion_rejection=None)


def test_expired_and_stale_results_do_not_rearm():
    lease = OperationLease()
    kwargs = dict(now_ns=199, now_sim_ns=110, epoch='0', current_input_id='i', current_state_ns=105, healthy=True)
    assert lease.reject(result(), **kwargs) is None
    assert lease.reject(result(), **dict(kwargs, now_ns=200)) == 'WORKER_DEADLINE'
    assert lease.reject(result(), **dict(kwargs, current_input_id='new')) == 'STALE_INPUT_OR_EPOCH'
    lease.sent({'operation_id':'sent', 'acceleration_mps2':.1})
    assert lease.watchdog(now_ns=1_000_000_000, last_accepted_ns=0, state_received_ns=1_000_000_000,
                          speed_mps=.1, worker_alive=True, logger_ok=True) == 'WORKER_STALL'
    assert lease.reject(result(), **kwargs) == 'WORKER_STALL'


@pytest.mark.parametrize('fault', ['logger', 'worker', 'state', 'speed'])
def test_independent_faults_latch_without_worker_result(fault):
    lease = OperationLease(); lease.sent({'operation_id':'x', 'acceleration_mps2':.1})
    args = dict(now_ns=1_000_000_000, last_accepted_ns=1_000_000_000, state_received_ns=1_000_000_000,
                speed_mps=.1, worker_alive=True, logger_ok=True)
    if fault == 'logger': args['logger_ok'] = False
    if fault == 'worker': args['worker_alive'] = False
    if fault == 'state': args['state_received_ns'] = 0
    if fault == 'speed': args['speed_mps'] = .31
    assert lease.watchdog(**args) and lease.fault


def test_no_motion_permission_from_finite_xy_alone():
    assert MotionEvidence().reason() == 'UNVERIFIED_ISOLATION'
    assert MotionEvidence(True, True, True, True, True, False, True, True).reason() == 'UNVERIFIED_COLLISION_MONITOR'


def test_isolation_rejects_extra_service_before_ros():
    path = ROOT/'tools/run_spatial_sim_dev_v4.py'
    spec = importlib.util.spec_from_file_location('dev_supervisor_test', path)
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    with pytest.raises(ValueError, match='UNEXPECTED_COMPOSE_SCOPE'):
        module.verify_dev_isolation([], 'codex-v4-dev-test')


def test_make_entry_and_no_existing_classic_controller_or_dataset():
    runner = (ROOT/'tools/spatial_dev_runner.py').read_text()
    assert "normal_racingkart_makefile_executed=False" in runner
    assert 'run_autoware.bash' not in runner and 'capture_run_fingerprint' not in runner
    assert "network_mode='service:simulator'" in runner
    script = (ROOT/'integrations/awsim_dev_v4/simulator.sh').read_text()
    assert 'run_simulator.bash dev' in script
