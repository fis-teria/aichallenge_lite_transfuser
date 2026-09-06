"""One fixed-model inference/MPC worker, with no ROS publisher capability.

All sensors are received by the separate supervisor. Queue capacity is one;
bundles contain bounded history, not a replacement Dataset or teacher stream.
"""
from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import queue
import time

import numpy as np
import torch
import yaml

from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference, reference_to_world
from aic_transfuser_lite.control.spatial_mpc_v4 import SpatialMPC
from aic_transfuser_lite.control.spatial_speed_profile_v4 import rolling_horizon
from aic_transfuser_lite.control.spatial_path_adapter_v4 import transform
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate, plain, sha
from aic_transfuser_lite.control.spatial_sim_guard_v4 import MotionEvidence, scan_coverage
from aic_transfuser_lite.control.sim_dispatch_v4 import AckermannDispatch
from .spatial_input_v4 import SpatialInputV4, PassiveCommand, Stamp, freeze_batch, INPUT_FIELDS
from .spatial_runtime_v4 import load_fixed, state_inventory, SpatialRuntimeV4
from .spatial_sim_adapter_v4 import Sample, align_observation, interpolate


def worker_main(inbox: object, outbox: object, stop: object, config_path: str, output_dir: str) -> None:
    """Spawn entry point. Always inventory all model state before and after."""
    torch.set_num_threads(1)
    cfg = yaml.safe_load(Path(config_path).read_text())
    output = Path(output_dir)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    adapter = SpatialInputV4(command_binding_known=True, command_policy='SIM_ONLY_POLICY_CHANGED')
    mpc = SpatialMPC(cfg)
    dispatch = AckermannDispatch(cfg)
    model = None
    load_map = None
    forward_calls = fit_calls = 0
    previous_epoch = None
    last_camera = -1
    snapshot_count = 0
    first_error = None
    trace = (output/'worker.jsonl').open('x', buffering=1)

    def record(event: dict) -> None:
        if trace.tell() > 128*1024*1024:
            raise RuntimeError('WORKER_LOG_BUDGET')
        trace.write(json.dumps(plain(event), allow_nan=False, separators=(',', ':'))+'\n')
        trace.flush()

    def notify(event: dict) -> None:
        try:
            outbox.put_nowait(event)
        except queue.Full:
            record(dict(event='RESULT_QUEUE_FULL', input_id=event.get('input_id')))

    try:
        model, load_map = load_fixed(device)
        (output/'checkpoint_load_map.json').write_text(json.dumps(load_map, indent=2))
        record(dict(event='FIXED_MODEL_READY', device=device, checkpoint=load_map['before_sha256']))
        notify(dict(event='READY', device=device))
        while not stop.is_set() and forward_calls < cfg['dev']['forward_limit']:
            try:
                bundle = inbox.get(timeout=.10)
            except queue.Empty:
                continue
            epoch = bundle['epoch']
            if epoch != previous_epoch:
                adapter.reset('EPOCH_CHANGE')
                mpc.warm = None
                last_camera = -1
                previous_epoch = epoch
            for sent in bundle['sent_commands']:
                if sent['epoch'] != epoch:
                    continue
                adapter.add_command(PassiveCommand(
                    Stamp(sent['sim_ns'], sent['monotonic_ns'], sent['monotonic_ns'],
                          'AWSIM_ROS', epoch, 'HOST_MONOTONIC'),
                    sent['steering_rad'], sent['desired_speed_reference_mps'],
                    sent['acceleration_mps2'], source='sim_sent'))
            alignments = []
            for camera in bundle['camera']:
                if camera.ns <= last_camera:
                    continue
                try:
                    obs, align = align_observation(camera, bundle['lidar'], bundle['velocity'],
                                                   bundle['steering'], cutoff_ns=bundle['cutoff_ns'],
                                                   grid_phase_ns=bundle['grid_phase_ns'])
                    appended = adapter.append(obs, bundle['cutoff_ns'])
                    alignments.append(dict(camera_ns=camera.ns, append=appended, alignment=align))
                    if appended in ('ACCEPTED', 'DUPLICATE'):
                        last_camera = camera.ns
                except ValueError as exc:
                    alignments.append(dict(camera_ns=camera.ns, rejection=str(exc)))
            if not adapter.frames or last_camera != bundle['camera'][-1].ns:
                record(dict(event='INPUT_NOT_READY', alignments=alignments))
                notify(dict(event='REJECTED', reason='INPUT_NOT_READY', input_id=bundle['input_id']))
                continue
            started = time.monotonic_ns()
            result = dict(event='CYCLE', epoch=epoch, input_id=bundle['input_id'],
                          observed_sim_ns=last_camera,
                          deadline_monotonic_ns=bundle['camera'][-1].received_ns+200_000_000,
                          forward_calls=forward_calls, solver_accepted=False, request=None,
                          motion_rejection='NOT_EVALUATED', alignments=alignments)
            try:
                batch, provenance = adapter.build(bundle['cutoff_ns'])
                batch = freeze_batch(batch, device)
                if device == 'cuda':
                    torch.cuda.synchronize()
                finalized = time.monotonic_ns()
                forward_calls += 1  # before call, including an exception
                result['forward_calls'] = forward_calls
                result['forward_id'] = f'{bundle["input_id"]}:forward:{forward_calls}'
                record(dict(event='FORWARD_STARTED', forward_id=result['forward_id'],
                            finalized_monotonic_ns=finalized, input_id=bundle['input_id']))
                with torch.inference_mode():
                    returned = model(batch)
                if not isinstance(returned, torch.Tensor) or returned.shape != (1, 20, 2) or returned.dtype != torch.float32:
                    raise ValueError('OUTPUT_CONTRACT')
                raw = SpatialRuntimeV4.snapshot(returned)
                snap_ready = time.monotonic_ns()
                result.update(raw_xy_m=raw.tolist(), raw_bits_hex=raw.tobytes().hex(), raw_sha256=sha(raw.tobytes()),
                              nominal_s_m=(np.arange(1, 21)/10).tolist(), output_frame='base_link@t_obs',
                              timing=dict(start_monotonic_ns=started, finalized_monotonic_ns=finalized,
                                          inference_end_monotonic_ns=snap_ready, inference_wall_s=(snap_ready-finalized)*1e-9),
                              input_masks={name:getattr(batch, name).cpu().tolist() for name in
                                           ('image_mask', 'lidar_mask', 'ego_feature_mask', 'command_mask')},
                              input_tensor_hashes={name:sha(getattr(batch, name).cpu().contiguous().numpy().tobytes())
                                                   for name in INPUT_FIELDS},
                              history=dict(command_policy=provenance['command_policy'],
                                           commands=[None if c is None else asdict(c) for c in provenance['commands']],
                                           sensor_camera_ns=[f.camera.header_ns for f in provenance['sensor_frames']],
                                           ego_camera_ns=[f.camera.header_ns for f in provenance['ego_frames']],
                                           cutoff_ns=bundle['cutoff_ns']))
                if snapshot_count < 1:
                    arrays = {name:getattr(batch, name).cpu().numpy() for name in INPUT_FIELDS}
                    arrays['raw_xy_m'] = raw
                    with (output/f'input_snapshot_{snapshot_count}.npz').open('xb') as f:
                        np.savez_compressed(f, **arrays)
                    snapshot_count += 1
                if not np.isfinite(raw).all():
                    raise ValueError('NONFINITE_OUTPUT')
                ego = adapter.frames[-1][0].ego_si
                initial = np.array([cfg['rear_x_in_base_m'], 0., 0., ego[0], ego[3]])
                candidate = SpatialPathCandidate(raw, np.arange(1, 21)/10, result['forward_id'], t_obs=last_camera)
                path = constrained_reference(candidate, cfg, initial)
                fit_calls += path.diagnostics.get('fit_solve_count', 0)
                result['reference'] = dict(xy_m=path.world_xy, actual_s_m=path.actual_s,
                                           reason=path.reason, diagnostics=path.diagnostics)
                if path.reason:
                    raise ValueError('REFERENCE_' + path.reason)
                # Actual pose samples are current GNSS+IMU, independently marked
                # approximate until the full time/extrinsic binding is verified.
                pose, pose_provenance = interpolate(bundle['pose'], last_camera, angle_columns=(2,))
                current = bundle['pose'][-1]
                world_path = reference_to_world(path, pose, observation_epoch=epoch, control_epoch=current.epoch)
                current_ego = bundle['velocity'][-1]
                current_delta = bundle['steering'][-1].value[0]
                rear_xy = transform(np.array([[cfg['rear_x_in_base_m'], 0.]]), current.value)[0]
                z = np.r_[rear_xy, current.value[2], current_ego.value[0], current_delta]
                progress = float(world_path.actual_s[np.argmin(np.linalg.norm(world_path.world_xy-rear_xy, axis=1))])
                # This bounded nearest vertex is ONLY progress within one accepted
                # reference, never raw-to-reference correspondence or distance traveled.
                scan_sample = bundle['lidar'][-1]
                scan_pose, _ = interpolate(bundle['pose'], scan_sample.ns, angle_columns=(2,))
                ref_states = np.column_stack([transform(world_path.world_xy, scan_pose, inverse=True),
                    np.asarray(path.diagnostics['reference_yaw_rad'])+pose[2]-scan_pose[2],
                    np.zeros((len(world_path.world_xy), 2))])
                coverage = scan_coverage(ref_states, scan_sample.value, cfg)
                evidence = MotionEvidence(isolation=bundle['isolation'], consumer=bundle['consumer'],
                    pose_timing=bundle['pose_timing_verified'], footprint_profile=cfg['dev']['footprint_profile_verified'],
                    clearance=coverage['verified'], collision_monitor=bundle['collision_monitor_healthy'],
                    logger=True, command_history=bool(batch.command_mask.any()))
                motion_reason = evidence.reason()
                permission = 'RUN' if cfg['enabled'] and bundle['run_requested'] and motion_reason is None else 'HOLD'
                previous = np.array([bundle['previous_acceleration'], 0.])
                delay = max(0., (time.monotonic_ns()-bundle['camera'][-1].received_ns)*1e-9)+.10
                reference = rolling_horizon(world_path, cfg, progress_s=progress, current_v=max(0., z[3]),
                    previous_a=previous[0], delay_s=delay, permission=permission,
                    safety_cap_mps=cfg['maximum_speed_mps'] if motion_reason is None else 0.)
                result.update(pose_at_observation=pose, pose_provenance=pose_provenance,
                              current_state=z, state_source_ns=current.ns, coverage=coverage,
                              evidence=asdict(evidence), permission=permission, speed_reference=reference,
                              motion_rejection=motion_reason or ('RUN_NOT_REQUESTED' if permission != 'RUN' else None))
                if reference['reason']:
                    raise ValueError(reference['reason'])
                solved = mpc.solve(z, reference, previous, {})
                result['mpc'] = dict(diagnostics=solved.diagnostics, states=solved.states, controls=solved.controls)
                result['solver_accepted'] = solved.accepted
                if solved.accepted:
                    rollout_scan = solved.states.copy()
                    rollout_scan[:, :2] = transform(solved.states[:, :2], scan_pose, inverse=True)
                    rollout_scan[:, 2] -= scan_pose[2]
                    result['rollout_coverage'] = scan_coverage(rollout_scan, scan_sample.value, cfg)
                    if not result['rollout_coverage']['verified']:
                        result['motion_rejection'] = 'ROLLOUT_FREE_SPACE_UNVERIFIED'
                    dt = bundle['control_dt_s']
                    result['request'] = dispatch.request(result['forward_id']+':mpc', solved.first_control,
                        measured_delta_rad=current_delta, control_dt_s=dt,
                        target_speed_mps=float(reference['speed'][0]))
            except Exception as exc:
                result['reason'] = type(exc).__name__+': '+str(exc)
            result.update(fit_calls=fit_calls, mpc_calls=mpc.calls,
                          cycle_end_monotonic_ns=time.monotonic_ns())
            record(result)
            notify(plain(result))
        notify(dict(event='WORKER_FINISHED', forward_calls=forward_calls, mpc_calls=mpc.calls))
    except BaseException as exc:
        first_error = type(exc).__name__+': '+str(exc)
        try:
            record(dict(event='WORKER_EXCEPTION', reason=first_error))
            notify(dict(event='WORKER_EXCEPTION', reason=first_error))
        except Exception:
            pass  # original error is retained below; supervisor independently stops
    finally:
        state_after = state_inventory(model) if model is not None else None
        summary = dict(forward_calls=forward_calls, mpc_calls=mpc.calls, reference_fit_calls=fit_calls,
                       sensor_tensor_snapshots=snapshot_count, error=first_error,
                       state_unchanged=state_after == load_map['state'] if load_map else None,
                       state_after=state_after, device=device, scope='SIM_E2E_CONTROLLED_TEST')
        try:
            (output/'worker_summary.json').write_text(json.dumps(summary, indent=2))
        finally:
            trace.close()
