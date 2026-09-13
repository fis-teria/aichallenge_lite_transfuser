"""Investigate time-prediction / PP compatibility without changing runtime policy."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any

import numpy as np

from aic_transfuser_lite.control.awsim_steering import steering_response_gain, command_steering
from aic_transfuser_lite.control.time_geometry_v2 import validate_time_geometry, TimeGeometryConfig
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose, TimePlan, prepare_time_reference
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length
from aic_transfuser_lite.control.waypoint_controller import ControllerConfig, control_from_waypoints
from aic_transfuser_lite.evaluation.time_lookahead_audit_v1 import audit_polyline, angle_for_point
from aic_transfuser_lite.evaluation.time_smoothing_v1 import distribution, probe_recorded_pp
from evaluate_time_awsim_trial import replay_recorded_control
from evaluate_time_smoothing import sha, read_json, write_json


def context(xy: np.ndarray, command: dict[str, Any], cfg: dict[str, Any]) -> tuple[Any, TimePlan, TimedBodyPose]:
    detail = command['details']
    observed = TimedBodyPose(**detail['observation_pose']); current = TimedBodyPose(**detail['current_pose'])
    plan = TimePlan(command['plan_id'], observed, xy)
    reference = prepare_time_reference(plan, current,
        rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'], 0.))
    return reference, plan, current


def analyze_one(xy: np.ndarray, command: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    reference, plan, current = context(xy, command, cfg)
    speed = command['speed_mps']; length = effective_response_length(max(0., speed), cfg['vehicle_model_policy'])
    minimum = max(1., .4+max(0., speed)*.5+speed**2/2)
    audit = audit_polyline(reference.xy_current_m, reference.remaining_sec, minimum_m=minimum,
                           maximum_m=minimum+.5, response_length_m=length)
    original = probe_recorded_pp(xy, command, cfg)
    try:
        geometry = validate_time_geometry(xy)
        geometry_reason = 'RESOLVED' if geometry['motion_resolved'] else 'MOTION_UNRESOLVED'
    except ValueError as exc:
        geometry_reason = str(exc)
    forward = reference.xy_current_m[1:][reference.xy_current_m[1:, 0] > 1e-6]
    stopping = bool(len(forward) and np.linalg.norm(forward[-1]) >= .1+max(0., speed)*.5+speed**2/2)
    midpoint = audit['continuous_midpoint']
    continuous_pass = geometry_reason == 'RESOLVED' and stopping and midpoint is not None
    nominal = None
    if continuous_pass:
        target = np.array(midpoint['midpoint_float32_xy_m'])
        control = control_from_waypoints(target[None], 5/3.6, max(0., speed),
            ControllerConfig(wheelbase_m=length, min_lookahead_m=1., max_steer_rad=.5,
                             min_accel_mps2=-1., max_accel_mps2=1., speed_kp=4.))
        assert abs(control.steering_rad) <= .3
        assert minimum <= np.linalg.norm(target) <= minimum+.5
        nominal = {'steer_rad': control.steering_rad, 'acceleration_mps2': control.acceleration_mps2,
                   'steady_ros_input_rad': control.steering_rad/steering_response_gain(cfg['steering_policy'])}
    # Frame/precision sensitivity only. These alternatives are not control proposals.
    no_offset = prepare_time_reference(plan, current, rear_axle_offset_m=(0., 0.))
    no_age = prepare_time_reference(plan, replace(current, stamp_ns=plan.observation.stamp_ns),
        rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'], 0.))

    def vertex_pass(points: np.ndarray) -> bool:
        return any(p[0] > 1e-6 and minimum <= np.linalg.norm(p) <= minimum+.5 and abs(angle_for_point(p, length)) <= .3
                   for p in np.asarray(points[1:], dtype=np.float32))

    inside = [p for p in audit['vertices'] if p['in_band']]
    last_inside = inside[-1] if inside else None
    return {'sim_ns': command['sim_ns'], 'plan_id': command['plan_id'], 'recorded_reason': command['reason'],
        'raw_pp_reason': original['reason'], 'geometry_reason': geometry_reason, 'speed_mps': speed,
        'plan_age_s': reference.age_sec, 'predicted_source_speed_mps': reference.target_speed_mps,
        'pose_translation_since_observation_m': math.hypot(current.x_m-plan.observation.x_m, current.y_m-plan.observation.y_m),
        'pose_yaw_change_rad': math.atan2(math.sin(current.yaw_rad-plan.observation.yaw_rad), math.cos(current.yaw_rad-plan.observation.yaw_rad)),
        'last_discrete_point_in_band': last_inside, 'continuous_nominal_pp_pass': continuous_pass,
        'continuous_nominal_control': nominal, 'reference_stopping_distance_pass': stopping,
        'no_rear_offset_vertex_pass': vertex_pass(no_offset.xy_current_m),
        'no_age_trim_vertex_pass': vertex_pass(no_age.xy_current_m), 'audit': audit}


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    raw = np.array([r['raw_pp_reason'] == 'PP_OK' for r in rows])
    continuous = np.array([r['continuous_nominal_pp_pass'] for r in rows])
    return {'commands': len(rows), 'raw_pp_pass': int(raw.sum()), 'continuous_nominal_pp_pass': int(continuous.sum()),
        'rescued_raw_rejections': int((~raw & continuous).sum()), 'lost_raw_pass': int((raw & ~continuous).sum()),
        'geometry_reasons': dict(Counter(r['geometry_reason'] for r in rows)),
        'float64_vertex_pass': sum(r['audit']['float64_vertex_pass'] for r in rows),
        'no_rear_offset_vertex_pass': sum(r['no_rear_offset_vertex_pass'] for r in rows),
        'no_age_trim_vertex_pass': sum(r['no_age_trim_vertex_pass'] for r in rows),
        'plan_age_s': distribution([r['plan_age_s'] for r in rows]),
        'pose_translation_m': distribution([r['pose_translation_since_observation_m'] for r in rows]),
        'pose_yaw_change_abs_rad': distribution([abs(r['pose_yaw_change_rad']) for r in rows]),
        'predicted_source_speed_mps': distribution([r['predicted_source_speed_mps'] for r in rows]),
        'float32_angle_error_abs_rad': distribution([r['audit']['maximum_float32_angle_error_rad'] for r in rows]),
        'midpoint_steering_margin_rad': distribution([r['audit']['continuous_midpoint']['midpoint_margin_rad'] for r in rows if r['continuous_nominal_pp_pass']]),
        'first_feasible_interval_length_m': distribution([r['audit']['continuous_midpoint']['interval_length_m'] for r in rows if r['continuous_nominal_pp_pass']]),
        'feasible_vertex_beyond_upper_extra_m': distribution([r['audit']['first_feasible_vertex_beyond_band']['distance_m']-r['audit']['preview_band_m'][1]
             for r in rows if r['audit']['first_feasible_vertex_beyond_band'] is not None])}


def trial(run: Path, evidence: Path, output: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    expected = read_json(evidence/'summary.json')
    for name, digest in expected['source_sha256'].items():
        if sha(run/name) != digest:
            raise ValueError('source changed: '+str(run/name))
    cfg = read_json(run/'trial_config.json'); validate_trial_config(cfg)
    host = read_json(run/'host_result.json')
    assert sha(run/'trial_config.json') == host['trial_config_sha256']
    records = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    plans = [json.loads(line) for line in (run/'inference.jsonl').read_text().splitlines()]
    plans = [p for p in plans if p.get('event') == 'PLAN']
    by_id = {p['plan_id']: p for p in plans}
    assert len(by_id) == len(plans) and all(p['checkpoint_sha256'] == cfg['checkpoint_sha256'] for p in plans)
    starts = [r['sim_ns'] for r in records if r.get('event') == 'ARMED']; assert len(starts) == 1
    start = starts[0]; commands = [r for r in records if r.get('event') == 'COMMAND_SENT']
    end = min(start+round(cfg['drive_limit_sim_s']*1e9), max(c['sim_ns'] for c in commands)+1)
    stops = [r['sim_ns'] for r in records if r.get('event') == 'STOP_REQUESTED']
    if stops: end = min(end, min(stops))
    active = [c for c in commands if start <= c['sim_ns'] < end]
    replay = replay_recorded_control(active, plans, cfg['geometry']['rear_axle_forward_in_base_link_m'],
        **{k: cfg[k] for k in ('speed_policy', 'obstacle_policy', 'steering_policy', 'lookahead_policy', 'vehicle_model_policy')})
    eligible = [c for c in active if c.get('plan_id') and c.get('speed_mps') is not None
                and all(k in c.get('details', {}) for k in ('observation_pose', 'current_pose'))]
    assert len(eligible) == expected['control_replay']['matched_commands']
    rows = []
    with (output/(run.name+'_audit.jsonl')).open('x') as stream:
        for c in eligible:
            row = analyze_one(np.asarray(by_id[c['plan_id']]['raw_xy_m']), c, cfg)
            rows.append(row); stream.write(json.dumps(row, allow_nan=False)+'\n')
    launch = [r for r in rows if r['sim_ns'] < start+5_200_000_000]
    first = rows[0]
    # Fixed frozen launch inputs: changing speed here is sensitivity, not simulation.
    launch_commands = [c for c in eligible if c['sim_ns'] < start+5_200_000_000]
    speed_sweep = {}
    for kmh in (0., 1., 3., 5.):
        probes = []
        for c in launch_commands:
            probes.append(analyze_one(np.asarray(by_id[c['plan_id']]['raw_xy_m']), {**c, 'speed_mps': kmh/3.6}, cfg))
        speed_sweep[str(kmh)] = summarize(probes)
    write_json(output/(run.name+'_first_command.json'), first)
    return {'run_id': run.name, 'source_sha256': expected['source_sha256'], 'config_sha256': sha(run/'trial_config.json'),
        'active_commands': len(active), 'unavailable_commands': len(active)-len(eligible), 'raw_replay': replay,
        'full': summarize(rows), 'launch': summarize(launch), 'frozen_launch_speed_sensitivity_kmh': speed_sweep,
        'first_command': first, 'first_nominal_mapping_from_zero_50ms':
            command_steering(first['continuous_nominal_control']['steer_rad'], 0., .05, policy=cfg['steering_policy'])
            if first['continuous_nominal_pp_pass'] else None}, cfg


def plot(result: dict[str, Any], output: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    for ax, name in zip(axes, ('baseline', 'candidate')):
        first = result['trials'][name]['first_command']; audit = first['audit']
        points = np.array([p['xy_m'] for p in audit['vertices']])
        line = np.concatenate([a+np.linspace(0, 1, 80)[:, None]*(b-a) for a, b in zip(points, points[1:])])
        distance = np.linalg.norm(line, axis=1)
        angle = [abs(angle_for_point(p, audit['response_length_m'])) for p in line]
        ax.plot(distance, angle, color='#0072b2', label='Same polyline between points')
        ax.scatter([p['distance_m'] for p in audit['vertices']], [abs(p['angle_rad']) for p in audit['vertices']],
                   c='#d55e00', s=25, label='Original 0.1 s vertices', zorder=3)
        midpoint = audit['continuous_midpoint']
        if midpoint:
            ax.scatter(midpoint['midpoint_distance_m'], abs(midpoint['midpoint_angle_rad']), marker='*', s=170,
                       c='#009e73', zorder=5, label='Strict float32 feasible midpoint')
        ax.axvspan(1., 1.5, color='#aaaaaa', alpha=.16); ax.axvline(1.5, color='grey', linestyle=':')
        ax.axhline(.3, c='black', linestyle='--', label='Tire-angle limit (0.3 rad)')
        ax.set(xlim=(1., 1.65), ylim=(.288, .33), xlabel='Distance from current rear axle (m)',
               title=name+' first launch command'); ax.grid(alpha=.2)
    axes[0].set_ylabel('Required absolute PP tire angle (rad)')
    axes[1].legend(fontsize=8, loc='upper right')
    fig.suptitle('Recorded launch: discrete vertices versus unchanged path segments (offline)')
    fig.tight_layout(); fig.savefig(output/'launch_segment_audit.png', dpi=170); plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--records', type=Path, required=True); ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); start = time.monotonic(); args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    write_json(args.output/'plan.json', {'source_commit': source, 'recorded_runs': ['baseline', 'candidate'],
        'tests': ['exact same-band segment existence', 'float32 vs float64 vertices', 'age trim and 1mm offset sensitivity',
                  'frozen launch inputs at 0,1,3,5 kmh'], 'runtime_changes': False,
        'limits': {'steering_rad': .3, 'additional_preview_m': .5}, 'records': str(args.records)})
    trials = {}; configs = {}
    for name, rid in [('baseline', 'codex-time-recovery-model-base01'), ('candidate', 'codex-time-recovery-model-candidate01')]:
        trials[name], configs[name] = trial(args.records/name/rid,
            repo/'docs/evidence/time_recovery_finetune_20260914'/f'{name}_evaluation', args.output)
        print(json.dumps({'trial': name, 'launch': trials[name]['launch'], 'full': trials[name]['full']}), flush=True)
    gain = steering_response_gain(configs['candidate']['steering_policy'])
    result = {'status': 'COMPLETE_OFFLINE_AUDIT', 'source_commit': source, 'trials': trials,
        'limit_contract': {'ros_input_limit_rad': .5, 'steering_gain': gain, 'steady_tire_limit_rad': .5*gain,
            'geometry_screen_max_steer_rad': TimeGeometryConfig().max_steer_rad,
            'response_length_at_0_m': effective_response_length(0., configs['candidate']['vehicle_model_policy']),
            'response_length_at_5kmh_m': effective_response_length(5/3.6, configs['candidate']['vehicle_model_policy'])},
        'elapsed_s': time.monotonic()-start, 'runtime_modified': False,
        'boundary': 'nominal geometry and PP only; no changed-state closed loop, actuator sequence or scan collision certification'}
    plot(result, args.output); write_json(args.output/'summary.json', result)
    print(json.dumps({'status': result['status'], 'seconds': result['elapsed_s']}), flush=True)


if __name__ == '__main__':
    main()
