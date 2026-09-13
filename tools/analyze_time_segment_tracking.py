"""Attribute a completed segment-policy trial using its unchanged recorded states."""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from aic_transfuser_lite.control.awsim_steering import command_steering
from aic_transfuser_lite.control.awsim_steering_response import SteeringResponseState, compensate_steering_response
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin
from aic_transfuser_lite.evaluation.time_smoothing_v1 import distribution
from evaluate_time_awsim_trial import response_record_matches


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    records = [json.loads(s) for s in (args.run/'control.jsonl').read_text().splitlines()]
    plans = [json.loads(s) for s in (args.run/'inference.jsonl').read_text().splitlines()]
    plans = {p['plan_id']: p for p in plans if p.get('event') == 'PLAN'}
    cfg = json.loads((args.run/'trial_config.json').read_text())
    armed = next(r for r in records if r['event'] == 'ARMED')
    scan_event = next(r for r in records if r['event'] == 'SCAN_GUARD_REJECTED')
    fault = next(r for r in records if r['event'] == 'COMMAND_SENT' and r['reason'] == 'STOPPING_SWEEP_OCCUPIED')
    commands = [r for r in records if r['event'] == 'COMMAND_SENT' and r['monotonic_ns'] >= armed['monotonic_ns']
                and r['monotonic_ns'] <= fault['monotonic_ns']]
    tracked = [r for r in commands if r['reason'] == 'TIME_PATH_TRACKING']
    selections = []; legacy_failures = []; metadata_matches = 0
    for command in commands:
        d = command.get('details', {})
        if not command.get('plan_id') or not all(k in d for k in ('observation_pose', 'current_pose')):
            continue
        plan = TimePlan(command['plan_id'], TimedBodyPose(**d['observation_pose']), np.array(plans[command['plan_id']]['raw_xy_m']))
        current = TimedBodyPose(**d['current_pose'])
        options = dict(speed_mps=command['speed_mps'], rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'], 0.),
                       speed_policy=cfg['speed_policy'], vehicle_model_policy=cfg['vehicle_model_policy'])
        if 'lookahead_selection' in d:
            new = time_trial_control(plan, current, **options, lookahead_policy=cfg['lookahead_policy'])
            assert response_record_matches(d['lookahead_selection'], new['lookahead_selection'])
            metadata_matches += 1
            selections.append({'time_s': (command['sim_ns']-armed['sim_ns'])/1e9, **d['lookahead_selection']})
            try:
                old = time_trial_control(plan, current, **options, lookahead_policy='stopping_preview_v1')
            except ValueError as exc:
                assert str(exc) == 'STEERING_FEASIBLE_LOOKAHEAD_MISSING'
                legacy_failures.append(selections[-1]['time_s'])
            else:
                for key in ('steer_rad', 'acceleration_mps2', 'target_speed_mps', 'lookahead_rear_m'):
                    assert old[key] == new[key]
    d = fault['details']; actuator = d['steering_actuator']; motion = fault['motion_observation']
    def margin(issued: float) -> dict:
        return scan_margin(scan_event['scan'], scan_event['scan_in_current_rear'], speed_mps=fault['speed_mps'],
            measured_rad=fault['measured_steer_rad'], issued_rad=issued,
            previous_rad=actuator['previous_tire_target_rad'], yaw_rate_radps=motion['heading_rate_radps'],
            lateral_mps=motion['reported_lateral_mps'])
    actual_margin = margin(actuator['issued_tire_target_rad'])
    assert actual_margin['reason'] == 'STOPPING_SWEEP_OCCUPIED'
    no_lead = command_steering(d['steer_rad'], actuator['previous_input_rad'], actuator['dt_s'], policy=cfg['steering_policy'])
    candidates = []
    previous_response = d['steering_response']['previous_state']
    for i, p in enumerate(np.array(d['reference_xy_rear_m'])[1:], 1):
        x, y = map(float, p.astype(np.float32)); radius = math.hypot(x, y)
        angle = math.atan(d['nominal_response_length_m']*2*y/(x*x+y*y))
        if x <= 1e-6 or not d['minimum_preview_distance_m'] <= radius <= d['minimum_preview_distance_m']+.5 or abs(angle) > .3:
            continue
        target, _, _ = compensate_steering_response(angle, fault['sim_ns'], SteeringResponseState(**previous_response), policy=cfg['steering_policy'])
        mapped = command_steering(target, actuator['previous_input_rad'], actuator['dt_s'], policy=cfg['steering_policy'])
        candidates.append({'reference_index':i,'radius_m':radius,'nominal_tire_rad':angle,
                           'issued_tire_target_rad':mapped['issued_tire_target_rad'], **margin(mapped['issued_tire_target_rad'])})
    last = [r for r in tracked if r['sim_ns'] >= fault['sim_ns']-5_000_000_000]
    def actuator_error(row: dict) -> float:
        return row['measured_steer_rad']-row['details']['steer_rad']
    summary = {'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        'scope':'First scan rejection and prior commands only; excludes later latched/frozen repeats',
        'first_fault_after_arm_s':(fault['sim_ns']-armed['sim_ns'])/1e9,
        'reasons_through_first_fault':dict(Counter(r['reason'] for r in commands)),
        'selection_counts':dict(Counter(s['kind'] for s in selections)), 'metadata_replay_matched':metadata_matches,
        'legacy_rejection_rescued_times_s':legacy_failures,
        'actual_missing_times_s':[(r['sim_ns']-armed['sim_ns'])/1e9 for r in commands if r['reason']=='STEERING_FEASIBLE_LOOKAHEAD_MISSING'],
        'segment_selection_times_s':[s['time_s'] for s in selections if s['kind']=='segment'],
        'last5s_measured_minus_nominal_tire_rad':distribution([actuator_error(r) for r in last]),
        'last5s_input_rate_limited_count':sum(abs(r['details']['steering_actuator']['issued_input_rad']-r['details']['steering_actuator']['requested_input_rad'])>1e-9 for r in last),
        'fault':{'speed_kmh':fault['speed_mps']*3.6, 'selection':d['lookahead_selection'],
                 'preview_band_m':[d['minimum_preview_distance_m'],d['minimum_preview_distance_m']+.5],
                 'reference_max_radius_m':float(np.linalg.norm(d['reference_xy_rear_m'],axis=1).max()),
                 'nominal_tire_rad':d['steer_rad'],'measured_tire_rad':fault['measured_steer_rad'],
                 'response_target_rad':actuator['issued_tire_target_rad'], 'actual_guard':actual_margin,
                 'without_lead_same_measured_state':margin(no_lead['issued_tire_target_rad']),
                 'hold_previous_same_measured_state':margin(actuator['previous_tire_target_rad']),
                 'admissible_original_target_sensitivity':candidates},
        'sensitivity_boundary':'One-step recalculation using the same scan/pose/yaw/velocity; not a changed-state simulation or permission to bypass a guard'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    (args.output/'fault_command.json').write_text(json.dumps(fault,indent=2,allow_nan=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    times = [(r['sim_ns']-armed['sim_ns'])/1e9 for r in tracked]
    fig, axes = plt.subplots(3,1,figsize=(11,9),sharex=True)
    axes[0].plot(times,[r['speed_mps']*3.6 for r in tracked],label='Measured speed')
    axes[0].axhline(5.,c='black',ls='--',label='Target 5 km/h');axes[0].set_ylabel('Speed (km/h)')
    axes[1].plot(times,[r['details']['steer_rad'] for r in tracked],label='Nominal PP tire request')
    axes[1].plot(times,[r['details']['steering_actuator']['issued_tire_target_rad'] for r in tracked],label='Mapped tire target',alpha=.8)
    axes[1].plot(times,[r['measured_steer_rad'] for r in tracked],label='Measured tire angle',alpha=.8)
    axes[1].set_ylabel('Tire angle (rad)')
    axes[2].plot(times,[r['details']['obstacle_guard']['minimum_ray_margin_m'] for r in tracked],label='Recorded guard margin')
    axes[2].scatter(summary['first_fault_after_arm_s'],actual_margin['minimum_ray_margin_m'],c='red',label='First rejected scan')
    axes[2].axhline(0.,c='black',ls='--');axes[2].set_ylabel('Ray margin (m)');axes[2].set_xlabel('Simulation seconds after arm')
    for ax in axes:ax.grid(alpha=.2);ax.legend(loc='best');ax.axvline(summary['first_fault_after_arm_s'],c='red',ls=':')
    fig.suptitle('AWSIM segment policy: launch succeeds; stopping-sweep rejection ends run')
    fig.tight_layout();fig.savefig(args.output/'tracking_and_margin.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,allow_nan=False))


if __name__ == '__main__':
    main()
