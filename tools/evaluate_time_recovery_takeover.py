"""Native WSL evaluation of closed AWSIM recovery trials, without new inference."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.data.time_recovery_collection_v1 import project_course
from aic_transfuser_lite.data.time_steering_pulse_v1 import nominal_recovery_errors
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin
from aic_transfuser_lite.evaluation.time_recovery_trial_v1 import recovery_window_metrics
from aic_transfuser_lite.runtime.time_recovery_takeover_v1 import validate_recovery_reference
from evaluate_time_awsim_trial import replay_recorded_control


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(run: Path, reference_root: Path) -> tuple[dict[str, Any], np.ndarray]:
    host = json.loads((run/'host_result.json').read_text())
    if host['scope'] != 'ONE_LAP_AFTER_TEACHER_BOOTSTRAP' or not host['official_start_requested']:
        raise ValueError('AUTHORIZED_RECOVERY_RUN_REQUIRED')
    config = json.loads((run/'trial_config.json').read_text())
    validate_trial_config(config)
    if sha(run/'trial_config.json') != host['trial_config_sha256']:
        raise ValueError('RECOVERY_CONFIG_HASH')
    reference_file = reference_root/(host['recovery_side']+'.json')
    if sha(reference_file) != host['recovery_reference_sha256']:
        raise ValueError('RECOVERY_REFERENCE_HASH')
    pulse, baseline, guide = validate_recovery_reference(json.loads(reference_file.read_text()))
    records = [json.loads(s) for s in (run/'control.jsonl').read_text().splitlines()]
    inference = [json.loads(s) for s in (run/'inference.jsonl').read_text().splitlines()]
    plans = [p for p in inference if p.get('event') == 'PLAN']
    if {p['checkpoint_sha256'] for p in plans} != {config['checkpoint_sha256']}:
        raise ValueError('RECOVERY_CHECKPOINT_IDENTITY')
    by_id = {p['plan_id']: p for p in plans}
    armed = [r for r in records if r['event'] == 'ARMED']
    publishers = [r for r in records if r['event'] == 'PUBLISHER_CREATED']
    if len(armed) != 1 or len(publishers) != 1:
        raise ValueError('RECOVERY_AUTHORITY_JOURNAL')
    pub = publishers[0]
    if pub['trial_config_sha256'] != host['trial_config_sha256']:
        raise ValueError('RECOVERY_PUBLISHER_CONFIG_IDENTITY')
    commands = [r for r in records if r['event'] == 'COMMAND_SENT' and r['sim_ns'] >= armed[0]['sim_ns']]
    taken = [r for r in records if r['event'] == 'RECOVERY_E2E_TAKEOVER']
    result = dict(run_id=host['run_id'], side=host['recovery_side'], checkpoint_sha256=config['checkpoint_sha256'],
        host_status=host['status'], host_error=host.get('error'), judge_lap_confirmed=host['judge_lap_confirmed'],
        judge_sections=[r['next'] for r in host.get('judge_section_events', [])],
        rviz_path_subscribers=host.get('rviz_path_subscribers'), cleanup_errors=host['cleanup_errors'],
        requested_stop_reason=host['last_control'].get('requested_stop_reason'),
        takeover_count=len(taken), all_command_reasons=dict(Counter(r['reason'] for r in commands)),
        source_sha256={f: sha(run/f) for f in ('host_result.json','control.jsonl','inference.jsonl','trial_config.json')})
    if not taken:
        result.update(status='RECOVERY_NOT_MEASURED_NO_TAKEOVER')
        return result, np.empty((0, 4))
    if len(taken) != 1:
        raise ValueError('RECOVERY_MULTIPLE_TAKEOVERS')
    event = taken[0]; start = event['sim_ns']
    zero = event['recovery_state']['pulse']['zero_ns']
    if not zero + 150_000_000 <= event['recovery']['observation_ns'] <= start <= zero + 1_000_000_000:
        raise ValueError('RECOVERY_TAKEOVER_CAUSAL_BOUNDARY')
    published_zero = [r for r in commands if r['monotonic_ns'] < event['monotonic_ns']
                      and r['reason'] == 'RECOVERY_TEACHER_BOOTSTRAP'
                      and r['recovery_state']['pulse']['zero_ns'] == zero
                      and r['recovery']['perturbation_rad'] == 0.]
    if not published_zero:
        raise ValueError('RECOVERY_UNPUBLISHED_ZERO')
    after = [r for r in commands if r['monotonic_ns'] >= event['monotonic_ns']]
    if any(r['control_owner'] != 'E2E' or r['reason'] == 'RECOVERY_TEACHER_BOOTSTRAP'
           or r['recovery_state']['takeover_ns'] != start for r in after):
        raise ValueError('RECOVERY_TEACHER_FALLBACK_OR_LOST_OWNERSHIP')
    # Exclude scheduled stop/freeze tails; retain rejected E2E decisions before stop.
    end = min([r['monotonic_ns'] for r in records if r['event']=='STOP_REQUESTED']
              or [after[-1]['monotonic_ns']+1])
    active = [r for r in after if r['monotonic_ns'] < end]
    replay = replay_recorded_control(active, plans, config['geometry']['rear_axle_forward_in_base_link_m'],
        **{k:config[k] for k in ('speed_policy','obstacle_policy','steering_policy','lookahead_policy','vehicle_model_policy')})
    measurements = {}
    takeover_pose_stamp = event['recovery']['current_pose']['stamp_ns']
    for command in active:
        trace = command.get('recovery')
        if not trace or trace['lateral_error_m'] is None or command['speed_mps'] is None:
            continue
        pose = trace['current_pose']
        projection = project_course(baseline, [pose['x_m'], pose['y_m']], pose['yaw_rad'])
        lateral, heading = nominal_recovery_errors(guide, s_m=projection['s_m'],
                                                  offset_m=projection['offset_m'], yaw_rad=pose['yaw_rad'])
        if abs(lateral-trace['lateral_error_m']) > 1e-9 or abs(heading-trace['heading_error_rad']) > 1e-9:
            raise ValueError('RECOVERY_MEASUREMENT_REPLAY')
        t = (pose['stamp_ns']-takeover_pose_stamp)/1e9
        if 0. <= t <= 10.:
            measurements[pose['stamp_ns']] = [t, lateral, heading, command['speed_mps']]
    series = np.array([v for _,v in sorted(measurements.items())]).reshape(-1,4)
    metrics = recovery_window_metrics(series) if len(series) else None
    initial = event['recovery']
    result.update(status='RECOVERED_10S' if metrics and metrics['recovered_in_ten_second_window'] else 'RECOVERY_NOT_CONFIRMED',
        takeover=initial, takeover_sim_ns=start, published_zero_sim_ns=zero,
        takeover_after_zero_s=(start-zero)/1e9, pulse_release_reason=event['recovery_state']['pulse']['reason'],
        initial_outward=bool(initial['lateral_error_m']*pulse.amplitude_rad > 0.
                             and initial['heading_error_rad']*pulse.amplitude_rad > 0.),
        metrics=metrics, control_replay=replay, e2e_command_count=len(active),
        e2e_command_reasons=dict(Counter(r['reason'] for r in active)),
        e2e_duration_sim_s=(active[-1]['sim_ns']-start)/1e9,
        no_teacher_fallback_verified=True,
        median_moving_speed_kmh=float(np.median([r['speed_mps']*3.6 for r in active if r['speed_mps'] is not None and r['speed_mps']>.5]))
            if any(r['speed_mps'] is not None and r['speed_mps']>.5 for r in active) else None)
    faults = [r for r in records if r['event']=='SCAN_GUARD_REJECTED' and r['monotonic_ns']>=event['monotonic_ns']]
    if faults:
        fault=faults[0]; motion=fault['motion_observation']
        margin=scan_margin(fault['scan'],fault['scan_in_current_rear'],speed_mps=fault['speed_mps'],
            measured_rad=fault['measured_steer_rad'],issued_rad=fault['issued_steer_rad'],
            previous_rad=fault['previous_steer_rad'],yaw_rate_radps=motion['heading_rate_radps'],
            lateral_mps=motion['reported_lateral_mps'])
        if margin['reason'] != fault['reason']:
            raise ValueError('RECOVERY_SCAN_REJECTION_REPLAY')
        result['first_scan_rejection'] = dict(after_takeover_s=(fault['sim_ns']-start)/1e9, **margin)
    # Initial model path in map coordinates; the guide is used only for diagnosis.
    first = next(r for r in active if r.get('plan_id') == event['plan_id'])
    observed=first['details']['observation_pose']; path=np.array(by_id[event['plan_id']]['raw_xy_m'])
    c,s=math.cos(observed['yaw_rad']),math.sin(observed['yaw_rad'])
    world=path @ np.array([[c,s],[-s,c]]) + [observed['x_m'], observed['y_m']]
    predicted=[]
    for i in (4,9,14,19,29):
        try:
            projection=project_course(baseline,world[i],observed['yaw_rad'])
            lateral,_=nominal_recovery_errors(guide,s_m=projection['s_m'],offset_m=projection['offset_m'],yaw_rad=observed['yaw_rad'])
            predicted.append(dict(horizon_s=(i+1)*.1,lateral_to_nominal_m=lateral,progress_m=projection['s_m']))
        except ValueError as exc:
            predicted.append(dict(horizon_s=(i+1)*.1,unavailable=str(exc)))
    result['initial_predicted_lateral'] = predicted
    return result, series


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,action='append',required=True)
    ap.add_argument('--reference-root',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    results=[];traces=[]
    for run in args.run:
        result,series=evaluate(run,args.reference_root);results.append(result);traces.append(series)
    summary=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        scope='ONE_PAIRED_TRIAL_PER_SIDE_MEASURED_SIM_RECOVERY_NOT_GENERALIZATION',
        teacher_bootstrap_excluded=True,raw_predictions_modified=False,runs=results)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    (args.output/'traces.json').write_text(json.dumps({r['run_id']:x.tolist() for r,x in zip(results,traces)},allow_nan=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    for result,x in zip(results,traces):
        if not len(x):continue
        col=0 if result['side']=='left' else 1
        label='Expanded' if '-candidate-' in result['run_id'] else 'Before'
        for row,values in enumerate((x[:,1]*100,np.rad2deg(x[:,2]),x[:,3]*3.6)):
            axes[row,col].plot(x[:,0],values,label=label)
    for col,side in enumerate(('Left perturbation','Right perturbation')):
        axes[0,col].set_title(side)
        for row,(label,threshold) in enumerate((('Lateral vs nominal [cm]',5.),('Heading vs nominal [deg]',2.),('Measured speed [km/h]',None))):
            ax=axes[row,col];ax.set_ylabel(label);ax.grid(alpha=.25)
            if threshold:
                ax.axhline(threshold,color='gray',ls='--',lw=.7);ax.axhline(-threshold,color='gray',ls='--',lw=.7)
            if ax.get_legend_handles_labels()[0]:ax.legend()
        axes[2,col].set_xlabel('Time since model takeover [sim s]')
    fig.suptitle('AWSIM recovery: fixed target 5 km/h, unchanged PP / safety guard')
    fig.tight_layout();fig.savefig(args.output/'recovery_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({r['run_id']:r['status'] for r in results}))


if __name__=='__main__':
    main()
