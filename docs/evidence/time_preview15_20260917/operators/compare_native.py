"""Compare measured single-run results, with identical active-interval metrics."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

base=Path('/home/thistle/e2e_autonomous/runs')
root=base/'time_preview15_20260917'
trials=[('Previous adaptive speed, stopping-distance PP',base/'time_curvature_response_20260917',
         'codex-time-curve15-response-lap04')]
for kmh in (15,20):
    trial=base/('time_preview'+str(kmh)+'_20260917')
    if (trial/'evaluation/summary.json').exists():
        trials.append((f'Time-scaled PP, ceiling {kmh} km/h',trial,f'codex-time-timepreview{kmh}-lap01'))
fig,axes=plt.subplots(len(trials),1,figsize=(11,3.2*len(trials)),squeeze=False)
results=[]
for ax,(label,directory,run_id) in zip(axes[:,0],trials):
    raw=directory/'raw'/run_id;summary=json.loads((directory/'evaluation/summary.json').read_bytes())
    data=(raw/'control.jsonl').read_bytes()
    manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
    assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
    rows=[json.loads(line) for line in data.splitlines()]
    start=summary['armed_sim_ns'];end=start+round(summary['active_duration_sim_s']*1e9)
    active=[r for r in rows if r.get('event')=='COMMAND_SENT' and start<=r['sim_ns']<end]
    tracking=[r for r in active if r['reason']=='TIME_PATH_TRACKING']
    speeds=[r['speed_mps']*3.6 for r in active if r.get('speed_mps') is not None and r['speed_mps']>.1]
    limits=Counter(r['details']['longitudinal_preview']['limiting_reason'] for r in tracking)
    result=dict(label=label,source_run=str(raw),status=summary['status'],lap_completed=summary['judge_lap_confirmed'],
        lap_records=summary['judge_laps'],recorded_distance_m=summary['recorded_pose_travel_m'],
        moving_actual_kmh=dict(zip(['min','median','p95','max'],np.percentile(speeds,[0,50,95,100]).tolist())),
        command_reasons=dict(Counter(r['reason'] for r in active)),limiting_reasons=dict(limits),
        scan_would_stop_commands=summary['scan_would_stop_commands'],stop_confirmed=summary['stop_confirmed_before_cleanup'],
        control_replay=summary['control_replay'])
    results.append(result)
    times=[(r['sim_ns']-start)/1e9 for r in active]
    ax.plot(times,[r['speed_mps']*3.6 if r.get('speed_mps') is not None else np.nan for r in active],label='Measured')
    ax.plot(times,[r['target_speed_mps']*3.6 for r in active],alpha=.55,label='Requested',linewidth=.8)
    ax.set(title=label,ylabel='Speed [km/h]',xlabel='Time after authorization [sim s]',ylim=(0,22))
    ax.grid(alpha=.2);ax.legend(loc='upper right')
fig.tight_layout();fig.savefig(root/'evaluation/speed_comparison.png',dpi=150);plt.close(fig)
result=dict(scope='SAME_SCENE_SINGLE_RUN_COMPARISON_NOT_REPEATED_SUCCESS_RATE',
    all_proximity_stop_policies='log_only_awsim_v1',same_checkpoint_sha256='1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a',
    physical_contact_certified=False,trials=results)
(root/'evaluation/comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
