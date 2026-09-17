"""Compare actual recorded trials; all metrics use the same active interval."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

base=Path('/home/thistle/e2e_autonomous/runs')
out=base/'time_curvature_launch_20260917/evaluation'
trials=[('Fixed 10 km/h','time_launch_10kmh_stop1m_20260917','codex-time-launch10-stop1m-lap01'),
        ('Fixed 15 km/h','time_launch_15kmh_stop1m_20260917','codex-time-launch15-stop1m-lap01'),
        ('Adaptive, ceiling 15 km/h','time_curvature_launch_20260917','codex-time-curve15-lap02')]
results=[];fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=False)
new_active=[]
for ax,(label,name,run_id) in zip(axes,trials):
    root=base/name;raw=root/'raw'/run_id
    summary=json.loads((root/'evaluation/summary.json').read_bytes())
    data=(raw/'control.jsonl').read_bytes()
    manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
    assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
    commands=[json.loads(line) for line in data.splitlines()]
    start=summary['armed_sim_ns'];end=start+round(summary['active_duration_sim_s']*1e9)
    active=[r for r in commands if r.get('event')=='COMMAND_SENT' and start<=r['sim_ns']<end]
    tracking=[r for r in active if r['reason']=='TIME_PATH_TRACKING']
    speeds=[r['speed_mps']*3.6 for r in active if r['speed_mps'] is not None and r['speed_mps']>.1]
    guards=[r for r in commands if r.get('event')=='SCAN_GUARD_REJECTED']
    margins=[r['details']['obstacle_guard']['minimum_ray_margin_m'] for r in tracking]
    reasons=Counter(r['reason'] for r in active)
    preview=[r['details']['longitudinal_preview'] for r in tracking if 'longitudinal_preview' in r['details']]
    result=dict(label=label,source_run=str(raw),lap_completed=summary['judge_lap_confirmed'],
        lap_records=summary['judge_laps'],recorded_distance_m=summary['recorded_pose_travel_m'],
        moving_actual_kmh=dict(zip(['min','median','p95','max'],np.percentile(speeds,[0,50,95,100]).tolist())),
        active_commands=len(active),tracking_commands=len(tracking),reason_counts=dict(reasons),
        lookahead_rejection_commands=reasons['STEERING_FEASIBLE_LOOKAHEAD_MISSING'],
        lookahead_rejection_fraction=reasons['STEERING_FEASIBLE_LOOKAHEAD_MISSING']/len(active),
        distinct_scan_rejections=len(guards),minimum_admitted_ray_margin_m=min(margins),
        margin_scope='Scan-to-padded-monitor ray margin, NOT physical wall clearance',
        stop_confirmed=summary['stop_confirmed_before_cleanup'],
        preview_limits=dict(Counter(p['limiting_reason'] for p in preview)),
        normal_acceleration_range_mps2=[min(r['acceleration_mps2'] for r in tracking),max(r['acceleration_mps2'] for r in tracking)],
        normal_braking_commands=sum(r['acceleration_mps2']<0 for r in tracking),
        control_replay=summary['control_replay'],control_sha256=hashlib.sha256(data).hexdigest())
    if preview:
        assert len(preview)==len(tracking)
        assert result['normal_acceleration_range_mps2'][1]<=1.+1e-9
        assert all(r['acceleration_mps2']<=.8+1e-9 for r in tracking if r['speed_mps']>=1.)
        assert all(r['details']['selected_lookahead_distance_m']>=r['details']['minimum_preview_distance_m'] for r in tracking)
        result['normal_target_kmh_percentiles']=np.percentile([p['target_speed_mps']*3.6 for p in preview],[0,10,50,90,100]).tolist()
        new_active=active
    results.append(result)
    times=[(r['sim_ns']-start)/1e9 for r in active]
    ax.plot(times,[r['speed_mps']*3.6 if r['speed_mps'] is not None else np.nan for r in active],label='Measured speed',color='#176aa2')
    ax.plot(times,[r['target_speed_mps']*3.6 for r in active],label='Requested speed',color='#dc9341',alpha=.55,linewidth=.8)
    ax.set(title=label,ylabel='Speed [km/h]',xlabel='Time after authorization [sim s]',ylim=(0,16))
    ax.legend(loc='upper right');ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'speed_comparison.png',dpi=150);plt.close(fig)
record=dict(scope='THREE_SINGLE_SAME_SCENE_RUNS_NOT_REPEATED_SUCCESS_RATE',trials=results,
    same_checkpoint_sha256='1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a',
    same_stopping_distance_policy='awsim_cap_1m_diagnostic_v1',
    contact_certified=False,contact_evidence_boundary='Judge lap and scan monitor only; no independent physical-contact sensor contract')
(out/'comparison.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
