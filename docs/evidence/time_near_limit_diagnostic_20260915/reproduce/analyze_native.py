"""One held-constant AWSIM diagnostic, replayed only in native WSL."""
from pathlib import Path
from collections import Counter
import json, math, sys
import numpy as np
sys.path.insert(0,str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import measured_reference, verify_files, compare
from compare_time_corner_tracking import runtime_poses
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import angle_delta
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin

root=Path('/home/thistle/e2e_autonomous/runs/time_near_limit_diagnostic_20260915')
normal=Path('/home/thistle/e2e_autonomous/runs/time_objective_driving_comparison_20260915_r2')
rid='codex-time-near-limit-01';raw=root/'raw'/rid
out=root/'diagnosis';out.mkdir(exist_ok=False)
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(x) for x in p.read_text().splitlines()]
def stats(v):
    if not v:return None
    a=np.asarray(v,float);assert np.isfinite(a).all()
    return dict(count=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()),min=float(a.min()))
def replay(f,profile):
    motion=f['motion_observation']
    return scan_margin(f['scan'],f['scan_in_current_rear'],speed_mps=f['speed_mps'],
        measured_rad=f['measured_steer_rad'],issued_rad=f['issued_steer_rad'],previous_rad=f['previous_steer_rad'],
        yaw_rate_radps=motion['heading_rate_radps'],lateral_mps=motion['reported_lateral_mps'],clearance_profile=profile)

assert read(root/'transfer_verification.json')['status']=='PASS'
summary=read(root/'evaluation/summary.json');host=read(raw/'host_result.json')
assert summary['control_replay']['status']=='PASS'
hashes=verify_files(raw,('control.jsonl','inference.jsonl','vehicle_observations.jsonl','trial_config.json','host_result.json'))
cfg=read(raw/'trial_config.json');assert cfg==read(Path('configs/control/time_path_near_limit_lap_20260915.json'))
cs=rows(raw/'control.jsonl');ps=rows(raw/'inference.jsonl')
arm=next(r for r in cs if r['event']=='ARMED')['sim_ns']
faults=[r for r in cs if r['event']=='SCAN_GUARD_REJECTED']
end=faults[0]['sim_ns'] if faults else arm+round(summary['active_duration_sim_s']*1e9)
active=[r for r in cs if r['event']=='COMMAND_SENT' and arm<=r['sim_ns']<=end and r.get('details',{}).get('current_pose')]
assert active
poses={r['details']['current_pose']['stamp_ns']:r['details']['current_pose'] for r in active}
poses=[p for _,p in sorted(poses.items())];xy=np.array([[p['x_m'],p['y_m']] for p in poses]);last=poses[-1]
result=dict(scope='ONE_AWSIM_NEAR_LIMIT_DIAGNOSTIC_NOT_STANDARD_PASS',source_hashes=hashes,
    status=summary['status'],host_error=summary['host_error'],judge_laps=summary['judge_laps'],judge_sections=summary['judge_sections'],
    stop_confirmed=summary['stop_confirmed_before_cleanup'],time_to_first_guard_or_end_s=(end-arm)/1e9,
    pose_travel_before_guard_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
    final_pose=last,checkpoint_sha256=summary['checkpoint_sha256'],control_replay=summary['control_replay'],
    measured_speed_kmh=stats([r['speed_mps']*3.6 for r in active if r['sim_ns']>=arm+3_000_000_000]),
    standard_stop_states={},references={},sealed_test_read=False)
for run in ('codex-time-obj-a-01','codex-time-obj-b-03','codex-time-obj-b-04','codex-time-obj-a-06'):
    path=normal/run/'raw'/run
    verify_files(path,('control.jsonl',))
    fault=next(r for r in rows(path/'control.jsonl') if r['event']=='SCAN_GUARD_REJECTED')
    values={p:replay(fault,p) for p in ('standard_v1','awsim_near_limit_v1')}
    assert values['standard_v1']['reason']==fault['reason']
    result['standard_stop_states'][run]=values
if faults:
    result['diagnostic_stop_state']={p:replay(faults[0],p) for p in ('standard_v1','awsim_near_limit_v1')}
    assert result['diagnostic_stop_state']['awsim_near_limit_v1']['reason']==faults[0]['reason']

plans={p['plan_id']:p for p in ps if p['event']=='PLAN'};actual=runtime_poses(raw/'vehicle_observations.jsonl')
refroot=Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914')
references={};details={}
for n in (30,31):
    name=f'codex-time-recovery-speedbase-r{n}';ref,meta=measured_reference(refroot/name,cfg);references[name]=ref
    cr,pr=compare(ref,[r for r in active if r['sim_ns']>=arm+55_000_000_000],plans,actual,cfg,arm,end)
    valid=[p for p in pr if p['status']=='EVALUATED'];tail=[p for p in valid if p['time_s']>=(end-arm)/1e9-10.]
    following=[p['following']['1'] for p in tail if p['following']['1']['status']=='EVALUATED']
    try:
        q=ref.project(np.array([last['x_m'],last['y_m']]),yaw_hint_rad=last['yaw_rad'])
        final=dict(status='EVALUATED',left_m=q.left_m,heading_deg=math.degrees(angle_delta(last['yaw_rad'],q.body_yaw_rad)))
    except ValueError as exc:final=dict(status=str(exc))
    result['references'][name]=dict(scope='Measured normal guide, progress 55 to 142 m, not road-center truth',source=meta,
        final_projection=final,plan_status_counts=dict(Counter(p['status'] for p in pr)),
        last10s_1s_following_abs_lateral_m=stats([abs(p['following_left_m']) for p in following]),
        last10s_1s_prediction_abs_lateral_m=stats([abs(p['prediction_left_m']) for p in following]),
        last_prediction=dict(time_s=valid[-1]['time_s'],anchor_left_m=valid[-1]['anchor_left_m'],horizons=valid[-1]['horizons']) if valid else None)
    details[name]=dict(commands=cr,plans=pr)
for name,data in [('summary.json',result),('attribution_details.json',details)]:
    with (out/name).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
origin=np.array([poses[0]['x_m'],poses[0]['y_m']]);fig,axes=plt.subplots(1,2,figsize=(13,6))
for r in references.values():
    for ax in axes:ax.plot(r.xy[:,0]-origin[0],r.xy[:,1]-origin[1],color='#969696',lw=2,alpha=.65)
for old in ('codex-time-obj-a-01','codex-time-obj-a-06'):
    cmds=rows(normal/old/'raw'/old/'control.jsonl')
    oldxy=np.array([[r['details']['current_pose'][k] for k in ('x_m','y_m')] for r in cmds if r['event']=='COMMAND_SENT' and r.get('details',{}).get('current_pose')])
    for ax in axes:ax.plot(oldxy[:,0]-origin[0],oldxy[:,1]-origin[1],'--',color='#3485b4',alpha=.65,label=old)
for ax in axes:
    ax.plot(xy[:,0]-origin[0],xy[:,1]-origin[1],color='#d15548',label='A: smaller monitor reserve')
    ax.plot(xy[-1,0]-origin[0],xy[-1,1]-origin[1],'x',color='#d15548',ms=10)
    ax.set(xlabel='Map X from start [m]',ylabel='Map Y from start [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
center=xy[-1]-origin;axes[1].set_xlim(center[0]-7,center[0]+7);axes[1].set_ylim(center[1]-7,center[1]+7)
axes[0].set_title('Same A checkpoint, target 5 km/h');axes[1].set_title('Near first guard; gray = measured normal guide')
axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'near_limit_comparison.png',dpi=150);plt.close(fig)
print(json.dumps({k:result[k] for k in ('status','host_error','time_to_first_guard_or_end_s','pose_travel_before_guard_m','diagnostic_stop_state','references') if k in result}))
