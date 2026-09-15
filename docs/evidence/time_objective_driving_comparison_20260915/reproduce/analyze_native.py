"""Read-only comparison of six verified runs; execute under the native WSL lock."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import sys
import numpy as np

sys.path.insert(0, str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import measured_reference, verify_files, compare, group_metrics
from compare_time_corner_tracking import runtime_poses
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import angle_delta
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin

plan=json.loads(Path('configs/control/time_objective_driving_comparison_20260915.json').read_text())
root=Path(plan['native_root']);out=root/'comparison';out.mkdir(exist_ok=False)
base=json.loads(Path(plan['models']['A']['config']).read_text())
refroot=Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914')
references={}
for num in (30,31):
    rid=f'codex-time-recovery-speedbase-r{num}'
    references[rid]=measured_reference(refroot/rid,base)


def read(path: Path):
    return json.loads(path.read_text())


def rows(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def write(path: Path, data) -> None:
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)


def stats(values) -> dict | None:
    if not len(values):return None
    a=np.array(values,float);assert np.isfinite(a).all()
    return dict(zip(['min','median','p95','max'],np.quantile(a,[0,.5,.95,1]).tolist()))


results=[];trajectories=[];reference_details={}
for t in plan['trials']:
    rid=t['run_id'];m=plan['models'][t['arm']];native=root/rid;raw=native/'raw'/rid
    transfer=read(native/'transfer_verification.json');assert transfer['status']=='PASS'
    summary=read(native/'evaluation/summary.json');host=read(raw/'host_result.json')
    hashes=verify_files(raw,('control.jsonl','inference.jsonl','vehicle_observations.jsonl','trial_config.json','host_result.json'))
    cfg=read(raw/'trial_config.json');assert cfg==json.loads(Path(m['config']).read_text())
    assert summary['checkpoint_sha256']==[m['sha256']] and summary['control_replay']['status']=='PASS'
    cs=rows(raw/'control.jsonl');ps=rows(raw/'inference.jsonl')
    arm=next(r for r in cs if r['event']=='ARMED')['sim_ns']
    commands=[r for r in cs if r['event']=='COMMAND_SENT' and r['sim_ns']>=arm]
    faults=[r for r in cs if r['event']=='SCAN_GUARD_REJECTED']
    end=faults[0]['sim_ns'] if faults else arm+round(summary['active_duration_sim_s']*1e9)
    active=[r for r in commands if r['sim_ns']<=end and r.get('details',{}).get('current_pose')]
    assert active
    poses={r['details']['current_pose']['stamp_ns']:r['details']['current_pose'] for r in active}
    poses=[p for _,p in sorted(poses.items())]
    xy=np.array([[p['x_m'],p['y_m']] for p in poses]);assert xy.shape[1]==2 and np.isfinite(xy).all()
    speed=[r['speed_mps']*3.6 for r in active if r['sim_ns']>=arm+3_000_000_000]
    first=poses[0];last=poses[-1]
    item=dict(**t,model=m['name'],status=summary['status'],judge_sections=summary['judge_sections'],
        judge_laps=summary['judge_laps'],host_error=summary['host_error'],stop_confirmed=summary['stop_confirmed_before_cleanup'],
        first_guard_or_last_command_after_arm_s=(end-arm)/1e9,pose_travel_before_guard_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        measured_speed_kmh=stats(speed),plan_age_s=stats([(r['sim_ns']-r['details']['observation_pose']['stamp_ns'])/1e9 for r in active if r['details'].get('observation_pose')]),
        initial_pose=first,final_pose=last,checkpoint_sha256=m['sha256'],source_hashes=hashes,
        control_replay=summary['control_replay'],active_command_reasons=summary['active_command_reasons'],
        positive_acceleration_commands=summary['positive_acceleration_commands'],requested_stop_reason=summary['requested_stop_reason'],
        rviz_path_subscribers=host.get('rviz_path_subscribers'),
        cleanup_errors=host['cleanup_errors'],historical_environment_preserved=read(native/'post_environment.json')['preserved'])
    assert not item['cleanup_errors'] and all(item['historical_environment_preserved'].values())
    if faults:
        fault=faults[0];motion=fault['motion_observation']
        margin=scan_margin(fault['scan'],fault['scan_in_current_rear'],speed_mps=fault['speed_mps'],
            measured_rad=fault['measured_steer_rad'],issued_rad=fault['issued_steer_rad'],previous_rad=fault['previous_steer_rad'],
            yaw_rate_radps=motion['heading_rate_radps'],lateral_mps=motion['reported_lateral_mps'])
        assert margin['reason']==fault['reason'];item['first_guard_replay']=margin
    chosen=[r for r in active if r['sim_ns']>=arm+55_000_000_000]
    plans={p['plan_id']:p for p in ps if p['event']=='PLAN'};pose_index=runtime_poses(raw/'vehicle_observations.jsonl')
    item['references']={};reference_details[rid]={}
    for name,(reference,meta) in references.items():
        cr,pr=compare(reference,chosen,plans,pose_index,cfg,arm,end)
        valid=[r for r in cr if r['status']=='EVALUATED'];pv=[r for r in pr if r['status']=='EVALUATED']
        try:
            fp=reference.project(np.array([last['x_m'],last['y_m']]),yaw_hint_rad=last['yaw_rad'])
            final_projection=dict(status='EVALUATED',left_m=fp.left_m,
                heading_deg=math.degrees(angle_delta(last['yaw_rad'],fp.body_yaw_rad)),local_reference_arc_m=fp.progress_m)
        except ValueError as exc:
            final_projection=dict(status=str(exc),left_m=None,heading_deg=None,local_reference_arc_m=None)
        tail=[p for p in pv if p['time_s']>=(end-arm)/1e9-10.]
        following=[p['following']['1'] for p in tail if p['following']['1']['status']=='EVALUATED']
        large=[p for p in tail if abs(p['anchor_left_m'])>=.2]
        snapshot=dict(final_projection=final_projection,command_status_counts=dict(Counter(r['status'] for r in cr)),
            plan_status_counts=dict(Counter(r['status'] for r in pr)),
            last10s_1s_following_abs_lateral_m=stats([abs(f['following_left_m']) for f in following]),
            last10s_1s_prediction_abs_lateral_m=stats([abs(f['prediction_left_m']) for f in following]),
            last10s_large_offset_plans=len(large),last10s_large_offset_3s_reduction_count=sum(abs(p['horizons']['3']['left_m'])<abs(p['anchor_left_m']) for p in large),
            groups=group_metrics(cr,pr))
        if pv:
            p=pv[-1];snapshot['last_prediction_within_reference']=dict(time_s=p['time_s'],left_m=p['anchor_left_m'],horizons=p['horizons'])
        item['references'][name]=snapshot
        reference_details[rid][name]=dict(commands=cr,plans=pr)
    results.append(item);trajectories.append((t,xy))
    print(json.dumps({k:item[k] for k in ['run_id','status','first_guard_or_last_command_after_arm_s','pose_travel_before_guard_m','host_error']}),flush=True)

initials=np.array([[r['initial_pose']['x_m'],r['initial_pose']['y_m']] for r in results])
initial_distance=np.linalg.norm(initials[:,None,:]-initials[None,:,:],axis=2)
output=dict(scope='SIX_FIXED_CONTROL_NORMAL_START_TRIALS_NOT_GENERAL_SUCCESS_RATE',plan=plan,
    nominal_reference_scope='Measured normal guide for course progress 55 to 142 m, not road-center ground truth',
    references={k:meta for k,(_,meta) in references.items()},initial_max_pairwise_distance_m=float(initial_distance.max()),
    results=results,sealed_test_read=False,new_training=False,new_teacher_collection=False)
write(out/'comparison.json',output);write(out/'attribution_details.json',reference_details)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
colors={'A':'#697580','B':'#1789b1','D':'#c44684'}
fig,axes=plt.subplots(1,2,figsize=(13,6));origin=initials[0]
for t,xy in trajectories:
    n=t['index'];arm=t['arm'];line='-' if n<=3 else '--'
    axes[0].plot(xy[:,0]-origin[0],xy[:,1]-origin[1],line,color=colors[arm],label=f'{arm} trial {n}',alpha=.85)
    axes[0].plot(xy[-1,0]-origin[0],xy[-1,1]-origin[1],'x',color=colors[arm])
for reference,_ in references.values():axes[0].plot(reference.xy[:,0]-origin[0],reference.xy[:,1]-origin[1],color='#bbbdc0',lw=2,alpha=.65)
axes[0].set(xlabel='Map X from initial pose [m]',ylabel='Map Y from initial pose [m]',title='Recorded travel; x = last pre-fault or completed pose')
axes[0].set_aspect('equal');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
axes[1].bar(range(6),[r['pose_travel_before_guard_m'] for r in results],color=[colors[r['arm']] for r in results])
axes[1].set_xticks(range(6),[r['arm']+'-'+str(r['index']) for r in results]);axes[1].set(ylabel='Recorded travel before stop [m]',title='One lap is the pass condition')
for i,r in enumerate(results):axes[1].text(i,r['pose_travel_before_guard_m']+1,r['status'].replace('LAP_NOT_COMPLETED','No lap').replace('LAP_COMPLETED','Lap'),ha='center',fontsize=8)
axes[1].grid(axis='y',alpha=.2);fig.suptitle('Fixed target 5 km/h, identical PP and monitor settings; 2 trials per model')
fig.tight_layout();fig.savefig(out/'driving_comparison.png',dpi=150);plt.close(fig)
print(json.dumps(dict(status='COMPLETE',output=str(out),initial_max_pairwise_distance_m=output['initial_max_pairwise_distance_m'])))
