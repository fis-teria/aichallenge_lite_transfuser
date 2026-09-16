"""Read-only paired geometry, recorded delay, and training-weight diagnosis."""
from __future__ import annotations
from collections import Counter
from dataclasses import fields
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path('/home/thistle/e2e_autonomous/runs/time_launch_regression_20260916/operators')))
import compare_native as c

OUT = c.ROOT/'supplement'

def read(path: Path):
    c.pin(path)
    return json.loads(path.read_bytes())

def reference(xy: np.ndarray, obs, current):
    return c.prepare_time_reference(c.TimePlan('diagnostic', obs, xy), current,
        rear_axle_offset_m=(c.CONTROL['geometry']['rear_axle_forward_in_base_link_m'], 0.)).xy_current_m

def minimum_angle(points: np.ndarray, speed: float) -> float | None:
    """Continuous-polyline geometric threshold, rad; not float32 PP acceptance.

    Bisection uses the existing algebraic interval checker, whose coefficient
    tolerance remains unchanged. Runtime probe is the authority for acceptance.
    """
    minimum = max(1., .4+max(0.,speed)*.5+speed*speed/2)
    maximum = minimum+1.
    response = c.effective_response_length(max(0.,speed), c.CONTROL['vehicle_model_policy'])
    def any_interval(limit):
        return any(c.segment_intervals(a,b,minimum,maximum,response,limit) for a,b in zip(points,points[1:]))
    if not any_interval(None):
        return None
    lo, hi = 0., math.pi/2-1e-6
    for _ in range(48):
        mid=(lo+hi)/2
        if any_interval(mid): hi=mid
        else: lo=mid
    return hi

def paired_geometry() -> None:
    base=c.ROOT/'baseline'
    anchors=read(base/'anchors.json'); controls=read(base/'controls.json')
    predictions={}
    for name in ['old','epoch1','epoch2','epoch3','teacher']:
        path=base/('teachers.npy' if name=='teacher' else name+'_predictions.npy')
        c.pin(path); predictions[name]=np.load(path,allow_pickle=False)
        assert predictions[name].shape==(92,30,2) and np.isfinite(predictions[name]).all()
    variants={**predictions,
        'old_x_epoch2_y':np.stack((predictions['old'][:,:,0],predictions['epoch2'][:,:,1]),axis=-1),
        'epoch2_x_old_y':np.stack((predictions['epoch2'][:,:,0],predictions['old'][:,:,1]),axis=-1)}
    by_id={r['anchor_id']:r for r in controls if r['age_s']==0.}
    rows=[]
    for i,a in enumerate(anchors):
        obs=c.TimedBodyPose(**a['observation_pose']);v=float(a['ego'][0])
        result={name:c.probe(p[i],obs,obs,v) for name,p in variants.items()}
        for name in predictions:
            assert result[name]==by_id[a['anchor_id']]['models'][name]
        near=-.25<=a['after_teacher_arm_s']<=.5
        geometry={}
        if near:
            for name,p in variants.items():
                points=reference(p[i],obs,obs)
                geometry[name]=dict(min_continuous_abs_angle_rad=minimum_angle(points,v),
                    endpoint_m=p[i,-1].tolist(),probe=result[name],xy_m=p[i].tolist())
        rows.append(dict(anchor_id=a['anchor_id'],run_id=a['run_id'],near_arm=near,
            after_teacher_arm_s=a['after_teacher_arm_s'],speed_mps=v,models=result,geometry=geometry))
    summary={}
    for group,chosen in [('all',rows),('near_arm',[r for r in rows if r['near_arm']]),
                         ('epoch2_failed',[r for r in rows if not r['models']['epoch2']['accepted']])]:
        summary[group]=dict(anchors=len(chosen),runs=len({r['run_id'] for r in chosen}),
            accepted={name:sum(r['models'][name]['accepted'] for r in chosen) for name in variants})
    summary['scope']='COORDINATE_HYBRIDS_ARE_OFFLINE_COUNTERFACTUALS_NOT_VALID_TEACHERS_OR_RUNTIME_PATHS'
    summary['near_arm_by_run']={rid:{name:sum(r['models'][name]['accepted'] for r in rows if r['near_arm'] and r['run_id']==rid)
        for name in variants} for rid in sorted({r['run_id'] for r in rows})}
    c.write(OUT/'paired_geometry_summary.json',summary)
    c.write(OUT/'near_arm_cases.json',[r for r in rows if r['near_arm']])
    selected=next(r for r in rows if r['near_arm'] and r['after_teacher_arm_s']>=0 and not r['models']['epoch2']['accepted'])
    c.write(OUT/'representative_case.json',selected)
    colors={'old':'#557899','epoch1':'#bd8a21','epoch2':'#c84645','epoch3':'#936ab4','teacher':'#218a68'}
    fig,axes=plt.subplots(1,2,figsize=(11.5,4.8),layout='constrained')
    for name in ['teacher','old','epoch1','epoch2']:
        p=np.asarray(selected['geometry'][name]['xy_m'])
        p[:,0]-=c.CONTROL['geometry']['rear_axle_forward_in_base_link_m']
        dense=np.concatenate([a+np.linspace(0,1,40)[:,None]*(b-a) for a,b in zip(p,p[1:])])
        distance=np.linalg.norm(dense,axis=1)
        response=c.effective_response_length(selected['speed_mps'],c.CONTROL['vehicle_model_policy'])
        angle=np.abs(np.arctan2(2*response*dense[:,1],distance**2))
        axes[0].plot(p[:,0],p[:,1],color=colors[name],label=name,linewidth=2)
        axes[1].plot(distance,angle,color=colors[name],label=name,linewidth=2)
    axes[0].scatter([0],[0],color='black',marker='x')
    axes[0].set(xlabel='Forward [m]',ylabel='Left [m]',title='Unmodified predicted paths');axes[0].set_aspect('equal')
    axes[1].axhline(.3,color='black',ls='--',label='Physical tire limit 0.3 rad')
    axes[1].axvspan(1,2,color='green',alpha=.07)
    axes[1].set(xlim=(1,2.1),ylim=(.18,.4),xlabel='Distance from rear axle [m]',ylabel='Required absolute tire angle [rad]',title='Same PP and physical limit')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Paired recorded-state comparison: r30, 0.040 s after teacher arm (offline)')
    fig.savefig(OUT/'paired_geometry.png',dpi=160);plt.close(fig)
    s=read(base/'summary.json')['near_arm_pp']['all']
    fig,ax=plt.subplots(figsize=(8.8,4.8),layout='constrained')
    for n,dy in [('teacher',.10),('old',.05),('epoch1',0.),('epoch2',0.),('epoch3',-.05)]:
        # Small visual offsets separate coincident curves; y tick labels are exact counts.
        ax.plot(c.AGES,[s[str(age)][n]['accepted']+dy for age in c.AGES],marker='o',label=n,color=colors[n])
    ax.set(xlabel='Observation-to-control age with recorded vehicle motion [s]',ylabel='Accepted frames / 14',
        title='Near launch: 14 correlated frames from 2 baseline runs (offline)',ylim=(0,15),yticks=range(0,15,2))
    ax.grid(alpha=.2);ax.legend(ncol=5,loc='lower center')
    fig.text(.5,.01,'Coincident lines offset by at most 0.1 frame for readability; no new driving trial.',ha='center',fontsize=8)
    fig.savefig(OUT/'paired_acceptance.png',dpi=160);plt.close(fig)
    print('PAIRED_GEOMETRY',json.dumps(summary),flush=True)

def failed_trial_delay() -> None:
    root=c.BASE/'runs/time_corner_model_lap_20260916'
    raw=root/'raw/codex-time-corner-lap01'
    manifest={r['path']:r for r in read(raw/'transfer_manifest.json')}
    for name in ['control.jsonl','inference.jsonl','trial_config.json']:
        c.pin(raw/name,manifest[name]['sha256'])
    config=json.loads((raw/'trial_config.json').read_bytes())
    for k in ['geometry','speed_policy','lookahead_policy','vehicle_model_policy']:
        assert config[k]==c.CONTROL[k]
    evaluation=read(root/'evaluation/summary.json')
    commands=[json.loads(x) for x in (raw/'control.jsonl').read_bytes().splitlines()]
    plans={r['plan_id']:r for x in (raw/'inference.jsonl').read_bytes().splitlines() if (r:=json.loads(x))['event']=='PLAN'}
    armed=evaluation['armed_sim_ns'];end=armed+round(evaluation['active_duration_sim_s']*1e9)
    commands=[r for r in commands if r['event']=='COMMAND_SENT' and armed<=r['sim_ns']<end]
    assert len(commands)==103
    rows=[]
    for r in commands:
        obs=c.TimedBodyPose(**r['details']['observation_pose']);current=c.TimedBodyPose(**r['details']['current_pose'])
        xy=np.asarray(plans[r['plan_id']]['raw_xy_m'],float)
        actual=c.probe(xy,obs,current,r['speed_mps']);zero=c.probe(xy,obs,obs,r['speed_mps'])
        assert actual['accepted']==(r['reason']=='TIME_PATH_TRACKING')
        if not actual['accepted']:assert actual['reason']==r['reason']
        rows.append(dict(after_arm_s=(r['sim_ns']-armed)/1e9,plan_id=r['plan_id'],
            actual_age_s=(current.stamp_ns-obs.stamp_ns)/1e9,actual=actual,
            age_zero_current_speed=zero))
    summary=dict(commands=len(rows),unique_plans=len({r['plan_id'] for r in rows}),
        actual_accepted=sum(r['actual']['accepted'] for r in rows),
        age_zero_current_speed_accepted=sum(r['age_zero_current_speed']['accepted'] for r in rows),
        failed_actual_recovered_age_zero=sum(not r['actual']['accepted'] and r['age_zero_current_speed']['accepted'] for r in rows),
        age_zero_failure_reasons=dict(Counter(r['age_zero_current_speed'].get('failure_geometry',r['age_zero_current_speed']['reason']) for r in rows if not r['age_zero_current_speed']['accepted'])),
        scope='REMOVE_PLAN_AGE_AND_POSE_CHANGE_HOLD_LOGGED_CURRENT_SPEED_FIXED_NOT_ZERO_LATENCY_NEW_DRIVE')
    c.write(OUT/'failed_trial_delay.json',summary);c.write(OUT/'failed_trial_delay_commands.json',rows)
    print('FAILED_TRIAL_DELAY',json.dumps(summary),flush=True)

def training_and_waiting() -> None:
    old=read(c.BASE/'runs/time_recovery_multiscale_20260916/training/result.json')
    new=read(c.TRAIN/'training_serial/result.json')
    inv=read(c.ROOT/'nominal/inventory.json')
    launch=sum(r['groups'].get('stationary_future_launch',0) for r in inv['splits']['train'])
    assert launch==152
    summary=dict(stationary_future_launch_train_frames=launch,independent_nominal_train_runs=12,
        old_presentations_per_epoch=old['anchors_visited']/old['epochs_completed'],
        new_presentations_per_epoch=new['anchors_visited']/new['epochs_completed'],
        optimizer_updates_old=old['optimizer_steps'],optimizer_updates_new=new['optimizer_steps'],
        nominal_per_epoch=36726,nominal_geometry_auxiliary_loss=False,
        selection=c.PLAN['selection'],selection_run_ids=c.PLAN['selection_run_ids'],
        initial_selection_metric_m=old['best_validation_run_macro_3s_m'],
        training_history=read(c.TRAIN/'training_serial/history.json'),
        scope='SAMPLING_AND_SELECTION_AUDIT_NOT_TRAINING_ABLATION')
    for prefix in ['old','new']:
        total=summary[prefix+'_presentations_per_epoch']
        summary[prefix+'_nominal_fraction']=36726/total
        summary[prefix+'_launch_frame_fraction']=launch/total
    ds=c.TimeTrainingCacheDataset(c.CACHE,'validation',verify_hashes=False)
    anchor_rows=read(c.ROOT/'nominal/anchors.json');indices={x:i for i,x in enumerate(ds.anchor_ids)}
    comparisons=[]
    for rid in ['5kmh_run03','5kmh_run06','8kmh_run06','8kmh_run09']:
        rows=[r for r in anchor_rows if r['run_id']==rid]
        a=next(r for r in rows if r['group']=='stationary_wait')
        b=next(r for r in reversed(rows) if r['group']=='stationary_future_launch')
        first=ds[indices[a['anchor_id']]].inputs;second=ds[indices[b['anchor_id']]].inputs
        assert first is not None and second is not None
        difference={}
        for field in fields(first):
            x,y=getattr(first,field.name),getattr(second,field.name)
            if isinstance(x,torch.Tensor):
                delta=(x.to(torch.float64)-y.to(torch.float64)).abs()
                difference[field.name]=dict(shape=list(x.shape),exact_equal=bool(torch.equal(x,y)),
                    mean_abs_delta=float(delta.mean()) if x.numel() else 0.,
                    max_abs_delta=float(delta.max()) if x.numel() else 0.)
        comparisons.append(dict(run_id=rid,waiting_anchor=a,launch_anchor=b,tensor_differences=difference))
    c.write(OUT/'training_audit.json',summary);c.write(OUT/'waiting_input_pairs.json',comparisons)
    print('TRAINING_AUDIT',json.dumps({k:v for k,v in summary.items() if k not in ('training_history','selection_run_ids')}),flush=True)
    print('WAITING_PAIRS',json.dumps([{k:r[k] for k in ['run_id','tensor_differences']} for r in comparisons]),flush=True)

def main() -> None:
    assert not subprocess.check_output(['git','status','--porcelain']).strip()
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='1903cb955671ac05e68259acb9f313010de1e279'
    torch.set_num_threads(4)
    OUT.mkdir(exist_ok=False)
    straight=np.column_stack((np.linspace(0,3,31),np.zeros(31)))
    assert minimum_angle(straight,0.)<1e-8
    assert minimum_angle(straight*.01,0.) is None
    paired_geometry();failed_trial_delay();training_and_waiting()
    c.write(OUT/'provenance.json',dict(status='PASS',git_commit='1903cb955671ac05e68259acb9f313010de1e279',
        sources=c.SOURCES,optimizer_steps=0,awsim_executions=0,sealed_test_read=False,
        paired_age_zero_matches_saved_controls=True,failed_trial_matches_recorded_acceptance=True,
        continuous_geometry_smoke_checks=2))

if __name__=='__main__':main()
