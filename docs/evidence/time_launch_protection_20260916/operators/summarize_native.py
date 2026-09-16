"""Summarize completed fixed-budget experiments; do not change the frozen gate."""
from __future__ import annotations
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_protection_v2_20260916')
out=root/'summary'
out.mkdir(exist_ok=False)
used={}
def read(p):
    used[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    return json.loads(p.read_bytes())
def write(name,value):
    (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

driver=read(root/'driver_status.json');assert driver['status']=='COMPLETE'
proof=read(root/'preparation/proof.json');selection=read(root/'evaluation_final/selection.json')
existing=read(root/'evaluation_existing/selection.json')
assert selection['source_commit']==proof['source_commit']==driver['source_commit']
reports={r['candidate_id']:read(root/'evaluation_final'/(r['candidate_id']+'.json')) for r in selection['decisions']}
oracle=read(root/'teacher_oracle_audit.json')
assert oracle['status']=='PASS' and oracle['selection']['decisions'][-1]['eligible']
calibration_runs={r['run_id'] for r in reports['initial']['launch'] if r['origin']=='awsim_calibration'}
cache=Path('/home/thistle/e2e_autonomous')/proof['plan']['cache']
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset
train=TimeTrainingCacheDataset(cache,'train',verify_hashes=False)
assert not calibration_runs & set(train.run_ids)
assert not {r['run_id'] for r in reports['initial']['launch']} & set(train.run_ids)
tables=[]
margin_details=[]
for decision in selection['decisions']:
    r=reports[decision['candidate_id']]
    initial_cases={c['case_id']:c for c in reports['initial']['launch']}
    for c in r['launch']:
        if not c['teacher_accepted'] or not c['accepted']:continue
        old=initial_cases[c['case_id']]
        reference=.3-abs(c['teacher_steer_rad'])
        if old['accepted']:reference=min(reference,.3-abs(old['steer_rad']))
        margin=.3-abs(c['steer_rad'])
        deficit=reference-margin-selection['policy']['launch_margin_tolerance_rad']
        if deficit>0:margin_details.append(dict(candidate_id=r['candidate_id'],case_id=c['case_id'],
            origin=c['origin'],age_s=c['age_s'],steer_rad=c['steer_rad'],teacher_steer_rad=c['teacher_steer_rad'],
            initial_steer_rad=old['steer_rad'],reference_margin_rad=reference,
            selected_margin_rad=margin,excess_margin_loss_rad=deficit))
    launch={}
    for origin in sorted({c['origin'] for c in r['launch']}):
        launch[origin]={}
        for age in proof['plan']['launch_ages_s']:
            cases=[c for c in r['launch'] if c['origin']==origin and c['age_s']==age and c['teacher_accepted']]
            launch[origin][str(age)]=dict(supported=len(cases),accepted=sum(c['accepted'] for c in cases),
                rejected=sum(not c['accepted'] for c in cases))
    tables.append(dict(candidate_id=r['candidate_id'],checkpoint_sha256=r['checkpoint_sha256'],xy=r['xy'],
        launch=launch,eligible=decision['eligible'],score=decision['score'],
        reason_counts=dict(Counter(reason.split(':')[0]+':'+reason.split(':')[1] for reason in decision['reasons']))))
arms={}
for arm in ['protocol_control','launch_balanced']:
    result=read(root/arm/'result.json');verification=read(root/arm/'verification.json')
    assert result['optimizer_steps']==verification['optimizer_steps']==proof['plan']['optimizer_steps_per_arm']
    assert result['anchors_visited']==proof['plan']['presentations_per_epoch']*proof['plan']['training']['epochs']
    assert result['reload_predictions_exact']
    arms[arm]=dict(result=result,verification=verification,history=read(root/arm/'history.json'))
assert arms['protocol_control']['result']['initial_weights_sha256']==arms['launch_balanced']['result']['initial_weights_sha256']
assert read(root/'protocol_control/initial_validation.json')==read(root/'launch_balanced/initial_validation.json')
write('comparison.json',dict(status='PASS',candidates=tables,selection=selection,
    same_initial_validation_exact=True,same_optimizer_step_budget=True,same_initial_weight_digest=True,
    training_elapsed_s={a:v['result']['training_elapsed_seconds'] for a,v in arms.items()},
    training_counts={a:dict(visited=sum(h['train_counts']['visited'] for h in v['history']),
        input_invalid=sum(h['train_counts']['input_invalid'] for h in v['history']),
        supported=sum(h['train_counts']['supported'] for h in v['history'])) for a,v in arms.items()},
    original_known_regression_rejected=not next(d for d in existing['decisions'] if d['candidate_id']=='previous_epoch2')['eligible'],
    launch_evaluation_runs_disjoint_from_training=True,oracle_gate_consistent=True,
    scope='SAME_DATA_DIFFERENT_PRESENTATION_ALLOCATION_OFFLINE_DEVELOPMENT_NOT_NEW_DRIVING'))
write('margin_regressions.json',dict(tolerance_rad=selection['policy']['launch_margin_tolerance_rad'],
    meaning='selected PP steering margin, not measured tracking or collision clearance',cases=margin_details))
write('preparation_summary.json',dict(source_commit=proof['source_commit'],proof_sha256=used[str(root/'preparation/proof.json')],
    cache_sha256=proof['cache_sha256'],population_sha256=proof['population_sha256'],plan=proof['plan'],
    runs=proof['runs'],samplers=proof['samplers'],
    train_exclusion_reasons=dict(Counter(r['reason'] for r in proof['train']['excluded'])),
    validation_exclusion_reasons=dict(Counter(r['reason'] for r in proof['validation']['excluded'])),
    validation_unsupported_active_anchors=proof['validation']['unsupported_active_anchors'],
    launch_missing_future_pose=proof['launch_missing_future_pose'],baseline_samples=proof['baseline_samples'],
    raw_sources=proof['raw_sources'],all_original_files_preserved=True,
    launch_calibration_runs=sorted(calibration_runs),launch_evaluation_runs_disjoint_from_training=True))

names=[r['candidate_id'] for r in tables]
fig,axes=plt.subplots(1,2,figsize=(12,5.6),layout='constrained')
for axis,origin,title in zip(axes,['nominal_validation','awsim_calibration'],['Nominal Ready-state launch','AWSIM calibration, after teacher arm']):
    counts=np.array([[t['launch'][origin][str(age)]['accepted'] for age in proof['plan']['launch_ages_s']] for t in tables])
    totals=np.array([[t['launch'][origin][str(age)]['supported'] for age in proof['plan']['launch_ages_s']] for t in tables])
    axis.imshow(counts/totals,vmin=0,vmax=1,cmap='RdYlGn',aspect='auto')
    axis.set(xticks=range(6),xticklabels=proof['plan']['launch_ages_s'],yticks=range(len(names)),
        yticklabels=[n.replace('protocol_control','control').replace('launch_balanced','launch 5%') for n in names],
        xlabel='Recorded observation-to-control age [s]',title=title)
    for i in range(len(names)):
        for j in range(6):axis.text(j,i,f'{counts[i,j]}/{totals[i,j]}',ha='center',va='center',fontsize=8,
            color='white' if counts[i,j]/totals[i,j]>=.95 else 'black')
fig.suptitle('PP-accepted correlated frames / teacher-supported frames; offline only')
fig.savefig(out/'launch_acceptance.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(11.5,4.4),layout='constrained')
for ax,stage,title in zip(axes,['nominal','recovery'],['Nominal driving (launch excluded)','Recovery validation']):
    initial=reports['initial']['xy'][stage]['ade_m']
    for arm,color in [('protocol_control','#4c78a8'),('launch_balanced','#e88c32')]:
        values=[initial,*[reports[f'{arm}_epoch{i}']['xy'][stage]['ade_m'] for i in (1,2,3)]]
        ax.plot(range(4),np.asarray(values)*100,marker='o',color=color,label=arm)
    ceiling=initial+max(selection['policy']['ade_tolerance_m'],initial*selection['policy']['relative_error_tolerance'])
    ax.axhline(ceiling*100,color='#ad3333',ls='--',label='Frozen ADE non-regression ceiling')
    ax.set(xlabel='Completed epochs',ylabel='Run-equal ADE [cm]',title=title,xticks=range(4))
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('Same initial weights, 5,682 updates per arm; ADE is one part of the gate')
fig.savefig(out/'stage_errors.png',dpi=160);plt.close(fig)
write('provenance.json',dict(status='PASS',source_git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    inputs=used,optimizer_steps=0,new_awsim_runs=0,test_read=False))
print(json.dumps(dict(status='SUMMARY_COMPLETE',selected=selection['selected_candidate_id'],
    runtime_test_allowed=selection['runtime_test_allowed'],candidates=tables)),flush=True)
