"""Sealed collection inventory and observed S00 comparison, not model evaluation."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import random_pulse_events

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_separated_20260915')
def read(p):return json.loads(p.read_bytes())
def write(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
plan=read(OUT/'selected_site_plan.json');reports=[];missing=[];holds=[]
if (OUT/'effective_data_plan.json').exists():plan=read(OUT/'effective_data_plan.json')
for r in plan['runs']:
    p=OUT/(r['run_id']+'_collection_summary.json')
    if not p.exists():missing.append(r['run_id']);continue
    result=read(p);assert result['run_id']==r['run_id'];reports.append(result)
for p in [OUT/'pilot_gate.json',*sorted(OUT.glob('pair*_holds.json'))]:
    if p.exists():holds.extend(read(p)['events'])
counts=Counter();event_counts=Counter();site_counts=Counter();outward_counts=Counter()
for r in reports:
    counts[r['split']]+=r['accepted'];event_counts[r['split']]+=r['success_events']
    for e in r['events']:
        if e['recovery_confirmed']:site_counts[e['site_id']]+=1
        outward_counts[r['split']]+=len(e.get('outward_anchor_ids',[]))
files=[]
for r in reports:
    for p in sorted((OUT/'prepared'/r['split']/r['run_id']).rglob('*')):
        if p.is_file():files.append(dict(path=str(p.relative_to(OUT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
index=dict(scope='OBSERVED_RECOVERY_COLLECTION_NOT_MODEL_IMPROVEMENT',plan_sha256=hashlib.sha256((OUT/'selected_site_plan.json').read_bytes()).hexdigest(),
    planned_runs=len(plan['runs']),completed_audited_runs=len(reports),pending_runs=missing,
    planned_events=22,observed_events=sum(len(r['events']) for r in reports),successful_events=sum(r['success_events'] for r in reports),
    accepted_anchors_by_split=dict(counts),successful_events_by_split=dict(event_counts),site_counts=dict(site_counts),
    outward_5to25cm_2to4deg_agreeing_both_normal_guides_by_split=dict(outward_counts),
    qualified_one_second_holds=sum(r['one_second_hold_qualified'] for r in holds),hold_assessed_events=len(holds),
    raw_root=str(RAW),prepared_root=str(OUT/'prepared'),teacher_shape=['N',30,2],future_step_s=.1,future_horizon_s=3.,
    runtime_source_commits=sorted({read(RAW/r['run_id']/'result.json')['source_sha'] for r in reports}),
    runs=reports,holds=holds,prepared_files=files,split_unit='complete_run',training_started=False,sealed_test_read=False)
if (OUT/'replacement_amendment.json').exists():
    index['replacement_amendment']=read(OUT/'replacement_amendment.json')
    index['failed_attempts_preserved_excluded_from_training']=[read(OUT/'failed_attempt_diagnosis.json')]
    index['effective_data_plan_sha256']=hashlib.sha256((OUT/'effective_data_plan.json').read_bytes()).hexdigest()
receipts=[read(p) for p in sorted(OUT.glob('separated_*_shipping.json'))]
index['storage_including_preserved_failed_attempt']=dict(
    transferred_run_count=sum(len(r['run_ids']) for r in receipts),
    archive_bytes=sum(r['archive_bytes'] for r in receipts),
    raw_regular_file_bytes=sum(r['source_regular_file_bytes'] for r in receipts),
    prepared_bytes=sum(r['bytes'] for r in files))
write(OUT/'collection_index.json',index)

fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True,constrained_layout=True)
for col,side in enumerate(('left','right')):
    newname='codex-time-recovery-separated-g01-'+side
    if not (OUT/(newname+'_event1_waveform.json')).exists():continue
    new=read(OUT/(newname+'_event1_waveform.json'))
    oldroot=RAW.with_name('time_recovery_sites_20260915')/('codex-time-recovery-sites-g01-'+side)
    data=(oldroot/'control.jsonl').read_bytes();assert hashlib.sha256(data).hexdigest()==read(oldroot/'transfer_manifest.json')['control.jsonl']['sha256']
    oldrows=[json.loads(s) for s in data.splitlines()];event=next(e for e in random_pulse_events(oldrows) if e['site_id']=='S00')
    ref=read(RAW/newname/'reference.json');guide=np.asarray(ref['steering_pulse']['steering_guide'])
    old=[]
    for r in oldrows:
        if not r.get('publication') or not r.get('projection') or r['phase'] not in ('hold','recovery'):continue
        t=(r['publication']['sim_ns']-event['start_publication_ns'])/1e9
        if not 0<=t<=8:continue
        g=float(np.interp(r['projection']['s_m'],guide[:,0],guide[:,1]))
        old.append(dict(t_s=t,guide_input_rad=g,issued_input_rad=r['issued_angle_rad'],lateral_m=r['pulse']['lateral_error_m']))
    for rows,label,color in ((old,'Previous: PP + pulse, start 116 m','#777777'),(new,'New: nominal guide + pulse, start 117 m','#1265b0')):
        rows=[r for r in rows if 0<=r['t_s']<=8]
        axes[0,col].plot([r['t_s'] for r in rows],[r['issued_input_rad']-r['guide_input_rad'] for r in rows],label=label,color=color)
        axes[1,col].plot([r['t_s'] for r in rows],[r['lateral_m']*100 for r in rows],label=label,color=color)
    axes[0,col].set_title('S00 '+side);axes[0,col].legend(fontsize=8)
    for ax in axes[:,col]:ax.axhline(0,color='black',lw=.5);ax.grid(alpha=.25);ax.set_xlim(0,8)
    axes[0,col].axvspan(.25,1.25,color='#1265b0',alpha=.10)
    axes[1,col].set_xlabel('Seconds from first published disturbance phase')
axes[0,0].set_ylabel('Issued minus nominal guide [ROS input rad]')
axes[1,0].set_ylabel('Measured lateral deviation [cm]')
fig.suptitle('Measured teacher collection: different start positions; no learned model comparison')
fig.savefig(OUT/'s00_observed_comparison.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in index.items() if k not in ('runs','holds','prepared_files')}),flush=True)
