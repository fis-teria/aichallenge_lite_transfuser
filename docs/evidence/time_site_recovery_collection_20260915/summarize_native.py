"""Publish a small collection index and measured recovery plots in native WSL."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
plan = json.loads((OUT/'selected_site_plan.json').read_text())
pairs = [json.loads((OUT/f'pair{i:02d}_audit.json').read_text()) for i in range(1, 5)]
runs = [r for p in pairs for r in p['runs']]
assert len(runs) == 8 and len({r['run_id'] for r in runs}) == 8
sites = sorted([plan['stop_site'], *plan['additional_sites']], key=lambda s:s['start_s_m'])
assert len(sites) == 11 and sum(s['site_id']=='S00' for s in sites) == 1
events = [dict(e, run_id=r['run_id'], split=r['split']) for r in runs for e in r['events']]
split_counts = {split:dict(runs=sum(r['split']==split for r in runs),
                         events=sum(e['split']==split for e in events),
                         anchors=sum(r['accepted'] for r in runs if r['split']==split))
                for split in ('train','validation')}
summary = dict(scope='MEASURED_RECOVERY_DATA_NOT_MODEL_PERFORMANCE', plan_sha256=hashlib.sha256((OUT/'selected_site_plan.json').read_bytes()).hexdigest(),
    site_selection_seed=plan['seed'], sites=sites, independent_normal_runs=2, independent_recovery_runs=8,
    planned_events=22, emitted_events=len(events), successful_events=sum(e['recovery_confirmed'] for e in events),
    skipped_events=sum(len(r['skipped_sites']) for r in runs),
    total_anchors=sum(r['accepted'] for r in runs), split_counts=split_counts,
    strict_outward_anchors=sum(len(e.get('outward_anchor_ids',[])) for e in events),
    all_22_events_meet_60_anchor_gate=len(events)==22 and all(r['all_sites_have_at_least_60_anchors'] for r in runs),
    events=events, run_summaries=[{k:v for k,v in r.items() if k not in ('events','prepared')} for r in runs],
    native_raw_root=str(RAW), native_prepared_root=str(OUT/'prepared'), model_training_started=False, sealed_test_read=False)
with (OUT/'collection_index.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
figures=OUT/'figures';figures.mkdir(exist_ok=False)
normal=[json.loads(line) for line in (RAW/plan['normal_runs'][0]/'control.jsonl').read_text().splitlines()]
normal=[r for r in normal if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('current_pose')]
xy=np.asarray([[r['current_pose']['x_m'],r['current_pose']['y_m']] for r in normal]);origin=xy.min(axis=0)
fig,ax=plt.subplots(figsize=(9,8),constrained_layout=True)
ax.plot(*(xy-origin).T,color='#77818d',linewidth=2,label='Current measured normal teacher')
for s in sites:
 p=np.asarray(s['map_xy_m'])-origin;stop=s['site_id']=='S00'
 ax.scatter(*p,marker='*' if stop else 'o',s=180 if stop else 60,color='#b42635' if stop else '#1265ae',zorder=5)
 ax.annotate(f"{s['site_id']}  s={s['start_s_m']:g} m",p,xytext=(6,7),textcoords='offset points',fontsize=9,
             bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=1.))
ax.set(title='Fixed stop site + 10 seeded random recovery sites',xlabel='Map x from local origin [m]',ylabel='Map y from local origin [m]')
ax.set_aspect('equal');ax.margins(.14);ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
fig.savefig(figures/'collected_sites.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(3,4,figsize=(15,10),sharex=True,sharey=True,constrained_layout=True)
traces={r['run_id']:json.loads((OUT/(r['run_id']+'_state_audit.json')).read_text())['control_states']
        for r in runs if (OUT/(r['run_id']+'_state_audit.json')).exists()}
for ax,s in zip(axes.flat,sites):
 for e in [e for e in events if e['site_id']==s['site_id']]:
  rows=[r for r in traces.get(e['run_id'],[]) if r['recovery_event_id']==e['event_id']]
  if rows:
   ax.plot([r['seconds_after_release'] for r in rows],[r['left_m'] for r in rows],
           color='#ba3d6b' if e['sign']>0 else '#187aa0',label='Left pulse' if e['sign']>0 else 'Right pulse')
 ax.axhline(.05,color='#999999',linestyle='--',linewidth=.7);ax.axhline(-.05,color='#999999',linestyle='--',linewidth=.7)
 ax.axvline(0.,color='#999999',linestyle=':',linewidth=.7)
 ax.set_title(s['site_id']+('  stop site' if s['site_id']=='S00' else ''),fontsize=11);ax.grid(alpha=.2)
 ax.set_xlim(-2.5,10.);ax.set_ylim(-.3,.3)
axes.flat[-1].axis('off');axes.flat[0].legend(fontsize=8)
fig.supxlabel('Time from published disturbance release [s]');fig.supylabel('Lateral error from measured normal guide [m]')
fig.suptitle('Observed recovery after steering disturbance; no learned model used',fontsize=14)
fig.savefig(figures/'recovery_lateral_errors.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k not in ('sites','events','run_summaries')}),flush=True)
