"""Measured S00 waveforms against matched-progress normal teacher, in WSL."""
from pathlib import Path
import hashlib
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import random_pulse_events

root=Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
rawroot=Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
out=root/'location_duration_review'

def load(name):
    raw=rawroot/name;data=(raw/'control.jsonl').read_bytes()
    assert hashlib.sha256(data).hexdigest()==json.loads((raw/'transfer_manifest.json').read_text())['control.jsonl']['sha256']
    return [json.loads(s) for s in data.splitlines()]

normal=load('codex-time-recovery-sites-normal-n03')
normal=[r for r in normal if r.get('publication') and r.get('projection') and 110<r['projection']['s_m']<130
        and r.get('reason')=='RECOVERY_TEACHER_TRACKING']
normal_s=np.array([r['projection']['s_m'] for r in normal]);assert np.all(np.diff(normal_s)>0)
fig,axes=plt.subplots(2,2,figsize=(12,7),sharex='col',constrained_layout=True)
reports=[]
for col,side in enumerate(('left','right')):
    name='codex-time-recovery-sites-g01-'+side;rows=load(name)
    event=next(e for e in random_pulse_events(rows) if e['site_id']=='S00')
    start=event['start_publication_ns']
    samples=[r for r in rows if r.get('publication') and r.get('projection')
             and -1<=(r['publication']['sim_ns']-start)/1e9<=6]
    t=np.array([(r['publication']['sim_ns']-start)/1e9 for r in samples])
    baseline=[min(normal,key=lambda n:abs(n['projection']['s_m']-r['projection']['s_m'])) for r in samples]
    requested=np.array([r['pulse']['requested_rad'] for r in samples])
    nominal_delta=np.array([r['nominal_angle_rad']-n['nominal_angle_rad'] for r,n in zip(samples,baseline)])
    issued_delta=np.array([r['issued_angle_rad']-n['issued_angle_rad'] for r,n in zip(samples,baseline)])
    measured_delta=np.array([r['measured_steering_rad']-n['measured_steering_rad'] for r,n in zip(samples,baseline)])
    lateral=np.array([r['pulse']['lateral_error_m']*100 for r in samples])
    axes[0,col].plot(t,requested,label='Requested additive pulse [ROS rad]',color='#c95132',linestyle='--')
    axes[0,col].plot(t,nominal_delta,label='PP command - normal [ROS rad]',color='#547c43')
    axes[0,col].plot(t,issued_delta,label='Issued command - normal [ROS rad]',color='#227bb6')
    axes[0,col].plot(t,measured_delta,label='Measured steering - normal [rad]',color='#8b43a4')
    axes[1,col].plot(t,lateral,color='#227bb6',label='Measured lateral deviation')
    axes[1,col].axhline(5,color='#888',linestyle=':',linewidth=.8)
    axes[1,col].axhline(-5,color='#888',linestyle=':',linewidth=.8)
    stop_t=[x for x,r in zip(t,samples) if 119<=r['projection']['s_m']<=120]
    for ax in axes[:,col]:
        ax.axvline((event['zero_publication_ns']-start)/1e9,color='#555',linestyle='--',linewidth=.8)
        if stop_t:ax.axvspan(min(stop_t),max(stop_t),color='#e7b755',alpha=.2,label='Known stop progress 119-120 m')
        ax.grid(alpha=.2);ax.set_xlim(-1,6)
    axes[0,col].set_title('S00 '+side.upper());axes[1,col].set_xlabel('Time since disturbance start [s]')
    report=[]
    for target in (0,.25,.5,.75,1.,1.25,1.5,1.75,2.,2.5,3.,4.):
        k=int(np.argmin(abs(t-target)));r=samples[k]
        report.append(dict(t_s=float(t[k]),s_m=r['projection']['s_m'],requested_rad=float(requested[k]),
            pp_minus_normal_rad=float(nominal_delta[k]),issued_minus_normal_rad=float(issued_delta[k]),
            measured_minus_normal_rad=float(measured_delta[k]),lateral_cm=float(lateral[k])))
    reports.append(dict(run_id=name,side=side,samples=report))
axes[0,0].set_ylabel('Control / measured angle [rad]');axes[1,0].set_ylabel('Lateral deviation from normal [cm]')
axes[0,0].legend(fontsize=7,loc='lower left');axes[1,0].legend(fontsize=8,loc='upper right')
fig.suptitle('Measured publication / steering traces; normal comparison is matched by course progress')
fig.savefig(out/'s00_control_and_lateral.png',dpi=150);plt.close(fig)
result=dict(scope='MEASURED_TRACES_WITH_MATCHED_NORMAL_NOT_EXACT_COUNTERFACTUAL',runs=reports)
with (out/'s00_control_samples.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result),flush=True)
