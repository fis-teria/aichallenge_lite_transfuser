"""Evaluate collected AWSIM tracking evidence under the WSL worktree lock."""
from pathlib import Path
from collections import Counter
import argparse,json,hashlib,subprocess
import numpy as np
from aic_transfuser_lite.control.long_sim_tracking_v4 import tracking_command

ap=argparse.ArgumentParser();ap.add_argument('root',type=Path);ap.add_argument('output',type=Path)
a=ap.parse_args();a.output.mkdir(exist_ok=False)
def load(name):return json.loads((a.root/name).read_text())
def rows(name):return [json.loads(s) for s in (a.root/name).read_text().splitlines()]
t=rows('tracking.jsonl');s=rows('shadow.jsonl');trace=rows('timing.jsonl')
host=load('host_result.json');probe=load('probe_result.json')
commands=[r for r in t if r['event']=='COMMAND_SENT']
plans={r['output_id']:r for r in s if r.get('event')=='PLAN'}
active=[r for r in commands if r['reason']=='V4_10_TRACKING']
positive=[r for r in active if r['acceleration_mps2']>0]
for r in active:
    p=plans[r['plan_id']];d=r['details']
    expected=tracking_command(p['raw_xy_m'],tuple(p['observation_pose_xyyaw']),tuple(d['current_pose']),d['speed_mps'],expected_points=36)
    assert abs(expected['acceleration_mps2']-r['acceleration_mps2'])<1e-8
    assert expected['prefix_points']==d['prefix_points']
    assert np.allclose(expected['lookahead_rear_m'],d['lookahead_rear_m'])
    assert abs(r['steer_rad'])<=.5 and 0<=r['sim_s']-p['source_s']<=.5
arm=next(r for r in t if r['event']=='ARMED')['sim_s']
vel=sorted([r for r in trace if r['role']=='velocity' and r['stamp_ns']*1e-9>=arm],key=lambda r:r['stamp_ns'])
x=np.array([r['stamp_ns']*1e-9-arm for r in vel]);v=np.array([r['speed_mps'] for r in vel])
distance=float(np.trapz(v,x)) if len(x)>1 else 0.
final=next((r['state'] for r in reversed(t) if r['event']=='CONTROLLER_END'),load('tracking_heartbeat.json'))
below_since=None;independent_stop=None
for time_s,speed in zip(x,v):
    if time_s<10:continue
    if speed<.03:
        if below_since is None:below_since=float(time_s)
        if time_s-below_since>=1. and independent_stop is None:independent_stop=float(time_s)
    else:below_since=None
result=dict(execution_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    checkpoint_identity=next(r['identity'] for r in s if r.get('event')=='MODEL_LOADED'),
    host_error=host.get('error'),probe_fault=probe['fault'],cleanup_errors=host['cleanup'],
    control_sources=probe['command_sources'],host_wall_s=host['wall_s'],
    observation_after_arm_sim_s=float(x[-1]) if len(x) else 0.,
    velocity_integrated_distance_m=distance,maximum_speed_after_arm_mps=float(v.max()) if len(v) else 0.,
    forward_count=len(plans),tracking_command_count=len(active),positive_tracking_count=len(positive),
    tracking_unique_plans=len({r['plan_id'] for r in active}),
    controller_final=final,reasons=dict(Counter(r['reason'] for r in commands)),
    prefix_points_counts=dict(Counter(r['details']['prefix_points'] for r in active)),
    replay_matching_commands=len(active),stop_method=host['stop_method'],
    independent_velocity_stop_confirmed_s=independent_stop,
    control_policy='SIM_ONLY_PP_TARGET_0.25_MPS_CONTIGUOUS_PREFIX_MAX_3M',
    collision_status='NOT_VERIFIED',full_lap=False,
    hashes={n:hashlib.sha256((a.root/n).read_bytes()).hexdigest() for n in
        ('tracking.jsonl','shadow.jsonl','timing.jsonl','host_result.json','probe_result.json')})
(a.output/'summary.json').write_text(json.dumps(result,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(2,1,figsize=(9,7))
axs[0].plot(x,v,label='Observed speed');axs[0].axhline(.25,ls='--',c='grey',label='Trial target')
axs[0].axvline(10,c='red',ls=':',label='Scheduled brake')
axs[0].set(xlabel='Seconds since host authorization [sim s]',ylabel='Speed [m/s]');axs[0].legend();axs[0].grid()
axs[1].plot([r['sim_s']-arm for r in commands],[r['acceleration_mps2'] for r in commands],label='Sent acceleration')
axs[1].set(xlim=(0,max(14,float(x[-1]) if len(x) else 14)),xlabel='Seconds since authorization [sim s]',ylabel='Acceleration request [m/s²]');axs[1].grid()
fig.suptitle('V4-10 predicted-path low-speed AWSIM trial\nVelocity observation and actual controller requests; no collision/full-lap claim')
fig.tight_layout();fig.savefig(a.output/'tracking.png',dpi=150)
print(json.dumps({k:val for k,val in result.items() if k not in ('hashes','checkpoint_identity','controller_final')},indent=2))
print('FINAL',json.dumps({k:val for k,val in final.items() if k!='plan_ids'}))
