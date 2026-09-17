"""Replay old controls and compare new policies on unchanged recorded states."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.time_preview_v1 import time_preview_distance
from tools.evaluate_time_awsim_trial import replay_recorded_control

base=Path('/home/thistle/e2e_autonomous/runs')
oldroot=base/'time_curvature_response_20260917'
raw=oldroot/'raw/codex-time-curve15-response-lap04'
out=base/'time_preview15_20260917/offline_preview';out.mkdir(exist_ok=False)
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
sources={}
def load(name: str) -> bytes:
    data=(raw/name).read_bytes();digest=hashlib.sha256(data).hexdigest()
    assert digest==manifest[name]['sha256'] and len(data)==manifest[name]['bytes']
    sources[name]=digest
    return data
rows=[json.loads(line) for line in load('control.jsonl').splitlines()]
plans=[json.loads(line) for line in load('inference.jsonl').splitlines()]
plans=[p for p in plans if 'raw_xy_m' in p]
by_id={p['plan_id']:p for p in plans}
config=json.loads(load('trial_config.json'))
summary=json.loads((oldroot/'evaluation/summary.json').read_bytes())
start=summary['armed_sim_ns'];end=start+round(summary['active_duration_sim_s']*1e9)
commands=[r for r in rows if r.get('event')=='COMMAND_SENT' and start<=r['sim_ns']<end]
offset=config['geometry']['rear_axle_forward_in_base_link_m']
options={k:config[k] for k in ('speed_policy','obstacle_policy','steering_policy','lookahead_policy',
    'vehicle_model_policy','stopping_distance_policy','scan_occupancy_policy')}
replay=replay_recorded_control(commands,plans,offset,**options)
assert replay['status']=='PASS' and replay['matched_commands']==3098
results=[]
for kmh in (15,20):
    counts=Counter();reasons=Counter();deltas=[];targets=[];steer_delta=[]
    for r in commands:
        d=r['details'];p=by_id[r['plan_id']]
        try:
            result=time_trial_control(TimePlan(r['plan_id'],TimedBodyPose(**d['observation_pose']),np.asarray(p['raw_xy_m'])),
                TimedBodyPose(**d['current_pose']),speed_mps=r['speed_mps'],rear_axle_offset_m=(offset,0.),
                speed_policy=f'curvature_time_preview_{kmh}kmh_v1',vehicle_model_policy=f'awsim_understeer_{kmh}kmh_trial_v1',
                lookahead_policy='velocity_time_preview_v1')
        except ValueError as exc:
            counts[str(exc)]+=1
        else:
            counts['ADMITTED']+=1
            reasons[result['longitudinal_preview']['limiting_reason']]+=1
            targets.append(result['target_speed_mps']*3.6)
            if r['reason']=='TIME_PATH_TRACKING':
                deltas.append((result['target_speed_mps']-r['target_speed_mps'])*3.6)
                steer_delta.append(abs(result['steer_rad']-d['steer_rad']))
    results.append(dict(ceiling_kmh=kmh,calculation_counts=dict(counts),limiting_reasons=dict(reasons),
        target_kmh_percentiles=np.percentile(targets,[0,50,95,100]).tolist(),
        target_delta_kmh_percentiles=np.percentile(deltas,[0,50,95,100]).tolist(),
        physical_tire_delta_abs_rad_percentiles=np.percentile(steer_delta,[0,50,95,100]).tolist()))
record=dict(scope='UNCHANGED_RECORDED_INPUT_COMPARISON_NOT_NEW_DRIVING_RESULT',sources=sources,
    old_control_replay=replay,trials=results)
(out/'comparison.json').write_text(json.dumps(record,indent=2)+'\n')
speeds=np.linspace(0,21/3.6,150)
fig,ax=plt.subplots(figsize=(9,5))
ax.plot(speeds*3.6,3*speeds,label='Ideal straight constant-speed 3 s prediction')
ax.plot(speeds*3.6,np.maximum(1.,.4+.5*speeds+speeds**2/2)+.5+.3*speeds,label='Old speed-planning coverage')
ax.plot(speeds*3.6,[time_preview_distance(v)+.5+.3*v for v in speeds],label='New time-preview coverage')
ax.set(xlabel='Measured speed [km/h]',ylabel='Distance [m]',title='Coverage scaling only; model output and driving performance not assumed')
ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'coverage_scaling.png',dpi=150);plt.close(fig)
print(json.dumps(record))
