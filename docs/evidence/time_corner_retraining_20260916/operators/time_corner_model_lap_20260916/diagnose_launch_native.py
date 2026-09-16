"""Audit recorded launch geometry without another driving trial or model tuning."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import math
import struct

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

from aic_transfuser_lite.control.polyline_lookahead_v1 import segment_intervals
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length

root = Path('/home/thistle/e2e_autonomous/runs/time_corner_model_lap_20260916')
run = root/'raw/codex-time-corner-lap01'
out = root/'launch_diagnosis'
out.mkdir(exist_ok=True)
assert {p.name for p in out.iterdir()} <= {'summary.json','commands.json','launch_geometry.png',
    'normal_rviz.png','rviz_drive_002.png','rviz_after_freeze.png','image_conversion.json'}
manifest = {r['path']:r for r in json.loads((run/'transfer_manifest.json').read_bytes())}
used = {}

def verified(name: str) -> bytes:
    value = (run/name).read_bytes()
    digest = hashlib.sha256(value).hexdigest()
    assert manifest[name]['bytes'] == len(value) and manifest[name]['sha256'] == digest, name
    used[name] = digest
    return value

def quantiles(values: list[float]) -> dict[str, float]:
    assert values and np.isfinite(values).all()
    return dict(zip(['min','median','max'],map(float,np.quantile(values,[0,.5,1]))))

config = json.loads(verified('trial_config.json'))
rows = [json.loads(line) for line in verified('control.jsonl').splitlines()]
plans = {r['plan_id']:r for line in verified('inference.jsonl').splitlines()
         if (r:=json.loads(line))['event']=='PLAN'}
evaluation = json.loads((root/'evaluation/summary.json').read_bytes())
assert evaluation['control_replay']['status']=='PASS'
assert evaluation['control_replay']['unavailable_commands']==0
armed = evaluation['armed_sim_ns']
end = armed+round(evaluation['active_duration_sim_s']*1e9)
commands = [r for r in rows if r['event']=='COMMAND_SENT' and armed<=r['sim_ns']<end]
assert len(commands)==evaluation['active_commands']
diagnostics=[]
paths=[]
for command in commands:
    d=command['details']
    plan=plans[command['plan_id']]
    raw=np.asarray(plan['raw_xy_m'],dtype=float)
    assert raw.shape==(30,2) and np.isfinite(raw).all()
    observed=TimedBodyPose(**d['observation_pose'])
    current=TimedBodyPose(**d['current_pose'])
    reference=prepare_time_reference(TimePlan(plan['plan_id'],observed,raw),current,
        rear_axle_offset_m=(config['geometry']['rear_axle_forward_in_base_link_m'],0.))
    points=reference.xy_current_m
    speed=command['speed_mps']
    minimum=max(1.,.4+max(0.,speed)*.5+speed*speed/2)
    maximum=minimum+1.
    response=effective_response_length(max(0.,speed),config['vehicle_model_policy'])
    radial=sum(len(segment_intervals(a,b,minimum,maximum,response,None))
               for a,b in zip(points,points[1:]))
    feasible=sum(len(segment_intervals(a,b,minimum,maximum,response,.3))
                 for a,b in zip(points,points[1:]))
    vertex_angles=[abs(math.atan(2*response*float(y)/float(x*x+y*y))) for x,y in points
                   if x>1e-6 and minimum<=math.hypot(x,y)<=maximum]
    diagnostics.append(dict(after_arm_s=(command['sim_ns']-armed)/1e9,reason=command['reason'],
        plan_id=plan['plan_id'],plan_age_s=reference.age_sec,search_band_m=[minimum,maximum],
        response_length_m=response,
        radial_segment_intervals=radial,steering_feasible_segment_intervals=feasible,
        min_vertex_required_abs_tire_rad=min(vertex_angles) if vertex_angles else None,
        endpoint_rear_m=points[-1].tolist(),current_pose=d['current_pose']))
    paths.append(points)
rejected=[r for r in diagnostics if r['reason']=='STEERING_FEASIBLE_LOOKAHEAD_MISSING']
assert len(rejected)==97 and all(r['radial_segment_intervals']>0 for r in rejected)
assert all(r['steering_feasible_segment_intervals']==0 for r in rejected)
assert not any(r['event']=='SCAN_GUARD_REJECTED' for r in rows)
summary=dict(status='PASS',scope='RECORDED_LAUNCH_PP_GEOMETRY_NOT_TEACHER_ERROR_OR_NEW_CLOSED_LOOP',
    active_commands=len(commands),reasons=dict(Counter(r['reason'] for r in commands)),
    rejected_with_radial_candidates=len(rejected),rejected_without_feasible_segments=len(rejected),
    physical_tire_limit_rad=.3,search_band_m=[1.,2.],
    rejected_min_vertex_angle_rad=quantiles([r['min_vertex_required_abs_tire_rad'] for r in rejected]),
    angle_statistic_scope='minimum over existing vertices, not continuous optimum',
    rejected_plan_age_s=quantiles([r['plan_age_s'] for r in rejected]),
    first_rejection=rejected[0],
    stopping_sweep_rejections=0,control_replay=evaluation['control_replay'],
    startup_predicted_geometry_is_direct_failure=True,
    retraining_data_or_loss_causal_attribution_established=False,
    source_sha256=used)
(out/'summary.json').write_text(json.dumps(summary,indent=2))
(out/'commands.json').write_text(json.dumps(diagnostics,indent=2))

fig,axes=plt.subplots(1,2,figsize=(11.8,4.7))
for axis,label in zip(axes,['Raw age-aligned prediction at launch','Physical tire angle required by PP']):
    axis.set_title(label);axis.grid(alpha=.25)
for i in range(0,len(paths),max(1,len(paths)//14)):
    points=paths[i]
    color='#b24439' if diagnostics[i]['reason']!='TIME_PATH_TRACKING' else '#2877aa'
    axes[0].plot(points[:,0],points[:,1],color=color,alpha=.35)
    distance=np.linalg.norm(points,axis=1)
    angles=np.arctan2(2*diagnostics[i]['response_length_m']*points[:,1],np.maximum(distance*distance,1e-6))
    axes[1].plot(distance,np.abs(angles),color=color,alpha=.35)
axes[0].scatter([0],[0],marker='x',color='black',label='Current rear axle')
axes[0].set(xlabel='Forward [m]',ylabel='Left [m]');axes[0].set_aspect('equal');axes[0].legend()
axes[1].axhline(.3,color='black',linestyle='--',label='Physical tire limit 0.3 rad')
axes[1].axvspan(1,2,color='#92b984',alpha=.15,label='Extended lookahead band [1,2] m')
axes[1].set(xlim=(.75,2.05),ylim=(.25,.5),xlabel='Distance from rear axle [m]',ylabel='Absolute required tire angle [rad]')
axes[1].legend(fontsize=8)
fig.suptitle('Launch failure: 97 / 103 commands had no steering-feasible target')
fig.tight_layout();fig.savefig(out/'launch_geometry.png',dpi=160);plt.close(fig)
images=[]
for name in ['normal_rviz.xwd','rviz_drive_002.xwd','rviz_after_freeze.xwd']:
    data=verified(name)
    h=struct.unpack('>25I',data[:100])
    assert h[1:4]==(7,2,24) and h[6:8]==(0,0) and h[11] in (24,32)
    assert h[13] in (4,5) and h[14:17]==(0xFF0000,0xFF00,0xFF)
    if h[13]==5:
        for i in range(h[19]):
            pixel,red,green,blue,flags,_=struct.unpack('>IHHHBB',data[h[0]+i*12:h[0]+(i+1)*12])
            for mask,shift,color,flag in [(0xFF0000,16,red,1),(0xFF00,8,green,2),(0xFF,0,blue,4)]:
                if flags&flag:assert color==((pixel&mask)>>shift)*257
    pixels=data[h[0]+h[19]*12:]
    assert len(pixels)==h[12]*h[5] and h[12]>=h[4]*(h[11]//8)
    destination=out/(Path(name).stem+'.png')
    Image.frombytes('RGB',(h[4],h[5]),pixels,'raw','BGR' if h[11]==24 else 'BGRX',h[12],1).save(destination)
    images.append(dict(source=name,source_sha256=used[name],output=destination.name,
        output_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
        size=[h[4],h[5]],bits_per_pixel=h[11],generated=False,resized=False))
(out/'image_conversion.json').write_text(json.dumps(images,indent=2))
summary['source_sha256']=used
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary))
