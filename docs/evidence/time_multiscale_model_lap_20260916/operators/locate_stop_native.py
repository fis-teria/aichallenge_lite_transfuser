"""Locate the recorded monitor trigger without rerunning AWSIM; native WSL."""
from pathlib import Path
import hashlib
import json
import math
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle
from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.vehicle_motion_v1 import stopping_motion

root=Path('/home/thistle/e2e_autonomous/runs/time_multiscale_model_lap_20260916')
run=root/'raw/codex-time-multiscale-lap01'
out=root/'stop_location';out.mkdir(exist_ok=True)
assert not any(out.iterdir())
source_files={}

def verified(run, name):
    manifest=json.loads((run/'transfer_manifest.json').read_text())
    if isinstance(manifest,list):manifest={r['path']:r for r in manifest}
    data=(run/name).read_bytes()
    assert len(data)==manifest[name]['bytes'] and hashlib.sha256(data).hexdigest()==manifest[name]['sha256'],name
    source_files[str(run/name)]=manifest[name]
    return data

def records(run, name):
    return [json.loads(line) for line in verified(run,name).splitlines()]

control=records(run,'control.jsonl')
guard=next(r for r in control if r['event']=='SCAN_GUARD_REJECTED')
armed=next(r['sim_ns'] for r in control if r['event']=='ARMED')
commands=[r for r in control if r['event']=='COMMAND_SENT' and armed<=r['sim_ns']<=guard['sim_ns']
          and r.get('details',{}).get('current_pose')]
command=commands[-1];details=command['details'];pose=details['current_pose']
assert command['reason']=='STOPPING_SWEEP_OCCUPIED'
by_stamp={r['details']['current_pose']['stamp_ns']:r['details']['current_pose'] for r in commands}
xy=np.array([[p['x_m'],p['y_m']] for _,p in sorted(by_stamp.items())])
origin=xy[0];end=np.array([pose['x_m'],pose['y_m']])
assert np.linalg.norm(end-xy[-1])<1e-8

old=root.parent/'time_random_model_lap_20260915/raw/codex-time-random-lap01'
old_commands=records(old,'control.jsonl')
old_stop=next(r['details']['current_pose'] for r in old_commands if r['event']=='COMMAND_SENT'
    and r['reason']=='STOPPING_SWEEP_OCCUPIED' and r.get('details',{}).get('current_pose'))
old_xy=np.array([old_stop['x_m'],old_stop['y_m']])

# This is a measured completed teacher trajectory, not a surveyed road centerline.
teacher=Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30')
result=json.loads(verified(teacher,'result.json'))
cfg=json.loads(verified(run,'trial_config.json'))
assert result['status']=='COMPLETE_LAP' and result['fixed_target_mps']==5/3.6
assert result['simulator_assets']=={'AWSIM_Data/level1':cfg['geometry']['scene_sha256'],**cfg['steering_asset_sha256']}
teacher_rows=[r for r in records(teacher,'control.jsonl') if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('current_pose')]
assert teacher_rows and all(r['phase']=='baseline' for r in teacher_rows)
teacher_xy=np.array([[r['current_pose']['x_m'],r['current_pose']['y_m']] for r in teacher_rows])

motion=guard['motion_observation'];v=guard['speed_mps']
model=stopping_motion(v,guard['measured_steer_rad'],guard['issued_steer_rad'],guard['previous_steer_rad'],
    policy='awsim_understeer_v1',heading_rate_radps=motion['heading_rate_radps'],reported_lateral_mps=motion['reported_lateral_mps'])
sensor=np.array(guard['scan_in_current_rear']);scan=guard['scan']
n,h,meta=curvature_support_envelope(*model['curvature_interval_per_m'],.4+.5*v+v*v/2,
    scan['angle_increment'],sensor[:2],lateral_padding_m=model['lateral_displacement_bound_m'])
ranges=np.asarray(scan['ranges'],float)
angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']+sensor[2]
valid=np.isfinite(ranges)&(ranges>=scan['range_min'])&(ranges<=scan['range_max'])
points=sensor[:2]+ranges[valid,None]*np.column_stack([np.cos(angles[valid]),np.sin(angles[valid])])
inside=np.all(n@points.T<=h[:,None]+1e-9,axis=0)
hits=points[inside];assert len(hits)==1
vertices=np.array([np.linalg.solve(n[[i,(i+1)%len(n)]],h[[i,(i+1)%len(n)]]) for i in range(len(n))])
assert vertices.shape==(64,2) and np.isfinite(vertices).all()
prediction=np.asarray(details['reference_xy_rear_m']);assert prediction.shape==(30,2)
rotation=np.array([[math.cos(pose['yaw_rad']),-math.sin(pose['yaw_rad'])],
                   [math.sin(pose['yaw_rad']), math.cos(pose['yaw_rad'])]])
rear=end+rotation@np.array([cfg['geometry']['rear_axle_forward_in_base_link_m'],0.])
world_hits=hits@rotation.T+rear
unity=verified(run,'codex-time-multiscale-lap01/awsim_unity.log').decode(errors='replace')
contact_lines=[s for s in unity.splitlines() if re.search(r'collis|collid|contact|crash',s,re.I)]
summary=dict(status='PASS',source='Verified recorded control/scan and completed teacher line; no AWSIM execution',
    current_pose=pose,first_guard_after_arm_s=(guard['sim_ns']-armed)/1e9,travel_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
    previous_stop_pose=old_stop,turn='right',trigger_points_rear_forward_left_m=hits.tolist(),
    trigger_points_map_xy_m=world_hits.tolist(),trigger_forward_from_vehicle_front_m=float(hits[0,0]-1.984),
    current_vehicle_front_from_rear_m=1.984,monitor_metadata=meta,contact_related_log_lines=contact_lines,
    physical_contact_confirmed=False,source_files=source_files,teacher_reference_scope='Completed nominal teacher path for location only, not road-center ground truth')
(out/'location.json').write_text(json.dumps(summary,indent=2))

fonts=[Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),Path('/mnt/c/Windows/Fonts/meiryo.ttc')]
font=next(p for p in fonts if p.exists())
prop=FontProperties(fname=str(font))
plt.rcParams['font.family']=prop.get_name()
from matplotlib import font_manager
font_manager.fontManager.addfont(str(font))
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,2,figsize=(12,5.5),gridspec_kw={'width_ratios':[1.35,1]})
ax=axes[0]
ax.plot(teacher_xy[:,0]-origin[0],teacher_xy[:,1]-origin[1],color='#bdc6ce',lw=2,label='完走した教師の実測線（参考）')
ax.plot(xy[:,0]-origin[0],xy[:,1]-origin[1],color='#2474b4',lw=2.2,label='今回の走行')
ax.scatter(0,0,s=60,color='#26834a',zorder=6,label='発進位置')
ax.scatter(*(old_xy-origin),s=90,marker='X',color='#d99118',zorder=7,label='以前の停止（区間2）')
ax.scatter(*(end-origin),s=160,facecolors='white',edgecolors='#ca303b',linewidths=2.5,zorder=8,label='今回の停止監視（区間4）')
ax.annotate('今回：右カーブで監視停止\n発進から約209m / 165秒',xy=end-origin,xytext=(12,18),textcoords='offset points',fontsize=10,color='#a52029',
    arrowprops=dict(arrowstyle='->',color='#a52029'),bbox=dict(boxstyle='round,pad=.4',fc='white',alpha=.95,ec='#dab8bb'))
ax.set(title='コース上の停止位置',xlabel='発進位置からのmap X [m]',ylabel='発進位置からのmap Y [m]')
ax.set_aspect('equal');ax.margins(.20);ax.grid(alpha=.2);ax.legend(loc='lower left',fontsize=8)

ax=axes[1]
ax.scatter(-points[:,1],points[:,0],s=8,c='#909ba5',label='停止時のLiDAR点')
ax.fill(-vertices[:,1],vertices[:,0],color='#f3b768',alpha=.35,label='停止監視領域')
ax.add_patch(Rectangle((-.65,-.510),1.3,2.494,facecolor='#4e6273',alpha=.75,label='現在の車体形状'))
ax.plot(-prediction[:,1],prediction[:,0],color='#cf3796',lw=2.5,label='E2E予測経路')
ax.scatter(-hits[:,1],hits[:,0],s=110,c='#ca303b',zorder=8,label='監視に入った1点')
ax.annotate('車体前端より約2.1m前\n進行方向の左側（カーブ外側）',xy=(-hits[0,1],hits[0,0]),xytext=(-2.5,5.2),fontsize=9,
    arrowprops=dict(arrowstyle='->',color='#a52029'),bbox=dict(boxstyle='round,pad=.3',fc='white',ec='#dab8bb'))
ax.annotate('前方',xy=(0,1.6),xytext=(0,.4),ha='center',color='white',fontsize=10,arrowprops=dict(arrowstyle='->',color='white'))
ax.set(title='停止時の車両周囲',xlim=(-3,4),ylim=(-1.1,6.8),xlabel='車体から右方向 [m]',ylabel='後車軸から前方 [m]')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='lower right',fontsize=8)
fig.suptitle('区間4の右カーブ：停止監視領域に障害物点を検出（接触は未確認）',fontsize=13)
fig.tight_layout();fig.savefig(out/'stop_location.png',dpi=160);plt.close(fig)
print(json.dumps(summary))
