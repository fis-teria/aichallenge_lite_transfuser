"""Reconstruct the first recorded sweep rejection, without changing admission."""
from pathlib import Path
import hashlib
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.vehicle_motion_v1 import stopping_motion, AWSIM_10KMH_POLICY
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_10kmh_20260917')
raw=root/'raw/codex-time-launch10-lap02'
out=root/'stop_location';out.mkdir(exist_ok=False)
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
data=(raw/'control.jsonl').read_bytes()
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
rows=[json.loads(line) for line in data.splitlines()]
armed=next(r['sim_ns'] for r in rows if r['event']=='ARMED')
guard=next(r for r in rows if r['event']=='SCAN_GUARD_REJECTED')
command=next(r for r in rows if r['event']=='COMMAND_SENT' and r['reason']=='STOPPING_SWEEP_OCCUPIED' and r.get('details',{}).get('current_pose'))
assert guard['sim_ns']==command['sim_ns']
d=command['details'];scan=guard['scan'];obs=guard['motion_observation'];speed=guard['speed_mps']
motion=stopping_motion(speed,guard['measured_steer_rad'],guard['issued_steer_rad'],guard['previous_steer_rad'],
    policy=AWSIM_10KMH_POLICY,heading_rate_radps=obs['heading_rate_radps'],reported_lateral_mps=obs['reported_lateral_mps'])
sensor=np.asarray(guard['scan_in_current_rear']);travel=.4+.5*speed+speed*speed/2
n,h,meta=curvature_support_envelope(*motion['curvature_interval_per_m'],travel,scan['angle_increment'],sensor[:2],
    lateral_padding_m=motion['lateral_displacement_bound_m'],vehicle_model_policy=AWSIM_10KMH_POLICY)
proof=scan_margin(scan,sensor,speed_mps=speed,measured_rad=guard['measured_steer_rad'],issued_rad=guard['issued_steer_rad'],
    previous_rad=guard['previous_steer_rad'],yaw_rate_radps=obs['heading_rate_radps'],lateral_mps=obs['reported_lateral_mps'],vehicle_model_policy=AWSIM_10KMH_POLICY)
assert proof['reason']==guard['reason'] and proof['minimum_ray_margin_m']<0
ranges=np.asarray(scan['ranges'],float);angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']+sensor[2]
valid=np.isfinite(ranges)&(ranges>=scan['range_min'])&(ranges<=scan['range_max'])
points=sensor[:2]+ranges[valid,None]*np.column_stack((np.cos(angles[valid]),np.sin(angles[valid])))
inside=np.all(n@points.T<=h[:,None]+1e-9,axis=0);hits=points[inside]
vertices=np.array([np.linalg.solve(n[[i,(i+1)%len(n)]],h[[i,(i+1)%len(n)]]) for i in range(len(n))])
prediction=np.asarray(d['reference_xy_rear_m'])
pose=d['current_pose'];rotation=np.array([[math.cos(pose['yaw_rad']),-math.sin(pose['yaw_rad'])],[math.sin(pose['yaw_rad']),math.cos(pose['yaw_rad'])]])
rear=np.array([pose['x_m'],pose['y_m']])+rotation@np.array([.0010000169277191162,0.])
result=dict(status='PASS',first_guard_after_arm_s=(guard['sim_ns']-armed)/1e9,current_pose=pose,speed_kmh=speed*3.6,
    selected_lookahead_m=d['selected_lookahead_distance_m'],minimum_preview_m=d['minimum_preview_distance_m'],
    source_prediction_endpoint_m=prediction[-1].tolist(),source_prediction_endpoint_norm_m=float(np.linalg.norm(prediction[-1])),
    predicted_source_speed_mps=d['predicted_source_speed_mps'],plan_age_s=d['plan_age_sec'],
    pp_required_tire_rad=d['steer_rad'],measured_tire_rad=guard['measured_steer_rad'],issued_tire_rad=guard['issued_steer_rad'],
    stopping_travel_m=travel,scan_replay=proof,motion=motion,monitor_metadata=meta,
    lidar_points_inside_envelope_rear_xy_m=hits.tolist(),lidar_points_inside_envelope_map_xy_m=(hits@rotation.T+rear).tolist(),
    physical_contact_confirmed=False,zero_speed_stop_confirmed=False,
    boundary='First guard decision reproduced; simulator was frozen at runtime fault, not a measured complete braking rollout.',
    control_sha256=manifest['control.jsonl']['sha256'])
(out/'diagnosis.json').write_text(json.dumps(result,indent=2))
fig,ax=plt.subplots(figsize=(7,7))
ax.scatter(-points[:,1],points[:,0],s=9,color='#8a959e',label='Recorded LiDAR points')
ax.fill(-vertices[:,1],vertices[:,0],color='#e2a356',alpha=.35,label='Stopping monitor envelope')
ax.add_patch(Rectangle((-.65,-.510),1.3,2.494,facecolor='#52697b',alpha=.8,label='Current vehicle body'))
ax.plot(-prediction[:,1],prediction[:,0],color='#c83a92',linewidth=2,label='E2E predicted path at current rear axle')
if len(hits):ax.scatter(-hits[:,1],hits[:,0],s=75,color='#b92838',zorder=5,label='LiDAR points inside envelope')
target=np.asarray(d['lookahead_rear_m']);ax.scatter(-target[1],target[0],marker='x',s=70,color='black',label='PP target')
ax.set(xlim=(-5,7),ylim=(-2,10),xlabel='Right of rear axle [m]',ylabel='Forward of rear axle [m]',
    title=f"First guard at {speed*3.6:.2f} km/h | stopping travel {travel:.2f} m\nRecorded ray margin {proof['minimum_ray_margin_m']:.3f} m")
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);fig.tight_layout()
fig.savefig(out/'stopping_monitor.png',dpi=150);plt.close(fig)
print(json.dumps(result))
