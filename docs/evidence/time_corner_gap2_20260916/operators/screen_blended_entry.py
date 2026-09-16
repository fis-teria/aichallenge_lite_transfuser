"""Map screen C06 interpolation and actual goal poses at constrained sites."""
from dataclasses import asdict, replace
from pathlib import Path
import itertools, json, math, sys
import numpy as np
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_map_body_v1 import body_path_is_free

root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes())
screen=read(out/'candidate_screen.json');inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
base=load_pose_course(inputs/'base.csv');occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
original=LargeRecoverySite(**next(c['site'] for c in screen['catalogs']['20']['corners'] if c['site']['corner_id']=='C06'))
trials=[]
for fraction,bias,heading in itertools.product((.35,.5,.65),(.06,.085,.1),(0.,.5,.75,1.)):
 site=replace(original,preparation_origin='blended_normal',preparation_normal_fraction=fraction,
  approach_distance_m=4.,settle_distance_m=4.,preparation_offset_bias_m=bias,preparation_heading_bias_rad=math.radians(heading))
 try:
  _,evidence=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
  row=dict(site=asdict(site),map_pass=True,evidence=evidence)
 except ValueError as exc:
  if not str(exc).startswith('LARGE_SITE_MAP_REJECTED'):raise
  row=dict(site=asdict(site),map_pass=False,reason=str(exc))
 trials.append(row)
valid=[r for r in trials if r['map_pass']]
assert valid
choice=min(valid,key=lambda r:(abs(r['site']['preparation_normal_fraction']-.5),abs(r['site']['preparation_offset_bias_m']-.1),abs(r['site']['preparation_heading_bias_rad']-math.radians(.5))))
with (out/'blended_entry_override.json').open('x') as f:json.dump({'20:C06':choice['site']},f,indent=2)
poses=[]
for key in ('60:C01','60:C03','20:C06','40:C06','60:C06'):
 cm,corner=key.split(':');site=LargeRecoverySite(**next(c['site'] for c in screen['catalogs'][cm]['corners'] if c['site']['corner_id']==corner))
 for delta_s in (-.5,0.,.5,1.):
  progress=site.release_s_m+delta_s
  nx,ny,yaw=[float(np.interp(progress,normal[:,0],np.unwrap(normal[:,k]) if k==3 else normal[:,k])) for k in (1,2,3)]
  for delta_lat,delta_heading in itertools.product((-.05,0.,.05),(-1.,0.,1.)):
   offset=site.target_offset_m+delta_lat;heading=yaw+site.target_heading_rad+math.radians(delta_heading)
   pose=[nx-math.sin(yaw)*offset,ny+math.cos(yaw)*offset,heading]
   free=body_path_is_free(occ,np.asarray([pose,pose]))
   poses.append(dict(key=key,progress_m=progress,offset_m=offset,heading_error_deg=math.degrees(site.target_heading_rad)+delta_heading,free=free))
with (out/'blended_entry_screen.json').open('x') as f:json.dump(dict(trials=trials,selected=choice,goal_pose_samples=poses,scope='Sampled static poses; no continuous feasibility or dynamic recovery proof'),f,indent=2)
print(json.dumps(dict(selected=choice['site'],passing=len(valid),goal_counts={key:sum(r['free'] for r in poses if r['key']==key) for key in sorted({r['key'] for r in poses})})),flush=True)
