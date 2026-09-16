"""Inspect supplementary earlier locations; never count them as original goals."""
from dataclasses import asdict,replace
from pathlib import Path
import itertools,json,math,sys
import numpy as np
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig,LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_map_body_v1 import body_path_is_free
root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes());screen=read(out/'candidate_screen.json')
inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
base=load_pose_course(inputs/'base.csv');occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
trials=[]
for key,locations in [('60:C01',(20.5,22.,24.)),('40:C06',(194.,196.,198.,200.,202.,204.,206.,208.)),('60:C06',(194.,196.,198.,200.,202.,204.,206.,208.))]:
 cm,corner=key.split(':');original=LargeRecoverySite(**next(c['site'] for c in screen['catalogs'][cm]['corners'] if c['site']['corner_id']==corner))
 for release in locations:
  poses=[]
  for progress,dlat,dhead in itertools.product((release,release+.5,release+1.),(-.05,0.,.05),(-1.,0.,1.)):
   nx,ny,yaw=[float(np.interp(progress,normal[:,0],np.unwrap(normal[:,k]) if k==3 else normal[:,k])) for k in (1,2,3)]
   offset=original.target_offset_m+dlat;angle=yaw+original.target_heading_rad+math.radians(dhead)
   pose=[nx-math.sin(yaw)*offset,ny+math.cos(yaw)*offset,angle]
   poses.append(dict(progress_m=progress,offset_m=offset,heading_delta_deg=dhead,free=body_path_is_free(occ,np.asarray([pose,pose]))))
  variants=[]
  if all(p['free'] for p in poses):
   for origin in ('nominal_path','blended_normal','measured_normal'):
    site=replace(original,site_id=corner+'E',corner_id=corner+'E',release_s_m=release,preparation_origin=origin,
      approach_distance_m=8.,settle_distance_m=4.,preparation_offset_bias_m=-original.target_offset_m*.1,
      preparation_heading_bias_rad=math.copysign(math.radians(.3),original.target_heading_rad))
    try:
     _,proof=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
     variants.append(dict(site=asdict(site),map_pass=True,evidence=proof))
    except ValueError as exc:
     if not str(exc).startswith('LARGE_SITE_MAP_REJECTED'):raise
     variants.append(dict(site=asdict(site),map_pass=False,reason=str(exc)))
  trials.append(dict(original_key=key,original_release_m=original.release_s_m,release_s_m=release,pose_samples=poses,variants=variants))
with (out/'earlier_goals_screen.json').open('x') as f:json.dump(dict(trials=trials,scope='Supplementary goals only; original coverage remains unchanged; static screening is not dynamic proof'),f,indent=2)
print(json.dumps([dict(key=r['original_key'],release=r['release_s_m'],free=sum(p['free'] for p in r['pose_samples']),feasible_origins=[v['site']['preparation_origin'] for v in r['variants'] if v['map_pass']]) for r in trials]),flush=True)
