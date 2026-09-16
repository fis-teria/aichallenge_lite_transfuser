"""Keep original goals, screen earlier admission with delayed lateral motion."""
from dataclasses import asdict,replace
from pathlib import Path
import json,math,sys
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig,LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes());screen=read(out/'candidate_screen.json')
inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
base=load_pose_course(inputs/'base.csv');occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
trials=[];selected={}
for key,delay in [('60:C03',6.),('60:C01',6.)]:
 original=LargeRecoverySite(**screen['selected'][key]['site'])
 site=replace(original,approach_distance_m=8.,settle_distance_m=4.,preparation_delay_m=delay,
  entry_window_lead_m=.5 if key=='60:C03' else 0.,preparation_offset_bias_m=-.06,
  preparation_heading_bias_rad=math.radians(.3))
 _,proof=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
 selected[key]=asdict(site);trials.append(dict(key=key,site=asdict(site),map_pass=True,evidence=proof))
with (out/'delayed_override.json').open('x') as f:json.dump(selected,f,indent=2)
with (out/'delayed_screen.json').open('x') as f:json.dump(dict(trials=trials,scope='STATIC_PATH_SCREEN_ONLY'),f,indent=2)
print(json.dumps(selected),flush=True)
