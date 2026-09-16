"""Look for a statically screened long C03 approach with unchanged goals."""
from dataclasses import asdict, replace
from pathlib import Path
import itertools,json,math,sys
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes());screen=read(out/'candidate_screen.json')
inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
base=load_pose_course(inputs/'base.csv');occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
original=LargeRecoverySite(**screen['selected']['60:C03']['site']);trials=[]
for fraction,bias,heading,ret in itertools.product((0.,),(-.06,-.085,-.1),(-1.,-1.5,-2.),(4.,6.)):
 site=replace(original,preparation_origin='nominal_path',preparation_normal_fraction=fraction,
  approach_distance_m=8.,settle_distance_m=4.,return_length_m=ret,entry_window_lead_m=.5,preparation_delay_m=6.,
  preparation_offset_bias_m=bias,preparation_heading_bias_rad=math.radians(heading))
 try:
  _,proof=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
  row=dict(site=asdict(site),map_pass=True,evidence=proof)
 except ValueError as exc:
  if not str(exc).startswith('LARGE_SITE_MAP_REJECTED'):raise
  row=dict(site=asdict(site),map_pass=False,reason=str(exc))
 trials.append(row)
good=[r for r in trials if r['map_pass']]
with (out/'c03_heading_screen.json').open('x') as f:json.dump(dict(trials=trials,scope='STATIC_PATH_SCREEN_ONLY'),f,indent=2)
if good:
 choice=min(good,key=lambda r:(abs(r['site']['preparation_normal_fraction']-.5),abs(r['site']['preparation_heading_bias_rad']-math.radians(-1.5)),abs(r['site']['preparation_offset_bias_m']+.06)))
 with (out/'c03_heading_override.json').open('x') as f:json.dump({'60:C03':choice['site']},f,indent=2)
print(json.dumps(dict(passing=len(good),trials=len(trials),selected=choice['site'] if good else None)),flush=True)
