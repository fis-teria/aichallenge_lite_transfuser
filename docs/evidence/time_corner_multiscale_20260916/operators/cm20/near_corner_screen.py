from pathlib import Path
from dataclasses import replace,asdict
import json,sys
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite,LargeRecoveryConfig
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
inputs=Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs')
run=Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03')
proof=json.loads(Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json').read_text())
normal=measured_normal_trace(run,proof['source_hashes'][run.name]);base=load_pose_course(inputs/'base.csv');occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
sites=[LargeRecoverySite(**r['site']) for r in json.loads(Path('configs/collection/corner_recovery_20260916.json').read_text())['corners']]
reports=[];selected=[]
for name in ('C03','C06'):
 site=next(s for s in sites if s.site_id==name);chosen=None
 for delta in (0.,-6.,6.,-8.,8.,-10.,10.):
  for approach,settle in ((4.,2.),(4.,4.),(6.,4.),(8.,4.)):
   s=replace(site,release_s_m=site.release_s_m+delta,approach_distance_m=approach,settle_distance_m=settle,return_length_m=4.)
   try:
    _,e=preparation_course(base,normal,LargeRecoveryConfig((s,),speed_policy='record_actual_v1'),occ)
    reports.append(dict(site=name,delta_m=delta,approach_m=approach,settle_m=settle,map_pass=True,evidence=e));chosen=s;break
   except ValueError as exc:
    if not str(exc).startswith('LARGE_SITE_MAP_REJECTED'):raise
    reports.append(dict(site=name,delta_m=delta,approach_m=approach,settle_m=settle,map_pass=False))
  if chosen:break
 if chosen:selected.append(dict(original_site=asdict(site),candidate=asdict(replace(chosen,site_id=name+'_NEAR',corner_id=name+'A')),same_original_entry=chosen.release_s_m==site.release_s_m))
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_multiscale20_20260916')
(out/'near_corner_screen.json').write_text(json.dumps(dict(attempts=reports,selected=selected),indent=2));print(json.dumps(selected))
