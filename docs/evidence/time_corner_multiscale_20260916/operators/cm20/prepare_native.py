from pathlib import Path
from dataclasses import asdict, replace
import json, sys, subprocess, shutil, math
import numpy as np
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite, LargeRecoveryConfig
from aic_transfuser_lite.data.time_corner_recovery_v1 import partition_corner_sites
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
inputs=Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs')
normal=Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03')
proof=Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json')
prior=Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
base=load_pose_course(inputs/'base.csv');trace=measured_normal_trace(normal,json.loads(proof.read_text())['source_hashes'][normal.name])
occupancy=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
catalog=json.loads(Path('configs/collection/corner_recovery_20260916.json').read_text())
original=[LargeRecoverySite(**r['site']) for r in catalog['corners']]

def generate(out, name, split, sites):
 side='left' if split=='train' else 'right'
 plan=dict(name=name,split=split,seed=20260916+(0 if split=='train' else 100),event_cap=len(sites),candidates=[asdict(s) for s in sites],required_site_ids=[s.site_id for s in sites],speed_policy='record_actual_v1')
 path=out/'plans'/(name+'.json');path.write_text(json.dumps(plan,indent=2))
 cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(inputs),'--normal-run',str(normal),'--normal-proof',str(proof),'--plan',str(path),'--side',side,'--output',str(out/'references'/name)]
 result=subprocess.run(cmd,capture_output=True,text=True)
 (out/(name+'_generation.log')).write_text(result.stdout+result.stderr)
 if result.returncode:raise RuntimeError(name+result.stderr[-2000:])
 print(json.dumps(dict(generated=name,sites=[s.site_id for s in sites])),flush=True)
 return plan

for cm in (20,40,60):
 out=Path('/home/thistle/e2e_autonomous/runs')/f'time_corner_multiscale{cm}_20260916'
 (out/'plans').mkdir();(out/'references').mkdir()
 full=[replace(s,target_offset_m=math.copysign(cm/100,s.target_offset_m)) for s in original]
 candidates=[];screens=[]
 if cm==20:
  # Same goals as the previous campaign, with C11's already-screened shorter approach.
  late=replace(full[4],site_id='C05_LATE',corner_id='C05A',release_s_m=185.61)
  selected=[late,full[6],replace(full[10],approach_distance_m=6.)]
  # Old catalog already has shortened C07 (4+2).
  groups=[selected,[full[0],full[3],full[9]]]
  candidates=[s for g in groups for s in g]
  for s in candidates:preparation_course(base,trace,LargeRecoveryConfig((s,),speed_policy='record_actual_v1'),occupancy)
 else:
  # Try the exact corner/offset/heading first. Only vary approach and static-return length.
  for site in full:
   chosen=None
   combinations=[(site.approach_distance_m,site.settle_distance_m,site.return_length_m),(6.,4.,6.),(4.,4.,4.),(4.,2.,4.),(8.,4.,4.)]
   for approach,settle,ret in dict.fromkeys(combinations):
    s=replace(site,approach_distance_m=approach,settle_distance_m=settle,return_length_m=ret)
    try:
     _,evidence=preparation_course(base,trace,LargeRecoveryConfig((s,),speed_policy='record_actual_v1'),occupancy)
     screens.append(dict(site=asdict(s),map_pass=True,evidence=evidence));chosen=s;break
    except ValueError as exc:
     if not str(exc).startswith('LARGE_SITE_MAP_REJECTED'):raise
     screens.append(dict(site=asdict(s),map_pass=False,reason=str(exc)))
   if chosen:candidates.append(chosen)
  groups=[list(g.sites) for g in partition_corner_sites(candidates)]
 # Catalog records all original goals, including deferred map failures.
 planned_catalog=dict(catalog,corners=[dict(row,site=asdict(site)) for row,site in zip(catalog['corners'],full)])
 (out/'catalog.json').write_text(json.dumps(planned_catalog,indent=2))
 (out/'map_screen.json').write_text(json.dumps(screens,indent=2))
 plans=[]
 for i,group in enumerate(groups):
  for split in ('train','validation'):plans.append(generate(out,f'{split}_lap{i+1:02}',split,group))
 report=dict(amplitude_cm=cm,speed_policy='record_actual_v1',maximum_runs=12,plans=plans,map_deferred=[s.corner_id for s in full if s.corner_id not in {c.corner_id for c in candidates}],original_goal_definition=planned_catalog)
 (out/'plans/plan.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(dict(amplitude_cm=cm,groups=[[s.site_id for s in g] for g in groups],deferred=report['map_deferred'])),flush=True)
