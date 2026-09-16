from pathlib import Path
import json,sys,subprocess,math,hashlib
for cm in (40,60):
 out=Path('/home/thistle/e2e_autonomous/runs')/f'time_corner_multiscale{cm}_20260916';schedule=[]
 for number,base in ((1,'lap01'),(2,'lap02'),(3,'lap03_late')):
  row=dict(pair=number)
  for split,side in [('train','left'),('validation','right')]:
   original_name=split+'_'+base;plan=json.loads((out/'plans'/(original_name+'.json')).read_text());plan['name']=original_name+'_h2'
   for s in plan['candidates']:
    s['heading_tolerance_rad']=math.radians(2.)
    if s['site_id']=='C07':s.update(approach_distance_m=8.,settle_distance_m=4.)
    if s['site_id']=='C11':s.update(approach_distance_m=6.,settle_distance_m=4.)
   plan['note']='Observed lateral target remains +/-5cm; heading tolerance uses existing 2deg setting. Original exact-entry coverage still evaluated separately.'
   path=out/'plans'/(plan['name']+'.json');path.write_text(json.dumps(plan,indent=2))
   cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs','/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs','--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03','--normal-proof','/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json','--plan',str(path),'--side',side,'--output',str(out/'references'/plan['name'])]
   p=subprocess.run(cmd,capture_output=True,text=True);(out/(plan['name']+'_generation.log')).write_text(p.stdout+p.stderr);assert p.returncode==0,p.stderr
   for suffix in ('.csv',):
    assert (out/'references'/original_name/(side+suffix)).read_bytes()==(out/'references'/plan['name']/(side+suffix)).read_bytes()
   row[side]=plan['name']
  schedule.append(row)
 (out/'collection_schedule.json').write_text(json.dumps(dict(heading_tolerance_deg=2,nominal_path_unchanged=True,preparation_changes={'C07':'8m approach + 4m settling now that overspeed is retained','C11':'6m approach + 4m settling with previously verified stable start window'},pairs=schedule,maximum_attempts=12),indent=2));print(json.dumps(dict(cm=cm,pairs=schedule)))
