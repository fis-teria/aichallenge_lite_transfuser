import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys
out=Path(OUT)
gate=json.loads((out/'test_gate_a488ab6.json').read_text());assert gate['full_exit']==0
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==gate['commit']
schedule=json.loads((out/'collection_schedule_effective.json').read_text())
schedule['pairs']=[p for p in schedule['pairs'] if p['pair']<=4]
for pair in (5,6):
 row=dict(pair=pair,recovery_duration_s=15.)
 for split,side in [('train','left'),('validation','right')]:
  if pair==5:
   original=split+'_lap03_late_h2_e2_c11';plan=json.loads((out/'plans'/(original+'.json')).read_text());plan['name']=original+'_f15'
  else:
   plan=json.loads((out/'plans'/(split+'_lap01_h2_e2.json')).read_text())
   other=json.loads((out/'plans'/(split+'_lap02_h2_e2.json')).read_text())
   plan['candidates']=[s for s in plan['candidates'] if s['site_id']=='C04']+[s for s in other['candidates'] if s['site_id'] in ('C08','C11')]
   plan['event_cap']=3;plan['required_site_ids']=[s['site_id'] for s in plan['candidates']]
   assert plan['required_site_ids']==['C04','C08','C11']
   plan['name']=split+'_lap06_f15'
  plan['recovery_duration_s']=15.;plan['note']='Explicit 15s observed recovery window for 60cm. Stability, sensor freshness, publication deadline and stopping sweep unchanged. Earlier 10s recordings keep their original interpretation.'
  path=out/'plans'/(plan['name']+'.json')
  with path.open('x') as f:json.dump(plan,f,indent=2)
  cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs','/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs','--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03','--normal-proof','/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json','--plan',str(path),'--side',side,'--output',str(out/'references'/plan['name'])]
  run=subprocess.run(cmd,capture_output=True,text=True);(out/(plan['name']+'_generation.log')).write_text(run.stdout+run.stderr);assert run.returncode==0,run.stderr
  if pair==5:
   for suffix in ('.csv','_preparation.csv'):
    assert (out/'references'/original/(side+suffix)).read_bytes()==(out/'references'/plan['name']/(side+suffix)).read_bytes()
  row[side]=plan['name']
 schedule['pairs'].append(row)
with (out/'collection_schedule_f15.json').open('x') as f:json.dump(schedule,f,indent=2)
print(json.dumps(schedule))
''',native=True,lock=True,timeout=600)
