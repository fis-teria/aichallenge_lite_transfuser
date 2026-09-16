"""Use the third available event slot for an independently repeated C11."""
import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys
out=Path(OUT);schedule=json.loads((out/'collection_schedule_e2.json').read_text())
assert json.loads((out/'test_gate_21ccfef.json').read_text())['full_exit']==0
for split,side in [('train','left'),('validation','right')]:
 plan=json.loads((out/'plans'/(split+'_lap03_late_h2_e2.json')).read_text())
 prior=json.loads((out/'plans'/(split+'_lap02_h2_e2.json')).read_text())
 assert [s['site_id'] for s in plan['candidates']]==['C05_LATE','C09']
 plan['candidates'].append(next(s for s in prior['candidates'] if s['site_id']=='C11'))
 plan['event_cap']=3;plan['required_site_ids']=[s['site_id'] for s in plan['candidates']]
 plan['name']+='_c11'
 path=out/'plans'/(plan['name']+'.json')
 with path.open('x') as f:json.dump(plan,f,indent=2)
 command=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs','/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs','--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03','--normal-proof','/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json','--plan',str(path),'--side',side,'--output',str(out/'references'/plan['name'])]
 run=subprocess.run(command,capture_output=True,text=True)
 (out/(plan['name']+'_generation.log')).write_text(run.stdout+run.stderr);assert run.returncode==0,run.stderr
 next(p for p in schedule['pairs'] if p['pair']==3)[side]=plan['name']
with (out/'collection_schedule_e2_c11.json').open('x') as f:json.dump(schedule,f,indent=2)
print(json.dumps(schedule))
''',native=True,lock=True,timeout=300)
