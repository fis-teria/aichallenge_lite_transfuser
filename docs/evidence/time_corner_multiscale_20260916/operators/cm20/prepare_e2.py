"""Regenerate future plans with tested explicit 2-degree entry readiness."""
from pathlib import Path
import copy,hashlib,json,math,subprocess,sys
base=Path('/home/thistle/e2e_autonomous/runs')
for cm in (40,60):
 out=base/f'time_corner_multiscale{cm}_20260916'
 gate=json.loads((out/'test_gate_21ccfef.json').read_text());assert gate['full_exit']==0
 assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==gate['commit']
 schedule=json.loads((out/'collection_schedule.json').read_text());rows=[]
 pairs=((2,'lap02'),(3,'lap03_late'),(4,'lap04_retry')) if cm==40 else ((1,'lap01'),(2,'lap02'),(3,'lap03_late'))
 for number,stem in pairs:
  row=dict(pair=number)
  for split,side in [('train','left'),('validation','right')]:
   origin=split+'_'+stem+'_h2'
   if number==4:
    p1=json.loads((out/'plans'/(split+'_lap01_h2.json')).read_text())
    p3=json.loads((out/'plans'/(split+'_lap03_late_h2.json')).read_text())
    plan=copy.deepcopy(p1)
    plan['candidates']=[s for s in p1['candidates'] if s['site_id'] in ('C01','C07')]+[s for s in p3['candidates'] if s['site_id']=='C11']
    assert [s['site_id'] for s in plan['candidates']]==['C01','C07','C11']
    plan['required_site_ids']=[s['site_id'] for s in plan['candidates']]
   else:plan=json.loads((out/'plans'/(origin+'.json')).read_text())
   plan['name']=origin+'_e2';plan['entry_heading_tolerance_rad']=math.radians(2.)
   plan['note']='Explicit entry heading tolerance 2deg; other readiness gates and original entry-state coverage criteria unchanged.'
   path=out/'plans'/(plan['name']+'.json')
   with path.open('x') as f:json.dump(plan,f,indent=2)
   command=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(base/'time_recovery_collection_20260913/inputs'),'--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03','--normal-proof',str(base/'time_recovery_separated_20260915/selected_site_plan.json'),'--plan',str(path),'--side',side,'--output',str(out/'references'/plan['name'])]
   run=subprocess.run(command,capture_output=True,text=True)
   (out/(plan['name']+'_generation.log')).write_text(run.stdout+run.stderr);assert run.returncode==0,run.stderr
   if number!=4:
    for suffix in ('.csv','_preparation.csv'):
     assert (out/'references'/origin/(side+suffix)).read_bytes()==(out/'references'/plan['name']/(side+suffix)).read_bytes()
   ref=json.loads((out/'references'/plan['name']/(side+'.json')).read_text())
   assert ref['large_recovery']['config']['entry_heading_tolerance_rad']==math.radians(2.)
   row[side]=plan['name']
  rows.append(row)
 schedule['entry_heading_tolerance_deg']=2
 if cm==40:rows=[dict(schedule['pairs'][0],entry_heading_tolerance_deg=1),*rows]
 schedule['pairs']=rows
 with (out/'collection_schedule_e2.json').open('x') as f:json.dump(schedule,f,indent=2)
 print(json.dumps(dict(cm=cm,pairs=rows)),flush=True)
