"""Audit copied closed bags, materialize real teachers, then update coverage."""
import argparse
import ops_corner as m

ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
m.remote('PAIR='+repr(a.pair)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys,hashlib
from aic_transfuser_lite.data.time_corner_recovery_v1 import corner_coverage
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_multiscale60_20260916')
raw=Path('/home/thistle/e2e_autonomous/raw/time_corner_multiscale60_20260916')
proof=json.loads((out/f'corner60_pair{PAIR:02}_20260916_verified.json').read_text())
assert proof['all_files_and_directory_structure_identical'] and proof['all_sqlite_quick_checks_passed']
for domain,split in [(1,'train'),(2,'validation')]:
 name=f'codex-time-recovery-corner60-p{PAIR:02}-d{domain}'
 assert name in {r['run_id'] for r in proof['runs']}
 log=out/(name+'_audit_execution.log')
 with log.open('x') as f:p=subprocess.run([sys.executable,'-u',str(out/'audit_corner.py'),'--run',name,'--split',split],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
 if p.returncode:raise RuntimeError(log.read_text()[-4500:])
 clock=out/(name+'_clock.json')
 with clock.with_suffix('.log').open('x') as f:p=subprocess.run([sys.executable,str(out/'clock_audit.py'),'--raw',str(raw/name),'--output',str(clock)],stdout=f,stderr=subprocess.STDOUT,timeout=600)
 if p.returncode:raise RuntimeError(clock.with_suffix('.log').read_text()[-4500:])
 summary=json.loads((out/(name+'_collection_summary.json')).read_text())
 print(json.dumps(dict(run=name,accepted=summary['accepted'],events=summary['events'])),flush=True)
catalog=json.loads((out/'catalog.json').read_text())
audits=[]
for p in sorted(out.glob('*_collection_summary.json')):
 row=json.loads(p.read_text());states=out/(row['run_id']+'_anchor_states.json')
 row['anchor_states']=json.loads(states.read_text()) if row['accepted'] else []
 audits.append(row)
coverage=corner_coverage([LargeRecoverySite(**c['site']) for c in catalog['corners']],audits)
coverage['summary_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.glob('*_collection_summary.json'))}
coverage['catalog_sha256']=hashlib.sha256((out/'catalog.json').read_bytes()).hexdigest()
with (out/f'coverage_after_pair{PAIR:02}.json').open('x') as f:json.dump(coverage,f,indent=2)
print(json.dumps(dict(complete=coverage['complete'],missing=coverage['missing'])),flush=True)
''',native=True,lock=True,timeout=5000)
