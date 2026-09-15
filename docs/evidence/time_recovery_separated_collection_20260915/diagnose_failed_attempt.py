import manage as m

m.remote(rf'''
from pathlib import Path
import json,subprocess,statistics,hashlib
root=Path({m.ROOT!r});p=root/'codex-time-recovery-separated-g05-left'
result=json.loads((p/'result.json').read_text())
print(json.dumps(dict(result={{k:v for k,v in result.items() if k not in ('last_control','nodes','manifest')}},
 last_control=result.get('last_control'),nodes=result.get('nodes')),ensure_ascii=False))
rows=[json.loads(s) for s in (p/'control.jsonl').read_text().splitlines()]
faults=[i for i,r in enumerate(rows) if r.get('fault')=='COLLECTION_COMPUTATION_TIMEOUT' or r.get('reason')=='COLLECTION_COMPUTATION_TIMEOUT']
print('row_count',len(rows),'fault_indexes',faults[:5])
if faults:
 i=faults[0]
 for row in rows[max(0,i-2):i+2]:print('FAULT_CONTEXT',json.dumps(row))
for key in ('processing_ms','decision_wall_ms','thread_cpu_ms'):
 values=sorted(float(r[key]) for r in rows if isinstance(r.get(key),(float,int)))
 if values:print(key,json.dumps(dict(n=len(values),median=statistics.median(values),p99=values[int(len(values)*.99)],maximum=max(values))))
print('FILES',json.dumps([str(f.relative_to(p)) for f in p.rglob('*') if f.is_file() and (f.suffix in ('.log','.json') or 'stderr' in f.name)]))
print('HOST_PROCESSES',subprocess.check_output(['ps','-eo','pid,ppid,psr,pcpu,pmem,comm','--sort=-pcpu'],text=True).splitlines()[:16])
print('SUPERVISOR_TAIL',(root/(p.name+'_supervisor.log')).read_text()[-2000:])
''')
