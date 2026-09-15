import sys
import manage as m
name=sys.argv[1]
assert name.startswith('codex-time-recovery-separated-')
m.remote(rf'''
from pathlib import Path
import json
data=(Path({m.ROOT!r})/{name!r}/'control.jsonl').read_text()
rows=[json.loads(s) for s in data.splitlines() if s.strip().endswith('}}')]
rows=[r for r in rows if r.get('publication') and r['pulse']['applied'] and r['phase']=='hold']
issues=[]
for a,b in zip(rows,rows[1:]):
 if a['random_pulse']['state']['event_id']!=b['random_pulse']['state']['event_id']:continue
 pa=a['publication'];pb=b['publication'];ds=pb['sim_ns']-pa['sim_ns'];dw=pb['monotonic_ns']-pa['monotonic_ns'];dq=pb['sequence']-pa['sequence']
 if ds>150000000 or dw>150000000 or dq!=1:
  issues.append(dict(event_id=b['random_pulse']['state']['event_id'],sim_interval_ms=ds/1e6,wall_interval_ms=dw/1e6,
   sequences=[pa['sequence'],pb['sequence']],weights=[a['guide_control']['guide_weight'],b['guide_control']['guide_weight']],
   issued=[a['issued_angle_rad'],b['issued_angle_rad']],processing=[a['processing_ms'],b['processing_ms']],gc=[a['gc_pauses'],b['gc_pauses']]))
print(json.dumps(dict(run_id={name!r},provisional_live_intervals=issues)))
''')
