import manage as m
m.remote(rf'''
from pathlib import Path
import hashlib,json
root=Path({m.RAW!r})/'codex-time-recovery-separated-g02-right'
data=(root/'control.jsonl').read_bytes();assert hashlib.sha256(data).hexdigest()==json.loads((root/'transfer_manifest.json').read_bytes())['control.jsonl']['sha256']
rows=[json.loads(s) for s in data.splitlines()]
rows=[r for r in rows if r.get('publication') and r['pulse']['applied'] and r['phase']=='hold' and r['random_pulse']['state']['event_id']==3]
last=None
for r in rows:
 p=r['publication'];g=r['guide_control']
 print(json.dumps(dict(sim=p['sim_ns'],elapsed=(p['sim_ns']-rows[0]['publication']['sim_ns'])/1e9,
 dt_sim_ms=(p['sim_ns']-last['sim_ns'])/1e6 if last else None,dt_wall_ms=(p['monotonic_ns']-last['monotonic_ns'])/1e6 if last else None,
 weight=g['guide_weight'],issued_delta=r['issued_angle_rad']-g['guide_angle_rad'],sequence=p['sequence'])))
 last=p
''',native=True,lock=True)
