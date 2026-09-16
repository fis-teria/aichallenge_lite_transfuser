import json
import ops_corner as m
m.remote(m.PREFIX+r"""
from pathlib import Path
import json
root=Path(ROOT);out=[]
for spec in json.loads((root/'parallel_plan.json').read_text())['instances']:
 p=root/spec['run_id'];path=p/'control.jsonl'
 if not path.exists():continue
 rows=[json.loads(s) for s in path.read_text().splitlines() if s.endswith('}')]
 applied=[r for r in rows if (r.get('large_recovery') or {}).get('applied')]
 events={}
 for r in applied:
  lr=r['large_recovery'];st=lr['state'];idx=st['site_cursor']
  if st['event_id'] and idx<len(lr['config']['sites']):
   key=st['event_id'];site=lr['config']['sites'][idx]
   e=events.setdefault(key,dict(site=site['site_id'],stage=None,reason=None,peak_speed_mps=0.,max_lateral_m=0.))
   e.update(stage=st['stage'],reason=st['reason']);e['peak_speed_mps']=max(e['peak_speed_mps'],r['speed_mps']);e['max_lateral_m']=max(e['max_lateral_m'],abs(lr['lateral_error_m']))
 h=(json.loads((p/'result.json').read_text())['last_control'] if (p/'result.json').exists() else json.loads((p/'control_heartbeat.json').read_text()));st=(h.get('large_recovery') or {}).get('state',{})
 out.append(dict(run=spec['run_id'],fault=h.get('fault'),s_m=(h.get('projection')or{}).get('s_m'),stage=st.get('stage'),reason=st.get('reason'),completed=st.get('completed_events'),above_old_band_rows=sum(r['speed_mps']>1.4 for r in applied),events=events))
print(json.dumps(out))
""",timeout=30)
