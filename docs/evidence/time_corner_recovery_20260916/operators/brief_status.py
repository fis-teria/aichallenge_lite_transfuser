"""Read only; show the active pair without dumping sensor/control payloads."""
import ops_corner as m

m.remote(m.PREFIX + r'''
from pathlib import Path
import json,shutil
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_bytes());rows=[]
for spec in plan['instances']:
 path=root/spec['run_id'];final=path/'result.json';heartbeat=path/'control_heartbeat.json'
 result=json.loads(final.read_bytes()) if final.exists() else {}
 control=result.get('last_control') or (json.loads(heartbeat.read_bytes()) if heartbeat.exists() else {})
 state=(control.get('large_recovery') or {}).get('state') or {}
 rows.append(dict(run_id=spec['run_id'],domain=spec['ros_domain_id'],authorized=(path/'drive_authorized.json').exists(),
  status=result.get('status'),fault=control.get('fault'),reason=control.get('reason'),
  progress_m=(control.get('projection') or {}).get('s_m'),speed_mps=control.get('speed_mps'),
  stage=state.get('stage'),events=state.get('completed_events'),skipped=state.get('skipped_sites'),
  ended=bool(result),closed=(path/'transfer_manifest.json').exists()))
print(json.dumps(dict(runs=rows,free_gib=shutil.disk_usage(root).free/2**30)))
''', timeout=30)
