import json
import manage as m
m.remote(rf'''
from pathlib import Path
import json,shutil,subprocess
root=Path({m.ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text());rows=[]
for a in ledger['attempts']:
 p=root/a['run_id'];r={{'run_id':p.name,'state':a['state']}}
 if (p/'result.json').exists() or (p/'MOVED_TO_WSL.json').exists():
  if (p/'result.json').exists():v=json.loads((p/'result.json').read_text())
  else:v=json.loads((p/'MOVED_TO_WSL.json').read_text())['result']
  c=v['last_control'];s=c.get('random_pulse',{{}}).get('state',{{}})
  r.update(status=v['status'],fault=c.get('fault'),recovered=s.get('completed_events'),events=s.get('event_id'),skipped=s.get('skipped_sites'))
 elif (p/'control_heartbeat.json').exists():
  c=json.loads((p/'control_heartbeat.json').read_text());s=c.get('random_pulse',{{}}).get('state',{{}})
  r.update(phase=c.get('phase'),fault=c.get('fault'),progress_m=(c.get('projection') or {{}}).get('s_m'),recovered=s.get('completed_events'),events=s.get('event_id'))
 rows.append(r)
print(json.dumps(dict(runs=rows,sealed=ledger['sealed'],free_gib=round(shutil.disk_usage(root).free/2**30,2),
 containers=subprocess.check_output(['docker','ps','--format','{{{{.Names}}}}'],text=True).splitlines())))
''')
for p in sorted(m.UNC.glob('pair*_audit.json')):
    r=json.loads(p.read_bytes());print(json.dumps(dict(pair=r['pair'],accepted=r['total_accepted'],events=r['events'],successful=r['successful_events'])))
stage_files=[p for p in m.UNC.glob('codex-*json') if any(k in p.name for k in ('bag_audit','causal_probe','state_audit','collection_summary'))]
print(json.dumps(dict(native_stage_file_count=len(stage_files),latest_stages=[p.name for p in sorted(stage_files,key=lambda p:p.stat().st_mtime)[-3:]])))
