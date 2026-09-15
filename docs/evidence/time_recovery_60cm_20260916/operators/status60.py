import ops60 as m
m.remote('ROOT='+repr(m.ROOT)+'\n'+r'''
from pathlib import Path
import json,shutil,subprocess
root=Path(ROOT);ledger=json.loads((root/'campaign_20260914.json').read_text());out=[]
for a in ledger['attempts']:
 if a['state']=='WSL_MOVED':continue
 run=root/a['run_id'];r={};h={}
 if (run/'result.json').exists():r=json.loads((run/'result.json').read_text());h=r.get('last_control') or {}
 elif (run/'control_heartbeat.json').exists():h=json.loads((run/'control_heartbeat.json').read_text())
 s=(h.get('large_recovery') or {}).get('state') or {}
 out.append(dict(run=a['run_id'],status=r.get('status'),progress=(h.get('projection') or {}).get('s_m'),
  phase=h.get('phase'),stage=s.get('stage'),completed=s.get('completed_events'),skipped=s.get('skipped_sites'),
  reason=s.get('reason'),fault=h.get('fault'),closed=(r.get('nodes') or {}).get('closed_bag')))
print(json.dumps(dict(runs=out,attempts=len(ledger['attempts']),free_gib=shutil.disk_usage(root).free/2**30)))
''')
