"""One bounded pair; preserve, transfer and verify both closed runs before exit."""
import argparse
import contextlib
import io
import json
import time
import ops_corner as m

ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);ap.add_argument('--left');ap.add_argument('--right');ap.add_argument('--prepared',action='store_true');a=ap.parse_args()
assert 1<=a.pair<=6
if not a.prepared:
    m.prepare_pair(a.pair,a.left,a.right);m.parallel.networks();m.parallel.isolation()
started=time.monotonic();launched=set();last=None
while time.monotonic()-started<2150:
    with contextlib.redirect_stdout(io.StringIO()):
        snapshot=json.loads(m.remote(m.PREFIX+r'''
from pathlib import Path
import json
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text());result=[]
for row in plan['instances']:
 p=root/row['run_id'];r=json.loads((p/'result.json').read_text()) if (p/'result.json').exists() else None
 c=(r or {}).get('last_control') or (json.loads((p/'control_heartbeat.json').read_text()) if (p/'control_heartbeat.json').exists() else {})
 st=(c.get('large_recovery') or {}).get('state',{})
 result.append(dict(side=row['side'],exists=p.exists(),authorized=(p/'drive_authorized.json').exists(),complete=r is not None and (p/'transfer_manifest.json').exists(),status=(r or {}).get('status'),fault=c.get('fault'),stage=st.get('stage'),completed=st.get('completed_events'),skipped=st.get('skipped_sites'),progress_m=(c.get('projection') or {}).get('s_m'),error=(r or {}).get('error')))
print(json.dumps(result))
''',timeout=30))
    if not snapshot[0]['exists']:
        m.start('left');launched.add('left')
    elif not snapshot[1]['exists'] and snapshot[0]['authorized'] and not snapshot[0]['complete']:
        m.start('right');launched.add('right')
    elif snapshot[0]['complete'] and not snapshot[1]['exists']:
        raise RuntimeError('FIRST_INSTANCE_DID_NOT_ADMIT_PAIR_PRESERVE_FOR_DIAGNOSIS')
    key=[{k:r[k] for k in ('side','authorized','complete','status','stage','completed','skipped','error')} for r in snapshot]
    if key!=last:
        print(json.dumps(dict(pair=a.pair,runs=snapshot)),flush=True);last=key
    if all(r['complete'] for r in snapshot):break
    time.sleep(5)
else:
    raise TimeoutError('PAIR_OUTER_LIMIT_LEAVE_BOUNDED_SUPERVISORS_TO_STOP')
m.parallel.remove_networks()
m.ship(a.pair)
print(json.dumps(dict(pair=a.pair,transferred_verified=True,results=snapshot)),flush=True)
