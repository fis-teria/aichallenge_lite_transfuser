"""Identify actual admission/preparation failures and raw fault measurements."""
from pathlib import Path
import argparse
import json
import math
import sqlite3
from rosbags.typesys import Stores, get_typestore, get_types_from_idl

parser=argparse.ArgumentParser();parser.add_argument('--pair',type=int,required=True);args=parser.parse_args()
root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap_20260916';raw=root/'raw/time_corner_gap_20260916'
read=lambda p:json.loads(p.read_bytes())
records=[]
for domain in (1,2):
    name=f'codex-time-recovery-cornergap-p{args.pair:02}-d{domain}'
    report=read(out/(name+'_collection_summary.json'));run=raw/name
    rows=[json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    events={e['event_id']:e for e in report['events']};seen=set();misses=[]
    for r in rows:
        lr=r.get('large_recovery') or {};st=lr.get('state') or {};reason=st.get('reason');eid=st.get('event_id')
        if not lr.get('applied') or reason not in ('TARGET_NOT_REACHED','PREPARATION_SPEED_LIMIT','HEADING_LIMIT','LATERAL_LIMIT','PREPARATION_TIMEOUT','RECOVERY_NOT_CONFIRMED','RECOVERY_NOT_STABLE_AT_END') or (eid,reason) in seen:
            continue
        seen.add((eid,reason));event=events[eid]
        misses.append(dict(event_id=eid,site_id=event['site_id'],reason=reason,sim_ns=r['sim_ns'],
            s_m=r['projection']['s_m'],speed_mps=r['speed_mps'],lateral_m=lr['lateral_error_m'],
            heading_rad=lr['heading_error_rad'],requested_offset_m=event['target_offset_m'],requested_heading_rad=event['target_heading_rad']))
    fault=report['fault'];first=next((r for r in rows if fault is not None and r.get('reason')==fault),None)
    fault_summary=None
    if first is not None:
        fault_summary={k:first.get(k) for k in ('sim_ns','reason','phase','projection','speed_mps','issued_angle_rad','nominal_angle_rad','measured_steering_rad','guard')}
    steering=[]
    if fault=='SWEEP_VEHICLE_STATE':
        capture=first['input_timing_ns']['steering']['capture_ns']
        store=get_typestore(Stores.ROS2_HUMBLE);definitions={}
        for path in (root/'runs/time_recovery_collection_20260913/types').rglob('*.idl'):
            definitions.update(get_types_from_idl(path.read_text()))
        store.register(definitions)
        db=next((run/'bag').glob('*.db3'))
        with sqlite3.connect('file:'+str(db)+'?mode=ro&immutable=1',uri=True) as conn:
            topic,kind=conn.execute("SELECT id,type FROM topics WHERE name='/vehicle/status/steering_status'").fetchone()
            for receipt,blob in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic,)):
                msg=store.deserialize_cdr(bytes(blob),kind);t=int(msg.stamp.sec)*10**9+int(msg.stamp.nanosec)
                if abs(t-capture)<=50_000_000:
                    angle=float(msg.steering_tire_angle)
                    steering.append(dict(capture_ns=t,receipt_ns=receipt,value_rad=angle if math.isfinite(angle) else repr(angle),finite=math.isfinite(angle),selected_capture=t==capture))
        assert any(r['selected_capture'] for r in steering)
    records.append(dict(run_id=name,status=report['result_status'],accepted=report['accepted'],
        untriggered_sites=[p['site_id'] for p in report['planned'] if p['site_id'] not in {e['site_id'] for e in events.values()}],
        preparation_failures=misses,fault=fault,fault_state=fault_summary,raw_steering_near_fault=steering))
with (out/f'fault_diagnosis_pair{args.pair:02}.json').open('x') as stream:json.dump(records,stream,indent=2,allow_nan=False)
print(json.dumps(records),flush=True)
