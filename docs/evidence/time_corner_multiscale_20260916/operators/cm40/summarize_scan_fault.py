import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\n'+r'''
from pathlib import Path
import hashlib,json
out=Path(OUT);raw=Path(RAW);name='codex-time-recovery-corner40-p04-d2';run=raw/name
snapshot=json.loads((out/'corner40_pair04_20260916_snapshot.json').read_text())
control=run/'control.jsonl';digest=hashlib.sha256(control.read_bytes()).hexdigest()
assert digest==next(r for r in snapshot['runs'] if r['run_id']==name)['files']['control.jsonl']['sha256']
rows=[json.loads(s) for s in control.read_text().splitlines()]
fault=next(r for r in rows if r.get('reason')=='STALE_scan');state=fault['large_recovery']['state']
report=dict(run_id=name,status=json.loads((run/'result.json').read_text())['status'],fault='STALE_scan',control_sha256=digest,
 first_fault_sim_ns=fault['sim_ns'],first_fault_monotonic_ns=fault['monotonic_ns'],first_fault_progress_m=fault['projection']['s_m'],
 speed_mps=fault['speed_mps'],decision_wall_ms=fault['decision_wall_ms'],thread_cpu_ms=fault['thread_cpu_ms'],
 used_scan=fault['input_timing_ns']['scan'],newest_received_scan_capture_ns=fault['latest_received_capture_ns']['scan'],
 completed_events_at_fault=state['completed_events'],last_event_release_ns=state['release_ns'],
 margin_after_last_event_tail_s=(fault['sim_ns']-state['next_allowed_ns'])/1e9,
 final_stop_confirmed=json.loads((run/'result.json').read_text())['last_control']['stop_confirmed'],
 adopted_teacher_anchors=0,raw_retained=True,
 interpretation='The selected scan became stale during a slow control decision; a newer scan arrived. The exact source of scheduling delay is not established. This is not an acquisition speed-limit stop. Completed earlier event candidates remain in the preserved raw run but are not promoted under the unchanged complete-lap/fault-free audit policy.')
with (out/'p04_scan_fault_diagnosis.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report))
''',native=True,lock=True,timeout=90)
