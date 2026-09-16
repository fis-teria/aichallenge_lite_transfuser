"""Record the first-pair readiness reset from hash-verified observations."""
import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\n'+r'''
from pathlib import Path
import hashlib,json,math
out=Path(OUT);raw=Path(RAW);receipt=json.loads((out/'corner40_pair01_20260916_verified.json').read_text())
assert receipt['all_files_and_directory_structure_identical']
snapshot=json.loads((out/'corner40_pair01_20260916_snapshot.json').read_text());reports=[]
for domain in (1,2):
 name=f'codex-time-recovery-corner40-p01-d{domain}';control=raw/name/'control.jsonl'
 digest=hashlib.sha256(control.read_bytes()).hexdigest()
 assert digest==next(r for r in snapshot['runs'] if r['run_id']==name)['files']['control.jsonl']['sha256']
 rows=[]
 for line in control.read_text().splitlines():
  r=json.loads(line);s=(r.get('projection') or {}).get('s_m');large=r.get('large_recovery') or {};state=large.get('state') or {}
  if s is None or not 221.5<=s<=223.5 or not large:continue
  stable=state.get('stable_since_ns');heading=math.degrees(large['heading_error_rad'])
  rows.append(dict(s_m=s,sim_ns=r['sim_ns'],monotonic_ns=r['monotonic_ns'],speed_mps=r['speed_mps'],lateral_m=large['lateral_error_m'],heading_deg=heading,stable_since_ns=stable,stable_s=(r['sim_ns']-stable)/1e9 if stable is not None else None,stage=state['stage'],skipped=state['skipped_sites'],minimum_ray_margin_m=r['guard'].get('minimum_ray_margin_m'),reason=state.get('reason')))
 reports.append(dict(run_id=name,control_sha256=digest,start_window_m=[222.,223.],old_entry_heading_tolerance_deg=1,new_entry_heading_tolerance_deg=2,rows=rows))
with (out/'c07_start_window_diagnosis.json').open('x') as f:json.dump(dict(status='OBSERVED_READINESS_RESET_NOT_SPEED_ABORT',runs=reports,interpretation='Counterfactual threshold diagnosis only; later runtime collection must independently prove successful entry and recovery.'),f,indent=2)
print(json.dumps([dict(run=r['run_id'],resets=[p for p in r['rows'] if p['stable_since_ns'] is None]) for r in reports]))
''',native=True,lock=True,timeout=90)
