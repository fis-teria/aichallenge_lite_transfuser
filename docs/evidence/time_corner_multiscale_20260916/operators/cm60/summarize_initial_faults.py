import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\n'+r'''
from pathlib import Path
import hashlib,json
out=Path(OUT);raw=Path(RAW);reports=[]
for pair in (1,2):
 snapshot=json.loads((out/f'corner60_pair{pair:02}_20260916_snapshot.json').read_text())
 for domain in (1,2):
  name=f'codex-time-recovery-corner60-p{pair:02}-d{domain}';run=raw/name
  result=json.loads((run/'result.json').read_text());control=run/'control.jsonl'
  digest=hashlib.sha256(control.read_bytes()).hexdigest()
  assert digest==next(r for r in snapshot['runs'] if r['run_id']==name)['files']['control.jsonl']['sha256']
  fault=result['last_control'].get('fault');rows=[json.loads(s) for s in control.read_text().splitlines()]
  first=next((r for r in rows if fault is not None and r.get('reason')==fault),{})
  fields={k:first.get(k) for k in ('sim_ns','monotonic_ns','speed_mps','processing_ms','decision_wall_ms','thread_cpu_ms','stage_wall_ms','snapshot_retry_reasons','input_timing_ns','scan_history_ns')}
  fields['last_pose_stamps_ns']=first.get('pose_history_ns',[])[-12:]
  reports.append(dict(run_id=name,control_sha256=digest,status=result['status'],fault=fault,error=result.get('error'),source_sha=result['source_sha'],ever_authorized=(run/'drive_authorized.json').exists(),adopted_teacher_anchors=0,raw_retained=True,first_fault=fields))
monitor=out/'pair02_resource_monitor.jsonl'
samples=[json.loads(s) for s in monitor.read_text().splitlines()]
monitor_report=dict(sha256=hashlib.sha256(monitor.read_bytes()).hexdigest(),samples=len(samples),
 minimum_available_memory_gib=min(r['memory_kb']['MemAvailable'] for r in samples)/1024**2,
 maximum_package_temperature_c=max(r['temperature_millic'].get('x86_pkg_temp',0) for r in samples)/1000,
 package_thermal_throttle_delta=samples[-1]['package_throttle_count']['0']-samples[0]['package_throttle_count']['0'],
 swap_in_pages=samples[-1]['swap_pages']['pswpin']-samples[0]['swap_pages']['pswpin'],
 swap_out_pages=samples[-1]['swap_pages']['pswpout']-samples[0]['swap_pages']['pswpout'])
with (out/'initial_latency_diagnosis.json').open('x') as f:json.dump(dict(runs=reports,pair02_resources=monitor_report,quality_policy_unchanged=True,mitigation='Restore original CPU allocation; at most three timing snapshot retries with the unchanged 100ms deadline, actual bracketing poses, freshness and physical guards.'),f,indent=2)
schedule=json.loads((out/'collection_schedule_e2_c11.json').read_text())
schedule['pairs']=[dict(pair=1,left='train_lap01_h2_e2',right='validation_lap01_h2_e2',outcome='failed_latency'),dict(pair=2,left='train_lap02_h2_e2',right='validation_lap02_h2_e2',outcome='failed_alignment_and_stationary_startup'),dict(pair=3,left='train_lap01_h2_e2',right='validation_lap01_h2_e2'),dict(pair=4,left='train_lap02_h2_e2',right='validation_lap02_h2_e2'),dict(pair=5,left='train_lap03_late_h2_e2_c11',right='validation_lap03_late_h2_e2_c11')]
with (out/'collection_schedule_effective.json').open('x') as f:json.dump(schedule,f,indent=2)
print(json.dumps(dict(runs=[{k:r[k] for k in ('run_id','status','fault','adopted_teacher_anchors')} for r in reports],resources=monitor_report)))
''',native=True,lock=True,timeout=120)
