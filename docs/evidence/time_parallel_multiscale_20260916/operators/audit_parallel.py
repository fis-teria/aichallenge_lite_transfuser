"""Reuse the sealed 60 cm auditor after the training lock is released."""
import ops as m

m.remote(r'''
from pathlib import Path
import json,subprocess,sys,hashlib
out=Path('/home/thistle/e2e_autonomous/runs/time_recovery_parallel_20260916')
raw=Path('/home/thistle/e2e_autonomous/raw/time_recovery_parallel_20260916')
base=Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
assignments=[('codex-time-recovery-parallel-d1-left-r01','train'),
 ('codex-time-recovery-parallel-d2-right-r01','validation'),
 ('codex-time-recovery-parallel-d1-left-r02','validation'),
 ('codex-time-recovery-parallel-d2-right-r02','validation')]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
verified={}
for receipt in sorted(out.glob('*_verified.json')):
    proof=json.loads(receipt.read_text())
    assert proof['all_files_and_directory_structure_identical'] and proof['all_sqlite_quick_checks_passed']
    for r in proof['runs']:
        assert r['run_id'] not in verified
        verified[r['run_id']]=r
assert set(verified)=={n for n,_ in assignments}
for name,split in assignments:
    script=('import importlib.util;from pathlib import Path;'
        's=importlib.util.spec_from_file_location("audit",'+repr(str(base/'audit60.py'))+');'
        'a=importlib.util.module_from_spec(s);s.loader.exec_module(a);'
        'a.OUT=Path('+repr(str(out))+');a.RAW=Path('+repr(str(raw))+');a.audit('+repr(name)+','+repr(split)+')')
    log=out/(name+'_audit_execution.log')
    with log.open('x') as stream:
        p=subprocess.run([sys.executable,'-u','-c',script],stdout=stream,stderr=subprocess.STDOUT,timeout=1800)
    if p.returncode:raise RuntimeError(log.read_text()[-7000:])
    clock=out/(name+'_clock.json')
    with clock.with_suffix('.log').open('x') as stream:
        p=subprocess.run([sys.executable,str(base/'clock_audit.py'),'--raw',str(raw/name),'--output',str(clock)],stdout=stream,stderr=subprocess.STDOUT,timeout=600)
    if p.returncode:raise RuntimeError(clock.with_suffix('.log').read_text()[-7000:])
    summary=json.loads((out/(name+'_collection_summary.json')).read_text())
    print(json.dumps(dict(run=name,split=split,accepted=summary['accepted'],three_event_qualified=summary['three_event_qualified'])),flush=True)
rows=[]
for name,split in assignments:
    p=out/(name+'_collection_summary.json');v=json.loads(p.read_text())
    rows.append(dict(run_id=name,split=split if v['accepted'] else 'excluded',target_abs_lateral_m=.6,
        status=v['result_status'],fault=v['fault'],accepted=v['accepted'],target_band_anchors=v['target_band_anchors'],
        three_event_qualified=v['three_event_qualified'],accepted_events=sum(e.get('accepted_camera_anchors',0)>0 for e in v['events']),
        summary_sha256=sha(p),raw_bytes=verified[name]['regular_file_bytes'],clock_sha256=sha(out/(name+'_clock.json'))))
pilot=[r for r in rows if r['run_id'].endswith('r02')]
index=dict(schema='parallel_recovery_collection_audit_v1',runs=rows,
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    reused_auditor_sha256=sha(base/'audit60.py'),reused_clock_auditor_sha256=sha(base/'clock_audit.py'),
    all_raw_hashes_and_sqlite_checks_passed=True,new_data_added_to_active_training=False,
    parallel_pair_qualified=all(r['three_event_qualified'] for r in pilot),
    parallel_pair_accepted=sum(r['accepted'] for r in pilot),parallel_pair_target_band_anchors=sum(r['target_band_anchors'] for r in pilot),
    split_policy='Whole runs; initial left calibration train, both simultaneous laps reserved validation; failed preflight excluded. Same sites, not held-out-site generalization.')
with (out/'collection_index.json').open('x') as f:json.dump(index,f,indent=2)
print(json.dumps(index),flush=True)
''',native=True,lock=True,timeout=6000)
