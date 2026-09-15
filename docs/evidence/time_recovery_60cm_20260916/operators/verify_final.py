"""Read-only final verification from the native WSL checkout under its lock."""
from pathlib import Path
import argparse,hashlib,json,subprocess
import numpy as np

REPO=Path.cwd()
OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
EVIDENCE=REPO/'docs/evidence/time_recovery_60cm_20260916'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_bytes())
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path);args=ap.parse_args()
assert REPO==Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
runtime='adfc444818a26bae021d463cca5312b5d37a9f9a'
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert not subprocess.check_output(['git','diff',runtime,head,'--','src','tools','configs','tests'])
manifest=read(EVIDENCE/'manifest.json')
for rel,expected in manifest['files'].items():
    p=EVIDENCE/rel;assert p.stat().st_size==expected['bytes'] and sha(p)==expected['sha256'],rel
gate=read(EVIDENCE/'test_gate.json')
assert gate['commit']==runtime and gate['full_exit']==0 and sha(EVIDENCE/'full_runtime_tests.log')==gate['log_sha256']
index=read(OUT/'collection_index.json');assert sha(OUT/'collection_index.json')==sha(EVIDENCE/'collection_index.json')
assert not set(index['splits']['train'])&set(index['splits']['validation'])
count=0;events=0;bands=0
for row in index['runs']:
    raw=Path(row['raw_path']);result=read(raw/'result.json')
    assert result['nodes']['closed_bag'] and result['last_control']['stop_confirmed']
    assert sha(raw/'control.jsonl')==row['control_sha256'] and sha(raw/'reference.json')==row['reference_sha256']
    assert row['event_cap']==3
    if not row['accepted']:continue
    assert result['status']=='COMPLETE_LAP' and result['last_control']['fault'] is None
    labels=Path(row['materialized_path'])/'teachers.npz';assert sha(labels)==row['teachers_sha256']
    with np.load(labels) as z:
        assert z['xy_m'].shape==(row['accepted'],30,2) and z['xy_mask'].all() and np.isfinite(z['xy_m']).all()
    count+=row['accepted'];events+=row['accepted_events'];bands+=row['target_band_anchors']
host=read(OUT/'host_final_checks.json');campaign=read(OUT/'campaign_final.json')
assert host['original_repo_unchanged'] and host['no_running_containers'] and host['new_raw_removed_only_after_verified_transfer']
assert host['collection_index_sha256']==sha(OUT/'collection_index.json') and campaign['sealed']
assert all(r['state']=='WSL_MOVED' for r in campaign['attempts'])
proof=dict(status='FINAL_VERIFICATION_PASS',head=head,runtime_source_commit=runtime,runtime_source_unchanged=True,
 evidence_files_verified=len(manifest['files']),evidence_manifest_sha256=sha(EVIDENCE/'manifest.json'),
 collection_index_sha256=sha(OUT/'collection_index.json'),teacher_anchors=count,accepted_recovery_events=events,
 target_55_to_65cm_anchors=bands,teacher_shape='[N, 30, 2]',all_teacher_masks_valid=True,
 split_overlap=False,split_sides=index['split_sides'],totals=index['totals'],full_runtime_test_log_sha256=gate['log_sha256'])
if args.output:
    with args.output.open('x') as f:json.dump(proof,f,indent=2)
print(json.dumps(proof))
