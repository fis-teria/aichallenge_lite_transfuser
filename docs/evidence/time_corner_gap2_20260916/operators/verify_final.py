"""Native validation of compact proof, runtime revisions, and adopted teachers."""
from pathlib import Path
import hashlib,json,math,subprocess
import numpy as np

repo=Path.cwd();out=repo.parent/'runs/time_corner_gap2_20260916';pack=repo/'docs/evidence/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=read(pack/'manifest.json')
for name,row in manifest['files'].items():
    p=pack/name;assert p.stat().st_size==row['bytes'] and sha(p)==row['sha256'],name
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
assert not subprocess.check_output(['git','diff','1b845eddad371c98aeb54d1176fd6493cc9c6ed4','HEAD','--','src','tools','tests'])
gates={}
for path in sorted(pack.glob('test_gate*.json')):
    gate=read(path);assert gate['full_exit']==0 and sha(pack/Path(gate['log']).name)==gate['log_sha256']
    gates[gate['commit']]=path.name
index=read(pack/'collection_index.json');assert sha(pack/'collection_index.json')==sha(out/'collection_index.json')
host=read(pack/'host_final_checks.json')
assert all(host[k] for k in ('awsim_files_unchanged','original_repo_unchanged','no_running_containers','no_owned_supervisors'))
assert host['collection_index_sha256']==sha(pack/'collection_index.json')
for rel,row in read(pack/'reference_manifest.json').items():
    p=out/rel;assert p.stat().st_size==row['bytes'] and sha(p)==row['sha256'],rel
assert not set(index['splits']['train'])&set(index['splits']['validation'])
anchors=events=0
for row in index['runs']:
    raw=Path(row['raw_path']);assert row['source_sha'] in gates
    assert sha(raw/'control.jsonl')==row['control_sha256'] and sha(raw/'reference.json')==row['reference_sha256']
    cfg=read(raw/'reference.json')['large_recovery']['config']
    assert cfg['speed_policy']=='record_actual_v1' and cfg['map_screen_policy']=='oriented_body_v1'
    assert cfg['recovery_duration_s']==15. and cfg['entry_heading_tolerance_rad']==math.radians(2.)
    assert cfg['failed_site_policy']=='continue_after_target_miss_v1'
    if row['accepted']:
        path=Path(row['materialized_path'])/'teachers.npz';assert sha(path)==row['teachers_sha256']
        with np.load(path) as labels:
            assert labels['xy_m'].shape==(row['accepted'],30,2) and labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
        anchors+=row['accepted'];events+=row['accepted_events']
assert anchors==sum(t['anchors'] for t in index['totals'].values()) and events==sum(t['events'] for t in index['totals'].values())
report=dict(status='CORNER_GAP2_FINAL_VERIFICATION_PASS',verified_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    tested_runtime_sources=gates,evidence_files=len(manifest['files']),runs=len(index['runs']),accepted_anchors=anchors,accepted_events=events,
    shape='[N,30,2]',full_future_masks=True,run_splits_disjoint=True,awsim_unchanged=True,all_requested_entry_states_covered=index['all_mandatory_corners_covered'])
with (out/'final_verification.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report))
