"""Validate packed evidence, tested source and all adopted native teachers."""
from pathlib import Path
import hashlib
import json
import math
import subprocess
import numpy as np

REPO = Path.cwd()
ROOT = REPO.parent
PACK = REPO/'docs/evidence/time_corner_gap_20260916'
OUT = ROOT/'runs/time_corner_gap_20260916'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
manifest = read(PACK/'manifest.json')
for name, row in manifest['files'].items():
    path = PACK/name
    assert path.stat().st_size == row['bytes'] and sha(path) == row['sha256'], name
source = 'd591c8f94d384df63748b3132b53c0c74900bf49'
mid = '62dbe8b1f6c3c62e558222a4bcd85c4c44adae67'
prior = 'cfd93663326dbb76c83027b5913cb9cefb97f8c1'
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
assert not subprocess.check_output(['git','diff',source,'HEAD','--','src','tools','tests'])
index = read(PACK/'collection_index.json')
assert sha(OUT/'collection_index.json') == sha(PACK/'collection_index.json')
host = read(PACK/'host_final_checks.json')
assert all(host[k] for k in ('awsim_files_unchanged','original_repo_unchanged','no_running_containers','no_owned_supervisors'))
assert host['collection_index_sha256'] == sha(PACK/'collection_index.json')
for name, commit in [('test_gate.json',prior),('test_gate_62dbe8b.json',mid),('test_gate_d591c8f.json',source)]:
    gate = read(PACK/name)
    assert gate['commit'] == commit and gate['full_exit'] == 0
    assert sha(PACK/Path(gate['log']).name) == gate['log_sha256']
for rel, row in read(PACK/'reference_manifest.json').items():
    path = OUT/rel
    assert path.stat().st_size == row['bytes'] and sha(path) == row['sha256'], rel
assert not set(index['splits']['train']) & set(index['splits']['validation'])
anchors = events = 0
for row in index['runs']:
    raw = Path(row['raw_path'])
    expected = prior if '-p01-' in row['run_id'] else mid if any(p in row['run_id'] for p in ('-p02-', '-p03-')) else source
    assert row['source_sha'] == expected
    assert sha(raw/'control.jsonl') == row['control_sha256']
    assert sha(raw/'reference.json') == row['reference_sha256']
    cfg = read(raw/'reference.json')['large_recovery']['config']
    assert cfg['speed_policy'] == 'record_actual_v1' and cfg['map_screen_policy'] == 'oriented_body_v1'
    assert cfg['recovery_duration_s'] == 15. and cfg['entry_heading_tolerance_rad'] == math.radians(2.)
    assert cfg.get('failed_site_policy','finish_without_more_events_v1') == ('finish_without_more_events_v1' if expected==prior else 'continue_after_target_miss_v1')
    if row['accepted']:
        path = Path(row['materialized_path'])/'teachers.npz'
        assert sha(path) == row['teachers_sha256']
        with np.load(path) as labels:
            assert labels['xy_m'].shape == (row['accepted'],30,2)
            assert labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
        anchors += row['accepted']
        events += row['accepted_events']
assert anchors == sum(t['anchors'] for t in index['totals'].values())
assert events == sum(t['events'] for t in index['totals'].values())
report = dict(status='CORNER_GAP_FINAL_VERIFICATION_PASS',
    verified_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(), tested_runtime_source=source,
    evidence_files=len(manifest['files']), runs=len(index['runs']), accepted_anchors=anchors, accepted_events=events,
    shape='[N,30,2]', full_future_masks=True, run_splits_disjoint=True, awsim_unchanged=True,
    all_requested_entry_states_covered=index['all_mandatory_corners_covered'])
with (OUT/'final_verification.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report))
