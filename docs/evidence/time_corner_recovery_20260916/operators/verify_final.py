"""Verify source, evidence, split identity and materialized labels in native WSL."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import numpy as np

REPO = Path.cwd()
OUT = REPO.parent / 'runs/time_corner_recovery_20260916'
EVIDENCE = REPO / 'docs/evidence/time_corner_recovery_20260916'
RUNTIME = '8cf58f10d797d95ba542328948be75edb5dc9a92'


def read(path: Path):
    return json.loads(path.read_bytes())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert REPO == Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
    assert not subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip()
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    assert not subprocess.check_output(['git', 'diff', RUNTIME, head, '--', 'src', 'tools', 'configs', 'tests'])
    manifest = read(EVIDENCE / 'manifest.json')
    for relative, expected in manifest['files'].items():
        path = EVIDENCE / relative
        assert path.stat().st_size == expected['bytes'] and sha(path) == expected['sha256'], relative
    gate = read(EVIDENCE / 'test_gate_8cf58f1.json')
    assert gate['commit'] == RUNTIME and gate['full_exit'] == 0
    assert sha(EVIDENCE / 'full_8cf58f1.log') == gate['log_sha256']
    index = read(OUT / 'collection_index.json')
    assert sha(OUT / 'collection_index.json') == sha(EVIDENCE / 'collection_index.json')
    assert not set(index['splits']['train']) & set(index['splits']['validation'])
    assert sha(OUT / 'coverage_final.json') == index['coverage_sha256']
    for row in read(EVIDENCE / 'reference_manifest.json').values():
        reference_path = Path(row['native_path'])
        assert reference_path.stat().st_size == row['bytes'] and sha(reference_path) == row['sha256']
        reference = read(reference_path)
        assert reference['large_recovery']['map_screen_pass'] == row['map_screen_pass']
        assert reference['large_recovery']['config'] == row['config']
    anchors = 0
    for row in index['runs']:
        raw = Path(row['raw_path'])
        result = read(raw / 'result.json')
        assert result['nodes']['closed_bag']
        if row['end_condition'] == 'STOP_CONFIRMED':
            assert result['last_control']['stop_confirmed']
        else:
            assert row['end_condition'] == 'NEVER_AUTHORIZED_STATIONARY_STARTUP_FAILURE'
            assert result['status'] == 'FAILED' and row['accepted'] == 0
            assert not (raw / 'drive_authorized.json').exists()
            assert result['last_control']['armed_ns'] is None
            assert abs(result['last_control']['speed_mps']) <= 1e-4
            assert result['last_control']['max_speed_mps'] <= 1e-4
            controls = [json.loads(line) for line in (raw / 'control.jsonl').read_text().splitlines()]
            assert not any((r.get('large_recovery') or {}).get('applied') for r in controls)
        assert sha(raw / 'control.jsonl') == row['control_sha256']
        assert sha(raw / 'reference.json') == row['reference_sha256']
        if not row['accepted']:
            continue
        assert result['status'] == 'COMPLETE_LAP' and result['last_control']['fault'] is None
        labels = Path(row['materialized_path']) / 'teachers.npz'
        assert sha(labels) == row['teachers_sha256']
        with np.load(labels) as values:
            assert values['xy_m'].shape == (row['accepted'], 30, 2)
            assert values['xy_mask'].all() and np.isfinite(values['xy_m']).all()
        anchors += row['accepted']
    host = read(OUT / 'host_final_checks.json')
    campaign = read(OUT / 'campaign_final.json')
    assert host['original_repo_unchanged'] and host['awsim_files_unchanged']
    assert host['no_running_containers'] and host['no_owned_supervisors']
    assert host['new_raw_removed_only_after_verified_transfer']
    assert host['collection_index_sha256'] == sha(OUT / 'collection_index.json')
    assert campaign['sealed'] and all(r['state'] == 'WSL_MOVED' for r in campaign['attempts'])
    proof = dict(status='FINAL_VERIFICATION_PASS', head=head, runtime_source_commit=RUNTIME,
                 runtime_source_unchanged=True, evidence_files_verified=len(manifest['files']),
                 evidence_manifest_sha256=sha(EVIDENCE / 'manifest.json'),
                 collection_index_sha256=sha(OUT / 'collection_index.json'), teacher_anchors=anchors,
                 teacher_shape='[N, 30, 2]', all_teacher_masks_valid=True, split_overlap=False,
                 totals=index['totals'], missing_mandatory_corners=index['missing_mandatory_corners'],
                 all_mandatory_corners_covered=index['all_mandatory_corners_covered'])
    with args.output.open('x') as stream:
        json.dump(proof, stream, indent=2, allow_nan=False)
    print(json.dumps(proof), flush=True)


if __name__ == '__main__':
    main()
