"""Verify the complete finite collection and publish a native WSL handoff index."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path('/home/thistle/e2e_autonomous')
PLAN_SHA = '04d757dde75e6b42062a6966d5f9d05bdad1649c7e4d51980f47ded5f68bb462'


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024**2), b''):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    out = ROOT/'runs/time_recovery_expansion_20260914'
    raw = ROOT/'raw/time_recovery_expansion_20260914'
    destination = out/'collection_index.json'
    assert not destination.exists()
    plan_path = Path(__file__).with_name('production_plan.json')
    assert sha(plan_path) == PLAN_SHA
    plan = json.loads(plan_path.read_text())
    source_hashes = {str(plan_path): PLAN_SHA}

    def read(path: Path) -> dict:
        source_hashes[str(path)] = sha(path)
        return json.loads(path.read_text())

    calibration = read(out/'calibration_summary.json')
    assert calibration['all_calibration_gates_pass']
    assert sha(out/'calibration_summary.json') == plan['calibration_summary_sha256']
    shipping = []
    raw_ids: set[str] = set()
    for number in range(1, 8):
        prefix = f'pair{number:02d}_20260914'
        verified = read(out/(prefix+'_verified.json'))
        sent = read(out/(prefix+'_shipping.json'))
        assert verified['all_files_and_directory_structure_identical'] and verified['all_sqlite_quick_checks_passed']
        assert verified['archive_sha256'] == sent['archive_sha256']
        assert verified['snapshot_sha256'] == sent['snapshot_sha256'] == sha(out/(prefix+'_snapshot.json'))
        ids = [r['run_id'] for r in verified['runs']]
        assert ids == sent['run_ids'] and not raw_ids.intersection(ids)
        assert sum(r['regular_file_bytes'] for r in verified['runs']) == sent['source_regular_file_bytes']
        assert (out/(prefix+'.tar.gz')).stat().st_size == sent['archive_bytes']
        raw_ids.update(ids); shipping.append(sent)

    rows = []
    all_anchor_ids: set[str] = set()
    materialized_ids: set[str] = set()
    for number in range(2, 8):
        prefix = f'pair{number:02d}_20260914'
        summary = read(out/(prefix+'_summary.json'))
        prepared = read(out/(prefix+'_prepared.json'))
        assigned = plan['runs'][2*(number-2):2*(number-1)]
        assert prepared['status'] == 'PASS' and prepared['production_plan_sha256'] == PLAN_SHA
        assert prepared['independently_collected_run_ids'] == [r['run_id'] for r in assigned]
        assert [r['run_id'] for r in summary['runs']] == [r['run_id'] for r in assigned]
        expected_prepared = {r['run_id'] for r in assigned if r['split'] != 'evaluation_reserved'}
        assert {r['run_id'] for r in prepared['runs']} == expected_prepared
        for assigned_row, state in zip(assigned, summary['runs']):
            name = assigned_row['run_id']; split = assigned_row['split']; run = raw/name
            result = read(run/'result.json')
            probe = read(out/(name.split('-')[-1]+'_causal_probe.json'))
            assert result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag'] and not result['cleanup_errors']
            assert result['last_control']['stop_confirmed'] and result['last_control']['fault'] is None
            assert result['source_sha'] == plan['source_commit']
            assert probe['all_phase_candidates_audited'] and probe['raw_manifest_sha256'] == sha(run/'transfer_manifest.json')
            assert state['recovery']['confirmed'] and state['target_count_agreeing_with_both_nominals'] > 0
            anchors = [r['anchor_id'] for r in probe['anchors'] if r['usable_full'] and r.get('phase_and_xy_full')]
            assert len(anchors) == state['accepted_primary']['count'] and len(set(anchors)) == len(anchors)
            assert not all_anchor_ids.intersection(anchors); all_anchor_ids.update(anchors)
            targets = state['target_anchor_ids_both_nominals']
            assert set(targets) <= set(anchors) and len(targets) == state['target_count_agreeing_with_both_nominals']
            generated = split != 'evaluation_reserved'
            if generated:
                info = next(r for r in prepared['runs'] if r['run_id'] == name)
                assert info['split'] == split and info['anchors'] == info['input_valid'] == len(anchors)
                assert not info['input_reason_refinements'] and info['real_input_replay_equal']
                for relative, digest in info['files'].items():
                    path = out/relative
                    assert path.resolve().is_relative_to(out) and sha(path) == digest, relative
                cache = out/'prepared'/split/name
                with np.load(cache/'labels.npz', allow_pickle=False) as labels:
                    assert labels['xy_m'].shape == (len(anchors), 30, 2)
                    assert np.isfinite(labels['xy_m']).all() and labels['xy_mask'].all()
                with np.load(cache/'inputs.npz', allow_pickle=False) as inputs:
                    assert inputs['input_valid'].shape == (len(anchors),) and inputs['input_valid'].all()
                actual = [json.loads(s) for s in (cache/'anchors.jsonl').read_text().splitlines()]
                assert [r['anchor_id'] for r in actual] == anchors
                assert all(r['run_id'] == name and r['split'] == split for r in actual)
                materialized_ids.add(name)
            else:
                assert not (out/'materialized'/name).exists()
                assert not any((out/'prepared'/s/name).exists() for s in ('train', 'validation', 'evaluation_reserved'))
            rows.append(dict(run_id=name, side=assigned_row['side'], split=split, raw_path=str(run),
                accepted_anchors=len(anchors), target_anchors=len(targets), target_anchor_ids=targets,
                teachers_and_inputs_generated=generated, lap_seconds=state['lap_seconds'],
                recovery=state['recovery'], minimum_guard_ray_margin_m=state['minimum_guard_ray_margin_m']))
    assert {p.name for p in (out/'materialized').iterdir()} == materialized_ids
    assert raw_ids == {r['run_id'] for r in plan['runs']} | set(plan['calibration_run_ids'])
    groups = {split: dict(runs=sum(r['split'] == split for r in rows),
        accepted_anchors=sum(r['accepted_anchors'] for r in rows if r['split'] == split),
        target_anchors=sum(r['target_anchors'] for r in rows if r['split'] == split))
        for split in ('train', 'validation', 'evaluation_reserved')}
    assert [groups[s]['runs'] for s in groups] == [8, 2, 2]
    report = dict(status='COMPLETE_VERIFIED', format='outward_recovery_collection_index_v1',
        scope='SAME_COURSE_SAME_CORNER_SMALL_RECOVERY_RUN_HOLDOUT', production_plan_sha256=PLAN_SHA,
        runtime_source_commit=plan['source_commit'], analysis_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        groups=groups, production_runs=rows, calibration_run_ids=plan['calibration_run_ids'],
        calibration_accepted_anchors=calibration['total_accepted'], calibration_final_test_eligible=False,
        total_new_raw_runs=len(raw_ids), total_original_regular_file_bytes=sum(s['source_regular_file_bytes'] for s in shipping),
        total_archives_bytes=sum(s['archive_bytes'] for s in shipping), sources_verified=True,
        generated_teacher_frames=sum(r['accepted_anchors'] for r in rows if r['teachers_and_inputs_generated']),
        teacher_xy_shape_per_anchor=[30, 2], teacher_dt_s=.1, teacher_horizon_s=3.,
        model_trained=False, model_performance_evaluated=False, combined_training_identity_created=False,
        prepared_file_hashes_reverified=True, raw_archive_hashes_verified_at_transfer=True,
        raw_archive_hashes_repeated_at_finalization=False, source_report_hashes=source_hashes)
    with destination.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ('status', 'groups', 'generated_teacher_frames',
        'total_new_raw_runs', 'total_original_regular_file_bytes', 'total_archives_bytes')}, indent=2))


if __name__ == '__main__':
    main()
