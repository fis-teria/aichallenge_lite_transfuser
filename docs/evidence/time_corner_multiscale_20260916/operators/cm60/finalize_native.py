"""Index verified corner collection; incomplete target coverage stays explicit."""
from pathlib import Path
import hashlib
import json
import subprocess

import numpy as np

from aic_transfuser_lite.data.time_corner_recovery_v1 import corner_coverage
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite, collection_speed_eligible

OUT = Path('/home/thistle/e2e_autonomous/runs/time_corner_multiscale60_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_multiscale60_20260916')


def read(path: Path):
    return json.loads(path.read_bytes())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    verified = {}
    receipts = {}
    for path in sorted(OUT.glob('corner60_pair*_verified.json')):
        receipt = read(path)
        assert receipt['all_files_and_directory_structure_identical']
        assert receipt['all_sqlite_quick_checks_passed']
        receipts[path.name] = dict(sha256=sha(path), archive_sha256=receipt['archive_sha256'],
                                  snapshot_sha256=receipt['snapshot_sha256'])
        for row in receipt['runs']:
            assert row['run_id'] not in verified
            assert Path(row['raw_path']) == RAW / row['run_id']
            verified[row['run_id']] = row
    assert 1 <= len(verified) <= 12
    assert len(list(OUT.glob('*_collection_summary.json'))) == len(verified)
    rows = []
    audits = []
    splits = {'train': [], 'validation': []}
    totals = {s: dict(anchors=0, target_band_anchors=0, events=0, runs=0) for s in splits}
    max_gap = 0.
    for name, verification in sorted(verified.items()):
        assert name.startswith('codex-time-recovery-corner60-p')
        raw = RAW / name
        report_path = OUT / (name + '_collection_summary.json')
        report = read(report_path)
        result = read(raw / 'result.json')
        assert report['result_status'] == result['status']
        assert report['fault'] == result['last_control'].get('fault')
        assert report['closed_bag']
        assert read(raw/'reference.json')['large_recovery']['config']['speed_policy']=='record_actual_v1'
        end_condition = 'STOP_CONFIRMED'
        if not report['stop_confirmed']:
            assert result['status'] == 'FAILED' and report['accepted'] == 0
            assert not (raw / 'drive_authorized.json').exists()
            assert result['last_control']['armed_ns'] is None
            assert abs(result['last_control']['speed_mps']) <= 1e-4
            assert result['last_control']['max_speed_mps'] <= 1e-4
            controls = [json.loads(line) for line in (raw / 'control.jsonl').read_text().splitlines()]
            assert not any((r.get('large_recovery') or {}).get('applied') for r in controls)
            end_condition = 'NEVER_AUTHORIZED_STATIONARY_STARTUP_FAILURE'
        assert not result['cleanup_errors']
        assert 1 <= report['event_cap'] <= 3
        assert result['fixed_target_mps'] == 5 / 3.6
        assert result['ros_domain_id'] == (1 if report['split'] == 'train' else 2)
        states = read(OUT / (name + '_anchor_states.json')) if report['accepted'] else []
        audits.append(dict(report, anchor_states=states))
        row = dict(run_id=name, split=report['split'], accepted=report['accepted'],
                   status=report['result_status'], fault=report['fault'],
                   judge_lap_confirmed=result.get('lap_confirmed', False),
                   stop_confirmed=bool(report['stop_confirmed']), end_condition=end_condition,
                   lap_times_s=[r['lap_seconds'] for r in result.get('judge_laps', [])],
                   event_cap=report['event_cap'], source_sha=result['source_sha'],
                   ros_domain_id=result['ros_domain_id'], raw_path=str(raw),
                   raw_bytes=verification['regular_file_bytes'],
                   target_band_anchors=report['target_band_anchors'],
                   completed_events=sum(bool(e['completed']) for e in report['events']),
                   accepted_events=sum(e.get('accepted_camera_anchors', 0) > 0 for e in report['events']),
                   end_reason=((result['last_control'].get('large_recovery') or {}).get('state') or {}).get('reason'),
                   sites=report['planned'], summary_sha256=sha(report_path),
                   control_sha256=sha(raw / 'control.jsonl'), reference_sha256=sha(raw / 'reference.json'),
                   clock_audit_sha256=sha(OUT / (name + '_clock.json')),
                   teacher_anchors_per_event=[dict(site_id=e['site_id'], corner_id=e.get('corner_id'),
                       accepted=e.get('accepted_camera_anchors', 0),
                       target_band=e.get('target_band_camera_anchors', 0)) for e in report['events']])
        row['maximum_measured_speed_mps'] = result['last_control']['max_speed_mps']
        row['collection_speed_policy'] = 'record_actual_v1'
        row['recovery_duration_s'] = read(raw/'reference.json')['large_recovery']['config'].get('recovery_duration_s',10.)
        row['above_legacy_event_speed_samples'] = sum(e['above_legacy_speed_samples'] for e in report['events'])
        row['above_legacy_accepted_anchor_count'] = sum(a['speed_mps'] > 1.4 for a in states)
        for event_row in row['teacher_anchors_per_event']:
            event = next(e for e in report['events'] if e['site_id'] == event_row['site_id'])
            site = LargeRecoverySite(**next(s for s in report['planned'] if s['site_id'] == event['site_id']))
            event_row['entry_state_anchors'] = sum(
                a['event_id'] == event['event_id'] and a['site_id'] == site.site_id
                and site.release_s_m - .5 <= a['base_s_m'] <= site.release_s_m + 3.
                and site.at_goal(a['lateral_m'], a['heading_rad']) and collection_speed_eligible(a['speed_mps'], event.get('speed_policy', 'bounded_5kmh_v1'))
                for a in states)
        if report['accepted']:
            assert result['status'] == 'COMPLETE_LAP' and row['fault'] is None
            assert len(states) == report['accepted'] == report['prepared']['anchors'] == report['prepared']['input_valid']
            assert not report['prepared']['input_reason_refinements']
            assert len({a['anchor_id'] for a in states}) == len(states)
            assert sum(a['target_band'] for a in states) == row['target_band_anchors']
            assert sum(e['accepted'] for e in row['teacher_anchors_per_event']) == len(states)
            destination = OUT / 'materialized' / name
            prepared = OUT / 'prepared' / report['split'] / name
            assert prepared.is_dir()
            with np.load(destination / 'teachers.npz') as labels:
                assert labels['xy_m'].shape == (len(states), 30, 2)
                assert labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
            row.update(materialized_path=str(destination), prepared_path=str(prepared),
                       teachers_sha256=sha(destination / 'teachers.npz'),
                       peak_abs_accepted_lateral_m=max(abs(a['lateral_m']) for a in states))
            splits[report['split']].append(name)
            total = totals[report['split']]
            total['anchors'] += len(states)
            total['target_band_anchors'] += row['target_band_anchors']
            total['events'] += row['accepted_events']
            total['runs'] += 1
            accepted_sites = {e['site_id'] for e in row['teacher_anchors_per_event'] if e['accepted']}
            for event in read(OUT / (name + '_clock.json'))['events']:
                if event['site_id'] in accepted_sites:
                    max_gap = max(max_gap, event['control_sim_gap']['max_ms'])
        rows.append(row)
    assert not set(splits['train']) & set(splits['validation'])
    catalog_path = (OUT/'catalog.json')
    sites = [LargeRecoverySite(**c['site']) for c in read(catalog_path)['corners']]
    coverage = corner_coverage(sites, audits)
    coverage['catalog_sha256'] = sha(catalog_path)
    with (OUT / 'coverage_final.json').open('x') as stream:
        json.dump(coverage, stream, indent=2, allow_nan=False)
    index = dict(schema='observed_corner_recovery_collection_v1',
                 index_source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                 runtime_host='graneple@192.168.3.10', native_wsl_raw=str(RAW),
                 maximum_attempts=12, maximum_events_per_lap=3, fixed_target_kmh=5,
                 labels='Observed future positions: 30 xy points in meters over 3 seconds',
                 inputs='Causal camera, LiDAR and ego history at the actual shifted pose',
                 split_policy='Whole runs; same corners, not held-out-corner generalization',
                 collection_speed_policy='record_actual_v1',
                 recovery_windows_s=sorted({r['recovery_duration_s'] for r in rows}),
                 retained_physical_guard_domain_mps=6/3.6,
                 training_started=False, merged_with_existing_dataset=False,
                 receipts=receipts, splits=splits, totals=totals, runs=rows,
                 raw_bytes=sum(r['raw_bytes'] for r in rows),
                 maximum_accepted_control_sim_gap_ms=max_gap,
                 all_raw_file_hashes_and_structure_verified=True,
                 all_sqlite_quick_checks_passed=True, all_runs_closed_and_safe_end_verified=True,
                 end_conditions={r['run_id']: r['end_condition'] for r in rows},
                 coverage_sha256=sha(OUT / 'coverage_final.json'),
                 all_mandatory_corners_covered=coverage['complete'],
                 missing_mandatory_corners=coverage['missing'])
    with (OUT / 'collection_index.json').open('x') as stream:
        json.dump(index, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(attempts=len(rows), totals=totals, raw_bytes=index['raw_bytes'],
                          missing=coverage['missing'], maximum_control_sim_gap_ms=max_gap)), flush=True)


if __name__ == '__main__':
    main()
