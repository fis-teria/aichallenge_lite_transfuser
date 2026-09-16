"""Index verified corner collection; incomplete target coverage stays explicit."""
from pathlib import Path
import hashlib
import json
import subprocess

import numpy as np

from aic_transfuser_lite.data.time_corner_recovery_v1 import corner_coverage
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite, collection_speed_eligible

OUT = Path('/home/thistle/e2e_autonomous/runs/time_corner_gap2_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_gap2_20260916')


def read(path: Path):
    return json.loads(path.read_bytes())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    verified = {}
    receipts = {}
    for path in sorted(OUT.glob('cornergap2_pair*_verified.json')):
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
        assert name.startswith('codex-time-recovery-cornergap2-p')
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
        row['failed_site_policy'] = read(raw/'reference.json')['large_recovery']['config'].get('failed_site_policy', 'finish_without_more_events_v1')
        row['map_screen_policy'] = read(raw/'reference.json')['large_recovery']['config']['map_screen_policy']
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
    screen = read(OUT / 'candidate_screen.json')
    coverage = {}
    amplitude_totals = {}
    for cm in ('20', '40', '60'):
        combined = list(audits)
        prior_roots = [OUT.parent / ('time_corner_multiscale'+cm+'_20260916'), OUT.parent/'time_corner_gap_20260916']
        if cm == '20':
            prior_roots.append(OUT.parent / 'time_corner_recovery_20260916')
        for prior in prior_roots:
            for path in sorted(prior.glob('*_collection_summary.json')):
                report = read(path)
                states = read(prior / (report['run_id']+'_anchor_states.json')) if report['accepted'] else []
                combined.append(dict(report, anchor_states=states))
        sites = [LargeRecoverySite(**c['site']) for c in screen['catalogs'][cm]['corners']]
        coverage[cm] = corner_coverage(sites, combined)
        amplitude_totals[cm] = {}
        for split in ('train', 'validation'):
            matched = [a for audit in audits if audit['split'] == split for a in audit['anchor_states']
                       if abs(abs(a['requested_offset_m']) - int(cm)/100.) < 1e-8]
            events = {(audit['run_id'], a['event_id']) for audit in audits if audit['split'] == split
                      for a in audit['anchor_states'] if abs(abs(a['requested_offset_m']) - int(cm)/100.) < 1e-8}
            amplitude_totals[cm][split] = dict(anchors=len(matched), events=len(events),
                target_band_anchors=sum(a['target_band'] for a in matched))
    assert sum(t['anchors'] for cm in amplitude_totals.values() for t in cm.values()) == sum(t['anchors'] for t in totals.values())
    newly_covered = {cm:{split:sorted(set(screen['missing_before'][cm][split])-set(report['missing'][split])) for split in ('train','validation')} for cm,report in coverage.items()}
    coverage = dict(amplitudes=coverage, newly_covered=newly_covered, missing_before=screen['missing_before'],
                   missing_after={cm: row['missing'] for cm,row in coverage.items()},
                   complete=all(row['complete'] for row in coverage.values()))
    with (OUT / 'coverage_final.json').open('x') as stream:
        json.dump(coverage, stream, indent=2, allow_nan=False)
    observations = []
    for path in sorted(OUT.glob('pair*_resource_monitor.jsonl')):
        samples = [json.loads(line) for line in path.read_text().splitlines()]
        moving = 0.
        for a, b in zip(samples, samples[1:]):
            dt = b['unix_s'] - a['unix_s']
            if not 0. < dt < 2.5:
                continue
            progress = []
            for before, after in zip(a['runs'], b['runs']):
                p, q = before.get('s_m'), after.get('s_m')
                progress.append(p is not None and q is not None and .03 < q-p < 5. and not after.get('fault'))
            if len(progress) == 2 and all(progress):
                moving += dt
        observations.append(dict(file=path.name, sha256=sha(path), samples=len(samples),
            simultaneously_moving_observed_wall_s=moving,
            minimum_available_memory_gib=min(r['memory_kb']['MemAvailable'] for r in samples)/2**20))
    with (OUT/'parallel_observation.json').open('x') as f:
        json.dump(dict(pairs=observations, ros_domain_ids=[1,2], scope='Two progressing vehicles in contemporaneous one-second host samples'),f,indent=2)
    index = dict(schema='observed_corner_recovery_collection_v1',
                 index_source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                 runtime_host='graneple@192.168.3.10', native_wsl_raw=str(RAW),
                 maximum_attempts=12, maximum_events_per_lap=3, fixed_target_kmh=5,
                 parallel_observation_sha256=sha(OUT/'parallel_observation.json'),
                 labels='Observed future positions: 30 xy points in meters over 3 seconds',
                 inputs='Causal camera, LiDAR and ego history at the actual shifted pose',
                 split_policy='Whole runs; same corners, not held-out-corner generalization',
                 collection_speed_policy='record_actual_v1',
                 recovery_windows_s=sorted({r['recovery_duration_s'] for r in rows}),
                 retained_physical_guard_domain_mps=6/3.6,
                 training_started=False, merged_with_existing_dataset=False,
                 receipts=receipts, splits=splits, totals=totals, totals_by_amplitude=amplitude_totals, runs=rows,
                 raw_bytes=sum(r['raw_bytes'] for r in rows),
                 maximum_accepted_control_sim_gap_ms=max_gap,
                 all_raw_file_hashes_and_structure_verified=True,
                 all_sqlite_quick_checks_passed=True, all_runs_closed_and_safe_end_verified=True,
                 end_conditions={r['run_id']: r['end_condition'] for r in rows},
                 coverage_sha256=sha(OUT / 'coverage_final.json'),
                 all_mandatory_corners_covered=coverage['complete'],
                 missing_mandatory_corners=coverage['missing_after'], newly_covered_conditions=newly_covered,
                 completed_laps=sum(r['status']=='COMPLETE_LAP' for r in rows),
                 above_legacy_accepted_anchor_count=sum(r['above_legacy_accepted_anchor_count'] for r in rows),
                 maximum_measured_speed_kmh=max(r['maximum_measured_speed_mps'] for r in rows)*3.6)
    with (OUT / 'collection_index.json').open('x') as stream:
        json.dump(index, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(attempts=len(rows), totals=totals, raw_bytes=index['raw_bytes'],
                          missing=coverage['missing_after'], maximum_control_sim_gap_ms=max_gap)), flush=True)


if __name__ == '__main__':
    main()
