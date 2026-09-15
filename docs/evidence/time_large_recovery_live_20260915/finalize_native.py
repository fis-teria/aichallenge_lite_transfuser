"""Index verified real AWSIM recordings without merging or starting training."""
from pathlib import Path
import hashlib
import json
import subprocess

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_large_live_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_large_live_20260915')
PREFIXES = ['large_g03_left_r01', 'large_g03_comparison', 'large_g03_left_r03',
            'large_g03_final', 'large_g03_right_r03', 'large_g05_final']
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> None:
    verified = {}
    receipts = {}
    for prefix in PREFIXES:
        p = OUT / (prefix + '_verified.json')
        v = read(p)
        assert v['all_files_and_directory_structure_identical']
        assert v['all_sqlite_quick_checks_passed']
        receipts[p.name] = {'sha256': sha(p), 'archive_sha256': v['archive_sha256'],
                            'snapshot_sha256': v['snapshot_sha256']}
        for row in v['runs']:
            name = row['run_id']
            assert name not in verified and Path(row['raw_path']) == RAW / name
            verified[name] = row
    assert len(verified) == 8
    reports = {name: read(OUT / (name + '_collection_summary.json')) for name in verified}
    assert len(list(OUT.glob('*_collection_summary.json'))) == len(reports)
    splits = {'train': [], 'validation': []}
    totals = {s: dict(anchors=0, target_band_anchors=0, accepted_events=0, runs=0)
              for s in splits}
    rows = []
    bands = {s: {'under_5cm': 0, '5_to_15cm': 0, '15_to_25cm': 0, '25cm_or_more': 0}
             for s in splits}
    for name, report in reports.items():
        result = read(RAW / name / 'result.json')
        reference = read(RAW / name / 'reference.json')
        summary_path = OUT / (name + '_collection_summary.json')
        assert report['result_status'] == result['status']
        assert report['fault'] == result['last_control']['fault']
        assert report['closed_bag'] and report['stop_confirmed']
        accepted = report['accepted']
        row = dict(run_id=name, split=report['split'] if accepted else 'excluded',
                   status=report['result_status'], fault=report['fault'],
                   collection_end_reason=result['last_control']['large_recovery']['state']['reason'],
                   event_cap=report['event_cap'], completed_events=sum(e['completed'] for e in report['events']),
                   accepted_events=sum(e.get('accepted_camera_anchors', 0) > 0 for e in report['events']),
                   accepted_camera_anchors=accepted, target_band_camera_anchors=report['target_band_anchors'],
                   all_planned_recovered=report['all_planned_recovered'],
                   increase_eligible=report['increase_eligible'],
                   raw_path=str(RAW / name), raw_bytes=verified[name]['regular_file_bytes'],
                   summary_sha256=sha(summary_path), control_sha256=sha(RAW / name / 'control.jsonl'),
                   reference_sha256=sha(RAW / name / 'reference.json'),
                   sites=reference['large_recovery']['config']['sites'])
        if accepted:
            assert report['result_status'] == 'COMPLETE_LAP' and report['fault'] is None
            assert accepted == report['prepared']['anchors'] == report['prepared']['input_valid']
            assert not report['prepared']['input_reason_refinements']
            assert accepted == sum(e['accepted_camera_anchors'] for e in report['events'])
            split = report['split']
            dest = OUT / 'materialized' / name
            row['teachers_sha256'] = sha(dest / 'teachers.npz')
            row['prepared_path'] = str(OUT / 'prepared' / split / name)
            row['materialized_path'] = str(dest)
            assert Path(row['prepared_path']).is_dir()
            splits[split].append(name)
            t = totals[split]
            t['runs'] += 1
            t['anchors'] += accepted
            t['target_band_anchors'] += report['target_band_anchors']
            t['accepted_events'] += row['accepted_events']
            states = read(OUT / (name + '_anchor_states.json'))
            assert len(states) == accepted
            assert sum(a['target_band'] for a in states) == report['target_band_anchors']
            for anchor in states:
                lat = abs(anchor['lateral_m'])
                key = ('under_5cm' if lat < .05 else '5_to_15cm' if lat < .15
                       else '15_to_25cm' if lat <= .25 else '25cm_or_more')
                bands[split][key] += 1
            row['peak_abs_accepted_lateral_m'] = max(abs(a['lateral_m']) for a in states)
            row['teacher_anchors_per_event'] = [
                {'site_id': e['site_id'], 'accepted': e.get('accepted_camera_anchors', 0),
                 'target_band': e.get('target_band_camera_anchors', 0)} for e in report['events']]
        rows.append(row)
    assert not set(splits['train']) & set(splits['validation'])
    # Both splits must contain independently recorded left and right runs.
    for names in splits.values():
        assert any('-left-' in n for n in names) and any('-right-' in n for n in names)
    index = dict(schema='real_large_recovery_collection_v1',
                 collection_source_commit='7780db94b261ffc6024f2cb8fe5971cd52ee93fa',
                 index_source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                 runtime_host='graneple@192.168.3.10', native_wsl_raw=str(RAW),
                 initial_events_per_lap=3, event_increment=2, maximum_tested_events_per_lap=5,
                 largest_fully_verified_event_cap=max(r['event_cap'] for r in rows if r['increase_eligible']),
                 latest_five_event_collection_eligible=reports['codex-time-recovery-large-live-g05-left-r01']['increase_eligible'],
                 target_abs_lateral_m=0.2, target_band_tolerance_m=0.05,
                 lateral_basis='relative to matched observed normal driving line; not ground-truth road center',
                 labels='measured future trajectory, 30 xy points in meters over 3 seconds',
                 inputs='causal camera / LiDAR / ego history from matched actual shifted observations',
                 training_started=False, merged_with_existing_dataset=False,
                 split_policy='whole runs; g03 left r04 and right r03 train; g03 right r02 and g05 left validation; calibration successes train',
                 receipts=receipts, splits=splits, totals=totals, absolute_lateral_bands=bands, runs=rows,
                 all_runs_closed_and_stopped=True,
                 all_raw_file_hashes_and_structure_verified=True,
                 all_sqlite_quick_checks_passed=True)
    with (OUT / 'collection_index.json').open('x') as f:
        json.dump(index, f, indent=2, allow_nan=False)
    print(json.dumps(dict(totals=totals,
                          completed_events=sum(r['completed_events'] for r in rows),
                          accepted_events=sum(r['accepted_events'] for r in rows),
                          complete_laps=sum(r['status'] == 'COMPLETE_LAP' for r in rows),
                          attempts=len(rows), raw_bytes=sum(r['raw_bytes'] for r in rows))))


if __name__ == '__main__':
    main()
