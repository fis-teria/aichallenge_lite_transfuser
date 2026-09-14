"""Bounded real-data replay audit, not training or split assignment.

Run under the native WSL worktree lock after the raw transfer audit passes.
This creates a derived hardlink view with the recording image's IDL definitions
so the unchanged time-corpus reader and causal selectors can read the new bag.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, recovery_teacher_mask
from aic_transfuser_lite.data.time_sqlite_reader_v1 import read_time_sqlite_run, load_event
from aic_transfuser_lite.control.vehicle_motion_v1 import MAX_CURVATURE_PER_M


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--types', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', choices=('baseline', 'recovery'), default='recovery')
    parser.add_argument('--runtime-config', type=Path,
                        default=Path('configs/control/time_path_vehicle_model_5kmh_20260913.json'))
    args = parser.parse_args()
    run = args.run.resolve()
    result = json.loads((run/'result.json').read_text())
    runtime_bytes = args.runtime_config.read_bytes()
    freeze_delay = json.loads(runtime_bytes)['camera_receipt_freeze_ns']
    assert type(freeze_delay) is int and freeze_delay == 50_000_000
    assert result['nodes']['closed_bag']
    assert not args.output.exists()
    view = args.output.parent/(run.name+'_causal_view_'+args.phase+'_freeze'+str(freeze_delay))
    view.mkdir(exist_ok=True)
    (view/'bag').mkdir(exist_ok=True)
    dbs = list((run/'bag').glob('*.db3'))
    assert len(dbs) == 1
    if (view/'bag'/dbs[0].name).exists():
        assert os.path.samefile(dbs[0], view/'bag'/dbs[0].name)
    else:
        os.link(dbs[0], view/'bag'/dbs[0].name)
    if (view/'types').exists():
        assert (view/'types').is_symlink() and (view/'types').resolve() == args.types.resolve()
    else:
        (view/'types').symlink_to(args.types.resolve(), target_is_directory=True)
    index = read_time_sqlite_run(view, run.name)
    assert len(index.epochs) == 1
    epoch = index.epochs[0]
    bounds = (epoch.first_sim_stamp_ns, epoch.last_sim_stamp_ns)
    controls = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    windows = collection_phase_windows(controls)
    cameras = {}
    for e in sorted(index.events, key=lambda e:(e.available_ns, e.sequence)):
        if e.role == 'camera':
            cameras.setdefault((e.epoch, e.capture_ns), e)
    candidates = [a for a in cameras.values()
        if any(w.phase == args.phase and w.start_ns+150_000_000 <= a.capture_ns < w.end_ns for w in windows)
        and recovery_teacher_mask(a.capture_ns, windows).all()]
    candidates.sort(key=lambda a:a.capture_ns)
    # Cover all candidates in a small pilot; keep larger diagnostic runs bounded.
    chosen = [candidates[i] for i in sorted(set(np.linspace(0, len(candidates)-1,
              min(256, len(candidates)), dtype=int).tolist()))] if candidates else []
    event_windows = EventWindows(index.events)
    velocities = {e.sequence:e for e in index.events if e.role == 'velocity'}
    invalid_velocity_ids = {rid for rid,e in velocities.items()
        if not np.isfinite(e.payload.yaw_rate_rps)
        or abs(e.payload.yaw_rate_rps) > max(.2,abs(e.payload.longitudinal_mps))*MAX_CURVATURE_PER_M}
    targeted = []
    for rid in sorted(invalid_velocity_ids):
        for delay in (100_000_000, 500_000_000):
            at = velocities[rid].capture_ns+delay
            candidate = next((a for a in candidates if at <= a.capture_ns <= at+200_000_000), None)
            if candidate is not None:
                targeted.append(candidate)
    chosen = sorted({a.sequence:a for a in (*chosen,*targeted[:8])}.values(),key=lambda a:a.capture_ns)
    config = TimeDatasetConfig()
    records = []
    full = []
    for anchor in chosen:
        events = event_windows.at(anchor)
        teacher, row = audit_anchor(events, anchor, config=config, bounds=bounds,
                                   freeze_ns=anchor.available_ns+freeze_delay, intervention_ns=None)
        invalid_history = sorted({rid for slot in row['history_row_ids']['velocity'] for rid in slot}
                                 & invalid_velocity_ids)
        row['invalid_raw_heading_history_row_ids'] = invalid_history
        if invalid_history:
            row.update(input_invalid_reason='RAW_HEADING_RATE_INVALID', input_eligible=False,
                       usable_full=False, usable_partial=False)
        phase_mask = recovery_teacher_mask(anchor.capture_ns, windows)
        if teacher is not None:
            row['phase_and_xy_full'] = bool((phase_mask & teacher.xy_mask).all())
        if row['usable_full'] and row.get('phase_and_xy_full'):
            full.append((anchor, row, teacher))
        records.append(row)
    smoke = []
    chosen_full = [full[i] for i in sorted(set(np.linspace(0,len(full)-1,min(3,len(full)),dtype=int).tolist()))] if full else []
    for anchor, row, teacher in chosen_full:
        sensor_ids = {rid for role in ('camera','lidar') for slot in row['history_row_ids'][role] for rid in slot}
        events = tuple(load_event(view,e) if e.sequence in sensor_ids else e
                       for e in event_windows.at(anchor)
                       if e.role not in ('camera','lidar') or e.sequence in sensor_ids)
        actual_anchor = next(e for e in events if e.sequence == anchor.sequence)
        sample = assemble_time_sample(events, actual_anchor, config=config,
            epoch_start_ns=bounds[0], epoch_end_ns=bounds[1], freeze_ns=anchor.available_ns+freeze_delay,
            intervention_ns=None)
        assert sample.inputs is not None and sample.teacher is not None
        np.testing.assert_array_equal(sample.teacher.xy_mask, teacher.xy_mask)
        np.testing.assert_allclose(sample.teacher.xy_m, teacher.xy_m, rtol=0, atol=0)
        assert sample.teacher.xy_mask.all() and np.isfinite(sample.teacher.xy_m).all()
        smoke.append(dict(anchor_id=row['anchor_id'], status='PASS',
            image_shape=list(sample.inputs.image.shape), lidar_shape=list(sample.inputs.lidar.shape),
            xy_shape=list(sample.teacher.xy_m.shape), final_xy_m=sample.teacher.xy_m[-1].tolist()))
    report = dict(run_id=run.name, phase=args.phase, status='PASS' if smoke else 'NO_USABLE_SMOKE',
        raw_status=result['status'], source_sha=result['source_sha'],
        raw_manifest_sha256=hashlib.sha256((run/'transfer_manifest.json').read_bytes()).hexdigest(),
        availability='bag_receipt_proxy_not_measured_preprocessing_completion', freeze_delay_ns=freeze_delay,
        runtime_config_sha256=hashlib.sha256(runtime_bytes).hexdigest(), runtime_config=str(args.runtime_config),
        phase_candidates_with_150ms_start_margin=len(candidates), audited_anchors=len(records),
        selection_policy='All candidates up to 256, otherwise uniform, plus up to 8 targeted at invalid raw heading histories',
        all_phase_candidates_audited=len(records)==len(candidates),
        audited_input_eligible=sum(r['input_eligible'] for r in records),
        audited_full_observed_future=sum(r['usable_full'] and r.get('phase_and_xy_full',False) for r in records),
        input_reasons=dict(Counter(r['input_invalid_reason'] or 'OK' for r in records)),
        teacher_reasons=dict(Counter(reason for r in records for reason in r['teacher_reasons'])),
        fallback_counts=index.fallback_counts, epochs=len(index.epochs), replay_smoke=smoke,
        invalid_raw_heading_message_count=len(invalid_velocity_ids),
        training_materialized=False, split_assigned=False,
        scope='Representative input/future replay only; no full-corpus acceptance or model evaluation',
        anchors=records)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!='anchors'},indent=2))


if __name__ == '__main__':
    main()
