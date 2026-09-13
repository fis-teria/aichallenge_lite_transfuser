"""Measured recovery-only teachers and a hash-verified nominal/recovery cache.

All targets are observed [30,2] XY metres at 0.1 s intervals. Approach/hold,
braking, missing future support and physically invalid raw yaw histories are
excluded with their original denominator retained in each run audit.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
from typing import Any, Sequence

import numpy as np
from torch.utils.data import Dataset

from aic_transfuser_lite.control.vehicle_motion_v1 import MAX_CURVATURE_PER_M
from .time_archive_v1 import _rename_without_replace
from .time_corpus_v1 import EventWindows, audit_anchor
from .time_dataset_v1 import TimeDatasetConfig, TimeSample
from .time_recovery_collection_v1 import PhaseWindow, recovery_teacher_mask
from .time_split_v1 import content_sha256, validate_time_split
from .time_sqlite_reader_v1 import read_time_sqlite_run
from .time_training_cache_v1 import (_json, _prepare_run, _sha, _write_json,
    verify_time_training_cache, TimeTrainingCacheDataset)


def phase_windows(rows: Sequence[dict[str, Any]]) -> tuple[PhaseWindow, ...]:
    """Sim-ns intervals; telemetry gaps over 150 ms cannot support teachers."""
    phases = {r['sim_ns']: r['phase'] for r in rows if r['sim_ns'] is not None}
    windows: list[PhaseWindow] = []
    ordered = sorted(phases)
    for a, b in zip(ordered, ordered[1:]):
        phase = phases[a] if b-a <= 150_000_000 else 'invalid'
        if windows and windows[-1].phase == phase and windows[-1].end_ns == a:
            windows[-1] = PhaseWindow(windows[-1].start_ns, b, phase)
        else:
            windows.append(PhaseWindow(a, b, phase))
    return tuple(windows)


def invalid_yaw_history(row: dict[str, Any], invalid_ids: set[int]) -> list[int]:
    return sorted({rid for slot in row['history_row_ids']['velocity'] for rid in slot} & invalid_ids)


def recovery_extension(base: dict[str, Any], additions: list[dict[str, Any]]) -> dict[str, Any]:
    value = {'format': 'time_recovery_extension_v1', 'scope': 'same_course_recovery_run_holdout',
             'base_manifest': base, 'additional_runs': additions,
             'runs': sorted(base['runs'] + additions, key=lambda r: r['run_id']), 'sources_verified': True}
    value['manifest_sha256'] = content_sha256({k:v for k,v in value.items() if k != 'sources_verified'})
    validate_time_split(value, require_verified=True)
    return value


def _verify_raw(run: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = _json(run/'transfer_manifest.json')
    for relative, expected in manifest.items():
        path = run/relative
        if not path.resolve().is_relative_to(run.resolve()):
            raise ValueError('raw manifest path escapes run')
        if 'sha256' in expected:
            if not path.is_file() or path.stat().st_size != expected['bytes'] or _sha(path) != expected['sha256']:
                raise ValueError(f'raw source mismatch: {relative}')
    result = _json(run/'result.json')
    if (result['status'] != 'COMPLETE_LAP' or not result['nodes']['closed_bag']
            or result['last_control']['fault'] is not None or not result['last_control']['stop_confirmed']):
        raise ValueError('recovery run is not a normally stopped complete lap')
    return manifest, result


def materialize_recovery_run(run: Path, destination: Path, *, split: str, types: Path,
                             previous_probe: Path) -> dict[str, Any]:
    """Recompute all recovery candidates and require prior audit agreement."""
    if split not in {'train', 'validation'}:
        raise ValueError('recovery materialization excludes test')
    manifest, result = _verify_raw(run)
    prior = _json(previous_probe)
    if (prior['raw_manifest_sha256'] != _sha(run/'transfer_manifest.json')
            or prior['run_id'] != run.name or prior['freeze_delay_ns'] != 50_000_000
            or not prior['all_phase_candidates_audited']):
        raise ValueError('previous complete causal audit does not match raw source')
    destination.mkdir(parents=True, exist_ok=False)
    raw = destination/'raw'
    (raw/'bag').mkdir(parents=True)
    dbs = list((run/'bag').glob('*.db3'))
    if len(dbs) != 1:
        raise ValueError('one closed SQLite bag required')
    os.link(dbs[0], raw/'bag'/dbs[0].name)
    shutil.copytree(types, raw/'types')
    index = read_time_sqlite_run(raw, run.name)
    if len(index.epochs) != 1:
        raise ValueError('one clock epoch required')
    bounds = (index.epochs[0].first_sim_stamp_ns, index.epochs[0].last_sim_stamp_ns)
    phases = phase_windows([json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()])
    cameras = {}
    for event in sorted(index.events, key=lambda e:(e.available_ns, e.sequence)):
        if event.role == 'camera':
            cameras.setdefault((event.epoch, event.capture_ns), event)
    candidates = sorted((a for a in cameras.values()
        if any(w.phase == 'recovery' and w.start_ns+150_000_000 <= a.capture_ns < w.end_ns for w in phases)
        and recovery_teacher_mask(a.capture_ns, phases).all()), key=lambda a:a.capture_ns)
    invalid_ids = {e.sequence for e in index.events if e.role == 'velocity'
        and (not np.isfinite(e.payload.yaw_rate_rps)
             or abs(e.payload.yaw_rate_rps) > max(.2, abs(e.payload.longitudinal_mps))*MAX_CURVATURE_PER_M)}
    windows = EventWindows(index.events)
    cfg = TimeDatasetConfig()
    labels, accepted, audited = [], [], []
    for anchor in candidates:
        teacher, row = audit_anchor(windows.at(anchor), anchor, config=cfg, bounds=bounds,
            freeze_ns=anchor.available_ns+50_000_000, intervention_ns=None)
        invalid = invalid_yaw_history(row, invalid_ids)
        row['invalid_raw_heading_history_row_ids'] = invalid
        if invalid:
            row.update(input_invalid_reason='RAW_HEADING_RATE_INVALID', input_eligible=False,
                       usable_full=False, usable_partial=False)
        row['phase_and_xy_full'] = teacher is not None and bool(
            (teacher.xy_mask & recovery_teacher_mask(anchor.capture_ns, phases)).all())
        audited.append(dict(row))
        if row['usable_full'] and row['phase_and_xy_full']:
            row.update(label_index=len(accepted), run_id=run.name, split=split)
            accepted.append(row)
            labels.append(teacher)
    expected = [r['anchor_id'] for r in prior['anchors'] if r['usable_full'] and r.get('phase_and_xy_full')]
    if len(candidates) != prior['audited_anchors'] or [r['anchor_id'] for r in accepted] != expected:
        raise ValueError('recovery acceptance drift from collection audit')
    if not accepted:
        raise ValueError('no usable recovery teachers')
    with (destination/'anchors.jsonl').open('x') as stream:
        for row in accepted:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    arrays = {key:np.stack([getattr(t,key) for t in labels]) for key in
              ('xy_m','xy_mask','velocity_mps','velocity_mask','interval_mask')}
    assert arrays['xy_m'].shape == (len(accepted),30,2) and arrays['xy_mask'].all()
    assert np.isfinite(arrays['xy_m']).all()
    np.savez_compressed(destination/'teachers.npz', **arrays)
    report = {'run_id':run.name, 'split':split, 'phase_candidates':len(candidates),
        'accepted':len(accepted), 'excluded':len(candidates)-len(accepted),
        'input_reasons':dict(Counter(r['input_invalid_reason'] or 'OK' for r in audited)),
        'anchors':audited, 'raw_manifest_sha256':_sha(run/'transfer_manifest.json'),
        'prior_probe_sha256':_sha(previous_probe), 'source_sha':result['source_sha'],
        'invalid_raw_yaw_message_count':len(invalid_ids), 'config':asdict(cfg),
        'freeze_delay_receipt_ns':50_000_000,
        'availability':'bag_receipt_proxy_not_measured_preprocessing_completion',
        'types':[{ 'path':p.relative_to(raw).as_posix(),'sha256':_sha(p)} for p in sorted((raw/'types').rglob('*.idl'))]}
    _write_json(destination/'audit.json', report)
    return report


def prepare_recovery_cache(base_cache: Path, output: Path, *, plan: dict[str, Any],
                           raw_roots: dict[str, Path], types: Path, evidence_root: Path) -> dict[str, Any]:
    """Reuse immutable nominal cache files by hardlink; never touch test data."""
    partial = output.with_name(output.name+'.partial')
    if output.exists() or partial.exists():
        raise FileExistsError('output or partial exists')
    original = verify_time_training_cache(base_cache)
    if original['manifest_sha256'] != plan['base_cache_sha256']:
        raise ValueError('nominal cache identity changed')
    partial.mkdir(parents=True)
    reports, additions = [], []
    for row in plan['runs']:
        run = raw_roots[row['root']]/row['run_id']
        source = partial/'materialized'/row['run_id']
        report = materialize_recovery_run(run, source, split=row['split'], types=types,
                                         previous_probe=evidence_root/row['probe'])
        report_small = {k:v for k,v in report.items() if k != 'anchors'}
        reports.append(report_small)
        bag = next((run/'bag').glob('*.db3'))
        additions.append({'run_id':run.name, 'split':row['split'], 'speed_cap_kmh':5,
            'sources':[{'path':f'runs/{run.name}/bag/{bag.name}', 'sha256':_sha(bag)}]})
        print(json.dumps({'materialized':report_small}), flush=True)
    split = recovery_extension(original['split_manifest'], additions)
    for record in original['cache_files']:
        target = partial/record['path']
        target.parent.mkdir(parents=True, exist_ok=True)
        os.link(base_cache/record['path'], target)
    prepared = list(original['runs'])
    for row in plan['runs']:
        result = _prepare_run(partial/'materialized'/row['run_id'], partial/row['split']/row['run_id'],
                              row['run_id'], TimeDatasetConfig())
        result['split'] = row['split']
        prepared.append(result)
        print(json.dumps({'cache_run_complete':result}), flush=True)
    # Keep materialized labels/audits and raw hardlink views for replay, while
    # the training inventory covers every file consumed by the cache loader.
    files = [{'path':p.relative_to(partial).as_posix(),'bytes':p.stat().st_size,'sha256':_sha(p)}
        for split_name in ('train','validation') for p in sorted((partial/split_name).rglob('*')) if p.is_file()]
    materialized_files = [{'path':p.relative_to(partial).as_posix(),'sha256':_sha(p)}
        for p in sorted((partial/'materialized').rglob('*')) if p.is_file() and 'raw' not in p.relative_to(partial).parts]
    identity = {'format':'time_training_cache_v1','base_cache_sha256':original['manifest_sha256'],
        'corpus_artifact_manifest_sha256':content_sha256(materialized_files),
        'contract':{'config':asdict(TimeDatasetConfig()),'freeze_delay_receipt_ns':50_000_000,
                    'split_sha256':split['manifest_sha256'],'selection':'full_observed_recovery_only_150ms_margin'},
        'config':asdict(TimeDatasetConfig()),'split_manifest':split,'splits':['train','validation'],
        'runs':prepared, 'cache_files':files, 'materialized_files':materialized_files,
        'recovery_audits':reports, 'plan':plan}
    identity['manifest_sha256'] = content_sha256(identity)
    _write_json(partial/'identity.json', identity)
    _rename_without_replace(partial, output)
    verify_time_training_cache(output)
    return identity


class RecoveryMixDataset(Dataset[TimeSample]):
    """Explicit train-only repetition, with unique and presented counts separate."""
    def __init__(self, dataset: TimeTrainingCacheDataset, recovery_run_ids: Sequence[str], *, repeats: int) -> None:
        if type(repeats) is not int or not 1 <= repeats <= 100:
            raise ValueError('recovery repetition must be an integer in [1,100]')
        from .time_split_v1 import assert_split_membership
        assert_split_membership(dataset.split_manifest, dataset.run_ids, split='train')
        ids = set(recovery_run_ids)
        if not ids or not ids <= set(dataset.run_ids):
            raise ValueError('recovery runs must belong to training')
        self.dataset = dataset
        self.indices = [i for i,rid in enumerate(dataset.run_ids) for _ in range(repeats if rid in ids else 1)]
        self.run_ids = [dataset.run_ids[i] for i in self.indices]
        self.anchor_ids = [dataset.anchor_ids[i] for i in self.indices]

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, index: int) -> TimeSample:
        return self.dataset[self.indices[index]]
