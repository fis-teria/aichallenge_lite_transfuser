"""Audit collected recovery runs before appending them to the training corpus."""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .time_recovery_append_v1 import MATERIAL_FILES, SENSOR_FILES, read_json, validate_prepared
from .time_recovery_training_v1 import _verify_raw
from .time_training_cache_v1 import _prepare_run, _sha
from .time_dataset_v1 import TimeDatasetConfig


def accepted_events(rows: Sequence[dict[str, Any]], events: Sequence[dict[str, Any]], *, large: bool) -> list[int]:
    """Keep only confirmed events represented by actual [N,30,2] teachers."""
    selected = {r['recovery_event_id'] for r in rows}
    ids = [e['event_id'] for e in events]
    if len(ids) != len(set(ids)) or not selected:
        raise ValueError('unique event records and nonempty accepted anchors required')
    allowed = {e['event_id'] for e in events if e['recovery_confirmed']
               and (not large or (e['completed'] and e['stable_at_end']))}
    if not selected <= allowed:
        raise ValueError('unconfirmed event entered the teacher population')
    return sorted(selected)


def early_recovery_ids(rows: Sequence[dict[str, Any]], *, duration_ns: int = 1_000_000_000) -> list[str]:
    """First second of each accepted event; sim nanoseconds, train-only use."""
    if type(duration_ns) is not int or duration_ns <= 0 or not rows:
        raise ValueError('nonempty anchors and positive duration required')
    if any(r['split'] != 'train' for r in rows):
        raise ValueError('validation anchors cannot influence the sampler')
    if len({r['anchor_id'] for r in rows}) != len(rows):
        raise ValueError('duplicate anchor IDs')
    starts: dict[tuple[str, int], int] = {}
    for r in rows:
        key = (r['run_id'], r['recovery_event_id'])
        starts[key] = min(starts.get(key, r['observation_ns']), r['observation_ns'])
    return sorted(r['anchor_id'] for r in rows
                  if r['observation_ns'] < starts[(r['run_id'], r['recovery_event_id'])] + duration_ns)


def compare_prepared(left: Path, right: Path) -> None:
    """Require full replay equality, including sensor arrays and causal row IDs."""
    if (left/'anchors.jsonl').read_bytes() != (right/'anchors.jsonl').read_bytes():
        raise ValueError('replayed anchor history differs')
    for name in SENSOR_FILES[1:]:
        if name.endswith('.npz'):
            with np.load(left/name, allow_pickle=False) as a, np.load(right/name, allow_pickle=False) as b:
                if set(a.files) != set(b.files) or any(not np.array_equal(a[k], b[k], equal_nan=True) for k in a.files):
                    raise ValueError('replayed tensor differs: '+name)
        else:
            a = np.load(left/name, mmap_mode='r', allow_pickle=False)
            b = np.load(right/name, mmap_mode='r', allow_pickle=False)
            if not np.array_equal(a, b, equal_nan=True):
                raise ValueError('replayed sensor differs: '+name)


def audit_collections(root: Path, collections: list[dict[str, Any]], output: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Rehash raw bags and regenerate every accepted input history in native WSL."""
    proof: dict[str, Any] = dict(status='PASS', runs=[], collections=collections,
        raw_rehashed=True, all_prepared_arrays_replayed=True, failed_events_excluded=True)
    additions = []
    seen: set[str] = set()
    for group in collections:
        analysis = root/group['analysis']
        index_path = analysis/'collection_index.json'
        if _sha(index_path) != group['index_sha256']:
            raise ValueError('pinned collection index changed')
        index = read_json(index_path)
        for collected in index['runs']:
            count = collected.get('accepted', collected.get('accepted_camera_anchors', 0))
            if not count:
                continue
            name, split = collected['run_id'], collected['split']
            if name in seen or split not in ('train', 'validation'):
                raise ValueError('duplicate or unassigned collected run')
            seen.add(name)
            summary_path = analysis/(name+'_collection_summary.json')
            if collected.get('summary_sha256') and _sha(summary_path) != collected['summary_sha256']:
                raise ValueError('collection summary changed')
            summary = read_json(summary_path)
            prepared = analysis/'prepared'/split/name
            material = analysis/'materialized'/name
            anchors = [json.loads(s) for s in (prepared/'anchors.jsonl').read_text().splitlines()]
            event_ids = accepted_events(anchors, summary['events'], large=group['mode']=='large')
            validate_prepared(prepared, run_id=name, split=split, count=count, event_ids=event_ids)
            raw = root/group['raw']/name
            _verify_raw(raw)
            audit = read_json(material/'audit.json')
            probe = analysis/(name+'_causal_probe.json')
            if (audit['raw_manifest_sha256'] != _sha(raw/'transfer_manifest.json')
                    or audit['prior_probe_sha256'] != _sha(probe)
                    or audit['accepted'] != count or audit['split'] != split
                    or collected.get('teachers_sha256', _sha(material/'teachers.npz')) != _sha(material/'teachers.npz')):
                raise ValueError('raw, causal probe and teacher correspondence changed')
            replay = output/'replay_check'/name
            metadata = _prepare_run(material, replay, name, TimeDatasetConfig())
            compare_prepared(prepared, replay)
            if metadata['anchors'] != count or metadata['input_valid'] != count or metadata['input_reason_refinements']:
                raise ValueError('replayed input eligibility changed')
            files = {p.relative_to(analysis).as_posix(): _sha(p) for p in
                     [*(prepared/n for n in SENSOR_FILES), *(material/n for n in MATERIAL_FILES)]}
            events = [dict(event_id=i, recovery_confirmed=True) for i in event_ids]
            row = dict(run_id=name, split=split, prepared=metadata, events=events, files=files,
                amplitude_group=collected.get('target_abs_lateral_m', group['amplitude_group']),
                source_index_sha256=group['index_sha256'], summary_sha256=_sha(summary_path),
                early_train_anchor_ids=early_recovery_ids(anchors) if split=='train' else [],
                accepted_anchor_count=count, accepted_event_count=len(event_ids))
            proof['runs'].append(row)
            additions.append(dict(run_id=name, split=split, anchors=count, event_ids=event_ids,
                analysis=group['analysis'], raw=group['raw'], probe=probe.name,
                proof='../'+output.name+'/source_verification.json'))
            print('COLLECTION_REPLAY_VERIFIED', name, count, flush=True)
    proof['totals'] = {s: dict(anchors=sum(r['accepted_anchor_count'] for r in proof['runs'] if r['split']==s),
        events=sum(r['accepted_event_count'] for r in proof['runs'] if r['split']==s),
        runs=sum(r['split']==s for r in proof['runs'])) for s in ('train','validation')}
    return proof, additions
