"""Append previously audited recovery caches without changing existing bytes."""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
from typing import Any

import numpy as np

from .time_archive_v1 import _rename_without_replace
from .time_recovery_training_v1 import recovery_extension, _verify_raw
from .time_split_v1 import content_sha256
from .time_training_cache_v1 import _sha, _checked_path, verify_time_training_cache

SENSOR_FILES = ('anchors.jsonl', 'camera_rgb.npy', 'inputs.npz', 'labels.npz', 'lidar.npy')
MATERIAL_FILES = ('anchors.jsonl', 'audit.json', 'teachers.npz')


def read_json(path: Path) -> Any:
    return json.loads(path.read_bytes())


def append_split(parent: dict[str, Any], additions: list[dict[str, Any]]) -> dict[str, Any]:
    """Preserve the original 20-run split and every previous recovery assignment."""
    if parent.get('format') != 'time_recovery_extension_v1':
        raise ValueError('a verified recovery extension is required')
    if not additions or any(r['split'] not in ('train', 'validation') for r in additions):
        raise ValueError('only assigned train/validation additions are permitted')
    before = {r['run_id']: r for r in parent['runs']}
    if set(before) & {r['run_id'] for r in additions}:
        raise ValueError('existing run cannot be replaced or reassigned')
    result = recovery_extension(deepcopy(parent['base_manifest']),
                                deepcopy(parent['additional_runs'] + additions))
    if any(next(r for r in result['runs'] if r['run_id'] == key) != value for key, value in before.items()):
        raise ValueError('original split changed')
    return result


def validate_prepared(path: Path, *, run_id: str, split: str, count: int,
                      event_ids: list[int]) -> list[dict[str, Any]]:
    """Require full [N,30,2] XY metres, causal input metadata and fixed event IDs."""
    rows = [json.loads(s) for s in (path/'anchors.jsonl').read_text().splitlines()]
    if split not in ('train', 'validation') or count < 1 or len(rows) != count:
        raise ValueError('invalid split or anchor count')
    if len({r['anchor_id'] for r in rows}) != count or any(
            r['run_id'] != run_id or r['split'] != split or r['label_index'] != i
            or r['input_invalid_reason'] is not None for i, r in enumerate(rows)):
        raise ValueError('anchor identity, order or input eligibility drift')
    if (bool(event_ids) and any('recovery_event_id' not in r for r in rows)
            or {r['recovery_event_id'] for r in rows if 'recovery_event_id' in r} != set(event_ids)):
        raise ValueError('recovery event assignment drift')
    with np.load(path/'labels.npz', allow_pickle=False) as labels:
        if (labels['xy_m'].shape != (count, 30, 2) or labels['xy_mask'].shape != (count, 30)
                or not labels['xy_mask'].all() or not np.isfinite(labels['xy_m']).all()):
            raise ValueError('teachers must be finite full [N,30,2] XY metres')
    with np.load(path/'inputs.npz', allow_pickle=False) as inputs:
        if inputs['input_valid'].shape != (count,) or not inputs['input_valid'].all():
            raise ValueError('all additional inputs must be valid')
        for key, shape in (('ego', (count, 10, 4)), ('command', (count, 10, 3)), ('dt', (count, 4, 2))):
            if inputs[key].shape != shape or not np.isfinite(inputs[key]).all():
                raise ValueError('input shape or finite values changed: '+key)
        for key, file in (('camera_refs', 'camera_rgb.npy'), ('lidar_refs', 'lidar.npy')):
            sensor = np.load(path/file, mmap_mode='r', allow_pickle=False)
            refs = inputs[key]
            if (refs.shape != (count, 4) or not np.issubdtype(refs.dtype, np.integer)
                    or np.any(refs < -1) or np.any(refs >= len(sensor)) or np.any(refs[:, -1] < 0)):
                raise ValueError('invalid sensor reference shape/range')
            if key == 'camera_refs' and (sensor.shape[1:] != (224, 384, 3) or sensor.dtype != np.uint8):
                raise ValueError('camera must be uint8 [M,224,384,3] RGB')
            if key == 'lidar_refs' and (sensor.shape[1:] != (2, 750) or not np.isfinite(sensor).all()):
                raise ValueError('LiDAR must be finite [M,2,750]')
    return rows


def prepare_append_cache(root: Path, output: Path, plan: dict[str, Any]) -> dict[str, Any]:
    """Rehash raw/new prepared data, then hardlink immutable files into a new cache."""
    root, output = root.resolve(), output.resolve()
    partial = output.with_name(output.name+'.partial')
    if not output.is_relative_to(root/'datasets/cache'):
        raise ValueError('native cache destination required')
    if output.exists() or partial.exists():
        raise FileExistsError('cache or partial output already exists')
    parent_path = root/plan['parent_cache']
    parent = verify_time_training_cache(parent_path)
    if parent['manifest_sha256'] != plan['parent_cache_sha256']:
        raise ValueError('pinned parent cache changed')
    for entry in parent['materialized_files']:
        if _sha(_checked_path(parent_path, entry['path'])) != entry['sha256']:
            raise ValueError('parent materialized evidence changed')
    additions, validated = [], []
    for spec in plan['additions']:
        analysis = root/spec['analysis']
        proof_path = analysis/spec['proof']
        if _sha(proof_path) != spec['proof_sha256']:
            raise ValueError('collection proof changed')
        proof = read_json(proof_path)
        if proof['status'] != 'PASS':
            raise ValueError('collection did not pass')
        matches = [r for r in proof['runs'] if r['run_id'] == spec['run_id']]
        if len(matches) != 1 or matches[0]['split'] != spec['split']:
            raise ValueError('collection assignment mismatch')
        collected = matches[0]
        metadata = collected.get('prepared', collected)
        if (metadata['anchors'] != spec['anchors'] or metadata['input_valid'] != spec['anchors']
                or metadata['input_reason_refinements']):
            raise ValueError('prepared count or eligibility changed')
        if spec['event_ids'] and ({r['event_id'] for r in collected['events']} != set(spec['event_ids'])
                                 or not all(r['recovery_confirmed'] for r in collected['events'])):
            raise ValueError('unconfirmed or changed random events')
        prepared = analysis/'prepared'/spec['split']/spec['run_id']
        material = analysis/'materialized'/spec['run_id']
        expected = {p.relative_to(analysis).as_posix() for p in
                    [*(prepared/n for n in SENSOR_FILES), *(material/n for n in MATERIAL_FILES)]}
        if set(collected['files']) != expected:
            raise ValueError('unexpected prepared/materialized file inventory')
        for rel, digest in collected['files'].items():
            if _sha(_checked_path(analysis, rel)) != digest:
                raise ValueError('prepared source hash mismatch: '+rel)
        raw = root/spec['raw']/spec['run_id']
        manifest, _ = _verify_raw(raw)
        audit = read_json(material/'audit.json')
        if (audit['raw_manifest_sha256'] != _sha(raw/'transfer_manifest.json')
                or audit['prior_probe_sha256'] != _sha(analysis/spec['probe'])
                or audit['accepted'] != spec['anchors'] or audit['split'] != spec['split']):
            raise ValueError('raw/probe/materialized correspondence drift')
        rows = validate_prepared(prepared, run_id=spec['run_id'], split=spec['split'],
                                 count=spec['anchors'], event_ids=spec['event_ids'])
        material_ids = [json.loads(s)['anchor_id'] for s in (material/'anchors.jsonl').read_text().splitlines()]
        if material_ids != [r['anchor_id'] for r in rows]:
            raise ValueError('materialized and prepared anchor order differ')
        bags = list((raw/'bag').glob('*.db3'))
        if len(bags) != 1 or _sha(material/'raw/bag'/bags[0].name) != manifest['bag/'+bags[0].name]['sha256']:
            raise ValueError('raw replay bag mismatch')
        additions.append(dict(run_id=spec['run_id'], split=spec['split'], speed_cap_kmh=5,
            sources=[dict(path=f"runs/{spec['run_id']}/bag/{bags[0].name}", sha256=manifest['bag/'+bags[0].name]['sha256'])]))
        validated.append((spec, prepared, material, metadata, audit))
        print('ADDITION_REHASHED', spec['run_id'], len(rows), flush=True)
    split = append_split(parent['split_manifest'], additions)
    partial.mkdir(parents=True)
    for entry in parent['cache_files']:
        source = _checked_path(parent_path, entry['path'])
        target = partial/entry['path']; target.parent.mkdir(parents=True, exist_ok=True)
        os.link(source, target)
    # Retain original replay views; no test cache/bag is traversed.
    for run in parent['split_manifest']['additional_runs']:
        name = run['run_id']
        shutil.copytree(parent_path/'materialized'/name, partial/'materialized'/name, copy_function=os.link)
    runs, audits = deepcopy(parent['runs']), deepcopy(parent['recovery_audits'])
    for spec, prepared, material, metadata, audit in validated:
        shutil.copytree(prepared, partial/spec['split']/spec['run_id'], copy_function=os.link)
        shutil.copytree(material, partial/'materialized'/spec['run_id'], copy_function=os.link)
        runs.append({**{k: metadata[k] for k in ('run_id', 'anchors', 'input_valid', 'real_input_replay_equal', 'input_reason_refinements')},
                     'split': spec['split']})
        audits.append({k: v for k, v in audit.items() if k != 'anchors'})
    files = [dict(path=p.relative_to(partial).as_posix(), bytes=p.stat().st_size, sha256=_sha(p))
             for name in ('train', 'validation') for p in sorted((partial/name).rglob('*')) if p.is_file()]
    material_files = [dict(path=p.relative_to(partial).as_posix(), sha256=_sha(p))
        for p in sorted((partial/'materialized').rglob('*')) if p.is_file() and 'raw' not in p.relative_to(partial).parts]
    consumed = {r['path']: r for r in files}
    if any(consumed.get(r['path']) != r for r in parent['cache_files']):
        raise ValueError('parent cache bytes changed')
    identity = {**deepcopy(parent), 'parent_cache_sha256': parent['manifest_sha256'],
        'contract': {**parent['contract'], 'split_sha256': split['manifest_sha256']},
        'split_manifest': split, 'runs': runs, 'cache_files': files, 'materialized_files': material_files,
        'corpus_artifact_manifest_sha256': content_sha256(material_files), 'recovery_audits': audits, 'plan': plan}
    identity.pop('manifest_sha256')
    identity['manifest_sha256'] = content_sha256(identity)
    with (partial/'identity.json').open('x') as stream:
        json.dump(identity, stream, indent=2, allow_nan=False)
    _rename_without_replace(partial, output)
    verify_time_training_cache(output)
    return identity
