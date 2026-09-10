"""Audit every saved >=10 m train candidate and continue the fixed V4-20 model.

Observed diagnostic labels only. Retain all prior eligible near/recovery anchors,
freeze validation, leave test assets unread; all distances are base_link@t_obs m.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import io
import json
from pathlib import Path
import shutil
import subprocess
import time
import traceback

import numpy as np
import torch

from .spatial_long_full_v4 import InputCache, all_train, epoch_batches
from .spatial_long_diagnosis_v4 import read_parent, restore_optimizer, cohort_summary
from .spatial_long_v4 import LongAccess, audit_target, band_loss, evaluate_metrics, POLICY
from .spatial_diagnostic_v4 import sha, digest, check_hash, write_json, optimizer_for, SPLIT_HASH, LEDGER_HASH
from aic_transfuser_lite.data.dataset_view_v3 import _ego_row
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import (
    INPUT_FIELDS, FEATURES, EGO_LIMITS, build_inputs, combine_inputs, epoch_keys,
)
from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from aic_transfuser_lite.models.temporal.gru import select_epoch_history

EXPANSION_CHECKPOINT_SHA = '07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33'
EXPANSION_EPOCHS = 2
EXPANSION_ACTIVE_SECONDS = 3600
PREPARATION_SECONDS = 1800


def check_preparation_time(started: float) -> None:
    if time.monotonic()-started >= PREPARATION_SECONDS:
        raise TimeoutError('teacher audit/input preparation budget exceeded')


def same_epoch(left: list | tuple, right: list | tuple) -> bool:
    """Compare live tuple keys and JSON list keys without dropping components."""
    return tuple(left) == tuple(right)


def ten_metre_candidates(annotations: list[dict]) -> list[dict]:
    """Metadata-only >=10 m provisional prefix; never select validation/test."""
    if len({r['sample_id'] for r in annotations}) != len(annotations):
        raise ValueError('duplicate ledger sample IDs')
    selected = []
    for row in annotations:
        if row['split'] != 'train':
            continue
        raw = row['h30_noise_filtered_arc_m_provisional']
        if raw in ('', None, 'null'):
            continue
        length = float(raw)
        if not np.isfinite(length) or length < 0:
            raise ValueError('invalid provisional distance in metres')
        if length >= 10:
            selected.append(row)
    return sorted(selected, key=lambda r: r['sample_id'])


def merge_train(previous: list[dict], newly_audited: list[dict]) -> list[dict]:
    """Union eligible train rows, preserving all prior short/recovery anchors."""
    old = all_train(previous)
    if len({r['sample_id'] for r in newly_audited}) != len(newly_audited):
        raise ValueError('duplicate new audit sample IDs')
    result = {r['sample_id']: r for r in old}
    for row in newly_audited:
        if row['split'] != 'train':
            raise ValueError('non-train expansion row')
        if row['sample_id'] in result:
            original = result[row['sample_id']]
            if (not row['diagnostic_eligible'] or row['reasons'] or any(
                original[k] != row[k] for k in ('run_id', 'future_sha256', 'row_index'))):
                raise ValueError('previous eligible teacher changed')
        if row['diagnostic_eligible']:
            if row['reasons'] or row['geometry']['covered_grid_m'] < 10:
                raise ValueError('expansion eligibility/support contradiction')
            result.setdefault(row['sample_id'], row)
    return sorted(result.values(), key=lambda r: (r['run_id'], r['stamp_ns'], r['sample_id']))


def expand_ten_metre_corpus(old: dict, audited: list[dict], arrays: dict,
                            output: Path, started: float) -> tuple[list[dict], dict]:
    """Verify raw [30,8] futures and generate observed XY[46,2]/bool mask[46]."""
    access = LongAccess(Path(old['config']['dataset_root']), Path(old['config']['split_manifest']))
    ledger = list(csv.DictReader(io.StringIO(check_hash(Path(old['config']['coverage_ledger']), LEDGER_HASH).decode())))
    candidates = ten_metre_candidates(ledger)
    if len(candidates) != 12698:
        raise ValueError('frozen ledger candidate count changed')
    index = {r['sample_id']: i for i, r in enumerate(access.rows)}
    keys = epoch_keys(access.rows)
    expanded = []
    for count, annotation in enumerate(candidates, 1):
        check_preparation_time(started)
        i = index[annotation['sample_id']]
        row = access.rows[i]
        if access.splits[row['run_id']] != 'train' or row['run_id'] != annotation['run_id']:
            raise ValueError('candidate split/run mismatch')
        blob = access.read(row['trajectory_path'])
        view, reasons = audit_target(np.load(io.BytesIO(blob), allow_pickle=False))
        access.future_blobs.clear()
        _, ego_mask = _ego_row(row, FEATURES, abs_limits=EGO_LIMITS)
        if not bool(ego_mask.all()):
            reasons.append('invalid_current_ego')
        if float(row['velocity_longitudinal_mps']) <= .05:
            reasons.append('stopped_or_reverse_observation')
        if view['covered_grid_m'] < 10:
            reasons.append('regenerated_support_below_10m')
        item = {'sample_id': row['sample_id'], 'row_index': i, 'run_id': row['run_id'],
            'segment_id': row['segment_id'], 'epoch_key': list(keys[i]), 'split': 'train',
            'stamp_ns': int(row['grid_stamp_ns']), 'future_path': row['trajectory_path'], 'future_sha256': sha(blob),
            'normal_recovery': annotation['normal_recovery'], 'reasons': sorted(set(reasons)),
            'diagnostic_eligible': not reasons, 'runtime_teacher_eligibility': 'UNKNOWN',
            'geometry': {k: v for k, v in view.items() if k not in ('xy', 'mask', 'grid_m', 'reliable_prefix_xy')}}
        expanded.append(item)
        if not reasons:
            if item['sample_id'] in arrays:
                np.testing.assert_array_equal(arrays[item['sample_id']][0], view['xy'])
                np.testing.assert_array_equal(arrays[item['sample_id']][1], view['mask'])
            arrays[item['sample_id']] = (view['xy'], view['mask'])
        if count % 1000 == 0:
            print(json.dumps({'phase': 'expanded_teacher_audit', 'audited': count, 'total': len(candidates)}), flush=True)
    train = merge_train(audited, expanded)
    previous_ids = {r['sample_id'] for r in all_train(audited)}
    summary = {'policy': {**POLICY, 'minimum_selected_gap_s': None}, 'candidate_count': len(candidates),
        'accepted_10m': sum(r['diagnostic_eligible'] for r in expanded),
        'rejected': sum(not r['diagnostic_eligible'] for r in expanded),
        'reasons': dict(Counter(reason for r in expanded for reason in r['reasons'])),
        'previous_train': len(previous_ids), 'new_train': sum(r['sample_id'] not in previous_ids for r in train),
        'total_train': len(train), 'test_assets_read': 0, 'ledger_sha256': LEDGER_HASH,
        'per_run': {run: {'candidates': sum(r['run_id'] == run for r in expanded),
            'accepted': sum(r['run_id'] == run and r['diagnostic_eligible'] for r in expanded)}
            for run in sorted({r['run_id'] for r in expanded})}}
    write_json(output/'expansion_summary.json', summary)
    write_json(output/'expanded_teacher_audit.json', {'summary': summary, 'rows': expanded})
    write_json(output/'audit_source_hashes.json', access.read_hashes)
    for path, expected in access.read_hashes.items():
        check_hash(access.root/path, expected)
    print(json.dumps({'phase': 'expanded_teacher_audit_complete', **summary}), flush=True)
    return train, arrays


def run(parent: Path, checkpoint: Path, output: Path) -> None:
    if output.exists() or not output.is_absolute() or str(output).startswith('/mnt/'):
        raise ValueError('new native Linux output required')
    if subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
        raise ValueError('dirty execution checkout')
    output.mkdir(parents=True)
    if shutil.disk_usage(output).free < 80 * 1024**3:
        raise ValueError('at least 80 GiB free required for native input cache/checkpoints')
    wall = time.monotonic()
    record = {'status': 'STARTED', 'execution_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent': str(parent), 'checkpoint': str(checkpoint), 'checkpoint_sha256': EXPANSION_CHECKPOINT_SHA,
        'epochs_limit': EXPANSION_EPOCHS, 'active_seconds_limit': EXPANSION_ACTIVE_SECONDS, 'optimizer_steps': 0, 'epochs_completed': 0,
        'seed': 44, 'test_assets_read': 0, 'runtime_promoted': False, 'initialization': 'V4_20_epoch12_model_and_optimizer'}
    write_json(output/'execution.json', record)
    try:
        old, original, audited, arrays = read_parent(parent)
        check_hash(checkpoint, EXPANSION_CHECKPOINT_SHA)
        train, arrays = expand_ten_metre_corpus(old, audited, arrays, output, wall)
        validation = [r for r in original if r['split'] == 'validation']
        if len(train) <= 1786 or len(validation) != 64:
            raise ValueError('unexpected frozen corpus size')
        if {r['run_id'] for r in train} & {r['run_id'] for r in validation}:
            raise ValueError('run split overlap')
        items = train + validation
        index = {r['sample_id']: i for i, r in enumerate(items)}
        fixed_ids = [index[r['sample_id']] for r in original]
        target = np.stack([arrays[r['sample_id']][0] for r in items])
        mask = np.stack([arrays[r['sample_id']][1] for r in items])
        if target.shape != (len(items), 46, 2) or mask.dtype != bool or not mask.any(1).all():
            raise ValueError('teacher shape/mask/support mismatch')
        write_json(output/'selection.json', {'items': items, 'identity': digest(items), 'fixed_eval_ids': fixed_ids,
            'train': cohort_summary(train, mask[:len(train)]), 'validation': cohort_summary(validation, mask[len(train):]),
            'selection': 'All accepted >=10m train candidates UNION prior 1786; no thinning; adjacent anchors correlated'})
        np.savez_compressed(output/'teachers.npz', xy=target, mask=mask, grid_m=LONG_GRID,
                            sample_ids=np.array([r['sample_id'] for r in items]))
        access = LongAccess(Path(old['config']['dataset_root']), Path(old['config']['split_manifest']))
        keys = epoch_keys(access.rows)
        for i, row in enumerate(items):
            check_preparation_time(wall)
            canonical = access.rows[row['row_index']]
            if (canonical['sample_id'] != row['sample_id'] or canonical['run_id'] != row['run_id']
                    or not same_epoch(keys[row['row_index']], row['epoch_key']) or access.splits[row['run_id']] != row['split']):
                raise ValueError('canonical sample/epoch/split mismatch')
            blob = access.read(row['future_path'])
            if sha(blob) != row['future_sha256']:
                raise ValueError('future identity mismatch')
            view, reasons = audit_target(np.load(io.BytesIO(blob), allow_pickle=False))
            access.future_blobs.clear()
            if reasons:
                raise ValueError('audited teacher eligibility changed')
            np.testing.assert_array_equal(view['xy'], target[i])
            np.testing.assert_array_equal(view['mask'], mask[i])
            for j in select_epoch_history(keys, anchor_index=row['row_index'], length=4).indices:
                access.allowed_sensor_paths.update(access.rows[j][k] for k in ('image_path', 'lidar_path', 'lidar_valid_path'))
        cache_root = output/'inputs'
        cache_root.mkdir()
        hashes, histories = {}, []
        for i, row in enumerate(items):
            check_preparation_time(wall)
            batch, history = build_inputs(access.rows, row['row_index'], access.read, keys=keys)
            hashes[i] = InputCache.save(cache_root/f'{i:05d}.pt', batch)
            histories.append(history)
            if (i+1) % 200 == 0:
                print(json.dumps({'phase': 'input_cache', 'anchors': i+1, 'total': len(items)}), flush=True)
        cache = InputCache(cache_root, hashes)
        # Real-anchor cache parity before training (no teacher fields serialized).
        for i in (fixed_ids[0], fixed_ids[-1]):
            rebuilt, _ = build_inputs(access.rows, items[i]['row_index'], access.read, keys=keys)
            loaded = cache.get(i)
            for field in INPUT_FIELDS:
                torch.testing.assert_close(getattr(rebuilt, field), getattr(loaded, field), rtol=0, atol=0)
        write_json(output/'input_provenance.json', {'histories': histories, 'asset_hashes': access.read_hashes,
            'cache_sha256': hashes, 'cache_tensor_parity': 'PASS_TWO_RECORDED_ANCHORS_EXACT',
            'manifest_sha256': sha(access.manifest_bytes), 'split_sha256': SPLIT_HASH})
        record['preparation_seconds'] = time.monotonic()-wall
        record['settings'] = {'batch_size': 8, 'microbatch': 2, 'backbone_lr': 1e-4, 'head_lr': 1e-3,
            'dtype': 'float32', 'schedule': None, 'loss': old['teacher_policy']['loss'],
            'teacher_policy': {**old['teacher_policy'], 'minimum_selected_gap_s': None},
            'input_contract': old['input_contract'], 'cache_capacity': 16, 'minimum_selected_gap_s': None}
        record['code_hashes'] = {p: sha(Path(p).read_bytes()) for p in subprocess.check_output(['git', 'ls-files', 'src'], text=True).splitlines()}
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable')
        torch.set_num_threads(4)
        torch.manual_seed(44)
        np.random.seed(44)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        torch.cuda.reset_peak_memory_stats()
        record['environment'] = {'torch': torch.__version__, 'cuda': torch.version.cuda, 'gpu': torch.cuda.get_device_name(0)}
        state = torch.load(checkpoint, map_location='cpu', weights_only=True)
        if state['epoch'] != 12 or state['additional_steps'] != 2688 or state['grid_m'] != LONG_GRID.tolist():
            raise ValueError('checkpoint phase/step mismatch')
        model = SpatialPathLongV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float().cuda()
        model.load_state_dict(state['model'], strict=True)
        optimizer = optimizer_for(model)
        restore_optimizer(optimizer, state['optimizer'])
        start_ages = sorted({int(s['step'].item()) for s in optimizer.state.values()})
        if start_ages != [5188]:
            raise ValueError('unexpected starting optimizer age')
        del state
        active = time.monotonic()
        def check_time() -> None:
            if time.monotonic()-active >= EXPANSION_ACTIVE_SECONDS:
                raise TimeoutError('active budget exceeded')

        def predict(ids: list[int]) -> np.ndarray:
            model.eval()
            result = []
            with torch.no_grad():
                for offset in range(0, len(ids), 2):
                    check_time()
                    result.append(model(combine_inputs([cache.get(i) for i in ids[offset:offset+2]], 'cuda')).cpu().numpy())
            return np.concatenate(result)

        def evaluate(epoch: int, ids: list[int], name: str) -> np.ndarray:
            predictions = predict(ids)
            metrics = evaluate_metrics(predictions, target[ids], mask[ids], [items[i] for i in ids])
            write_json(output/f'{name}_{epoch:02d}_metrics.json', metrics)
            np.savez_compressed(output/f'{name}_{epoch:02d}_predictions.npz', xy=predictions,
                                sample_ids=np.array([items[i]['sample_id'] for i in ids]))
            print(json.dumps({'phase': name, 'epoch': epoch,
                'val_near_m': metrics['validation']['bands']['near_0_2m']['anchor_mean_error_m'],
                'val_at10_m': metrics['validation']['bands']['at_10m']['anchor_mean_error_m'],
                'val_far_m': metrics['validation']['bands']['far_10_20m']['anchor_mean_error_m']}), flush=True)
            return predictions

        initial = evaluate(0, fixed_ids, 'fixed')
        source_record = json.loads((checkpoint.parent/'execution.json').read_text())
        if source_record['status'] != 'COMPLETE':
            raise ValueError('source training incomplete')
        check_hash(checkpoint.parent/'fixed_12_predictions.npz',
                   source_record['artifact_hashes']['fixed_12_predictions.npz'])
        source_pred = np.load(checkpoint.parent/'fixed_12_predictions.npz', allow_pickle=False)
        source_map = {s: i for i, s in enumerate(source_pred['sample_ids'])}
        expected = source_pred['xy'][[source_map[items[i]['sample_id']] for i in fixed_ids]]
        np.testing.assert_allclose(initial, expected, rtol=1e-5, atol=2e-5)
        record['initial_replay_max_abs_m'] = float(np.abs(initial-expected).max())
        visits = Counter()
        with (output/'training.jsonl').open('x') as log:
            for epoch in range(1, EXPANSION_EPOCHS+1):
                losses = []
                for ids in epoch_batches(len(train), epoch):
                    check_time()
                    if time.monotonic()-active > EXPANSION_ACTIVE_SECONDS-600:
                        raise TimeoutError('final evaluation reserve reached before all epochs')
                    model.train()
                    optimizer.zero_grad(set_to_none=True)
                    total = 0.
                    for offset in range(0, len(ids), 2):
                        part = ids[offset:offset+2]
                        prediction = model(combine_inputs([cache.get(i) for i in part], 'cuda'))
                        loss = band_loss(prediction, torch.from_numpy(target[part]).cuda(), torch.from_numpy(mask[part]).cuda())
                        if loss is None:
                            raise ValueError('empty support')
                        weighted = loss * len(part)/len(ids)
                        weighted.backward()
                        total += float(weighted.detach())
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 1., error_if_nonfinite=True)
                    optimizer.step()
                    if not all(torch.isfinite(p).all() for p in model.parameters()):
                        raise ValueError('nonfinite model')
                    visits.update(ids)
                    record['optimizer_steps'] += 1
                    losses.append(total)
                    log.write(json.dumps({'step': record['optimizer_steps'], 'epoch': epoch, 'anchors': len(ids),
                                          'loss': total, 'gradient_norm': float(norm)})+'\n')
                    log.flush()
                    if record['optimizer_steps'] % 100 == 0:
                        print(json.dumps({'phase': 'training_progress', 'epoch': epoch, 'steps': record['optimizer_steps'], 'active_seconds': time.monotonic()-active}), flush=True)
                if len(visits) != len(train) or any(visits[i] != epoch for i in range(len(train))):
                    raise ValueError('epoch did not visit every train teacher exactly once')
                record['epochs_completed'] = epoch
                record['presentations_per_anchor'] = epoch
                print(json.dumps({'phase': 'training', 'epoch': epoch, 'steps': record['optimizer_steps'],
                                  'mean_batch_loss': float(np.mean(losses)), 'all_train_visited': len(visits)}), flush=True)
                if epoch in (1, 2):
                    evaluate(epoch, fixed_ids, 'fixed')
                    torch.save({'model': model.state_dict(), 'optimizer': optimizer.state_dict(), 'epoch': epoch,
                        'additional_steps': record['optimizer_steps'], 'parent_optimizer_steps': 5188,
                        'grid_m': LONG_GRID.tolist(), 'selection_identity': digest(items),
                        'execution_commit': record['execution_commit'], 'torch_rng_state': torch.get_rng_state(),
                        'cuda_rng_state': torch.cuda.get_rng_state(), 'tier': 'OBSERVED_DIAGNOSTIC_ONLY'}, output/f'epoch_{epoch:02d}.pt')
                write_json(output/'execution.json', record)
        evaluate(EXPANSION_EPOCHS, list(range(len(items))), 'all_train_and_validation')
        saved = torch.load(output/'epoch_02.pt', map_location='cpu', weights_only=True)
        model.load_state_dict(saved['model'], strict=True)
        replay = predict(fixed_ids[:2])
        expected = np.load(output/'fixed_02_predictions.npz')['xy'][:2]
        np.testing.assert_allclose(replay, expected, rtol=1e-5, atol=2e-5)
        record['checkpoint_replay_max_abs_m'] = float(np.abs(replay-expected).max())
        for path, expected_hash in list(access.read_hashes.items()):
            check_hash(access.root/path, expected_hash)
        for i, expected_hash in hashes.items():
            check_hash(cache_root/f'{i:05d}.pt', expected_hash)
        check_hash(checkpoint, EXPANSION_CHECKPOINT_SHA)
        check_hash(access.root/'manifest.yaml', sha(access.manifest_bytes))
        check_hash(Path(old['config']['split_manifest']), SPLIT_HASH)
        record.update(status='COMPLETE', active_seconds=time.monotonic()-active,
            peak_allocated_bytes=torch.cuda.max_memory_allocated(), post_run_identity='PASS',
            train_presentations=sum(visits.values()), cache_bytes=sum(p.stat().st_size for p in cache_root.iterdir()))
    except Exception:
        record.update(status='FAILED', traceback=traceback.format_exc())
        raise
    finally:
        record['wall_seconds'] = time.monotonic()-wall
        record['artifact_hashes'] = {p.name: sha(p.read_bytes()) for p in output.iterdir() if p.is_file() and p.name != 'execution.json'}
        write_json(output/'execution.json', record)
        print(json.dumps({'status': record['status'], 'epochs': record['epochs_completed'], 'output': str(output)}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.parent, args.checkpoint, args.output)
