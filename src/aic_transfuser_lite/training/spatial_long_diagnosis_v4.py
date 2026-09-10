"""Finite paired learning-duration/data-quantity probes for the 20 m model.

Uses frozen, previously audited observed teachers. No threshold/model/loss tuning,
new teacher adoption, test assets, ROS or runtime promotion. Native WSL only.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import io
import json
from pathlib import Path
import random
import subprocess
import time
import traceback

import numpy as np
import torch
import yaml

from .spatial_long_v4 import LongAccess, band_loss, evaluate_metrics, audit_target, BANDS
from .spatial_diagnostic_v4 import sha, digest, write_json, check_hash, optimizer_for, SPLIT_HASH
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import build_inputs, combine_inputs, epoch_keys
from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from aic_transfuser_lite.models.temporal.gru import select_epoch_history

PARENT_FINAL_SHA = '0cb82d3dcfba81e11fbd89c51b9e793eca00849d1cb551c33c5398568f8394f6'
PARENT_INITIAL_SHA = 'ba66991ee9437fed22087d1221a1e05bb4ce268b8e93fc557c00af931df27651'
PARENT_COMMIT = '9bc02287c862511f36bdb31f634bf9ade3b55d3a'
PHASES = {'tiny8': 1000, 'same128': 1000, 'expanded256': 2000}
LIMIT_SECONDS = 2400


def stratum(row: dict) -> tuple[str, int]:
    """Run and observed support bin in m; shape is reported, never tuned."""
    support = row['geometry']['covered_grid_m']
    band = 20 if support >= 20 else 10 if support >= 10 else 2 if support >= 2 else 0
    return row['run_id'], band


def expand_selection(original: list[dict], audited: list[dict]) -> list[dict]:
    """Double train anchors within every run/support stratum, preserving originals.

    Inputs are metadata only. Require unique IDs, eligible train rows and >=0.5 s
    same-run gap, including original anchors. Fail if exact quotas are impossible.
    """
    if not original or any(r['split'] != 'train' or not r['diagnostic_eligible'] for r in original):
        raise ValueError('original must contain eligible train anchors only')
    if len({r['sample_id'] for r in original}) != len(original):
        raise ValueError('duplicate original ID')
    if len({r['sample_id'] for r in audited}) != len(audited):
        raise ValueError('duplicate audited ID')
    original_by_id = {r['sample_id']: r for r in original}
    for row in audited:
        if row['sample_id'] in original_by_id and row != original_by_id[row['sample_id']]:
            raise ValueError('original/audit metadata mismatch')
    if not set(original_by_id) <= {r['sample_id'] for r in audited}:
        raise ValueError('original missing from audit')
    for i, row in enumerate(original):
        if any(other['run_id'] == row['run_id'] and abs(other['stamp_ns']-row['stamp_ns']) < 500_000_000
               for other in original[:i]):
            raise ValueError('original same-run gap below 0.5 s')
    quotas = Counter(stratum(r) for r in original)
    buckets: dict[tuple, list[dict]] = defaultdict(list)
    for row in audited:
        if (row['split'] == 'train' and row['diagnostic_eligible'] and row['sample_id'] not in original_by_id
                and stratum(row) in quotas):
            buckets[stratum(row)].append(row)
    rng = random.Random(43)
    for key in sorted(buckets):
        buckets[key].sort(key=lambda r: r['sample_id'])
        rng.shuffle(buckets[key])
    chosen = list(original)
    for key in sorted(quotas):
        added = 0
        for row in buckets[key]:
            if all(other['run_id'] != row['run_id'] or abs(other['stamp_ns']-row['stamp_ns']) >= 500_000_000
                   for other in chosen):
                chosen.append(row)
                added += 1
                if added == quotas[key]:
                    break
        if added != quotas[key]:
            raise ValueError(f'insufficient expansion stratum {key}: {added}/{quotas[key]}')
    return chosen


def cohort_summary(rows: list[dict], mask: np.ndarray) -> dict:
    """Metadata and actual observed point/run denominators for [N,46] masks."""
    if mask.shape != (len(rows), 46) or mask.dtype != bool:
        raise ValueError('cohort mask shape/type mismatch')
    return {'anchors': len(rows), 'runs': len({r['run_id'] for r in rows}),
        'shape_counts': dict(Counter(r['geometry']['shape'] for r in rows)),
        'normal_recovery': dict(Counter(r['normal_recovery'] for r in rows)),
        'strata': {f'{run}/{band}m': count for (run, band), count in sorted(Counter(stratum(r) for r in rows).items())},
        'bands': {name: {'anchors': int((mask[:, band].sum(1) > 0).sum()), 'points': int(mask[:, band].sum()),
                        'runs': len({r['run_id'] for r, valid in zip(rows, mask[:, band].any(1)) if valid})}
                  for name, band in {**BANDS, 'at_20m': LONG_GRID == 20}.items()}}


def read_parent(parent: Path) -> tuple[dict, list[dict], list[dict], dict[str, tuple[np.ndarray, np.ndarray]]]:
    """Verify immutable parent artifacts before using any weights or teachers."""
    record = json.loads((parent/'execution.json').read_text())
    if record['status'] != 'COMPLETE' or record['steps'] != 500 or record['execution_commit'] != PARENT_COMMIT:
        raise ValueError('unexpected parent execution identity')
    for name in ('initial.pt', 'final.pt', 'selection.json', 'teacher_audit.json', 'diagnostic_teachers.npz',
                 'selected_teachers.npz', 'predictions_final.npz', 'overfit.json'):
        check_hash(parent/name, record['artifact_hashes'][name])
    check_hash(parent/'initial.pt', PARENT_INITIAL_SHA)
    check_hash(parent/'final.pt', PARENT_FINAL_SHA)
    selected = json.loads((parent/'selection.json').read_text())['items']
    audited = json.loads((parent/'teacher_audit.json').read_text())['rows']
    with np.load(parent/'diagnostic_teachers.npz', allow_pickle=False) as data:
        np.testing.assert_array_equal(data['grid_m'], LONG_GRID)
        if len(set(data['sample_ids'])) != len(data['sample_ids']):
            raise ValueError('duplicate teacher IDs')
        arrays = {str(s): (xy.copy(), mask.copy()) for s, xy, mask in zip(data['sample_ids'], data['xy'], data['mask'])}
    return record, selected, audited, arrays


def run(parent: Path, output: Path) -> None:
    if output.exists() or not output.is_absolute() or str(output).startswith('/mnt/'):
        raise ValueError('new native Linux absolute output required')
    if subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
        raise ValueError('dirty execution checkout')
    output.mkdir(parents=True)
    wall_start = time.monotonic()
    record = {'status': 'STARTED', 'execution_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent': str(parent), 'parent_checkpoint_sha256': PARENT_FINAL_SHA, 'parent_initial_sha256': PARENT_INITIAL_SHA,
        'phase_steps': PHASES, 'active_limit_seconds': LIMIT_SECONDS, 'branch_rng_seed': 43,
        'paired_start': 'same128 and expanded256 identical step500 model+optimizer; RNG restarted equally',
        'loss_model_unchanged': True, 'validation_selection_unchanged': True, 'runtime_promoted': False,
        'test_assets_read': 0, 'phases': {}}
    write_json(output/'execution.json', record)
    try:
        old, selected, audited, arrays = read_parent(parent)
        record['parent_execution_sha256'] = sha((parent/'execution.json').read_bytes())
        old_train = [r for r in selected if r['split'] == 'train']
        validation = [r for r in selected if r['split'] == 'validation']
        if len(old_train) != 128 or len(validation) != 64:
            raise ValueError('parent cohort size mismatch')
        expanded = expand_selection(old_train, audited)
        if len(expanded) != 256:
            raise ValueError('expanded cohort size mismatch')
        items = expanded + validation
        target = np.stack([arrays[r['sample_id']][0] for r in items])
        mask = np.stack([arrays[r['sample_id']][1] for r in items])
        if target.shape != (320, 46, 2) or mask.dtype != bool or not mask.any(1).all():
            raise ValueError('invalid selected teacher shape/support')
        if {r['run_id'] for r in expanded} & {r['run_id'] for r in validation}:
            raise ValueError('train/validation run overlap')
        old_ids, expanded_ids, val_ids = list(range(128)), list(range(256)), list(range(256, 320))
        small_names = json.loads((parent/'overfit.json').read_text())['sample_ids']
        index = {r['sample_id']: i for i, r in enumerate(items)}
        tiny_ids = [index[s] for s in small_names]
        if len(tiny_ids) != 8 or not set(tiny_ids) <= set(old_ids):
            raise ValueError('tiny cohort mismatch')
        write_json(output/'selection.json', {'items': items, 'identity': digest(items), 'tiny_ids': tiny_ids,
            'cohorts': {name: cohort_summary([items[i] for i in ids], mask[ids]) for name, ids in
                        {'tiny8': tiny_ids, 'same128': old_ids, 'expanded256': expanded_ids, 'validation': val_ids}.items()},
            'frozen_before_learning': True, 'selection_rule': 'double original run/support-bin quotas, >=0.5s same-run gap'})
        np.savez_compressed(output/'teachers.npz', xy=target, mask=mask, grid_m=LONG_GRID,
                            sample_ids=np.array([r['sample_id'] for r in items]))
        access = LongAccess(Path(old['config']['dataset_root']), Path(old['config']['split_manifest']))
        keys = epoch_keys(access.rows)
        # Regenerate every used target from its same-anchor, hash-verified canonical future.
        for i, row in enumerate(items):
            canonical = access.rows[row['row_index']]
            if (canonical['sample_id'] != row['sample_id'] or canonical['run_id'] != row['run_id']
                    or access.splits[row['run_id']] != row['split'] or list(keys[row['row_index']]) != row['epoch_key']):
                raise ValueError('canonical row/epoch/split identity mismatch')
            blob = access.read(row['future_path'])
            if sha(blob) != row['future_sha256']:
                raise ValueError('future identity mismatch')
            view, reasons = audit_target(np.load(io.BytesIO(blob), allow_pickle=False))
            access.future_blobs.clear()
            if reasons:
                raise ValueError('previously eligible geometry changed')
            np.testing.assert_array_equal(view['xy'], target[i])
            np.testing.assert_array_equal(view['mask'], mask[i])
            for j in select_epoch_history(keys, anchor_index=row['row_index'], length=4).indices:
                access.allowed_sensor_paths.update(access.rows[j][k] for k in ('image_path', 'lidar_path', 'lidar_valid_path'))
        batches, histories = [], []
        for row in items:
            batch, history = build_inputs(access.rows, row['row_index'], access.read, keys=keys)
            batches.append(batch)
            histories.append(history)
        write_json(output/'input_provenance.json', {'histories': histories, 'asset_hashes': access.read_hashes,
            'manifest_sha256': sha(access.manifest_bytes), 'dataset_identity': access.manifest['manifest_sha256'], 'split_sha256': SPLIT_HASH})
        record['code_hashes'] = {p: sha(Path(p).read_bytes()) for p in subprocess.check_output(
            ['git', 'ls-files', 'src'], text=True).splitlines()}
        record['settings'] = {'dtype': 'float32', 'batch_size': 8, 'microbatch': 2, 'backbone_lr': 1e-4,
                              'head_lr': 1e-3, 'clip_norm': 1., 'augmentation': False, 'schedule': None,
                              'teacher_policy': old['teacher_policy'], 'input_contract': old['input_contract']}
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable')
        torch.set_num_threads(4)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        torch.cuda.reset_peak_memory_stats()
        record['environment'] = {'torch': torch.__version__, 'cuda': torch.version.cuda, 'gpu': torch.cuda.get_device_name(0)}
        active = time.monotonic()
        def check_time() -> None:
            if time.monotonic()-active >= LIMIT_SECONDS:
                raise TimeoutError('global diagnostic active budget exhausted')

        def predict(model: SpatialPathLongV4, ids: list[int]) -> np.ndarray:
            model.eval()
            values = []
            with torch.no_grad():
                for offset in range(0, len(ids), 2):
                    check_time()
                    part = ids[offset:offset+2]
                    values.append(model(combine_inputs([batches[i] for i in part], 'cuda')).cpu().numpy())
            return np.concatenate(values)

        def snapshot(model: SpatialPathLongV4, phase: str, step: int, ids: list[int]) -> dict:
            predictions = predict(model, ids)
            rows = [items[i] for i in ids]
            metrics = evaluate_metrics(predictions, target[ids], mask[ids], rows)
            np.savez_compressed(output/f'{phase}_{step:04d}_predictions.npz', xy=predictions,
                                sample_ids=np.array([r['sample_id'] for r in rows]))
            # Always evaluate old128 and fixed val64 for a paired comparison.
            if phase != 'tiny8':
                mapping = {i: j for j, i in enumerate(ids)}
                common_ids = old_ids + val_ids
                common_pred = predictions[[mapping[i] for i in common_ids]]
                metrics['common128_and_validation'] = evaluate_metrics(common_pred, target[common_ids], mask[common_ids],
                                                                      [items[i] for i in common_ids])
            write_json(output/f'{phase}_{step:04d}_metrics.json', metrics)
            print(json.dumps({'phase': phase, 'additional_step': step,
                'train_far_m': metrics['train']['bands']['far_10_20m']['anchor_mean_error_m'],
                'val_far_m': metrics['validation']['bands']['far_10_20m']['anchor_mean_error_m']}), flush=True)
            return metrics

        original_predictions = np.load(parent/'predictions_final.npz', allow_pickle=False)['xy']
        final_state = torch.load(parent/'final.pt', map_location='cpu', weights_only=True)
        initial_state = torch.load(parent/'initial.pt', map_location='cpu', weights_only=True)
        if final_state['steps'] != 500 or final_state['selection_identity'] != digest(selected):
            raise ValueError('checkpoint metadata identity mismatch')
        for phase, step_limit in PHASES.items():
            check_time()
            phase_start = time.monotonic()
            torch.manual_seed(43)
            np.random.seed(43)
            model = SpatialPathLongV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float().cuda()
            model.load_state_dict((initial_state if phase == 'tiny8' else final_state)['model'], strict=True)
            optimizer = optimizer_for(model)
            if phase != 'tiny8':
                optimizer.load_state_dict(final_state['optimizer'])
                replay = predict(model, old_ids + val_ids)
                np.testing.assert_allclose(replay, original_predictions, rtol=1e-5, atol=2e-5)
            torch.manual_seed(43)  # Same stochastic start for paired branches, not exact historical RNG resume.
            rng = random.Random(43)
            train_ids = tiny_ids if phase == 'tiny8' else old_ids if phase == 'same128' else expanded_ids
            eval_ids = tiny_ids if phase == 'tiny8' else train_ids + val_ids
            phase_record = {'steps': 0, 'initialization': 'original_random_initial' if phase == 'tiny8' else 'step500_model_and_optimizer',
                            'train_anchors': len(train_ids), 'initial_metrics': snapshot(model, phase, 0, eval_ids)}
            record['phases'][phase] = phase_record
            write_json(output/'execution.json', record)
            exposures = Counter()
            with (output/f'{phase}_training.jsonl').open('x') as log:
                for step in range(1, step_limit+1):
                    check_time()
                    if time.monotonic()-active > LIMIT_SECONDS-120:
                        raise TimeoutError('diagnostic reserve reached before complete paired comparison')
                    ids = list(tiny_ids) if phase == 'tiny8' else rng.sample(train_ids, 8)
                    exposures.update(ids)
                    model.train()
                    optimizer.zero_grad(set_to_none=True)
                    loss_value = 0.
                    for offset in range(0, 8, 2):
                        part = ids[offset:offset+2]
                        prediction = model(combine_inputs([batches[i] for i in part], 'cuda'))
                        loss = band_loss(prediction, torch.from_numpy(target[part]).cuda(), torch.from_numpy(mask[part]).cuda())
                        if loss is None:
                            raise ValueError('empty training support')
                        (loss/4).backward()
                        loss_value += float(loss.detach())/4
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 1., error_if_nonfinite=True)
                    optimizer.step()
                    if not all(torch.isfinite(p).all() for p in model.parameters()):
                        raise ValueError('nonfinite model parameter')
                    result = {'step': step, 'loss': loss_value, 'gradient_norm': float(norm)}
                    log.write(json.dumps(result)+'\n')
                    log.flush()
                    phase_record['steps'] = step
                    if step % 100 == 0:
                        print(json.dumps({'phase': phase, **result}), flush=True)
                    milestones = (100, 500, 1000) if phase == 'tiny8' else (500, 1000, 2000)
                    if step in milestones:
                        metrics = snapshot(model, phase, step, eval_ids)
                        phase_record['latest_metrics'] = metrics
                        phase_record['exposures'] = {items[i]['sample_id']: exposures[i] for i in train_ids}
                        phase_record['mean_additional_presentations'] = sum(exposures.values())/len(train_ids)
                        write_json(output/'execution.json', record)
                        if step in (1000, 2000):
                            torch.save({'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                                'phase': phase, 'additional_steps': step, 'parent_steps': 0 if phase == 'tiny8' else 500,
                                'grid_m': LONG_GRID.tolist(), 'execution_commit': record['execution_commit'],
                                'selection_identity': digest(items), 'python_rng_state': rng.getstate(),
                                'torch_rng_state': torch.get_rng_state(), 'cuda_rng_state': torch.cuda.get_rng_state(),
                                'tier': 'OBSERVED_DIAGNOSTIC_ONLY'}, output/f'{phase}_{step:04d}.pt')
                phase_record['active_seconds'] = time.monotonic()-phase_start
            # Verify each final saved checkpoint can be strictly restored and replayed.
            saved = torch.load(output/f'{phase}_{step_limit:04d}.pt', map_location='cpu', weights_only=True)
            model.load_state_dict(saved['model'], strict=True)
            final_pred = np.load(output/f'{phase}_{step_limit:04d}_predictions.npz')['xy']
            restored = predict(model, eval_ids[:2])
            np.testing.assert_allclose(restored, final_pred[:2], rtol=1e-5, atol=2e-5)
            phase_record['checkpoint_reload_max_abs_m'] = float(np.abs(restored-final_pred[:2]).max())
            del model, optimizer, saved
            torch.cuda.empty_cache()
        for path, expected in list(access.read_hashes.items()):
            check_hash(access.root/path, expected)
        check_hash(access.root/'manifest.yaml', sha(access.manifest_bytes))
        check_hash(Path(old['config']['split_manifest']), SPLIT_HASH)
        check_hash(parent/'final.pt', PARENT_FINAL_SHA)
        record.update(status='COMPLETE', active_seconds=time.monotonic()-active,
                      peak_allocated_bytes=torch.cuda.max_memory_allocated(), post_run_identity='PASS')
    except Exception:
        record.update(status='FAILED', traceback=traceback.format_exc())
        raise
    finally:
        record['wall_seconds'] = time.monotonic()-wall_start
        record['artifact_hashes'] = {p.name: sha(p.read_bytes()) for p in output.iterdir() if p.is_file() and p.name != 'execution.json'}
        write_json(output/'execution.json', record)
        print(json.dumps({'status': record['status'], 'output': str(output)}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.parent, args.output)
