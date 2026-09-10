"""Use every previously audited train teacher, with bounded native input caching."""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
import io
import json
from pathlib import Path
import random
import shutil
import subprocess
import time
import traceback

import numpy as np
import torch

from .spatial_long_diagnosis_v4 import read_parent, restore_optimizer, cohort_summary
from .spatial_long_v4 import LongAccess, audit_target, band_loss, evaluate_metrics
from .spatial_diagnostic_v4 import sha, digest, check_hash, write_json, optimizer_for, SPLIT_HASH
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS, build_inputs, combine_inputs, epoch_keys
from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from aic_transfuser_lite.models.temporal.gru import select_epoch_history

CHECKPOINT_SHA = 'dc350ed3e892ecdb265795328aef0c2f6940f03a0f257e45e585d13c13362bf9'
EPOCHS = 16
ACTIVE_SECONDS = 2400


def all_train(audited: list[dict]) -> list[dict]:
    """Select all eligible train anchors, without validation or distance thinning."""
    if len({r['sample_id'] for r in audited}) != len(audited):
        raise ValueError('duplicate audit sample IDs')
    rows = [r for r in audited if r['split'] == 'train' and r['diagnostic_eligible']]
    if not rows or any(r['reasons'] for r in rows):
        raise ValueError('no eligible train rows or contradictory audit')
    return sorted(rows, key=lambda r: (r['run_id'], r['stamp_ns'], r['sample_id']))


def epoch_batches(count: int, epoch: int, batch_size: int = 8) -> list[list[int]]:
    """Exactly one visit per anchor/epoch, including a smaller final batch."""
    if count < 1 or epoch < 1 or batch_size < 1:
        raise ValueError('positive count/epoch/batch size required')
    ids = list(range(count))
    random.Random(44 + epoch).shuffle(ids)
    return [ids[i:i+batch_size] for i in range(0, count, batch_size)]


class InputCache:
    """Hash-verified disk tensors; bounded CPU LRU, teacher fields forbidden.

    Each .pt contains only the nine INPUT_FIELDS of a [B=1] ModelBatchV3.
    The cache preserves preprocessing float32 bytes; no precision conversion.
    """
    def __init__(self, root: Path, hashes: dict[int, str], capacity: int = 16):
        if capacity < 1:
            raise ValueError('positive cache capacity required')
        self.root, self.hashes, self.capacity = root, hashes, capacity
        self.memory: OrderedDict[int, ModelBatchV3] = OrderedDict()

    @staticmethod
    def save(path: Path, batch: ModelBatchV3) -> str:
        batch.validate()
        if path.exists() or batch.batch_size != 1 or batch.targets is not None:
            raise ValueError('new input-only single-anchor cache required')
        torch.save({k: getattr(batch, k).cpu() for k in INPUT_FIELDS}, path)
        return sha(path.read_bytes())

    def get(self, index: int) -> ModelBatchV3:
        if index not in self.hashes:
            raise ValueError('unknown cache index')
        if index in self.memory:
            self.memory.move_to_end(index)
            return self.memory[index]
        blob = check_hash(self.root/f'{index:05d}.pt', self.hashes[index])
        values = torch.load(io.BytesIO(blob), map_location='cpu', weights_only=True)
        if set(values) != set(INPUT_FIELDS):
            raise ValueError('unexpected cached fields')
        batch = ModelBatchV3(**values, targets=None, requested_outputs=frozenset({'trajectory'}))
        batch.validate()
        if batch.batch_size != 1:
            raise ValueError('cache must contain one anchor')
        self.memory[index] = batch
        if len(self.memory) > self.capacity:
            self.memory.popitem(last=False)
        return batch


def run(parent: Path, checkpoint: Path, output: Path) -> None:
    if output.exists() or not output.is_absolute() or str(output).startswith('/mnt/'):
        raise ValueError('new native Linux output required')
    if subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
        raise ValueError('dirty execution checkout')
    output.mkdir(parents=True)
    if shutil.disk_usage(output).free < 12 * 1024**3:
        raise ValueError('at least 12 GiB free required for native input cache/checkpoints')
    wall = time.monotonic()
    record = {'status': 'STARTED', 'execution_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent': str(parent), 'checkpoint': str(checkpoint), 'checkpoint_sha256': CHECKPOINT_SHA,
        'epochs_limit': EPOCHS, 'active_seconds_limit': ACTIVE_SECONDS, 'optimizer_steps': 0, 'epochs_completed': 0,
        'seed': 44, 'test_assets_read': 0, 'runtime_promoted': False, 'initialization': 'expanded256_step2500_model_and_optimizer'}
    write_json(output/'execution.json', record)
    try:
        old, original, audited, arrays = read_parent(parent)
        check_hash(checkpoint, CHECKPOINT_SHA)
        train = all_train(audited)
        validation = [r for r in original if r['split'] == 'validation']
        if len(train) != 1786 or len(validation) != 64:
            raise ValueError('unexpected frozen corpus size')
        if {r['run_id'] for r in train} & {r['run_id'] for r in validation}:
            raise ValueError('run split overlap')
        items = train + validation
        index = {r['sample_id']: i for i, r in enumerate(items)}
        fixed_ids = [index[r['sample_id']] for r in original]
        target = np.stack([arrays[r['sample_id']][0] for r in items])
        mask = np.stack([arrays[r['sample_id']][1] for r in items])
        if target.shape != (1850, 46, 2) or mask.dtype != bool or not mask.any(1).all():
            raise ValueError('teacher shape/mask/support mismatch')
        write_json(output/'selection.json', {'items': items, 'identity': digest(items), 'fixed_eval_ids': fixed_ids,
            'train': cohort_summary(train, mask[:1786]), 'validation': cohort_summary(validation, mask[1786:]),
            'selection': 'ALL previously audited eligible train; no 0.5s thinning; adjacent anchors correlated'})
        np.savez_compressed(output/'teachers.npz', xy=target, mask=mask, grid_m=LONG_GRID,
                            sample_ids=np.array([r['sample_id'] for r in items]))
        access = LongAccess(Path(old['config']['dataset_root']), Path(old['config']['split_manifest']))
        keys = epoch_keys(access.rows)
        for i, row in enumerate(items):
            canonical = access.rows[row['row_index']]
            if (canonical['sample_id'] != row['sample_id'] or canonical['run_id'] != row['run_id']
                    or list(keys[row['row_index']]) != row['epoch_key'] or access.splits[row['run_id']] != row['split']):
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
        record['settings'] = {'batch_size': 8, 'microbatch': 2, 'backbone_lr': 1e-4, 'head_lr': 1e-3,
            'dtype': 'float32', 'schedule': None, 'loss': old['teacher_policy']['loss'],
            'teacher_policy': old['teacher_policy'], 'input_contract': old['input_contract'], 'cache_capacity': 16}
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
        if state['phase'] != 'expanded256' or state['additional_steps'] != 2000:
            raise ValueError('checkpoint phase/step mismatch')
        model = SpatialPathLongV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float().cuda()
        model.load_state_dict(state['model'], strict=True)
        optimizer = optimizer_for(model)
        restore_optimizer(optimizer, state['optimizer'])
        start_ages = sorted({int(s['step'].item()) for s in optimizer.state.values()})
        if start_ages != [2500]:
            raise ValueError('unexpected starting optimizer age')
        del state
        active = time.monotonic()
        def check_time() -> None:
            if time.monotonic()-active >= ACTIVE_SECONDS:
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
                'val_far_m': metrics['validation']['bands']['far_10_20m']['anchor_mean_error_m']}), flush=True)
            return predictions

        initial = evaluate(0, fixed_ids, 'fixed')
        source_pred = np.load(checkpoint.parent/'expanded256_2000_predictions.npz', allow_pickle=False)
        source_map = {s: i for i, s in enumerate(source_pred['sample_ids'])}
        expected = source_pred['xy'][[source_map[items[i]['sample_id']] for i in fixed_ids]]
        np.testing.assert_allclose(initial, expected, rtol=1e-5, atol=2e-5)
        record['initial_replay_max_abs_m'] = float(np.abs(initial-expected).max())
        visits = Counter()
        with (output/'training.jsonl').open('x') as log:
            for epoch in range(1, EPOCHS+1):
                losses = []
                for ids in epoch_batches(len(train), epoch):
                    check_time()
                    if time.monotonic()-active > ACTIVE_SECONDS-180:
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
                if len(visits) != len(train) or any(visits[i] != epoch for i in range(len(train))):
                    raise ValueError('epoch did not visit every train teacher exactly once')
                record['epochs_completed'] = epoch
                record['presentations_per_anchor'] = epoch
                print(json.dumps({'phase': 'training', 'epoch': epoch, 'steps': record['optimizer_steps'],
                                  'mean_batch_loss': float(np.mean(losses)), 'all_train_visited': len(visits)}), flush=True)
                if epoch in (4, 8, 12, 16):
                    evaluate(epoch, fixed_ids, 'fixed')
                    torch.save({'model': model.state_dict(), 'optimizer': optimizer.state_dict(), 'epoch': epoch,
                        'additional_steps': record['optimizer_steps'], 'parent_optimizer_steps': 2500,
                        'grid_m': LONG_GRID.tolist(), 'selection_identity': digest(items),
                        'execution_commit': record['execution_commit'], 'torch_rng_state': torch.get_rng_state(),
                        'cuda_rng_state': torch.cuda.get_rng_state(), 'tier': 'OBSERVED_DIAGNOSTIC_ONLY'}, output/f'epoch_{epoch:02d}.pt')
                write_json(output/'execution.json', record)
        evaluate(EPOCHS, list(range(len(items))), 'all_train_and_validation')
        saved = torch.load(output/'epoch_16.pt', map_location='cpu', weights_only=True)
        model.load_state_dict(saved['model'], strict=True)
        replay = predict(fixed_ids[:2])
        expected = np.load(output/'fixed_16_predictions.npz')['xy'][:2]
        np.testing.assert_allclose(replay, expected, rtol=1e-5, atol=2e-5)
        record['checkpoint_replay_max_abs_m'] = float(np.abs(replay-expected).max())
        for path, expected_hash in list(access.read_hashes.items()):
            check_hash(access.root/path, expected_hash)
        for i, expected_hash in hashes.items():
            check_hash(cache_root/f'{i:05d}.pt', expected_hash)
        check_hash(checkpoint, CHECKPOINT_SHA)
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
