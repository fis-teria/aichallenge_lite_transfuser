"""Finite V4-10 training with native on-demand inputs and host-volume guards."""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
import io
import json
from pathlib import Path
import shutil
import subprocess
import time
import traceback

import numpy as np
import torch
from torch.nn import functional as F

from .spatial_long_expand10_v4 import expand_ten_metre_corpus, same_epoch, check_preparation_time
from .spatial_long_full_v4 import epoch_batches
from .spatial_long_diagnosis_v4 import read_parent
from .spatial_long_v4 import LongAccess, audit_target, band_loss, evaluate_metrics
from .spatial_diagnostic_v4 import sha, digest, check_hash, write_json, optimizer_for, SPLIT_HASH
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS, build_inputs, combine_inputs, epoch_keys
from aic_transfuser_lite.models.spatial_path_ten_v4 import SpatialPathTenV4
from aic_transfuser_lite.models.temporal.gru import select_epoch_history

TEN_GRID = LONG_GRID[:36].copy()
SOURCE_SHA = '07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33'
EPOCHS = 2
ACTIVE_SECONDS = 3600


def check_host_space() -> None:
    """D: backs this recovered WSL VHDX; keep 3 GiB physical free as reserve."""
    if not Path('/mnt/d').is_mount() or shutil.disk_usage('/mnt/d').free < 3 * 1024**3:
        raise RuntimeError('physical WSL backing volume D: unavailable or below 3 GiB free')


def loss_ten(pred: torch.Tensor, target: torch.Tensor, mask: torch.Tensor) -> torch.Tensor | None:
    """[B,36,2] metres, bool [B,36]; equal supported near/middle bands."""
    if pred.ndim != 3 or pred.shape != target.shape or pred.shape[1:] != (36, 2) or mask.shape != pred.shape[:2] or mask.dtype != torch.bool:
        raise ValueError('V4-10 shape/mask mismatch')
    return band_loss(F.pad(pred, (0, 0, 0, 10)), F.pad(target, (0, 0, 0, 10)), F.pad(mask, (0, 10), value=False))


def metrics_ten(pred: np.ndarray, target: np.ndarray, mask: np.ndarray, items: list[dict]) -> dict:
    if pred.shape != target.shape or pred.shape[1:] != (36, 2) or mask.shape != pred.shape[:2]:
        raise ValueError('V4-10 metric shape mismatch')
    result = evaluate_metrics(np.pad(pred, ((0,0),(0,10),(0,0))),
        np.pad(target, ((0,0),(0,10),(0,0))), np.pad(mask, ((0,0),(0,10))), items)
    for split, values in result.items():
        count = sum(r['split'] == split for r in items)
        values['unobserved_points'] -= count * 10
        values['bands'] = {k:v for k,v in values['bands'].items() if k not in ('far_10_20m','at_20m')}
        values['horizon_m'] = 10
    return result


def cohort_ten(items: list[dict], mask: np.ndarray) -> dict:
    return {'anchors': len(items), 'runs': len({r['run_id'] for r in items}),
        'normal_recovery': dict(Counter(r['normal_recovery'] for r in items)),
        'at10_anchors': int(mask[:, -1].sum()), 'observed_points': int(mask.sum()), 'horizon_m': 10}


class OnDemandInputs:
    """Bounded CPU LRU; regenerate input-only float32 batches, no disk cache."""
    def __init__(self, access: LongAccess, items: list[dict], keys: list, capacity: int = 16):
        if capacity < 1:
            raise ValueError('positive cache capacity required')
        self.access, self.items, self.keys, self.capacity = access, items, keys, capacity
        self.memory: OrderedDict[int, ModelBatchV3] = OrderedDict()
        self.histories: dict[int, dict] = {}

    def get(self, index: int) -> ModelBatchV3:
        if not 0 <= index < len(self.items):
            raise ValueError('unknown input index')
        if index in self.memory:
            self.memory.move_to_end(index)
            return self.memory[index]
        batch, history = build_inputs(self.access.rows, self.items[index]['row_index'], self.access.read, keys=self.keys)
        batch.validate()
        if batch.batch_size != 1 or batch.targets is not None:
            raise ValueError('input-only single anchor required')
        self.histories[index] = history
        self.memory[index] = batch
        if len(self.memory) > self.capacity:
            self.memory.popitem(last=False)
        return batch


def run(parent: Path, checkpoint: Path, output: Path) -> None:
    if output.exists() or not output.is_absolute() or str(output).startswith('/mnt/'):
        raise ValueError('new native Linux output required')
    if subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
        raise ValueError('dirty execution checkout')
    check_host_space()
    output.mkdir(parents=True)
    if shutil.disk_usage(output).free < 3 * 1024**3:
        raise ValueError('at least 3 GiB native free required')
    wall = time.monotonic()
    record = {'status': 'STARTED', 'execution_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent': str(parent), 'checkpoint': str(checkpoint), 'checkpoint_sha256': SOURCE_SHA,
        'epochs_limit': EPOCHS, 'active_seconds_limit': ACTIVE_SECONDS, 'optimizer_steps': 0, 'epochs_completed': 0,
        'seed': 44, 'test_assets_read': 0, 'runtime_promoted': False, 'initialization': 'V4_20_epoch12_first36_points_NEW_optimizer'}
    write_json(output/'execution.json', record)
    try:
        old, original, audited, arrays = read_parent(parent)
        check_hash(checkpoint, SOURCE_SHA)
        train, arrays = expand_ten_metre_corpus(old, audited, arrays, output, wall)
        validation = [r for r in original if r['split'] == 'validation']
        if len(train) <= 1786 or len(validation) != 64:
            raise ValueError('unexpected frozen corpus size')
        if {r['run_id'] for r in train} & {r['run_id'] for r in validation}:
            raise ValueError('run split overlap')
        items = train + validation
        index = {r['sample_id']: i for i, r in enumerate(items)}
        fixed_ids = [index[r['sample_id']] for r in original]
        target = np.stack([arrays[r['sample_id']][0][:36] for r in items])
        mask = np.stack([arrays[r['sample_id']][1][:36] for r in items])
        if target.shape != (len(items), 36, 2) or mask.dtype != bool or not mask.any(1).all():
            raise ValueError('teacher shape/mask/support mismatch')
        write_json(output/'selection.json', {'items': items, 'identity': digest(items), 'fixed_eval_ids': fixed_ids,
            'train': cohort_ten(train, mask[:len(train)]), 'validation': cohort_ten(validation, mask[len(train):]),
            'selection': 'All accepted >=10m train candidates UNION prior 1786; no thinning; adjacent anchors correlated'})
        np.savez_compressed(output/'teachers.npz', xy=target, mask=mask, grid_m=TEN_GRID,
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
            np.testing.assert_array_equal(view['xy'][:36], target[i])
            np.testing.assert_array_equal(view['mask'][:36], mask[i])
            for j in select_epoch_history(keys, anchor_index=row['row_index'], length=4).indices:
                access.allowed_sensor_paths.update(access.rows[j][k] for k in ('image_path', 'lidar_path', 'lidar_valid_path'))
        cache = OnDemandInputs(access, items, keys)
        # Independently regenerate two real anchors and compare every input tensor.
        for i in (fixed_ids[0], fixed_ids[-1]):
            rebuilt, _ = build_inputs(access.rows, items[i]['row_index'], access.read, keys=keys)
            loaded = cache.get(i)
            for field in INPUT_FIELDS:
                torch.testing.assert_close(getattr(rebuilt, field), getattr(loaded, field), rtol=0, atol=0)
        write_json(output/'input_provenance.json', {'mode': 'on_demand_float32_CPU_LRU_16',
            'manifest_sha256': sha(access.manifest_bytes), 'split_sha256': SPLIT_HASH,
            'cache_tensor_parity': 'PASS_TWO_REAL_ANCHORS_EXACT', 'disk_input_cache_bytes': 0})
        record['preparation_seconds'] = time.monotonic()-wall
        record['settings'] = {'batch_size': 8, 'microbatch': 2, 'backbone_lr': 1e-4, 'head_lr': 1e-3,
            'dtype': 'float32', 'schedule': None, 'loss': 'equal near/middle observed bands per anchor; no points beyond 10m',
            'teacher_policy': {**old['teacher_policy'], 'minimum_selected_gap_s': None, 'grid_m': TEN_GRID.tolist()},
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
        model = SpatialPathTenV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float().cuda()
        model.initialize_from_twenty(state['model'])
        optimizer = optimizer_for(model)
        if optimizer.state:
            raise ValueError('new V4-10 optimizer must be empty')
        del state
        active = time.monotonic()
        def check_time() -> None:
            if time.monotonic()-active >= ACTIVE_SECONDS:
                raise TimeoutError('active budget exceeded')
            check_host_space()

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
            metrics = metrics_ten(predictions, target[ids], mask[ids], [items[i] for i in ids])
            write_json(output/f'{name}_{epoch:02d}_metrics.json', metrics)
            np.savez_compressed(output/f'{name}_{epoch:02d}_predictions.npz', xy=predictions,
                                sample_ids=np.array([items[i]['sample_id'] for i in ids]))
            print(json.dumps({'phase': name, 'epoch': epoch,
                'val_near_m': metrics['validation']['bands']['near_0_2m']['anchor_mean_error_m'],
                'val_at10_m': metrics['validation']['bands']['at_10m']['anchor_mean_error_m'],
                'val_middle_m': metrics['validation']['bands']['middle_2_10m']['anchor_mean_error_m']}), flush=True)
            return predictions

        initial = evaluate(0, fixed_ids, 'fixed')
        source_record = json.loads((checkpoint.parent/'execution.json').read_text())
        if source_record['status'] != 'COMPLETE':
            raise ValueError('source training incomplete')
        check_hash(checkpoint.parent/'fixed_12_predictions.npz',
                   source_record['artifact_hashes']['fixed_12_predictions.npz'])
        source_pred = np.load(checkpoint.parent/'fixed_12_predictions.npz', allow_pickle=False)
        source_map = {s: i for i, s in enumerate(source_pred['sample_ids'])}
        expected = source_pred['xy'][[source_map[items[i]['sample_id']] for i in fixed_ids], :36]
        np.testing.assert_allclose(initial, expected, rtol=1e-5, atol=2e-5)
        record['initial_replay_max_abs_m'] = float(np.abs(initial-expected).max())
        visits = Counter()
        with (output/'training.jsonl').open('x') as log:
            for epoch in range(1, EPOCHS+1):
                losses = []
                for ids in epoch_batches(len(train), epoch):
                    check_time()
                    if time.monotonic()-active > ACTIVE_SECONDS-600:
                        raise TimeoutError('final evaluation reserve reached before all epochs')
                    model.train()
                    optimizer.zero_grad(set_to_none=True)
                    total = 0.
                    for offset in range(0, len(ids), 2):
                        part = ids[offset:offset+2]
                        prediction = model(combine_inputs([cache.get(i) for i in part], 'cuda'))
                        loss = loss_ten(prediction, torch.from_numpy(target[part]).cuda(), torch.from_numpy(mask[part]).cuda())
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
                        'additional_steps': record['optimizer_steps'], 'source_optimizer_steps_not_loaded': 5188,
                        'grid_m': TEN_GRID.tolist(), 'model_type': 'SpatialPathTenV4', 'selection_identity': digest(items),
                        'execution_commit': record['execution_commit'], 'torch_rng_state': torch.get_rng_state(),
                        'cuda_rng_state': torch.cuda.get_rng_state(), 'tier': 'OBSERVED_DIAGNOSTIC_ONLY'}, output/f'epoch_{epoch:02d}.pt')
                write_json(output/'execution.json', record)
        evaluate(EPOCHS, list(range(len(items))), 'all_train_and_validation')
        saved = torch.load(output/'epoch_02.pt', map_location='cpu', weights_only=True)
        model.load_state_dict(saved['model'], strict=True)
        replay = predict(fixed_ids[:2])
        expected = np.load(output/'fixed_02_predictions.npz')['xy'][:2]
        np.testing.assert_allclose(replay, expected, rtol=1e-5, atol=2e-5)
        record['checkpoint_replay_max_abs_m'] = float(np.abs(replay-expected).max())
        for path, expected_hash in list(access.read_hashes.items()):
            check_hash(access.root/path, expected_hash)
        write_json(output/'input_provenance.json', {'mode': 'on_demand_float32_CPU_LRU_16',
            'manifest_sha256': sha(access.manifest_bytes), 'split_sha256': SPLIT_HASH,
            'cache_tensor_parity': 'PASS_TWO_REAL_ANCHORS_EXACT', 'disk_input_cache_bytes': 0,
            'histories': cache.histories, 'asset_hashes': access.read_hashes})
        check_hash(checkpoint, SOURCE_SHA)
        check_hash(access.root/'manifest.yaml', sha(access.manifest_bytes))
        check_hash(Path(old['config']['split_manifest']), SPLIT_HASH)
        record.update(status='COMPLETE', active_seconds=time.monotonic()-active,
            peak_allocated_bytes=torch.cuda.max_memory_allocated(), post_run_identity='PASS',
            train_presentations=sum(visits.values()), cache_bytes=0)
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
