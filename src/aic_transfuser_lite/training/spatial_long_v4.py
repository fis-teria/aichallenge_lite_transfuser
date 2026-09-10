"""Bounded 20 m observed-teacher diagnostic; separate from fixed 2 m/runtime.

Train and validation assets are hash verified; test futures/sensors are unread.
Coordinates are base_link@t_obs metres. No physical teacher adoption is implied.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import csv
import io
import json
from pathlib import Path
import random
import subprocess
import time
import traceback

import numpy as np
import torch
from torch.nn import functional as F
import yaml

from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID, long_spatial_target
from aic_transfuser_lite.data.spatial_diagnostic_view_v4 import polyline_length, self_intersects
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import (
    INPUT_CONTRACT, FEATURES, EGO_LIMITS, build_inputs, combine_inputs, epoch_keys,
)
from aic_transfuser_lite.data.dataset_view_v3 import _ego_row
from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from aic_transfuser_lite.models.temporal.gru import select_epoch_history
from .spatial_diagnostic_v4 import (
    CanonicalAccess, SPLIT_HASH, LEDGER_HASH, check_hash, sha, digest, write_json,
    deterministic_candidates, optimizer_for,
)

BANDS = {'near_0_2m': LONG_GRID <= 2, 'middle_2_10m': (LONG_GRID > 2) & (LONG_GRID <= 10),
         'far_10_20m': LONG_GRID > 10}
POLICY = {'tier': 'OBSERVED_DIAGNOSTIC_ONLY', 'runtime_teacher_eligibility': 'UNKNOWN',
          'grid_m': LONG_GRID.tolist(), 'origin_scored': False, 'extrapolation': False,
          'max_deviation_upper_m': 0.15, 'max_corner_cut_floor_m': 0.1,
          'max_corner_cut_fraction': 0.02, 'deviation_sample_spacing_m': 0.05,
          'threshold_basis': 'predeclared provisional geometry diagnostics, not safety tolerances',
          'loss': 'equal present bands per anchor, equal observed points per band, equal supported anchors',
          'minimum_selected_gap_s': 0.5}


def directed_distance_upper(source: np.ndarray, reference: np.ndarray) -> float:
    """Polyline directed distance upper bound in m, sampled every <=0.05 m.

    Distance to a set is 1-Lipschitz; add half the maximum sampling gap.
    Origin is included for geometry only. Inputs are finite [N,2], N>=1.
    """
    if any(p.ndim != 2 or p.shape[1] != 2 or len(p) == 0 or not np.isfinite(p).all()
           for p in (source, reference)):
        raise ValueError('invalid geometry polyline')
    samples, gap = [source[:1]], 0.0
    for a, b in zip(source[:-1], source[1:]):
        length = float(np.linalg.norm(b-a))
        count = max(1, int(np.ceil(length / POLICY['deviation_sample_spacing_m'])))
        samples.append(a + np.arange(1, count+1)[:, None] / count * (b-a))
        gap = max(gap, length/count)
    points = np.concatenate(samples)
    if len(reference) == 1:
        distance = np.linalg.norm(points-reference[0], axis=1)
    else:
        a, delta = reference[:-1], np.diff(reference, axis=0)
        denom = np.sum(delta*delta, axis=1)
        weight = np.sum((points[:, None]-a)*delta, axis=2) / np.maximum(denom, 1e-24)
        projection = a + np.clip(weight, 0, 1)[..., None]*delta
        distance = np.linalg.norm(points[:, None]-projection, axis=2).min(axis=1)
    return float(distance.max() + gap/2)


def audit_target(future: np.ndarray) -> tuple[dict, list[str]]:
    """h30 [30,8] -> long target plus full covered-horizon geometry audit."""
    view = long_spatial_target(future)
    reasons = [f for f in view['flags'] if f != 'large_resampling_corner_cut']
    if view['cut_reason'] in ('position_jump', 'invalid_time_grid_or_gap', 'nonfinite_valid'):
        reasons.append(view['cut_reason'])
    if not view['mask'].any():
        reasons.append('no_observed_grid_support')
    raw = view['reliable_prefix_xy']
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw, axis=0), axis=1))]
    covered = float(LONG_GRID[view['mask']][-1]) if view['mask'].any() else 0.
    end = np.array([np.interp(covered, arc, raw[:, a]) for a in (0, 1)])
    clipped = np.concatenate((raw[arc < covered], end[None]))
    sampled = np.concatenate((np.zeros((1, 2)), view['xy'][view['mask']]))
    upper = max(directed_distance_upper(clipped, sampled), directed_distance_upper(sampled, clipped))
    cut = max(0., covered - polyline_length(view['xy'][view['mask']]))
    if upper > POLICY['max_deviation_upper_m']:
        reasons.append('long_resampling_deviation')
    if cut > max(POLICY['max_corner_cut_floor_m'], POLICY['max_corner_cut_fraction']*covered):
        reasons.append('long_resampling_corner_cut')
    view.update(covered_grid_m=covered, long_corner_cut_m=cut, deviation_upper_m=upper,
                shape='unknown' if not covered else 'left' if end[1] > .2 else 'right' if end[1] < -.2 else 'straight')
    return view, sorted(set(reasons))


def band_loss(predicted: torch.Tensor, target: torch.Tensor, mask: torch.Tensor) -> torch.Tensor | None:
    """[B,46,2] metres, bool [B,46]. Equal observed bands then anchors.

    Unobserved target values may be NaN and have exactly zero gradient.
    Empty anchors are excluded; returns None for a wholly unsupported batch.
    """
    if (predicted.ndim != 3 or predicted.shape != target.shape or predicted.shape[1:] != (46, 2)
            or mask.shape != target.shape[:2] or mask.dtype != torch.bool):
        raise ValueError('long loss shape/mask mismatch')
    if not torch.isfinite(predicted).all() or not torch.isfinite(target[mask]).all():
        raise ValueError('nonfinite valid long loss input')
    present = mask.any(dim=1)
    if not present.any():
        return None
    safe_p = torch.where(mask[..., None], predicted, 0.)
    safe_t = torch.where(mask[..., None], target, 0.)
    point = F.smooth_l1_loss(safe_p, safe_t, beta=.1, reduction='none').mean(-1)
    losses, supports = [], []
    for band in BANDS.values():
        valid = mask & torch.as_tensor(band, device=mask.device)
        count = valid.sum(1)
        losses.append((point*valid).sum(1)/count.clamp_min(1))
        supports.append(count > 0)
    return (torch.stack(losses, 1).sum(1)/torch.stack(supports, 1).sum(1).clamp_min(1))[present].mean()


def evaluate_metrics(pred: np.ndarray, target: np.ndarray, mask: np.ndarray, items: list[dict]) -> dict:
    """Distance errors in metres with observed anchor/point/run denominators."""
    if pred.shape != target.shape or not np.isfinite(pred).all():
        raise ValueError('invalid predictions')
    err = np.linalg.norm(pred-target, axis=2)
    result = {}
    for split in ('train', 'validation'):
        ids = np.array([i for i, r in enumerate(items) if r['split'] == split], dtype=int)
        groups = {}
        for name, band in {**BANDS, **{f'at_{d}m': LONG_GRID == d for d in (2, 5, 10, 20)}}.items():
            valid = mask[ids] & band
            counts = valid.sum(1)
            values = np.where(valid, err[ids], 0).sum(1)/np.maximum(counts, 1)
            groups[name] = {'anchors': int((counts > 0).sum()), 'points': int(valid.sum()),
                'runs': len({items[i]['run_id'] for i, c in zip(ids, counts) if c > 0}),
                'anchor_mean_error_m': float(values[counts > 0].mean()) if np.any(counts) else None,
                'point_mean_error_m': float(err[ids][valid].mean()) if valid.any() else None}
        def subgroup(part: list[int]) -> dict:
            return {'anchors': len(part), 'observed_points': int(mask[part].sum()),
                'anchor_mean_error_m': float(np.mean([err[i, mask[i]].mean() for i in part if mask[i].any()])),
                'prediction_length_on_support_m': float(np.mean([polyline_length(pred[i, mask[i]]) for i in part])),
                'teacher_length_on_support_m': float(np.mean([polyline_length(target[i, mask[i]]) for i in part]))}
        result[split] = {'anchors': len(ids), 'runs': len({items[i]['run_id'] for i in ids}),
            'unobserved_points': int((~mask[ids]).sum()), 'bands': groups,
            'supported_prediction_self_intersections': sum(self_intersects(pred[i, mask[i]]) for i in ids),
            'shape_counts': dict(Counter(items[i]['geometry']['shape'] for i in ids)),
            'per_run': {run: subgroup([i for i in ids if items[i]['run_id'] == run])
                        for run in sorted({items[i]['run_id'] for i in ids})},
            'per_shape': {shape: subgroup([i for i in ids if items[i]['geometry']['shape'] == shape])
                          for shape in sorted({items[i]['geometry']['shape'] for i in ids})}}
    return result


class LongAccess(CanonicalAccess):
    """Reuse immutable identity/read guards, extend metadata to train+validation.

    Future blobs are released after each conversion; own explicit candidate budget
    is enforced by prepare. Base read still verifies every byte against manifest.
    """
    def __init__(self, root: Path, split_path: Path):
        super().__init__(root, split_path)
        assignments = json.loads(check_hash(split_path, SPLIT_HASH))['assignments']
        self.splits = {r['run_id']: r['split'] for r in assignments}
        rows = list(csv.DictReader(io.StringIO(self.read('samples.csv').decode())))
        self.rows = sorted([r for r in rows if self.splits[r['run_id']] in ('train', 'validation')],
                           key=lambda r: (r['run_id'], r['segment_id'], int(r['grid_stamp_ns'])))


def prepare(access: LongAccess, config: dict, output: Path) -> tuple[list[dict], np.ndarray, np.ndarray]:
    annotations = {r['sample_id']: r for r in csv.DictReader(io.StringIO(
        check_hash(Path(config['coverage_ledger']), LEDGER_HASH).decode()))}
    keys = epoch_keys(access.rows)
    audit, eligible, arrays = [], [], {}
    for split in ('train', 'validation'):
        indices = [i for i, row in enumerate(access.rows) if access.splits[row['run_id']] == split]
        order = deterministic_candidates([access.rows[i] for i in indices])[:config['candidates'][split]]
        for local in order:
            index = indices[local]
            row = access.rows[index]
            blob = access.read(row['trajectory_path'])
            future = np.load(io.BytesIO(blob), allow_pickle=False)
            view, reasons = audit_target(future)
            access.future_blobs.clear()
            _, ego_mask = _ego_row(row, FEATURES, abs_limits=EGO_LIMITS)
            if not bool(ego_mask.all()):
                reasons.append('invalid_current_ego')
            if float(row['velocity_longitudinal_mps']) <= .05:
                reasons.append('stopped_or_reverse_observation')
            annotation = annotations.get(row['sample_id'])
            if (annotation is None or annotation['run_id'] != row['run_id']
                    or {'val': 'validation'}.get(annotation['split'], annotation['split']) != split):
                raise ValueError('ledger sample/run/split mismatch')
            item = {'sample_id': row['sample_id'], 'row_index': index, 'run_id': row['run_id'],
                'segment_id': row['segment_id'], 'epoch_key': keys[index], 'split': split,
                'stamp_ns': int(row['grid_stamp_ns']), 'future_path': row['trajectory_path'], 'future_sha256': sha(blob),
                'normal_recovery': annotation.get('normal_recovery', 'UNKNOWN'), 'reasons': sorted(set(reasons)),
                'diagnostic_eligible': not reasons, 'runtime_teacher_eligibility': 'UNKNOWN',
                'geometry': {k: v for k, v in view.items() if k not in ('xy', 'mask', 'grid_m', 'reliable_prefix_xy')}}
            audit.append(item)
            if not reasons:
                eligible.append(item)
                arrays[item['sample_id']] = (view['xy'], view['mask'])
        print(json.dumps({'phase': 'teacher_audit', 'split': split, 'candidates': len(order),
                          'eligible': sum(r['split'] == split for r in eligible)}), flush=True)
    # Distance-support/run round-robin is fixed before training, with >=0.5 s gap.
    selected = []
    for split in ('train', 'validation'):
        buckets = defaultdict(list)
        for item in eligible:
            if item['split'] == split:
                support = item['geometry']['covered_grid_m']
                band = 20 if support >= 20 else 10 if support >= 10 else 2 if support >= 2 else 0
                buckets[(-band, item['run_id'])].append(item)
        chosen = []
        while len(chosen) < config['selected'][split] and any(buckets.values()):
            for key in sorted(buckets):
                if not buckets[key] or len(chosen) >= config['selected'][split]:
                    continue
                item = buckets[key].pop(0)
                if all(r['run_id'] != item['run_id'] or abs(r['stamp_ns']-item['stamp_ns']) >= 500_000_000 for r in chosen):
                    chosen.append(item)
        selected.extend(chosen)
    write_json(output/'teacher_audit.json', {'policy': POLICY, 'rows': audit,
        'counts': {s: {'audited': sum(r['split'] == s for r in audit),
                      'eligible': sum(r['split'] == s for r in eligible),
                      'reasons': dict(Counter(reason for r in audit if r['split'] == s for reason in r['reasons']))}
                   for s in ('train', 'validation')}, 'test_assets_read': 0})
    # Save EVERY eligible audited teacher, not just selected learning anchors.
    xy = np.stack([arrays[r['sample_id']][0] for r in eligible])
    mask = np.stack([arrays[r['sample_id']][1] for r in eligible])
    np.savez_compressed(output/'diagnostic_teachers.npz', xy=xy, mask=mask, grid_m=LONG_GRID,
        sample_ids=np.array([r['sample_id'] for r in eligible]), splits=np.array([r['split'] for r in eligible]))
    write_json(output/'selection.json', {'items': selected, 'identity': digest(selected), 'frozen_before_training': True})
    for split in ('train', 'validation'):
        part = [r for r in selected if r['split'] == split]
        if len(part) < 8 or not any(arrays[r['sample_id']][1][-1] for r in part):
            raise ValueError('insufficient selected support including 20 m: '+split)
    return selected, np.stack([arrays[r['sample_id']][0] for r in selected]), np.stack([arrays[r['sample_id']][1] for r in selected])


def run(config_path: Path, output: Path) -> None:
    config = yaml.safe_load(config_path.read_text())
    if not 1 <= config['steps'] <= 500 or not 1 <= config['active_seconds'] <= 900:
        raise ValueError('invalid diagnostic budget')
    if config['candidates'] != {'train': 2048, 'validation': 1024} or config['selected'] != {'train': 128, 'validation': 64}:
        raise ValueError('unexpected candidate/selection budget')
    if output.exists() or not output.is_absolute() or str(output).startswith('/mnt/'):
        raise ValueError('new native Linux absolute output required')
    output.mkdir(parents=True)
    started = time.monotonic()
    record = {'status': 'STARTED', 'config_sha256': sha(config_path.read_bytes()), 'config': config,
        'execution_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'initialization': 'SCRATCH', 'checkpoint_source': None, 'loaded_checkpoint_parameters': [],
        'teacher_policy': POLICY, 'input_contract': INPUT_CONTRACT, 'test_assets_read': 0,
        'runtime_promotion': False, 'steps': 0}
    write_json(output/'execution.json', record)
    try:
        if subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
            raise ValueError('dirty execution checkout')
        record['code_hashes'] = {p: sha(Path(p).read_bytes()) for p in subprocess.check_output(
            ['git', 'ls-files', 'src', 'configs/spatial_long_v4.yaml'], text=True).splitlines()}
        access = LongAccess(Path(config['dataset_root']), Path(config['split_manifest']))
        selected, target, mask = prepare(access, config, output)
        keys = epoch_keys(access.rows)
        for item in selected:
            for i in select_epoch_history(keys, anchor_index=item['row_index'], length=4).indices:
                access.allowed_sensor_paths.update(access.rows[i][k] for k in ('image_path', 'lidar_path', 'lidar_valid_path'))
        batches, histories = [], []
        for item in selected:
            batch, history = build_inputs(access.rows, item['row_index'], access.read, keys=keys)
            batches.append(batch)
            histories.append(history)
        write_json(output/'input_provenance.json', {'histories': histories, 'asset_hashes': access.read_hashes,
                                                   'dataset_summary': access.summary})
        np.savez_compressed(output/'selected_teachers.npz', xy=target, mask=mask, grid_m=LONG_GRID,
                            sample_ids=np.array([r['sample_id'] for r in selected]))
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable')
        torch.set_num_threads(4)
        torch.manual_seed(42)
        np.random.seed(42)
        random.seed(42)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        torch.cuda.reset_peak_memory_stats()
        record['environment'] = {'torch': torch.__version__, 'cuda': torch.version.cuda,
                                 'gpu': torch.cuda.get_device_name(0)}
        model = SpatialPathLongV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float().cuda()
        torch.save({'model': model.state_dict(), 'execution_commit': record['execution_commit'], 'grid_m': LONG_GRID.tolist()}, output/'initial.pt')
        train_ids = [i for i, r in enumerate(selected) if r['split'] == 'train']
        active = time.monotonic()
        def check_time() -> None:
            if time.monotonic()-active > config['active_seconds']:
                raise TimeoutError('active diagnostic budget exhausted')

        def predict(current: SpatialPathLongV4, ids: list[int]) -> np.ndarray:
            current.eval()
            predictions = []
            with torch.no_grad():
                for offset in range(0, len(ids), 2):
                    check_time()
                    part = ids[offset:offset+2]
                    predictions.append(current(combine_inputs([batches[i] for i in part], 'cuda')).cpu().numpy())
            return np.concatenate(predictions)

        def update(current: SpatialPathLongV4, optimizer: torch.optim.Optimizer, ids: list[int]) -> dict:
            current.train()
            optimizer.zero_grad(set_to_none=True)
            loss_value = 0.
            for offset in range(0, len(ids), 2):
                check_time()
                part = ids[offset:offset+2]
                pred = current(combine_inputs([batches[i] for i in part], 'cuda'))
                loss = band_loss(pred, torch.from_numpy(target[part]).cuda(), torch.from_numpy(mask[part]).cuda())
                if loss is None:
                    raise ValueError('selected batch lacks support')
                weighted = loss * len(part)/len(ids)
                weighted.backward()
                loss_value += float(weighted.detach())
            norm = torch.nn.utils.clip_grad_norm_(current.parameters(), 1., error_if_nonfinite=True)
            optimizer.step()
            if not all(torch.isfinite(p).all() for p in current.parameters()):
                raise ValueError('nonfinite updated model')
            return {'loss': loss_value, 'gradient_norm_before_clip': float(norm)}

        all_ids = list(range(len(selected)))
        initial = predict(model, all_ids)
        write_json(output/'metrics_initial.json', evaluate_metrics(initial, target, mask, selected))
        np.savez_compressed(output/'predictions_initial.npz', xy=initial)
        straight = np.broadcast_to(np.stack((LONG_GRID, np.zeros(46)), -1), target.shape)
        template = np.zeros((46, 2), dtype=np.float32)
        for k in range(46):
            valid = [i for i in train_ids if mask[i, k]]
            if not valid:
                raise ValueError('train mean baseline has unsupported grid')
            template[k] = target[valid, k].mean(0)
        write_json(output/'baselines.json', {name: evaluate_metrics(pred, target, mask, selected)
            for name, pred in {'straight': straight, 'train_mean': np.broadcast_to(template, target.shape)}.items()})
        # Independent few-anchor overfit plus first-step gradient smoke, never inherited.
        smoke = deepcopy(model)
        smoke_optimizer = optimizer_for(smoke)
        small = train_ids[:8]
        before = predict(smoke, small)
        smoke_rows = []
        for step in range(100):
            smoke_rows.append({'step': step+1, **update(smoke, smoke_optimizer, small)})
        after = predict(smoke, small)
        write_json(output/'overfit.json', {'steps': 100, 'weights_inherited': False, 'updates': smoke_rows,
            'sample_ids': [selected[i]['sample_id'] for i in small],
            'initial_observed_point_ade_m': float(np.linalg.norm(before-target[small], axis=2)[mask[small]].mean()),
            'final_observed_point_ade_m': float(np.linalg.norm(after-target[small], axis=2)[mask[small]].mean())})
        del smoke, smoke_optimizer
        optimizer = optimizer_for(model)
        rng = random.Random(42)
        with (output/'training.jsonl').open('x') as log:
            for step in range(config['steps']):
                if time.monotonic()-active > config['active_seconds']-90:
                    record['stop_reason'] = 'final_eval_reserve'
                    break
                result = {'step': step+1, **update(model, optimizer, rng.sample(train_ids, 8))}
                log.write(json.dumps(result)+'\n')
                log.flush()
                record['steps'] = step+1
                if (step+1) % 50 == 0:
                    print(json.dumps({'phase': 'training', **result}), flush=True)
        final = predict(model, all_ids)
        write_json(output/'metrics_final.json', evaluate_metrics(final, target, mask, selected))
        np.savez_compressed(output/'predictions_final.npz', xy=final)
        record['active_seconds'] = time.monotonic()-active
        torch.save({'model': model.state_dict(), 'optimizer': optimizer.state_dict(), 'steps': record['steps'],
            'execution_commit': record['execution_commit'], 'grid_m': LONG_GRID.tolist(), 'tier': 'OBSERVED_DIAGNOSTIC_ONLY',
            'config': config, 'selection_identity': digest(selected)}, output/'final.pt')
        # Re-read selected assets/futures and root identity before declaring success.
        for path, expected in list(access.read_hashes.items()):
            check_hash(access.root/path, expected)
        check_hash(access.root/'manifest.yaml', sha(access.manifest_bytes))
        check_hash(Path(config['split_manifest']), SPLIT_HASH)
        record.update(status='COMPLETE', gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
                      post_run_asset_rehash='PASS', wall_seconds=time.monotonic()-started)
    except Exception:
        record.update(status='FAILED', traceback=traceback.format_exc(), wall_seconds=time.monotonic()-started)
        raise
    finally:
        record['artifact_hashes'] = {p.name: sha(p.read_bytes()) for p in output.iterdir() if p.is_file() and p.name != 'execution.json'}
        write_json(output/'execution.json', record)
        print(json.dumps({'status': record['status'], 'steps': record['steps'], 'output': str(output)}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.config, args.output)
