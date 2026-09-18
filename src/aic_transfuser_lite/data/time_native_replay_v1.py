"""Add audited native teachers without removing an existing replay presentation."""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import Dataset

from .time_dataset_v1 import TimeSample
from .time_split_v1 import assert_split_membership, content_sha256, validate_time_split
from .time_training_cache_v1 import _sha


def extend_train_split(base: dict[str, Any], additions: list[dict[str, Any]], *,
                       group: str) -> dict[str, Any]:
    """Keep the original holdout; put every new run in the same scenario in train."""
    validate_time_split(base, require_verified=True)
    if (base['format'] != 'time_recovery_extension_v1' or not group or not additions
            or any(r.get('split') != 'train' for r in additions)):
        raise ValueError('verified existing extension and a whole train-only scenario required')
    result = copy.deepcopy(base)
    result['additional_runs'].extend(copy.deepcopy(additions))
    result['runs'] = sorted(result['base_manifest']['runs'] + result['additional_runs'],
                            key=lambda r: r['run_id'])
    result['manifest_sha256'] = content_sha256({k: v for k, v in result.items()
        if k not in {'manifest_sha256', 'sources_verified'}})
    validate_time_split(result, require_verified=True)
    return result


def check_native_replay(row: dict[str, Any], replay: dict[str, Any], sample: TimeSample,
                        xy_m: np.ndarray, velocity_mps: np.ndarray, *,
                        expected_split: str = 'unassigned',
                        expected_group: str = 'all_corners_20260918') -> None:
    """Require unchanged causal histories and all 30 observed metre/m/s targets."""
    if expected_split not in {'unassigned', 'train'} or not isinstance(expected_group, str) or not expected_group.strip():
        raise ValueError('explicit native train/unassigned scenario group required')
    if (row.get('split') != expected_split or row.get('split_group') != expected_group
            or row.get('allowed_targets') != ['xy_m', 'velocity_mps']
            or row.get('stop_label_valid') is not False or row.get('mode_label_valid') is not False
            or row.get('forward_avoidance_eligible') is not False
            or row.get('teacher_pose_prefix_eligible') is not True):
        raise ValueError('native selection identity or permitted targets changed')
    if (replay.get('history_row_ids') != row['history_row_ids'] or not replay.get('usable_full')
            or sample.anchor_id != row['anchor_id'] or sample.run != row['run_id']
            or sample.observation_ns != row['observation_ns'] or sample.freeze_ns != row['freeze_ns']):
        raise ValueError('causal history, sample identity or freeze changed')
    batch, teacher = sample.inputs, sample.teacher
    if batch is None or teacher is None or batch.targets is not None:
        raise ValueError('complete inference inputs and separately held teacher required')
    if (batch.image.shape != (1, 4, 3, 224, 384) or batch.lidar.shape != (1, 4, 2, 750)
            or not teacher.xy_mask.all() or not teacher.velocity_mask.all()
            or sample.stop_probability is not None or sample.environment_stop_intent is not None):
        raise ValueError('input shape, full teacher support or unknown stop label changed')
    if (xy_m.shape != (30, 2) or velocity_mps.shape != (30,)
            or not np.isfinite(xy_m).all() or not np.isfinite(velocity_mps).all()):
        raise ValueError('finite XY [30,2] m and velocity [30] m/s required')
    np.testing.assert_allclose(teacher.xy_m, xy_m, atol=1e-5, rtol=0)
    np.testing.assert_allclose(teacher.velocity_mps, velocity_mps, atol=1e-5, rtol=0)
    batch.validate(require_current=True)


class NativeReplayDataset(Dataset):
    """Locally generated, hash-verified TimeSample shards, memory mapped on CPU.

    Pickles are trusted only after checking the frozen local preparation hashes;
    this is not an importer for arbitrary external PyTorch objects.
    """
    def __init__(self, root: Path, *, manifest_sha256: str) -> None:
        path = root / 'native_manifest.json'
        if _sha(path) != manifest_sha256:
            raise ValueError('native manifest changed')
        self.manifest = json.loads(path.read_bytes())
        self.root = root
        self._shards: dict[str, list[TimeSample]] = {}
        self._index: list[tuple[str, int]] = []
        self.run_ids: list[str] = []
        self.anchor_ids: list[str] = []
        self.uses: list[str] = []
        for shard in self.manifest['shards']:
            path = (root / shard['path']).resolve()
            if not path.is_relative_to(root.resolve()) or _sha(path) != shard['sha256']:
                raise ValueError('native shard path or hash changed')
            self._shards[shard['path']] = torch.load(path, weights_only=False, mmap=True)
            samples = self._shards[shard['path']]
            if len(samples) != len(shard['anchors']):
                raise ValueError('native shard count changed')
            for i, row in enumerate(shard['anchors']):
                if (samples[i].run, samples[i].anchor_id) != (shard['run_id'], row['anchor_id']):
                    raise ValueError('native shard anchor order changed')
                self._index.append((shard['path'], i))
                self.run_ids.append(shard['run_id'])
                self.anchor_ids.append(row['anchor_id'])
                self.uses.append(row['selection_use'])
        if not self.anchor_ids or len(set(self.anchor_ids)) != len(self.anchor_ids):
            raise ValueError('nonempty unique native anchors required')

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, index: int) -> TimeSample:
        name, row = self._index[index]
        return self._shards[name][row]


class AdditiveReplayDataset(Dataset):
    """Preserve every old slot, then append whole native passes; shuffle at training."""
    def __init__(self, previous: Any, native: Any, *, repeats: int,
                 split_manifest: dict[str, Any]) -> None:
        if type(repeats) is not int or repeats <= 0 or not len(previous) or not len(native):
            raise ValueError('nonempty datasets and positive integer native repeats required')
        if set(previous.run_ids) & set(native.run_ids) or set(previous.anchor_ids) & set(native.anchor_ids):
            raise ValueError('native and replay sources must be disjoint')
        if len(set(native.anchor_ids)) != len(native):
            raise ValueError('native source anchors must be unique before repetition')
        self.previous, self.native, self.repeats = previous, native, repeats
        self.run_ids = list(previous.run_ids) + list(native.run_ids) * repeats
        self.anchor_ids = list(previous.anchor_ids) + list(native.anchor_ids) * repeats
        assert_split_membership(split_manifest, self.run_ids, split='train')
        self.audit = dict(previous_presentations=len(previous), native_unique=len(native),
            native_presentations=len(native) * repeats, presentations=len(self.anchor_ids),
            previous_anchor_order_sha256=content_sha256(list(previous.anchor_ids)),
            combined_anchor_order_sha256=content_sha256(self.anchor_ids),
            previous_slots_preserved=True)

    def __len__(self) -> int:
        return len(self.anchor_ids)

    def __getitem__(self, index: int) -> TimeSample:
        if not 0 <= index < len(self):
            raise IndexError(index)
        if index < len(self.previous):
            return self.previous[index]
        return self.native[(index - len(self.previous)) % len(self.native)]
