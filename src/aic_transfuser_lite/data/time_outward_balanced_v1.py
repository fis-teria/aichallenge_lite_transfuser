"""Train-only state balancing within an unchanged recovery presentation budget."""
from __future__ import annotations

from collections import Counter
from typing import Sequence

import numpy as np
from torch.utils.data import Dataset

from .time_dataset_v1 import TimeSample
from .time_recovery_training_v1 import MatchedRecoveryMixDataset
from .time_split_v1 import assert_split_membership


class OutwardBalancedMixDataset(Dataset[TimeSample]):
    """Preserve nominal slots and all unique inputs/teachers, [30,2] metres.

    A fixed fraction of recovery slots presents audited outward-state anchors.
    Those slots are run-balanced; remaining recovery anchors are sample-balanced.
    Repetitions do not create independent observations or alter sensor tensors.
    """

    def __init__(self, reference: MatchedRecoveryMixDataset, recovery_run_ids: Sequence[str],
                 target_anchor_ids: Sequence[str], *, target_fraction: float, seed: int) -> None:
        if (type(seed) is not int or seed < 0 or not np.isfinite(target_fraction)
                or not 0. < target_fraction < 1.):
            raise ValueError("positive fraction below one and nonnegative integer seed required")
        full = reference.dataset
        assert_split_membership(full.split_manifest, full.run_ids, split="train")
        ids, targets = set(recovery_run_ids), set(target_anchor_ids)
        if not ids or not ids <= set(full.run_ids) or not targets or len(targets) != len(target_anchor_ids):
            raise ValueError("unique train-only recovery targets required")
        if len(set(full.anchor_ids)) != len(full.anchor_ids):
            raise ValueError("underlying anchors must be unique")
        pool = [i for i, rid in enumerate(full.run_ids) if rid in ids]
        available = {full.anchor_ids[i] for i in pool}
        if not targets <= available:
            raise ValueError("target anchor absent from recovery training pool")
        slots = [j for j, i in enumerate(reference.indices) if full.run_ids[i] in ids]
        quota = int(round(len(slots) * target_fraction))
        grouped: dict[str, list[int]] = {}
        other = []
        for i in pool:
            if full.anchor_ids[i] in targets:
                grouped.setdefault(full.run_ids[i], []).append(i)
            else:
                other.append(i)
        rng = np.random.default_rng(seed)

        def repeat(group: list[int], count: int) -> list[int]:
            if not group or count < len(group):
                raise ValueError("presentation budget must cover every unique recovery anchor")
            q, r = divmod(count, len(group))
            return np.tile(group, q).tolist() + rng.permutation(group)[:r].tolist()

        q, remainder = divmod(quota, len(grouped))
        extra = set(rng.permutation(sorted(grouped))[:remainder].tolist())
        chosen = [i for rid in sorted(grouped) for i in repeat(grouped[rid], q + int(rid in extra))]
        chosen += repeat(other, len(slots) - quota)
        rng.shuffle(chosen)
        self.dataset = full
        self.indices = list(reference.indices)
        for slot, index in zip(slots, chosen, strict=True):
            self.indices[slot] = index
        self.run_ids = [full.run_ids[i] for i in self.indices]
        self.anchor_ids = [full.anchor_ids[i] for i in self.indices]
        self.recovery_presentations = len(slots)
        self.unique_recovery_anchors = len(pool)
        self.audit = {
            "unique_recovery_anchors": len(pool), "recovery_presentations": len(slots),
            "target_unique_anchors": len(targets), "target_presentations": quota,
            "target_fraction_within_recovery": target_fraction,
            "target_run_presentations": dict(Counter(self.run_ids[j] for j in slots if self.anchor_ids[j] in targets)),
            "per_anchor_presentations": dict(Counter(self.anchor_ids[j] for j in slots)),
            "nominal_slots_preserved": all(self.indices[j] == reference.indices[j]
                for j in range(len(reference)) if full.run_ids[reference.indices[j]] not in ids),
        }
        if not self.audit["nominal_slots_preserved"] or set(self.indices) != set(reference.indices):
            raise ValueError("presentation change dropped data or changed nominal slots")

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, index: int) -> TimeSample:
        return self.dataset[self.indices[index]]
