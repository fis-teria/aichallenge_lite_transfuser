"""Train-only observed teacher station matching; no synthetic avoidance targets."""
from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
import torch
from torch.nn import functional as F

from ..data.time_dataset_v1 import TimeSample
from ..data.time_split_v1 import assert_split_membership


def teacher_station_brackets(xy: np.ndarray) -> tuple[list[int], list[float]] | None:
    """Bracket first forward x=1/2/3 m crossings in observed [30,2] metre XY.

    No extrapolation or artificial origin. Require a strictly forward prefix
    through 3 m. Indices/weights select observation-relative teacher times;
    the prediction is interpolated at these SAME times, not its own x crossing.
    """
    if xy.shape != (30, 2) or not np.isfinite(xy).all():
        raise ValueError('finite [30,2] metre teacher required')
    end = np.flatnonzero(xy[:, 0] >= 3.)
    if not len(end):
        return None
    prefix = xy[:int(end[0]) + 1, 0]
    if len(prefix) < 2 or prefix[0] > 1. or np.any(np.diff(prefix) <= 0):
        return None
    lower, alpha = [], []
    for station in (1., 2., 3.):
        upper = max(1, int(np.searchsorted(prefix, station, side='left')))
        lo = upper - 1
        lower.append(lo)
        alpha.append(float((station - prefix[lo]) / (prefix[upper] - prefix[lo])))
    return lower, alpha


def station_values(xy: torch.Tensor, lower: torch.Tensor, alpha: torch.Tensor) -> torch.Tensor:
    """Interpolate [B,30,2] m at three fixed label times; output [B,3,2] m."""
    if (xy.ndim != 3 or xy.shape[1:] != (30, 2) or lower.shape != (len(xy), 3)
            or alpha.shape != lower.shape or lower.dtype != torch.long
            or (lower < 0).any() or (lower >= 29).any()
            or not torch.isfinite(alpha).all() or (alpha < 0).any() or (alpha > 1).any()
            or not torch.isfinite(xy).all()):
        raise ValueError('finite [B,30,2] XY, [B,3] integer brackets and [0,1] weights required')
    indices = lower[..., None].expand(-1, -1, 2)
    return torch.lerp(xy.gather(1, indices), xy.gather(1, indices + 1), alpha[..., None])


class NativeNearFieldObjective:
    """Add mean XY Smooth L1 at observed 1/2/3 m teacher times on native train.

    Input prediction/target [B,30,2] m and support [B,30] bool. Per-anchor means
    are summed; the existing runner divides by ALL mixed supported anchors.
    Old output retention/recovery are delegated unchanged to base.
    """
    def __init__(self, base: Any, rows: Sequence[dict[str, Any]], *, weight: float,
                 split_manifest: dict[str, Any]) -> None:
        if not math.isfinite(weight) or weight < 0 or not rows:
            raise ValueError('nonnegative finite weight and nonempty targets required')
        if len({r['anchor_id'] for r in rows}) != len(rows):
            raise ValueError('unique target anchors required')
        assert_split_membership(split_manifest, [r['run_id'] for r in rows], split='train')
        for row in rows:
            xy = np.asarray(row['teacher_xy_m'], dtype=float)
            brackets = teacher_station_brackets(xy)
            if brackets is None or brackets != (row['lower'], row['alpha']):
                raise ValueError('targets must match observed teacher brackets')
        self.base, self.weight = base, weight
        self.rows = {r['anchor_id']: dict(r) for r in rows}
        self.last_terms: dict[str, float] = {}

    def validate_samples(self, samples: Sequence[TimeSample]) -> None:
        self.base.validate_samples(samples)
        for sample in samples:
            row = self.rows.get(sample.anchor_id)
            if row is not None and (sample.run != row['run_id'] or sample.teacher is None
                    or not sample.teacher.xy_mask.all()
                    or not np.allclose(sample.teacher.xy_m, row['teacher_xy_m'], atol=1e-6, rtol=0)):
                raise ValueError('native near-field teacher/run changed')

    def __call__(self, prediction: torch.Tensor, target: torch.Tensor,
                 support: torch.Tensor, samples: Sequence[TimeSample]) -> torch.Tensor:
        if (prediction.shape != (len(samples), 30, 2) or target.shape != prediction.shape
                or support.shape != prediction.shape[:2] or support.dtype != torch.bool):
            raise ValueError('expected [B,30,2] XY and [B,30] bool support')
        self.validate_samples(samples)
        total = self.base(prediction, target, support, samples)
        term = prediction.sum() * 0.
        chosen = [i for i, s in enumerate(samples) if s.anchor_id in self.rows]
        if self.weight and chosen:
            if not support[chosen].all():
                raise ValueError('complete selected native support required')
            expected = prediction.new_tensor([self.rows[samples[i].anchor_id]['teacher_xy_m'] for i in chosen])
            if not torch.allclose(target[chosen], expected, atol=1e-6, rtol=0):
                raise ValueError('batch targets differ from observed native teacher')
            lower = torch.tensor([self.rows[samples[i].anchor_id]['lower'] for i in chosen],
                                 device=prediction.device, dtype=torch.long)
            alpha = prediction.new_tensor([self.rows[samples[i].anchor_id]['alpha'] for i in chosen])
            p = station_values(prediction[chosen].float(), lower, alpha)
            t = station_values(target[chosen].detach().float(), lower, alpha)
            term = F.smooth_l1_loss(p, t, beta=.02, reduction='none').mean((1,2)).sum() * self.weight
        self.last_terms = dict(self.base.last_terms, weighted_native_near_field_sum_m=float(term.detach()))
        return total + term
