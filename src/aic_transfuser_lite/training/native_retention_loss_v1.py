"""Old-train-only output retention; no deployment or default objective changes."""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import torch
from torch.nn import functional as F

from ..data.time_dataset_v1 import TimeSample
from ..data.time_split_v1 import assert_split_membership


def output_retention_sum(prediction: torch.Tensor, reference: torch.Tensor,
                         support: torch.Tensor, *, beta_m: float = 0.02) -> torch.Tensor:
    """Sum per-anchor Smooth L1 on [B,30,2] metre XY; support [B,30] bool.

    Coordinates and valid horizons are averaged within each supported anchor.
    The caller divides the SUM by the whole mixed batch's supported count.
    Reference is detached. Missing horizons contribute neither value nor gradient.
    """
    if (prediction.ndim != 3 or prediction.shape[1:] != (30, 2)
            or reference.shape != prediction.shape or support.shape != prediction.shape[:2]
            or support.dtype != torch.bool or not math.isfinite(beta_m) or beta_m <= 0):
        raise ValueError("finite positive beta_m and [B,30,2] XY/[B,30] bool support required")
    if not torch.isfinite(prediction).all() or not torch.isfinite(reference[support]).all():
        raise ValueError("finite predictions and supported reference required")
    p = torch.where(support[..., None], prediction, 0.0)
    r = torch.where(support[..., None], reference.detach(), 0.0)
    point_loss = F.smooth_l1_loss(p, r, beta=beta_m, reduction="none").mean(-1)
    per_anchor = point_loss.sum(-1) / support.sum(-1).clamp_min(1)
    return per_anchor[support.any(-1)].sum()


class RetainedNativeObjective:
    """Keep old recovery unchanged; add opt-in native geometry and old output loss."""

    def __init__(self, recovery: Any, native: Any, *, geometry_weight: float,
                 retention_weight: float, references: Mapping[str, torch.Tensor],
                 old_anchor_runs: Mapping[str, str], split_manifest: dict[str, Any]) -> None:
        if any(not math.isfinite(w) or w < 0 for w in (geometry_weight, retention_weight)):
            raise ValueError("nonnegative finite objective weights required")
        assert_split_membership(split_manifest, list(old_anchor_runs.values()), split="train")
        if not references.keys() <= old_anchor_runs.keys():
            raise ValueError("references must belong to old train anchors only")
        for value in references.values():
            if value.shape != (30, 2) or not torch.isfinite(value).all():
                raise ValueError("reference must be finite [30,2] metres")
        self.recovery, self.native = recovery, native
        self.geometry_weight, self.retention_weight = geometry_weight, retention_weight
        self.references, self.old_anchor_runs = references, old_anchor_runs
        self.last_terms: dict[str, float] = {}

    def validate_samples(self, samples: Sequence[TimeSample]) -> None:
        self.recovery.validate_samples(samples)
        if self.geometry_weight:
            self.native.validate_samples(samples)
        for s in samples:
            if s.anchor_id in self.old_anchor_runs:
                if s.run != self.old_anchor_runs[s.anchor_id]:
                    raise ValueError("old anchor/run identity mismatch")
                if (self.retention_weight and s.inputs is not None and s.teacher is not None
                        and s.teacher.xy_mask.any() and s.anchor_id not in self.references):
                    raise ValueError("missing old train reference")

    def __call__(self, prediction: torch.Tensor, target: torch.Tensor,
                 support: torch.Tensor, samples: Sequence[TimeSample]) -> torch.Tensor:
        self.validate_samples(samples)
        total = self.recovery(prediction, target, support, samples)
        geometry = prediction.sum() * 0.0
        retention = prediction.sum() * 0.0
        if self.geometry_weight:
            geometry = self.native(prediction, target, support, samples) * self.geometry_weight
        if self.retention_weight:
            chosen = [i for i, s in enumerate(samples)
                      if s.anchor_id in self.old_anchor_runs and support[i].any()]
            if chosen:
                ref = torch.stack([self.references[samples[i].anchor_id] for i in chosen]).to(prediction)
                retention = output_retention_sum(prediction[chosen], ref, support[chosen]) * self.retention_weight
        self.last_terms = {"old_retention_sum_m": float(retention.detach()),
                           "weighted_native_geometry_sum": float(geometry.detach())}
        return total + geometry + retention
