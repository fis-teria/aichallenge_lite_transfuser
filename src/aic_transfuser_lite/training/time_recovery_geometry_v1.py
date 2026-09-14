"""Train-only near PP geometry and far lateral supervision, never model inputs."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Sequence

import numpy as np
import torch

from ..data.time_dataset_v1 import TimeSample
from ..data.time_split_v1 import assert_split_membership, content_sha256


@dataclass(frozen=True)
class RecoveryGeometryPlan:
    """Weights convert radians to metres; far lateral error is already metres."""
    near_weight_m_per_rad: float = 1.0
    far_weight: float = 0.5
    far_start_s: float = 2.0
    rear_axle_offset_m: tuple[float, float] = (0.0010000169277191162, 0.0)

    def validate(self) -> None:
        if (not all(math.isfinite(v) and v >= 0 for v in
                    (self.near_weight_m_per_rad, self.far_weight))
                or self.near_weight_m_per_rad + self.far_weight <= 0):
            raise ValueError("nonnegative finite auxiliary weights, at least one positive, required")
        if self.far_start_s != 2.0:
            raise ValueError("far interval is fixed at 2.0 through 3.0 s inclusive")
        if (len(self.rear_axle_offset_m) != 2
                or not all(math.isfinite(v) for v in self.rear_axle_offset_m)
                or math.hypot(*self.rear_axle_offset_m) > .002):
            raise ValueError("explicit rear offset within the runtime 2 mm contract required")


def fixed_time_pp_angle(xy_m: torch.Tensor, horizon_s: torch.Tensor,
                        response_length_m: torch.Tensor,
                        rear_axle_offset_m: tuple[float, float]) -> torch.Tensor:
    """[B,30,2] metres -> [B] physical tire radians at teacher-fixed [B] times.

    Linear temporal interpolation matches the age-zero runtime polyline target.
    This is a differentiable surrogate, not runtime target selection, feasibility
    rejection, steering saturation, latency compensation, or obstacle supervision.
    """
    if xy_m.ndim != 3 or xy_m.shape[1:] != (30, 2):
        raise ValueError("trajectory must have shape [B,30,2] metres")
    n = len(xy_m)
    if horizon_s.shape != (n,) or response_length_m.shape != (n,):
        raise ValueError("one horizon and response length per trajectory required")
    if (not torch.isfinite(xy_m).all() or not torch.isfinite(horizon_s).all()
            or not torch.isfinite(response_length_m).all()
            or not ((horizon_s >= .1) & (horizon_s <= 3.)).all()
            or not (response_length_m > 0).all()):
        raise ValueError("finite trajectories, 0.1..3 s horizons, positive metre response lengths required")
    if (len(rear_axle_offset_m) != 2 or not all(math.isfinite(v) for v in rear_axle_offset_m)
            or math.hypot(*rear_axle_offset_m) > .002):
        raise ValueError("runtime rear axle offset contract required")
    position = horizon_s * 10. - 1.
    lower = position.floor().long().clamp(0, 28)
    fraction = position - lower
    rows = torch.arange(n, device=xy_m.device)
    target = (xy_m[rows, lower] * (1. - fraction[:, None])
              + xy_m[rows, lower + 1] * fraction[:, None])
    target = target - xy_m.new_tensor(rear_axle_offset_m)
    return torch.atan(2. * response_length_m * target[:, 1]
                      / target.square().sum(dim=1).clamp_min(1e-6))


class RecoveryGeometryObjective:
    """Sum auxiliary loss over recovery anchors; caller divides by ALL support.

    Each target row pins anchor_id, train run_id, teacher-selected horizon_s and
    the speed-dependent response_length_m. These are labels used after forward.
    Nominal samples retain the original objective, including partial support.
    Audited recovery samples must have complete teachers; missing support fails.
    """

    def __init__(self, rows: Sequence[dict[str, Any]], *, split_manifest: dict[str, Any],
                 plan: RecoveryGeometryPlan, controller: dict[str, Any]) -> None:
        plan.validate()
        if not rows or len({r['anchor_id'] for r in rows}) != len(rows):
            raise ValueError("nonempty unique recovery anchor targets required")
        assert_split_membership(split_manifest, [r['run_id'] for r in rows], split='train')
        if any(not r['run_id'].startswith('codex-time-recovery-') for r in rows):
            raise ValueError("auxiliary objective is recovery-only")
        for row in rows:
            if (not math.isfinite(row['horizon_s']) or not .1 <= row['horizon_s'] <= 3.
                    or not math.isfinite(row['response_length_m']) or row['response_length_m'] <= 0):
                raise ValueError("invalid recovery teacher geometry")
        self.rows = {r['anchor_id']: dict(r) for r in rows}
        self.plan = plan
        self.identity = dict(format='teacher_fixed_time_pp_and_far_y_v1',
            plan={**asdict(plan), 'rear_axle_offset_m': list(plan.rear_axle_offset_m)},
            controller=controller, controller_age_s=0.,
            target_count=len(rows), targets_sha256=content_sha256(list(rows)),
            split_manifest_sha256=split_manifest['manifest_sha256'],
            normalization='sum_auxiliary_over_recovery_divide_all_supported_batch_anchors',
            runtime_target_selection_in_loss=False)

    def validate_samples(self, samples: Sequence[TimeSample]) -> None:
        """Check before the runner's empty-input/zero-support early returns."""
        for sample in samples:
            row = self.rows.get(sample.anchor_id)
            if sample.run.startswith('codex-time-recovery-') or row is not None:
                if row is None or row['run_id'] != sample.run:
                    raise ValueError("recovery anchor missing or mismatched in frozen auxiliary targets")
                if (sample.inputs is None or sample.teacher is None
                        or not sample.teacher.xy_mask.all() or not np.isfinite(sample.teacher.xy_m).all()):
                    raise ValueError("recovery auxiliary requires complete input and teacher support")

    def __call__(self, prediction: torch.Tensor, target: torch.Tensor,
                 support: torch.Tensor, samples: Sequence[TimeSample]) -> torch.Tensor:
        if (prediction.shape != (len(samples), 30, 2) or target.shape != prediction.shape
                or support.shape != (len(samples), 30) or support.dtype != torch.bool):
            raise ValueError("expected aligned [B,30,2] predictions/teachers and [B,30] bool support")
        chosen = []
        rows = []
        for i, sample in enumerate(samples):
            row = self.rows.get(sample.anchor_id)
            if sample.run.startswith('codex-time-recovery-') and row is None:
                raise ValueError("recovery anchor missing from frozen auxiliary targets")
            if row is not None:
                if row['run_id'] != sample.run or sample.inputs is None:
                    raise ValueError("auxiliary anchor run/input mismatch")
                chosen.append(i)
                rows.append(row)
        if not chosen:
            return prediction.sum() * 0.
        if not support[chosen].all():
            raise ValueError("recovery auxiliary requires the complete audited 30-point teacher")
        p, t = prediction[chosen].float(), target[chosen].detach().float()
        if not torch.isfinite(t).all():
            raise ValueError("nonfinite recovery teacher")
        h = p.new_tensor([r['horizon_s'] for r in rows])
        length = p.new_tensor([r['response_length_m'] for r in rows])
        pred_angle = fixed_time_pp_angle(p, h, length, self.plan.rear_axle_offset_m)
        teacher_angle = fixed_time_pp_angle(t, h, length, self.plan.rear_axle_offset_m)
        near = (pred_angle - teacher_angle).abs()
        far = (p[:, 19:, 1] - t[:, 19:, 1]).abs().mean(dim=1)
        return (self.plan.near_weight_m_per_rad * near + self.plan.far_weight * far).sum()
