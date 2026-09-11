"""Support-aware objective and optimizer-window helpers for TimePath.

The denominator is the number of supported anchors, after each anchor's valid-point mean. This
module deliberately does not turn missing future poses into zero targets.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.models.time_path_v1 import STEPS


@dataclass(frozen=True)
class TimeWindowResult:
    supported_anchors: int
    microbatch_count: int
    updated: bool
    loss: float | None


def time_loss_sum_and_count(prediction: torch.Tensor, target: torch.Tensor,
                            mask: torch.Tensor) -> tuple[torch.Tensor, int]:
    """Return summed per-anchor L1 loss and its supported-anchor count."""
    if prediction.ndim != 3 or prediction.shape[1:] != (STEPS, 2):
        raise ValueError("time prediction must be [B,30,2]")
    if target.shape != prediction.shape or mask.shape != prediction.shape[:2] or mask.dtype is not torch.bool:
        raise ValueError("time objective shape/mask mismatch")
    if not torch.isfinite(prediction).all() or not torch.isfinite(target[mask]).all():
        raise FloatingPointError("non-finite supported time objective value")
    supported = mask.any(dim=1)
    count = int(supported.sum().item())
    if count == 0:
        return prediction.sum() * 0.0, 0
    safe_p = torch.where(mask[..., None], prediction, 0.0)
    safe_t = torch.where(mask[..., None], target, 0.0)
    per_horizon = torch.abs(safe_p - safe_t).mean(-1)
    per_anchor = per_horizon.sum(1) / mask.sum(1).clamp_min(1)
    return per_anchor[supported].sum(), count


def accumulate_time_window(
    model: nn.Module,
    microbatches: Sequence[ModelBatchV3],
    optimizer: torch.optim.Optimizer,
    *,
    scheduler: object | None = None,
) -> TimeWindowResult:
    """Train one accumulation window, weighted by total supported anchors.

    Microbatches with zero support are skipped without clearing prior grads.
    The optimizer and optional scheduler advance only when the whole window
    contains at least one supported anchor.  The model is expected to return
    ``[B,30,2]`` and batches must already be on the model's device.
    """
    if not microbatches:
        raise ValueError("time objective window requires at least one microbatch")
    counts: list[int] = []
    for batch in microbatches:
        if batch.targets is None:
            raise ValueError("time objective batch has no targets")
        batch.targets.validate(batch_size=batch.batch_size)
        if batch.targets.trajectory_mask.shape[1:] != (30,):
            raise ValueError("TimePath training requires 30 time points")
        counts.append(int(batch.targets.trajectory_mask.any(dim=1).sum().item()))
    total = sum(counts)
    optimizer.zero_grad(set_to_none=True)
    if total == 0:
        return TimeWindowResult(0, len(microbatches), False, None)
    loss_total = 0.0
    try:
        for batch, count in zip(microbatches, counts):
            if count == 0:
                continue
            prediction = model(batch)
            if not isinstance(prediction, torch.Tensor):
                raise TypeError("TimePath model must return a trajectory tensor")
            summed, actual = time_loss_sum_and_count(
                prediction, batch.targets.trajectory_xy_m, batch.targets.trajectory_mask
            )
            if actual != count:
                raise RuntimeError("support count changed between accumulation and forward")
            (summed / total).backward()
            loss_total += float(summed.detach().cpu())
    except Exception:
        optimizer.zero_grad(set_to_none=True)
        raise
    scheduler_step = None
    if scheduler is not None:
        scheduler_step = getattr(scheduler, "step", None)
        if not callable(scheduler_step):
            optimizer.zero_grad(set_to_none=True)
            raise TypeError("scheduler must provide step()")
    if any(p.grad is not None and not torch.isfinite(p.grad).all() for p in model.parameters()):
        optimizer.zero_grad(set_to_none=True)
        raise FloatingPointError("nonfinite time training gradient")
    optimizer.step()
    if scheduler_step is not None:
        scheduler_step()
    return TimeWindowResult(total, len(microbatches), True, loss_total / total)


def accumulate_prediction_window(
    predictions: Iterable[torch.Tensor], targets: Iterable[torch.Tensor],
    masks: Iterable[torch.Tensor],
) -> tuple[torch.Tensor, int]:
    """Accumulate detached or differentiable predictions without a model call."""
    total_loss: torch.Tensor | None = None
    total = 0
    prediction_list, target_list, mask_list = list(predictions), list(targets), list(masks)
    if not (len(prediction_list) == len(target_list) == len(mask_list)):
        raise ValueError("prediction, target and mask windows must have equal lengths")
    for prediction, target, mask in zip(prediction_list, target_list, mask_list):
        value, count = time_loss_sum_and_count(prediction, target, mask)
        total_loss = value if total_loss is None else total_loss + value
        total += count
    if total_loss is None:
        raise ValueError("prediction window requires at least one microbatch")
    return total_loss, total
