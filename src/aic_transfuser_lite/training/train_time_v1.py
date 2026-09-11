"""Bounded TimePath training/evaluation integration, separate from the V3 trainer.

Only optimizer-window boundaries can be resumed. The input sequence and its
order are frozen by the teacher manifest; no test runs are used for fitting.
This library has no automatic real-data training or ROS execution entry point.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, fields, replace
from typing import Any, Sequence

import numpy as np
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_split_v1 import assert_split_membership
from aic_transfuser_lite.evaluation.time_metrics_v1 import (
    time_horizon_metrics, zero_velocity_baseline, constant_velocity_baseline,
)
from .time_objective_v1 import accumulate_time_window


@dataclass(frozen=True)
class TimeTrainingBudget:
    max_anchors: int
    max_optimizer_steps: int
    microbatch_size: int = 2
    accumulation_steps: int = 1

    def __post_init__(self) -> None:
        if any(type(v) is not int or v <= 0 for v in vars(self).values()):
            raise ValueError("training requires positive integer finite budgets")


@dataclass(frozen=True)
class TimeTrainingProgress:
    next_anchor_index: int = 0
    anchors_visited: int = 0
    optimizer_steps: int = 0

    def __post_init__(self) -> None:
        if any(type(v) is not int or v < 0 for v in vars(self).values()):
            raise ValueError("training progress must contain nonnegative integers")


def _to_device(batch: ModelBatchV3, device: torch.device) -> ModelBatchV3:
    return replace(batch, **{f.name: getattr(batch, f.name).to(device)
                            for f in fields(batch) if isinstance(getattr(batch, f.name), torch.Tensor)})


def training_batch(samples: Sequence[TimeSample], device: torch.device) -> ModelBatchV3:
    """Collate already-eligible samples; labels remain outside the model inputs.

    Targets: XY [B,30,2] m, longitudinal velocity [B,30] m/s, independent bool
    masks. Interval masks and stop evidence stay in sample metadata for auditing.
    """
    if not samples or any(s.inputs is None for s in samples):
        raise ValueError("training_batch requires nonempty samples with valid inputs")
    kwargs = {f.name: torch.cat([getattr(s.inputs, f.name) for s in samples], dim=0).to(device)
              for f in fields(ModelBatchV3) if isinstance(getattr(samples[0].inputs, f.name), torch.Tensor)}
    target, mask, velocity, vmask = [], [], [], []
    for s in samples:
        t = s.teacher
        target.append(torch.from_numpy(t.xy_m.copy()) if t is not None else torch.full((30,2), float("nan")))
        mask.append(torch.from_numpy(t.xy_mask.copy()) if t is not None else torch.zeros(30, dtype=torch.bool))
        velocity.append(torch.from_numpy(t.velocity_mps.copy()) if t is not None else torch.full((30,), float("nan")))
        vmask.append(torch.from_numpy(t.velocity_mask.copy()) if t is not None else torch.zeros(30, dtype=torch.bool))
    labels = TrainingTargetsV3(*(torch.stack(v).to(device) for v in (target, mask, velocity, vmask)))
    batch = ModelBatchV3(**kwargs, targets=labels, requested_outputs=frozenset({"trajectory"}))
    batch.validate(require_current=False)
    return batch


def train_time_epoch(model: torch.nn.Module, samples: Sequence[TimeSample], optimizer: torch.optim.Optimizer,
                     *, split_manifest: dict[str, Any], budget: TimeTrainingBudget,
                     progress: TimeTrainingProgress = TimeTrainingProgress(),
                     scheduler: Any = None) -> tuple[TimeTrainingProgress, dict[str, Any]]:
    """One ordered, finite pass (or continuation) through train-only samples.

    A missing input counts against the presentation budget and stays in reports.
    All-invalid windows consume anchors but never optimizer/scheduler updates.
    Epoch transitions/resetting a cursor must be explicit in the outer experiment.
    """
    assert_split_membership(split_manifest, (s.run for s in samples), split="train")
    if progress.next_anchor_index > len(samples):
        raise ValueError("resume cursor exceeds frozen teacher sequence")
    model.train()
    device = next(model.parameters()).device
    index, visited, steps = progress.next_anchor_index, progress.anchors_visited, progress.optimizer_steps
    counts: Counter[str] = Counter()
    weighted_loss = 0.0
    support = 0
    width = budget.microbatch_size * budget.accumulation_steps
    while index < len(samples) and visited < budget.max_anchors and steps < budget.max_optimizer_steps:
        end = min(len(samples), index+width, index+budget.max_anchors-visited)
        window = samples[index:end]
        eligible = []
        for s in window:
            counts["anchors_visited"] += 1
            if s.inputs is None:
                counts["input_invalid"] += 1
                counts[f"input_reason:{s.input_invalid_reason}"] += 1
            else:
                eligible.append(s)
                if s.teacher is None or not s.teacher.xy_mask.any():
                    counts["teacher_unsupported"] += 1
        if eligible:
            batches = [training_batch(eligible[i:i+budget.microbatch_size], device)
                       for i in range(0, len(eligible), budget.microbatch_size)]
            result = accumulate_time_window(model, batches, optimizer, scheduler=scheduler)
            if result.updated:
                steps += 1
                support += result.supported_anchors
                weighted_loss += result.loss * result.supported_anchors
        visited += end-index
        index = end
    return TimeTrainingProgress(index, visited, steps), {
        "counts": dict(counts), "supported_anchors": support,
        "loss_m": weighted_loss/support if support else None,
        "weighting": "mean_valid_L1_xy_per_anchor_then_mean_supported_anchors",
        "order": "frozen_teacher_manifest_order", "runtime_ready": False,
    }


def evaluate_time_samples(model: torch.nn.Module, samples: Sequence[TimeSample], *,
                          split_manifest: dict[str, Any], split: str = "validation",
                          final_test: bool = False) -> dict[str, Any]:
    """All-anchor offline evaluation. Explicit final_test unlocks sealed test runs.

    Accepted means finite output from a valid input here. Controller/Supervisor
    acceptance, stale rate and physical progress require the later runtime trial.
    """
    if type(final_test) is not bool or (split == "test" and not final_test):
        raise ValueError("test is sealed; final_test=True is required for final evaluation")
    assert_split_membership(split_manifest, (s.run for s in samples), split=split)
    n = len(samples)
    predictions = torch.full((n,30,2), float("nan"))
    target = torch.full_like(predictions, float("nan"))
    mask = torch.zeros((n,30), dtype=torch.bool)
    iv = torch.zeros(n, dtype=torch.bool)
    cv = torch.full_like(predictions, float("nan"))
    invalid_reasons: Counter[str] = Counter()
    teacher_reasons: Counter[str] = Counter()
    stop_reasons: Counter[str] = Counter()
    model.eval()
    device = next(model.parameters()).device
    with torch.no_grad():
        for i,s in enumerate(samples):
            teacher_reasons.update(s.teacher_reasons)
            stop_reasons[s.stop_reason] += 1
            if s.teacher is not None:
                target[i] = torch.from_numpy(s.teacher.xy_m.copy())
                mask[i] = torch.from_numpy(s.teacher.xy_mask.copy())
            if s.inputs is None:
                invalid_reasons[f"input:{s.input_invalid_reason}"] += 1
                continue
            iv[i] = True
            if s.inputs.ego_feature_mask[0,-1,:2].all():
                cv[i] = constant_velocity_baseline(s.inputs.ego[:,-1,:2].cpu())[0]
            try:
                output = model(_to_device(s.inputs,device))
                if not isinstance(output, torch.Tensor) or output.shape != (1,30,2):
                    raise ValueError("invalid time output shape")
                predictions[i] = output[0].cpu()
            except ValueError as exc:
                invalid_reasons[f"output:{exc}"] += 1
    runs = [s.run for s in samples]
    kwargs = {"input_valid": iv, "run_ids": runs}
    result = time_horizon_metrics(predictions, target, mask, **kwargs)
    result["baselines"] = {
        "zero_motion": time_horizon_metrics(zero_velocity_baseline(n),target,mask,**kwargs),
        "constant_observed_velocity": time_horizon_metrics(cv,target,mask,**kwargs),
    }
    result.update({"invalid_reasons":dict(invalid_reasons), "teacher_reasons":dict(teacher_reasons),
                   "stop_reasons":dict(stop_reasons), "stop_intent_status":"NOT_EVALUATED",
                   "stop_intent_annotation_count":sum(s.environment_stop_intent is not None for s in samples),
                   "runtime_stale_rate":None, "physical_progress_m":None,
                   "scope":"offline_fixed_condition_holdout", "runtime_ready":False})
    return result
