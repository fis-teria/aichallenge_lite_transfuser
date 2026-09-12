"""Bounded, validation-only batched evaluation for the TimePath baseline."""
from __future__ import annotations

from collections import Counter
from contextlib import nullcontext
from typing import Any, Sequence

import torch
from torch.utils.data import DataLoader

from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_split_v1 import assert_split_membership
from aic_transfuser_lite.evaluation.time_metrics_v1 import (
    constant_velocity_baseline, time_horizon_metrics, zero_velocity_baseline,
)
from aic_transfuser_lite.training.train_time_v1 import _to_device, training_batch


def _identity_collate(batch: list[TimeSample]) -> list[TimeSample]:
    return batch


def evaluate_time_batched(
    model: torch.nn.Module,
    samples: Sequence[TimeSample],
    *,
    run_ids: Sequence[str],
    split_manifest: dict[str, Any],
    batch_size: int = 32,
    workers: int = 0,
    precision: str = "float32",
) -> tuple[dict[str, Any], torch.Tensor]:
    """Evaluate a finite, caller-selected validation sequence in mini-batches.

    ``run_ids`` is the cheap membership list supplied by the caller; this function
    does not inspect or enumerate corpus images.  Every sample remains an anchor,
    including invalid inputs and unsupported teachers.
    """
    if len(run_ids) != len(samples):
        raise ValueError("run_ids length mismatch")
    if type(batch_size) is not int or batch_size <= 0:
        raise ValueError("batch_size must be a positive integer")
    if type(workers) is not int or workers < 0:
        raise ValueError("workers must be a nonnegative integer")
    if precision not in {"float32", "bf16"}:
        raise ValueError("precision must be float32 or bf16")
    if precision == "bf16" and not torch.cuda.is_available():
        raise ValueError("bf16 evaluation requires CUDA")
    assert_split_membership(split_manifest, run_ids, split="validation")

    n = len(samples)
    predictions = torch.full((n, 30, 2), float("nan"))
    target = torch.full_like(predictions, float("nan"))
    mask = torch.zeros((n, 30), dtype=torch.bool)
    input_valid = torch.zeros(n, dtype=torch.bool)
    constant = torch.full_like(predictions, float("nan"))
    invalid_reasons: Counter[str] = Counter()
    teacher_reasons: Counter[str] = Counter()
    stop_reasons: Counter[str] = Counter()
    annotation_count = 0
    try:
        device = next(model.parameters()).device
    except StopIteration:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    generator = torch.Generator()
    generator.manual_seed(0)
    loader = DataLoader(samples, batch_size=batch_size, shuffle=False,
                        num_workers=workers, collate_fn=_identity_collate,
                        generator=generator, **({"prefetch_factor": 1, "multiprocessing_context": "spawn"} if workers else {}))
    offset = 0
    with torch.no_grad():
        for batch in loader:
            base = offset
            offset += len(batch)
            eligible = []
            for j, sample in enumerate(batch):
                index = base + j
                if sample.run != run_ids[index]:
                    raise ValueError("run_ids do not match sample metadata")
                teacher_reasons.update(sample.teacher_reasons)
                stop_reasons[sample.stop_reason] += 1
                if sample.environment_stop_intent is not None:
                    annotation_count += 1
                if sample.teacher is not None:
                    target[index] = torch.from_numpy(sample.teacher.xy_m.copy())
                    mask[index] = torch.from_numpy(sample.teacher.xy_mask.copy())
                if sample.inputs is None:
                    invalid_reasons[f"input:{sample.input_invalid_reason}"] += 1
                    continue
                input_valid[index] = True
                if sample.inputs.ego_feature_mask[0, -1, :2].all():
                    constant[index] = constant_velocity_baseline(sample.inputs.ego[:, -1, :2].cpu())[0]
                eligible.append((index, sample))
            if not eligible:
                continue
            try:
                xb = training_batch([s for _, s in eligible], device)
                autocast = (torch.autocast(device_type="cuda", dtype=torch.bfloat16)
                            if precision == "bf16" else nullcontext())
                with autocast:
                    output = model(_to_device(xb, device))
                if not isinstance(output, torch.Tensor) or output.shape != (len(eligible), 30, 2):
                    raise ValueError("invalid time output shape")
                predictions[[i for i, _ in eligible]] = output.float().cpu()
            except ValueError as exc:
                # A malformed peer must not discard valid peers in the same batch.
                for index, sample in eligible:
                    try:
                        one = training_batch([sample], device)
                        autocast = (torch.autocast(device_type="cuda", dtype=torch.bfloat16)
                                    if precision == "bf16" else nullcontext())
                        with autocast:
                            output = model(_to_device(one, device))
                        if not isinstance(output, torch.Tensor) or output.shape != (1, 30, 2):
                            raise ValueError("invalid time output shape")
                        predictions[index] = output[0].float().cpu()
                    except ValueError as item_exc:
                        invalid_reasons[f"output:{item_exc}"] += 1

    kwargs = {"input_valid": input_valid, "run_ids": list(run_ids)}
    result = time_horizon_metrics(predictions, target, mask, **kwargs)
    result["baselines"] = {
        "zero_motion": time_horizon_metrics(zero_velocity_baseline(n), target, mask, **kwargs),
        "constant_observed_velocity": time_horizon_metrics(constant, target, mask, **kwargs),
    }
    result.update({"invalid_reasons": dict(invalid_reasons),
                   "teacher_reasons": dict(teacher_reasons),
                   "stop_reasons": dict(stop_reasons),
                   "stop_intent_status": "NOT_EVALUATED",
                   "stop_intent_annotation_count": annotation_count,
                   "runtime_stale_rate": None, "physical_progress_m": None,
                   "scope": "offline_fixed_condition_holdout", "runtime_ready": False})
    return result, predictions
