"""Frozen-weight fit diagnostics with explicit train/validation membership."""
from __future__ import annotations

from dataclasses import replace
from typing import Any, Sequence

import torch
from torch.utils.data import DataLoader

from ..data.time_dataset_v1 import TimeSample
from ..data.time_split_v1 import assert_split_membership
from ..training.train_time_v1 import training_batch


def _collate(samples: list[TimeSample]) -> list[TimeSample]:
    return samples


def predict_recovery_fit(
    model: torch.nn.Module, samples: Sequence[TimeSample], *, run_ids: Sequence[str],
    anchor_ids: Sequence[str], split_manifest: dict[str, Any], split: str,
    batch_size: int = 32, workers: int = 0,
) -> torch.Tensor:
    """Return [N,30,2] XY in metres for unique, fully supported recovery anchors.

    Train membership is diagnosed as train, never relabelled as holdout. Missing
    inputs/teachers, order drift and malformed predictions fail explicitly; no
    hard sample disappears from the denominator. No labels reach model.forward.
    Weights are evaluated without gradients, optimizer steps or checkpoint writes.
    """
    if split not in {"train", "validation"}:
        raise ValueError("recovery fit accepts train/validation only; test is sealed")
    if (not len(samples) == len(run_ids) == len(anchor_ids) or not len(samples)
            or len(set(anchor_ids)) != len(anchor_ids)):
        raise ValueError("unique nonempty anchors and matching metadata required")
    if type(batch_size) is not int or batch_size <= 0 or type(workers) is not int or workers < 0:
        raise ValueError("positive batch size and nonnegative workers required")
    assert_split_membership(split_manifest, run_ids, split=split)
    generator = torch.Generator().manual_seed(0)
    loader = DataLoader(samples, batch_size=batch_size, shuffle=False, num_workers=workers,
        collate_fn=_collate, generator=generator,
        **({"prefetch_factor": 1, "multiprocessing_context": "spawn"} if workers else {}))
    device = next(model.parameters()).device
    predictions = torch.empty((len(samples), 30, 2), dtype=torch.float32)
    offset = 0
    model.eval()
    with torch.inference_mode():
        for batch in loader:
            for j, sample in enumerate(batch):
                if (sample.run, sample.anchor_id) != (run_ids[offset+j], anchor_ids[offset+j]):
                    raise ValueError("recovery sample metadata/order mismatch")
                if sample.inputs is None:
                    raise ValueError(f"recovery input missing: {sample.anchor_id}: {sample.input_invalid_reason}")
                if sample.teacher is None or not sample.teacher.xy_mask.all():
                    raise ValueError(f"recovery teacher incomplete: {sample.anchor_id}")
                xy = torch.as_tensor(sample.teacher.xy_m)
                if xy.shape != (30, 2) or not torch.isfinite(xy).all():
                    raise ValueError(f"recovery teacher must be finite [30,2] metres: {sample.anchor_id}")
            inputs = replace(training_batch(batch, device), targets=None)
            output = model(inputs)
            if not isinstance(output, torch.Tensor) or output.shape != (len(batch), 30, 2):
                raise ValueError("recovery prediction must be [B,30,2] metres")
            if not torch.isfinite(output).all():
                raise ValueError("recovery prediction must be finite")
            predictions[offset:offset+len(batch)] = output.float().cpu()
            offset += len(batch)
    if offset != len(samples):
        raise ValueError("recovery dataset length changed")
    return predictions
