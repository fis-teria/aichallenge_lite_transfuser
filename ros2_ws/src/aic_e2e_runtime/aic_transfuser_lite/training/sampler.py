from __future__ import annotations

from dataclasses import dataclass
import hashlib
from typing import Any

import numpy as np
import pandas as pd
import torch
from torch.utils.data import WeightedRandomSampler

from .metrics import CURVATURE_BUCKET_NAMES, trajectory_max_abs_curvature


@dataclass(frozen=True)
class SamplingPlan:
    weights: torch.Tensor
    summary: dict[str, Any]


def build_capped_curvature_recovery_plan(
    frame: pd.DataFrame,
    *,
    num_waypoints: int,
    straight_threshold_per_m: float,
    sharp_threshold_per_m: float,
    max_weight: float,
    recovery_weight: float,
) -> SamplingPlan:
    """Balance curvature buckets without exceeding the configured weight cap."""
    if len(frame) == 0:
        raise ValueError("Cannot build a sampling plan for an empty dataset")
    if not 0.0 < straight_threshold_per_m < sharp_threshold_per_m:
        raise ValueError("Curvature thresholds must satisfy 0 < straight < sharp")
    if max_weight < 1.0:
        raise ValueError("max_weight must be >= 1")
    if not 1.0 <= recovery_weight <= max_weight:
        raise ValueError("recovery_weight must be within [1, max_weight]")
    required = {
        column
        for index in range(num_waypoints)
        for column in (f"wp_{index}_x", f"wp_{index}_y")
    } | {"recovery_flag"}
    missing = sorted(required.difference(frame.columns))
    if missing:
        raise ValueError(f"Sampling frame is missing columns: {missing}")

    waypoint_values = np.stack(
        [
            frame[[f"wp_{index}_x", f"wp_{index}_y"]].to_numpy(
                dtype=np.float32
            )
            for index in range(num_waypoints)
        ],
        axis=1,
    )
    curvature = trajectory_max_abs_curvature(torch.from_numpy(waypoint_values))
    curvature_values = curvature.detach().cpu().numpy()
    bucket_indices = np.where(
        curvature_values < straight_threshold_per_m,
        0,
        np.where(curvature_values < sharp_threshold_per_m, 1, 2),
    ).astype(np.int64)
    bucket_counts = {
        name: int(np.count_nonzero(bucket_indices == index))
        for index, name in enumerate(CURVATURE_BUCKET_NAMES)
    }
    nonzero_counts = [count for count in bucket_counts.values() if count > 0]
    if not nonzero_counts:
        raise ValueError("No curvature bucket contains a sample")
    majority_count = max(nonzero_counts)
    bucket_weights = {
        name: (
            min(max_weight, majority_count / count) if count > 0 else None
        )
        for name, count in bucket_counts.items()
    }
    weights = np.asarray(
        [
            float(bucket_weights[CURVATURE_BUCKET_NAMES[index]])
            for index in bucket_indices
        ],
        dtype=np.float64,
    )
    recovery = frame["recovery_flag"].astype(float).to_numpy() > 0.5
    weights[recovery] = np.maximum(weights[recovery], recovery_weight)
    weights = np.clip(weights, 1.0, max_weight)
    tensor = torch.from_numpy(weights).to(dtype=torch.double)
    summary = {
        "type": "capped_inverse_frequency_curvature_recovery",
        "sample_count": len(frame),
        "straight_threshold_per_m": float(straight_threshold_per_m),
        "sharp_threshold_per_m": float(sharp_threshold_per_m),
        "max_weight": float(max_weight),
        "recovery_weight": float(recovery_weight),
        "recovery_positive_count": int(recovery.sum()),
        "bucket_counts": bucket_counts,
        "bucket_weights": bucket_weights,
        "weight_min": float(weights.min()),
        "weight_max": float(weights.max()),
        "weight_mean": float(weights.mean()),
        "weighted_bucket_mass": {
            name: float(weights[bucket_indices == index].sum())
            for index, name in enumerate(CURVATURE_BUCKET_NAMES)
        },
        "weights_sha256": hashlib.sha256(weights.tobytes()).hexdigest(),
    }
    return SamplingPlan(weights=tensor, summary=summary)


def seeded_weighted_sampler(
    dataset: Any,
    weights: torch.Tensor,
    seed: int,
) -> tuple[WeightedRandomSampler, torch.Generator, list[int]]:
    if len(weights) != len(dataset):
        raise ValueError("Sampler weights length must equal dataset length")
    generator = torch.Generator().manual_seed(seed)
    sampler = WeightedRandomSampler(
        weights=weights,
        num_samples=len(dataset),
        replacement=True,
        generator=generator,
    )
    state = generator.get_state()
    first_epoch_order = [int(index) for index in sampler]
    generator.set_state(state)
    return sampler, generator, first_epoch_order
