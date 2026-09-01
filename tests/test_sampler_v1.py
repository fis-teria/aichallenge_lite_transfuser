from __future__ import annotations

import numpy as np
import pandas as pd
import pytest
import torch

from aic_transfuser_lite.training.sampler import (
    build_capped_curvature_recovery_plan,
    seeded_weighted_sampler,
)


def circular_path(curvature_per_m: float) -> torch.Tensor:
    distances = torch.arange(0.5, 3.01, 0.5)
    if curvature_per_m == 0.0:
        return torch.stack((distances, torch.zeros_like(distances)), dim=-1)
    radius = 1.0 / curvature_per_m
    theta = distances / radius
    return torch.stack(
        (radius * torch.sin(theta), radius * (1.0 - torch.cos(theta))), dim=-1
    )


def frame_with_buckets() -> pd.DataFrame:
    paths = [circular_path(0.0)]
    paths.extend(circular_path(0.08) for _ in range(2))
    paths.extend(circular_path(0.18) for _ in range(4))
    rows: list[dict[str, float]] = []
    for row_index, path in enumerate(paths):
        row: dict[str, float] = {
            "recovery_flag": float(row_index == 0),
        }
        for index, point in enumerate(path):
            row[f"wp_{index}_x"] = float(point[0])
            row[f"wp_{index}_y"] = float(point[1])
        rows.append(row)
    return pd.DataFrame(rows)


def test_inverse_frequency_plan_balances_observed_buckets_and_caps_weights() -> None:
    plan = build_capped_curvature_recovery_plan(
        frame_with_buckets(),
        num_waypoints=6,
        straight_threshold_per_m=0.03,
        sharp_threshold_per_m=0.12,
        max_weight=4.0,
        recovery_weight=4.0,
    )
    np.testing.assert_allclose(plan.weights.numpy(), [4.0, 2.0, 2.0, 1, 1, 1, 1])
    assert plan.summary["bucket_counts"] == {
        "straight": 1,
        "curve": 2,
        "sharp": 4,
    }
    assert plan.summary["weighted_bucket_mass"] == pytest.approx(
        {"straight": 4.0, "curve": 4.0, "sharp": 4.0}
    )
    assert plan.summary["weight_max"] == 4.0
    assert plan.summary["recovery_positive_count"] == 1


def test_weighted_sampler_preview_is_replayed_from_the_same_generator_state() -> None:
    dataset = list(range(7))
    plan = build_capped_curvature_recovery_plan(
        frame_with_buckets(),
        num_waypoints=6,
        straight_threshold_per_m=0.03,
        sharp_threshold_per_m=0.12,
        max_weight=4.0,
        recovery_weight=4.0,
    )
    first, _, preview = seeded_weighted_sampler(dataset, plan.weights, 42)
    assert [int(index) for index in first] == preview
    second, _, second_preview = seeded_weighted_sampler(dataset, plan.weights, 42)
    assert second_preview == preview
    assert [int(index) for index in second] == preview
