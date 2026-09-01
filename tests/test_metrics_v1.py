from __future__ import annotations

import math

import pytest
import torch

from aic_transfuser_lite.training.metrics import (
    V1MetricAccumulator,
    controller_proxy_steering,
    curvature_bucket_indices,
    trajectory_max_abs_curvature,
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


def test_curvature_estimator_and_bucket_boundaries() -> None:
    paths = torch.stack(
        (circular_path(0.0), circular_path(0.08), circular_path(0.18))
    )
    curvature = trajectory_max_abs_curvature(paths)
    assert curvature[0].item() == pytest.approx(0.0, abs=1e-6)
    assert curvature[1].item() == pytest.approx(0.08, rel=0.02)
    assert curvature[2].item() == pytest.approx(0.18, rel=0.02)
    assert curvature_bucket_indices(
        paths,
        straight_threshold_per_m=0.03,
        sharp_threshold_per_m=0.12,
    ).tolist() == [0, 1, 2]


def test_controller_proxy_applies_the_dataset_wheelbase() -> None:
    path = torch.tensor([[[1.0, 0.1], [2.0, 0.2]]])
    expected_curvature = 2.0 * 0.1 / (1.0**2 + 0.1**2)
    expected = torch.atan(torch.tensor(1.087 * expected_curvature))
    assert controller_proxy_steering(path).item() == pytest.approx(expected.item())


def test_metric_accumulator_reports_horizon_bucket_and_signed_bias() -> None:
    target = torch.stack(
        (circular_path(0.0), circular_path(0.08), circular_path(0.18))
    )
    predicted = target.clone()
    predicted[2, :, 1] += 0.1
    outputs = {
        "waypoints": predicted,
        "target_speed": torch.tensor([[2.0], [3.0], [4.0]]),
    }
    batch = {
        "waypoints": target,
        "target_speed": torch.tensor([[2.0], [3.5], [4.0]]),
    }
    accumulator = V1MetricAccumulator(num_waypoints=6)
    accumulator.update(outputs, batch)
    result = accumulator.finalize()

    assert result["sample_count"] == 3
    assert len(result["waypoint_horizon_mae_m"]) == 6
    assert result["speed_mae_mps"] == pytest.approx(0.5 / 3.0)
    assert result["controller_proxy_bias_rad"] > 0.0
    assert result["curvature_buckets"]["straight"]["sample_count"] == 1
    assert result["curvature_buckets"]["curve"]["sample_count"] == 1
    assert result["curvature_buckets"]["sharp"]["sample_count"] == 1
    assert result["curvature_buckets"]["straight"]["ade_m"] == pytest.approx(0.0)
    assert result["curvature_buckets"]["sharp"]["ade_m"] == pytest.approx(0.1)
    assert math.isfinite(result["curvature_buckets"]["sharp"]["controller_proxy_mae_rad"])
