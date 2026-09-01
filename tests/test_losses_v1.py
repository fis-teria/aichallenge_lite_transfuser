from __future__ import annotations

import pytest
import torch

from aic_transfuser_lite.training.losses_v1 import (
    compute_v1_multitask_loss,
    normalized_horizon_weights,
    target_relative_shape_loss,
)


def test_target_relative_shape_does_not_penalize_correct_curvature() -> None:
    target = torch.tensor(
        [[[1.0, 0.0], [2.0, 0.2], [3.0, 0.8], [4.0, 1.8]]],
        dtype=torch.float32,
    )
    assert target_relative_shape_loss(target.clone(), target) == 0
    straight = torch.tensor(
        [[[1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0]]],
        dtype=torch.float32,
    )
    assert float(target_relative_shape_loss(straight, target)) > 0.0


def test_v1_loss_exposes_raw_and_weighted_components() -> None:
    target = torch.tensor(
        [[[1.0, 0.0], [2.0, 0.2], [3.0, 0.8]]], dtype=torch.float32
    )
    outputs = {
        "waypoints": target + 0.25,
        "target_speed": torch.tensor([[3.0]]),
    }
    batch = {
        "waypoints": target,
        "target_speed": torch.tensor([[2.5]]),
    }
    weights = {
        "waypoint": 1.0,
        "speed": 0.2,
        "shape": 0.0,
        "stop": 0.0,
        "mode": 0.0,
        "direct_control": 0.0,
    }
    total, raw, weighted = compute_v1_multitask_loss(
        outputs, batch, weights, [1.0, 1.0, 1.0]
    )
    assert set(raw) == {"waypoint", "speed", "shape"}
    assert torch.allclose(weighted["speed"], raw["speed"] * 0.2)
    assert torch.allclose(weighted["shape"], torch.zeros_like(weighted["shape"]))
    assert torch.allclose(total, sum(weighted.values()))


def test_horizon_weights_are_normalized_to_mean_one() -> None:
    weights = normalized_horizon_weights(
        [1.4, 1.3, 1.2, 0.9, 0.7, 0.5],
        device=torch.device("cpu"),
        dtype=torch.float32,
    )
    assert float(weights.mean()) == pytest.approx(1.0)


def test_nonzero_optional_loss_without_head_is_rejected() -> None:
    outputs = {
        "waypoints": torch.zeros(1, 3, 2),
        "target_speed": torch.zeros(1, 1),
    }
    batch = {
        "waypoints": torch.zeros(1, 3, 2),
        "target_speed": torch.zeros(1, 1),
    }
    weights = {
        "waypoint": 1.0,
        "speed": 0.2,
        "shape": 0.0,
        "stop": 1.0,
        "mode": 0.0,
        "direct_control": 0.0,
    }
    with pytest.raises(ValueError, match="Head is disabled"):
        compute_v1_multitask_loss(outputs, batch, weights, [1.0, 1.0, 1.0])
