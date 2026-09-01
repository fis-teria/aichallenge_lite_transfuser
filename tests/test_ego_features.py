from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.data.ego_features import (
    LEGACY_EGO_FEATURES,
    configured_ego_features,
    select_ego_features,
)
from aic_transfuser_lite.models.factory import build_model
from tools.evaluate_checkpoint import apply_scenario, scenario_feature_indices


ROOT = Path(__file__).resolve().parents[1]


def test_legacy_config_keeps_five_feature_contract() -> None:
    config = load_config(ROOT / "configs/transfuser_lite_v0.yaml")
    assert configured_ego_features(config["data"]) == LEGACY_EGO_FEATURES


def test_speed_only_contract_builds_one_dimensional_model() -> None:
    config = deepcopy(load_config(ROOT / "configs/transfuser_lite_v0.yaml"))
    config["data"]["ego_dim"] = 1
    config["data"]["ego_features"] = ["speed_mps"]
    model = build_model(config).eval()
    with torch.inference_mode():
        output = model(
            torch.zeros(1, 3, config["data"]["image_height"], config["data"]["image_width"]),
            torch.ones(1, config["data"]["lidar_points"]),
            torch.tensor([[5.0]]),
        )
    assert output["waypoints"].shape == (1, config["data"]["num_waypoints"], 2)


def test_feature_contract_rejects_dimension_drift_and_unknown_names() -> None:
    with pytest.raises(ValueError, match="does not match"):
        configured_ego_features({"ego_dim": 5, "ego_features": ["speed_mps"]})
    with pytest.raises(ValueError, match="Unsupported"):
        configured_ego_features({"ego_dim": 1, "ego_features": ["command"]})


def test_feature_selection_is_ordered_and_explicit() -> None:
    result = select_ego_features(
        ("speed_mps", "yaw_rate_rps"),
        {"speed_mps": 4.0, "yaw_rate_rps": -0.2},
    )
    assert result == (4.0, -0.2)


def test_turn_state_ablation_is_structural_noop_when_features_absent() -> None:
    features = ("speed_mps",)
    image = torch.zeros(2, 3, 2, 2)
    lidar = torch.zeros(2, 4)
    ego = torch.tensor([[3.0], [5.0]])
    generator = torch.Generator().manual_seed(1)
    _, _, ablated = apply_scenario(
        image, lidar, ego, "turn_state_zero", generator, features
    )
    assert torch.equal(ablated, ego)
    assert scenario_feature_indices(features, "turn_state_zero") == ()


def test_legacy_turn_state_indices_are_named_not_hardcoded_by_shape() -> None:
    assert scenario_feature_indices(LEGACY_EGO_FEATURES, "turn_state_zero") == (2, 3)
