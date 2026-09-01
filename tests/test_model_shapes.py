from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.models.factory import build_model


ROOT = Path(__file__).resolve().parents[1]


def test_transfuser_shapes() -> None:
    config = load_config(ROOT / "configs/transfuser_lite_v0.yaml")
    model = build_model(config).eval()
    batch = 2
    data = config["data"]
    with torch.inference_mode():
        output = model(
            torch.randn(batch, 3, data["image_height"], data["image_width"]),
            torch.rand(batch, data["lidar_points"]),
            torch.randn(batch, data["ego_dim"]),
        )
    assert output["waypoints"].shape == (batch, data["num_waypoints"], 2)
    assert output["target_speed"].shape == (batch, 1)
    assert output["stop_logit"].shape == (batch, 1)
    assert torch.isfinite(output["waypoints"]).all()


def test_lidar_only_shapes() -> None:
    config = load_config(ROOT / "configs/lidar_only_v0.yaml")
    model = build_model(config).eval()
    batch = 2
    data = config["data"]
    with torch.inference_mode():
        output = model(
            torch.rand(batch, data["lidar_points"]),
            torch.randn(batch, data["ego_dim"]),
        )
    assert output["waypoints"].shape == (batch, data["num_waypoints"], 2)


def test_late_fusion_shapes() -> None:
    config = load_config(ROOT / "configs/late_fusion_v0.yaml")
    model = build_model(config).eval()
    batch = 2
    data = config["data"]
    with torch.inference_mode():
        output = model(
            torch.randn(batch, 3, data["image_height"], data["image_width"]),
            torch.rand(batch, data["lidar_points"]),
            torch.randn(batch, data["ego_dim"]),
        )
    assert output["waypoints"].shape == (batch, data["num_waypoints"], 2)
    assert output["target_speed"].shape == (batch, 1)
    assert output["stop_logit"].shape == (batch, 1)
    assert output["mode_logits"].shape == (batch, len(data["mode_classes"]))
    assert output["direct_control"].shape == (batch, 2)
    assert all(torch.isfinite(value).all() for value in output.values())


def test_late_fusion_rejects_mismatched_batch_sizes() -> None:
    config = load_config(ROOT / "configs/late_fusion_v0.yaml")
    model = build_model(config).eval()
    data = config["data"]

    with pytest.raises(ValueError, match="batch sizes must match"):
        model(
            torch.randn(2, 3, data["image_height"], data["image_width"]),
            torch.rand(1, data["lidar_points"]),
            torch.randn(2, data["ego_dim"]),
        )
