from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.runtime.inference_core import infer
from aic_transfuser_lite.training.losses import compute_multitask_loss


ROOT = Path(__file__).resolve().parents[1]


def disable_optional_heads(config: dict) -> dict:
    result = deepcopy(config)
    result["model"].setdefault("heads", {}).update(
        {
            "stop": False,
            "behavior_mode": False,
            "direct_control_aux": False,
        }
    )
    return result


@pytest.mark.parametrize(
    "config_name", ["transfuser_lite_v0.yaml", "late_fusion_v0.yaml", "lidar_only_v0.yaml"]
)
def test_optional_heads_are_not_constructed_or_returned(config_name: str) -> None:
    config = disable_optional_heads(load_config(ROOT / "configs" / config_name))
    model = build_model(config).eval()
    data = config["data"]
    batch = 2
    image = torch.randn(batch, 3, data["image_height"], data["image_width"])
    lidar = torch.rand(batch, data["lidar_points"])
    ego = torch.randn(batch, data["ego_dim"])
    with torch.inference_mode():
        if config["model"]["name"] == "lidar_only":
            output = model(lidar, ego)
        else:
            output = model(image, lidar, ego)

    assert set(output) == {"waypoints", "target_speed"}
    assert model.heads.stop is None
    assert model.heads.mode is None
    assert model.heads.direct_control is None


def test_v0_checkpoint_still_loads_strictly_with_all_outputs() -> None:
    checkpoint = torch.load(ROOT / "runs/transfuser_lite_v0/best.pt", map_location="cpu")
    model = build_model(checkpoint["config"]).eval()
    model.load_state_dict(checkpoint["model"], strict=True)
    data = checkpoint["config"]["data"]
    with torch.inference_mode():
        output = model(
            torch.zeros(1, 3, data["image_height"], data["image_width"]),
            torch.ones(1, data["lidar_points"]),
            torch.zeros(1, data["ego_dim"]),
        )
    assert set(output) == {
        "waypoints",
        "target_speed",
        "stop_logit",
        "mode_logits",
        "direct_control",
    }


def minimal_outputs() -> dict[str, torch.Tensor]:
    return {
        "waypoints": torch.zeros(2, 6, 2),
        "target_speed": torch.zeros(2, 1),
    }


def minimal_batch() -> dict[str, torch.Tensor]:
    return {
        "waypoints": torch.zeros(2, 6, 2),
        "target_speed": torch.zeros(2, 1),
    }


def test_zero_weight_losses_accept_disabled_optional_heads() -> None:
    loss, parts = compute_multitask_loss(
        minimal_outputs(),
        minimal_batch(),
        {
            "waypoint": 1.0,
            "speed": 0.2,
            "smoothness": 0.0,
            "stop": 0.0,
            "mode": 0.0,
            "direct_control": 0.0,
        },
    )
    assert torch.isfinite(loss)
    assert set(parts) == {"waypoint", "speed", "smoothness"}


@pytest.mark.parametrize("weight_name", ["stop", "mode", "direct_control"])
def test_nonzero_loss_rejects_disabled_optional_head(weight_name: str) -> None:
    weights = {
        "waypoint": 1.0,
        "speed": 0.2,
        "smoothness": 0.0,
        "stop": 0.0,
        "mode": 0.0,
        "direct_control": 0.0,
    }
    weights[weight_name] = 1.0
    with pytest.raises(ValueError, match="Head is disabled"):
        compute_multitask_loss(minimal_outputs(), minimal_batch(), weights)


class MinimalRuntimeModel(torch.nn.Module):
    def forward(
        self, image: torch.Tensor, lidar: torch.Tensor, ego: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        batch = image.shape[0]
        return {
            "waypoints": torch.zeros(batch, 6, 2, device=image.device),
            "target_speed": torch.ones(batch, 1, device=image.device),
        }


def test_inference_does_not_require_optional_outputs() -> None:
    output = infer(
        MinimalRuntimeModel(),
        image=torch.zeros(1, 3, 180, 320),
        lidar=torch.ones(1, 1080),
        ego=torch.zeros(1, 5),
    )
    assert set(output) == {"waypoints", "target_speed"}
