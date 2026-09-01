from __future__ import annotations

import copy
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.config import load_config, load_v1_config
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.models.lidar_encoder import Lidar1DEncoder
from aic_transfuser_lite.models.transfuser_lite import AICTransFuserLite
from aic_transfuser_lite.models.transfuser_lite_v1 import AICTransFuserLiteV1


ROOT = Path(__file__).resolve().parents[1]
STATIC_CONFIG = ROOT / "configs/transfuser_lite_v1_static.yaml"


def test_static_v1_factory_and_exact_input_output_contract() -> None:
    config = copy.deepcopy(load_v1_config(STATIC_CONFIG))
    config["model"]["camera"]["pretrained"] = False
    model = build_model(config).eval()

    assert isinstance(model, AICTransFuserLiteV1)
    assert model.camera.token_count == 60
    assert model.lidar.token_count == 64
    assert model.lidar.input_channels == 2
    assert model.lidar.angle_sincos.shape == (64, 2)
    assert model.fusion.content_tokens == 125
    assert model.fusion.total_tokens == 126
    assert model.fusion.pooling == "learned_cls"
    assert model.heads.stop is None
    assert model.heads.mode is None
    assert model.heads.direct_control is None
    assert model.camera.pretrained_provenance()["requested"] is False

    with torch.inference_mode():
        output = model(
            torch.randn(2, 3, 180, 320),
            torch.rand(2, 2, 750),
            torch.rand(2, 1),
        )
    assert set(output) == {"waypoints", "target_speed"}
    assert output["waypoints"].shape == (2, 6, 2)
    assert output["target_speed"].shape == (2, 1)


def test_static_v1_rejects_input_shape_or_batch_drift() -> None:
    config = copy.deepcopy(load_v1_config(STATIC_CONFIG))
    config["model"]["camera"]["pretrained"] = False
    model = build_model(config).eval()
    image = torch.randn(1, 3, 180, 320)
    lidar = torch.rand(1, 2, 750)
    ego = torch.rand(1, 1)

    with pytest.raises(ValueError, match=r"Expected lidar \[B,2,750\]"):
        model(image, lidar[:, :1], ego)
    with pytest.raises(ValueError, match="batch sizes differ"):
        model(image, lidar.repeat(2, 1, 1), ego)
    with pytest.raises(ValueError, match=r"Expected image \[B,3,180,320\]"):
        model(image[:, :, :-1], lidar, ego)


def test_angle_encoding_has_fixed_geometry_and_trainable_projection() -> None:
    encoder = Lidar1DEncoder(
        output_dim=128,
        token_count=64,
        input_channels=2,
        lidar_points=750,
        angle_min_rad=-1.5,
        angle_increment_rad=3.0 / 749.0,
        use_angle_encoding=True,
    )
    assert encoder.angle_sincos is not None
    torch.testing.assert_close(
        encoder.angle_sincos[0],
        torch.tensor([torch.sin(torch.tensor(-1.5)), torch.cos(torch.tensor(-1.5))]),
    )
    torch.testing.assert_close(
        encoder.angle_sincos[-1],
        torch.tensor([torch.sin(torch.tensor(1.5)), torch.cos(torch.tensor(1.5))]),
    )

    output = encoder(torch.rand(2, 2, 750))
    assert output.shape == (2, 64, 128)
    output.square().mean().backward()
    assert encoder.angle_projection is not None
    gradient = encoder.angle_projection.weight.grad
    assert gradient is not None
    assert torch.isfinite(gradient).all()
    assert torch.count_nonzero(gradient) > 0


def test_factory_keeps_format1_and_v0_on_legacy_model_path() -> None:
    phase1 = copy.deepcopy(
        load_v1_config(ROOT / "configs/diagnostics/v1_training_stack_smoke.yaml")
    )
    phase1["model"]["camera"]["pretrained"] = False
    assert isinstance(build_model(phase1), AICTransFuserLite)

    checkpoint = torch.load(
        ROOT / "runs/transfuser_lite_v0/best.pt",
        map_location="cpu",
        weights_only=False,
    )
    legacy = build_model(load_config(ROOT / "configs/transfuser_lite_v0.yaml"))
    legacy.load_state_dict(checkpoint["model"], strict=True)
    assert isinstance(legacy, AICTransFuserLite)
    assert legacy.lidar.input_channels == 1
    assert legacy.lidar.angle_projection is None
