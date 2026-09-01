from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.models.factory import build_model


ROOT = Path(__file__).resolve().parents[1]
E3A_CONFIG = ROOT / "configs/diagnostics/e3a_speed_only_ego_seed42.yaml"
E3B_CONFIG = ROOT / "configs/diagnostics/e3b_learned_cls_seed42.yaml"


def test_e3b_config_changes_only_fusion_pooling() -> None:
    e3a = load_config(E3A_CONFIG)
    e3b = load_config(E3B_CONFIG)
    expected = deepcopy(e3a)
    expected["model"]["fusion"]["pooling"] = "learned_cls"
    assert e3b == expected


def test_component_seeded_cls_adds_only_one_parameter() -> None:
    e3a = build_model(load_config(E3A_CONFIG))
    e3b = build_model(load_config(E3B_CONFIG))
    e3a_state = e3a.state_dict()
    e3b_state = e3b.state_dict()

    assert set(e3b_state) - set(e3a_state) == {"fusion.cls_token"}
    assert set(e3a_state) - set(e3b_state) == set()
    for name, value in e3a_state.items():
        assert value.shape == e3b_state[name].shape, name
        assert torch.equal(value, e3b_state[name]), name
    assert e3b_state["fusion.cls_token"].shape == (1, 1, 128)
    assert sum(parameter.numel() for parameter in e3b.parameters()) == (
        sum(parameter.numel() for parameter in e3a.parameters()) + 128
    )


def test_pooling_selects_cls_for_e3b_and_ego_for_e3a() -> None:
    e3a = build_model(load_config(E3A_CONFIG)).fusion.eval()
    e3b = build_model(load_config(E3B_CONFIG)).fusion.eval()
    batch = 2
    image = torch.randn(batch, e3a.image_tokens, 128)
    lidar = torch.randn(batch, e3a.lidar_tokens, 128)
    ego = torch.randn(batch, 1, 128)

    with torch.inference_mode():
        e3a_fused, e3a_pooled = e3a(image, lidar, ego)
        e3b_fused, e3b_pooled = e3b(image, lidar, ego)

    assert e3a_fused.shape == (batch, e3a.total_tokens, 128)
    assert e3b_fused.shape == (batch, e3b.total_tokens, 128)
    assert e3b.total_tokens == e3a.total_tokens + 1
    torch.testing.assert_close(e3a_pooled, e3a_fused[:, -1], rtol=0, atol=0)
    torch.testing.assert_close(e3b_pooled, e3b_fused[:, 0], rtol=0, atol=0)


def test_cls_parameter_receives_gradient() -> None:
    fusion = build_model(load_config(E3B_CONFIG)).fusion
    batch = 2
    image = torch.randn(batch, fusion.image_tokens, 128)
    lidar = torch.randn(batch, fusion.lidar_tokens, 128)
    ego = torch.randn(batch, 1, 128)
    _, pooled = fusion(image, lidar, ego)
    pooled.square().mean().backward()

    assert fusion.cls_token is not None
    assert fusion.cls_token.grad is not None
    assert torch.isfinite(fusion.cls_token.grad).all()
    assert torch.count_nonzero(fusion.cls_token.grad) > 0


def test_legacy_e3a_checkpoint_still_loads_strictly() -> None:
    checkpoint = torch.load(
        ROOT / "runs/diagnostics/e3a_speed_only_ego_seed42_100ep/best.pt",
        map_location="cpu",
        weights_only=False,
    )
    model = build_model(checkpoint["config"])
    model.load_state_dict(checkpoint["model"], strict=True)
    assert model.fusion.pooling == "ego"
    assert model.fusion.cls_token is None


def test_unknown_pooling_is_rejected() -> None:
    config = deepcopy(load_config(E3A_CONFIG))
    config["model"]["fusion"]["pooling"] = "mystery"
    with pytest.raises(ValueError, match="model.fusion.pooling"):
        build_model(config)
