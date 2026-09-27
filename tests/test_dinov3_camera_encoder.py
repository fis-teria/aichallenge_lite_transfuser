from __future__ import annotations

import hashlib

import pytest
import torch
from torch import nn

from aic_transfuser_lite.models.camera_encoder import CameraEncoder
from aic_transfuser_lite.models.dinov3_camera_encoder import (
    DINOV3_EMBED_DIM,
    DinoV3CameraEncoder,
    build_camera_encoder,
)
from aic_transfuser_lite.models.time_backbone_v1 import encode_valid


class FakeDinoV3(nn.Module):
    embed_dim = DINOV3_EMBED_DIM

    def __init__(self) -> None:
        super().__init__()
        self.patch = nn.Conv2d(3, self.embed_dim, kernel_size=16, stride=16, bias=False)
        self.training_seen: list[bool] = []

    def forward_features(self, image: torch.Tensor):
        self.training_seen.append(self.training)
        tokens = self.patch(image).flatten(2).transpose(1, 2)
        return {"x_norm_patchtokens": tokens}


def adapter() -> DinoV3CameraEncoder:
    return DinoV3CameraEncoder(
        output_dim=32,
        token_h=2,
        token_w=3,
        frozen=True,
        backbone=FakeDinoV3(),
    )


def test_shape_history_mask_freeze_and_projection_gradient():
    model = adapter().train()
    images = torch.randn(2, 3, 3, 32, 48)
    mask = torch.tensor([[False, True, True], [True, False, True]])
    tokens = encode_valid(model, images, mask)
    assert tokens.shape == (2, 3, 6, 32)
    assert torch.count_nonzero(tokens[~mask]) == 0
    tokens.sum().backward()
    assert all(parameter.grad is None for parameter in model.backbone.parameters())
    assert model.projection.weight.grad is not None
    assert torch.isfinite(model.projection.weight.grad).all()
    assert model.backbone.training_seen and not any(model.backbone.training_seen)


def test_invalid_image_and_patch_token_contract_fail_closed():
    model = adapter()
    with pytest.raises(ValueError, match="divisible"):
        model(torch.randn(1, 3, 31, 48))
    with pytest.raises(ValueError, match="finite"):
        model(torch.full((1, 3, 32, 48), float("nan")))

    class WrongTokens(FakeDinoV3):
        def forward_features(self, image):
            return {"x_norm_patchtokens": torch.zeros(image.shape[0], 1, self.embed_dim)}

    wrong = DinoV3CameraEncoder(output_dim=8, token_h=1, token_w=1, backbone=WrongTokens())
    with pytest.raises(RuntimeError, match="token count mismatch"):
        wrong(torch.zeros(1, 3, 32, 48))


def test_factory_preserves_resnet_and_requires_explicit_dino_weights(tmp_path):
    resnet = build_camera_encoder(
        {"backbone": "resnet18", "output_dim": 16, "token_h": 2, "token_w": 2, "pretrained": False}
    )
    assert isinstance(resnet, CameraEncoder)
    with pytest.raises(FileNotFoundError, match="explicit"):
        build_camera_encoder(
            {"backbone": "dinov3_vits16", "output_dim": 16, "token_h": 2, "token_w": 2}
        )
    repository = tmp_path / "dinov3"
    repository.mkdir()
    (repository / "hubconf.py").write_text("# fixture", encoding="utf-8")
    checkpoint = tmp_path / "weights.pth"
    checkpoint.write_bytes(b"fixture")
    digest = hashlib.sha256(b"fixture").hexdigest()
    with pytest.raises(ValueError, match="mismatch"):
        DinoV3CameraEncoder(
            output_dim=16,
            token_h=2,
            token_w=2,
            repository_path=repository,
            checkpoint_path=checkpoint,
            checkpoint_sha256="0" * 64,
        )
    assert digest != "0" * 64


def test_provenance_is_explicit_for_injected_backbone():
    value = adapter().pretrained_provenance()
    assert value["model_name"] == "dinov3_vits16"
    assert value["patch_size"] == 16
    assert value["embed_dim"] == 384
    assert value["backbone_frozen"] is True
    assert value["injected_test_backbone"] is True
