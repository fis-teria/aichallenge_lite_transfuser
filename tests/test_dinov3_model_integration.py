"""DINO camera selection through real model/config/checkpoint construction."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from typing import Any

import pytest
import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.models.camera_encoder import CameraEncoder
from aic_transfuser_lite.models.dinov3_camera_encoder import DinoV3CameraEncoder
from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TimeCheckpointIdentity, load_time_checkpoint, save_time_checkpoint,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.train_v3 import (
    build_full_control_model_v3, full_control_model_kwargs_v3, load_full_control_config_v3,
)


ROOT = Path(__file__).resolve().parents[1]


class FakeDinoV3(nn.Module):
    """Only the official patch-token interface; no pretrained-weight claim."""

    embed_dim = 384

    def __init__(self) -> None:
        super().__init__()
        self.patch = nn.Conv2d(3, 384, kernel_size=16, stride=16, bias=False)
        self.seen: list[tuple[int, bool]] = []

    def forward_features(self, image: torch.Tensor) -> dict[str, torch.Tensor]:
        self.seen.append((image.shape[0], self.training))
        return {"x_norm_patchtokens": self.patch(image).flatten(2).transpose(1, 2)}


@pytest.fixture
def dino_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    repository = tmp_path / "dinov3"
    repository.mkdir()
    (repository / "hubconf.py").write_text("# local fixture\n", encoding="utf-8")
    checkpoint = tmp_path / "weights.pth"
    checkpoint.write_bytes(b"test-only-weight-identity")

    def load(repo: str, name: str, *, source: str, weights: str) -> nn.Module:
        assert Path(repo) == repository
        assert name == "dinov3_vits16" and source == "local"
        assert Path(weights) == checkpoint
        return FakeDinoV3()

    monkeypatch.setattr(torch.hub, "load", load)
    return {"backbone": "dinov3_vits16", "frozen": True,
            "repository_path": str(repository), "checkpoint_path": str(checkpoint),
            "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest()}


def config(camera: dict[str, Any] | None = None) -> TimeModelConfig:
    return TimeModelConfig(image_height=32, image_width=32, lidar_points=32,
                           hidden_dim=16, camera_tokens_hw=(2, 2), lidar_tokens=4,
                           fusion_depth=1, fusion_heads=4, camera_encoder=camera)


def batch() -> ModelBatchV3:
    mask = torch.tensor([[False, True, False, True]] * 2)
    return ModelBatchV3(
        image=torch.randn(2, 4, 3, 32, 32), image_mask=mask,
        lidar=torch.rand(2, 4, 2, 32), lidar_mask=mask.clone(),
        ego=torch.zeros(2, 10, 4), ego_feature_mask=torch.ones(2, 10, 4, dtype=torch.bool),
        command_history=torch.zeros(2, 10, 3), command_mask=torch.zeros(2, 10, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(2, 4, 2), requested_outputs=frozenset({"trajectory"}),
    )


def test_time_model_dino_mask_gradient_and_optimizer(dino_config: dict[str, Any]) -> None:
    torch.manual_seed(4)
    model = build_time_model(config(dino_config)).train()
    camera = model.backbone.camera
    assert isinstance(camera, DinoV3CameraEncoder)
    inputs = batch()
    prediction = model(inputs)
    assert prediction.shape == (2, 30, 2) and torch.isfinite(prediction).all()
    assert camera.backbone.seen == [(4, False)]
    before_backbone = camera.backbone.patch.weight.detach().clone()
    before_projection = camera.projection.weight.detach().clone()
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3)
    prediction.square().mean().backward()
    assert all(not p.requires_grad and p.grad is None for p in camera.backbone.parameters())
    for module in (camera.projection, model.backbone.fusion, model.delta_head):
        gradients = [p.grad for p in module.parameters() if p.grad is not None]
        assert gradients and all(torch.isfinite(g).all() for g in gradients)
        assert sum(g.abs().sum().item() for g in gradients) > 0
    optimizer.step()
    torch.testing.assert_close(camera.backbone.patch.weight, before_backbone, rtol=0, atol=0)
    assert not torch.equal(camera.projection.weight, before_projection)
    model.eval()
    changed = inputs.image.clone()
    changed[~inputs.image_mask] = 1e6
    with torch.no_grad():
        torch.testing.assert_close(model(inputs), model(replace(inputs, image=changed)), rtol=0, atol=0)


def test_dino_checkpoint_roundtrip_and_camera_identity(
    dino_config: dict[str, Any], tmp_path: Path,
) -> None:
    settings = config(dino_config)
    model = build_time_model(settings).eval()
    identity = TimeCheckpointIdentity("a" * 64, "b" * 64, "c" * 64, "test-only")
    path = tmp_path / "model.pt"
    save_time_checkpoint(path, config=settings, identity=identity, model=model,
                         optimizer=None, scheduler=None, epoch=0, global_step=0)
    restored_config = TimeModelConfig.from_dict(settings.to_dict())
    restored = build_time_model(restored_config).eval()
    load_time_checkpoint(path, config=restored_config, identity=identity, model=restored, mode="finetune")
    inputs = batch()
    with torch.no_grad():
        torch.testing.assert_close(model(inputs), restored(inputs), rtol=0, atol=0)
    altered = deepcopy(model.state_dict())
    altered["_extra_state"]["backbone"]["camera_encoder"]["checkpoint_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="configuration mismatch"):
        restored.load_state_dict(altered, strict=True)
    with pytest.raises(ValueError, match="model/config mismatch: camera_encoder"):
        save_time_checkpoint(tmp_path / "mislabeled.pt", config=replace(settings, camera_encoder=None),
                             identity=identity, model=model, optimizer=None, scheduler=None,
                             epoch=0, global_step=0)
    # Caller-owned dictionaries must not mutate the recorded checkpoint identity.
    dino_config["checkpoint_sha256"] = "1" * 64
    assert model.get_extra_state()["backbone"]["camera_encoder"]["checkpoint_sha256"] != "1" * 64


def test_legacy_config_and_checkpoint_identity_are_unchanged() -> None:
    legacy = json.loads((ROOT / "configs/time_path_p1/command_off.json").read_text(encoding="utf-8"))
    assert "camera_encoder" not in legacy
    assert TimeModelConfig.from_dict(legacy).to_dict() == legacy
    assert "camera_encoder" not in config().model_kwargs()
    model = build_time_model(config())
    assert isinstance(model.backbone.camera, CameraEncoder)
    assert "camera_encoder" not in model.get_extra_state()["backbone"]
    restored = build_time_model(config())
    restored.load_state_dict(model.state_dict(), strict=True)
    missing = dict(legacy)
    missing.pop("image_height")
    with pytest.raises(ValueError, match="keys mismatch"):
        TimeModelConfig.from_dict(missing)


@pytest.mark.parametrize("overrides, message", [
    ({"output_dim": 32}, "fusion shape"),
    ({"token_h": True}, "fusion shape"),
    ({"frozen": False}, "frozen=true"),
    ({"checkpoint_sha256": "bad"}, "SHA-256"),
    ({"backbone": "typo"}, "unsupported camera"),
    ({"unexpected": 1}, "unknown camera_encoder"),
])
def test_camera_config_mismatch_fails_before_loading(overrides: dict[str, Any], message: str) -> None:
    with pytest.raises((TypeError, ValueError), match=message):
        config({"backbone": "dinov3_vits16", **overrides}).validate()


def test_missing_local_weights_and_patch_shape_are_explicit() -> None:
    with pytest.raises(FileNotFoundError, match="explicit"):
        build_time_model(config({"backbone": "dinov3_vits16"}))
    with pytest.raises(ValueError, match="divisible"):
        replace(config({"backbone": "dinov3_vits16"}), image_width=33).validate()


def test_portable_dino_template_requires_machine_local_paths() -> None:
    value = json.loads((ROOT / "configs/time_path_p1/command_off_dinov3.json").read_text(encoding="utf-8"))
    settings = TimeModelConfig.from_dict(value)
    assert settings.to_dict() == value
    assert settings.camera_encoder["repository_path"] is None
    assert settings.camera_encoder["checkpoint_path"] is None
    with pytest.raises(FileNotFoundError, match="explicit"):
        build_time_model(settings)


def test_full_control_builder_forwards_camera_config(dino_config: dict[str, Any]) -> None:
    settings = load_full_control_config_v3(ROOT / "configs/models/full_control_lite_v3.yaml")
    assert "camera_encoder" not in full_control_model_kwargs_v3(settings)
    settings["model"]["camera_encoder"] = dino_config
    model = build_full_control_model_v3(settings)
    assert isinstance(model.camera, DinoV3CameraEncoder)
    assert model.camera.token_count == 16 and model.camera.output_dim == 128
    saved = full_control_model_kwargs_v3(settings)
    dino_config["frozen"] = False
    assert saved["camera_encoder"]["frozen"] is True
