from __future__ import annotations

import copy
from pathlib import Path

import pytest
import torch
from torch import nn

from aic_transfuser_lite.config import (
    ConfigValidationError,
    load_v1_config,
    validate_v1_config,
)
from aic_transfuser_lite.training.checkpoint_v1 import (
    V1_CHECKPOINT_FORMAT,
    load_v1_checkpoint,
    save_v1_checkpoint,
)
from aic_transfuser_lite.training.train_v1 import (
    WarmupCosineScheduler,
    build_v1_optimizer,
    controller_proxy_steering,
)


ROOT = Path(__file__).resolve().parents[1]
V1_CONFIG = ROOT / "configs/diagnostics/v1_training_stack_smoke.yaml"


class TinyGroupedModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.camera = nn.Module()
        self.camera.backbone = nn.Sequential(
            nn.Conv2d(3, 4, kernel_size=1, bias=True),
            nn.BatchNorm2d(4),
        )
        self.projection = nn.Linear(4, 4)
        self.norm = nn.LayerNorm(4)
        self.attention = nn.MultiheadAttention(4, 1, batch_first=True)
        self.position = nn.Parameter(torch.ones(1, 4))


def test_optimizer_groups_cover_every_parameter_once() -> None:
    model = TinyGroupedModel()
    config = load_v1_config(V1_CONFIG)
    optimizer, contract = build_v1_optimizer(model, config)
    grouped = [parameter for group in optimizer.param_groups for parameter in group["params"]]
    assert len(grouped) == len({id(parameter) for parameter in grouped})
    assert {id(parameter) for parameter in grouped} == {
        id(parameter) for parameter in model.parameters()
    }
    by_name = {item["group_name"]: item for item in contract}
    assert by_name["backbone_decay"]["learning_rate"] == pytest.approx(3e-5)
    assert by_name["main_decay"]["learning_rate"] == pytest.approx(3e-4)
    assert by_name["backbone_no_decay"]["weight_decay"] == 0.0
    assert by_name["main_no_decay"]["weight_decay"] == 0.0
    no_decay_names = set(by_name["main_no_decay"]["parameter_names"])
    assert "projection.bias" in no_decay_names
    assert "norm.weight" in no_decay_names
    assert "position" in no_decay_names
    assert "attention.in_proj_bias" in no_decay_names
    decay_names = {
        name
        for group_name in ("main_decay", "backbone_decay")
        for name in by_name[group_name]["parameter_names"]
    }
    assert not any(name.rsplit(".", 1)[-1].endswith("bias") for name in decay_names)


def test_scheduler_warmup_cosine_and_resume_state() -> None:
    parameter = nn.Parameter(torch.ones(()))
    optimizer = torch.optim.AdamW(
        [{"params": [parameter], "lr": 1.0, "initial_lr": 1.0, "group_name": "main"}]
    )
    scheduler = WarmupCosineScheduler(
        optimizer, total_steps=20, warmup_fraction=0.1, min_lr_ratio=0.01
    )
    assert scheduler.get_last_lr()[0] == pytest.approx(0.5)
    scheduler.step()
    assert scheduler.get_last_lr()[0] == pytest.approx(1.0)
    for _ in range(19):
        scheduler.step()
    assert scheduler.get_last_lr()[0] == pytest.approx(0.01)

    restored_optimizer = torch.optim.AdamW(
        [{"params": [nn.Parameter(torch.ones(()))], "lr": 1.0, "initial_lr": 1.0, "group_name": "main"}]
    )
    restored = WarmupCosineScheduler(
        restored_optimizer,
        total_steps=20,
        warmup_fraction=0.1,
        min_lr_ratio=0.01,
    )
    restored.load_state_dict(scheduler.state_dict())
    assert restored.step_count == scheduler.step_count
    assert restored.get_last_lr() == pytest.approx(scheduler.get_last_lr())


def test_controller_proxy_is_zero_for_same_path_and_preserves_turn_sign() -> None:
    straight = torch.tensor([[[1.0, 0.0], [2.0, 0.0]]])
    left = torch.tensor([[[1.0, 0.5], [2.0, 1.0]]])
    right = left.clone()
    right[:, :, 1] *= -1
    assert controller_proxy_steering(straight).item() == pytest.approx(0.0)
    assert controller_proxy_steering(left).item() > 0.0
    assert controller_proxy_steering(right).item() < 0.0


def test_v1_checkpoint_round_trip_and_legacy_rejection(tmp_path: Path) -> None:
    payload = {
        "format_version": V1_CHECKPOINT_FORMAT,
        "model": {"value": torch.ones(1)},
        "config": {"schema_version": "transfuser_lite_v1"},
        "resolved_config": {"schema_version": "transfuser_lite_v1"},
        "epoch": 1,
        "global_step": 2,
        "resolved_config_sha256": "a" * 64,
        "dataset_manifest_sha256": "b" * 64,
    }
    path = tmp_path / "checkpoint.pt"
    save_v1_checkpoint(path, payload)
    loaded = load_v1_checkpoint(path)
    assert loaded["format_version"] == V1_CHECKPOINT_FORMAT
    assert torch.equal(loaded["model"]["value"], torch.ones(1))

    with pytest.raises(ValueError, match="legacy v0 checkpoints"):
        load_v1_checkpoint(ROOT / "runs/transfuser_lite_v0/best.pt")


def test_optimizer_enum_remains_fixed_to_adamw() -> None:
    config = load_v1_config(V1_CONFIG)
    changed = copy.deepcopy(config)
    changed["training"]["optimizer"] = "sgd"
    with pytest.raises(ConfigValidationError, match="training.optimizer"):
        validate_v1_config(changed)
