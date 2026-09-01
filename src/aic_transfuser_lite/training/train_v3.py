from __future__ import annotations

from pathlib import Path
from typing import Sequence

import torch
import yaml

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3

from .checkpoint_v3 import (
    ExperimentIdentityV3,
    load_checkpoint_v3,
    save_checkpoint_v3,
)
from .losses_v3 import LossWeightsV3, compute_losses_v3
from .sampler_v3 import DeterministicSamplerV3


class TrainerV3:
    """Small deterministic trainer whose pause boundary is one optimizer step."""

    def __init__(
        self,
        *,
        model: torch.nn.Module,
        batches: Sequence[ModelBatchV3],
        optimizer: torch.optim.Optimizer,
        identity: ExperimentIdentityV3,
        loss_weights: LossWeightsV3 = LossWeightsV3(),
        scheduler: object | None = None,
    ) -> None:
        if not batches:
            raise ValueError("TrainerV3 requires at least one batch")
        self.model = model
        self.batches = batches
        self.optimizer = optimizer
        self.identity = identity
        self.loss_weights = loss_weights
        self.scheduler = scheduler
        self.sampler = DeterministicSamplerV3(len(batches), seed=identity.seed)
        self.global_step = 0
        self.logs: list[dict[str, float]] = []

    def train_steps(self, count: int) -> list[dict[str, float]]:
        if count < 0:
            raise ValueError("training step count must be non-negative")
        self.model.train()
        produced: list[dict[str, float]] = []
        for _ in range(count):
            batch = self.batches[self.sampler.next_index()]
            if batch.targets is None:
                raise ValueError("training batch is missing targets")
            self.optimizer.zero_grad(set_to_none=True)
            report = compute_losses_v3(self.model(batch), batch.targets, self.loss_weights)
            if not torch.isfinite(report.total):
                raise FloatingPointError("non-finite V3 loss before backward")
            report.total.backward()
            self.optimizer.step()
            if self.scheduler is not None:
                self.scheduler.step()
            self.global_step += 1
            row = {"step": float(self.global_step), **report.scalar_log()}
            self.logs.append(row)
            produced.append(row)
        return produced

    def save(self, path: Path) -> None:
        save_checkpoint_v3(
            path,
            identity=self.identity,
            model=self.model,
            optimizer=self.optimizer,
            scheduler=self.scheduler,
            sampler_state=self.sampler.state_dict(),
            global_step=self.global_step,
        )

    def resume(self, path: Path) -> None:
        sampler_state, self.global_step = load_checkpoint_v3(
            path,
            expected_identity=self.identity,
            model=self.model,
            optimizer=self.optimizer,
            scheduler=self.scheduler,
        )
        self.sampler.load_state_dict(sampler_state)


def load_full_control_config_v3(path: str | Path) -> dict[str, object]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("format") != "aic_model_config_v3":
        raise ValueError("not an aic_model_config_v3 configuration")
    for section in ("model", "data", "loss", "training"):
        if not isinstance(raw.get(section), dict):
            raise ValueError(f"full-control config missing {section} mapping")
    model = raw["model"]
    data = raw["data"]
    if model.get("name") != "full_control_lite_v3" or not bool(model.get("control_head_enabled")):
        raise ValueError("full-control training requires enabled FullControlLiteV3 control head")
    if list(data.get("ego_features", [])) == []:
        raise ValueError("full-control config requires explicit ego_features")
    if float(raw["loss"].get("current_control", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero current_control loss")
    return raw


def build_full_control_model_v3(config: dict[str, object]) -> FullControlLiteV3:
    model = config["model"]
    data = config["data"]
    bounds = model["control_bounds"]
    return FullControlLiteV3(
        image_height=int(data["image_height"]), image_width=int(data["image_width"]),
        lidar_points=int(data["lidar_points"]), ego_dim=len(data["ego_features"]),
        hidden_dim=int(model["hidden_dim"]), trajectory_steps=int(model["trajectory_steps"]),
        candidates=int(model["candidates"]), camera_tokens_hw=tuple(model["camera_tokens_hw"]),
        lidar_tokens=int(model["lidar_tokens"]), fusion_depth=int(model["fusion_depth"]),
        fusion_heads=int(model["fusion_heads"]), max_sensor_history=int(model["max_sensor_history"]),
        max_ego_history=int(model["max_ego_history"]), control_head_enabled=True,
        max_steering_rad=float(bounds["max_steering_rad"]), max_speed_mps=float(bounds["max_speed_mps"]),
        min_acceleration_mps2=float(bounds["min_acceleration_mps2"]),
        max_acceleration_mps2=float(bounds["max_acceleration_mps2"]),
    )


def move_batch_v3(batch: ModelBatchV3, device: torch.device) -> ModelBatchV3:
    targets = batch.targets
    moved_targets = None
    if targets is not None:
        moved_targets = targets.__class__(
            trajectory_xy_m=targets.trajectory_xy_m.to(device),
            trajectory_mask=targets.trajectory_mask.to(device), speed_mps=targets.speed_mps.to(device),
            speed_mask=targets.speed_mask.to(device),
            current_control=None if targets.current_control is None else targets.current_control.to(device),
            current_control_mask=None if targets.current_control_mask is None else targets.current_control_mask.to(device),
            control_provenance=targets.control_provenance,
        )
    return ModelBatchV3(
        image=batch.image.to(device), image_mask=batch.image_mask.to(device),
        lidar=batch.lidar.to(device), lidar_mask=batch.lidar_mask.to(device),
        ego=batch.ego.to(device), ego_feature_mask=batch.ego_feature_mask.to(device),
        command_history=batch.command_history.to(device), command_mask=batch.command_mask.to(device),
        sensor_dt_sec=batch.sensor_dt_sec.to(device), targets=moved_targets,
        requested_outputs=batch.requested_outputs,
    )
