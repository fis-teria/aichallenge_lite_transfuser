from __future__ import annotations

from pathlib import Path
from typing import Sequence

import torch
import yaml

from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_ONTOLOGY_V1,
    BEHAVIOR_SIDE_NAMES_V1,
)
from aic_transfuser_lite.contracts.model_batch_v3 import (
    COMMAND_HISTORY_ALIGNMENT_V3,
    ModelBatchV3,
)
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3

from .checkpoint_v3 import (
    ExperimentIdentityV3,
    load_checkpoint_v3,
    save_checkpoint_v3,
)
from .losses_v3 import LossWeightsV3, compute_losses_v3
from .sampler_v3 import DeterministicSamplerV3


def balanced_class_weights_v3(
    batches: Sequence[ModelBatchV3], *, target_name: str, mask_name: str, class_count: int,
    require_all_classes: bool = True,
) -> tuple[float, ...]:
    """Derive inverse-frequency weights from the selected training split only."""

    counter = getattr(batches, "class_counts", None)
    if callable(counter):
        counts = counter(target_name, class_count)
    else:
        counts = torch.zeros(class_count, dtype=torch.long)
        for batch in batches:
            if batch.targets is None:
                continue
            target = getattr(batch.targets, target_name)
            mask = getattr(batch.targets, mask_name)
            if target is None or mask is None:
                continue
            selected = target[mask].detach().cpu()
            counts += torch.bincount(selected, minlength=class_count)[:class_count]
    missing = torch.nonzero(counts == 0, as_tuple=False).flatten().tolist()
    if missing and require_all_classes:
        raise ValueError(f"training split has no valid {target_name} samples for classes {missing}")
    total = float(counts.sum())
    divisor = counts.to(torch.float64)
    values = torch.where(
        divisor > 0, total / (float(class_count) * divisor), torch.ones_like(divisor)
    )
    return tuple(float(value) for value in values)


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
        first_parameter = next(iter(model.parameters()), None)
        self.device = torch.device("cpu") if first_parameter is None else first_parameter.device
        self.sampler = DeterministicSamplerV3(len(batches), seed=identity.seed)
        self.global_step = 0
        self.logs: list[dict[str, float]] = []

    def train_steps(self, count: int) -> list[dict[str, float]]:
        if count < 0:
            raise ValueError("training step count must be non-negative")
        if bool(getattr(self.model, "freeze_except_control_sequence", False)):
            self.model.eval()
            sequence_head = getattr(self.model, "control_sequence_head", None)
            if sequence_head is None:
                raise ValueError("frozen-backbone training requires a control sequence head")
            sequence_head.train()
        else:
            self.model.train()
        produced: list[dict[str, float]] = []
        for _ in range(count):
            batch = move_batch_v3(self.batches[self.sampler.next_index()], self.device)
            if batch.targets is None:
                raise ValueError("training batch is missing targets")
            self.optimizer.zero_grad(set_to_none=True)
            output = self.model(batch)
            report = compute_losses_v3(output, batch.targets, self.loss_weights)
            if not torch.isfinite(report.total):
                raise FloatingPointError("non-finite V3 loss before backward")
            report.total.backward()
            self.optimizer.step()
            if self.scheduler is not None:
                self.scheduler.step()
            self.global_step += 1
            row = {"step": float(self.global_step), **report.scalar_log()}
            if output.behavior_logits is not None and batch.targets.behavior_class is not None:
                _add_masked_accuracy(
                    row, "behavior", output.behavior_logits,
                    batch.targets.behavior_class, batch.targets.behavior_mask,
                )
            if output.behavior_side_logits is not None and batch.targets.behavior_side is not None:
                _add_masked_accuracy(
                    row, "behavior_side", output.behavior_side_logits,
                    batch.targets.behavior_side, batch.targets.behavior_side_mask,
                )
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
    required_ego_prefix = [
        "longitudinal_speed_mps",
        "lateral_speed_mps",
        "yaw_rate_rps",
    ]
    ego_features = list(data.get("ego_features", []))
    if ego_features not in (
        required_ego_prefix,
        [*required_ego_prefix, "actual_steering_rad"],
    ):
        raise ValueError("full-control config has an unsupported explicit ego feature order")
    if float(raw["loss"].get("current_control", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero current_control loss")
    if not bool(model.get("control_sequence_head_enabled")):
        raise ValueError("full-control training requires enabled control_sequence head")
    if int(model.get("control_sequence_steps", 0)) <= 0:
        raise ValueError("full-control training requires positive control_sequence_steps")
    if float(raw["loss"].get("control_sequence", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero control_sequence loss")
    if not bool(model.get("behavior_head_enabled")):
        raise ValueError("full-control config requires enabled behavior head")
    if int(model.get("behavior_classes", 0)) != 5 or int(model.get("behavior_sides", 0)) != 3:
        raise ValueError("full-control behavior head requires aic_behavior_v1 dimensions 5/3")
    if float(raw["loss"].get("behavior", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero behavior loss")
    if float(raw["loss"].get("behavior_side", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero behavior_side loss")
    if int(raw["training"].get("checkpoint_every_steps", 0)) <= 0:
        raise ValueError("full-control training requires positive checkpoint_every_steps")
    targets = raw.get("targets")
    expected_targets = {
        "behavior_ontology": BEHAVIOR_ONTOLOGY_V1,
        "behavior_classes": list(BEHAVIOR_CLASS_NAMES_V1),
        "behavior_sides": list(BEHAVIOR_SIDE_NAMES_V1),
    }
    if not isinstance(targets, dict) or any(targets.get(key) != value for key, value in expected_targets.items()):
        raise ValueError("full-control config behavior target ontology/order mismatch")
    if targets.get("command_history_alignment") != COMMAND_HISTORY_ALIGNMENT_V3:
        raise ValueError("full-control config requires causal command history alignment")
    command_history_length = int(data.get("command_history_length", 0))
    if not 0 < command_history_length <= int(model.get("max_ego_history", 0)):
        raise ValueError("command history length must fit max_ego_history")
    return raw


def build_full_control_model_v3(config: dict[str, object]) -> FullControlLiteV3:
    return FullControlLiteV3(**full_control_model_kwargs_v3(config))


def full_control_model_kwargs_v3(config: dict[str, object]) -> dict[str, object]:
    """Return the exact constructor contract embedded in runtime artifacts."""

    model = config["model"]
    data = config["data"]
    bounds = model["control_bounds"]
    return {
        "image_height": int(data["image_height"]),
        "image_width": int(data["image_width"]),
        "lidar_points": int(data["lidar_points"]),
        "ego_dim": len(data["ego_features"]),
        "hidden_dim": int(model["hidden_dim"]),
        "trajectory_steps": int(model["trajectory_steps"]),
        "candidates": int(model["candidates"]),
        "camera_tokens_hw": tuple(model["camera_tokens_hw"]),
        "lidar_tokens": int(model["lidar_tokens"]),
        "fusion_depth": int(model["fusion_depth"]),
        "fusion_heads": int(model["fusion_heads"]),
        "max_sensor_history": int(model["max_sensor_history"]),
        "max_ego_history": int(model["max_ego_history"]),
        "command_history_alignment": str(
            config["targets"]["command_history_alignment"]
        ),
        "control_head_enabled": True,
        "control_sequence_head_enabled": True,
        "control_sequence_steps": int(model["control_sequence_steps"]),
        "control_dt_sec": float(model["control_dt_sec"]),
        "max_steering_rad": float(bounds["max_steering_rad"]),
        "max_steering_rate_radps": float(bounds["max_steering_rate_radps"]),
        "max_speed_mps": float(bounds["max_speed_mps"]),
        "min_acceleration_mps2": float(bounds["min_acceleration_mps2"]),
        "max_acceleration_mps2": float(bounds["max_acceleration_mps2"]),
        "min_jerk_mps3": float(bounds["min_jerk_mps3"]),
        "max_jerk_mps3": float(bounds["max_jerk_mps3"]),
        "behavior_head_enabled": bool(model["behavior_head_enabled"]),
        "behavior_classes": int(model["behavior_classes"]),
        "behavior_sides": int(model["behavior_sides"]),
    }


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
            control_sequence=None if targets.control_sequence is None else targets.control_sequence.to(device),
            control_sequence_mask=None if targets.control_sequence_mask is None else targets.control_sequence_mask.to(device),
            behavior_class=None if targets.behavior_class is None else targets.behavior_class.to(device),
            behavior_mask=None if targets.behavior_mask is None else targets.behavior_mask.to(device),
            behavior_side=None if targets.behavior_side is None else targets.behavior_side.to(device),
            behavior_side_mask=None if targets.behavior_side_mask is None else targets.behavior_side_mask.to(device),
        )
    return ModelBatchV3(
        image=batch.image.to(device), image_mask=batch.image_mask.to(device),
        lidar=batch.lidar.to(device), lidar_mask=batch.lidar_mask.to(device),
        ego=batch.ego.to(device), ego_feature_mask=batch.ego_feature_mask.to(device),
        command_history=batch.command_history.to(device), command_mask=batch.command_mask.to(device),
        sensor_dt_sec=batch.sensor_dt_sec.to(device), targets=moved_targets,
        requested_outputs=batch.requested_outputs,
    )


def _add_masked_accuracy(
    row: dict[str, float],
    name: str,
    logits: torch.Tensor,
    target: torch.Tensor,
    mask: torch.Tensor | None,
) -> None:
    if mask is None or not bool(mask.any()):
        return
    correct = logits[mask].argmax(dim=1) == target[mask]
    row[f"metric/{name}_accuracy"] = float(correct.float().mean().detach().cpu())
