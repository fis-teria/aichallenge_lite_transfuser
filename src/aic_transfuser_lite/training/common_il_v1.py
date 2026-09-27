"""Masked, ROS-independent IL updates and a distinct checkpoint format."""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from pathlib import Path
from typing import Any, Sequence

import torch
from torch import nn
from torch.nn import functional as F

from aic_transfuser_lite.contracts.common_il_v1 import CommonILPrediction, CommonILTargets, common_il_contract
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.models.common_il_v1 import CommonILV1
from .time_config_v1 import TimeModelConfig

CHECKPOINT_FORMAT = "aic_common_il_checkpoint_v1"


@dataclass(frozen=True)
class CommonILLossWeights:
    waypoint: float = 1.0
    speed: float = 1.0
    stop: float = 1.0

    def __post_init__(self) -> None:
        if (any(type(v) is not float or not math.isfinite(v) or v < 0 for v in vars(self).values())
                or not any(vars(self).values())):
            raise ValueError("IL weights must be finite nonnegative floats with nonzero sum")


def collate_common_il(samples: Sequence[TimeSample], device: torch.device) -> tuple[ModelBatchV3, CommonILTargets]:
    """Use valid-input samples only; label masks retain independent support.

    XY selects indices 4,9,14,19,24,29 of the existing 0.1 s teacher grid.
    Speed is measured longitudinal velocity at 0.5 s (index 4), not chord speed.
    Explicit environment_stop_intent is the only source of stop labels.
    """
    if not samples or any(s.inputs is None or s.inputs.targets is not None for s in samples):
        raise ValueError("nonempty input-only valid policy samples required")
    kwargs = {f.name: torch.cat([getattr(s.inputs, f.name) for s in samples], dim=0).to(device)
              for f in fields(ModelBatchV3) if isinstance(getattr(samples[0].inputs, f.name), torch.Tensor)}
    inputs = ModelBatchV3(**kwargs, targets=None, requested_outputs=frozenset({"trajectory"}))
    xy, xy_mask, speed, speed_mask, stop, stop_mask = [], [], [], [], [], []
    for sample in samples:
        teacher = sample.teacher
        xy.append(torch.from_numpy(teacher.xy_m[4::5].copy()) if teacher is not None else torch.full((6, 2), float("nan")))
        xy_mask.append(torch.from_numpy(teacher.xy_mask[4::5].copy()) if teacher is not None else torch.zeros(6, dtype=torch.bool))
        valid_speed = teacher is not None and bool(teacher.velocity_mask[4])
        speed.append([float(teacher.velocity_mps[4]) if valid_speed else 0.0])
        speed_mask.append([valid_speed])
        intent = sample.environment_stop_intent
        if intent is not None and type(intent) is not bool:
            raise ValueError("environment stop intent must be bool or unknown")
        stop.append([float(intent or False)])
        stop_mask.append([intent is not None])
    targets = CommonILTargets(torch.stack(xy).to(device), torch.stack(xy_mask).to(device),
        torch.tensor(speed, dtype=torch.float32, device=device), torch.tensor(speed_mask, dtype=torch.bool, device=device),
        torch.tensor(stop, dtype=torch.float32, device=device), torch.tensor(stop_mask, dtype=torch.bool, device=device))
    inputs.validate(require_current=False)
    targets.validate()
    return inputs, targets


def target_support(targets: CommonILTargets) -> dict[str, int]:
    targets.validate()
    return {"waypoint": int(targets.waypoint_mask.any(1).sum().item()),
            "speed": int(targets.target_speed_mask.sum().item()), "stop": int(targets.stop_mask.sum().item()),
            "stop_positive": int((targets.stop_target[targets.stop_mask] == 1).sum().item())}


def common_il_loss(prediction: CommonILPrediction, targets: CommonILTargets,
                   weights: CommonILLossWeights = CommonILLossWeights()) -> tuple[torch.Tensor | None, dict[str, Any]]:
    """Per-head supported-sample means: XY L1 [m], speed L1 [m/s], stop BCE.

    Unsupported/disabled heads contribute no graph, so AdamW cannot update
    their parameters using weight decay or old momentum on a zero gradient.
    """
    prediction.validate()
    targets.validate()
    if (prediction.waypoints_m.shape != targets.waypoints_m.shape
            or prediction.waypoints_m.device != targets.waypoints_m.device):
        raise ValueError("prediction/teacher batch or device mismatch")
    support = target_support(targets)
    components: dict[str, torch.Tensor | None] = {"waypoint": None, "speed": None, "stop": None}
    if weights.waypoint and support["waypoint"]:
        mask = targets.waypoint_mask
        safe_prediction = torch.where(mask[..., None], prediction.waypoints_m.float(), 0.0)
        safe_target = torch.where(mask[..., None], targets.waypoints_m.float(), 0.0)
        errors = (safe_prediction - safe_target).abs().mean(-1)
        components["waypoint"] = (errors.sum(1) / mask.sum(1).clamp_min(1))[mask.any(1)].mean()
    if weights.speed and support["speed"]:
        mask = targets.target_speed_mask
        components["speed"] = F.l1_loss(prediction.target_speed_mps.float()[mask], targets.target_speed_mps.float()[mask])
    if weights.stop and support["stop"]:
        mask = targets.stop_mask
        components["stop"] = F.binary_cross_entropy_with_logits(prediction.stop_logit.float()[mask], targets.stop_target.float()[mask])
    terms = [getattr(weights, name) * value for name, value in components.items() if value is not None]
    loss = sum(terms) if terms else None
    return loss, {"support": support, "loss": {k: float(v.detach()) if v is not None else None for k, v in components.items()},
                  "total": float(loss.detach()) if loss is not None else None}


def train_common_il_batch(model: CommonILV1, samples: Sequence[TimeSample], optimizer: torch.optim.Optimizer,
                          *, weights: CommonILLossWeights = CommonILLossWeights(), max_grad_norm: float = 1.0) -> dict[str, Any]:
    if not math.isfinite(max_grad_norm) or max_grad_norm <= 0:
        raise ValueError("max_grad_norm must be finite and positive")
    optimizer.zero_grad(set_to_none=True)
    valid = [sample for sample in samples if sample.inputs is not None]
    result: dict[str, Any] = {"visited": len(samples), "invalid_inputs": len(samples) - len(valid), "updated": False}
    if not valid:
        return result
    inputs, targets = collate_common_il(valid, next(model.parameters()).device)
    support = target_support(targets)
    result["support"] = support
    if not any(support[name] and weight for name, weight in asdict(weights).items()):
        return result  # No forward, including no BatchNorm running-state update.
    model.train()
    loss, metrics = common_il_loss(model(inputs), targets, weights)
    if loss is None or not torch.isfinite(loss):
        raise ValueError("invalid supported common IL loss")
    loss.backward()
    norm = nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm, error_if_nonfinite=True)
    optimizer.step()
    return {**result, **metrics, "updated": True, "gradient_norm_before_clip": float(norm)}


def save_common_il_checkpoint(path: Path, model: CommonILV1, *, metadata: dict[str, Any]) -> None:
    """A diagnostic artifact, not a runtime deployment or resumable optimizer."""
    import json
    json.dumps(metadata, allow_nan=False)
    payload = {"format": CHECKPOINT_FORMAT, "contract": common_il_contract(),
               "model_config": model.config.to_dict(), "initialization_seed": model.initialization_seed,
               "model_state_dict": model.state_dict(), "metadata": metadata,
               "camera_provenance": model.backbone.camera.pretrained_provenance()}
    with path.open("xb") as stream:
        torch.save(payload, stream)


def load_common_il_checkpoint(path: Path, *, device: str = "cpu") -> tuple[CommonILV1, dict[str, Any]]:
    payload = torch.load(path, map_location="cpu", weights_only=True)
    if payload.get("format") != CHECKPOINT_FORMAT or payload.get("contract") != common_il_contract():
        raise ValueError("not a common IL checkpoint")
    model = CommonILV1(TimeModelConfig.from_dict(payload["model_config"]), initialization_seed=payload["initialization_seed"])
    model.load_state_dict(payload["model_state_dict"], strict=True)
    return model.to(device).eval(), payload["metadata"]
