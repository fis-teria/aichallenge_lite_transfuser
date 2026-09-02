from __future__ import annotations

from dataclasses import dataclass
import math

import torch
from torch.nn import functional as F

from aic_transfuser_lite.contracts.model_batch_v3 import TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3


@dataclass(frozen=True)
class LossWeightsV3:
    trajectory: float = 1.0
    speed_profile: float = 1.0
    current_control: float = 0.0
    behavior: float = 0.0
    behavior_side: float = 0.0
    behavior_class_weights: tuple[float, ...] | None = None
    behavior_side_class_weights: tuple[float, ...] | None = None

    def validate(self) -> None:
        if any(value < 0.0 for value in (
            self.trajectory, self.speed_profile, self.current_control,
            self.behavior, self.behavior_side,
        )):
            raise ValueError("loss weights must be non-negative")
        for name, values, size in (
            ("behavior", self.behavior_class_weights, 5),
            ("behavior_side", self.behavior_side_class_weights, 3),
        ):
            if values is not None and (
                len(values) != size
                or any(not math.isfinite(value) or value <= 0.0 for value in values)
            ):
                raise ValueError(f"{name} class weights must contain {size} finite positive values")


@dataclass(frozen=True)
class LossReportV3:
    total: torch.Tensor
    raw: dict[str, torch.Tensor]
    weighted: dict[str, torch.Tensor]

    def scalar_log(self) -> dict[str, float]:
        values = {"loss/total": float(self.total.detach())}
        values.update({f"loss_raw/{key}": float(value.detach()) for key, value in self.raw.items()})
        values.update({f"loss_weighted/{key}": float(value.detach()) for key, value in self.weighted.items()})
        return values


def compute_losses_v3(
    output: ModelOutputV3, targets: TrainingTargetsV3, weights: LossWeightsV3
) -> LossReportV3:
    """Compute masked SI-unit losses for the K=1 V3 baseline."""
    weights.validate()
    if output.trajectory_xy.shape[1] != 1:
        raise ValueError("V3-014 loss supports candidates K=1")
    predicted_xy = output.trajectory_xy[:, 0]
    predicted_speed = output.trajectory_speed_mps[:, 0]
    if predicted_xy.shape != targets.trajectory_xy_m.shape:
        raise ValueError("trajectory prediction/target shape mismatch")
    if predicted_speed.shape != targets.speed_mps.shape:
        raise ValueError("speed prediction/target shape mismatch")
    trajectory = _masked_mean(
        torch.abs(predicted_xy - targets.trajectory_xy_m).mean(dim=-1),
        targets.trajectory_mask,
        "trajectory",
    )
    speed = _masked_mean(
        torch.abs(predicted_speed - targets.speed_mps), targets.speed_mask, "speed_profile"
    )
    raw = {"trajectory": trajectory, "speed_profile": speed}
    weighted = {
        "trajectory": trajectory * weights.trajectory,
        "speed_profile": speed * weights.speed_profile,
    }
    if weights.current_control > 0.0:
        if output.current_control is None:
            raise ValueError("current_control loss weight is nonzero but head output is absent")
        if targets.current_control is None or targets.current_control_mask is None:
            raise ValueError("current_control loss weight is nonzero but target is absent")
        prediction = output.current_control[:, 0]
        element_loss = torch.abs(prediction - targets.current_control)
        control = _masked_mean(element_loss, targets.current_control_mask, "current_control")
        raw["current_control"] = control
        weighted["current_control"] = control * weights.current_control
        if targets.control_provenance is None:
            raise ValueError("current control provenance is required")
        for provenance in ("nominal", "final_fallback"):
            rows = torch.tensor(
                [value == provenance for value in targets.control_provenance],
                dtype=torch.bool,
                device=prediction.device,
            )
            provenance_mask = targets.current_control_mask & rows.unsqueeze(1)
            if bool(provenance_mask.any()):
                raw[f"current_control_{provenance}"] = _masked_mean(
                    element_loss, provenance_mask, f"current_control_{provenance}"
                )
    if weights.behavior > 0.0:
        if output.behavior_logits is None or targets.behavior_class is None or targets.behavior_mask is None:
            raise ValueError("behavior loss is nonzero but output/target is absent")
        behavior = _masked_cross_entropy(
            output.behavior_logits, targets.behavior_class, targets.behavior_mask,
            weights.behavior_class_weights, "behavior",
        )
        raw["behavior"] = behavior
        weighted["behavior"] = behavior * weights.behavior
    if weights.behavior_side > 0.0:
        if output.behavior_side_logits is None or targets.behavior_side is None or targets.behavior_side_mask is None:
            raise ValueError("behavior_side loss is nonzero but output/target is absent")
        side = _masked_cross_entropy(
            output.behavior_side_logits, targets.behavior_side, targets.behavior_side_mask,
            weights.behavior_side_class_weights, "behavior_side",
        )
        raw["behavior_side"] = side
        weighted["behavior_side"] = side * weights.behavior_side
    total = sum(weighted.values())
    if not torch.isfinite(total):
        raise FloatingPointError("non-finite V3 loss")
    return LossReportV3(total=total, raw=raw, weighted=weighted)


def enforce_trajectory_regression_gate(
    *, candidate_ade_m: float, baseline_ade_m: float, max_relative_regression: float
) -> None:
    """Reject a control-auxiliary checkpoint whose trajectory ADE regresses too far."""
    values = (candidate_ade_m, baseline_ade_m, max_relative_regression)
    if not all(torch.isfinite(torch.tensor(value)) for value in values):
        raise ValueError("trajectory regression gate values must be finite")
    if candidate_ade_m < 0.0 or baseline_ade_m <= 0.0 or max_relative_regression < 0.0:
        raise ValueError("trajectory regression gate values are out of range")
    limit = baseline_ade_m * (1.0 + max_relative_regression)
    if candidate_ade_m > limit:
        raise RuntimeError(
            f"trajectory regression gate failed: candidate={candidate_ade_m:.6f}m limit={limit:.6f}m"
        )


def _masked_mean(values: torch.Tensor, mask: torch.Tensor, name: str) -> torch.Tensor:
    if values.shape != mask.shape or mask.dtype != torch.bool:
        raise ValueError(f"{name} values/mask mismatch")
    if not bool(mask.any()):
        raise ValueError(f"{name} target has no valid elements")
    selected = values[mask]
    if not torch.isfinite(selected).all():
        raise FloatingPointError(f"non-finite raw {name} loss")
    return selected.mean()


def _masked_cross_entropy(
    logits: torch.Tensor,
    target: torch.Tensor,
    mask: torch.Tensor,
    class_weights: tuple[float, ...] | None,
    name: str,
) -> torch.Tensor:
    if logits.ndim != 2 or target.shape != mask.shape or target.shape != logits.shape[:1]:
        raise ValueError(f"{name} logits/target/mask mismatch")
    if mask.dtype != torch.bool:
        raise ValueError(f"{name} mask must be boolean")
    if not bool(mask.any()):
        return logits.sum() * 0.0
    weight = None if class_weights is None else logits.new_tensor(class_weights)
    value = F.cross_entropy(logits[mask], target[mask], weight=weight)
    if not torch.isfinite(value):
        raise FloatingPointError(f"non-finite raw {name} loss")
    return value
