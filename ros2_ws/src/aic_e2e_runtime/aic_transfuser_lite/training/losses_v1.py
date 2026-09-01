from __future__ import annotations

from typing import Any

import torch
from torch.nn import functional as F


def normalized_horizon_weights(
    values: list[float], *, device: torch.device, dtype: torch.dtype
) -> torch.Tensor:
    weights = torch.tensor(values, device=device, dtype=dtype)
    if weights.ndim != 1 or len(weights) == 0 or not torch.all(weights > 0):
        raise ValueError("waypoint horizon weights must be a positive one-dimensional list")
    return weights / weights.mean()


def target_relative_shape_loss(
    predicted: torch.Tensor, target: torch.Tensor
) -> torch.Tensor:
    if predicted.shape != target.shape:
        raise ValueError(
            f"predicted/target waypoint shapes differ: {predicted.shape} vs {target.shape}"
        )
    if predicted.shape[1] < 3:
        return predicted.new_zeros(())
    predicted_second = (
        predicted[:, 2:] - 2.0 * predicted[:, 1:-1] + predicted[:, :-2]
    )
    target_second = target[:, 2:] - 2.0 * target[:, 1:-1] + target[:, :-2]
    return F.smooth_l1_loss(predicted_second, target_second, beta=0.5)


def compute_v1_multitask_loss(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, torch.Tensor],
    weights: dict[str, Any],
    horizon_weights: list[float],
) -> tuple[
    torch.Tensor,
    dict[str, torch.Tensor],
    dict[str, torch.Tensor],
]:
    """Compute the versioned v1 objective and expose raw/weighted components."""
    predicted_waypoints = outputs["waypoints"]
    target_waypoints = batch["waypoints"]
    if predicted_waypoints.shape != target_waypoints.shape:
        raise ValueError(
            "predicted/target waypoint shapes differ: "
            f"{predicted_waypoints.shape} vs {target_waypoints.shape}"
        )
    horizon = normalized_horizon_weights(
        horizon_weights,
        device=predicted_waypoints.device,
        dtype=predicted_waypoints.dtype,
    )
    elementwise_waypoint = F.smooth_l1_loss(
        predicted_waypoints,
        target_waypoints,
        beta=0.5,
        reduction="none",
    )
    raw: dict[str, torch.Tensor] = {
        "waypoint": (elementwise_waypoint.mean(dim=-1) * horizon.unsqueeze(0)).mean(),
        "speed": F.smooth_l1_loss(
            outputs["target_speed"], batch["target_speed"], beta=0.5
        ),
        "shape": target_relative_shape_loss(predicted_waypoints, target_waypoints),
    }

    optional = (
        ("stop", "stop_logit", "stop"),
        ("mode", "mode_logits", "mode"),
        ("direct_control", "direct_control", "direct_control"),
    )
    for loss_name, output_name, batch_name in optional:
        configured_weight = float(weights[loss_name])
        if output_name not in outputs:
            if configured_weight != 0.0:
                raise ValueError(
                    f"{loss_name} loss weight is non-zero but its Head is disabled"
                )
            continue
        if loss_name == "stop":
            raw[loss_name] = F.binary_cross_entropy_with_logits(
                outputs[output_name], batch[batch_name]
            )
        elif loss_name == "mode":
            raw[loss_name] = F.cross_entropy(
                outputs[output_name], batch[batch_name]
            )
        else:
            raw[loss_name] = F.smooth_l1_loss(
                outputs[output_name], batch[batch_name], beta=0.5
            )

    weighted = {
        name: value * float(weights[name])
        for name, value in raw.items()
    }
    total = predicted_waypoints.new_zeros(())
    for value in weighted.values():
        total = total + value
    return total, raw, weighted
