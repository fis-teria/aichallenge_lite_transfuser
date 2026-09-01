from __future__ import annotations

from typing import Any

import torch
from torch.nn import functional as F


def waypoint_smoothness(waypoints: torch.Tensor) -> torch.Tensor:
    if waypoints.shape[1] < 3:
        return waypoints.new_zeros(())
    second = waypoints[:, 2:] - 2.0 * waypoints[:, 1:-1] + waypoints[:, :-2]
    return second.abs().mean()


def compute_multitask_loss(
    outputs: dict[str, torch.Tensor],
    batch: dict[str, torch.Tensor],
    weights: dict[str, Any],
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    parts: dict[str, torch.Tensor] = {}
    parts["waypoint"] = F.smooth_l1_loss(outputs["waypoints"], batch["waypoints"])
    parts["speed"] = F.smooth_l1_loss(outputs["target_speed"], batch["target_speed"])
    parts["smoothness"] = waypoint_smoothness(outputs["waypoints"])

    stop_weight = float(weights.get("stop", 0.0))
    if "stop_logit" in outputs:
        if stop_weight != 0.0:
            parts["stop"] = F.binary_cross_entropy_with_logits(
                outputs["stop_logit"], batch["stop"]
            )
    elif stop_weight != 0.0:
        raise ValueError("stop loss weight is non-zero but stop Head is disabled")

    mode_weight = float(weights.get("mode", 0.0))
    if "mode_logits" in outputs:
        if mode_weight != 0.0:
            parts["mode"] = F.cross_entropy(outputs["mode_logits"], batch["mode"])
    elif mode_weight != 0.0:
        raise ValueError("mode loss weight is non-zero but behavior-mode Head is disabled")

    direct_weight = float(weights.get("direct_control", 0.0))
    if "direct_control" in outputs:
        if direct_weight != 0.0:
            parts["direct_control"] = F.smooth_l1_loss(
                outputs["direct_control"], batch["direct_control"]
            )
    elif direct_weight != 0.0:
        raise ValueError(
            "direct-control loss weight is non-zero but direct-control Head is disabled"
        )

    total = outputs["waypoints"].new_zeros(())
    for name, value in parts.items():
        total = total + float(weights.get(name, 0.0)) * value
    return total, parts
