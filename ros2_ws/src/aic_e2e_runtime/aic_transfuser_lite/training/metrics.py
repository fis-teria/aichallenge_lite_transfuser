from __future__ import annotations

from typing import Any

import torch


CURVATURE_BUCKET_NAMES = ("straight", "curve", "sharp")
DEFAULT_CONTROLLER_WHEELBASE_M = 1.087
CONTROLLER_MIN_LOOKAHEAD_M = 1.0
CONTROLLER_MAX_STEER_RAD = 0.6


def controller_proxy_steering(
    waypoints: torch.Tensor,
    *,
    wheelbase_m: float = DEFAULT_CONTROLLER_WHEELBASE_M,
) -> torch.Tensor:
    """Match the established 1 m pure-pursuit metric controller."""
    if waypoints.ndim != 3 or waypoints.shape[-1] != 2:
        raise ValueError(f"Expected waypoints [B,N,2], got {tuple(waypoints.shape)}")
    if not 0.0 < wheelbase_m < 10.0:
        raise ValueError("wheelbase_m must be within (0, 10) metres")
    distances = torch.linalg.vector_norm(waypoints, dim=-1)
    eligible = distances >= CONTROLLER_MIN_LOOKAHEAD_M
    first = eligible.to(torch.int64).argmax(dim=1)
    fallback = torch.full_like(first, waypoints.shape[1] - 1)
    indices = torch.where(eligible.any(dim=1), first, fallback)
    selected = waypoints[
        torch.arange(len(waypoints), device=waypoints.device), indices
    ]
    x = selected[:, 0]
    y = selected[:, 1]
    lookahead_sq = torch.clamp(x.square() + y.square(), min=1e-6)
    curvature = 2.0 * y / lookahead_sq
    return torch.atan(float(wheelbase_m) * curvature).clamp(
        -CONTROLLER_MAX_STEER_RAD, CONTROLLER_MAX_STEER_RAD
    )


def trajectory_max_abs_curvature(waypoints: torch.Tensor) -> torch.Tensor:
    """Estimate per-sample maximum three-point curvature including the origin."""
    if waypoints.ndim != 3 or waypoints.shape[-1] != 2:
        raise ValueError(f"Expected waypoints [B,N,2], got {tuple(waypoints.shape)}")
    if waypoints.shape[1] < 2:
        return torch.zeros(
            waypoints.shape[0], device=waypoints.device, dtype=waypoints.dtype
        )
    origin = torch.zeros(
        waypoints.shape[0], 1, 2, device=waypoints.device, dtype=waypoints.dtype
    )
    points = torch.cat((origin, waypoints), dim=1)
    segment_a = points[:, 1:-1] - points[:, :-2]
    segment_b = points[:, 2:] - points[:, 1:-1]
    chord = points[:, 2:] - points[:, :-2]
    denominator = (
        torch.linalg.vector_norm(segment_a, dim=-1)
        * torch.linalg.vector_norm(segment_b, dim=-1)
        * torch.linalg.vector_norm(chord, dim=-1)
    )
    cross = (
        segment_a[..., 0] * segment_b[..., 1]
        - segment_a[..., 1] * segment_b[..., 0]
    )
    curvature = torch.where(
        denominator > 1e-9,
        2.0 * cross / torch.clamp(denominator, min=1e-9),
        torch.zeros_like(cross),
    )
    return curvature.abs().amax(dim=1)


def curvature_bucket_indices(
    waypoints: torch.Tensor,
    *,
    straight_threshold_per_m: float,
    sharp_threshold_per_m: float,
) -> torch.Tensor:
    if not 0.0 < straight_threshold_per_m < sharp_threshold_per_m:
        raise ValueError("Curvature thresholds must satisfy 0 < straight < sharp")
    curvature = trajectory_max_abs_curvature(waypoints)
    return torch.where(
        curvature < straight_threshold_per_m,
        torch.zeros_like(curvature, dtype=torch.long),
        torch.where(
            curvature < sharp_threshold_per_m,
            torch.ones_like(curvature, dtype=torch.long),
            torch.full_like(curvature, 2, dtype=torch.long),
        ),
    )


class V1MetricAccumulator:
    """Accumulate gate-aligned metrics without retaining prediction tensors."""

    def __init__(
        self,
        *,
        num_waypoints: int,
        straight_threshold_per_m: float = 0.03,
        sharp_threshold_per_m: float = 0.12,
        controller_wheelbase_m: float = DEFAULT_CONTROLLER_WHEELBASE_M,
    ) -> None:
        if num_waypoints <= 0:
            raise ValueError("num_waypoints must be positive")
        if not 0.0 < straight_threshold_per_m < sharp_threshold_per_m:
            raise ValueError("Curvature thresholds must satisfy 0 < straight < sharp")
        self.num_waypoints = int(num_waypoints)
        self.straight_threshold_per_m = float(straight_threshold_per_m)
        self.sharp_threshold_per_m = float(sharp_threshold_per_m)
        if not 0.0 < controller_wheelbase_m < 10.0:
            raise ValueError("controller_wheelbase_m must be within (0, 10) metres")
        self.controller_wheelbase_m = float(controller_wheelbase_m)
        self.count = 0
        self.ade_sum = 0.0
        self.fde_sum = 0.0
        self.horizon_distance_sum = [0.0] * self.num_waypoints
        self.speed_mae_sum = 0.0
        self.control_abs_sum = 0.0
        self.control_bias_sum = 0.0
        self.bucket: dict[str, dict[str, float | int]] = {
            name: {
                "sample_count": 0,
                "ade_sum": 0.0,
                "control_abs_sum": 0.0,
                "control_bias_sum": 0.0,
            }
            for name in CURVATURE_BUCKET_NAMES
        }

    @torch.no_grad()
    def update(
        self,
        outputs: dict[str, torch.Tensor],
        batch: dict[str, torch.Tensor],
    ) -> None:
        predicted = outputs["waypoints"]
        target = batch["waypoints"]
        if predicted.shape != target.shape:
            raise ValueError(
                f"Predicted/target waypoint shapes differ: {predicted.shape} vs {target.shape}"
            )
        if predicted.shape[1] != self.num_waypoints:
            raise ValueError("Waypoint count differs from metric contract")
        batch_size = int(predicted.shape[0])
        distance = torch.linalg.vector_norm(predicted - target, dim=-1)
        ade_per_sample = distance.mean(dim=1)
        predicted_control = controller_proxy_steering(
            predicted, wheelbase_m=self.controller_wheelbase_m
        )
        target_control = controller_proxy_steering(
            target, wheelbase_m=self.controller_wheelbase_m
        )
        control_error = predicted_control - target_control
        bucket_indices = curvature_bucket_indices(
            target,
            straight_threshold_per_m=self.straight_threshold_per_m,
            sharp_threshold_per_m=self.sharp_threshold_per_m,
        )

        self.count += batch_size
        self.ade_sum += float(ade_per_sample.sum().detach().cpu())
        self.fde_sum += float(distance[:, -1].sum().detach().cpu())
        horizon_sum = distance.sum(dim=0).detach().cpu().tolist()
        self.horizon_distance_sum = [
            previous + float(value)
            for previous, value in zip(self.horizon_distance_sum, horizon_sum)
        ]
        self.speed_mae_sum += float(
            (outputs["target_speed"] - batch["target_speed"])
            .abs()
            .mean(dim=1)
            .sum()
            .detach()
            .cpu()
        )
        self.control_abs_sum += float(control_error.abs().sum().detach().cpu())
        self.control_bias_sum += float(control_error.sum().detach().cpu())

        for index, name in enumerate(CURVATURE_BUCKET_NAMES):
            selected = bucket_indices == index
            selected_count = int(selected.sum().detach().cpu())
            if selected_count == 0:
                continue
            values = self.bucket[name]
            values["sample_count"] = int(values["sample_count"]) + selected_count
            values["ade_sum"] = float(values["ade_sum"]) + float(
                ade_per_sample[selected].sum().detach().cpu()
            )
            values["control_abs_sum"] = float(values["control_abs_sum"]) + float(
                control_error[selected].abs().sum().detach().cpu()
            )
            values["control_bias_sum"] = float(values["control_bias_sum"]) + float(
                control_error[selected].sum().detach().cpu()
            )

    def finalize(self) -> dict[str, Any]:
        if self.count <= 0:
            raise ValueError("No samples were accumulated")
        buckets: dict[str, dict[str, float | int | None]] = {}
        for name in CURVATURE_BUCKET_NAMES:
            values = self.bucket[name]
            count = int(values["sample_count"])
            buckets[name] = {
                "sample_count": count,
                "ade_m": float(values["ade_sum"]) / count if count else None,
                "controller_proxy_mae_rad": (
                    float(values["control_abs_sum"]) / count if count else None
                ),
                "controller_proxy_bias_rad": (
                    float(values["control_bias_sum"]) / count if count else None
                ),
            }
        return {
            "sample_count": self.count,
            "ade_m": self.ade_sum / self.count,
            "fde_m": self.fde_sum / self.count,
            "waypoint_horizon_mae_m": [
                value / self.count for value in self.horizon_distance_sum
            ],
            "speed_mae_mps": self.speed_mae_sum / self.count,
            "controller_proxy_mae_rad": self.control_abs_sum / self.count,
            "controller_proxy_bias_rad": self.control_bias_sum / self.count,
            "curvature_thresholds_per_m": {
                "straight_upper": self.straight_threshold_per_m,
                "sharp_lower": self.sharp_threshold_per_m,
            },
            "controller_proxy_contract": {
                "type": "pure_pursuit",
                "wheelbase_m": self.controller_wheelbase_m,
                "min_lookahead_m": CONTROLLER_MIN_LOOKAHEAD_M,
                "max_steer_rad": CONTROLLER_MAX_STEER_RAD,
            },
            "curvature_buckets": buckets,
        }
