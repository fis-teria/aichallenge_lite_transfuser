from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3

from .train_v3 import move_batch_v3


@dataclass(frozen=True)
class OfflineMetricsV3:
    batch_count: int
    sample_count: int
    trajectory_ade_m: float
    speed_mae_mps: float
    current_control_mae: tuple[float, float, float] | None
    control_sequence_mae: tuple[float, float, float] | None

    def to_dict(self) -> dict[str, object]:
        return {
            "batch_count": self.batch_count,
            "sample_count": self.sample_count,
            "trajectory_ade_m": self.trajectory_ade_m,
            "speed_mae_mps": self.speed_mae_mps,
            "current_control_mae": self.current_control_mae,
            "control_sequence_mae": self.control_sequence_mae,
            "control_order": ("steering_rad", "speed_mps", "acceleration_mps2"),
        }


def evaluate_offline_v3(
    model: torch.nn.Module,
    batches: Sequence[ModelBatchV3],
    *,
    device: torch.device,
) -> OfflineMetricsV3:
    """Compute masked SI-unit metrics without changing model parameters."""

    if not batches:
        raise ValueError("offline evaluation requires at least one batch")
    trajectory_sum = speed_sum = 0.0
    trajectory_count = speed_count = 0
    current_sum = torch.zeros(3, dtype=torch.float64)
    current_count = torch.zeros(3, dtype=torch.long)
    sequence_sum = torch.zeros(3, dtype=torch.float64)
    sequence_count = torch.zeros(3, dtype=torch.long)
    sample_count = 0
    was_training = model.training
    model.eval()
    try:
        with torch.inference_mode():
            for source in batches:
                batch = move_batch_v3(source, device)
                if batch.targets is None:
                    raise ValueError("offline evaluation batch has no targets")
                output = model(batch)
                target = batch.targets
                points = torch.linalg.vector_norm(
                    output.trajectory_xy[:, 0] - target.trajectory_xy_m, dim=-1
                )
                trajectory_sum += float(points[target.trajectory_mask].sum().cpu())
                trajectory_count += int(target.trajectory_mask.sum().cpu())
                speed_error = torch.abs(output.trajectory_speed_mps[:, 0] - target.speed_mps)
                speed_sum += float(speed_error[target.speed_mask].sum().cpu())
                speed_count += int(target.speed_mask.sum().cpu())
                if output.current_control is not None and target.current_control is not None:
                    mask = target.current_control_mask
                    if mask is None:
                        raise ValueError("current control target mask is absent")
                    error = torch.abs(output.current_control[:, 0] - target.current_control)
                    current_sum += (error * mask).sum(dim=0).double().cpu()
                    current_count += mask.sum(dim=0).long().cpu()
                if output.control_sequence is not None and target.control_sequence is not None:
                    mask = target.control_sequence_mask
                    if mask is None:
                        raise ValueError("control sequence target mask is absent")
                    error = torch.abs(output.control_sequence[:, 0] - target.control_sequence)
                    sequence_sum += (error * mask).sum(dim=(0, 1)).double().cpu()
                    sequence_count += mask.sum(dim=(0, 1)).long().cpu()
                sample_count += batch.batch_size
    finally:
        model.train(was_training)
    if trajectory_count == 0 or speed_count == 0:
        raise ValueError("offline evaluation has no valid trajectory/speed targets")
    return OfflineMetricsV3(
        batch_count=len(batches),
        sample_count=sample_count,
        trajectory_ade_m=trajectory_sum / trajectory_count,
        speed_mae_mps=speed_sum / speed_count,
        current_control_mae=_per_dimension(current_sum, current_count),
        control_sequence_mae=_per_dimension(sequence_sum, sequence_count),
    )


def _per_dimension(
    total: torch.Tensor, count: torch.Tensor
) -> tuple[float, float, float] | None:
    if not bool((count > 0).any()):
        return None
    if not bool((count > 0).all()):
        raise ValueError("control metric has a dimension with no valid target")
    return tuple(float(value) for value in total / count)  # type: ignore[return-value]
