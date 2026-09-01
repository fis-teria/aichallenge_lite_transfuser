from __future__ import annotations

import torch

from aic_transfuser_lite.contracts.model_batch_v3 import TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3


def trajectory_metrics_v3(
    output: ModelOutputV3, targets: TrainingTargetsV3
) -> dict[str, float]:
    distance = torch.linalg.vector_norm(
        output.trajectory_xy[:, 0] - targets.trajectory_xy_m, dim=-1
    )
    mask = targets.trajectory_mask
    if not bool(mask.any()):
        raise ValueError("trajectory metrics require valid targets")
    ade = distance[mask].mean()
    last_indices = mask.long().sum(dim=1) - 1
    valid_rows = last_indices >= 0
    fde = distance[valid_rows, last_indices[valid_rows]].mean()
    if not torch.isfinite(ade) or not torch.isfinite(fde):
        raise FloatingPointError("non-finite trajectory metric")
    return {"trajectory_ade_m": float(ade), "trajectory_fde_m": float(fde)}
