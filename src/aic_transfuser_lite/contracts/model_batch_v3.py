from __future__ import annotations

from dataclasses import dataclass

import torch


MODEL_BATCH_FORMAT_V3 = "aic_model_batch_v3"
KNOWN_REQUESTED_OUTPUTS = frozenset(
    {
        "trajectory",
        "speed_profile",
        "trajectory_log_sigma",
        "stop",
        "risk",
        "behavior",
        "current_control",
        "control_sequence",
    }
)


@dataclass(frozen=True)
class TrainingTargetsV3:
    trajectory_xy_m: torch.Tensor
    trajectory_mask: torch.Tensor
    speed_mps: torch.Tensor
    speed_mask: torch.Tensor
    current_control: torch.Tensor | None = None
    current_control_mask: torch.Tensor | None = None
    control_provenance: tuple[str, ...] | None = None

    def validate(self, *, batch_size: int) -> None:
        if self.trajectory_xy_m.ndim != 3 or self.trajectory_xy_m.shape[0] != batch_size or self.trajectory_xy_m.shape[-1] != 2:
            raise ValueError("trajectory target must be [B,N,2]")
        expected = self.trajectory_xy_m.shape[:2]
        if self.trajectory_mask.shape != expected or self.trajectory_mask.dtype != torch.bool:
            raise ValueError("trajectory mask must be bool [B,N]")
        if self.speed_mps.shape != expected or self.speed_mask.shape != expected:
            raise ValueError("speed target and mask must be [B,N]")
        if self.speed_mask.dtype != torch.bool:
            raise ValueError("speed mask must be boolean")
        _finite_where(self.trajectory_xy_m, self.trajectory_mask.unsqueeze(-1), "trajectory target")
        _finite_where(self.speed_mps, self.speed_mask, "speed target")
        if self.current_control is None:
            if self.current_control_mask is not None or self.control_provenance is not None:
                raise ValueError("control mask/provenance require current_control target")
        else:
            if self.current_control.shape != (batch_size, 3):
                raise ValueError("current_control target must be [B,3]")
            if self.current_control_mask is None or self.current_control_mask.shape != (batch_size, 3) or self.current_control_mask.dtype != torch.bool:
                raise ValueError("current_control_mask must be bool [B,3]")
            _finite_where(self.current_control, self.current_control_mask, "current control target")
            if self.control_provenance is None or len(self.control_provenance) != batch_size:
                raise ValueError("control provenance must contain one entry per batch item")


@dataclass(frozen=True)
class ModelBatchV3:
    """Typed temporal batch using SI units and explicit validity masks."""

    image: torch.Tensor
    image_mask: torch.Tensor
    lidar: torch.Tensor
    lidar_mask: torch.Tensor
    ego: torch.Tensor
    ego_feature_mask: torch.Tensor
    command_history: torch.Tensor
    command_mask: torch.Tensor
    sensor_dt_sec: torch.Tensor
    targets: TrainingTargetsV3 | None = None
    requested_outputs: frozenset[str] = frozenset({"trajectory", "speed_profile"})

    def validate(self, *, require_current: bool = True) -> None:
        if self.image.ndim != 5 or self.image.shape[2] != 3:
            raise ValueError("image must be [B,Tc,3,H,W]")
        if self.lidar.ndim != 4 or self.lidar.shape[2] != 2:
            raise ValueError("lidar must be [B,Tl,2,P]")
        if self.ego.ndim != 3 or self.command_history.ndim != 3 or self.command_history.shape[2] != 3:
            raise ValueError("ego must be [B,Te,Fe] and command_history [B,Tu,3]")
        batch = self.image.shape[0]
        if any(value.shape[0] != batch for value in (self.lidar, self.ego, self.command_history, self.sensor_dt_sec)):
            raise ValueError("all ModelBatchV3 tensors must share batch dimension")
        masks = (
            (self.image_mask, self.image.shape[:2], "image_mask"),
            (self.lidar_mask, self.lidar.shape[:2], "lidar_mask"),
            (self.ego_feature_mask, self.ego.shape, "ego_feature_mask"),
            (self.command_mask, self.command_history.shape[:2], "command_mask"),
        )
        for mask, shape, name in masks:
            if mask.shape != shape or mask.dtype != torch.bool:
                raise ValueError(f"{name} must be boolean with shape {tuple(shape)}")
        if self.sensor_dt_sec.ndim != 3 or self.sensor_dt_sec.shape[:2] != self.image.shape[:2]:
            raise ValueError("sensor_dt_sec must be [B,Tc,M]")
        for name, tensor in (
            ("image", self.image),
            ("lidar", self.lidar),
            ("ego", self.ego),
            ("command_history", self.command_history),
            ("sensor_dt_sec", self.sensor_dt_sec),
        ):
            if not torch.isfinite(tensor).all():
                raise ValueError(f"{name} must be finite; missing values belong in masks")
        if require_current:
            if not bool(self.image_mask[:, -1].all()) or not bool(self.lidar_mask[:, -1].all()):
                raise ValueError("current Camera and LiDAR steps must be valid")
            if not bool(self.ego_feature_mask[:, -1].all()):
                raise ValueError("current ego features must be valid")
        unknown = sorted(self.requested_outputs.difference(KNOWN_REQUESTED_OUTPUTS))
        if unknown:
            raise ValueError(f"unknown requested outputs: {unknown}")
        if "trajectory" not in self.requested_outputs:
            raise ValueError("trajectory is a mandatory V3 co-output")
        if self.targets is not None:
            self.targets.validate(batch_size=batch)

    @property
    def batch_size(self) -> int:
        return int(self.image.shape[0])


def _finite_where(tensor: torch.Tensor, mask: torch.Tensor, name: str) -> None:
    expanded = mask.expand_as(tensor)
    if not torch.isfinite(tensor[expanded]).all():
        raise ValueError(f"{name} contains non-finite valid values")
