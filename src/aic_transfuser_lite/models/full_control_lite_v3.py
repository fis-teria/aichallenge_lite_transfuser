from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import (
    COMMAND_HISTORY_ALIGNMENT_V3,
    ModelBatchV3,
)
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3

from .camera_encoder import CameraEncoder
from .ego_encoder import EgoEncoder
from .fusion import TokenFusionTransformer
from .heads.speed_profile import SpeedProfileHead
from .heads.trajectory import TrajectoryHead
from .heads.control_sequence import ControlSequenceHead, FutureControlSequenceHead
from .heads.behavior import BehaviorHeadV1
from .lidar_encoder import Lidar1DEncoder
from .encoders.ego_history import EgoHistoryEncoder
from .temporal.gru import MaskedGRUTemporalEncoder


@dataclass(frozen=True)
class WeightMigrationReport:
    loaded: tuple[str, ...]
    shape_mismatch: tuple[str, ...]
    unmapped_v1: tuple[str, ...]
    new_v3: tuple[str, ...]


class FullControlLiteV3(nn.Module):
    """Initial T=1, K=1 dense trajectory and speed-profile network."""

    def __init__(
        self,
        *,
        image_height: int,
        image_width: int,
        lidar_points: int,
        ego_dim: int,
        hidden_dim: int = 128,
        trajectory_steps: int = 15,
        candidates: int = 1,
        camera_tokens_hw: tuple[int, int] = (4, 4),
        lidar_tokens: int = 32,
        fusion_depth: int = 2,
        fusion_heads: int = 4,
        lidar_angle_min_rad: float = -3.141592653589793,
        lidar_angle_increment_rad: float | None = None,
        max_sensor_history: int = 4,
        max_ego_history: int = 10,
        command_history_alignment: str = COMMAND_HISTORY_ALIGNMENT_V3,
        control_head_enabled: bool = False,
        control_sequence_head_enabled: bool = False,
        control_sequence_steps: int = 10,
        control_dt_sec: float = 0.1,
        max_steering_rad: float = 0.6,
        max_steering_rate_radps: float = 0.8,
        max_speed_mps: float = 12.0,
        min_acceleration_mps2: float = -4.0,
        max_acceleration_mps2: float = 2.0,
        min_jerk_mps3: float = -8.0,
        max_jerk_mps3: float = 4.0,
        behavior_head_enabled: bool = False,
        behavior_classes: int = 5,
        behavior_sides: int = 3,
    ) -> None:
        super().__init__()
        if trajectory_steps != 15 or candidates != 1:
            raise ValueError("V3-013 baseline requires trajectory_steps=15 and candidates=1")
        if image_height <= 0 or image_width <= 0 or lidar_points <= 1 or ego_dim <= 0:
            raise ValueError("input dimensions must be positive")
        if command_history_alignment != COMMAND_HISTORY_ALIGNMENT_V3:
            raise ValueError(
                "FullControlLiteV3 requires causal_previous_only command history"
            )
        if lidar_angle_increment_rad is None:
            lidar_angle_increment_rad = 2.0 * torch.pi / float(lidar_points)
        self.image_height = image_height
        self.image_width = image_width
        self.lidar_points = lidar_points
        self.ego_dim = ego_dim
        self.trajectory_steps = trajectory_steps
        self.candidates = candidates
        self.control_head_enabled = control_head_enabled
        self.control_sequence_head_enabled = control_sequence_head_enabled
        self.control_sequence_steps = control_sequence_steps
        self.max_sensor_history = max_sensor_history
        self.max_ego_history = max_ego_history
        self.command_history_alignment = command_history_alignment
        self.camera = CameraEncoder(
            output_dim=hidden_dim,
            token_h=camera_tokens_hw[0],
            token_w=camera_tokens_hw[1],
            pretrained=False,
        )
        self.lidar = Lidar1DEncoder(
            output_dim=hidden_dim,
            token_count=lidar_tokens,
            input_channels=2,
            lidar_points=lidar_points,
            angle_min_rad=lidar_angle_min_rad,
            angle_increment_rad=float(lidar_angle_increment_rad),
            use_angle_encoding=True,
        )
        self.ego = EgoEncoder(input_dim=ego_dim, hidden_dim=hidden_dim, output_dim=hidden_dim)
        self.camera_temporal = MaskedGRUTemporalEncoder(hidden_dim, hidden_dim)
        self.lidar_temporal = MaskedGRUTemporalEncoder(hidden_dim, hidden_dim)
        self.ego_history = EgoHistoryEncoder(ego_dim, hidden_dim)
        self.temporal_projection = nn.Linear(hidden_dim * 3, hidden_dim)
        self.fusion = TokenFusionTransformer(
            dim=hidden_dim,
            image_tokens=self.camera.token_count,
            lidar_tokens=self.lidar.token_count,
            depth=fusion_depth,
            heads=fusion_heads,
            dropout=0.1,
            pooling="ego",
        )
        self.trajectory_head = TrajectoryHead(
            hidden_dim, candidates=candidates, steps=trajectory_steps
        )
        self.speed_profile_head = SpeedProfileHead(
            hidden_dim, candidates=candidates, steps=trajectory_steps
        )
        self.control_head = (
            ControlSequenceHead(
                hidden_dim,
                candidates=candidates,
                max_steering_rad=max_steering_rad,
                max_speed_mps=max_speed_mps,
                min_acceleration_mps2=min_acceleration_mps2,
                max_acceleration_mps2=max_acceleration_mps2,
            )
            if control_head_enabled
            else None
        )
        self.control_sequence_head = (
            FutureControlSequenceHead(
                hidden_dim,
                steps=control_sequence_steps,
                candidates=candidates,
                control_dt_sec=control_dt_sec,
                max_steering_rad=max_steering_rad,
                max_steering_rate_radps=max_steering_rate_radps,
                max_speed_mps=max_speed_mps,
                min_acceleration_mps2=min_acceleration_mps2,
                max_acceleration_mps2=max_acceleration_mps2,
                min_jerk_mps3=min_jerk_mps3,
                max_jerk_mps3=max_jerk_mps3,
            )
            if control_sequence_head_enabled
            else None
        )
        self.behavior_head = (
            BehaviorHeadV1(hidden_dim, classes=behavior_classes, sides=behavior_sides)
            if behavior_head_enabled
            else None
        )

    def forward(self, batch: ModelBatchV3) -> ModelOutputV3:
        batch.validate(require_current=True)
        if batch.image.shape[1] > self.max_sensor_history or batch.lidar.shape[1] > self.max_sensor_history:
            raise ValueError("Camera/LiDAR history exceeds configured maximum")
        if batch.ego.shape[1] > self.max_ego_history or batch.command_history.shape[1] > self.max_ego_history:
            raise ValueError("ego/command history exceeds configured maximum")
        image = batch.image[:, -1]
        lidar = batch.lidar[:, -1]
        ego = batch.ego[:, -1]
        if tuple(image.shape[1:]) != (3, self.image_height, self.image_width):
            raise ValueError("image shape does not match configured H/W")
        if tuple(lidar.shape[1:]) != (2, self.lidar_points):
            raise ValueError("LiDAR shape does not match configured point count")
        if ego.shape[1] != self.ego_dim:
            raise ValueError("ego feature dimension does not match configuration")
        batch_size, camera_steps = batch.image.shape[:2]
        camera_tokens_all = self.camera(
            batch.image.reshape(batch_size * camera_steps, *batch.image.shape[2:])
        ).view(batch_size, camera_steps, self.camera.token_count, -1)
        lidar_steps = batch.lidar.shape[1]
        lidar_tokens_all = self.lidar(
            batch.lidar.reshape(batch_size * lidar_steps, *batch.lidar.shape[2:])
        ).view(batch_size, lidar_steps, self.lidar.token_count, -1)
        camera_history = self.camera_temporal(camera_tokens_all.mean(dim=2), batch.image_mask)
        lidar_history = self.lidar_temporal(lidar_tokens_all.mean(dim=2), batch.lidar_mask)
        ego_history = self.ego_history(
            batch.ego,
            batch.ego_feature_mask,
            batch.command_history,
            batch.command_mask,
        )
        temporal = self.temporal_projection(
            torch.cat((camera_history, lidar_history, ego_history), dim=-1)
        )
        current_ego = self.ego(ego) + temporal.unsqueeze(1)
        _, pooled = self.fusion(
            camera_tokens_all[:, -1], lidar_tokens_all[:, -1], current_ego
        )
        trajectory, candidate_logits = self.trajectory_head(pooled)
        speed = self.speed_profile_head(pooled)
        current_control = None
        if "current_control" in batch.requested_outputs:
            if self.control_head is None:
                raise ValueError("current_control requested but control head is absent")
            current_control = self.control_head(pooled)
        control_sequence = None
        if "control_sequence" in batch.requested_outputs:
            if self.control_sequence_head is None:
                raise ValueError("control_sequence requested but sequence head is absent")
            latest_command = batch.command_history[:, -1]
            initial_acceleration = torch.where(
                batch.command_mask[:, -1],
                latest_command[:, 2],
                torch.zeros_like(latest_command[:, 2]),
            )
            initial_steering = (
                ego[:, 3] if self.ego_dim >= 4 else torch.zeros_like(ego[:, 0])
            )
            initial_control = torch.stack(
                (initial_steering, ego[:, 0], initial_acceleration), dim=-1
            )
            control_sequence = self.control_sequence_head(pooled, initial_control)
        behavior_logits = behavior_side_logits = None
        if "behavior" in batch.requested_outputs or "behavior_side" in batch.requested_outputs:
            if self.behavior_head is None:
                raise ValueError("behavior output requested but behavior head is absent")
            predicted_behavior, predicted_side = self.behavior_head(pooled)
            if "behavior" in batch.requested_outputs:
                behavior_logits = predicted_behavior
            if "behavior_side" in batch.requested_outputs:
                behavior_side_logits = predicted_side
        output = ModelOutputV3(
            trajectory_xy=trajectory,
            trajectory_speed_mps=speed,
            candidate_logits=candidate_logits,
            current_control=current_control,
            control_sequence=control_sequence,
            behavior_logits=behavior_logits,
            behavior_side_logits=behavior_side_logits,
        )
        output.validate(
            batch_size=batch.batch_size,
            candidates=self.candidates,
            trajectory_steps=self.trajectory_steps,
            requested_outputs=batch.requested_outputs,
        )
        return output

    def migrate_v1_weights(
        self, v1_state_dict: Mapping[str, torch.Tensor]
    ) -> WeightMigrationReport:
        """Load exact name/shape matches and report every non-migrated key."""
        current = self.state_dict()
        loaded: list[str] = []
        mismatch: list[str] = []
        unmapped: list[str] = []
        patch: dict[str, torch.Tensor] = {}
        for name, value in v1_state_dict.items():
            if name not in current:
                unmapped.append(name)
            elif current[name].shape != value.shape:
                mismatch.append(name)
            else:
                patch[name] = value
                loaded.append(name)
        self.load_state_dict(patch, strict=False)
        return WeightMigrationReport(
            loaded=tuple(sorted(loaded)),
            shape_mismatch=tuple(sorted(mismatch)),
            unmapped_v1=tuple(sorted(unmapped)),
            new_v3=tuple(sorted(set(current).difference(patch))),
        )
