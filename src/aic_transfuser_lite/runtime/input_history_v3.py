from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.runtime.residual_control import ExternalControllerCommand


@dataclass(frozen=True)
class RuntimeObservationTensorV3:
    """One synchronized runtime observation before temporal left padding.

    ``image`` is ``[3,H,W]``, ``lidar`` is ``[2,P]``, ``ego`` is ``[F]`` in
    SI units, and ``sensor_dt_sec`` is ``[camera age, LiDAR-Camera]`` in
    seconds. ``stamp_sec`` is the Camera anchor timestamp and must increase
    strictly within one runtime segment.
    """

    stamp_sec: float
    image: torch.Tensor
    lidar: torch.Tensor
    ego: torch.Tensor
    sensor_dt_sec: torch.Tensor

    def validate(self) -> None:
        if not math.isfinite(float(self.stamp_sec)) or self.stamp_sec < 0.0:
            raise ValueError("runtime observation stamp must be finite and non-negative")
        if self.image.ndim != 3 or self.image.shape[0] != 3:
            raise ValueError("runtime observation image must be [3,H,W]")
        if self.lidar.ndim != 2 or self.lidar.shape[0] != 2:
            raise ValueError("runtime observation LiDAR must be [2,P]")
        if self.ego.ndim != 1:
            raise ValueError("runtime observation ego must be [F]")
        if self.sensor_dt_sec.ndim != 1:
            raise ValueError("runtime observation sensor_dt_sec must be [M]")
        for name, value in (
            ("image", self.image),
            ("lidar", self.lidar),
            ("ego", self.ego),
            ("sensor_dt_sec", self.sensor_dt_sec),
        ):
            if not torch.isfinite(value).all():
                raise ValueError(f"runtime observation {name} must be finite")


class RuntimeObservationHistoryV3:
    """Stateful, segment-safe runtime observation history.

    Timestamp regression or a gap larger than ``maximum_gap_sec`` begins a new
    segment. The current observation remains valid while older history is
    discarded. The caller must clear causal command history at the same time.
    """

    def __init__(self, *, maximum_length: int, maximum_gap_sec: float) -> None:
        if maximum_length <= 0:
            raise ValueError("runtime observation history length must be positive")
        if not math.isfinite(maximum_gap_sec) or maximum_gap_sec <= 0.0:
            raise ValueError("runtime observation maximum gap must be finite and positive")
        self.maximum_length = int(maximum_length)
        self.maximum_gap_sec = float(maximum_gap_sec)
        self._values: deque[RuntimeObservationTensorV3] = deque(
            maxlen=self.maximum_length
        )

    @property
    def values(self) -> tuple[RuntimeObservationTensorV3, ...]:
        return tuple(self._values)

    def reset(self) -> None:
        self._values.clear()

    def append(self, observation: RuntimeObservationTensorV3) -> str | None:
        observation.validate()
        reset_reason = None
        if self._values:
            delta = float(observation.stamp_sec) - float(self._values[-1].stamp_sec)
            if delta <= 0.0:
                reset_reason = "timestamp_non_monotonic"
            elif delta > self.maximum_gap_sec:
                reset_reason = "timestamp_gap"
            if reset_reason is not None:
                self.reset()
        self._values.append(observation)
        return reset_reason


def _left_padded_observations(
    values: Sequence[RuntimeObservationTensorV3], *, length: int
) -> tuple[tuple[RuntimeObservationTensorV3, ...], tuple[bool, ...]]:
    if length <= 0:
        raise ValueError("runtime temporal history length must be positive")
    if not values:
        raise ValueError("runtime temporal history requires a current observation")
    selected = tuple(values[-length:])
    for left, right in zip(selected, selected[1:]):
        if right.stamp_sec <= left.stamp_sec:
            raise ValueError("runtime temporal observations must be strictly ordered")
    pad = length - len(selected)
    return (
        tuple([selected[0]] * pad + list(selected)),
        tuple([False] * pad + [True] * len(selected)),
    )


def build_runtime_temporal_batch_v3(
    observations: Sequence[RuntimeObservationTensorV3],
    commands_before_anchor: Sequence[ExternalControllerCommand],
    *,
    sensor_history_length: int,
    ego_history_length: int,
    command_history_length: int,
    requested_outputs: frozenset[str],
) -> ModelBatchV3:
    """Build the exact temporal inference contract with past-only commands."""

    sensor_values, sensor_mask = _left_padded_observations(
        observations, length=sensor_history_length
    )
    ego_values, ego_mask = _left_padded_observations(
        observations, length=ego_history_length
    )
    if command_history_length <= 0:
        raise ValueError("runtime command history length must be positive")
    selected_commands = tuple(commands_before_anchor[-command_history_length:])
    encoded_commands: list[tuple[float, float, float]] = []
    for command in selected_commands:
        values = (
            float(command.steering_rad),
            float(command.speed_mps),
            float(command.acceleration_mps2),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("runtime command history must be finite")
        if values[1] < 0.0:
            raise ValueError("runtime command history speed must be non-negative")
        encoded_commands.append(values)
    command_pad = command_history_length - len(encoded_commands)
    encoded_commands = [(0.0, 0.0, 0.0)] * command_pad + encoded_commands
    command_mask = [False] * command_pad + [True] * len(selected_commands)

    device = sensor_values[-1].image.device
    batch = ModelBatchV3(
        image=torch.stack([item.image for item in sensor_values])[None],
        image_mask=torch.tensor(sensor_mask, dtype=torch.bool, device=device)[None],
        lidar=torch.stack([item.lidar for item in sensor_values])[None],
        lidar_mask=torch.tensor(sensor_mask, dtype=torch.bool, device=device)[None],
        ego=torch.stack([item.ego for item in ego_values])[None],
        ego_feature_mask=torch.tensor(
            ego_mask, dtype=torch.bool, device=device
        )[None, :, None].expand(-1, -1, ego_values[-1].ego.shape[0]),
        command_history=torch.tensor(
            encoded_commands, dtype=torch.float32, device=device
        )[None],
        command_mask=torch.tensor(
            command_mask, dtype=torch.bool, device=device
        )[None],
        sensor_dt_sec=torch.stack(
            [item.sensor_dt_sec for item in sensor_values]
        )[None],
        requested_outputs=requested_outputs,
    )
    batch.validate(require_current=True)
    return batch
