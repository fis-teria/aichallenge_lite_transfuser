"""Training-only future observations and applied actions for temporal learning.

The policy input remains :class:`ModelBatchV3` inside ``TimeSample``.  Future
observations/actions live in a sibling field and therefore cannot be passed to
the policy accidentally through the existing model batch contract.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import torch
from torch.utils.data import Dataset

from .image_preprocess import preprocess_image
from .mcap_converter_v2 import (
    TimedCommand,
    TimedImage,
    TimedLidar,
    TimedSteering,
    TimedVelocity,
)
from .time_dataset_v1 import (
    TimeDatasetConfig,
    TimeSample,
    _lidar,
    assemble_time_sample,
)
from .time_history_v1 import TimeEvent


FORMAT = "future_sequence_cache_v1"


@dataclass(frozen=True)
class FutureSequenceConfig:
    """Future target layout; horizons are seconds after observation time."""

    horizons_sec: tuple[float, ...] = (0.5, 1.0)
    tolerance_ns: int = 50_000_000
    preprocess_version: str = "time_preprocess_v1"
    encoder_identity: str = "raw_sensor_targets_v1"
    augmentation_identity: str = "disabled_for_cached_targets"

    def __post_init__(self) -> None:
        if (
            type(self.horizons_sec) is not tuple
            or not self.horizons_sec
            or any(type(value) is not float or not math.isfinite(value) or value <= 0.0 for value in self.horizons_sec)
            or tuple(sorted(set(self.horizons_sec))) != self.horizons_sec
        ):
            raise ValueError("future horizons must be unique increasing positive floats")
        if type(self.tolerance_ns) is not int or self.tolerance_ns < 0:
            raise ValueError("future tolerance must be a non-negative integer ns")
        if not all(
            type(value) is str and value
            for value in (
                self.preprocess_version,
                self.encoder_identity,
                self.augmentation_identity,
            )
        ):
            raise ValueError("future cache identities must be non-empty strings")


@dataclass(frozen=True)
class FutureSequenceTargets:
    """Training-only future tensors.

    Shapes are ``image [F,3,H,W]``, ``lidar [F,2,P]``, ``ego [F,4]`` and
    ``applied_action [F,3]``.  Ego units are m/s, m/s, rad/s and rad.  Applied
    action units are steering rad, speed m/s and acceleration m/s2.
    """

    horizons_sec: torch.Tensor
    image: torch.Tensor
    image_mask: torch.Tensor
    lidar: torch.Tensor
    lidar_mask: torch.Tensor
    ego: torch.Tensor
    ego_mask: torch.Tensor
    applied_action: torch.Tensor
    applied_action_mask: torch.Tensor
    source_timestamp_ns: torch.Tensor

    def validate(self, *, image_shape: tuple[int, int, int], lidar_shape: tuple[int, int]) -> None:
        count = int(self.horizons_sec.shape[0])
        expected = {
            "horizons_sec": ((count,), torch.float32),
            "image": ((count, *image_shape), torch.float32),
            "image_mask": ((count,), torch.bool),
            "lidar": ((count, *lidar_shape), torch.float32),
            "lidar_mask": ((count,), torch.bool),
            "ego": ((count, 4), torch.float32),
            "ego_mask": ((count, 4), torch.bool),
            "applied_action": ((count, 3), torch.float32),
            "applied_action_mask": ((count,), torch.bool),
            "source_timestamp_ns": ((count, 4), torch.int64),
        }
        for name, (shape, dtype) in expected.items():
            value = getattr(self, name)
            if tuple(value.shape) != shape or value.dtype != dtype:
                raise ValueError(f"future target {name} shape/dtype mismatch")
            if value.is_floating_point() and not torch.isfinite(value).all():
                raise ValueError(f"future target {name} must be finite")


@dataclass(frozen=True)
class TemporalTrainingSample:
    """Policy sample plus a strictly training-only future-target sibling."""

    policy: TimeSample
    future: FutureSequenceTargets
    target_speed_mps: torch.Tensor
    target_speed_mask: torch.Tensor
    stop_target: torch.Tensor
    stop_mask: torch.Tensor

    def validate(self) -> None:
        for name in ("target_speed_mps", "stop_target"):
            value = getattr(self, name)
            if value.shape != (1,) or value.dtype != torch.float32 or not torch.isfinite(value).all():
                raise ValueError(f"{name} must be finite float32 [1]")
        for name in ("target_speed_mask", "stop_mask"):
            value = getattr(self, name)
            if value.shape != (1,) or value.dtype != torch.bool:
                raise ValueError(f"{name} must be bool [1]")
        if self.stop_mask.item() and not 0.0 <= self.stop_target.item() <= 1.0:
            raise ValueError("valid stop target must be in [0,1]")


def _nearest(
    events: Sequence[TimeEvent],
    anchor: TimeEvent,
    role: str,
    target_ns: int,
    tolerance_ns: int,
    epoch_bounds: tuple[int, int],
) -> TimeEvent | None:
    if not epoch_bounds[0] <= target_ns <= epoch_bounds[1]:
        return None
    candidates = [
        event
        for event in events
        if event.role == role
        and (event.run, event.epoch, event.capture_clock)
        == (anchor.run, anchor.epoch, anchor.capture_clock)
        and epoch_bounds[0] <= event.capture_ns <= epoch_bounds[1]
        and abs(event.capture_ns - target_ns) <= tolerance_ns
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda event: (abs(event.capture_ns - target_ns), -event.available_ns, -event.sequence))


def build_future_sequence_targets(
    events: Sequence[TimeEvent],
    anchor: TimeEvent,
    *,
    dataset_config: TimeDatasetConfig,
    future_config: FutureSequenceConfig,
    epoch_bounds: tuple[int, int],
) -> FutureSequenceTargets:
    """Build future targets without applying the observation-time availability cut."""

    if anchor.role != "camera" or not epoch_bounds[0] <= anchor.capture_ns <= epoch_bounds[1]:
        raise ValueError("future anchor must be a camera inside its run epoch")
    count = len(future_config.horizons_sec)
    image = np.zeros((count, *dataset_config.image_shape), dtype=np.float32)
    lidar = np.zeros((count, *dataset_config.lidar_shape), dtype=np.float32)
    ego = np.zeros((count, 4), dtype=np.float32)
    action = np.zeros((count, 3), dtype=np.float32)
    image_mask = np.zeros(count, dtype=bool)
    lidar_mask = np.zeros(count, dtype=bool)
    ego_mask = np.zeros((count, 4), dtype=bool)
    action_mask = np.zeros(count, dtype=bool)
    stamps = np.full((count, 4), -1, dtype=np.int64)
    for index, horizon_sec in enumerate(future_config.horizons_sec):
        target_ns = anchor.capture_ns + int(round(horizon_sec * 1_000_000_000.0))
        selected = {
            role: _nearest(events, anchor, role, target_ns, future_config.tolerance_ns, epoch_bounds)
            for role in ("camera", "lidar", "velocity", "final_command")
        }
        camera = selected["camera"]
        if camera is not None and isinstance(camera.payload, TimedImage):
            value = preprocess_image(
                camera.payload.image_rgb,
                height=dataset_config.image_shape[1],
                width=dataset_config.image_shape[2],
            ).numpy()
            if np.isfinite(value).all():
                image[index] = value
                image_mask[index] = True
                stamps[index, 0] = camera.capture_ns
        scan = selected["lidar"]
        if scan is not None and isinstance(scan.payload, TimedLidar):
            try:
                value = _lidar(scan.payload, dataset_config)
            except ValueError:
                value = None
            if value is not None and np.isfinite(value).all():
                lidar[index] = value
                lidar_mask[index] = True
                stamps[index, 1] = scan.capture_ns
        velocity = selected["velocity"]
        if velocity is not None and isinstance(velocity.payload, TimedVelocity):
            values = np.asarray(
                [
                    velocity.payload.longitudinal_mps,
                    velocity.payload.lateral_mps,
                    velocity.payload.yaw_rate_rps,
                ],
                dtype=np.float32,
            )
            valid = np.isfinite(values)
            ego[index, :3] = np.where(valid, values, 0.0)
            ego_mask[index, :3] = valid
            stamps[index, 2] = velocity.capture_ns
        steering = _nearest(
            events,
            anchor,
            "actual_steering",
            target_ns,
            future_config.tolerance_ns,
            epoch_bounds,
        )
        if steering is not None and isinstance(steering.payload, TimedSteering):
            value = float(steering.payload.steering_rad)
            if math.isfinite(value):
                ego[index, 3] = value
                ego_mask[index, 3] = True
        command = selected["final_command"]
        if command is not None and isinstance(command.payload, TimedCommand):
            values = np.asarray(
                [command.payload.steering_rad, command.payload.speed_mps, command.payload.acceleration_mps2],
                dtype=np.float32,
            )
            if np.isfinite(values).all():
                action[index] = values
                action_mask[index] = True
                stamps[index, 3] = command.capture_ns
    result = FutureSequenceTargets(
        torch.tensor(future_config.horizons_sec, dtype=torch.float32),
        torch.from_numpy(image),
        torch.from_numpy(image_mask),
        torch.from_numpy(lidar),
        torch.from_numpy(lidar_mask),
        torch.from_numpy(ego),
        torch.from_numpy(ego_mask),
        torch.from_numpy(action),
        torch.from_numpy(action_mask),
        torch.from_numpy(stamps),
    )
    result.validate(image_shape=dataset_config.image_shape, lidar_shape=dataset_config.lidar_shape)
    return result


def build_temporal_training_sample(
    events: Sequence[TimeEvent],
    anchor: TimeEvent,
    *,
    dataset_config: TimeDatasetConfig,
    future_config: FutureSequenceConfig,
    epoch_bounds: tuple[int, int],
    freeze_ns: int,
    environment_stop_intent: bool | None = None,
) -> TemporalTrainingSample:
    """Build policy inputs and future teachers through independent branches."""

    policy = assemble_time_sample(
        events,
        anchor,
        config=dataset_config,
        epoch_start_ns=epoch_bounds[0],
        epoch_end_ns=epoch_bounds[1],
        freeze_ns=freeze_ns,
        environment_stop_intent=environment_stop_intent,
    )
    future = build_future_sequence_targets(
        events,
        anchor,
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds=epoch_bounds,
    )
    speed = torch.zeros(1, dtype=torch.float32)
    speed_mask = torch.zeros(1, dtype=torch.bool)
    if policy.teacher is not None:
        offset_index = 4  # 0.5 s on the fixed 0.1 s TimeTeacher grid.
        if bool(policy.teacher.velocity_mask[offset_index]):
            value = float(policy.teacher.velocity_mps[offset_index])
            if math.isfinite(value):
                speed[0] = value
                speed_mask[0] = True
    stop = torch.tensor([float(environment_stop_intent or False)], dtype=torch.float32)
    stop_mask = torch.tensor([environment_stop_intent is not None], dtype=torch.bool)
    result = TemporalTrainingSample(policy, future, speed, speed_mask, stop, stop_mask)
    result.validate()
    return result


def cache_identity(
    *,
    source_sha256: str,
    dataset_config: TimeDatasetConfig,
    future_config: FutureSequenceConfig,
) -> dict[str, Any]:
    """Return a stable identity that invalidates on source or preprocessing change."""

    if len(source_sha256) != 64 or any(character not in "0123456789abcdef" for character in source_sha256):
        raise ValueError("source_sha256 must be lowercase SHA-256")
    value: dict[str, Any] = {
        "format": FORMAT,
        "source_sha256": source_sha256,
        "dataset_config": asdict(dataset_config),
        "future_config": asdict(future_config),
    }
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    value["identity_sha256"] = hashlib.sha256(encoded).hexdigest()
    return value


class FutureSequenceCache:
    """Small immutable per-anchor NPZ cache under one verified identity."""

    def __init__(self, root: Path, identity: dict[str, Any]) -> None:
        self.root = root.resolve()
        expected = dict(identity)
        digest = expected.pop("identity_sha256", None)
        encoded = json.dumps(expected, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if digest != hashlib.sha256(encoded).hexdigest() or expected.get("format") != FORMAT:
            raise ValueError("future cache identity mismatch")
        self.identity = identity
        identity_path = self.root / "identity.json"
        if identity_path.exists():
            if json.loads(identity_path.read_text(encoding="utf-8")) != identity:
                raise ValueError("future cache root belongs to another source/config")
        else:
            self.root.mkdir(parents=True, exist_ok=True)
            identity_path.write_text(json.dumps(identity, indent=2, allow_nan=False), encoding="utf-8")
        (self.root / "records").mkdir(exist_ok=True)

    def get_or_build(
        self,
        anchor_id: str,
        builder: Callable[[], FutureSequenceTargets],
    ) -> tuple[FutureSequenceTargets, bool]:
        """Return ``(targets, reused)``; a partial write never replaces a record."""

        if type(anchor_id) is not str or not anchor_id:
            raise ValueError("anchor_id is required")
        key = hashlib.sha256(anchor_id.encode("utf-8")).hexdigest()
        path = self.root / "records" / f"{key}.npz"
        if path.exists():
            with np.load(path, allow_pickle=False) as bundle:
                return _targets_from_arrays({name: bundle[name].copy() for name in bundle.files}), True
        targets = builder()
        arrays = {name: getattr(targets, name).cpu().numpy() for name in targets.__dataclass_fields__}
        temporary = path.with_suffix(".partial.npz")
        if temporary.exists():
            raise FileExistsError(f"partial future cache exists: {temporary}")
        np.savez_compressed(temporary, **arrays)
        temporary.replace(path)
        return targets, False


def _targets_from_arrays(arrays: dict[str, np.ndarray]) -> FutureSequenceTargets:
    expected = tuple(FutureSequenceTargets.__dataclass_fields__)
    if set(arrays) != set(expected):
        raise ValueError("future cache record fields mismatch")
    return FutureSequenceTargets(*(torch.from_numpy(arrays[name]) for name in expected))


class TemporalTrainingDataset(Dataset[TemporalTrainingSample]):
    """Dataset wrapper whose future fields are never inserted into ModelBatchV3."""

    def __init__(
        self,
        records: Sequence[tuple[Sequence[TimeEvent], TimeEvent, int]],
        *,
        dataset_config: TimeDatasetConfig,
        future_config: FutureSequenceConfig,
        epoch_bounds: dict[tuple[str, str], tuple[int, int]],
        stop_intent: dict[str, bool] | None = None,
    ) -> None:
        self._records = tuple(records)
        self._dataset_config = dataset_config
        self._future_config = future_config
        self._bounds = dict(epoch_bounds)
        self._stop_intent = dict(stop_intent or {})

    def __len__(self) -> int:
        return len(self._records)

    def __getitem__(self, index: int) -> TemporalTrainingSample:
        events, anchor, freeze_ns = self._records[index]
        key = (anchor.run, anchor.epoch)
        if key not in self._bounds:
            raise ValueError(f"epoch bounds missing for {key}")
        return build_temporal_training_sample(
            events,
            anchor,
            dataset_config=self._dataset_config,
            future_config=self._future_config,
            epoch_bounds=self._bounds[key],
            freeze_ns=freeze_ns,
            environment_stop_intent=self._stop_intent.get(f"{anchor.run}:{anchor.epoch}:{anchor.capture_ns}"),
        )
