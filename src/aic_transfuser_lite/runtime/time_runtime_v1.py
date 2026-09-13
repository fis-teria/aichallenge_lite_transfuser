"""Teacher-free online boundary for the trained 30 x 2 time model.

Capture uses simulator nanoseconds; availability uses local monotonic nanoseconds.
The latter replaces the offline bag-receipt proxy explicitly, never numerically
mixing clock domains. Input selection/materialization is shared with training.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, fields, replace
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_inputs
from aic_transfuser_lite.data.time_history_v1 import TimeEvent
from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TIME_CHECKPOINT_FORMAT, TimeCheckpointIdentity, load_time_checkpoint,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

FREEZE_DELAY_NS = 50_000_000
ROLES = frozenset({"camera", "lidar", "velocity", "actual_steering"})


@dataclass(frozen=True)
class FrozenTimeInput:
    anchor: TimeEvent
    events: tuple[TimeEvent, ...]
    epoch_start_ns: int
    epoch_end_ns: int
    freeze_ns: int

    def assemble(self, config: TimeDatasetConfig) -> tuple[ModelBatchV3, tuple[dict[str, Any], ...]]:
        return assemble_time_inputs(self.events, self.anchor, config=config,
                                    epoch_start_ns=self.epoch_start_ns,
                                    epoch_end_ns=self.epoch_end_ns, freeze_ns=self.freeze_ns)


class TimeInputBuffer:
    """Single ROS ingestion thread; immutable snapshots handed to one worker.

    Late samples never enter an already-frozen input. Bounded role queues retain
    duplicate captures for the shared availability-first resolution. Clock reset
    clears pending anchors/history; a worker result carries its original epoch.
    """

    def __init__(self, config: TimeModelConfig, *, run_id: str) -> None:
        config.validate()
        if config.use_command_history or not run_id:
            raise ValueError("runtime v1 requires explicit command-OFF model and run ID")
        self.config = replace(config.dataset_config(), available_clock="monotonic")
        self.run_id = run_id
        self.epoch_index = 0
        self.clock_ns: int | None = None
        self.epoch_start_ns = 0
        self.sequence = 0
        self.events: dict[str, deque[TimeEvent]] = {r: deque(maxlen=256 if r != "camera" else 32) for r in ROLES}
        self.pending: deque[TimeEvent] = deque(maxlen=8)
        self.dropped_anchors = 0

    @property
    def epoch(self) -> str:
        return str(self.epoch_index)

    def on_clock(self, stamp_ns: int) -> bool:
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError("invalid simulator clock")
        reset = self.clock_ns is not None and stamp_ns < self.clock_ns
        if self.clock_ns is None or reset:
            self.epoch_start_ns = stamp_ns
        if reset:
            self.epoch_index += 1
            for queue in self.events.values():
                queue.clear()
            self.pending.clear()
        self.clock_ns = stamp_ns
        return reset

    def add(self, role: str, payload: Any, *, received_ns: int) -> TimeEvent:
        if role not in ROLES or self.clock_ns is None:
            raise ValueError("UNSUPPORTED_ROLE_OR_NO_CLOCK")
        if type(received_ns) is not int or received_ns < 0:
            raise ValueError("invalid monotonic receipt")
        capture_ns = payload.timestamp_ns
        if type(capture_ns) is not int or not self.epoch_start_ns <= capture_ns <= self.clock_ns + 50_000_000:
            raise ValueError("CAPTURE_OUTSIDE_CURRENT_EPOCH")
        self.sequence += 1
        event = TimeEvent(role, self.run_id, self.epoch, self.config.capture_clock,
                          "monotonic", capture_ns, received_ns, self.sequence, payload,
                          "ros_callback_monotonic_receipt")
        self.events[role].append(event)
        if role == "camera":
            if len(self.pending) == self.pending.maxlen:
                self.dropped_anchors += 1
            self.pending.append(event)
        return event

    def freeze_latest(self, now_ns: int) -> FrozenTimeInput | None:
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("invalid monotonic now")
        due = [e for e in self.pending if e.available_ns + FREEZE_DELAY_NS <= now_ns]
        if not due:
            return None
        # Acquisitions can arrive out of order. Never prefer an older capture just
        # because it was the last receipt. A worker may skip anchors, never time slots.
        anchor = max(due, key=lambda e: (e.capture_ns, e.available_ns, e.sequence))
        removed = [e for e in self.pending if e.capture_ns <= anchor.capture_ns]
        self.dropped_anchors += max(0, len(removed) - 1)
        self.pending = deque((e for e in self.pending if e.capture_ns > anchor.capture_ns), maxlen=8)
        freeze = anchor.available_ns + FREEZE_DELAY_NS
        events = tuple(e for queue in self.events.values() for e in queue
                       if e.available_ns <= freeze and e.epoch == anchor.epoch)
        return FrozenTimeInput(anchor, events, self.epoch_start_ns,
                               max(self.clock_ns or 0, anchor.capture_ns), freeze)


class TimeRuntimeModel:
    """Hash-bound trusted-local checkpoint, strict config/state, FP32 inference."""

    def __init__(self, checkpoint: Path, *, expected_sha256: str, device: str) -> None:
        if len(expected_sha256) != 64 or any(c not in "0123456789abcdef" for c in expected_sha256):
            raise ValueError("explicit lowercase checkpoint SHA-256 required")
        digest = hashlib.sha256()
        with checkpoint.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != expected_sha256:
            raise ValueError("CHECKPOINT_SHA_MISMATCH")
        payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
        if not isinstance(payload, dict) or payload.get("format") != TIME_CHECKPOINT_FORMAT:
            raise ValueError("TIME_CHECKPOINT_REQUIRED")
        self.config = TimeModelConfig.from_dict(payload["config"])
        if self.config.use_command_history:
            raise ValueError("runtime v1 supports the selected command-OFF arm")
        contract = payload.get("teacher_manifest", {}).get("contract", {})
        if (contract.get("freeze_delay_receipt_ns") != FREEZE_DELAY_NS
                or contract.get("frame") != "base_link_at_observation"
                or contract.get("points") != 30 or contract.get("dt_s") != .1):
            raise ValueError("TEACHER_RUNTIME_CONTRACT_MISMATCH")
        self.identity = TimeCheckpointIdentity(**payload["identity"])
        self.epoch = int(payload["epoch"])
        self.sha256 = expected_sha256
        del payload
        self.device = torch.device(device)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        self.model = build_time_model(self.config)
        load_time_checkpoint(checkpoint, config=self.config, identity=self.identity,
                             model=self.model, mode="finetune")
        self.model.to(self.device).eval()

    def predict(self, batch: ModelBatchV3) -> np.ndarray:
        """Finite float32 [30,2] metres in observation base_link, no targets."""
        if batch.targets is not None:
            raise ValueError("RUNTIME_TEACHER_INPUT_FORBIDDEN")
        batch.validate(require_current=False)
        moved = replace(batch, **{f.name: getattr(batch, f.name).to(self.device)
                                  for f in fields(batch) if isinstance(getattr(batch, f.name), torch.Tensor)})
        with torch.inference_mode():
            result = self.model(moved).float().cpu().numpy()
        if result.shape != (1, 30, 2) or not np.isfinite(result).all():
            raise ValueError("INVALID_TIME_PREDICTION")
        return result[0].copy()


def decode_ros_image(message: Any) -> np.ndarray:
    """Decode RGB/BGR/RGBA/BGRA uint8 ROS Image, honoring padded row stride."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(message.encoding)
    if channels is None or message.width <= 0 or message.height <= 0:
        raise ValueError("UNSUPPORTED_IMAGE_ENCODING_OR_SHAPE")
    if message.step < message.width * channels or len(message.data) != message.height * message.step:
        raise ValueError("IMAGE_STRIDE_OR_SIZE_MISMATCH")
    data = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.step)
    rgb = data[:, :message.width * channels].reshape(message.height, message.width, channels)[:, :, :3]
    if message.encoding in {"bgr8", "bgra8"}:
        rgb = rgb[:, :, ::-1]
    return rgb.copy()
