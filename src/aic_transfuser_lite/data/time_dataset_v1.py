"""Shared causal inputs and separate full-run teachers for the time baseline.

Pure decoded-event logic; ROS, bag I/O and large corpus materialization stay outside
Dataset workers. Camera capture and available-clock freeze are separate values.
"""
from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np
import torch
from torch.utils.data import Dataset

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .canonical_converter_v3 import _interpolate_pose_indexed
from .synchronization_v3 import IndexedTimedValues, TimedValue
from .mcap_converter_v2 import TimedImage, TimedLidar, TimedPose, TimedVelocity, TimedCommand
from .image_preprocess import preprocess_image
from .normalization import normalize_lidar_range_and_validity
from .time_history_v1 import TimeEvent, materialize_history, select_time_history
from .time_teacher_v1 import TimeTeacher, build_time_teacher


@dataclass(frozen=True)
class TimeDatasetConfig:
    image_shape: tuple[int, int, int] = (3, 224, 384)
    lidar_shape: tuple[int, int] = (2, 750)
    ego_features: int = 4
    camera_history_length: int = 4
    lidar_history_length: int = 4
    ego_history_length: int = 10
    command_history_length: int = 10
    tolerance_ns: int = 50_000_000
    teacher_tolerance_ms: float = 50.0
    lidar_min_range_m: float = 0.0
    lidar_max_range_m: float = 25.0
    lidar_angle_min_rad: float = -math.pi
    capture_clock: str = "sim"
    available_clock: str = "bag_receipt"
    pose_world_frame: str = "map"
    pose_body_frame: str = "base_link"

    def __post_init__(self) -> None:
        for value, size in ((self.image_shape, 3), (self.lidar_shape, 2)):
            if type(value) is not tuple or len(value) != size or any(type(x) is not int or x <= 0 for x in value):
                raise ValueError("invalid time input shape")
        if self.image_shape[0] != 3 or self.lidar_shape[0] != 2 or self.lidar_shape[1] < 2:
            raise ValueError("expected RGB CHW and range/validity [2,P]")
        if type(self.ego_features) is not int or self.ego_features != 4:
            raise ValueError("ego schema is speed, lateral speed, yaw rate, measured steering")
        for length in (self.camera_history_length, self.lidar_history_length,
                       self.ego_history_length, self.command_history_length):
            if type(length) is not int or not 1 <= length <= 10:
                raise ValueError("history length must be integer in [1,10]")
        if type(self.tolerance_ns) is not int or self.tolerance_ns < 0:
            raise ValueError("history tolerance must be nonnegative integer ns")
        for v in (self.teacher_tolerance_ms, self.lidar_min_range_m,
                  self.lidar_max_range_m, self.lidar_angle_min_rad):
            if type(v) is not float or not math.isfinite(v):
                raise ValueError("preprocessing numeric settings must be finite floats")
        if self.teacher_tolerance_ms < 0 or not 0 <= self.lidar_min_range_m < self.lidar_max_range_m:
            raise ValueError("invalid tolerance/range bounds")
        if not all(type(x) is str and x for x in (self.capture_clock, self.available_clock, self.pose_world_frame)):
            raise ValueError("explicit clock and frame identities required")
        if self.pose_body_frame != "base_link":
            raise ValueError("time model labels require base_link; rear axle conversion is downstream")


@dataclass(frozen=True)
class TimeSample:
    """One camera anchor, retained even when input or teacher cannot be constructed."""
    inputs: ModelBatchV3 | None
    teacher: TimeTeacher | None
    run: str
    anchor_id: str
    observation_ns: int
    input_invalid_reason: str | None = None
    teacher_reasons: tuple[str, ...] = ()
    provenance: tuple[dict[str, Any], ...] = ()
    stop_reason: str = "UNKNOWN"
    stop_probability: None = None
    freeze_ns: int | None = None
    environment_stop_intent: bool | None = None
    observed_stationary: bool | None = None


def anchor_identity(anchor: TimeEvent) -> str:
    return f"{anchor.run}:{anchor.epoch}:{anchor.capture_ns}"


def _eligible(events: Sequence[TimeEvent], anchor: TimeEvent, config: TimeDatasetConfig,
              freeze_ns: int, bounds: tuple[int, int]) -> tuple[TimeEvent, ...]:
    # Deduplicate AFTER the availability cut and domain/epoch/bounds selection.
    selected: dict[tuple[str, int], TimeEvent] = {}
    for e in events:
        if ((e.run, e.epoch, e.capture_clock, e.available_clock) !=
                (anchor.run, anchor.epoch, config.capture_clock, config.available_clock)
                or e.available_ns > freeze_ns or not bounds[0] <= e.capture_ns <= bounds[1]):
            continue
        key = (e.role, e.capture_ns)
        old = selected.get(key)
        if old is None or (e.available_ns, e.sequence) > (old.available_ns, old.sequence):
            selected[key] = e
    return tuple(selected.values())


def _refs(role: str, slot: int, events: Sequence[TimeEvent]) -> dict[str, Any]:
    return {"role": role, "slot_ns": slot, "sources": [
        {"capture_ns": e.capture_ns, "available_ns": e.available_ns,
         "sequence": e.sequence, "capture_clock": e.capture_clock,
         "available_clock": e.available_clock, "availability_source": e.availability_source}
        for e in events]}


def _endpoints(events: Sequence[TimeEvent], role: str, stamp: int, tolerance_ns: int) -> tuple[TimeEvent, ...]:
    rows = sorted((e for e in events if e.role == role), key=lambda e: e.capture_ns)
    stamps = [e.capture_ns for e in rows]
    i = bisect_left(stamps, stamp)
    if i < len(rows) and stamps[i] == stamp:
        return (rows[i],)
    if i == 0 or i == len(rows) or stamp-stamps[i-1] > tolerance_ns or stamps[i]-stamp > tolerance_ns:
        return ()
    return (rows[i-1], rows[i])


def _pose_anchor(events: Sequence[TimeEvent], camera: TimeEvent, config: TimeDatasetConfig) -> tuple[TimeEvent, dict[str, Any]]:
    endpoints = _endpoints(events, "pose", camera.capture_ns, int(config.teacher_tolerance_ms * 1e6))
    if not endpoints:
        raise ValueError("OBSERVATION_POSE_MISSING")
    if any(not isinstance(e.payload, TimedPose) or
           (e.payload.frame_id, e.payload.child_frame_id) != (config.pose_world_frame, config.pose_body_frame)
           or e.payload.timestamp_ns != e.capture_ns for e in endpoints):
        raise ValueError("OBSERVATION_POSE_FRAME_OR_STAMP_MISMATCH")
    indexed = IndexedTimedValues.from_values(tuple(TimedValue(e.capture_ns, e.payload) for e in endpoints))
    pose, _ = _interpolate_pose_indexed(indexed, camera.capture_ns, tolerance_ms=config.teacher_tolerance_ms)
    return (TimeEvent("pose", camera.run, camera.epoch, camera.capture_clock, camera.available_clock,
                      camera.capture_ns, max(e.available_ns for e in endpoints), camera.sequence,
                      pose, "derived_from_available_pose_endpoints"),
            _refs("observation_pose", camera.capture_ns, endpoints))


def _lidar(payload: TimedLidar, config: TimeDatasetConfig) -> np.ndarray:
    if not isinstance(payload, TimedLidar):
        raise ValueError("INVALID_LIDAR_PAYLOAD")
    values = np.asarray(payload.ranges_m, dtype=np.float32)
    if (values.ndim != 1 or values.size < 2 or not math.isfinite(payload.angle_increment_rad)
            or payload.angle_increment_rad <= 0 or not math.isfinite(payload.angle_min_rad)
            or not 0 <= payload.range_min_m < payload.range_max_m):
        raise ValueError("INVALID_LIDAR_GEOMETRY")
    # Common angular grid. Never interpolate invalid returns into valid obstacle ranges.
    grid = config.lidar_angle_min_rad + np.arange(config.lidar_shape[1]) * (2 * math.pi / config.lidar_shape[1])
    indices = np.rint((grid-payload.angle_min_rad)/payload.angle_increment_rad).astype(np.int64)
    inside = (indices >= 0) & (indices < values.size)
    ranges = values[np.clip(indices, 0, values.size-1)]
    valid = (inside & np.isfinite(ranges) & (ranges >= max(payload.range_min_m, config.lidar_min_range_m))
             & (ranges <= min(payload.range_max_m, config.lidar_max_range_m)))
    return normalize_lidar_range_and_validity(ranges, valid, min_range_m=config.lidar_min_range_m,
                                              max_range_m=config.lidar_max_range_m)


def assemble_time_inputs(events: Sequence[TimeEvent], anchor: TimeEvent, *, config: TimeDatasetConfig,
                         epoch_start_ns: int, epoch_end_ns: int, freeze_ns: int) -> tuple[ModelBatchV3, tuple[dict[str, Any], ...]]:
    """Pure offline/runtime input builder; never calls the teacher branch.

    Fixed 100ms slots, past-only camera/LiDAR/sent-command capture. Ego is
    interpolated at each requested slot, using only endpoints available by freeze.
    Feature units are [m/s, m/s, rad/s, rad]; optional missing features have masks.
    """
    if type(freeze_ns) is not int or freeze_ns < 0:
        raise ValueError("freeze must be nonnegative integer available-clock ns")
    if (anchor.role != "camera" or not isinstance(anchor.payload, TimedImage)
            or anchor.payload.timestamp_ns != anchor.capture_ns
            or (anchor.capture_clock, anchor.available_clock) != (config.capture_clock, config.available_clock)):
        raise ValueError("INVALID_CAMERA_ANCHOR_IDENTITY")
    if not epoch_start_ns <= anchor.capture_ns <= epoch_end_ns:
        raise ValueError("ANCHOR_OUTSIDE_EPOCH")
    eligible = _eligible(events, anchor, config, freeze_ns, (epoch_start_ns, epoch_end_ns))
    provenance: list[dict[str, Any]] = []
    def slots(role: str, length: int, previous: bool = False) -> tuple[TimeEvent | None, ...]:
        selected = select_time_history(eligible, role=role, run=anchor.run, epoch=anchor.epoch,
            capture_clock=config.capture_clock, available_clock=config.available_clock,
            freeze_ns=freeze_ns, observation_ns=anchor.capture_ns, length=length,
            tolerance_ns=config.tolerance_ns, previous_only=previous)
        for i, event in enumerate(selected):
            stamp = anchor.capture_ns - (length-1-i+int(previous))*100_000_000
            provenance.append(_refs(role, stamp, () if event is None else (event,)))
        return selected
    images = slots("camera", config.camera_history_length)
    scans = slots("lidar", config.lidar_history_length)
    commands = slots("final_command", config.command_history_length, True)
    if images[-1] is None or images[-1].capture_ns != anchor.capture_ns or scans[-1] is None:
        raise ValueError("CURRENT_SENSOR_MISSING")
    image, image_mask = materialize_history(images, lambda p: preprocess_image(p.image_rgb,
        height=config.image_shape[1], width=config.image_shape[2]).numpy(), shape=config.image_shape)
    lidar, lidar_mask = materialize_history(scans, lambda p: _lidar(p, config), shape=config.lidar_shape)
    def command_value(p: TimedCommand) -> np.ndarray:
        if not isinstance(p, TimedCommand):
            raise ValueError("INVALID_SENT_COMMAND")
        return np.asarray([p.steering_rad, p.speed_mps, p.acceleration_mps2], dtype=np.float32)
    command, command_mask = materialize_history(commands, command_value, shape=(3,))
    ego = np.zeros((config.ego_history_length, 4), dtype=np.float32)
    ego_mask = np.zeros_like(ego, dtype=bool)
    steering = slots("actual_steering", config.ego_history_length)
    for i in range(config.ego_history_length):
        stamp = anchor.capture_ns - (config.ego_history_length-1-i)*100_000_000
        ep = _endpoints(eligible, "velocity", stamp, config.tolerance_ns)
        provenance.append(_refs("velocity", stamp, ep))
        if ep:
            if any(not isinstance(e.payload, TimedVelocity) for e in ep):
                raise ValueError("INVALID_VELOCITY_PAYLOAD")
            vals = np.asarray([[e.payload.longitudinal_mps, e.payload.lateral_mps, e.payload.yaw_rate_rps] for e in ep])
            valid = np.isfinite(vals).all(axis=0)
            a = 0.0 if len(ep) == 1 else (stamp-ep[0].capture_ns)/(ep[1].capture_ns-ep[0].capture_ns)
            value = vals[0] if len(ep) == 1 else vals[0]*(1-a)+vals[1]*a
            ego[i,:3] = np.where(valid, value, 0.0)
            ego_mask[i,:3] = valid
        if steering[i] is not None:
            value = float(steering[i].payload.steering_rad)
            if math.isfinite(value):
                ego[i,3] = value
                ego_mask[i,3] = True
    if not ego_mask[-1,0]:
        raise ValueError("CURRENT_LONGITUDINAL_SPEED_MISSING")
    dt = np.zeros((config.camera_history_length, 2), dtype=np.float32)
    for i, e in enumerate(images):
        slot = anchor.capture_ns-(config.camera_history_length-1-i)*100_000_000
        if e is not None:
            dt[i,0] = (e.capture_ns-slot)/1e9
        near = [s for s in scans if s is not None and 0 <= slot-s.capture_ns <= config.tolerance_ns]
        if near:
            dt[i,1] = (max(s.capture_ns for s in near)-slot)/1e9
    batch = ModelBatchV3(torch.from_numpy(image[None]), torch.from_numpy(image_mask[None]),
        torch.from_numpy(lidar[None]), torch.from_numpy(lidar_mask[None]),
        torch.from_numpy(ego[None]), torch.from_numpy(ego_mask[None]),
        torch.from_numpy(command[None]), torch.from_numpy(command_mask[None]),
        torch.from_numpy(dt[None]), targets=None, requested_outputs=frozenset({"trajectory"}))
    batch.validate(require_current=False)
    return batch, tuple(provenance)


def assemble_time_sample(events: Sequence[TimeEvent], anchor: TimeEvent, *, config: TimeDatasetConfig,
                         epoch_start_ns: int, epoch_end_ns: int, freeze_ns: int,
                         intervention_ns: int | None = None,
                         environment_stop_intent: bool | None = None,
                         safety_brake: bool | None = None,
                         observed_stationary: bool | None = None) -> TimeSample:
    """Teacher and causal inputs are independent; failures preserve the anchor.

    Stop evidence must be explicitly supplied by an audited annotation source.
    UNKNOWN is the default; no uncalibrated threshold becomes a stop-intent label.
    """
    if any(v is not None and type(v) is not bool for v in
           (environment_stop_intent, safety_brake, observed_stationary)):
        raise ValueError("stop evidence must be bool or unknown")
    if any(type(v) is not int or v < 0 for v in (freeze_ns, epoch_start_ns, epoch_end_ns)) or epoch_end_ns < epoch_start_ns:
        raise ValueError("invalid clock bounds")
    teacher = None
    reasons: tuple[str, ...] = ()
    provenance: tuple[dict[str, Any], ...] = ()
    try:
        eligible = _eligible(events, anchor, config, freeze_ns, (epoch_start_ns, epoch_end_ns))
        pose_anchor, ref = _pose_anchor(eligible, anchor, config)
        # Full-run future labels are intentionally NOT availability-cut at observation.
        teacher_events = tuple(e for e in events if e.available_clock == config.available_clock)
        teacher = build_time_teacher(teacher_events, pose_anchor, epoch_start_ns=epoch_start_ns,
            epoch_end_ns=epoch_end_ns, intervention_ns=intervention_ns,
            tolerance_ms=config.teacher_tolerance_ms)
        reasons = teacher.reasons
        provenance = (ref,)
    except ValueError as exc:
        reasons = (str(exc),)*30
    inputs = None
    invalid = None
    try:
        inputs, refs = assemble_time_inputs(events, anchor, config=config, epoch_start_ns=epoch_start_ns,
                                           epoch_end_ns=epoch_end_ns, freeze_ns=freeze_ns)
        provenance += refs
    except ValueError as exc:
        invalid = str(exc)
    if intervention_ns is not None and anchor.capture_ns+3_000_000_000 >= intervention_ns:
        reason = "COLLECTION_INTERVENTION"
    elif safety_brake is True:
        reason = "SAFETY_BRAKE"
    elif environment_stop_intent is True:
        reason = "ENVIRONMENT_STOP_INTENT"
    elif "EPOCH_END" in reasons:
        reason = "HORIZON_END"
    elif observed_stationary is True:
        reason = "OBSERVED_STATIONARY"
    else:
        reason = "UNKNOWN"
    return TimeSample(inputs, teacher, anchor.run, anchor_identity(anchor), anchor.capture_ns,
                      invalid, reasons, provenance, reason, None, freeze_ns,
                      environment_stop_intent, observed_stationary)


class TimeDataset(Dataset[TimeSample]):
    """Each record owns its available-clock cut; no run-wide freeze timestamp."""
    def __init__(self, records: Sequence[tuple[Sequence[TimeEvent], TimeEvent, int]], *,
                 config: TimeDatasetConfig, epoch_bounds: dict[tuple[str, str], tuple[int, int]],
                 intervention_ns: dict[tuple[str, str], int] | None = None) -> None:
        self._records = tuple(records)
        self._config = config
        self._bounds = dict(epoch_bounds)
        self._interventions = dict(intervention_ns or {})

    def __len__(self) -> int:
        return len(self._records)

    def __getitem__(self, index: int) -> TimeSample:
        events, anchor, freeze = self._records[index]
        key = (anchor.run, anchor.epoch)
        if key not in self._bounds:
            return TimeSample(None, None, anchor.run, anchor_identity(anchor), anchor.capture_ns,
                              "EPOCH_BOUNDS_MISSING", freeze_ns=freeze)
        start, end = self._bounds[key]
        return assemble_time_sample(events, anchor, config=self._config, epoch_start_ns=start,
            epoch_end_ns=end, freeze_ns=freeze, intervention_ns=self._interventions.get(key))
