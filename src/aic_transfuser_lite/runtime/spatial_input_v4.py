"""V4 bounded, causal grid adapter. No ROS, asset reader, teacher or control calls."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json

import numpy as np
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS, FEATURES, EGO_LIMITS, BOUNDS
from aic_transfuser_lite.data.dataset_view_v3 import _ego_row, _selected_command
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.data.normalization import normalize_lidar_range_and_validity

INPUT_ID = '77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7'
SHAPES = ((1,4,3,224,384),(1,4),(1,4,2,750),(1,4),(1,10,4),(1,10,4),(1,10,3),(1,10),(1,4,2))
MASKS = {'image_mask', 'lidar_mask', 'ego_feature_mask', 'command_mask'}


@dataclass(frozen=True)
class Stamp:
    """Transport only: header/device ns and independent process-monotonic ns."""
    header_ns: int
    received_ns: int
    available_ns: int
    clock_id: str = 'synthetic_ros'
    epoch: str = '0'
    monotonic_id: str = 'synthetic_process'
    acquisition_ns: int | None = None

    def usable(self, finalized_ns: int, reference: Stamp) -> bool:
        return (self.clock_id, self.epoch, self.monotonic_id) == (reference.clock_id, reference.epoch, reference.monotonic_id) and self.received_ns <= self.available_ns <= finalized_ns


@dataclass(frozen=True)
class PassiveCommand:
    stamp: Stamp
    steering_rad: float
    speed_mps: float
    acceleration_mps2: float
    source: str = 'nominal'
    valid: bool = True


@dataclass(frozen=True)
class GridObservation:
    """One caller-synchronized grid: RGB uint8 HWC, ranges [750], ego SI [4].

    Ego must already refer to camera time (exact or evidenced interpolation).
    Wrapper does not silently relabel nearest ego as interpolated ego.
    """
    grid_ns: int
    camera: Stamp
    lidar: Stamp
    ego_stamp: Stamp
    rgb: np.ndarray
    ranges_m: np.ndarray
    ego_si: tuple[float, float, float, float]
    steering_valid: bool = True
    sample_id: str = 'transport_only'
    range_min_m: float = 0.0
    range_max_m: float = 25.0


def freeze_batch(batch: ModelBatchV3, device: str = 'cpu') -> ModelBatchV3:
    """Clone exactly the nine tensors; targets and caller IDs cannot reach forward."""
    values = {}
    for name, shape in zip(INPUT_FIELDS, SHAPES):
        t = getattr(batch, name)
        dtype = torch.bool if name in MASKS else torch.float32
        if not isinstance(t, torch.Tensor) or tuple(t.shape) != shape or t.dtype != dtype:
            raise ValueError('INPUT_CONTRACT_ERROR:' + name)
        if not bool(torch.isfinite(t).all()):
            raise ValueError('NONFINITE_INPUT:' + name)
        values[name] = t.detach().to(device=device).clone().contiguous()
    return ModelBatchV3(**values, targets=None, requested_outputs=frozenset({'trajectory'}))


class SpatialInputV4:
    """At most 11 grid slots / 64 passive commands; epoch reset clears both."""
    def __init__(self, *, command_binding_known: bool = False, final_fallback_verified: bool = False):
        self.command_binding_known = command_binding_known
        self.final_fallback_verified = final_fallback_verified
        self.frames: deque = deque(maxlen=11)
        self.commands: deque[PassiveCommand] = deque(maxlen=64)
        self.reset_count = 0
        self.last_reason = 'WARM_UP'

    def reset(self, reason: str) -> None:
        self.frames.clear()
        self.commands.clear()
        self.reset_count += 1
        self.last_reason = reason

    def add_command(self, command: PassiveCommand) -> str:
        if command.source not in ('nominal', 'final_fallback'):
            raise ValueError('unknown command source')
        if self.frames:
            reference = self.frames[-1][0].camera
            if (command.stamp.clock_id, command.stamp.epoch, command.stamp.monotonic_id) != (reference.clock_id, reference.epoch, reference.monotonic_id):
                return 'COMMAND_EPOCH_REJECTED'
        for old in self.commands:
            if old.source == command.source and old.stamp.header_ns == command.stamp.header_ns and old.stamp.epoch == command.stamp.epoch:
                return 'DUPLICATE_COMMAND'
        # Late arrival is retained with availability time, never made retroactively available.
        self.commands.append(command)
        return 'ACCEPTED'

    def append(self, item: GridObservation, finalized_ns: int) -> str:
        if not all(s.usable(finalized_ns, item.camera) for s in (item.camera, item.lidar, item.ego_stamp)):
            return 'NOT_AVAILABLE_OR_CLOCK_MISMATCH'
        if item.ego_stamp.header_ns != item.camera.header_ns:
            return 'EGO_ALIGNMENT_UNPROVEN'
        if self.frames:
            prev = self.frames[-1][0]
            if item.grid_ns == prev.grid_ns and item.camera.epoch == prev.camera.epoch:
                return 'DUPLICATE'
            if (item.camera.clock_id, item.camera.epoch, item.camera.monotonic_id) != (prev.camera.clock_id, prev.camera.epoch, prev.camera.monotonic_id):
                self.reset('CLOCK_RESET')
            elif item.grid_ns < prev.grid_ns or item.camera.header_ns < prev.camera.header_ns:
                self.reset('CLOCK_REGRESSION')
            elif item.grid_ns - prev.grid_ns > 200_000_000:
                self.reset('GAP_RESET')
        if np.asarray(item.rgb).ndim != 3 or np.asarray(item.rgb).shape[-1] != 3 or item.rgb.dtype != np.uint8:
            raise ValueError('RGB must be uint8 HWC')
        ranges = np.asarray(item.ranges_m, dtype=np.float32)
        if ranges.shape != (750,):
            raise ValueError('LiDAR must be 750 beams; resampling not authorized')
        if not np.isfinite([item.range_min_m,item.range_max_m]).all() or item.range_min_m>=item.range_max_m:
            raise ValueError('invalid sensor range geometry')
        valid = np.isfinite(ranges) & (ranges >= item.range_min_m) & (ranges <= item.range_max_m)
        image = preprocess_image(item.rgb, height=224, width=384)
        lidar = torch.from_numpy(normalize_lidar_range_and_validity(ranges, valid, min_range_m=0, max_range_m=25))
        row = dict(zip(('velocity_longitudinal_mps','velocity_lateral_mps','yaw_rate_rps','actual_steering_rad'), map(str, item.ego_si)))
        row['actual_steering_valid'] = str(item.steering_valid).lower()
        ego, mask = _ego_row(row, FEATURES, abs_limits=EGO_LIMITS)
        timing = torch.tensor([(item.camera.header_ns-item.grid_ns)*1e-9, (item.lidar.header_ns-item.camera.header_ns)*1e-9], dtype=torch.float32)
        # No mutable sensor buffers remain reachable in the retained frame.
        transport = replace(item, rgb=np.empty((0,0,3), dtype=np.uint8), ranges_m=np.empty(0, dtype=np.float32))
        self.frames.append((transport,image,lidar,ego,mask,timing))
        return 'ACCEPTED'

    def build(self, finalized_ns: int) -> tuple[ModelBatchV3, dict]:
        if not self.frames:
            raise ValueError('NO_FRAME')
        if not self.command_binding_known:
            raise ValueError('BLOCKED_REAL_INPUT_BINDING')
        current = self.frames[-1][0]
        frames = list(self.frames)
        if any(not f[0].camera.usable(finalized_ns, current.camera) for f in frames):
            raise ValueError('HISTORY_UNAVAILABLE')
        def padded(n: int):
            selected = frames[-n:]
            pad = n-len(selected)
            return [selected[0]]*pad+selected, [False]*pad+[True]*len(selected)
        sensor, sm = padded(4)
        ego, em = padded(10)
        past = frames[:-1][-10:]
        commands, cm, cp = [], [], []
        for frame in past:
            anchor = frame[0].camera
            row = {'nominal_command': '{}', 'final_command': '{}'}
            chosen = {}
            for source, field in (('nominal','nominal_command'), ('final_fallback','final_command')):
                if source == 'final_fallback' and not self.final_fallback_verified:
                    continue
                eligible = [c for c in self.commands if c.source == source and c.stamp.usable(finalized_ns, current.camera)
                            and c.stamp.header_ns <= anchor.header_ns and c.stamp.header_ns < current.camera.header_ns
                            and anchor.header_ns-c.stamp.header_ns <= 50_000_000]
                if eligible:
                    c = max(eligible, key=lambda c: c.stamp.header_ns)
                    chosen[source] = c
                    row[field] = json.dumps(dict(valid=c.valid, steering_rad=c.steering_rad, speed_mps=c.speed_mps, acceleration_mps2=c.acceleration_mps2))
            selected = _selected_command(row, bounds=BOUNDS)
            commands.append(selected[0] if selected else torch.zeros(3))
            cm.append(selected is not None)
            cp.append(chosen[selected[1]] if selected else None)
        pad = 10-len(commands)
        if len(frames) > 1 and not any(cm):
            raise ValueError('COMMAND_SOURCE_STALE_OR_MISSING')
        batch = ModelBatchV3(image=torch.stack([f[1] for f in sensor])[None], image_mask=torch.tensor([sm]),
            lidar=torch.stack([f[2] for f in sensor])[None], lidar_mask=torch.tensor([sm]),
            ego=torch.stack([f[3] for f in ego])[None], ego_feature_mask=(torch.stack([f[4] for f in ego]) & torch.tensor(em)[:,None])[None],
            command_history=torch.stack([torch.zeros(3)]*pad+commands)[None], command_mask=torch.tensor([[False]*pad+cm]),
            sensor_dt_sec=torch.stack([f[5] for f in sensor])[None], targets=None, requested_outputs=frozenset({'trajectory'}))
        return freeze_batch(batch), {'sensor_frames': [f[0] for f in sensor], 'sensor_masks':sm,
            'ego_frames':[f[0] for f in ego], 'ego_masks':em, 'commands':[None]*pad+cp,
            'command_padding':[True]*pad+[False]*len(past),
            'command_frames':[None]*pad+[f[0] for f in past],
            'selection_cutoff_ns':finalized_ns, 't_obs_ns':current.camera.header_ns,
            'reset_count':self.reset_count, 'source':'SYNTHETIC_OR_PASSIVE_TRANSPORT'}
