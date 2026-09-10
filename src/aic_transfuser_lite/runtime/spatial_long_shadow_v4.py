"""Explicit 46-point AWSIM observation trial; no vehicle control promotion."""
from __future__ import annotations

import hashlib
import io
from pathlib import Path
import numpy as np
import torch

from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from .publisherless_shadow_v4 import ShadowSession, inference_device_metadata
from .spatial_input_v4 import freeze_batch

CHECKPOINT_SHA = '07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33'


class LongShadowSession(ShadowSession):
    output_points = 46
    output_source = 'LONG_V4_20M_EPOCH12_UNCORRECTED'


def validate_trial(config: dict) -> None:
    if (set(config) != {'checkpoint', 'sha256'} or
            not Path(config['checkpoint']).is_absolute() or config['sha256'] != CHECKPOINT_SHA):
        raise ValueError('EXPLICIT_LONG_EPOCH12_IDENTITY_REQUIRED')


def snapshot(result: torch.Tensor) -> np.ndarray:
    """Copy float32 [1,46,2] vehicle-relative XY metres, without correction."""
    if result.shape != (1, 46, 2) or result.dtype != torch.float32 or not torch.isfinite(result).all():
        raise ValueError('LONG_OUTPUT_SHAPE_DTYPE_FINITE')
    return result.detach().cpu().numpy()[0].copy()


def long_infer_factory(config: dict):
    validate_trial(config)
    path = Path(config['checkpoint'])
    if path.is_symlink():
        raise ValueError('CHECKPOINT_SYMLINK')
    blob = path.read_bytes()
    if hashlib.sha256(blob).hexdigest() != CHECKPOINT_SHA:
        raise ValueError('CHECKPOINT_IDENTITY')
    payload = torch.load(io.BytesIO(blob), map_location='cpu', weights_only=True)
    if payload['epoch'] != 12 or not np.array_equal(payload['grid_m'], LONG_GRID):
        raise ValueError('CHECKPOINT_GRID_EPOCH')
    model = SpatialPathLongV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4)
    expected = model.state_dict()
    state = payload['model']
    if set(state) != set(expected):
        raise ValueError('STATE_KEYS')
    for name, value in state.items():
        if (value.shape != expected[name].shape or value.dtype != expected[name].dtype or
                not torch.isfinite(value).all()):
            raise ValueError('INVALID_STATE:' + name)
    model.load_state_dict(state, strict=True)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    model.eval().requires_grad_(False).to(device)
    if hashlib.sha256(path.read_bytes()).hexdigest() != CHECKPOINT_SHA:
        raise ValueError('CHECKPOINT_CHANGED')
    identity = dict(path=str(path), before_sha256=CHECKPOINT_SHA, after_sha256=CHECKPOINT_SHA,
                    epoch=12, grid_m=np.asarray(LONG_GRID).tolist(), strict=True, weights_only=True,
                    execution_device=inference_device_metadata(model), scope='SHADOW_ONLY')
    def infer(batch):
        with torch.inference_mode():
            return snapshot(model(freeze_batch(batch, device)))
    return infer, identity
