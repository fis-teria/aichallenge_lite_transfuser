"""Explicit 36-point AWSIM observation trial; no vehicle control promotion."""
from __future__ import annotations

import hashlib
import io
from pathlib import Path
import numpy as np
import torch

from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
TEN_GRID = LONG_GRID[:36].copy()
from aic_transfuser_lite.models.spatial_path_ten_v4 import SpatialPathTenV4
from .publisherless_shadow_v4 import ShadowSession, inference_device_metadata
from .spatial_input_v4 import freeze_batch

CHECKPOINT_SHA = '32752af8d3ebd023382ec405a0f72d529b4120471adabf4f12469c6baf7be22e'


class TenShadowSession(ShadowSession):
    output_points = 36
    output_source = 'TEN_V4_10M_EPOCH2_UNCORRECTED'


def validate_trial(config: dict) -> None:
    if (set(config) != {'checkpoint', 'sha256'} or
            not Path(config['checkpoint']).is_absolute() or config['sha256'] != CHECKPOINT_SHA):
        raise ValueError('EXPLICIT_TEN_EPOCH2_IDENTITY_REQUIRED')


def snapshot(result: torch.Tensor) -> np.ndarray:
    """Copy float32 [1,36,2] vehicle-relative XY metres, without correction."""
    if result.shape != (1, 36, 2) or result.dtype != torch.float32 or not torch.isfinite(result).all():
        raise ValueError('LONG_OUTPUT_SHAPE_DTYPE_FINITE')
    return result.detach().cpu().numpy()[0].copy()


def ten_infer_factory(config: dict):
    validate_trial(config)
    path = Path(config['checkpoint'])
    if path.is_symlink():
        raise ValueError('CHECKPOINT_SYMLINK')
    blob = path.read_bytes()
    if hashlib.sha256(blob).hexdigest() != CHECKPOINT_SHA:
        raise ValueError('CHECKPOINT_IDENTITY')
    payload = torch.load(io.BytesIO(blob), map_location='cpu', weights_only=True)
    if payload['epoch'] != 2 or payload.get('model_type') != 'SpatialPathTenV4' or not np.array_equal(payload['grid_m'], TEN_GRID):
        raise ValueError('CHECKPOINT_GRID_EPOCH')
    model = SpatialPathTenV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4)
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
                    epoch=2, grid_m=np.asarray(TEN_GRID).tolist(), strict=True, weights_only=True,
                    execution_device=inference_device_metadata(model), scope='SHADOW_ONLY')
    def infer(batch):
        with torch.inference_mode():
            return snapshot(model(freeze_batch(batch, device)))
    return infer, identity
