"""Time-only backbone: valid CNN frames and fixed-slot time encoding.

Legacy FullControlLiteV3/V4 behavior and state dictionaries remain unchanged.
"""
from __future__ import annotations
from typing import Any
import torch
from torch import nn
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .full_control_lite_v3 import FullControlLiteV3
from .temporal.gru import MaskedGRUTemporalEncoder


class SlotGRU(MaskedGRUTemporalEncoder):
    def __init__(self, hidden: int, *, allow_empty: bool = False, previous_only: bool = False):
        super().__init__(hidden,hidden,allow_empty=allow_empty)
        self.slot_projection=nn.Linear(1,hidden,bias=False)
        self.previous_only=previous_only

    def forward(self, values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        if values.ndim != 3:raise ValueError('slot values must be [B,T,D]')
        offsets=(torch.arange(values.shape[1],device=values.device,dtype=values.dtype)
                 -values.shape[1]+1-int(self.previous_only))*0.1
        return super().forward(values+self.slot_projection(offsets[None,:,None]),mask)


def encode_valid(encoder: nn.Module, values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    """[B,T,...] -> [B,T,N,D]; masked frames never enter CNN or BatchNorm."""
    if mask.dtype != torch.bool or mask.shape != values.shape[:2] or not mask.any():
        raise ValueError('invalid CNN history mask')
    tokens=encoder(values[mask])
    result=tokens.new_zeros((*mask.shape,*tokens.shape[1:]))
    result[mask]=tokens
    return result


class TimeBackboneV1(FullControlLiteV3):
    def __init__(self, **kwargs: Any):
        super().__init__(**kwargs)
        hidden=int(kwargs.get('hidden_dim',128))
        self.camera_temporal=SlotGRU(hidden)
        self.lidar_temporal=SlotGRU(hidden)
        self.ego_history.ego_temporal=SlotGRU(hidden)
        self.ego_history.command_temporal=SlotGRU(hidden,allow_empty=True,previous_only=True)

    def forward_features(self, batch: ModelBatchV3) -> torch.Tensor:
        batch.validate(require_current=True)
        if batch.image.shape[1]>self.max_sensor_history or batch.lidar.shape[1]>self.max_sensor_history:
            raise ValueError('sensor history exceeds configured maximum')
        if batch.ego.shape[1]>self.max_ego_history or batch.command_history.shape[1]>self.max_ego_history:
            raise ValueError('ego/command history exceeds configured maximum')
        if batch.image.shape[2:]!=(3,self.image_height,self.image_width) or batch.lidar.shape[2:]!=(2,self.lidar_points) or batch.ego.shape[2]!=self.ego_dim:
            raise ValueError('time input dimensions mismatch')
        camera=encode_valid(self.camera,batch.image,batch.image_mask)
        lidar=encode_valid(self.lidar,batch.lidar,batch.lidar_mask)
        temporal=self.temporal_projection(torch.cat((
            self.camera_temporal(camera.mean(2),batch.image_mask),
            self.lidar_temporal(lidar.mean(2),batch.lidar_mask),
            self.ego_history(batch.ego,batch.ego_feature_mask,batch.command_history,batch.command_mask)),dim=-1))
        current=self.ego(batch.ego[:,-1])+temporal[:,None]
        return self.fusion(camera[:,-1],lidar[:,-1],current)[1]
