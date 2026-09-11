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
        self.ego_mask_projection=nn.Linear(self.ego_dim,hidden,bias=False)

    def _ego_features(self, batch: ModelBatchV3) -> tuple[torch.Tensor, torch.Tensor]:
        """Feature 0 is measured longitudinal speed; other SI features may be absent.

        Both current and historical paths sanitize masked values before projection.
        A mask embedding distinguishes a measured zero from an unavailable feature.
        """
        mask=batch.ego_feature_mask
        values=torch.where(mask,batch.ego,0.0)
        encoded=self.ego_history.ego_projection(values)+self.ego_mask_projection(mask.to(values.dtype))
        ego_hidden=self.ego_history.ego_temporal(encoded,mask[...,0])
        command=torch.where(batch.command_mask[...,None],batch.command_history,0.0)
        command_hidden=self.ego_history.command_temporal(
            self.ego_history.command_projection(command),batch.command_mask)
        history=self.ego_history.output(torch.cat((ego_hidden,command_hidden),dim=-1))
        current=self.ego(values[:,-1])+self.ego_mask_projection(mask[:,-1].to(values.dtype))[:,None]
        return history,current

    def forward_features(self, batch: ModelBatchV3) -> torch.Tensor:
        batch.validate(require_current=False)
        if (not batch.image_mask[:,-1].all() or not batch.lidar_mask[:,-1].all()
                or not batch.ego_feature_mask[:,-1,0].all()):
            raise ValueError('current Camera, LiDAR and longitudinal speed must be valid')
        if batch.image.shape[1]>self.max_sensor_history or batch.lidar.shape[1]>self.max_sensor_history:
            raise ValueError('sensor history exceeds configured maximum')
        if batch.ego.shape[1]>self.max_ego_history or batch.command_history.shape[1]>self.max_ego_history:
            raise ValueError('ego/command history exceeds configured maximum')
        if batch.image.shape[2:]!=(3,self.image_height,self.image_width) or batch.lidar.shape[2:]!=(2,self.lidar_points) or batch.ego.shape[2]!=self.ego_dim:
            raise ValueError('time input dimensions mismatch')
        camera=encode_valid(self.camera,batch.image,batch.image_mask)
        lidar=encode_valid(self.lidar,batch.lidar,batch.lidar_mask)
        ego_history,current=self._ego_features(batch)
        temporal=self.temporal_projection(torch.cat((
            self.camera_temporal(camera.mean(2),batch.image_mask),
            self.lidar_temporal(lidar.mean(2),batch.lidar_mask),
            ego_history),dim=-1))
        current=current+temporal[:,None]
        return self.fusion(camera[:,-1],lidar[:,-1],current)[1]
