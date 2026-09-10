"""V4-10 diagnostic model: 36 spatial XY points up to 10 m."""
from dataclasses import replace
from typing import Mapping

import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4


class SpatialPathTenV4(SpatialPathDiagnosticV4):
    def __init__(self, **kwargs: object):
        super().__init__(**kwargs)
        hidden = int(kwargs.get('hidden_dim', 128))
        self.path_head = nn.Sequential(nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, 72))

    def forward(self, batch: ModelBatchV3) -> torch.Tensor:
        """Input-only batch -> [B,36,2] base_link@t_obs coordinates in metres."""
        inputs = replace(batch, targets=None, requested_outputs=frozenset({'trajectory'}))
        result = self.path_head(self.backbone.forward_features(inputs)).reshape(batch.batch_size, 36, 2)
        assert result.shape == (batch.batch_size, 36, 2)
        return result

    def initialize_from_twenty(self, state: Mapping[str, torch.Tensor]) -> None:
        """Strictly import V4-20, selecting its first 72 final-head coordinates."""
        if state['path_head.2.weight'].shape != (92, self.path_head[2].in_features) or state['path_head.2.bias'].shape != (92,):
            raise ValueError('expected V4-20 output head')
        if not all(torch.isfinite(t).all() for t in state.values()):
            raise ValueError('nonfinite source model')
        selected = dict(state)
        for name in ('path_head.2.weight', 'path_head.2.bias'):
            selected[name] = selected[name][:72].clone()
        self.load_state_dict(selected, strict=True)
