"""Separate untrained 46-point/20m candidate; not a fixed-V4 runtime replacement."""
from dataclasses import replace
from torch import nn
from .spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4


class SpatialPathLongV4(SpatialPathDiagnosticV4):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        hidden=int(kwargs.get('hidden_dim',128))
        self.path_head=nn.Sequential(nn.Linear(hidden,hidden),nn.ReLU(),nn.Linear(hidden,92))

    def forward(self,batch):
        inputs=replace(batch,targets=None,requested_outputs=frozenset({'trajectory'}))
        features=self.backbone.forward_features(inputs)
        result=self.path_head(features).reshape(batch.batch_size,46,2)
        assert result.shape==(batch.batch_size,46,2)
        return result
