from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3


@dataclass(frozen=True)
class TokenBundle:
    tokens: torch.Tensor
    mask: torch.Tensor

    def validate(self) -> None:
        if self.tokens.ndim != 3 or self.mask.shape != self.tokens.shape[:2]:
            raise ValueError("tokens/mask must be [B,N,D]/[B,N]")
        if self.mask.dtype != torch.bool or not torch.isfinite(self.tokens).all():
            raise ValueError("token mask must be bool and tokens finite")


class V3Model(Protocol):
    def forward(self, batch: ModelBatchV3) -> ModelOutputV3: ...


class TokenEncoder(Protocol):
    def forward(self, values: torch.Tensor, mask: torch.Tensor) -> TokenBundle: ...
