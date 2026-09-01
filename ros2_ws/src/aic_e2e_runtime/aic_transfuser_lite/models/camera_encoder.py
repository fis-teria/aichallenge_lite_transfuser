from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import torch
from torch import nn
from torchvision.models import ResNet18_Weights, resnet18


class CameraEncoder(nn.Module):
    """ResNet18 backbone that returns image tokens [B, N, C]."""

    def __init__(
        self,
        output_dim: int = 128,
        token_h: int = 8,
        token_w: int = 8,
        pretrained: bool = False,
    ) -> None:
        super().__init__()
        weights = ResNet18_Weights.DEFAULT if pretrained else None
        network = resnet18(weights=weights)
        self.pretrained_requested = bool(pretrained)
        self.pretrained_weights_name = weights.name if weights is not None else None
        self.pretrained_weights_url = weights.url if weights is not None else None
        self.backbone = nn.Sequential(*list(network.children())[:-2])
        self.projection = nn.Conv2d(512, output_dim, kernel_size=1)
        self.pool = nn.AdaptiveAvgPool2d((token_h, token_w))
        self.output_dim = output_dim
        self.token_count = token_h * token_w

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        if image.ndim != 4 or image.shape[1] != 3:
            raise ValueError(f"Expected image [B,3,H,W], got {tuple(image.shape)}")
        features = self.pool(self.projection(self.backbone(image)))
        return features.flatten(2).transpose(1, 2)

    def pretrained_provenance(self) -> dict[str, Any]:
        """Return the resolved torchvision weight identity and cached-file hash."""
        if not self.pretrained_requested or self.pretrained_weights_url is None:
            return {
                "requested": False,
                "weights_name": None,
                "url": None,
                "cache_path": None,
                "sha256": None,
            }
        filename = Path(urlparse(self.pretrained_weights_url).path).name
        cache_path = Path(torch.hub.get_dir()) / "checkpoints" / filename
        digest: str | None = None
        if cache_path.is_file():
            hasher = hashlib.sha256()
            with cache_path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    hasher.update(chunk)
            digest = hasher.hexdigest()
        return {
            "requested": True,
            "weights_name": self.pretrained_weights_name,
            "url": self.pretrained_weights_url,
            "cache_path": str(cache_path),
            "sha256": digest,
        }
