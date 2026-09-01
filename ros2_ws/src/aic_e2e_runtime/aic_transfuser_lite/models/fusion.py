from __future__ import annotations

import torch
from torch import nn


class TokenFusionTransformer(nn.Module):
    """Fuse image, LiDAR, and ego tokens with self-attention."""

    def __init__(
        self,
        dim: int,
        image_tokens: int,
        lidar_tokens: int,
        depth: int = 3,
        heads: int = 4,
        mlp_ratio: int = 4,
        dropout: float = 0.1,
        pooling: str = "ego",
    ) -> None:
        super().__init__()
        if pooling not in {"ego", "learned_cls"}:
            raise ValueError(
                f"Unsupported model.fusion.pooling={pooling!r}; "
                "expected 'ego' or 'learned_cls'"
            )
        self.pooling = pooling
        self.image_tokens = image_tokens
        self.lidar_tokens = lidar_tokens
        self.content_tokens = image_tokens + lidar_tokens + 1
        self.total_tokens = self.content_tokens + int(pooling == "learned_cls")
        self.position = nn.Parameter(torch.zeros(1, self.content_tokens, dim))
        self.modality = nn.Parameter(torch.zeros(3, 1, dim))
        nn.init.trunc_normal_(self.position, std=0.02)
        nn.init.trunc_normal_(self.modality, std=0.02)

        layer = nn.TransformerEncoderLayer(
            d_model=dim,
            nhead=heads,
            dim_feedforward=dim * mlp_ratio,
            dropout=dropout,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(layer, num_layers=depth)
        self.norm = nn.LayerNorm(dim)
        if pooling == "learned_cls":
            self.cls_token = nn.Parameter(torch.empty(1, 1, dim))
            nn.init.trunc_normal_(self.cls_token, std=0.02)
        else:
            # None is not serialized, preserving legacy state_dict keys exactly.
            self.register_parameter("cls_token", None)

    def forward(
        self,
        image_tokens: torch.Tensor,
        lidar_tokens: torch.Tensor,
        ego_token: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if image_tokens.shape[1] != self.image_tokens:
            raise ValueError(
                f"Expected {self.image_tokens} image tokens, got {image_tokens.shape[1]}"
            )
        if lidar_tokens.shape[1] != self.lidar_tokens:
            raise ValueError(
                f"Expected {self.lidar_tokens} lidar tokens, got {lidar_tokens.shape[1]}"
            )
        image_tokens = image_tokens + self.modality[0]
        lidar_tokens = lidar_tokens + self.modality[1]
        ego_token = ego_token + self.modality[2]
        content = torch.cat([image_tokens, lidar_tokens, ego_token], dim=1)
        content = content + self.position
        if self.pooling == "learned_cls":
            if self.cls_token is None:
                raise RuntimeError("learned_cls pooling is missing its CLS parameter")
            cls_token = self.cls_token.expand(content.shape[0], -1, -1)
            tokens = torch.cat([cls_token, content], dim=1)
        else:
            tokens = content
        fused = self.norm(self.encoder(tokens))
        pooled = fused[:, 0] if self.pooling == "learned_cls" else fused[:, -1]
        return fused, pooled
