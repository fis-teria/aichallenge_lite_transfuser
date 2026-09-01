from __future__ import annotations

import torch
from torch import nn


class Lidar1DEncoder(nn.Module):
    """Encode a native-order LaserScan into tokens ``[B,N,C]``.

    The default one-channel path preserves the legacy ``[B,P]`` contract and
    state-dict keys. Dataset v2 explicitly selects two input channels and fixed
    sin/cos beam-angle encoding.
    """

    def __init__(
        self,
        output_dim: int = 128,
        token_count: int = 64,
        *,
        input_channels: int = 1,
        lidar_points: int | None = None,
        angle_min_rad: float | None = None,
        angle_increment_rad: float | None = None,
        use_angle_encoding: bool = False,
    ) -> None:
        super().__init__()
        if input_channels not in {1, 2}:
            raise ValueError("LiDAR input_channels must be 1 or 2")
        if token_count <= 0:
            raise ValueError("LiDAR token_count must be positive")
        if lidar_points is not None and lidar_points <= 1:
            raise ValueError("LiDAR lidar_points must be greater than one")
        if use_angle_encoding:
            if lidar_points is None:
                raise ValueError("Angle encoding requires lidar_points")
            if angle_min_rad is None or angle_increment_rad is None:
                raise ValueError(
                    "Angle encoding requires angle_min_rad and angle_increment_rad"
                )
            if angle_increment_rad <= 0.0:
                raise ValueError("angle_increment_rad must be positive")

        self.network = nn.Sequential(
            nn.Conv1d(input_channels, 32, kernel_size=9, stride=2, padding=4),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),
            nn.Conv1d(32, 64, kernel_size=7, stride=2, padding=3),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Conv1d(64, output_dim, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm1d(output_dim),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool1d(token_count),
        )
        self.output_dim = output_dim
        self.token_count = token_count
        self.input_channels = input_channels
        self.lidar_points = lidar_points
        self.use_angle_encoding = use_angle_encoding
        if use_angle_encoding:
            assert lidar_points is not None
            assert angle_min_rad is not None
            assert angle_increment_rad is not None
            angle_max_rad = angle_min_rad + (lidar_points - 1) * angle_increment_rad
            token_angles = torch.linspace(
                angle_min_rad,
                angle_max_rad,
                steps=token_count,
                dtype=torch.float32,
            )
            angle_sincos = torch.stack(
                (torch.sin(token_angles), torch.cos(token_angles)), dim=-1
            )
            self.register_buffer("angle_sincos", angle_sincos, persistent=True)
            self.angle_projection: nn.Linear | None = nn.Linear(
                2, output_dim, bias=False
            )
        else:
            self.register_buffer("angle_sincos", None, persistent=False)
            self.angle_projection = None

    def forward(self, lidar: torch.Tensor) -> torch.Tensor:
        if self.input_channels == 1:
            if lidar.ndim != 2:
                raise ValueError(f"Expected lidar [B,P], got {tuple(lidar.shape)}")
            network_input = lidar.unsqueeze(1)
        else:
            if lidar.ndim != 3 or lidar.shape[1] != self.input_channels:
                raise ValueError(
                    f"Expected lidar [B,{self.input_channels},P], got {tuple(lidar.shape)}"
                )
            network_input = lidar
        if self.lidar_points is not None and network_input.shape[-1] != self.lidar_points:
            raise ValueError(
                f"Expected {self.lidar_points} LiDAR beams, got {network_input.shape[-1]}"
            )
        tokens = self.network(network_input).transpose(1, 2)
        if self.use_angle_encoding:
            if self.angle_projection is None or self.angle_sincos is None:
                raise RuntimeError("LiDAR angle encoding was not initialized")
            position = self.angle_projection(
                self.angle_sincos.to(device=tokens.device, dtype=tokens.dtype)
            )
            tokens = tokens + position.unsqueeze(0)
        return tokens
