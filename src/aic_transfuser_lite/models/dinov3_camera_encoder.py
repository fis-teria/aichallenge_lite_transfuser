"""Frozen local DINOv3 ViT-S/16 camera adapter.

No network download is performed.  Production construction requires an
explicit local DINOv3 repository, checkpoint, and full SHA-256.
"""
from __future__ import annotations

import hashlib
from copy import deepcopy
from pathlib import Path
from typing import Any

import torch
from torch import nn
from torch.nn import functional as F


DINOV3_MODEL_NAME = "dinov3_vits16"
DINOV3_PATCH_SIZE = 16
DINOV3_EMBED_DIM = 384
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def resolve_camera_encoder_config(
    config: dict[str, Any], *, output_dim: int, token_h: int, token_w: int,
    image_height: int, image_width: int,
) -> dict[str, Any]:
    """Validate an optional model camera config against the shared fusion shape.

    Paths may be null in a portable config template; construction checks that
    the local repository and checkpoint actually exist before loading anything.
    Image preprocessing remains the shared Dataset/runtime responsibility.
    """
    if type(config) is not dict:
        raise TypeError("camera_encoder must be a dict")
    name = config.get("backbone")
    common = {"backbone", "output_dim", "token_h", "token_w"}
    if name == DINOV3_MODEL_NAME:
        allowed = common | {"repository_path", "checkpoint_path", "checkpoint_sha256", "frozen"}
        if config.get("frozen", True) is not True:
            raise ValueError("initial DINOv3 comparison requires frozen=true")
        if image_height % DINOV3_PATCH_SIZE or image_width % DINOV3_PATCH_SIZE:
            raise ValueError("DINOv3 image H/W must be divisible by patch size 16")
        for key in ("repository_path", "checkpoint_path"):
            value = config.get(key)
            if value is not None and (type(value) is not str or not value.strip()):
                raise ValueError(f"camera_encoder.{key} must be a nonempty string or null")
        digest = config.get("checkpoint_sha256")
        if digest is not None and (
            type(digest) is not str or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError("DINOv3 checkpoint_sha256 must be a lowercase full SHA-256")
    elif name == "resnet18":
        allowed = common | {"pretrained"}
        if type(config.get("pretrained", False)) is not bool:
            raise TypeError("camera_encoder.pretrained must be bool")
    else:
        raise ValueError(f"unsupported camera backbone: {name!r}")
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f"unknown camera_encoder keys: {sorted(unknown)}")
    resolved = deepcopy(config)
    for key, expected in (("output_dim", output_dim), ("token_h", token_h), ("token_w", token_w)):
        actual = config.get(key, expected)
        if type(actual) is not int or actual <= 0 or actual != expected:
            raise ValueError(f"camera_encoder.{key} must match model fusion shape ({expected})")
        resolved[key] = expected
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DinoV3CameraEncoder(nn.Module):
    """Project frozen DINOv3 patch tokens to ``[B, token_h*token_w, D]``.

    Input is ImageNet-normalized RGB ``[B,3,H,W]``. H and W must be divisible
    by 16; resizing is a Dataset responsibility and this adapter never crops.
    """

    def __init__(
        self,
        *,
        output_dim: int,
        token_h: int,
        token_w: int,
        repository_path: str | Path | None = None,
        checkpoint_path: str | Path | None = None,
        checkpoint_sha256: str | None = None,
        frozen: bool = True,
        backbone: nn.Module | None = None,
    ) -> None:
        super().__init__()
        if any(type(value) is not int or value <= 0 for value in (output_dim, token_h, token_w)):
            raise ValueError("DINOv3 projection and token dimensions must be positive ints")
        if frozen is not True:
            raise ValueError("initial DINOv3 comparison requires frozen=true")
        self.output_dim = output_dim
        self.token_h = token_h
        self.token_w = token_w
        self.token_count = token_h * token_w
        self.patch_size = DINOV3_PATCH_SIZE
        self._injected_backbone = backbone is not None
        self.repository_path: str | None = None
        self.checkpoint_path: str | None = None
        self.checkpoint_sha256: str | None = None
        if backbone is None:
            if repository_path is None or checkpoint_path is None or checkpoint_sha256 is None:
                raise FileNotFoundError(
                    "DINOv3 requires explicit repository_path, checkpoint_path, and checkpoint_sha256"
                )
            repository = Path(repository_path).expanduser().resolve()
            checkpoint = Path(checkpoint_path).expanduser().resolve()
            if not repository.is_dir() or not (repository / "hubconf.py").is_file():
                raise FileNotFoundError(f"DINOv3 local repository/hubconf.py missing: {repository}")
            if not checkpoint.is_file():
                raise FileNotFoundError(f"DINOv3 checkpoint missing: {checkpoint}")
            if (
                len(checkpoint_sha256) != 64
                or checkpoint_sha256.lower() != checkpoint_sha256
                or any(character not in "0123456789abcdef" for character in checkpoint_sha256)
            ):
                raise ValueError("DINOv3 checkpoint_sha256 must be a lowercase full SHA-256")
            actual_sha256 = _sha256(checkpoint)
            if actual_sha256 != checkpoint_sha256:
                raise ValueError(
                    f"DINOv3 checkpoint SHA-256 mismatch: expected={checkpoint_sha256}, actual={actual_sha256}"
                )
            try:
                backbone = torch.hub.load(
                    str(repository),
                    DINOV3_MODEL_NAME,
                    source="local",
                    weights=str(checkpoint),
                )
            except Exception as exc:
                raise RuntimeError(f"failed to construct local {DINOV3_MODEL_NAME}: {exc}") from exc
            self.repository_path = str(repository)
            self.checkpoint_path = str(checkpoint)
            self.checkpoint_sha256 = actual_sha256
        if not isinstance(backbone, nn.Module):
            raise TypeError("DINOv3 backbone must be torch.nn.Module")
        embed_dim = int(getattr(backbone, "embed_dim", DINOV3_EMBED_DIM))
        if embed_dim != DINOV3_EMBED_DIM:
            raise ValueError(f"DINOv3 ViT-S/16 embed_dim must be {DINOV3_EMBED_DIM}, got {embed_dim}")
        self.backbone = backbone
        self.backbone.requires_grad_(False)
        self.backbone.eval()
        self.projection = nn.Linear(DINOV3_EMBED_DIM, output_dim)

    def train(self, mode: bool = True) -> "DinoV3CameraEncoder":
        super().train(mode)
        self.backbone.eval()
        return self

    def _patch_tokens(self, image: torch.Tensor) -> torch.Tensor:
        with torch.no_grad():
            if hasattr(self.backbone, "forward_features"):
                output = self.backbone.forward_features(image)
            else:
                output = self.backbone(image)
        if isinstance(output, dict):
            if "x_norm_patchtokens" not in output:
                raise RuntimeError("DINOv3 forward_features omitted x_norm_patchtokens")
            output = output["x_norm_patchtokens"]
        if not isinstance(output, torch.Tensor) or output.ndim != 3:
            raise RuntimeError("DINOv3 patch tokens must be tensor [B,N,384]")
        if output.shape[0] != image.shape[0] or output.shape[2] != DINOV3_EMBED_DIM:
            raise RuntimeError("DINOv3 patch-token batch/embed dimension mismatch")
        expected = (image.shape[2] // self.patch_size) * (image.shape[3] // self.patch_size)
        if output.shape[1] != expected:
            raise RuntimeError(f"DINOv3 patch-token count mismatch: expected={expected}, actual={output.shape[1]}")
        if not torch.isfinite(output).all():
            raise RuntimeError("DINOv3 patch tokens must be finite")
        return output

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        if image.ndim != 4 or image.shape[1] != 3:
            raise ValueError(f"DINOv3 image must be [B,3,H,W], got {tuple(image.shape)}")
        if image.shape[2] % self.patch_size or image.shape[3] % self.patch_size:
            raise ValueError("DINOv3 image H/W must be divisible by patch size 16")
        if not image.is_floating_point() or not torch.isfinite(image).all():
            raise ValueError("DINOv3 image must be finite floating ImageNet-normalized RGB")
        patches = self._patch_tokens(image)
        grid = patches.transpose(1, 2).reshape(
            image.shape[0], DINOV3_EMBED_DIM, image.shape[2] // self.patch_size, image.shape[3] // self.patch_size
        )
        pooled = F.adaptive_avg_pool2d(grid, (self.token_h, self.token_w))
        return self.projection(pooled.flatten(2).transpose(1, 2))

    def pretrained_provenance(self) -> dict[str, Any]:
        return {
            "implementation": "facebookresearch/dinov3",
            "model_name": DINOV3_MODEL_NAME,
            "patch_size": DINOV3_PATCH_SIZE,
            "embed_dim": DINOV3_EMBED_DIM,
            "repository_path": self.repository_path,
            "checkpoint_path": self.checkpoint_path,
            "checkpoint_sha256": self.checkpoint_sha256,
            "backbone_frozen": True,
            "input_normalization": {"mean": IMAGENET_MEAN, "std": IMAGENET_STD},
            "injected_test_backbone": self._injected_backbone,
        }


def build_camera_encoder(config: dict[str, Any], *, backbone: nn.Module | None = None) -> nn.Module:
    """Build ResNet18 or DINOv3 without changing the legacy ResNet constructor."""

    name = str(config.get("backbone", ""))
    output_dim = int(config["output_dim"])
    token_h = int(config["token_h"])
    token_w = int(config["token_w"])
    if name == "resnet18":
        if backbone is not None:
            raise ValueError("injected backbone is supported only for DINOv3 tests")
        from .camera_encoder import CameraEncoder

        return CameraEncoder(
            output_dim=output_dim,
            token_h=token_h,
            token_w=token_w,
            pretrained=bool(config.get("pretrained", False)),
        )
    if name == DINOV3_MODEL_NAME:
        return DinoV3CameraEncoder(
            output_dim=output_dim,
            token_h=token_h,
            token_w=token_w,
            repository_path=config.get("repository_path"),
            checkpoint_path=config.get("checkpoint_path"),
            checkpoint_sha256=config.get("checkpoint_sha256"),
            frozen=config.get("frozen", True),
            backbone=backbone,
        )
    raise ValueError(f"unsupported camera backbone: {name!r}")
