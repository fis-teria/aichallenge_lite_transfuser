"""Explicit BatchNorm policies for isolated native-teacher diagnostics."""
from __future__ import annotations

import torch
from torch import nn


def freeze_batchnorm_statistics(model: nn.Module) -> list[str]:
    """Call AFTER model.train(); freeze running statistics, keep affine gradients.

    This is opt-in. It does not freeze the camera/LiDAR weights or other layers.
    A later model.train() must be followed by this function again.
    """
    names = []
    for name, module in model.named_modules():
        if isinstance(module, nn.modules.batchnorm._BatchNorm):
            if not module.track_running_stats:
                raise ValueError(f"BatchNorm needs stored statistics: {name}")
            module.eval()
            names.append(name)
    if not names:
        raise ValueError("model has no BatchNorm statistics")
    return names


def copy_batchnorm_statistics(destination: nn.Module, source: nn.Module,
                              *, prefix: str = "") -> list[str]:
    """Copy ONLY mean/variance/counters, never parameters; tensors keep their units.

    Models must have identical selected BatchNorm module names and buffer shapes.
    All buffers are validated before copying so a mismatch cannot partly apply.
    """
    def selected(model: nn.Module) -> dict[str, nn.Module]:
        return {name: module for name, module in model.named_modules()
                if isinstance(module, nn.modules.batchnorm._BatchNorm)
                and (not prefix or name == prefix or name.startswith(prefix + "."))}

    dst, src = selected(destination), selected(source)
    if not dst or dst.keys() != src.keys():
        raise ValueError("matching nonempty BatchNorm modules required")
    pairs = []
    for name in dst:
        for key in ("running_mean", "running_var", "num_batches_tracked"):
            target, value = getattr(dst[name], key), getattr(src[name], key)
            if (target is None or value is None or target.shape != value.shape
                    or target.dtype != value.dtype or not torch.isfinite(value).all()
                    or (key == "running_var" and (value < 0).any())):
                raise ValueError(f"invalid BatchNorm buffer: {name}.{key}")
            pairs.append((target, value))
    with torch.no_grad():
        for target, value in pairs:
            target.copy_(value)
    return sorted(dst)
