"""Explicit BatchNorm policies for isolated native-teacher diagnostics."""
from __future__ import annotations

import torch
from torch import nn


def gradient_alignment(left: list[torch.Tensor | None],
                       right: list[torch.Tensor | None]) -> dict[str, float | None]:
    """Cosine and L2 norms of two gradients; absent gradients mean zero.

    Norm units depend on the loss and parameter units. Zero vectors have an
    undefined cosine (None), not a fabricated positive or negative alignment.
    """
    if not left or len(left) != len(right):
        raise ValueError("matching nonempty gradient lists required")
    dot, a2, b2 = 0.0, 0.0, 0.0
    for a, b in zip(left, right):
        if a is not None and b is not None and a.shape != b.shape:
            raise ValueError("gradient shapes differ")
        for value in (a, b):
            if value is not None and not torch.isfinite(value).all():
                raise ValueError("finite gradients required")
        if a is not None:
            a2 += float(a.detach().double().square().sum())
        if b is not None:
            b2 += float(b.detach().double().square().sum())
        if a is not None and b is not None:
            dot += float((a.detach().double() * b.detach().double()).sum())
    an, bn = a2 ** 0.5, b2 ** 0.5
    return {"old_norm": an, "native_norm": bn,
            "cosine": max(-1.0, min(1.0, dot / (an * bn))) if an and bn else None,
            "native_to_old_norm": bn / an if an else None}


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
