from __future__ import annotations

from typing import Any

import torch


@torch.inference_mode()
def infer(
    model: torch.nn.Module,
    *,
    image: torch.Tensor,
    lidar: torch.Tensor,
    ego: torch.Tensor,
    model_name: str = "transfuser_lite",
) -> dict[str, Any]:
    """Run a batch-size-one inference and return CPU tensors."""
    model.eval()
    if model_name == "lidar_only":
        output = model(lidar, ego)
    else:
        output = model(image, lidar, ego)
    result = {key: value.detach().cpu() for key, value in output.items()}
    if "stop_logit" in result:
        result["stop_probability"] = torch.sigmoid(result["stop_logit"])
    return result


@torch.inference_mode()
def infer_v1(
    model: torch.nn.Module,
    *,
    image: torch.Tensor,
    lidar: torch.Tensor,
    ego: torch.Tensor,
    use_amp: bool,
) -> dict[str, torch.Tensor]:
    """Run the static-v1 output contract with CUDA AMP when requested."""

    model.eval()
    device_type = image.device.type
    if lidar.device != image.device or ego.device != image.device:
        raise ValueError("v1 inference tensors must share one device")
    amp_enabled = bool(use_amp and device_type == "cuda")
    with torch.autocast(
        device_type=device_type,
        dtype=torch.float16,
        enabled=amp_enabled,
    ):
        output = model(image, lidar, ego)
    expected = {"waypoints", "target_speed"}
    if set(output) != expected:
        raise RuntimeError(
            f"Static v1 output contract drifted: expected {expected}, got {set(output)}"
        )
    result = {
        key: value.detach().to(device="cpu", dtype=torch.float32)
        for key, value in output.items()
    }
    if result["waypoints"].ndim != 3 or result["waypoints"].shape[-1] != 2:
        raise RuntimeError("Static v1 waypoint output must be [B,N,2]")
    if result["target_speed"].shape != (image.shape[0], 1):
        raise RuntimeError("Static v1 target-speed output must be [B,1]")
    if not all(torch.isfinite(value).all() for value in result.values()):
        raise RuntimeError("Static v1 inference produced a non-finite output")
    return result
