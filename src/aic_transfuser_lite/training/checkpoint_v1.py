from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import torch


V1_CHECKPOINT_FORMAT = "transfuser_lite_v1_checkpoint_v1"


def save_v1_checkpoint(path: str | Path, payload: dict[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if payload.get("format_version") != V1_CHECKPOINT_FORMAT:
        raise ValueError("Refusing to save an unversioned v1 checkpoint")
    temporary = destination.with_name(f".{destination.name}.tmp-{os.getpid()}")
    try:
        torch.save(payload, temporary)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def load_v1_checkpoint(
    path: str | Path, *, map_location: str | torch.device = "cpu"
) -> dict[str, Any]:
    checkpoint_path = Path(path)
    payload = torch.load(checkpoint_path, map_location=map_location, weights_only=False)
    if not isinstance(payload, dict):
        raise ValueError(f"Checkpoint root must be a mapping: {checkpoint_path}")
    if payload.get("format_version") != V1_CHECKPOINT_FORMAT:
        raise ValueError(
            "Not a TransFuser Lite v1 checkpoint; legacy v0 checkpoints are "
            "intentionally rejected by the v1 loader"
        )
    required = {
        "model",
        "config",
        "resolved_config",
        "epoch",
        "global_step",
        "resolved_config_sha256",
        "dataset_manifest_sha256",
    }
    missing = sorted(required - set(payload))
    if missing:
        raise ValueError(f"Incomplete v1 checkpoint; missing keys: {missing}")
    return payload
