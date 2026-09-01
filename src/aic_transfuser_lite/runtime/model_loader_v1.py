from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
from pathlib import Path

import torch
import yaml

from aic_transfuser_lite.config import validate_v1_config
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.training.checkpoint_v1 import load_v1_checkpoint


@dataclass(frozen=True)
class LoadedRuntimeModelV1:
    model: torch.nn.Module
    config: dict[str, object]
    checkpoint_sha256: str
    resolved_config_sha256: str
    checkpoint_epoch: int


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolved_config_sha256(config: dict[str, object]) -> str:
    encoded = yaml.safe_dump(
        config,
        sort_keys=False,
        allow_unicode=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_runtime_model_v1(
    checkpoint_path: str | Path,
    *,
    device: torch.device,
    expected_checkpoint_sha256: str,
) -> LoadedRuntimeModelV1:
    """Strictly construct the exact static-v1 model embedded in a checkpoint."""

    path = Path(checkpoint_path)
    expected = str(expected_checkpoint_sha256).strip().lower()
    if len(expected) != 64 or any(character not in "0123456789abcdef" for character in expected):
        raise ValueError("expected_checkpoint_sha256 must be a lowercase SHA-256 hex digest")
    actual = sha256_file(path)
    if actual != expected:
        raise ValueError(
            f"checkpoint SHA-256 mismatch: actual={actual}, expected={expected}"
        )
    checkpoint = load_v1_checkpoint(path, map_location="cpu")
    config = copy.deepcopy(checkpoint["resolved_config"])
    if checkpoint["config"] != config:
        raise ValueError("checkpoint config and resolved_config differ")
    validate_v1_config(config)
    calculated_config_hash = resolved_config_sha256(config)
    if calculated_config_hash != checkpoint["resolved_config_sha256"]:
        raise ValueError(
            "checkpoint resolved_config_sha256 does not match its embedded config"
        )

    construction = copy.deepcopy(config)
    construction["model"]["camera"]["pretrained"] = False
    model = build_model(construction).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval()
    return LoadedRuntimeModelV1(
        model=model,
        config=config,
        checkpoint_sha256=actual,
        resolved_config_sha256=calculated_config_hash,
        checkpoint_epoch=int(checkpoint["epoch"]),
    )
