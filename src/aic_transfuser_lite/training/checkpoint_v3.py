from __future__ import annotations

import os
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch


CHECKPOINT_FORMAT_V3 = "aic_training_checkpoint_v3"


@dataclass(frozen=True)
class ExperimentIdentityV3:
    dataset_hash: str
    split_hash: str
    view_hash: str
    contract_hash: str
    seed: int

    def validate(self) -> None:
        values = (self.dataset_hash, self.split_hash, self.view_hash, self.contract_hash)
        if any(not value for value in values) or self.seed < 0:
            raise ValueError("experiment identity fields must be non-empty and seed non-negative")


def save_checkpoint_v3(
    path: Path,
    *,
    identity: ExperimentIdentityV3,
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: Any,
    sampler_state: dict[str, int],
    global_step: int,
) -> None:
    identity.validate()
    if global_step < 0:
        raise ValueError("global_step must be non-negative")
    payload = {
        "format": CHECKPOINT_FORMAT_V3,
        "identity": asdict(identity),
        "model": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict() if scheduler is not None else None,
        "sampler": sampler_state,
        "global_step": global_step,
        "rng": {
            "python": random.getstate(),
            "numpy": np.random.get_state(),
            "torch": torch.get_rng_state(),
            "cuda": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    os.replace(temporary, path)


def load_checkpoint_v3(
    path: Path,
    *,
    expected_identity: ExperimentIdentityV3,
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: Any,
) -> tuple[dict[str, int], int]:
    expected_identity.validate()
    payload = torch.load(path, map_location="cpu")
    if payload.get("format") != CHECKPOINT_FORMAT_V3:
        raise ValueError("not an AIC V3 training checkpoint")
    actual_identity = ExperimentIdentityV3(**payload["identity"])
    mismatches = [
        key
        for key in asdict(expected_identity)
        if getattr(expected_identity, key) != getattr(actual_identity, key)
    ]
    if mismatches:
        raise ValueError(f"checkpoint experiment identity mismatch: {mismatches}")
    model.load_state_dict(payload["model"], strict=True)
    optimizer.load_state_dict(payload["optimizer"])
    if scheduler is not None:
        if payload["scheduler"] is None:
            raise ValueError("checkpoint does not contain scheduler state")
        scheduler.load_state_dict(payload["scheduler"])
    elif payload["scheduler"] is not None:
        raise ValueError("checkpoint contains scheduler state but trainer does not")
    random.setstate(payload["rng"]["python"])
    np.random.set_state(payload["rng"]["numpy"])
    torch.set_rng_state(payload["rng"]["torch"])
    if payload["rng"]["cuda"] is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(payload["rng"]["cuda"])
    return dict(payload["sampler"]), int(payload["global_step"])
