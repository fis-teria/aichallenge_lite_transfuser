from __future__ import annotations

import os
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

from aic_transfuser_lite.contracts.behavior_v1 import BEHAVIOR_ONTOLOGY_V1


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
    numpy_rng = np.random.get_state()
    payload = {
        "format": CHECKPOINT_FORMAT_V3,
        "identity": asdict(identity),
        "model": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict() if scheduler is not None else None,
        "sampler": sampler_state,
        "global_step": global_step,
        "behavior_ontology": (
            BEHAVIOR_ONTOLOGY_V1
            if getattr(model, "behavior_head", None) is not None
            else None
        ),
        "rng": {
            "python": random.getstate(),
            "numpy": {
                "bit_generator": numpy_rng[0],
                "state": numpy_rng[1].tolist(),
                "position": int(numpy_rng[2]),
                "has_gauss": int(numpy_rng[3]),
                "cached_gaussian": float(numpy_rng[4]),
            },
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
    payload = torch.load(path, map_location="cpu", weights_only=True)
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
    if (
        getattr(model, "behavior_head", None) is not None
        and payload.get("behavior_ontology") != BEHAVIOR_ONTOLOGY_V1
    ):
        raise ValueError("checkpoint behavior ontology mismatch")
    model.load_state_dict(payload["model"], strict=True)
    optimizer.load_state_dict(payload["optimizer"])
    if scheduler is not None:
        if payload["scheduler"] is None:
            raise ValueError("checkpoint does not contain scheduler state")
        scheduler.load_state_dict(payload["scheduler"])
    elif payload["scheduler"] is not None:
        raise ValueError("checkpoint contains scheduler state but trainer does not")
    random.setstate(payload["rng"]["python"])
    numpy_rng = payload["rng"]["numpy"]
    np.random.set_state(
        (
            str(numpy_rng["bit_generator"]),
            np.asarray(numpy_rng["state"], dtype=np.uint32),
            int(numpy_rng["position"]),
            int(numpy_rng["has_gauss"]),
            float(numpy_rng["cached_gaussian"]),
        )
    )
    torch.set_rng_state(payload["rng"]["torch"])
    if payload["rng"]["cuda"] is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(payload["rng"]["cuda"])
    return dict(payload["sampler"]), int(payload["global_step"])
