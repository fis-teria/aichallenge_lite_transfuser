"""Trusted-local checkpoint contract for the proposed time model."""
from __future__ import annotations

import os
import random
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .time_config_v1 import TimeModelConfig
from aic_transfuser_lite.data.time_split_v1 import validate_time_split
from aic_transfuser_lite.data.time_split_v1 import content_sha256

TIME_CHECKPOINT_FORMAT = "aic_time_training_checkpoint_v1"


@dataclass(frozen=True)
class TimeCheckpointIdentity:
    split_manifest_sha256: str
    teacher_manifest_sha256: str
    source_sha256: str
    lineage: str
    source_git_commit: str = ""

    def validate(self) -> None:
        for name, value in self.__dict__.items():
            if name == "source_git_commit" and value == "":
                continue
            if type(value) is not str or not value:
                raise ValueError(f"checkpoint identity {name} is required")
        for name in ("split_manifest_sha256", "teacher_manifest_sha256", "source_sha256"):
            value = getattr(self, name).lower()
            if len(value) != 64 or any(ch not in "0123456789abcdef" for ch in value):
                raise ValueError(f"checkpoint identity {name} must be SHA-256")
        if self.source_git_commit and (
            len(self.source_git_commit) != 40
            or any(ch not in "0123456789abcdefABCDEF" for ch in self.source_git_commit)
        ):
            raise ValueError("source_git_commit must be a 40-character hex SHA")


def _manifest_sha(value: dict[str, Any]) -> str:
    return content_sha256(value)


def _validate_manifest(value: dict[str, Any] | None, expected: str, name: str) -> None:
    if value is None:
        return
    if type(value) is not dict:
        raise ValueError("manifest must be a dictionary")
    declared = value.get("manifest_sha256")
    if name == "split":
        validate_time_split(value)
        actual = _manifest_sha({key: item for key, item in value.items()
                                if key not in {"manifest_sha256", "sources_verified"}})
    else:
        actual = _manifest_sha({key: item for key, item in value.items() if key != "manifest_sha256"})
    if declared != actual or declared != expected:
        raise ValueError(f"{name} manifest digest mismatch")


def _rng_state() -> dict[str, Any]:
    state = np.random.get_state()
    return {
        "python": random.getstate(),
        "numpy": (state[0], state[1], int(state[2]), int(state[3]), float(state[4])),
        "torch": torch.get_rng_state(),
        "cuda": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
    }


def _restore_rng(value: dict[str, Any]) -> None:
    random.setstate(value["python"])
    name, keys, pos, has_gauss, cached = value["numpy"]
    np.random.set_state((str(name), np.asarray(keys, dtype=np.uint32), int(pos), int(has_gauss), float(cached)))
    torch.set_rng_state(value["torch"])
    if value.get("cuda") is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(value["cuda"])


def save_time_checkpoint(
    path: str | Path, *, config: TimeModelConfig, identity: TimeCheckpointIdentity,
    model: torch.nn.Module, optimizer: torch.optim.Optimizer | None,
    scheduler: Any, epoch: int, global_step: int,
    split_manifest: dict[str, Any] | None = None,
    teacher_manifest: dict[str, Any] | None = None,
    next_anchor_index: int = 0,
    training_state: dict[str, Any] | None = None,
) -> None:
    config.validate(); identity.validate()
    from aic_transfuser_lite.models.time_path_v1 import TimePathV1
    if not isinstance(model, TimePathV1):
        raise TypeError("Time checkpoint requires TimePathV1 model")
    _validate_model_config(model, config)
    _validate_manifest(split_manifest, identity.split_manifest_sha256, "split")
    _validate_manifest(teacher_manifest, identity.teacher_manifest_sha256, "teacher")
    if type(epoch) is not int or epoch < 0 or type(global_step) is not int or global_step < 0:
        raise ValueError("epoch/global_step must be non-negative ints")
    if type(next_anchor_index) is not int or next_anchor_index < 0:
        raise ValueError("next_anchor_index must be a non-negative int")
    if training_state is not None and type(training_state) is not dict:
        raise TypeError("training_state must be a dict")
    if training_state is not None:
        json.dumps(training_state, allow_nan=False)
    payload = {
        "format": TIME_CHECKPOINT_FORMAT, "config": config.to_dict(), "identity": identity.__dict__.copy(),
        "split_manifest": split_manifest, "teacher_manifest": teacher_manifest,
        "model": model.state_dict(), "optimizer": None if optimizer is None else optimizer.state_dict(),
        "scheduler": None if scheduler is None else scheduler.state_dict(),
        "epoch": epoch, "global_step": global_step, "rng": _rng_state(),
        "next_anchor_index": next_anchor_index, "training_state": training_state,
        "optimizer_type": _class_name(optimizer), "scheduler_type": _class_name(scheduler),
        "optimizer_parameters": _optimizer_names(model, optimizer),
    }
    target = Path(path); target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    torch.save(payload, temporary)
    os.replace(temporary, target)


def load_time_checkpoint(
    path: str | Path, *, config: TimeModelConfig, identity: TimeCheckpointIdentity,
    model: torch.nn.Module, optimizer: torch.optim.Optimizer | None = None,
    scheduler: Any = None, mode: str = "resume",
) -> tuple[int, int]:
    """Load a trusted local file; ``resume`` restores optimizer/RNG, ``finetune`` does not."""
    config.validate(); identity.validate()
    from aic_transfuser_lite.models.time_path_v1 import TimePathV1
    if not isinstance(model, TimePathV1):
        raise TypeError("Time checkpoint requires TimePathV1 model")
    _validate_model_config(model, config)
    if mode not in {"resume", "finetune"}:
        raise ValueError("mode must be resume or finetune")
    payload = torch.load(Path(path), map_location="cpu", weights_only=False)
    if type(payload) is not dict or payload.get("format") != TIME_CHECKPOINT_FORMAT:
        raise ValueError("not a TimePathV1 checkpoint")
    if TimeModelConfig.from_dict(payload.get("config", {})).to_dict() != config.to_dict():
        raise ValueError("time checkpoint config mismatch")
    if payload.get("identity") != identity.__dict__:
        raise ValueError("time checkpoint identity mismatch")
    _validate_progress(payload)
    _validate_manifest(payload.get("split_manifest"), identity.split_manifest_sha256, "split")
    _validate_manifest(payload.get("teacher_manifest"), identity.teacher_manifest_sha256, "teacher")
    if mode == "resume":
        if optimizer is None or payload.get("optimizer") is None:
            raise ValueError("resume requires optimizer state")
        if len(optimizer.param_groups) != len(payload["optimizer"].get("param_groups", [])):
            raise ValueError("optimizer parameter-group structure mismatch")
        if (_class_name(optimizer) != payload.get("optimizer_type") or
                _optimizer_names(model, optimizer) != payload.get("optimizer_parameters") or
                _class_name(scheduler) != payload.get("scheduler_type")):
            raise ValueError("optimizer/scheduler type or parameter ownership mismatch")
        if any(len(a["params"]) != len(b["params"]) for a,b in
               zip(optimizer.param_groups,payload["optimizer"]["param_groups"],strict=True)):
            raise ValueError("optimizer parameter-group length mismatch")
        if scheduler is not None and payload.get("scheduler") is None:
            raise ValueError("resume scheduler state missing")
        if scheduler is None and payload.get("scheduler") is not None:
            raise ValueError("resume checkpoint has scheduler but caller does not")
        _validate_rng(payload["rng"])
    current = model.state_dict()
    saved = payload["model"]
    if type(saved) is not dict and not isinstance(saved, dict):
        raise ValueError("invalid time state dictionary")
    if set(current) != set(saved):
        raise ValueError("time checkpoint state keys mismatch")
    # Validate every key before load_state_dict can modify any live parameter.
    model.set_extra_state(saved["_extra_state"])
    for name,value in current.items():
        if isinstance(value,torch.Tensor):
            other=saved[name]
            if (not isinstance(other,torch.Tensor) or other.shape != value.shape or other.dtype != value.dtype
                    or not torch.isfinite(other).all()):
                raise ValueError(f"time checkpoint tensor mismatch: {name}")
    model.load_state_dict(payload["model"], strict=True)
    if mode == "finetune":
        return 0, 0
    optimizer.load_state_dict(payload["optimizer"])
    if scheduler is not None:
        scheduler.load_state_dict(payload["scheduler"])
    _restore_rng(payload["rng"])
    return int(payload["epoch"]), int(payload["global_step"])


def inspect_time_checkpoint(path: str | Path, *, config: TimeModelConfig,
                            identity: TimeCheckpointIdentity) -> dict[str, Any]:
    """Validate metadata and return the resume cursor without constructing a model."""
    config.validate(); identity.validate()
    payload = torch.load(Path(path), map_location="cpu", weights_only=False)
    if type(payload) is not dict or payload.get("format") != TIME_CHECKPOINT_FORMAT:
        raise ValueError("not a TimePathV1 checkpoint")
    if TimeModelConfig.from_dict(payload.get("config", {})).to_dict() != config.to_dict():
        raise ValueError("time checkpoint config mismatch")
    if payload.get("identity") != identity.__dict__:
        raise ValueError("time checkpoint identity mismatch")
    _validate_progress(payload)
    _validate_manifest(payload.get("split_manifest"), identity.split_manifest_sha256, "split")
    _validate_manifest(payload.get("teacher_manifest"), identity.teacher_manifest_sha256, "teacher")
    return {"epoch": int(payload["epoch"]), "global_step": int(payload["global_step"]),
            "next_anchor_index": int(payload.get("next_anchor_index", 0)),
            "training_state": payload.get("training_state")}


def _validate_model_config(model: torch.nn.Module, config: TimeModelConfig) -> None:
    state = model.get_extra_state()
    if state.get("command_history") is not config.use_command_history:
        raise ValueError("model/config command history mismatch")
    if state.get("history") != config.history_version:
        raise ValueError("model/config history contract mismatch")
    backbone = state.get("backbone")
    if not isinstance(backbone, dict) or int(backbone.get("trajectory_steps", -1)) != 15:
        raise ValueError("unexpected TimePath backbone configuration")
    for key, expected in (("image_height", config.image_height), ("image_width", config.image_width),
                          ("lidar_points", config.lidar_points), ("ego_dim", config.ego_dim),
                          ("hidden_dim", config.hidden_dim), ("max_sensor_history", config.max_sensor_history),
                          ("max_ego_history", config.max_ego_history)):
        if int(backbone.get(key, -1)) != expected:
            raise ValueError(f"model/config mismatch: {key}")
    if tuple(backbone.get("camera_tokens_hw", ())) != config.camera_tokens_hw:
        raise ValueError("model/config mismatch: camera_tokens_hw")
    if int(backbone.get("lidar_tokens", -1)) != config.lidar_tokens:
        raise ValueError("model/config mismatch: lidar_tokens")
    for key in ("fusion_depth", "fusion_heads", "lidar_angle_min_rad"):
        if backbone.get(key) != getattr(config,key):
            raise ValueError(f"model/config mismatch: {key}")


def _class_name(value: Any) -> str | None:
    return None if value is None else f"{type(value).__module__}.{type(value).__qualname__}"


def _optimizer_names(model: torch.nn.Module, optimizer: torch.optim.Optimizer | None) -> list[list[str]] | None:
    if optimizer is None:
        return None
    names = {id(p): name for name,p in model.named_parameters()}
    if any(id(p) not in names for g in optimizer.param_groups for p in g["params"]):
        raise ValueError("optimizer contains parameters outside this model")
    return [[names[id(p)] for p in g["params"]] for g in optimizer.param_groups]


def _validate_progress(payload: dict[str, Any]) -> None:
    for key in ("epoch", "global_step", "next_anchor_index"):
        if type(payload.get(key)) is not int or payload[key] < 0:
            raise ValueError(f"invalid checkpoint cursor: {key}")
    state = payload.get("training_state")
    if state is not None:
        if type(state) is not dict:
            raise ValueError("training_state must be dict")
        json.dumps(state, allow_nan=False)


def _validate_rng(value: dict[str, Any]) -> None:
    # Local generators validate CPU states without changing global randomness.
    random.Random().setstate(value["python"])
    np.random.RandomState().set_state(value["numpy"])
    torch.Generator().set_state(value["torch"])
    states = value.get("cuda")
    if states is not None and (not torch.cuda.is_available() or len(states) != torch.cuda.device_count()):
        raise ValueError("resume CUDA RNG topology mismatch; use explicit finetune for a different platform")
