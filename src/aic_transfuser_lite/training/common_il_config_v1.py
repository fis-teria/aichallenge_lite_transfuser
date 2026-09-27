"""Resolve the two IL arms without allowing hidden output/loss/budget changes."""
from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path
from typing import Any

from .common_il_v1 import CommonILLossWeights
from .time_config_v1 import TimeModelConfig


def load_common_il_experiment(path: Path, *, workspace: Path,
                              dinov3_model_config: Path | None = None) -> tuple[TimeModelConfig, dict[str, Any]]:
    value = json.loads(path.read_text(encoding="utf-8"))
    keys = {"format", "time_backbone_config", "camera_encoder", "initialization_seed", "sampler_seed",
            "loss_weights", "pilot", "dataset"}
    if not isinstance(value, dict) or set(value) != keys or value["format"] != "common_il_experiment_v1":
        raise ValueError("common IL experiment schema mismatch")
    for key in ("initialization_seed", "sampler_seed"):
        if type(value[key]) is not int or value[key] < 0:
            raise ValueError(f"{key} must be nonnegative integer")
    CommonILLossWeights(**value["loss_weights"])
    plan = value["pilot"]
    if set(plan) != {"max_optimizer_steps", "batch_size", "learning_rate", "weight_decay", "max_grad_norm"}:
        raise ValueError("pilot budget schema mismatch")
    for key, cap in (("max_optimizer_steps", 100), ("batch_size", 16)):
        if type(plan[key]) is not int or not 1 <= plan[key] <= cap:
            raise ValueError(f"bounded pilot {key} must be 1..{cap}")
    for key in ("learning_rate", "max_grad_norm", "weight_decay"):
        if type(plan[key]) is not float or not math.isfinite(plan[key]) or plan[key] < 0:
            raise ValueError(f"invalid pilot {key}")
        if key != "weight_decay" and plan[key] == 0:
            raise ValueError(f"positive pilot {key} required")
    data = value["dataset"]
    if (set(data) != {"split", "run_ids", "samples_per_run", "stop_annotation"}
            or data["split"] != "train" or data["stop_annotation"] != "explicit_only"
            or type(data["run_ids"]) is not list or not 1 <= len(data["run_ids"]) <= 2
            or any(type(r) is not str or not r for r in data["run_ids"])
            or len(set(data["run_ids"])) != len(data["run_ids"])
            or type(data["samples_per_run"]) is not int or not 4 <= data["samples_per_run"] <= 128):
        raise ValueError("bounded train-run dataset settings required")
    model = TimeModelConfig.from_dict(json.loads((workspace / value["time_backbone_config"]).read_text(encoding="utf-8")))
    model = replace(model, camera_encoder=value["camera_encoder"])
    model.validate()
    if dinov3_model_config is not None:
        assets = TimeModelConfig.from_dict(json.loads(dinov3_model_config.read_text(encoding="utf-8")))
        if (model.camera_encoder.get("backbone") != "dinov3_vits16"
                or assets.camera_encoder is None or assets.camera_encoder.get("backbone") != "dinov3_vits16"
                or replace(assets, camera_encoder=None).to_dict() != replace(model, camera_encoder=None).to_dict()):
            raise ValueError("DINO local config must change only the matching camera assets")
        model = replace(model, camera_encoder=assets.camera_encoder)
    return model, value
