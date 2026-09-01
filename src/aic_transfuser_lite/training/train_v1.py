from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml
from torch import nn
from torch.utils.data import DataLoader, RandomSampler

from aic_transfuser_lite.config import load_v1_config
from aic_transfuser_lite.data.dataset import DrivingDataset
from aic_transfuser_lite.data.dataset_v2 import DrivingDatasetV2
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.training.checkpoint_v1 import (
    V1_CHECKPOINT_FORMAT,
    load_v1_checkpoint,
    save_v1_checkpoint,
)
from aic_transfuser_lite.training.losses_v1 import compute_v1_multitask_loss
from aic_transfuser_lite.training.metrics import (
    V1MetricAccumulator,
    controller_proxy_steering,
)
from aic_transfuser_lite.training.sampler import (
    build_capped_curvature_recovery_plan,
    seeded_weighted_sampler,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Versioned, reproducible TransFuser Lite v1 trainer"
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--train-index", required=True)
    parser.add_argument("--val-index", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--resume", default=None)
    parser.add_argument(
        "--stop-after-epoch",
        type=int,
        default=None,
        help="Operational pause hook; it does not alter the resolved training config",
    )
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    return parser.parse_args()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_text(path: str | Path, value: str) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(value, encoding="utf-8")
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_write_json(path: str | Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
    )


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def seed_worker(worker_id: int) -> None:
    del worker_id
    worker_seed = torch.initial_seed() % (2**32)
    random.seed(worker_seed)
    np.random.seed(worker_seed)


def seeded_random_sampler(
    dataset: Any, seed: int
) -> tuple[RandomSampler, torch.Generator, list[int]]:
    generator = torch.Generator().manual_seed(seed)
    sampler = RandomSampler(dataset, generator=generator)
    state = generator.get_state()
    first_epoch_order = [int(index) for index in sampler]
    generator.set_state(state)
    return sampler, generator, first_epoch_order


def order_sha256(order: list[int]) -> str:
    values = np.asarray(order, dtype=np.int64)
    return hashlib.sha256(values.tobytes()).hexdigest()


def resolve_device(requested: str) -> torch.device:
    if requested == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("--device cuda requested but CUDA is unavailable")
        return torch.device("cuda")
    if requested == "cpu":
        return torch.device("cpu")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def git_revision(root: Path) -> str:
    if not (root / ".git").exists():
        return "no-git-repository"
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "git-revision-error"


def _norm_parameter_names(model: nn.Module) -> set[str]:
    norm_types = (
        nn.BatchNorm1d,
        nn.BatchNorm2d,
        nn.BatchNorm3d,
        nn.GroupNorm,
        nn.InstanceNorm1d,
        nn.InstanceNorm2d,
        nn.InstanceNorm3d,
        nn.LayerNorm,
    )
    result: set[str] = set()
    for module_name, module in model.named_modules():
        if not isinstance(module, norm_types):
            continue
        for parameter_name, _ in module.named_parameters(recurse=False):
            result.add(
                f"{module_name}.{parameter_name}" if module_name else parameter_name
            )
    return result


def build_v1_optimizer(
    model: nn.Module, config: dict[str, Any]
) -> tuple[torch.optim.AdamW, list[dict[str, Any]]]:
    training = config["training"]
    norm_names = _norm_parameter_names(model)
    grouped: dict[str, list[nn.Parameter]] = {
        "main_decay": [],
        "main_no_decay": [],
        "backbone_decay": [],
        "backbone_no_decay": [],
    }
    grouped_names: dict[str, list[str]] = {name: [] for name in grouped}
    seen: set[int] = set()
    for name, parameter in model.named_parameters():
        if id(parameter) in seen:
            raise RuntimeError(f"Duplicate parameter object encountered: {name}")
        seen.add(id(parameter))
        backbone = name.startswith("camera.backbone.")
        no_decay = (
            name.rsplit(".", 1)[-1].endswith("bias")
            or name in norm_names
            or "position" in name.lower()
            or "positional" in name.lower()
        )
        group_name = (
            ("backbone" if backbone else "main")
            + ("_no_decay" if no_decay else "_decay")
        )
        grouped[group_name].append(parameter)
        grouped_names[group_name].append(name)

    expected = {id(parameter) for parameter in model.parameters()}
    if seen != expected:
        raise RuntimeError("Optimizer parameter grouping did not cover the model exactly")

    parameter_groups: list[dict[str, Any]] = []
    metadata: list[dict[str, Any]] = []
    for name in (
        "main_decay",
        "main_no_decay",
        "backbone_decay",
        "backbone_no_decay",
    ):
        parameters = grouped[name]
        if not parameters:
            continue
        backbone = name.startswith("backbone")
        no_decay = name.endswith("no_decay")
        learning_rate = float(
            training[
                "backbone_learning_rate" if backbone else "main_learning_rate"
            ]
        )
        weight_decay = 0.0 if no_decay else float(training["weight_decay"])
        parameter_groups.append(
            {
                "params": parameters,
                "lr": learning_rate,
                "initial_lr": learning_rate,
                "weight_decay": weight_decay,
                "group_name": name,
            }
        )
        metadata.append(
            {
                "group_name": name,
                "learning_rate": learning_rate,
                "weight_decay": weight_decay,
                "parameter_tensors": len(parameters),
                "parameter_values": int(sum(item.numel() for item in parameters)),
                "parameter_names": grouped_names[name],
            }
        )
    optimizer = torch.optim.AdamW(parameter_groups)
    return optimizer, metadata


class WarmupCosineScheduler:
    """Step-based warmup followed by cosine decay with serializable state."""

    def __init__(
        self,
        optimizer: torch.optim.Optimizer,
        *,
        total_steps: int,
        warmup_fraction: float,
        min_lr_ratio: float,
    ) -> None:
        if total_steps < 1:
            raise ValueError("total_steps must be positive")
        self.optimizer = optimizer
        self.total_steps = int(total_steps)
        self.warmup_steps = int(math.ceil(total_steps * warmup_fraction))
        self.min_lr_ratio = float(min_lr_ratio)
        self.step_count = 0
        self.base_lrs = [float(group["initial_lr"]) for group in optimizer.param_groups]
        self._apply()

    def _factor(self) -> float:
        if self.warmup_steps and self.step_count < self.warmup_steps:
            return float(self.step_count + 1) / float(self.warmup_steps)
        decay_steps = max(self.total_steps - self.warmup_steps, 1)
        progress = min(
            max(self.step_count - self.warmup_steps, 0) / decay_steps,
            1.0,
        )
        cosine = 0.5 * (1.0 + math.cos(math.pi * progress))
        return self.min_lr_ratio + (1.0 - self.min_lr_ratio) * cosine

    def _apply(self) -> None:
        factor = self._factor()
        for group, base_lr in zip(self.optimizer.param_groups, self.base_lrs):
            group["lr"] = base_lr * factor

    def step(self) -> None:
        self.step_count += 1
        self._apply()

    def get_last_lr(self) -> list[float]:
        return [float(group["lr"]) for group in self.optimizer.param_groups]

    def state_dict(self) -> dict[str, Any]:
        return {
            "total_steps": self.total_steps,
            "warmup_steps": self.warmup_steps,
            "min_lr_ratio": self.min_lr_ratio,
            "step_count": self.step_count,
            "base_lrs": self.base_lrs,
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        for name in ("total_steps", "warmup_steps", "min_lr_ratio", "base_lrs"):
            if state[name] != getattr(self, name):
                raise ValueError(f"Scheduler contract mismatch for {name}")
        self.step_count = int(state["step_count"])
        if not 0 <= self.step_count <= self.total_steps:
            raise ValueError("Scheduler step_count is outside the configured run")
        self._apply()


def move_batch(
    batch: dict[str, torch.Tensor], device: torch.device
) -> dict[str, torch.Tensor]:
    return {
        key: value.to(device, non_blocking=(device.type == "cuda"))
        for key, value in batch.items()
    }


def forward_model(
    model: nn.Module, batch: dict[str, torch.Tensor]
) -> dict[str, torch.Tensor]:
    return model(batch["image"], batch["lidar"], batch["ego"])


def _set_backbone_trainable(model: nn.Module, trainable: bool) -> None:
    camera = getattr(model, "camera", None)
    backbone = getattr(camera, "backbone", None)
    if backbone is None:
        raise RuntimeError("v1 trainer requires model.camera.backbone")
    for parameter in backbone.parameters():
        parameter.requires_grad_(trainable)


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    *,
    optimizer: torch.optim.Optimizer | None,
    scheduler: WarmupCosineScheduler | None,
    scaler: torch.amp.GradScaler,
    device: torch.device,
    loss_weights: dict[str, Any],
    horizon_weights: list[float],
    grad_clip_norm: float,
    amp_enabled: bool,
    backbone_frozen: bool,
    global_step: int,
    num_waypoints: int,
    straight_curvature_threshold_per_m: float,
    sharp_curvature_threshold_per_m: float,
    controller_wheelbase_m: float,
) -> tuple[dict[str, Any], int]:
    training = optimizer is not None
    model.train(training)
    if training and backbone_frozen:
        model.camera.backbone.eval()

    count = 0
    total_loss = 0.0
    raw_totals: dict[str, float] = {}
    weighted_totals: dict[str, float] = {}
    metric_accumulator = V1MetricAccumulator(
        num_waypoints=num_waypoints,
        straight_threshold_per_m=straight_curvature_threshold_per_m,
        sharp_threshold_per_m=sharp_curvature_threshold_per_m,
        controller_wheelbase_m=controller_wheelbase_m,
    )
    grad_norms: list[float] = []

    for batch in loader:
        batch = move_batch(batch, device)
        batch_size = int(batch["ego"].shape[0])
        if training:
            assert optimizer is not None and scheduler is not None
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=amp_enabled,
            ):
                outputs = forward_model(model, batch)
                loss, raw, weighted = compute_v1_multitask_loss(
                    outputs,
                    batch,
                    loss_weights,
                    horizon_weights,
                )
        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            grad_norm = torch.nn.utils.clip_grad_norm_(
                model.parameters(), grad_clip_norm
            )
            grad_norms.append(float(grad_norm.detach().cpu()))
            scaler.step(optimizer)
            scaler.update()
            scheduler.step()
            global_step += 1

        with torch.no_grad():
            metric_accumulator.update(outputs, batch)

        total_loss += float(loss.detach().cpu()) * batch_size
        for name, value in raw.items():
            raw_totals[name] = raw_totals.get(name, 0.0) + float(
                value.detach().cpu()
            ) * batch_size
        for name, value in weighted.items():
            weighted_totals[name] = weighted_totals.get(name, 0.0) + float(
                value.detach().cpu()
            ) * batch_size
        count += batch_size

    denominator = max(count, 1)
    metrics: dict[str, Any] = {
        "sample_count": count,
        "loss_total": total_loss / denominator,
        "raw_losses": {
            name: value / denominator for name, value in sorted(raw_totals.items())
        },
        "weighted_losses": {
            name: value / denominator
            for name, value in sorted(weighted_totals.items())
        },
    }
    metrics.update(metric_accumulator.finalize())
    if training:
        metrics["grad_norm_mean"] = (
            sum(grad_norms) / len(grad_norms) if grad_norms else 0.0
        )
        metrics["grad_norm_max"] = max(grad_norms, default=0.0)
        metrics["amp_scale"] = float(scaler.get_scale())
    return metrics, global_step


def _dataset_summary(dataset: Any) -> dict[str, Any]:
    run_ids = (
        sorted(str(value) for value in dataset.frame["run_id"].dropna().unique())
        if "run_id" in dataset.frame.columns
        else []
    )
    target_speed = dataset.frame["target_speed_mps"].astype(float)
    stop_positive_count = (
        int((dataset.frame["stop_flag"].astype(float) > 0.5).sum())
        if "stop_flag" in dataset.frame.columns
        else 0
    )
    summary = {
        "path": str(dataset.index_path.resolve()),
        "sha256": sha256_file(dataset.index_path),
        "sample_count": len(dataset),
        "run_ids": run_ids,
        "target_speed_std_mps": float(target_speed.std(ddof=0)),
        "stop_positive_count": stop_positive_count,
        "dataset_loader": type(dataset).__name__,
    }
    metadata_path = getattr(dataset, "metadata_path", None)
    if metadata_path is not None:
        summary["metadata_path"] = str(Path(metadata_path).resolve())
        summary["metadata_sha256"] = sha256_file(metadata_path)
    if "label_provenance" in dataset.frame.columns:
        summary["label_provenance_counts"] = {
            str(name): int(count)
            for name, count in dataset.frame["label_provenance"].value_counts().items()
        }
    return summary


def _dataset_controller_wheelbase_m(dataset: Any) -> float:
    metadata = getattr(dataset, "metadata", None)
    if not isinstance(metadata, dict):
        return 1.087
    provenance = metadata.get("vehicle_config_provenance")
    if not isinstance(provenance, dict):
        raise ValueError("Dataset v2 metadata lacks vehicle_config_provenance")
    value = provenance.get("wheelbase_m")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Dataset v2 metadata wheelbase_m must be numeric")
    wheelbase_m = float(value)
    if not 0.0 < wheelbase_m < 10.0:
        raise ValueError("Dataset v2 metadata wheelbase_m is outside (0, 10) metres")
    return wheelbase_m


def _rng_state() -> dict[str, Any]:
    return {
        "python": random.getstate(),
        "numpy": np.random.get_state(),
        "torch_cpu": torch.get_rng_state(),
        "torch_cuda": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [],
    }


def _restore_rng_state(state: dict[str, Any]) -> None:
    random.setstate(state["python"])
    np.random.set_state(state["numpy"])
    torch.set_rng_state(state["torch_cpu"])
    if torch.cuda.is_available() and state["torch_cuda"]:
        torch.cuda.set_rng_state_all(state["torch_cuda"])


def _checkpoint_payload(
    *,
    role: str,
    model: nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: WarmupCosineScheduler,
    scaler: torch.amp.GradScaler,
    config: dict[str, Any],
    epoch: int,
    global_step: int,
    resolved_config_sha256: str,
    dataset_manifest_sha256: str,
    best_metrics: dict[str, float],
    selected_checkpoints: dict[str, Any],
    early_best_ade: float,
    epochs_without_improvement: int,
    sampler_generator: torch.Generator,
    train_worker_generator: torch.Generator,
    val_worker_generator: torch.Generator,
    amp_effective: bool,
    optimizer_group_contract: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "format_version": V1_CHECKPOINT_FORMAT,
        "checkpoint_role": role,
        "model": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "scaler": scaler.state_dict(),
        "config": config,
        "resolved_config": config,
        "epoch": epoch,
        "global_step": global_step,
        "resolved_config_sha256": resolved_config_sha256,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "best_metrics": best_metrics,
        "selected_checkpoints": selected_checkpoints,
        "early_stopping": {
            "best_ade_m": early_best_ade,
            "epochs_without_improvement": epochs_without_improvement,
        },
        "rng_state": _rng_state(),
        "sampler_generator_state": sampler_generator.get_state(),
        "train_worker_generator_state": train_worker_generator.get_state(),
        "val_worker_generator_state": val_worker_generator.get_state(),
        "amp_effective": amp_effective,
        "optimizer_group_contract": optimizer_group_contract,
    }


def train_v1(
    *,
    config_path: str | Path,
    train_index: str | Path,
    val_index: str | Path,
    output: str | Path,
    resume: str | Path | None = None,
    stop_after_epoch: int | None = None,
    requested_device: str = "auto",
    invocation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    config_path = Path(config_path).resolve()
    train_index = Path(train_index).resolve()
    val_index = Path(val_index).resolve()
    output_dir = Path(output).resolve()
    resume_path = Path(resume).resolve() if resume is not None else None

    config = load_v1_config(config_path)
    training = config["training"]
    epochs = int(training["epochs"])
    if stop_after_epoch is not None and not 1 <= stop_after_epoch <= epochs:
        raise ValueError("stop_after_epoch must be within configured epochs")

    if resume_path is None:
        if output_dir.exists() and any(output_dir.iterdir()):
            raise FileExistsError(
                f"Refusing non-empty output without --resume: {output_dir}"
            )
        output_dir.mkdir(parents=True, exist_ok=True)
    else:
        if not output_dir.is_dir():
            raise FileNotFoundError(f"Resume output directory not found: {output_dir}")
        if not resume_path.is_file():
            raise FileNotFoundError(f"Resume checkpoint not found: {resume_path}")

    seed = int(config["project"]["seed"])
    set_seed(seed)
    device = resolve_device(requested_device)
    amp_requested = bool(training["mixed_precision"])
    amp_effective = amp_requested and device.type == "cuda"

    data_format_version = int(config["data"].get("format_version", 1))
    if data_format_version == 2:
        train_data = DrivingDatasetV2(train_index, config, training=True)
        val_data = DrivingDatasetV2(val_index, config, training=False)
    else:
        train_data = DrivingDataset(train_index, config)
        val_data = DrivingDataset(val_index, config)
    train_summary = _dataset_summary(train_data)
    val_summary = _dataset_summary(val_data)
    run_overlap = sorted(set(train_summary["run_ids"]) & set(val_summary["run_ids"]))
    if run_overlap:
        raise ValueError(f"train/validation run overlap detected: {run_overlap}")
    train_wheelbase_m = _dataset_controller_wheelbase_m(train_data)
    val_wheelbase_m = _dataset_controller_wheelbase_m(val_data)
    if not math.isclose(train_wheelbase_m, val_wheelbase_m, abs_tol=1e-9):
        raise ValueError("train/validation controller wheelbase contracts differ")
    controller_wheelbase_m = train_wheelbase_m
    heads = config["model"]["heads"]
    if heads["stop"] and train_summary["stop_positive_count"] == 0:
        raise ValueError("stop Head is enabled but the training split has zero positives")

    sampling_plan = None
    sampling_summary: dict[str, Any] = {"type": "seeded_random_without_replacement"}
    straight_curvature_threshold_per_m = 0.03
    sharp_curvature_threshold_per_m = 0.12
    if data_format_version == 2:
        sampler_cfg = training["sampler"]
        straight_curvature_threshold_per_m = float(
            sampler_cfg["straight_threshold_per_m"]
        )
        sharp_curvature_threshold_per_m = float(
            sampler_cfg["sharp_threshold_per_m"]
        )
        sampling_plan = build_capped_curvature_recovery_plan(
            train_data.frame,
            num_waypoints=int(config["data"]["num_waypoints"]),
            straight_threshold_per_m=straight_curvature_threshold_per_m,
            sharp_threshold_per_m=sharp_curvature_threshold_per_m,
            max_weight=float(sampler_cfg["max_weight"]),
            recovery_weight=float(sampler_cfg["recovery_weight"]),
        )
        sampling_summary = sampling_plan.summary

    warnings: list[str] = []
    if train_summary["target_speed_std_mps"] < 0.3:
        warnings.append(
            "training target_speed_mps std is below 0.3 m/s; speed Head may be a constant-prediction problem"
        )
    if amp_requested and not amp_effective:
        warnings.append("mixed precision requested but disabled because the resolved device is CPU")

    resolved_path = output_dir / "resolved_config.yaml"
    resolved_text = yaml.safe_dump(config, sort_keys=False, allow_unicode=True)
    dataset_manifest_path = output_dir / "dataset_manifest.json"
    dataset_manifest = {
        "train": train_summary,
        "validation": val_summary,
        "run_overlap": run_overlap,
        "training_sampling": sampling_summary,
    }
    dataset_manifest_text = (
        json.dumps(dataset_manifest, indent=2, ensure_ascii=False, allow_nan=False)
        + "\n"
    )
    if resume_path is None:
        atomic_write_text(resolved_path, resolved_text)
        atomic_write_text(dataset_manifest_path, dataset_manifest_text)
    else:
        if resolved_path.read_text(encoding="utf-8") != resolved_text:
            raise ValueError("Resolved config differs from the original run")
        if dataset_manifest_path.read_text(encoding="utf-8") != dataset_manifest_text:
            raise ValueError("Dataset manifest differs from the original run")
    resolved_config_sha256 = sha256_file(resolved_path)
    dataset_manifest_sha256 = sha256_file(dataset_manifest_path)

    data_order_seed = int(training["data_order_seed"])
    if sampling_plan is None:
        train_sampler, sampler_generator, first_epoch_order = seeded_random_sampler(
            train_data, data_order_seed
        )
    else:
        train_sampler, sampler_generator, first_epoch_order = seeded_weighted_sampler(
            train_data, sampling_plan.weights, data_order_seed
        )
    train_worker_generator = torch.Generator().manual_seed(data_order_seed + 1)
    val_worker_generator = torch.Generator().manual_seed(data_order_seed + 2)
    loader_kwargs: dict[str, Any] = {
        "batch_size": int(training["batch_size"]),
        "num_workers": int(training["num_workers"]),
        "worker_init_fn": seed_worker,
        "pin_memory": bool(training["pin_memory"]),
    }
    if int(training["num_workers"]) > 0:
        loader_kwargs["persistent_workers"] = bool(training["persistent_workers"])
        loader_kwargs["prefetch_factor"] = int(training["prefetch_factor"])
    train_loader = DataLoader(
        train_data,
        sampler=train_sampler,
        generator=train_worker_generator,
        **loader_kwargs,
    )
    val_loader = DataLoader(
        val_data,
        shuffle=False,
        generator=val_worker_generator,
        **loader_kwargs,
    )
    data_order = {
        "data_order_seed": data_order_seed,
        "sampler": sampling_summary,
        "first_epoch_order_sha256": order_sha256(first_epoch_order),
        "first_batch_indices": first_epoch_order[: int(training["batch_size"])],
    }
    if resume_path is None:
        atomic_write_json(output_dir / "data_order.json", data_order)
    elif json.loads((output_dir / "data_order.json").read_text(encoding="utf-8")) != data_order:
        raise ValueError("Data-order contract differs from the original run")

    model = build_model(config).to(device)
    optimizer, optimizer_group_contract = build_v1_optimizer(model, config)
    total_steps = epochs * len(train_loader)
    scheduler = WarmupCosineScheduler(
        optimizer,
        total_steps=total_steps,
        warmup_fraction=float(training["warmup_fraction"]),
        min_lr_ratio=float(training["min_lr_ratio"]),
    )
    scaler = torch.amp.GradScaler("cuda", enabled=amp_effective)

    history_path = output_dir / "history.json"
    history: list[dict[str, Any]] = []
    start_epoch = 1
    global_step = 0
    corner_metric_key = (
        "sharp_controller_proxy_mae_rad"
        if data_format_version == 2
        else "controller_proxy_mae_rad"
    )
    best_metrics = {
        "ade_m": float("inf"),
        corner_metric_key: float("inf"),
        "speed_mae_mps": float("inf"),
    }
    selected_checkpoints: dict[str, Any] = {}
    early_best_ade = float("inf")
    epochs_without_improvement = 0

    if resume_path is not None:
        # Keep serialized RNG states on CPU. load_state_dict transfers model and
        # optimizer tensors to their parameter devices, while torch.Generator
        # explicitly requires a CPU ByteTensor for its saved state.
        checkpoint = load_v1_checkpoint(resume_path, map_location="cpu")
        if checkpoint["resolved_config_sha256"] != resolved_config_sha256:
            raise ValueError("Resume checkpoint resolved-config hash differs")
        if checkpoint["dataset_manifest_sha256"] != dataset_manifest_sha256:
            raise ValueError("Resume checkpoint dataset-manifest hash differs")
        if checkpoint["resolved_config"] != config:
            raise ValueError("Resume checkpoint resolved config differs")
        model.load_state_dict(checkpoint["model"], strict=True)
        optimizer.load_state_dict(checkpoint["optimizer"])
        scheduler.load_state_dict(checkpoint["scheduler"])
        scaler.load_state_dict(checkpoint["scaler"])
        sampler_generator.set_state(checkpoint["sampler_generator_state"])
        train_worker_generator.set_state(checkpoint["train_worker_generator_state"])
        val_worker_generator.set_state(checkpoint["val_worker_generator_state"])
        _restore_rng_state(checkpoint["rng_state"])
        start_epoch = int(checkpoint["epoch"]) + 1
        global_step = int(checkpoint["global_step"])
        best_metrics = {
            name: float(value) for name, value in checkpoint["best_metrics"].items()
        }
        selected_checkpoints = dict(checkpoint["selected_checkpoints"])
        early_best_ade = float(checkpoint["early_stopping"]["best_ade_m"])
        epochs_without_improvement = int(
            checkpoint["early_stopping"]["epochs_without_improvement"]
        )
        history = json.loads(history_path.read_text(encoding="utf-8"))
        if not history or int(history[-1]["epoch"]) != start_epoch - 1:
            raise ValueError("History does not end at the resume checkpoint epoch")
        if global_step != scheduler.step_count:
            raise ValueError("Resume global_step and scheduler state differ")
        if start_epoch > epochs:
            raise ValueError("Resume checkpoint already reached configured epochs")
        if stop_after_epoch is not None and stop_after_epoch < start_epoch:
            raise ValueError("stop_after_epoch precedes the resume epoch")

    repo_root = Path(__file__).resolve().parents[3]
    source_paths = {
        "config": repo_root / "src/aic_transfuser_lite/config.py",
        "train_v1": Path(__file__).resolve(),
        "losses_v1": repo_root / "src/aic_transfuser_lite/training/losses_v1.py",
        "checkpoint_v1": repo_root / "src/aic_transfuser_lite/training/checkpoint_v1.py",
        "metrics": repo_root / "src/aic_transfuser_lite/training/metrics.py",
        "sampler": repo_root / "src/aic_transfuser_lite/training/sampler.py",
        "image_preprocess": repo_root / "src/aic_transfuser_lite/data/image_preprocess.py",
        "model_factory": repo_root / "src/aic_transfuser_lite/models/factory.py",
        "camera_encoder": repo_root / "src/aic_transfuser_lite/models/camera_encoder.py",
        "lidar_encoder": repo_root / "src/aic_transfuser_lite/models/lidar_encoder.py",
        "ego_encoder": repo_root / "src/aic_transfuser_lite/models/ego_encoder.py",
        "fusion": repo_root / "src/aic_transfuser_lite/models/fusion.py",
        "heads": repo_root / "src/aic_transfuser_lite/models/heads.py",
    }
    if data_format_version == 2:
        source_paths.update(
            {
                "dataset": repo_root / "src/aic_transfuser_lite/data/dataset_v2.py",
                "normalization": repo_root
                / "src/aic_transfuser_lite/data/normalization.py",
                "model": repo_root
                / "src/aic_transfuser_lite/models/transfuser_lite_v1.py",
            }
        )
    else:
        source_paths.update(
            {
                "dataset": repo_root / "src/aic_transfuser_lite/data/dataset.py",
                "model": repo_root
                / "src/aic_transfuser_lite/models/transfuser_lite.py",
            }
        )
    current_source_sha256 = {
        name: sha256_file(path) for name, path in source_paths.items()
    }
    camera = getattr(model, "camera", None)
    provenance_fn = getattr(camera, "pretrained_provenance", None)
    camera_pretrained_provenance = (
        provenance_fn() if callable(provenance_fn) else None
    )
    if (
        data_format_version == 2
        and bool(config["model"]["camera"]["pretrained"])
        and (
            not isinstance(camera_pretrained_provenance, dict)
            or camera_pretrained_provenance.get("sha256") is None
        )
    ):
        raise RuntimeError("Pretrained Camera weight provenance is incomplete")

    run_manifest_path = output_dir / "run_manifest.json"
    if run_manifest_path.exists():
        run_manifest = json.loads(run_manifest_path.read_text(encoding="utf-8"))
        if run_manifest["source_sha256"] != current_source_sha256:
            raise ValueError("Training source differs from the original run")
    else:
        run_manifest = {
            "format_version": "transfuser_lite_v1_run_manifest_v1",
            "created_at_utc": utc_now(),
            "repository": str(repo_root),
            "git_revision": git_revision(repo_root),
            "config_source": str(config_path),
            "config_source_sha256": sha256_file(config_path),
            "resolved_config_sha256": resolved_config_sha256,
            "dataset_manifest_sha256": dataset_manifest_sha256,
            "dataset_format_version": data_format_version,
            "metric_contract_version": "transfuser_lite_v1_metrics_v3",
            "controller_wheelbase_m": controller_wheelbase_m,
            "training_sampling": sampling_summary,
            "source_sha256": current_source_sha256,
            "device": str(device),
            "cuda_device": (
                torch.cuda.get_device_name(device) if device.type == "cuda" else None
            ),
            "torch_version": torch.__version__,
            "mixed_precision_requested": amp_requested,
            "mixed_precision_effective": amp_effective,
            "camera_pretrained_provenance": camera_pretrained_provenance,
            "warnings": warnings,
            "optimizer_group_contract": optimizer_group_contract,
            "invocations": [],
        }
    run_manifest["status"] = "RUNNING"
    run_manifest["updated_at_utc"] = utc_now()
    run_manifest["invocations"].append(
        {
            "time_utc": utc_now(),
            "resume": str(resume_path) if resume_path is not None else None,
            "stop_after_epoch": stop_after_epoch,
            "requested_device": requested_device,
            "arguments": invocation or {},
        }
    )
    atomic_write_json(run_manifest_path, run_manifest)

    selection_contract = {
        "best_ade.pt": "validation.ade_m",
        "best_corner_control.pt": (
            "validation.curvature_buckets.sharp.controller_proxy_mae_rad"
            if data_format_version == 2
            else "validation.controller_proxy_mae_rad"
        ),
        "best_speed.pt": "validation.speed_mae_mps",
        "last.pt": "latest complete epoch for resume",
    }
    stopped_early = False
    paused = False
    for epoch in range(start_epoch, epochs + 1):
        epoch_started = time.perf_counter()
        backbone_frozen = epoch <= int(training["freeze_backbone_epochs"])
        _set_backbone_trainable(model, not backbone_frozen)
        learning_rates_at_start = {
            str(group["group_name"]): float(group["lr"])
            for group in optimizer.param_groups
        }
        train_metrics, global_step = run_epoch(
            model,
            train_loader,
            optimizer=optimizer,
            scheduler=scheduler,
            scaler=scaler,
            device=device,
            loss_weights=config["loss_weights"],
            horizon_weights=training["waypoint_horizon_weights"],
            grad_clip_norm=float(training["grad_clip_norm"]),
            amp_enabled=amp_effective,
            backbone_frozen=backbone_frozen,
            global_step=global_step,
            num_waypoints=int(config["data"]["num_waypoints"]),
            straight_curvature_threshold_per_m=(
                straight_curvature_threshold_per_m
            ),
            sharp_curvature_threshold_per_m=sharp_curvature_threshold_per_m,
            controller_wheelbase_m=controller_wheelbase_m,
        )
        validation_metrics, _ = run_epoch(
            model,
            val_loader,
            optimizer=None,
            scheduler=None,
            scaler=scaler,
            device=device,
            loss_weights=config["loss_weights"],
            horizon_weights=training["waypoint_horizon_weights"],
            grad_clip_norm=float(training["grad_clip_norm"]),
            amp_enabled=amp_effective,
            backbone_frozen=False,
            global_step=global_step,
            num_waypoints=int(config["data"]["num_waypoints"]),
            straight_curvature_threshold_per_m=(
                straight_curvature_threshold_per_m
            ),
            sharp_curvature_threshold_per_m=sharp_curvature_threshold_per_m,
            controller_wheelbase_m=controller_wheelbase_m,
        )
        if data_format_version == 2:
            sharp_metrics = validation_metrics["curvature_buckets"]["sharp"]
            if (
                int(sharp_metrics["sample_count"]) <= 0
                or sharp_metrics["controller_proxy_mae_rad"] is None
            ):
                raise RuntimeError(
                    "Dataset v2 validation split contains no sharp-curvature samples"
                )
            corner_metric_value = float(
                sharp_metrics["controller_proxy_mae_rad"]
            )
        else:
            corner_metric_value = float(
                validation_metrics["controller_proxy_mae_rad"]
            )
        record = {
            "epoch": epoch,
            "global_step": global_step,
            "backbone_frozen": backbone_frozen,
            "learning_rates_at_start": learning_rates_at_start,
            "learning_rates_at_end": {
                str(group["group_name"]): float(group["lr"])
                for group in optimizer.param_groups
            },
            "train": train_metrics,
            "validation": validation_metrics,
            "epoch_seconds": time.perf_counter() - epoch_started,
        }
        history.append(record)
        atomic_write_json(history_path, history)
        print(
            json.dumps(
                {
                    "epoch": epoch,
                    "global_step": global_step,
                    "train_loss": train_metrics["loss_total"],
                    "val_loss": validation_metrics["loss_total"],
                    "val_ade_m": validation_metrics["ade_m"],
                    "val_fde_m": validation_metrics["fde_m"],
                    "val_corner_control_mae_rad": corner_metric_value,
                    "val_corner_metric": corner_metric_key,
                    "amp_scale": train_metrics["amp_scale"],
                    "backbone_frozen": backbone_frozen,
                },
                allow_nan=False,
            ),
            flush=True,
        )

        metric_targets = {
            "ade_m": ("best_ade.pt", float(validation_metrics["ade_m"])),
            corner_metric_key: (
                "best_corner_control.pt",
                corner_metric_value,
            ),
            "speed_mae_mps": (
                "best_speed.pt",
                float(validation_metrics["speed_mae_mps"]),
            ),
        }
        improved_roles: list[tuple[str, str, float]] = []
        for metric_name, (filename, value) in metric_targets.items():
            if value < best_metrics[metric_name]:
                best_metrics[metric_name] = value
                selected_checkpoints[filename] = {
                    "epoch": epoch,
                    "global_step": global_step,
                    "metric": metric_name,
                    "value": value,
                }
                improved_roles.append((filename, metric_name, value))

        current_ade = float(validation_metrics["ade_m"])
        if current_ade < early_best_ade - float(
            training["early_stopping_min_delta_m"]
        ):
            early_best_ade = current_ade
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1

        common_payload = dict(
            model=model,
            optimizer=optimizer,
            scheduler=scheduler,
            scaler=scaler,
            config=config,
            epoch=epoch,
            global_step=global_step,
            resolved_config_sha256=resolved_config_sha256,
            dataset_manifest_sha256=dataset_manifest_sha256,
            best_metrics=best_metrics,
            selected_checkpoints=selected_checkpoints,
            early_best_ade=early_best_ade,
            epochs_without_improvement=epochs_without_improvement,
            sampler_generator=sampler_generator,
            train_worker_generator=train_worker_generator,
            val_worker_generator=val_worker_generator,
            amp_effective=amp_effective,
            optimizer_group_contract=optimizer_group_contract,
        )
        for filename, metric_name, value in improved_roles:
            save_v1_checkpoint(
                output_dir / filename,
                _checkpoint_payload(
                    role=f"best:{metric_name}:{value}", **common_payload
                ),
            )
        save_v1_checkpoint(
            output_dir / "last.pt",
            _checkpoint_payload(role="last", **common_payload),
        )
        atomic_write_json(
            output_dir / "checkpoint_manifest.json",
            {
                "format_version": "transfuser_lite_v1_checkpoint_manifest_v1",
                "selection_contract": selection_contract,
                "selected": selected_checkpoints,
                "last": {"epoch": epoch, "global_step": global_step},
                "resolved_config_sha256": resolved_config_sha256,
                "dataset_manifest_sha256": dataset_manifest_sha256,
            },
        )

        if stop_after_epoch is not None and epoch >= stop_after_epoch:
            paused = epoch < epochs
            break
        if epochs_without_improvement >= int(training["early_stopping_patience"]):
            stopped_early = True
            break

    final_epoch = int(history[-1]["epoch"])
    run_manifest["status"] = (
        "PAUSED" if paused else "EARLY_STOPPED" if stopped_early else "COMPLETED"
    )
    run_manifest["updated_at_utc"] = utc_now()
    run_manifest["completed_epoch"] = final_epoch
    run_manifest["global_step"] = global_step
    run_manifest["best_metrics"] = best_metrics
    run_manifest["selected_checkpoints"] = selected_checkpoints
    run_manifest["early_stopped"] = stopped_early
    atomic_write_json(run_manifest_path, run_manifest)
    result = {
        "status": run_manifest["status"],
        "completed_epoch": final_epoch,
        "global_step": global_step,
        "best_metrics": best_metrics,
        "selected_checkpoints": selected_checkpoints,
        "amp_effective": amp_effective,
        "resolved_config_sha256": resolved_config_sha256,
        "dataset_manifest_sha256": dataset_manifest_sha256,
    }
    atomic_write_json(output_dir / "result.json", result)
    return result


def main() -> None:
    args = parse_args()
    result = train_v1(
        config_path=args.config,
        train_index=args.train_index,
        val_index=args.val_index,
        output=args.output,
        resume=args.resume,
        stop_after_epoch=args.stop_after_epoch,
        requested_device=args.device,
        invocation=vars(args),
    )
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
