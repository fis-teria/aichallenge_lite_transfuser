"""Finite, resumable full-corpus TimePath training with a sealed test split."""
from __future__ import annotations

from contextlib import nullcontext
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import random
import time
from typing import Any, Sequence

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, Subset

from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_split_v1 import assert_split_membership, content_sha256
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from .train_time_v1 import training_batch
from .time_objective_v1 import time_loss_sum_and_count
from .time_checkpoint_v1 import (TimeCheckpointIdentity, inspect_time_checkpoint,
                                  load_time_checkpoint, save_time_checkpoint)
from .time_config_v1 import TimeModelConfig, build_time_model


@dataclass(frozen=True)
class CorpusTrainingPlan:
    epochs: int = 10
    batch_size: int = 32
    workers: int = 4
    seed: int = 42
    learning_rate: float = 0.0003
    weight_decay: float = 0.0001
    max_grad_norm: float = 1.0
    precision: str = "bf16"
    checkpoint_every_steps: int = 250
    log_every_batches: int = 50

    def validate(self) -> None:
        if any(type(v) is not int or v <= 0 for v in
               (self.epochs, self.batch_size, self.checkpoint_every_steps, self.log_every_batches)):
            raise ValueError("positive finite epoch/batch/checkpoint budgets required")
        if any(type(v) is not int or v < 0 for v in (self.workers, self.seed)):
            raise ValueError("workers/seed must be nonnegative integers")
        if any(type(v) is not float or not math.isfinite(v) or v <= 0 for v in
               (self.learning_rate, self.max_grad_norm)):
            raise ValueError("learning rate and clipping norm must be positive finite floats")
        if type(self.weight_decay) is not float or not math.isfinite(self.weight_decay) or self.weight_decay < 0:
            raise ValueError("weight decay must be nonnegative finite float")
        if self.precision not in {"float32", "bf16"}:
            raise ValueError("precision must be float32 or bf16")


def epoch_order(count: int, seed: int, epoch: int) -> list[int]:
    """Frozen within-train permutation; whole-run split assignments never change."""
    if any(type(v) is not int or v < 0 for v in (count, seed, epoch)):
        raise ValueError("order arguments must be nonnegative integers")
    return np.random.default_rng(np.random.SeedSequence([seed, epoch])).permutation(count).tolist()


def collate_samples(samples: list[TimeSample]) -> list[TimeSample]:
    return samples


def train_corpus_batch(model: torch.nn.Module, samples: Sequence[TimeSample],
                       optimizer: torch.optim.Optimizer, *, precision: str,
                       max_grad_norm: float, scheduler: Any = None) -> dict[str, Any]:
    """One optimizer window, per-supported-anchor L1, FP32 loss and parameters.

BF16 applies only to model forward; no loss scaler is used or needed. Missing
inputs still count as presented anchors, and zero support does not update Adam.
"""
    device = next(model.parameters()).device
    if precision not in {"float32", "bf16"} or (precision == "bf16" and device.type != "cuda"):
        raise ValueError("BF16 training requires CUDA")
    if not math.isfinite(max_grad_norm) or max_grad_norm <= 0:
        raise ValueError("positive gradient clipping norm required")
    eligible = [s for s in samples if s.inputs is not None]
    result = {"visited": len(samples), "input_invalid": len(samples) - len(eligible),
              "teacher_unsupported": sum(s.teacher is None or not s.teacher.xy_mask.any() for s in eligible),
              "supported": 0, "loss_sum_m": 0.0, "updated": False, "grad_norm": None}
    optimizer.zero_grad(set_to_none=True)
    if not eligible:
        return result
    batch = training_batch(eligible, device)
    count = int(batch.targets.trajectory_mask.any(dim=1).sum().item())
    if count == 0:
        return result
    context = torch.autocast("cuda", dtype=torch.bfloat16) if precision == "bf16" else nullcontext()
    try:
        with context:
            prediction = model(batch)
        summed, supported = time_loss_sum_and_count(prediction.float(), batch.targets.trajectory_xy_m,
                                                     batch.targets.trajectory_mask)
        if supported != count:
            raise RuntimeError("support changed during forward")
        (summed / count).backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm, error_if_nonfinite=True)
        optimizer.step()
        if scheduler is not None:
            scheduler.step()
    except Exception:
        optimizer.zero_grad(set_to_none=True)
        raise
    return {**result, "supported": count, "loss_sum_m": float(summed.detach().cpu()),
            "updated": True, "grad_norm": float(grad_norm.detach().cpu())}


def _json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, path)


def weights_digest(model: torch.nn.Module) -> str:
    """Tensor-only digest, so OFF/ON semantics can differ with identical weights."""
    digest = hashlib.sha256()
    for name, value in model.state_dict().items():
        if isinstance(value, torch.Tensor):
            digest.update(name.encode("utf-8"))
            digest.update(value.detach().cpu().contiguous().numpy().tobytes())
    return digest.hexdigest()


def run_training_arm(train: Dataset[TimeSample], validation: Dataset[TimeSample], *,
                     train_run_ids: Sequence[str], validation_run_ids: Sequence[str],
                     split_manifest: dict[str, Any], teacher_manifest: dict[str, Any],
                     identity: TimeCheckpointIdentity, config: TimeModelConfig,
                     plan: CorpusTrainingPlan, output: Path, device: str = "cuda",
                     resume: bool = False) -> dict[str, Any]:
    """Complete a finite arm, select on validation run-macro 3s error, reload best."""
    plan.validate(); config.validate(); identity.validate()
    if len(train_run_ids) != len(train) or len(validation_run_ids) != len(validation):
        raise ValueError("one run identity per anchor required")
    assert_split_membership(split_manifest, train_run_ids, split="train")
    assert_split_membership(split_manifest, validation_run_ids, split="validation")
    if not len(train) or not len(validation):
        raise ValueError("nonempty train and validation required")
    if device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    if plan.precision == "bf16" and (device != "cuda" or not torch.cuda.is_bf16_supported()):
        raise ValueError("BF16 plan requires a supported CUDA device")
    if resume:
        if not (output / "last.pt").is_file():
            raise FileNotFoundError("resume checkpoint missing")
    else:
        output.mkdir(parents=True, exist_ok=False)
    random.seed(plan.seed); np.random.seed(plan.seed); torch.manual_seed(plan.seed)
    if device == "cuda":
        torch.cuda.manual_seed_all(plan.seed)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    model = build_time_model(config).to(device)
    initial_sha = weights_digest(model)
    optimizer = torch.optim.AdamW(model.parameters(), lr=plan.learning_rate, weight_decay=plan.weight_decay)
    max_steps = math.ceil(len(train) / plan.batch_size) * plan.epochs
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max_steps,
                                                          eta_min=plan.learning_rate * .1)
    frozen_plan = {**asdict(plan), "max_anchors": len(train) * plan.epochs,
        "max_optimizer_steps": max_steps, "order": "seeded_per_epoch_natural_anchor_permutation",
        "selection": "minimum_validation_run_macro_3s_error_m", "allow_tf32": False,
        "cudnn_benchmark": False, "cudnn_deterministic": True, "loss_scaler": None,
        "device_type": device, "torch_version": torch.__version__}
    state: dict[str, Any] = {"plan": frozen_plan, "history": [], "best_score_m": None,
        "best_epoch": None, "initial_weights_sha256": initial_sha, "total_anchors_visited": 0,
        "epoch_totals": {"visited": 0, "input_invalid": 0, "teacher_unsupported": 0,
                         "supported": 0, "loss_sum_m": 0.0}, "elapsed_seconds": 0.0}
    epoch = cursor = global_step = 0
    if resume:
        info = inspect_time_checkpoint(output / "last.pt", config=config, identity=identity)
        saved = info["training_state"]
        if not isinstance(saved, dict) or saved.get("plan") != frozen_plan:
            raise ValueError("resume plan differs from checkpoint")
        state = saved
        epoch, global_step = load_time_checkpoint(output / "last.pt", config=config, identity=identity,
            model=model, optimizer=optimizer, scheduler=scheduler, mode="resume")
        cursor = info["next_anchor_index"]
        if cursor > len(train) or epoch > plan.epochs:
            raise ValueError("checkpoint progress exceeds frozen budget")
    else:
        _json(output / "plan.json", {**frozen_plan, "config": config.to_dict(),
              "identity": asdict(identity), "initial_weights_sha256": initial_sha})
    started = time.monotonic()
    prior_elapsed = float(state["elapsed_seconds"])

    def emit(phase: str, **values: Any) -> None:
        status = {"phase": phase, "arm": output.name, "epoch": epoch + 1,
                  "epochs": plan.epochs, "epoch_cursor": cursor, "anchors_per_epoch": len(train),
                  "optimizer_steps": global_step, "elapsed_seconds": prior_elapsed + time.monotonic() - started,
                  **values}
        _json(output / "status.json", status)
        print(json.dumps(status, allow_nan=False), flush=True)

    def save(path: Path, saved_epoch: int, saved_cursor: int) -> None:
        state["elapsed_seconds"] = prior_elapsed + time.monotonic() - started
        save_time_checkpoint(path, config=config, identity=identity, model=model, optimizer=optimizer,
            scheduler=scheduler, epoch=saved_epoch, global_step=global_step, next_anchor_index=saved_cursor,
            split_manifest=split_manifest, teacher_manifest=teacher_manifest, training_state=state)

    if not resume:
        save(output / "initial.pt", 0, 0)
        emit("INITIAL_VALIDATION")
        metrics, _ = evaluate_time_batched(model, validation, run_ids=validation_run_ids,
            split_manifest=split_manifest, batch_size=plan.batch_size, workers=plan.workers, precision=plan.precision)
        _json(output / "initial_validation.json", metrics)
        save(output / "last.pt", 0, 0)
    for epoch in range(epoch, plan.epochs):
        model.train()
        order = epoch_order(len(train), plan.seed, epoch)
        order_sha = content_sha256([teacher_manifest["manifest_sha256"], epoch, order])
        if cursor and state.get("epoch_order_sha256") != order_sha:
            raise ValueError("resume epoch presentation order changed")
        state["epoch_order_sha256"] = order_sha
        generator = torch.Generator().manual_seed(plan.seed + epoch)
        loader = DataLoader(Subset(train, order[cursor:]), batch_size=plan.batch_size,
            num_workers=plan.workers, collate_fn=collate_samples, generator=generator,
            persistent_workers=False, **({"prefetch_factor": 1, "multiprocessing_context": "spawn"} if plan.workers else {}))
        totals = state["epoch_totals"]
        emit("TRAINING")
        for batch_number, samples in enumerate(loader, 1):
            result = train_corpus_batch(model, samples, optimizer, precision=plan.precision,
                                       max_grad_norm=plan.max_grad_norm, scheduler=scheduler)
            cursor += len(samples)
            state["total_anchors_visited"] += len(samples)
            for key in totals:
                totals[key] += result[key]
            global_step += int(result["updated"])
            if global_step > max_steps or state["total_anchors_visited"] > frozen_plan["max_anchors"]:
                raise RuntimeError("finite training budget exceeded")
            if batch_number % plan.log_every_batches == 0:
                emit("TRAINING", train_l1_m=totals["loss_sum_m"] / totals["supported"] if totals["supported"] else None,
                     learning_rate=optimizer.param_groups[0]["lr"])
            if result["updated"] and global_step % plan.checkpoint_every_steps == 0:
                save(output / "last.pt", epoch, cursor)
        if cursor != len(train) or totals["visited"] != len(train):
            raise RuntimeError("epoch did not visit the complete natural anchor population")
        if totals["supported"] == 0:
            raise RuntimeError("no supported training anchors; no trained candidate exists")
        save(output / "last.pt", epoch, cursor)
        emit("VALIDATING", train_l1_m=totals["loss_sum_m"] / totals["supported"])
        metrics, predictions = evaluate_time_batched(model, validation, run_ids=validation_run_ids,
            split_manifest=split_manifest, batch_size=plan.batch_size, workers=plan.workers, precision=plan.precision)
        score = metrics["run_macro_mean"]["3s"]["raw_error_m"]
        if score is None or not math.isfinite(score):
            raise RuntimeError("validation has no finite 3s score")
        _json(output / f"validation_epoch_{epoch + 1:02d}.json", metrics)
        np.save(output / f"validation_epoch_{epoch + 1:02d}.npy", predictions.numpy(), allow_pickle=False)
        row = {"epoch": epoch + 1, "optimizer_steps": global_step,
               "train_l1_m": totals["loss_sum_m"] / totals["supported"], "train_counts": dict(totals),
               "validation_run_macro_3s_m": score, "validation_ade_m": metrics["all_point_ade_m"],
               "epoch_order_sha256": order_sha}
        state["history"].append(row)
        improved = state["best_score_m"] is None or score < state["best_score_m"]
        if improved:
            state["best_score_m"], state["best_epoch"] = score, epoch + 1
        state["epoch_totals"] = {key: 0.0 if key == "loss_sum_m" else 0 for key in totals}
        cursor = 0
        if improved:
            save(output / "best.pt", epoch + 1, 0)
        save(output / "last.pt", epoch + 1, 0)
        _json(output / "history.json", state["history"])
        emit("EPOCH_COMPLETE", **row, best_epoch=state["best_epoch"])
    emit("RELOADING_BEST", best_epoch=state["best_epoch"])
    selected = build_time_model(config).to(device)
    load_time_checkpoint(output / "best.pt", config=config, identity=identity, model=selected, mode="finetune")
    metrics, predictions = evaluate_time_batched(selected, validation, run_ids=validation_run_ids,
        split_manifest=split_manifest, batch_size=plan.batch_size, workers=plan.workers, precision=plan.precision)
    original = np.load(output / f"validation_epoch_{state['best_epoch']:02d}.npy", allow_pickle=False)
    np.testing.assert_allclose(predictions.numpy(), original, rtol=0, atol=0, equal_nan=True)
    _json(output / "best_validation.json", metrics)
    result = {"status": "COMPLETE", "config": config.to_dict(), "plan": frozen_plan,
              "epochs_completed": plan.epochs, "optimizer_steps": global_step,
              "anchors_visited": state["total_anchors_visited"], "best_epoch": state["best_epoch"],
              "best_validation_run_macro_3s_m": state["best_score_m"], "best_validation_ade_m": metrics["all_point_ade_m"],
              "initial_weights_sha256": state["initial_weights_sha256"],
              "reload_predictions_exact": True, "best_checkpoint": str((output / "best.pt").resolve()),
              "training_elapsed_seconds": prior_elapsed + time.monotonic() - started,
              "scope": "offline_fixed_condition_run_holdout", "test_evaluated": False, "runtime_ready": False}
    _json(output / "result.json", result)
    emit("COMPLETE", best_epoch=state["best_epoch"], validation_run_macro_3s_m=state["best_score_m"])
    return result
