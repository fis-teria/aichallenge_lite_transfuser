"""Finite common-head fit / real-cache IL pilot; never deploys a model."""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import time
from typing import Any, Sequence

import numpy as np
import torch

from aic_transfuser_lite.contracts.common_il_v1 import CommonILTargets, common_il_contract
from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset
from aic_transfuser_lite.models.common_il_v1 import CommonILHeadsV1, CommonILV1
from aic_transfuser_lite.training.common_il_config_v1 import load_common_il_experiment
from aic_transfuser_lite.training.common_il_v1 import (
    CommonILLossWeights, collate_common_il, common_il_loss, load_common_il_checkpoint,
    save_common_il_checkpoint, train_common_il_batch,
)

ROOT = Path(__file__).resolve().parents[1]


def write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def evaluate(model: CommonILV1, samples: Sequence[TimeSample], batch_size: int,
             weights: CommonILLossWeights) -> dict[str, Any]:
    """Training-subset diagnostic, not heldout driving/generalization evidence."""
    totals = {name: 0.0 for name in ("waypoint", "speed", "stop")}
    counts = {name: 0 for name in totals}
    positives = 0
    model.eval()
    with torch.no_grad():
        for i in range(0, len(samples), batch_size):
            valid = [sample for sample in samples[i:i+batch_size] if sample.inputs is not None]
            if not valid:
                continue
            inputs, labels = collate_common_il(valid, next(model.parameters()).device)
            _, metrics = common_il_loss(model(inputs), labels, weights)
            positives += metrics["support"]["stop_positive"]
            for name in totals:
                if metrics["loss"][name] is not None:
                    count = metrics["support"][name]
                    totals[name] += metrics["loss"][name] * count
                    counts[name] += count
    return {"support": counts, "stop_positive": positives,
            "loss": {name: totals[name] / counts[name] if counts[name] else None for name in totals}}


def head_fit(output: Path, hidden_dim: int, seed: int, device: torch.device) -> dict[str, Any]:
    """Four explicit synthetic feature/label pairs, 64 optimizer steps."""
    torch.manual_seed(seed + 2)
    model = CommonILHeadsV1(hidden_dim).to(device)
    generator = torch.Generator().manual_seed(seed + 3)
    features = torch.randn(4, hidden_dim, generator=generator).to(device)
    speed = torch.tensor([[2.], [0.], [1.], [0.]], device=device)
    times = torch.tensor(common_il_contract()["waypoint_time_sec"], device=device)
    xy = torch.stack((speed * times[None], torch.zeros(4, 6, device=device)), dim=-1)
    target = CommonILTargets(xy, torch.ones(4, 6, dtype=torch.bool, device=device), speed,
        torch.ones(4, 1, dtype=torch.bool, device=device), torch.tensor([[0.], [1.], [0.], [1.]], device=device),
        torch.ones(4, 1, dtype=torch.bool, device=device))
    optimizer = torch.optim.AdamW(model.parameters(), lr=.01, weight_decay=0.)
    _, before = common_il_loss(model(features), target)
    trace = []
    for step in range(64):
        optimizer.zero_grad(set_to_none=True)
        loss, metrics = common_il_loss(model(features), target)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0, error_if_nonfinite=True)
        optimizer.step()
        trace.append({"step": step + 1, **metrics})
    with torch.no_grad():
        _, after = common_il_loss(model(features), target)
    result = {"status": "PASS" if all(after["loss"][name] < before["loss"][name] for name in before["loss"]) else "FIT_NOT_DEMONSTRATED",
              "dataset_id": "synthetic_fixed_fused_features_v1", "optimizer_steps": 64,
              "before": before, "after": after, "trace": trace, "scope": "head fit only; no camera/encoder or driving evidence"}
    write_json(output / "report.json", result)
    return result


def read_stop_annotations(path: Path | None, dataset: TimeTrainingCacheDataset) -> tuple[dict[str, bool], str | None]:
    if path is None:
        return {}, None
    content = path.read_bytes()
    value = json.loads(content)
    if (set(value) != {"format", "cache_manifest_sha256", "rule_version", "labels"}
            or value["format"] != "explicit_environment_stop_v1"
            or value["cache_manifest_sha256"] != dataset.identity["manifest_sha256"]
            or type(value["rule_version"]) is not str or not value["rule_version"].strip()
            or type(value["labels"]) is not dict
            or not set(value["labels"]) <= set(dataset.anchor_ids)
            or any(type(label) is not bool for label in value["labels"].values())):
        raise ValueError("explicit stop annotations require matching source, rule and train anchor IDs")
    return value["labels"], hashlib.sha256(content).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("head-fit", "real"))
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--cache", type=Path)
    parser.add_argument("--stop-annotations", type=Path)
    parser.add_argument("--dinov3-model-config", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    args = parser.parse_args()
    if args.mode == "real" and args.cache is None:
        parser.error("real mode requires an existing verified --cache")
    config, experiment = load_common_il_experiment(args.config, workspace=ROOT, dinov3_model_config=args.dinov3_model_config)
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    device = torch.device(args.device)
    torch.set_num_threads(2)
    torch.manual_seed(experiment["initialization_seed"])
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    args.output.mkdir(parents=True, exist_ok=False)
    budget = {"mode": args.mode, "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "experiment": experiment, "resolved_backbone_config": config.to_dict(), "contract": common_il_contract(),
        "max_optimizer_steps": 64 if args.mode == "head-fit" else experiment["pilot"]["max_optimizer_steps"],
        "max_presentations": 256 if args.mode == "head-fit" else experiment["pilot"]["max_optimizer_steps"] * experiment["pilot"]["batch_size"] * 2,
        "outer_timeout_sec": 900, "precision": "float32", "device": args.device, "threads": 2,
        "tf32": False, "runtime_ready": False, "test_used": False}
    write_json(args.output / "plan.json", budget)
    if args.mode == "head-fit":
        result = head_fit(args.output, config.hidden_dim, experiment["initialization_seed"], device)
        print(json.dumps({key: value for key, value in result.items() if key != "trace"}), flush=True)
        if result["status"] != "PASS":
            raise RuntimeError("finite head-fit did not reduce all supported losses")
        return
    started = time.perf_counter()
    # Fail early on unavailable DINO assets, before reading the corpus.
    model = CommonILV1(config, initialization_seed=experiment["initialization_seed"]).to(device)
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats()
    dataset = TimeTrainingCacheDataset(args.cache, "train", verify_hashes=True)
    if dataset.config != config.dataset_config():
        raise ValueError("cache preprocessing differs from model configuration")
    annotations, annotation_sha = read_stop_annotations(args.stop_annotations, dataset)
    indices = []
    for run in experiment["dataset"]["run_ids"]:
        group = [i for i, rid in enumerate(dataset.run_ids) if rid == run]
        if not group:
            raise ValueError(f"requested run is absent from train split: {run}")
        indices.extend(group[i] for i in np.linspace(0, len(group) - 1,
                       min(len(group), experiment["dataset"]["samples_per_run"]), dtype=int))
    samples = [replace(dataset[i], environment_stop_intent=annotations.get(dataset.anchor_ids[i])) for i in indices]
    identity = {"cache_manifest_sha256": dataset.identity["manifest_sha256"],
                "anchors": [s.anchor_id for s in samples], "stop_annotation_sha256": annotation_sha}
    identity["dataset_sha256"] = content_sha256(identity)
    write_json(args.output / "dataset.json", identity)
    preparation_sec = time.perf_counter() - started
    print(json.dumps({"stage": "ready", "samples": len(samples), "preparation_sec": preparation_sec,
                      "dataset_sha256": identity["dataset_sha256"]}), flush=True)
    plan = experiment["pilot"]
    weights = CommonILLossWeights(**experiment["loss_weights"])
    optimizer = torch.optim.AdamW((p for p in model.parameters() if p.requires_grad), lr=plan["learning_rate"], weight_decay=plan["weight_decay"])
    before = evaluate(model, samples, plan["batch_size"], weights)
    stop_before = {name: value.detach().clone() for name, value in model.heads.stop_head.state_dict().items()}
    rng = np.random.default_rng(experiment["sampler_seed"])
    order: list[int] = []
    steps = presented = 0
    trace = []
    train_started = time.perf_counter()
    while steps < plan["max_optimizer_steps"] and presented < budget["max_presentations"]:
        if time.perf_counter() - started > 880:
            raise TimeoutError("pilot wall-clock budget exhausted")
        if not order:
            order = rng.permutation(len(samples)).tolist()
        selected = order[:plan["batch_size"]]
        del order[:len(selected)]
        metrics = train_common_il_batch(model, [samples[i] for i in selected], optimizer, weights=weights,
                                        max_grad_norm=plan["max_grad_norm"])
        presented += len(selected)
        steps += int(metrics["updated"])
        trace.append({"optimizer_step": steps, "anchor_ids": [samples[i].anchor_id for i in selected], **metrics})
    training_sec = time.perf_counter() - train_started
    after = evaluate(model, samples, plan["batch_size"], weights)
    stop_unchanged = all(torch.equal(value, stop_before[name]) for name, value in model.heads.stop_head.state_dict().items())
    if before["support"]["stop"] == 0 and not stop_unchanged:
        raise AssertionError("unsupervised stop head changed")
    result = {"status": "PASS" if steps == plan["max_optimizer_steps"] else "INSUFFICIENT_LABEL_SUPPORT",
        "dataset_sha256": identity["dataset_sha256"], "optimizer_steps": steps, "presentations": presented,
        "selected": len(samples), "valid_inputs": sum(s.inputs is not None for s in samples),
        "before": before, "after": after, "stop_head_unchanged": stop_unchanged,
        "stop_status": "NOT_SUPERVISED" if before["support"]["stop"] == 0 else "SUPERVISED_DIAGNOSTIC_ONLY",
        "preparation_sec": preparation_sec, "training_sec": training_sec,
        "peak_allocated_cuda_bytes": torch.cuda.max_memory_allocated() if device.type == "cuda" else None,
        "device_name": torch.cuda.get_device_name() if device.type == "cuda" else "cpu",
        "camera_provenance": model.backbone.camera.pretrained_provenance(),
        "trace": trace, "scope": "train-subset fit only; no validation/test/generalization or driving claim",
        "runtime_ready": False}
    save_common_il_checkpoint(args.output / "pilot.pt", model, metadata={k: v for k, v in result.items() if k != "trace"})
    replay_inputs, _ = collate_common_il([s for s in samples if s.inputs is not None][:2], device)
    model.eval()
    with torch.no_grad():
        expected = model(replay_inputs)
    del optimizer, model
    loaded, _ = load_common_il_checkpoint(args.output / "pilot.pt", device=args.device)
    with torch.no_grad():
        replay = loaded(replay_inputs)
    for name in ("waypoints_m", "target_speed_mps", "stop_logit"):
        torch.testing.assert_close(getattr(replay, name), getattr(expected, name), rtol=0, atol=0)
    result["checkpoint_replay"] = "EXACT"
    write_json(args.output / "report.json", result)
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    if result["status"] != "PASS":
        raise RuntimeError("finite pilot ran out of supervised presentations")


if __name__ == "__main__":
    main()
