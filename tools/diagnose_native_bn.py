"""Separate learned-parameter and BatchNorm-statistic drift; no training/deployment."""
from __future__ import annotations

import argparse
from collections import defaultdict
import gc
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import torch
from torch.utils.data import Subset

from train_time_launch_protection import read, write, stage_errors
from train_time_native_replay import predict_samples
from aic_transfuser_lite.data.time_native_replay_v1 import NativeReplayDataset
from aic_transfuser_lite.data.time_split_v1 import assert_split_membership
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, _sha, verify_time_training_cache
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.training.native_bn_policy_v1 import copy_batchnorm_statistics
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--per-run", type=int, default=32, help="validation windows/run; 0 means full existing validation")
    args = parser.parse_args()
    root = args.root.resolve()
    repo = Path(__file__).resolve().parents[1]
    if str(root).startswith("/mnt/") or not torch.cuda.is_available() or args.per_run < 0:
        raise ValueError("native WSL CUDA and nonnegative validation limit required")
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=repo).strip():
        raise ValueError("clean committed source required")
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.manual_seed(42)
    args.output.mkdir(parents=True, exist_ok=False)
    prior = root / "runs/native701_retention_20260919"
    prep = root / "runs/time_native701_prepare_20260919/preparation"
    plan = read(repo / "configs/time_path_p1/native701_prepare_20260919.json")
    previous = read(repo / plan["previous_plan"])
    proof = read(prep / "proof.json")
    old_proof = read(root / plan["previous_experiment"] / "preparation/proof.json")
    for name, digest in proof["previous_artifacts"].items():
        if _sha(root / name) != digest:
            raise ValueError("old artifact changed: " + name)
    if _sha(prep / "split_manifest.json") != proof["split_file_sha256"]:
        raise ValueError("split changed")
    if verify_time_training_cache(root / previous["cache"])["manifest_sha256"] != previous["cache_sha256"]:
        raise ValueError("cache changed")
    split = read(prep / "split_manifest.json")
    val = TimeTrainingCacheDataset(root / previous["cache"], "validation", verify_hashes=False)
    native = NativeReplayDataset(prep, manifest_sha256=proof["native_manifest_sha256"])
    assert_split_membership(split, native.run_ids, split="train")
    groups: dict[str, list[int]] = defaultdict(list)
    for i in old_proof["validation"]["selected_indices"]:
        groups[val.run_ids[i]].append(i)
    selected = []
    for indices in groups.values():
        indices.sort()
        positions = np.linspace(0, len(indices) - 1, min(args.per_run, len(indices)), dtype=int) if args.per_run else range(len(indices))
        selected.extend(indices[int(p)] for p in positions)
    selected.sort()
    chosen = set(selected)
    stages = {name: [i for i in indices if i in chosen]
              for name, indices in old_proof["validation"]["stages"].items()}
    target = np.stack([native[i].teacher.xy_m for i in range(len(native))])
    front = read(prior / "resolved_plan.json")["front_indices"]
    checkpoints = {name: read(prior / (name + ".json")) for name in ("initial", "focused", "focused_geometry")}
    for report in checkpoints.values():
        if _sha(Path(report["checkpoint"])) != report["checkpoint_sha256"]:
            raise ValueError("checkpoint changed")

    def load(name: str) -> torch.nn.Module:
        path = Path(checkpoints[name]["checkpoint"])
        payload = torch.load(path, map_location="cpu", weights_only=False)
        config = TimeModelConfig.from_dict(payload["config"])
        model = build_time_model(config).cuda()
        load_time_checkpoint(path, config=config, identity=TimeCheckpointIdentity(**payload["identity"]), model=model, mode="finetune")
        return model.eval()

    modes = [("initial", None, ""), ("focused", None, ""),
             ("focused", "initial", ""), ("focused_geometry", None, ""),
             ("focused_geometry", "initial", ""), ("initial", "focused_geometry", ""),
             ("focused_geometry", "initial", "backbone.camera"),
             ("focused_geometry", "initial", "backbone.lidar")]
    identity = dict(source_git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
                    native_manifest_sha256=proof["native_manifest_sha256"],
                    checkpoints={k: v["checkpoint_sha256"] for k, v in checkpoints.items()},
                    validation_indices=selected, validation_per_run=args.per_run,
                    native_anchors=len(native), front_anchors=len(front),
                    scope="BN attribution on development validation and training fit; no checkpoint selection or deployment",
                    optimizer_steps=0, aws_sim_runs=0, test_split_used=False)
    write(args.output / "identity.json", identity)
    reports = []
    for weights, buffers, prefix in modes:
        name = weights + ("__bn_" + buffers + ("__" + prefix if prefix else "") if buffers else "__original")
        started = time.monotonic()
        print("EVALUATING", name, flush=True)
        model = load(weights)
        copied = []
        if buffers:
            donor = load(buffers)
            copied = copy_batchnorm_statistics(model, donor, prefix=prefix)
            del donor
        _, prediction = evaluate_time_batched(model, Subset(val, selected), run_ids=[val.run_ids[i] for i in selected],
                                              split_manifest=split, batch_size=32, workers=0, precision="float32")
        values = prediction.numpy()
        native_prediction = predict_samples(model, native)
        if not np.isfinite(values).all() or not np.isfinite(native_prediction).all():
            raise ValueError("nonfinite diagnostic prediction")
        report = dict(name=name, copied_modules=copied, xy=stage_errors(values, val, selected, stages),
                      front_ade_m=float(np.linalg.norm(native_prediction[front] - target[front], axis=2).mean()),
                      native_ade_m=float(np.linalg.norm(native_prediction - target, axis=2).mean()),
                      wall_s=time.monotonic() - started)
        if buffers is None:
            saved = np.load(prior / (weights + "_native_predictions.npy"))
            difference = float(np.max(np.abs(saved - native_prediction)))
            report["native_replay_max_abs_m"] = difference
            if difference > 1e-5:
                raise ValueError(f"saved prediction replay drift {difference}")
        write(args.output / (name + ".json"), report)
        np.save(args.output / (name + "_native.npy"), native_prediction)
        reports.append(report)
        write(args.output / "summary.json", dict(identity=identity, reports=reports))
        print(json.dumps(report), flush=True)
        del model
        gc.collect()
        torch.cuda.empty_cache()
    print("BN_DIAGNOSTIC_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
