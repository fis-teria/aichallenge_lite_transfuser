"""Bounded training-only gradient interference probe; zero optimizer steps."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

import torch

from train_time_launch_protection import read, write, sampler_context
from compare_time_recovery_objectives import make_objective
from aic_transfuser_lite.data.time_native_replay_v1 import NativeReplayDataset
from aic_transfuser_lite.data.time_split_v1 import assert_split_membership
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.training.native_bn_policy_v1 import freeze_batchnorm_statistics, gradient_alignment
from aic_transfuser_lite.training.native_fit_v1 import NativeGeometryObjective
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_objective_v1 import time_loss_sum_and_count
from aic_transfuser_lite.training.train_time_v1 import training_batch


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batches", type=int, default=8)
    args = parser.parse_args()
    root = args.root.resolve()
    repo = Path(__file__).resolve().parents[1]
    if (str(root).startswith("/mnt/") or not torch.cuda.is_available()
            or not 1 <= args.batches <= 16
            or subprocess.check_output(["git", "status", "--porcelain"], cwd=repo).strip()):
        raise ValueError("clean native WSL CUDA source and 1..16 batches required")
    args.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
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
    full = TimeTrainingCacheDataset(root / previous["cache"], "train", verify_hashes=False)
    samplers, old = sampler_context(root, previous, old_proof, full)
    replay = samplers["launch_balanced"]
    if replay.audit != proof["previous_sampler"]:
        raise ValueError("old replay changed")
    split = read(prep / "split_manifest.json")
    native = NativeReplayDataset(prep, manifest_sha256=proof["native_manifest_sha256"])
    targets = read(root / previous["parent_experiment"] / "recovery_geometry_targets.json")
    recovery = make_objective(dict(cache=dict(split_manifest=split), controller=old_proof["auxiliary_identity"]["controller"]), old, targets)
    if any(recovery.identity[k] != v for k, v in old_proof["auxiliary_identity"].items() if k != "split_manifest_sha256"):
        raise ValueError("old recovery objective changed")
    geometry = NativeGeometryObjective(read(prior / "native_geometry_targets.json"), split_manifest=split)
    source = root / plan["initialization"]
    if _sha(source) != plan["initialization_sha256"]:
        raise ValueError("initializer changed")
    payload = torch.load(source, map_location="cpu", weights_only=False)
    cfg = TimeModelConfig.from_dict(payload["config"])
    model = build_time_model(cfg).cuda()
    load_time_checkpoint(source, config=cfg, identity=TimeCheckpointIdentity(**payload["identity"]), model=model, mode="finetune")
    named = [(name, p) for name, p in model.named_parameters() if p.requires_grad]
    params = [p for _, p in named]
    groups = {"all": list(range(len(named)))}
    for prefix in ("backbone.camera", "backbone.lidar", "backbone.ego", "backbone.fusion", "decoder", "delta_head"):
        groups[prefix] = [i for i, (name, _) in enumerate(named) if name.startswith(prefix + ".")]
    schedule_path = prior / "focused_schedule.json"
    schedule = read(schedule_path)[:args.batches]
    reports = []
    identity = dict(source_git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
                    initializer_sha256=plan["initialization_sha256"], schedule_sha256=_sha(schedule_path),
                    native_manifest_sha256=proof["native_manifest_sha256"], batches=args.batches,
                    optimizer_steps=0, validation_used=False, test_used=False,
                    scope="initial-weight train-gradient diagnostic, no capacity or closed-loop claim")
    for policy in ("dynamic_bn", "frozen_bn"):
        for step, entries in enumerate(schedule):
            # Restore BOTH parameters and buffers: every probe is at the same initial model.
            model.load_state_dict(payload["model"], strict=True)
            model.train()
            if policy == "frozen_bn":
                freeze_batchnorm_statistics(model)
            torch.manual_seed(42 + step)
            old_samples = [replay[i] for kind, i in entries if kind == "old" and replay[i].inputs is not None]
            native_samples = [native[i] for kind, i in entries if kind == "native"]
            samples = old_samples + native_samples
            assert_split_membership(split, [s.run for s in samples], split="train")
            batch = training_batch(samples, torch.device("cuda"))
            prediction = model(batch)
            n = len(old_samples)
            old_target, new_target = batch.targets.trajectory_xy_m[:n], batch.targets.trajectory_xy_m[n:]
            old_mask, new_mask = batch.targets.trajectory_mask[:n], batch.targets.trajectory_mask[n:]
            old_sum, old_count = time_loss_sum_and_count(prediction[:n], old_target, old_mask)
            new_sum, new_count = time_loss_sum_and_count(prediction[n:], new_target, new_mask)
            old_loss = (old_sum + recovery(prediction[:n], old_target, old_mask, old_samples)) / old_count
            native_xy = new_sum / new_count
            native_geometry = native_xy + geometry(prediction[n:], new_target, new_mask, native_samples) / new_count
            left = torch.autograd.grad(old_loss, params, retain_graph=True, allow_unused=True)
            for name, loss in (("xy", native_xy), ("xy_geometry", native_geometry)):
                right = torch.autograd.grad(loss, params, retain_graph=name == "xy", allow_unused=True)
                metrics = {key: gradient_alignment([left[i] for i in indices], [right[i] for i in indices])
                           for key, indices in groups.items() if indices}
                reports.append(dict(policy=policy, batch=step, native_objective=name,
                                    old_supported=old_count, native_supported=new_count,
                                    old_loss=float(old_loss.detach()), native_loss=float(loss.detach()),
                                    groups=metrics))
                print(json.dumps(reports[-1]), flush=True)
                del right
            del left, prediction, batch, old_loss, native_xy, native_geometry
            write(args.output / "summary.json", dict(identity=identity, reports=reports))
    print("GRADIENT_DIAGNOSTIC_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
