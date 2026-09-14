"""Native WSL fixed-data comparison of sampling and checkpoint selection."""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import resource
import subprocess
from typing import Any

import numpy as np
import torch
from torch.utils.data import Subset

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.time_outward_balanced_v1 import OutwardBalancedMixDataset
from aic_transfuser_lite.data.time_recovery_training_v1 import MatchedRecoveryMixDataset, RecoveryMixDataset
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score, select_epoch
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe, component_errors, summarize_pp
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm


def read(path: Path) -> Any:
    return json.loads(path.read_bytes())


def write(path: Path, value: Any) -> None:
    path.write_bytes((json.dumps(value, indent=2, allow_nan=False) + "\n").encode())


def training_source_proof(repo: Path, historical: str) -> dict[str, Any]:
    """Allow only the optional epoch archive; compare all training AST otherwise."""
    runner = "src/aic_transfuser_lite/training/time_corpus_runner_v1.py"
    old = subprocess.check_output(["git", "show", historical + ":" + runner], cwd=repo, text=True)
    current = ast.parse((repo / runner).read_text())
    function = next(n for n in current.body if isinstance(n, ast.FunctionDef) and n.name == "run_training_arm")
    at = next(i for i, a in enumerate(function.args.kwonlyargs) if a.arg == "retain_epoch_checkpoints")
    function.args.kwonlyargs.pop(at); function.args.kw_defaults.pop(at)
    permitted = [ast.parse(s).body[0] for s in (
        'if type(retain_epoch_checkpoints) is not bool:\n raise ValueError("retain_epoch_checkpoints must be bool")',
        'if retain_epoch_checkpoints:\n frozen_plan["retain_epoch_checkpoints"] = True',
        'if retain_epoch_checkpoints:\n save(output / f"epoch_{epoch + 1:02d}.pt", epoch + 1, 0)',
    )]
    remaining = {ast.dump(n) for n in permitted}

    class RemoveArchive(ast.NodeTransformer):
        def visit_If(self, node: ast.If) -> ast.AST | None:
            key = ast.dump(node)
            if key in remaining:
                remaining.remove(key)
                return None
            return self.generic_visit(node)

    current = RemoveArchive().visit(current)
    if remaining or ast.dump(current) != ast.dump(ast.parse(old)):
        raise ValueError("training runner changed beyond tested epoch retention")
    paths = ["src/aic_transfuser_lite/models", "src/aic_transfuser_lite/training",
        "src/aic_transfuser_lite/contracts", "src/aic_transfuser_lite/evaluation/time_batched_v1.py",
        "src/aic_transfuser_lite/evaluation/time_metrics_v1.py", "src/aic_transfuser_lite/data/time_training_cache_v1.py",
        "src/aic_transfuser_lite/data/time_dataset_v1.py", "src/aic_transfuser_lite/data/time_teacher_v1.py",
        "src/aic_transfuser_lite/data/image_preprocess.py", "src/aic_transfuser_lite/data/time_recovery_training_v1.py"]
    changed = subprocess.check_output(["git", "diff", "--name-only", historical, "HEAD", "--", *paths], cwd=repo, text=True).splitlines()
    if changed != [runner]:
        raise ValueError(f"other historical training dependencies changed: {changed}")
    return {"historical_source_commit": historical, "only_change": runner,
            "training_ast_equal_after_removing_epoch_archive": True}


def context(root: Path, repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    if plan["reserved_test_usage"] != "sealed" or plan["data_changes"] or plan["loss_changes"]:
        raise ValueError("fixed data/loss and sealed test required")
    source_proof = training_source_proof(repo, plan["historical_source_commit"])
    cache = verify_time_training_cache(root / plan["cache"])
    if cache["manifest_sha256"] != plan["cache_sha256"]:
        raise ValueError("pinned cache changed")
    prior = cache["plan"]
    old_cache = verify_time_training_cache(root / prior["reference_cache"])
    if old_cache["manifest_sha256"] != prior["reference_cache_sha256"]:
        raise ValueError("original presentation reference cache changed")
    collection_path = root / prior["collection_index"]
    if _sha(collection_path) != prior["collection_index_sha256"]:
        raise ValueError("audited outward labels changed")
    collection = read(collection_path)
    train_rows = [r for r in collection["production_runs"] if r["split"] == "train"]
    targets = [a for r in train_rows for a in r["target_anchor_ids"]]
    if len(targets) != plan["expected_outward_train_anchors"] or len(train_rows) != plan["expected_outward_train_runs"]:
        raise ValueError("outward training population changed")
    full = TimeTrainingCacheDataset(root / plan["cache"], "train", verify_hashes=False)
    validation = TimeTrainingCacheDataset(root / plan["cache"], "validation", verify_hashes=False)
    original = TimeTrainingCacheDataset(root / prior["reference_cache"], "train", verify_hashes=False)
    ids = [r["run_id"] for r in prior["runs"] if r["split"] == "train"]
    old_ids = [r["run_id"] for r in old_cache["plan"]["runs"] if r["split"] == "train"]
    reference = RecoveryMixDataset(original, old_ids, repeats=prior["reference_recovery_repeats"])
    uniform = MatchedRecoveryMixDataset(full, reference, ids, seed=plan["training"]["seed"])
    balanced = OutwardBalancedMixDataset(uniform, ids, targets,
        target_fraction=plan["outward_fraction_within_recovery"], seed=plan["training"]["seed"])
    if (len(uniform) != plan["expected_presentations_per_epoch"] or len(full) != prior["expected"]["unique_train"]
            or balanced.audit["target_presentations"] != plan["expected_outward_presentations"]):
        raise ValueError("presentation counts changed")
    target_indices = [i for i, aid in enumerate(full.anchor_ids) if aid in set(targets)]
    if not full.input_valid[target_indices].all() or not full.xy_mask[target_indices].all():
        raise ValueError("outward sampling would repeat unsupported anchors")
    selection = [i for i, rid in enumerate(validation.run_ids) if rid in plan["selection_run_ids"]]
    if (plan["selection_run_ids"] != prior["selection_run_ids"]
            or set(plan["comparison_only_run_ids"]) & set(plan["selection_run_ids"])
            or len(selection) != prior["expected"]["selection_validation"]):
        raise ValueError("validation roles changed")
    historical = root / plan["historical_training"]
    if _sha(historical / "best.pt") != plan["historical_checkpoint_sha256"]:
        raise ValueError("historical control weights changed")
    payload = torch.load(historical / "best.pt", map_location="cpu", weights_only=False)
    historical_plan = read(historical / "plan.json")
    training = CorpusTrainingPlan(**plan["training"])
    if any(historical_plan[k] != v for k, v in asdict(training).items()) or historical_plan["torch_version"] != torch.__version__:
        raise ValueError("training parameters or torch changed")
    if (payload["teacher_manifest"]["train_anchor_order_sha256"] != content_sha256(uniform.anchor_ids)
            or payload["teacher_manifest"]["selection_validation_anchor_ids_sha256"] != content_sha256([validation.anchor_ids[i] for i in selection])):
        raise ValueError("historical data order differs")
    controller = {**read(repo / plan["controller"]), "lookahead_policy": plan["lookahead_policy"]}
    if plan["pp_selection_age_s"] != 0. or plan["pp_rejection_penalty_rad"] != .6:
        raise ValueError("frozen PP selection definition changed")
    return {"cache": cache, "prior": prior, "uniform": uniform, "balanced": balanced,
        "validation": validation, "selection": selection, "historical": historical,
        "controller": controller, "training": training, "source_proof": source_proof,
        "historical_payload": payload, "targets": targets}


def pp_rows(dataset: TimeTrainingCacheDataset, indices: list[int], prediction: np.ndarray,
            controller: dict[str, Any], *, teacher: bool = False) -> list[dict[str, Any]]:
    if prediction.shape != (len(indices), 30, 2):
        raise ValueError("PP input must be [N,30,2] metres")
    rows = []
    for j, i in enumerate(indices):
        anchor = dataset._anchors[i]
        run, local = dataset._index[i]
        speed = float(dataset._runs[run]["inputs"]["ego"][local, -1, 0])
        pose = TimedBodyPose(int(anchor["observation_ns"]), "sim", "0", "map", "base_link", 0., 0., 0.)
        if not dataset.input_valid[i]:
            result = {"applicable": False, "accepted": False, "reason": "INPUT_INVALID"}
        elif teacher and not dataset.xy_mask[i].all():
            result = {"applicable": False, "accepted": False, "reason": "TEACHER_FUTURE_INCOMPLETE"}
        else:
            result = pp_probe(prediction[j], pose, pose, speed, controller)
        rows.append(result)
    return rows


def epoch_scores(ctx: dict[str, Any], training: Path, out: Path) -> dict[str, Any]:
    ds, indices = ctx["validation"], ctx["selection"]
    truth = pp_rows(ds, indices, ds.targets[indices], ctx["controller"], teacher=True)
    runs = [ds.run_ids[i] for i in indices]
    rows = []
    for epoch in range(1, ctx["training"].epochs + 1):
        path = training / f"validation_epoch_{epoch:02d}.npy"
        values = np.load(path, allow_pickle=False)
        pp = pp_rows(ds, indices, values, ctx["controller"])
        metrics = read(training / f"validation_epoch_{epoch:02d}.json")
        rows.append({"epoch": epoch, "prediction_sha256": _sha(path),
            "xy_3s_m": metrics["run_macro_mean"]["3s"]["raw_error_m"],
            "pp": pp_agreement_score(truth, pp, runs)})
        print(json.dumps({"phase": "EPOCH_SCORED", "arm": training.name, **{k: rows[-1][k] for k in ("epoch", "xy_3s_m")},
            "pp_score_rad": rows[-1]["pp"]["run_macro_penalized_rad"]}), flush=True)
    result = {"epochs": rows, "choices": {p: select_epoch(rows, p) for p in ("endpoint_3s", "teacher_pp")},
              "controller": ctx["controller"], "comparison_only_runs_used": False}
    write(out, result)
    return result


def prepare(ctx: dict[str, Any], plan: dict[str, Any], root: Path, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=False)
    write(out / "experiment_plan.json", plan)
    full = ctx["uniform"].dataset
    proof = {"source": ctx["source_proof"], "cache_sha256": ctx["cache"]["manifest_sha256"],
        "data_split_unchanged": True, "unique_train": len(full),
        "unique_recovery_train": ctx["uniform"].unique_recovery_anchors,
        "uniform_presentation_order_sha256": content_sha256(ctx["uniform"].anchor_ids),
        "balanced_presentation_order_sha256": content_sha256(ctx["balanced"].anchor_ids),
        "uniform_outward_presentations": sum(a in set(ctx["targets"]) for a in ctx["uniform"].anchor_ids),
        "balanced": ctx["balanced"].audit, "comparison_only_run_ids": plan["comparison_only_run_ids"],
        "test_data_read": False, "awsim_started": False}
    write(out / "data_and_budget_verification.json", proof)
    choices = epoch_scores(ctx, ctx["historical"], out / "uniform_selection.json")
    best = read(ctx["historical"] / "result.json")["best_epoch"]
    payload = ctx["historical_payload"]
    config, identity = TimeModelConfig.from_dict(payload["config"]), TimeCheckpointIdentity(**payload["identity"])
    model = build_time_model(config).to("cuda")
    load_time_checkpoint(ctx["historical"] / "best.pt", config=config, identity=identity, model=model, mode="finetune")
    ds, ix = ctx["validation"], ctx["selection"]
    metrics, predicted = evaluate_time_batched(model, Subset(ds, ix), run_ids=[ds.run_ids[i] for i in ix],
        split_manifest=ctx["cache"]["split_manifest"], batch_size=32, workers=4, precision="float32")
    np.testing.assert_allclose(predicted.numpy(), np.load(ctx["historical"] / f"validation_epoch_{best:02d}.npy"), rtol=0, atol=0, equal_nan=True)
    if metrics != read(ctx["historical"] / "best_validation.json"):
        raise ValueError("historical validation did not replay exactly")
    write(out / "prepare_result.json", {"status": "PREPARED", "historical_best_replayed_exactly": True,
        "uniform_retraining_needed": any(e != best for e in choices["choices"].values()),
        "uniform_choices": choices["choices"], "comparison_data_not_used_for_selection": True})
    print("METHOD_COMPARISON_PREPARED", flush=True)


def train(ctx: dict[str, Any], plan: dict[str, Any], root: Path, out: Path, arm: str, resume: bool) -> None:
    prepared = read(out / "prepare_result.json")
    if prepared["status"] != "PREPARED" or read(out / "experiment_plan.json") != plan:
        raise ValueError("verified preparation and frozen plan required")
    if arm == "uniform" and not prepared["uniform_retraining_needed"]:
        raise ValueError("historical selected checkpoints exist; uniform retraining not budgeted")
    ds, validation, ix = ctx[arm], ctx["validation"], ctx["selection"]
    cache, prior = ctx["cache"], ctx["prior"]
    source = root / prior["initialization"]
    if _sha(source) != prior["initialization_sha256"]:
        raise ValueError("initial weights changed")
    payload = torch.load(source, map_location="cpu", weights_only=False)
    config, source_identity = TimeModelConfig.from_dict(payload["config"]), TimeCheckpointIdentity(**payload["identity"])
    teacher = {"format": "fixed_data_method_comparison_v1", "cache_sha256": cache["manifest_sha256"],
        "arm": arm, "plan": plan, "train_order_sha256": content_sha256(ds.anchor_ids),
        "selection_order_sha256": content_sha256([validation.anchor_ids[i] for i in ix])}
    teacher["manifest_sha256"] = content_sha256(teacher)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    identity = TimeCheckpointIdentity(cache["split_manifest"]["manifest_sha256"], teacher["manifest_sha256"],
        content_sha256(ds.anchor_ids), "fixed_data_method_" + arm, commit)
    destination = out / (arm + "_training")
    if resume:
        identity = TimeCheckpointIdentity(**read(destination / "plan.json")["identity"])
    result = run_training_arm(ds, Subset(validation, ix), train_run_ids=ds.run_ids,
        validation_run_ids=[validation.run_ids[i] for i in ix], split_manifest=cache["split_manifest"],
        teacher_manifest=teacher, identity=identity, config=config, plan=ctx["training"], output=destination,
        initialization=source, initialization_sha256=prior["initialization_sha256"],
        initialization_identity=source_identity, resume=resume, retain_epoch_checkpoints=True)
    if result["optimizer_steps"] != plan["expected_optimizer_steps"]:
        raise ValueError("optimizer update budget differs")
    history = read(destination / "history.json")
    original = read(ctx["historical"] / "history.json")
    if read(destination / "initial_validation.json") != read(ctx["historical"] / "initial_validation.json"):
        raise ValueError("same initialization validation differed")
    for current, old in zip(history, original, strict=True):
        if any(current["train_counts"][k] != old["train_counts"][k]
               for k in ("visited", "input_invalid", "teacher_unsupported", "supported")):
            raise ValueError("training support budget differs")
        if arm == "uniform":
            epoch = current["epoch"]
            np.testing.assert_allclose(np.load(destination / f"validation_epoch_{epoch:02d}.npy"),
                np.load(ctx["historical"] / f"validation_epoch_{epoch:02d}.npy"), rtol=0, atol=0, equal_nan=True)
    epoch_scores(ctx, destination, out / (arm + "_selection.json"))
    write(out / (arm + "_verification.json"), {"status": "COMPLETE", "initial_validation_exact": True,
        "support_and_update_budget_matched": True, "uniform_history_exact": arm == "uniform",
        "optimizer_steps": result["optimizer_steps"], "epoch_checkpoints_retained": ctx["training"].epochs})


def compare(ctx: dict[str, Any], plan: dict[str, Any], root: Path, out: Path) -> None:
    if read(out / "experiment_plan.json") != plan or read(out / "balanced_verification.json")["status"] != "COMPLETE":
        raise ValueError("completed frozen training required")
    result_dir = out / "comparison"
    result_dir.mkdir(exist_ok=False)
    ds = ctx["validation"]
    all_indices = list(range(len(ds)))
    old_ids = {r["run_id"] for r in ctx["prior"]["runs"] if r["root"] != "expansion" and r["split"] == "validation"}
    new_ids = set(plan["comparison_only_run_ids"])
    collection = read(root / ctx["prior"]["collection_index"])
    target_ids = {a for r in collection["production_runs"] if r["run_id"] in new_ids for a in r["target_anchor_ids"]}
    groups = {"nominal": [i for i, r in enumerate(ds.run_ids) if r not in old_ids | new_ids],
        "old_recovery": [i for i, r in enumerate(ds.run_ids) if r in old_ids],
        "comparison_recovery": [i for i, r in enumerate(ds.run_ids) if r in new_ids],
        "comparison_outward": [i for i, a in enumerate(ds.anchor_ids) if a in target_ids]}
    for rid in sorted(new_ids):
        groups[rid] = [i for i, r in enumerate(ds.run_ids) if r == rid]
    truth = pp_rows(ds, all_indices, ds.targets, ctx["controller"], teacher=True)
    reports, evaluated, cells = {}, {}, {}
    for arm in ("uniform", "balanced"):
        chosen = read(out / (arm + "_selection.json"))["choices"]
        for policy, epoch in chosen.items():
            if arm == "uniform" and not read(out / "prepare_result.json")["uniform_retraining_needed"]:
                checkpoint = ctx["historical"] / "best.pt"
            else:
                if read(out / (arm + "_verification.json"))["status"] != "COMPLETE":
                    raise ValueError("selected training arm incomplete")
                checkpoint = out / (arm + "_training") / f"epoch_{epoch:02d}.pt"
            key = arm + "_epoch" + str(epoch)
            cells[arm + "/" + policy] = key
            if key in evaluated:
                continue
            payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
            if payload["epoch"] != epoch:
                raise ValueError("selected checkpoint epoch mismatch")
            config, identity = TimeModelConfig.from_dict(payload["config"]), TimeCheckpointIdentity(**payload["identity"])
            model = build_time_model(config).to("cuda")
            load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode="finetune")
            metrics, tensor = evaluate_time_batched(model, ds, run_ids=ds.run_ids,
                split_manifest=ctx["cache"]["split_manifest"], batch_size=32, workers=4, precision="float32")
            prediction = tensor.numpy()
            np.save(result_dir / (key + "_predictions.npy"), prediction, allow_pickle=False)
            source_dir = ctx["historical"] if checkpoint.parent == ctx["historical"] else checkpoint.parent
            np.testing.assert_allclose(prediction[ctx["selection"]],
                np.load(source_dir / f"validation_epoch_{epoch:02d}.npy"), rtol=0, atol=0, equal_nan=True)
            pps = pp_rows(ds, all_indices, prediction, ctx["controller"])
            report = {"epoch": epoch, "checkpoint": str(checkpoint), "sha256": _sha(checkpoint), "groups": {}}
            for group, ix in groups.items():
                runs = [ds.run_ids[i] for i in ix]
                group_metrics = time_horizon_metrics(tensor[ix], torch.from_numpy(ds.targets[ix]),
                    torch.from_numpy(ds.xy_mask[ix]), input_valid=torch.from_numpy(ds.input_valid[ix]), run_ids=runs)
                report["groups"][group] = {"xy": group_metrics,
                    "components": component_errors(prediction[ix], ds.targets[ix], ds.xy_mask[ix], ds.input_valid[ix], runs),
                    "pp": pp_agreement_score([truth[i] for i in ix], [pps[i] for i in ix], runs),
                    "pp_applicability": summarize_pp([pps[i] for i in ix])}
            reports[key] = report; evaluated[key] = True
            write(result_dir / (key + "_metrics.json"), report)
            del model, payload; torch.cuda.empty_cache()
            print(json.dumps({"phase": "COMPARISON_MODEL_COMPLETE", "model": key, "epoch": epoch}), flush=True)
    summary = {"status": "COMPLETE", "scope": plan["scope"], "cells": cells, "models": reports,
        "selection_runs": plan["selection_run_ids"], "comparison_only_runs": plan["comparison_only_run_ids"],
        "comparison_runs_previously_reported": True, "sealed_test_read": False,
        "data_and_loss_unchanged": True, "new_awsim_trials": 0}
    write(result_dir / "summary.json", summary)
    print("METHOD_COMPARISON_COMPLETE", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "train", "compare"))
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path(".."))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arm", choices=("uniform", "balanced"), default="balanced")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(4)
    if not torch.cuda.is_available():
        raise RuntimeError("native WSL CUDA is required")
    torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    root, repo, plan = args.root.resolve(), Path(__file__).resolve().parents[1], read(args.plan)
    ctx = context(root, repo, plan)
    if args.command == "prepare":
        prepare(ctx, plan, root, args.output)
    elif args.command == "train":
        train(ctx, plan, root, args.output, args.arm, args.resume)
    else:
        compare(ctx, plan, root, args.output)


if __name__ == "__main__":
    main()
