"""Native WSL entrypoint for verified time cache and finite OFF/ON training."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import gc
import json
from pathlib import Path
import resource
import subprocess

import torch
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import (TimeTrainingCacheDataset,
    prepare_time_training_cache, verify_time_training_cache, _sha)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare-cache", "train"))
    parser.add_argument("--corpus", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--precision", choices=("float32", "bf16"), default="bf16")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(4)
    # Scope the descriptor limit to this process and its loader children.
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    if args.command == "prepare-cache":
        result = prepare_time_training_cache(args.corpus, args.cache)
        print(json.dumps({"cache_complete": True, "manifest_sha256": result["manifest_sha256"]}), flush=True)
        return
    if args.output is None:
        parser.error("--output is required for train")
    if not torch.cuda.is_available():
        raise RuntimeError("real corpus training requires CUDA in WSL")
    cache_identity = verify_time_training_cache(args.cache)
    if _sha(args.corpus / "artifact_manifest.json") != cache_identity["corpus_artifact_manifest_sha256"]:
        raise ValueError("cache does not belong to requested corpus")
    train = TimeTrainingCacheDataset(args.cache, "train", verify_hashes=False)
    validation = TimeTrainingCacheDataset(args.cache, "validation", verify_hashes=False)
    config = TimeModelConfig()
    if train.config != config.dataset_config() or validation.config != config.dataset_config():
        raise ValueError("cached inputs disagree with model preprocessing")
    plan = CorpusTrainingPlan(epochs=args.epochs, batch_size=args.batch_size, workers=args.workers,
                              seed=args.seed, precision=args.precision)
    plan.validate()
    repository = Path(__file__).resolve().parent.parent
    code = sorted((repository / "src" / "aic_transfuser_lite").rglob("*.py"))
    teacher = {"format": "time_corpus_training_identity_v1", "cache_sha256": cache_identity["manifest_sha256"],
        "contract": cache_identity["contract"], "train_anchor_order_sha256": content_sha256(train.anchor_ids),
        "validation_anchor_order_sha256": content_sha256(validation.anchor_ids),
        "train_anchor_count": len(train), "validation_anchor_count": len(validation),
        "source_code_sha256": content_sha256([(p.relative_to(repository).as_posix(), _sha(p)) for p in code])}
    teacher["manifest_sha256"] = content_sha256(teacher)
    source_sha = content_sha256([r for r in train.split_manifest["runs"] if r["split"] == "train"])
    git_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip()
    if args.resume:
        if not args.output.is_dir():
            raise FileNotFoundError("resume output missing")
    else:
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / "teacher_manifest.json").write_text(json.dumps(teacher, indent=2), encoding="utf-8")
    results = []
    for use_command in (False, True):
        name = "command_on" if use_command else "command_off"
        output = args.output / name
        source_commit = git_sha
        if args.resume and (output / "plan.json").is_file():
            source_commit = json.loads((output / "plan.json").read_text())["identity"]["source_git_commit"]
        identity = TimeCheckpointIdentity(train.split_manifest["manifest_sha256"], teacher["manifest_sha256"],
                                          source_sha, "scratch_verified_20laps", source_commit)
        result = run_training_arm(train, validation, train_run_ids=train.run_ids, validation_run_ids=validation.run_ids,
            split_manifest=train.split_manifest, teacher_manifest=teacher, identity=identity,
            config=TimeModelConfig(use_command_history=use_command), plan=plan, output=output,
            resume=args.resume and (output / "last.pt").is_file())
        results.append({"arm": name, **result})
        gc.collect(); torch.cuda.empty_cache()
    if results[0]["initial_weights_sha256"] != results[1]["initial_weights_sha256"]:
        raise RuntimeError("OFF/ON initial weights differ")
    if results[0]["anchors_visited"] != results[1]["anchors_visited"]:
        raise RuntimeError("OFF/ON presentation budgets differ")
    comparison = {"status": "COMPLETE", "arms": results, "initial_weights_equal": True,
        "presentation_budgets_equal": True, "test_evaluated": False,
        "recommended_offline_candidate": min(results, key=lambda r: r["best_validation_run_macro_3s_m"])["arm"],
        "scope": "same_course_same_conditions_run_holdout", "runtime_ready": False}
    (args.output / "comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    print(json.dumps(comparison), flush=True)


if __name__ == "__main__":
    main()
