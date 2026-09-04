from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from typing import Any, Sequence
from urllib.parse import unquote, urlparse

from .data.audit_v3 import audit_dataset
from .data.bag_inventory import BagInventoryRecord, discover_bag_inventories
from .data.behavior_view_v1 import build_behavior_view_v1
from .data.canonical_converter_v3 import (
    convert_decoded_run_v3,
    load_dataset_v3_converter_config,
    write_prepared_dataset_v3,
)
from .data.clock_segments import ClockEpoch
from .contracts.model_batch_v3 import (
    COMMAND_HISTORY_ALIGNMENT_V3,
    CONTROL_SEQUENCE_ALIGNMENT_V3,
)
from .data.dataset_view_v3 import (
    ControlTargetBoundsV3,
    load_temporal_training_batches_v3,
    load_v1_compatibility_view_config,
)
from .data.mcap_reader_v3 import read_run_messages_v3
from .data.split_v3 import (
    SplitGroupKey,
    SplitRunRecord,
    build_split_manifest_v3,
    load_split_config_v3,
)
from .data.storage_v3 import validate_complete_dataset
from .data.topic_profile_v3 import assess_topic_profile_v3, load_topic_profile_v3
from .training.checkpoint_v3 import ExperimentIdentityV3
from .training.losses_v3 import LossWeightsV3
from .training.train_v3 import (
    TrainerV3, balanced_class_weights_v3, build_full_control_model_v3,
    evaluate_trajectory_speed_v3, full_control_model_kwargs_v3,
    is_better_trajectory_checkpoint_v3, launch_readiness_gate_config_v3,
    load_full_control_config_v3,
    motion_target_filter_config_v3,
)
import torch
import yaml


EXIT_SUCCESS = 0
EXIT_VALIDATION = 2
EXIT_PARTIAL = 3
EXIT_COMPATIBILITY = 4
EXIT_GATE = 5
EXIT_INTERNAL = 10


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aic-e2e")
    domains = parser.add_subparsers(dest="domain", required=True)

    bag = domains.add_parser("bag")
    bag_commands = bag.add_subparsers(dest="command", required=True)
    bag_scan = bag_commands.add_parser("scan")
    _input_output(bag_scan)
    bag_validate = bag_commands.add_parser("validate")
    _input_output(bag_validate)
    bag_validate.add_argument("--config", required=True, help="Topic profile V3 YAML")

    dataset = domains.add_parser("dataset")
    dataset_commands = dataset.add_subparsers(dest="command", required=True)
    dataset_build = dataset_commands.add_parser("build")
    _input_output(dataset_build)
    dataset_build.add_argument("--config", required=True, help="Dataset V3 converter YAML")
    dataset_build.add_argument("--topic-profile", required=True)
    dataset_build.add_argument("--dataset-id", required=True)
    dataset_build.add_argument("--scenario-id", default="unknown")
    dataset_build.add_argument("--resume", action="store_true")
    dataset_build.add_argument("--dry-run", action="store_true")
    dataset_build.add_argument("--retry-failed", action="store_true")
    dataset_build.add_argument("--jobs", type=int, default=1)

    dataset_audit = dataset_commands.add_parser("audit")
    dataset_audit.add_argument("--dataset-root", required=True)
    dataset_audit.add_argument("--output", required=True)

    dataset_split = dataset_commands.add_parser("split")
    dataset_split.add_argument("--runs-json", required=True)
    dataset_split.add_argument("--dataset-manifest-sha256", required=True)
    dataset_split.add_argument("--config", required=True)
    dataset_split.add_argument("--output", required=True)
    dataset_split.add_argument("--dry-run", action="store_true")
    dataset_split.add_argument("--resume", action="store_true")

    view = domains.add_parser("view")
    view_commands = view.add_subparsers(dest="command", required=True)
    view_build = view_commands.add_parser("build")
    view_build.add_argument("--dataset-root", required=True)
    view_build.add_argument("--config", required=True)
    view_build.add_argument("--output", required=True)
    view_build.add_argument("--dry-run", action="store_true")
    view_build.add_argument("--resume", action="store_true")

    behavior = domains.add_parser("behavior")
    behavior_commands = behavior.add_subparsers(dest="command", required=True)
    behavior_build = behavior_commands.add_parser("build")
    behavior_build.add_argument("--dataset-root", required=True)
    behavior_build.add_argument(
        "--run-source", nargs=3, action="append", required=True,
        metavar=("RUN_ID", "AUTOWARE_LOG", "BAG_DIRECTORY"),
    )
    behavior_build.add_argument("--output", required=True)
    behavior_build.add_argument("--max-gap-ms", type=float, default=500.0)

    train = domains.add_parser("train")
    train.add_argument("--config", required=True)
    train.add_argument("--dataset-root", required=True)
    train.add_argument("--split-manifest", required=True)
    train.add_argument("--view-config", default="configs/data/view_temporal_v3.yaml")
    train.add_argument("--behavior-view", required=True)
    train.add_argument("--output", required=True)
    train.add_argument("--epochs", type=int)
    train.add_argument("--batch-size", type=int)
    train.add_argument("--device", default="auto")
    train.add_argument("--max-batches", type=int)
    train.add_argument("--checkpoint-every-steps", type=int)
    train.add_argument("--init-checkpoint")
    train.add_argument("--resume-initialization-checkpoint")
    train.add_argument("--freeze-migrated", action="store_true")
    train.add_argument("--resume", action="store_true")
    train.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        if args.domain == "bag" and args.command == "scan":
            return _bag_scan(args)
        if args.domain == "bag" and args.command == "validate":
            return _bag_validate(args)
        if args.domain == "dataset" and args.command == "build":
            return _dataset_build(args)
        if args.domain == "dataset" and args.command == "audit":
            return _dataset_audit(args)
        if args.domain == "dataset" and args.command == "split":
            return _dataset_split(args)
        if args.domain == "view" and args.command == "build":
            return _view_build(args)
        if args.domain == "behavior" and args.command == "build":
            return _behavior_build(args)
        if args.domain == "train":
            return _train_v3(args)
        raise ValueError("unknown command")
    except (ValueError, TypeError, KeyError, FileNotFoundError, FileExistsError) as error:
        _emit_error(EXIT_VALIDATION, error)
        return EXIT_VALIDATION
    except Exception as error:  # boundary converts unexpected failures to a structured CLI status
        _emit_error(EXIT_INTERNAL, error)
        return EXIT_INTERNAL


def _bag_scan(args: argparse.Namespace) -> int:
    records = _selected_inventories(args.input_root, args.only_run)
    payload = {"format_version": "aic_bag_inventory_v1", "bags": [item.to_dict() for item in records]}
    _write_json(Path(args.output), payload, resume=False)
    return EXIT_SUCCESS if all(item.scan_status == "PASS" for item in records) else EXIT_PARTIAL


def _bag_validate(args: argparse.Namespace) -> int:
    profile = load_topic_profile_v3(args.config)
    records = _selected_inventories(args.input_root, args.only_run)
    results = []
    failed = False
    for record in records:
        observed = {topic.name: topic.message_type for topic in record.topics}
        assessment = assess_topic_profile_v3(profile, observed)
        accepted = record.scan_status == "PASS" and assessment.conversion_accepted
        failed |= not accepted
        results.append(
            {
                "bag_id": record.bag_id,
                "accepted_for_conversion": accepted,
                "available_roles": sorted(assessment.available_roles),
                "missing_for_recording": list(assessment.missing_for_recording),
                "missing_for_conversion": list(assessment.missing_for_conversion),
                "capabilities": sorted(assessment.capabilities.available),
                "unavailable_capabilities": assessment.capabilities.unavailable,
                "scan_errors": list(record.scan_errors),
            }
        )
    _write_json(Path(args.output), {"format_version": "aic_bag_validation_v1", "bags": results}, resume=False)
    return EXIT_PARTIAL if failed else EXIT_SUCCESS


def _dataset_build(args: argparse.Namespace) -> int:
    if args.jobs < 1:
        raise ValueError("--jobs must be at least one")
    config = load_dataset_v3_converter_config(args.config)
    profile = load_topic_profile_v3(args.topic_profile)
    output = Path(args.output).resolve()
    if args.resume and output.is_dir():
        manifest = validate_complete_dataset(output)
        if manifest.get("dataset_id") != args.dataset_id:
            raise ValueError("resume dataset_id differs from completed output")
        print(json.dumps({"status": "COMPLETE", "resumed": True, "output": str(output)}))
        return EXIT_SUCCESS
    records = _selected_inventories(args.input_root, args.only_run)
    plan = {
        "format_version": "aic_dataset_build_plan_v1",
        "dataset_id": args.dataset_id,
        "output": str(output),
        "runs": [record.bag_id for record in records],
        "jobs": args.jobs,
        "retry_failed": bool(args.retry_failed),
    }
    if args.dry_run:
        print(json.dumps(plan, ensure_ascii=False, sort_keys=True))
        return EXIT_SUCCESS
    prepared = []
    for record in records:
        if record.scan_status != "PASS":
            raise ValueError(f"bag {record.bag_id} failed metadata scan: {record.scan_errors}")
        observed = {topic.name: topic.message_type for topic in record.topics}
        assessment = assess_topic_profile_v3(profile, observed)
        if not assessment.conversion_accepted:
            raise ValueError(
                f"bag {record.bag_id} lacks conversion roles: {assessment.missing_for_conversion}"
            )
        bag_dir = _file_uri_to_path(record.source_path)
        streams = read_run_messages_v3(bag_dir, profile=profile)
        stamps = [item.timestamp_ns for item in streams.images]
        if not stamps:
            raise ValueError(f"bag {record.bag_id} has no Camera samples")
        epoch = ClockEpoch(
            "epoch0000", 0, len(stamps) - 1, min(stamps), max(stamps), min(stamps), max(stamps), None
        )
        content_hash = hashlib.sha256(
            f"{record.metadata_sha256}:{record.storage_sha256}".encode("utf-8")
        ).hexdigest()
        prepared.append(
            convert_decoded_run_v3(
                streams,
                run_id=bag_dir.name,
                scenario_id=args.scenario_id,
                source_uri=record.source_path,
                source_hash=content_hash,
                topic_profile_id=profile.profile_id,
                epochs=(epoch,),
                config=config,
            )
        )
    summary = write_prepared_dataset_v3(
        output,
        dataset_id=args.dataset_id,
        topic_profile_id=profile.profile_id,
        runs=prepared,
        jpeg_quality=config.jpeg_quality,
    )
    print(json.dumps({"status": "COMPLETE", "manifest_sha256": summary.manifest_sha256}))
    return EXIT_SUCCESS


def _dataset_audit(args: argparse.Namespace) -> int:
    report = audit_dataset(args.dataset_root, output_directory=args.output)
    print(json.dumps({"status": report.overall_status.value, "dataset_id": report.dataset_id}))
    return EXIT_GATE if report.overall_status.value == "FAIL" else EXIT_SUCCESS


def _dataset_split(args: argparse.Namespace) -> int:
    raw = json.loads(Path(args.runs_json).read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise ValueError("runs JSON must be a list")
    runs = []
    group_fields = tuple(SplitGroupKey.__dataclass_fields__)
    for item in raw:
        if not isinstance(item, dict) or not isinstance(item.get("group"), dict):
            raise ValueError("each split run must contain a group mapping")
        runs.append(
            SplitRunRecord(
                run_id=str(item["run_id"]),
                source_hash=str(item["source_hash"]),
                group=SplitGroupKey(**{name: str(item["group"].get(name, "")) for name in group_fields}),
                trajectory_fingerprint=str(item.get("trajectory_fingerprint", "unknown")),
            )
        )
    manifest = build_split_manifest_v3(
        runs,
        dataset_manifest_sha256=args.dataset_manifest_sha256,
        config=load_split_config_v3(args.config),
    )
    if args.dry_run:
        print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    else:
        _write_json(Path(args.output), manifest, resume=args.resume)
    return EXIT_SUCCESS


def _view_build(args: argparse.Namespace) -> int:
    dataset_manifest = validate_complete_dataset(args.dataset_root)
    config = load_v1_compatibility_view_config(args.config)
    payload = {
        "format_version": "aic_model_view_manifest_v1",
        "view_id": config.view_id,
        "dataset_id": dataset_manifest["dataset_id"],
        "dataset_manifest_sha256": dataset_manifest["manifest_sha256"],
        "view_config_sha256": _sha256(Path(args.config)),
        "materialization": "lazy_from_canonical_assets",
    }
    payload["manifest_sha256"] = hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    if args.dry_run:
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    else:
        _write_json(Path(args.output), payload, resume=args.resume)
    return EXIT_SUCCESS


def _behavior_build(args: argparse.Namespace) -> int:
    payload = build_behavior_view_v1(
        dataset_root=args.dataset_root,
        run_sources=tuple((item[0], item[1], item[2]) for item in args.run_source),
        output_root=args.output,
        max_gap_ms=float(args.max_gap_ms),
    )
    print(json.dumps({
        "status": "COMPLETE", "ontology": payload["ontology"],
        "valid_behavior_count": payload["valid_behavior_count"],
    }))
    return EXIT_SUCCESS


def _train_v3(args: argparse.Namespace) -> int:
    if args.resume and args.init_checkpoint:
        raise ValueError("--resume and --init-checkpoint are mutually exclusive")
    if args.init_checkpoint and args.resume_initialization_checkpoint:
        raise ValueError(
            "--init-checkpoint and --resume-initialization-checkpoint are mutually exclusive"
        )
    if args.freeze_migrated and not (args.init_checkpoint or args.resume):
        raise ValueError("--freeze-migrated requires initialization or resume")
    if args.resume_initialization_checkpoint and not args.resume:
        raise ValueError("--resume-initialization-checkpoint requires --resume")
    config = load_full_control_config_v3(args.config)
    launch_gate = launch_readiness_gate_config_v3(config)
    model_cfg, data_cfg, loss_cfg, training_cfg = (
        config["model"], config["data"], config["loss"], config["training"]
    )
    view = yaml.safe_load(Path(args.view_config).read_text(encoding="utf-8"))
    if (
        not isinstance(view, dict)
        or view.get("command_history_alignment") != COMMAND_HISTORY_ALIGNMENT_V3
    ):
        raise ValueError("temporal view requires causal command history alignment")
    if view.get("control_sequence_alignment") != CONTROL_SEQUENCE_ALIGNMENT_V3:
        raise ValueError("temporal view requires exact-grid control sequence alignment")
    if int(view.get("command_history_length", 0)) != int(
        data_cfg["command_history_length"]
    ):
        raise ValueError("temporal view and model command history lengths differ")
    batch_size = int(args.batch_size or training_cfg["micro_batch_size"])
    epochs = int(args.epochs or training_cfg["epochs"])
    if epochs <= 0 or batch_size <= 0:
        raise ValueError("epochs and batch size must be positive")
    control_target_bounds = ControlTargetBoundsV3(
        max_steering_rad=float(model_cfg["control_bounds"]["max_steering_rad"]),
        max_steering_rate_radps=float(
            model_cfg["control_bounds"]["max_steering_rate_radps"]
        ),
        max_speed_mps=float(model_cfg["control_bounds"]["max_speed_mps"]),
        min_acceleration_mps2=float(
            model_cfg["control_bounds"]["min_acceleration_mps2"]
        ),
        max_acceleration_mps2=float(
            model_cfg["control_bounds"]["max_acceleration_mps2"]
        ),
        min_jerk_mps3=float(model_cfg["control_bounds"]["min_jerk_mps3"]),
        max_jerk_mps3=float(model_cfg["control_bounds"]["max_jerk_mps3"]),
        control_dt_sec=float(model_cfg["control_dt_sec"]),
    )
    batch_loader_arguments = {
        "image_height": int(data_cfg["image_height"]),
        "image_width": int(data_cfg["image_width"]),
        "lidar_points": int(data_cfg["lidar_points"]),
        "lidar_min_range_m": float(data_cfg["lidar_min_range_m"]),
        "lidar_max_range_m": float(data_cfg["lidar_max_range_m"]),
        "ego_features": tuple(data_cfg["ego_features"]),
        "ego_abs_limits": data_cfg.get("ego_abs_limits"),
        "trajectory_steps": int(model_cfg["trajectory_steps"]),
        "control_sequence_steps": int(model_cfg["control_sequence_steps"]),
        "camera_history_length": int(view["camera_history_length"]),
        "ego_history_length": int(view["ego_history_length"]),
        "command_history_length": int(view["command_history_length"]),
        "control_target_bounds": control_target_bounds,
        "batch_size": batch_size,
        "behavior_view_root": args.behavior_view,
        "motion_target_filter": motion_target_filter_config_v3(config),
    }
    batches = load_temporal_training_batches_v3(
        args.dataset_root, args.split_manifest, split="train",
        max_batches=args.max_batches, **batch_loader_arguments,
    )
    validation_batches = None
    if args.max_batches is None:
        validation_batches = load_temporal_training_batches_v3(
            args.dataset_root,
            args.split_manifest,
            split="validation",
            **batch_loader_arguments,
        )
    behavior_weights = balanced_class_weights_v3(
        batches, target_name="behavior_class", mask_name="behavior_mask", class_count=5,
        require_all_classes=args.max_batches is None,
    )
    side_weights = balanced_class_weights_v3(
        batches, target_name="behavior_side", mask_name="behavior_side_mask", class_count=3,
        require_all_classes=args.max_batches is None,
    )
    dataset_manifest = validate_complete_dataset(args.dataset_root)
    contract_hash = hashlib.sha256(
        (
            Path("schemas/model_batch_v3.schema.json").read_bytes()
            + Path("schemas/model_output_v3.schema.json").read_bytes()
            + Path(args.config).read_bytes()
        )
    ).hexdigest()
    identity = ExperimentIdentityV3(
        dataset_hash=str(dataset_manifest["manifest_sha256"]),
        split_hash=_sha256(Path(args.split_manifest)),
        view_hash=_combined_sha256(
            Path(args.view_config), Path(args.behavior_view) / "manifest.json"
        ),
        contract_hash=contract_hash, seed=42,
    )
    requested = args.device
    requested = ("cuda" if torch.cuda.is_available() else "cpu") if requested == "auto" else requested
    device = torch.device(requested)
    if args.dry_run:
        print(json.dumps({
            "status": "DRY_RUN", "batches": len(batches), "device": str(device),
            "validation_batches": (
                None if validation_batches is None else len(validation_batches)
            ),
            "behavior_class_weights": behavior_weights,
            "behavior_side_class_weights": side_weights,
            "motion_target_rejected_train": int(
                getattr(batches, "motion_target_rejected_count", 0)
            ),
            "motion_target_rejected_validation": (
                None
                if validation_batches is None
                else int(
                    getattr(validation_batches, "motion_target_rejected_count", 0)
                )
            ),
            "launch_readiness_gate_enabled": launch_gate is not None,
        }))
        return EXIT_SUCCESS
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    model = build_full_control_model_v3(config).to(device)
    initialization = None
    if args.init_checkpoint:
        initialization_path = Path(args.init_checkpoint).expanduser().resolve()
        payload = torch.load(initialization_path, map_location="cpu", weights_only=True)
        if not isinstance(payload, dict) or not isinstance(payload.get("model"), dict):
            raise ValueError("initial checkpoint has no model state mapping")
        migration = model.migrate_v1_weights(payload["model"])
        if not migration.loaded:
            raise ValueError("initial checkpoint has no compatible model weights")
        initialization = {
            "checkpoint_sha256": _sha256(initialization_path),
            "freeze_migrated": bool(args.freeze_migrated),
            "loaded_key_count": len(migration.loaded),
            "shape_mismatch": list(migration.shape_mismatch),
            "unmapped_source": list(migration.unmapped_v1),
            "new_key_count": len(migration.new_v3),
        }
    elif args.resume_initialization_checkpoint:
        initialization_path = Path(
            args.resume_initialization_checkpoint
        ).expanduser().resolve()
        payload = torch.load(initialization_path, map_location="cpu", weights_only=True)
        if not isinstance(payload, dict) or not isinstance(payload.get("model"), dict):
            raise ValueError("resume initialization checkpoint has no model state mapping")
        migration = model.migrate_v1_weights(payload["model"])
        if not migration.loaded:
            raise ValueError("resume initialization checkpoint has no compatible model weights")
        initialization = {
            "checkpoint_sha256": _sha256(initialization_path),
            "freeze_migrated": False,
            "loaded_key_count": len(migration.loaded),
            "shape_mismatch": list(migration.shape_mismatch),
            "unmapped_source": list(migration.unmapped_v1),
            "new_key_count": len(migration.new_v3),
            "resume_provenance_only": True,
        }
    if args.freeze_migrated:
        model.freeze_except_control_sequence = True
        for name, parameter in model.named_parameters():
            if not name.startswith("control_sequence_head."):
                parameter.requires_grad_(False)
    trainable = [parameter for parameter in model.parameters() if parameter.requires_grad]
    if not trainable:
        raise ValueError("training configuration froze every model parameter")
    optimizer = torch.optim.AdamW(trainable, lr=1e-4)
    trainer = TrainerV3(
        model=model, batches=batches, optimizer=optimizer,
        identity=identity,
        loss_weights=LossWeightsV3(
            trajectory=float(loss_cfg["trajectory"]),
            speed_profile=float(loss_cfg["speed_profile"]),
            current_control=float(loss_cfg["current_control"]),
            behavior=float(loss_cfg["behavior"]),
            behavior_side=float(loss_cfg["behavior_side"]),
            behavior_class_weights=behavior_weights,
            behavior_side_class_weights=side_weights,
            control_sequence=float(loss_cfg["control_sequence"]),
            plan_consistency=float(loss_cfg.get("plan_consistency", 0.0)),
            plan_step_sec=float(loss_cfg.get("plan_step_sec", 0.1)),
        ),
        gradient_accumulation_steps=int(
            training_cfg.get("gradient_accumulation_steps", 1)
        ),
    )
    checkpoint = output / "last.pt"
    if args.resume:
        if not checkpoint.is_file():
            raise FileNotFoundError(f"resume checkpoint missing: {checkpoint}")
        trainer.resume(checkpoint)
    gradient_accumulation_steps = int(
        training_cfg.get("gradient_accumulation_steps", 1)
    )
    optimizer_steps_per_epoch = math.ceil(
        len(batches) / gradient_accumulation_steps
    )
    target_steps = epochs * optimizer_steps_per_epoch
    checkpoint_every_steps = int(
        training_cfg["checkpoint_every_steps"]
        if args.checkpoint_every_steps is None
        else args.checkpoint_every_steps
    )
    if checkpoint_every_steps <= 0:
        raise ValueError("--checkpoint-every-steps must be positive")
    validation_history_path = output / "validation_history.json"
    validation_history: list[dict[str, Any]] = []
    best_metrics: dict[str, float] | None = None
    if args.resume and validation_history_path.is_file():
        prior_validation = json.loads(
            validation_history_path.read_text(encoding="utf-8")
        )
        if prior_validation.get("identity") != identity.__dict__:
            raise ValueError("validation history experiment identity mismatch")
        if int(prior_validation.get("optimizer_steps_per_epoch", -1)) != (
            optimizer_steps_per_epoch
        ):
            raise ValueError("validation history epoch size mismatch")
        validation_history = list(prior_validation.get("epochs", []))
        if validation_history and int(validation_history[-1]["global_step"]) > trainer.global_step:
            raise ValueError("validation history is ahead of the resume checkpoint")
        selected = prior_validation.get("best")
        if selected is not None:
            if not (output / "best_trajectory.pt").is_file():
                raise FileNotFoundError(
                    "validation history selects a missing best_trajectory.pt"
                )
            best_metrics = {
                "trajectory_ade_m": float(selected["trajectory_ade_m"]),
                "speed_profile_mae_mps": float(selected["speed_profile_mae_mps"]),
            }

    def validate_epoch_boundary() -> None:
        nonlocal best_metrics
        if validation_batches is None:
            return
        if trainer.sampler.offset != len(batches):
            raise RuntimeError("validation requested outside an exact sampler epoch boundary")
        epoch = trainer.sampler.epoch + 1
        if any(int(row["epoch"]) == epoch for row in validation_history):
            return
        metrics = evaluate_trajectory_speed_v3(
            model, validation_batches, launch_gate=launch_gate
        )
        row: dict[str, Any] = {
            "epoch": epoch,
            "global_step": trainer.global_step,
            **metrics,
        }
        epoch_checkpoint = output / f"epoch_{epoch:03d}.pt"
        _atomic_copy(checkpoint, epoch_checkpoint)
        row["checkpoint"] = epoch_checkpoint.name
        launch_gate_passed = (
            launch_gate is None or bool(metrics["launch_gate_pass"])
        )
        if launch_gate_passed and is_better_trajectory_checkpoint_v3(
            metrics, best_metrics
        ):
            best_metrics = {
                "trajectory_ade_m": float(metrics["trajectory_ade_m"]),
                "speed_profile_mae_mps": float(metrics["speed_profile_mae_mps"]),
            }
            _atomic_copy(checkpoint, output / "best_trajectory.pt")
            row["promoted"] = True
        else:
            row["promoted"] = False
        if launch_gate is not None:
            row["promotion_gate"] = "pass" if launch_gate_passed else "launch_failed"
        validation_history.append(row)
        eligible_rows = [
            item
            for item in validation_history
            if launch_gate is None or bool(item.get("launch_gate_pass", False))
        ]
        best_row = (
            None
            if not eligible_rows
            else min(
                eligible_rows,
                key=lambda item: (
                    float(item["trajectory_ade_m"]),
                    float(item["speed_profile_mae_mps"]),
                ),
            )
        )
        _write_json_atomic(
            validation_history_path,
            {
                "format": "aic_v3_validation_history_v1",
                "identity": identity.__dict__,
                "optimizer_steps_per_epoch": optimizer_steps_per_epoch,
                "selection": (
                    ["trajectory_ade_m", "speed_profile_mae_mps"]
                    if launch_gate is None
                    else [
                        "launch_gate_pass",
                        "trajectory_ade_m",
                        "speed_profile_mae_mps",
                    ]
                ),
                "epochs": validation_history,
                "best": best_row,
            },
        )

    completed_epochs = trainer.sampler.epoch + int(
        trainer.sampler.offset == len(batches)
    )
    if completed_epochs > epochs or trainer.global_step > target_steps:
        raise ValueError("resume checkpoint is ahead of requested training epochs")
    if trainer.sampler.offset == len(batches) and completed_epochs > 0:
        validate_epoch_boundary()
    if completed_epochs == epochs:
        trainer.save(checkpoint)
    while completed_epochs < epochs:
        micro_batches_remaining = (
            len(batches)
            if trainer.sampler.offset == len(batches)
            else len(batches) - trainer.sampler.offset
        )
        full_optimizer_steps = (
            micro_batches_remaining // gradient_accumulation_steps
        )
        if full_optimizer_steps > 0:
            step_count = min(checkpoint_every_steps, full_optimizer_steps)
            trainer.train_steps(step_count)
        else:
            trainer.train_steps(
                1,
                micro_batches_per_optimizer_step=micro_batches_remaining,
            )
        trainer.save(checkpoint)
        if trainer.sampler.offset == len(batches):
            validate_epoch_boundary()
            completed_epochs = trainer.sampler.epoch + 1
    if trainer.global_step != target_steps:
        raise RuntimeError("training step count does not match exact epoch contract")
    promoted_checkpoint = output / "best_trajectory.pt"
    launch_gate_evaluated = launch_gate is not None and validation_batches is not None
    launch_gate_passed = (
        not launch_gate_evaluated or promoted_checkpoint.is_file()
    )
    selected_checkpoint = (
        promoted_checkpoint if promoted_checkpoint.is_file() else checkpoint
    )
    runtime_artifact = output / "runtime_artifact.json"
    if launch_gate_passed:
        runtime_artifact.write_text(
            json.dumps(
                {
                    "format": "aic_runtime_artifact_v3",
                    "checkpoint_sha256": _sha256(selected_checkpoint),
                    "contract_hash": contract_hash,
                    "capabilities": [
                        "trajectory", "speed_profile", "current_control", "control_sequence",
                        "behavior", "behavior_side",
                    ],
                    "model_kwargs": full_control_model_kwargs_v3(config),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    eligible_validation_rows = [
        item
        for item in validation_history
        if launch_gate is None or bool(item.get("launch_gate_pass", False))
    ]
    run_manifest = {
        "format": "aic_full_control_training_run_v3", "global_step": trainer.global_step,
        "epochs": epochs,
        "micro_batches_per_epoch": len(batches),
        "gradient_accumulation_steps": gradient_accumulation_steps,
        "checkpoint_every_steps": checkpoint_every_steps,
        "optimizer_steps_per_epoch": optimizer_steps_per_epoch,
        "identity": identity.__dict__,
        "config_sha256": _sha256(Path(args.config)), "device": str(device), "last_log": trainer.logs[-1] if trainer.logs else None,
        "behavior_ontology": "aic_behavior_v1",
        "behavior_view_manifest_sha256": _sha256(Path(args.behavior_view) / "manifest.json"),
        "runtime_artifact_manifest_sha256": (
            _sha256(runtime_artifact) if launch_gate_passed else None
        ),
        "selected_checkpoint": (
            selected_checkpoint.name if launch_gate_passed else None
        ),
        "launch_readiness_gate_passed": (
            launch_gate_passed if launch_gate_evaluated else None
        ),
        "validation_history": (
            None if validation_batches is None else validation_history_path.name
        ),
        "best_validation": (
            None
            if not eligible_validation_rows
            else min(
                eligible_validation_rows,
                key=lambda item: (
                    float(item["trajectory_ade_m"]),
                    float(item["speed_profile_mae_mps"]),
                ),
            )
        ),
        "motion_target_filter": {
            "train_rejected": int(
                getattr(batches, "motion_target_rejected_count", 0)
            ),
            "validation_rejected": (
                None
                if validation_batches is None
                else int(
                    getattr(validation_batches, "motion_target_rejected_count", 0)
                )
            ),
        },
        "initialization": initialization,
    }
    if args.resume and initialization is None:
        previous_manifest_path = output / "run_manifest.json"
        if previous_manifest_path.is_file():
            previous_manifest = json.loads(previous_manifest_path.read_text(encoding="utf-8"))
            run_manifest["initialization"] = previous_manifest.get("initialization")
    (output / "run_manifest.json").write_text(
        json.dumps(run_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "status": "COMPLETE" if launch_gate_passed else "GATE_FAILED",
        "checkpoint": str(selected_checkpoint),
        "runtime_artifact": str(runtime_artifact) if launch_gate_passed else None,
        "global_step": trainer.global_step,
    }))
    return EXIT_SUCCESS if launch_gate_passed else EXIT_GATE


def _input_output(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--only-run", action="append", default=[])


def _selected_inventories(input_root: str, only_runs: Sequence[str]) -> tuple[BagInventoryRecord, ...]:
    records = discover_bag_inventories(input_root)
    selected = tuple(
        record
        for record in records
        if not only_runs or _file_uri_to_path(record.source_path).name in set(only_runs)
    )
    if not selected:
        raise ValueError("no bags matched the requested input and --only-run filters")
    return selected


def _write_json(path: Path, value: Any, *, resume: bool) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    if path.exists():
        if resume and path.read_text(encoding="utf-8") == text:
            return
        raise FileExistsError(f"output already exists: {path}")
    path.write_text(text, encoding="utf-8")


def _write_json_atomic(path: Path, value: Any) -> None:
    """Atomically replace a small generated JSON state file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _atomic_copy(source: Path, destination: Path) -> None:
    """Publish an immutable checkpoint copy without exposing partial bytes."""

    if not source.is_file():
        raise FileNotFoundError(f"checkpoint copy source missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    shutil.copy2(source, temporary)
    os.replace(temporary, destination)


def _file_uri_to_path(uri: str) -> Path:
    parsed = urlparse(uri)
    if parsed.scheme != "file":
        raise ValueError(f"only local file bag URIs are supported: {uri!r}")
    value = unquote(parsed.path)
    if len(value) >= 3 and value[0] == "/" and value[2] == ":":
        value = value[1:]
    return Path(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _combined_sha256(*paths: Path) -> str:
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _emit_error(code: int, error: Exception) -> None:
    print(
        json.dumps(
            {"status": "ERROR", "exit_code": code, "error_type": type(error).__name__, "message": str(error)},
            ensure_ascii=False,
        ),
        file=sys.stderr,
    )


if __name__ == "__main__":
    raise SystemExit(main())
