from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
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
from .data.dataset_view_v3 import load_v1_compatibility_view_config
from .data.dataset_view_v3 import load_temporal_training_batches_v3
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
    full_control_model_kwargs_v3, load_full_control_config_v3, move_batch_v3,
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
    config = load_full_control_config_v3(args.config)
    model_cfg, data_cfg, loss_cfg, training_cfg = (
        config["model"], config["data"], config["loss"], config["training"]
    )
    view = yaml.safe_load(Path(args.view_config).read_text(encoding="utf-8"))
    batch_size = int(args.batch_size or training_cfg["micro_batch_size"])
    epochs = int(args.epochs or training_cfg["epochs"])
    if epochs <= 0 or batch_size <= 0:
        raise ValueError("epochs and batch size must be positive")
    batches = load_temporal_training_batches_v3(
        args.dataset_root, args.split_manifest, split="train",
        image_height=int(data_cfg["image_height"]), image_width=int(data_cfg["image_width"]),
        lidar_points=int(data_cfg["lidar_points"]),
        lidar_min_range_m=float(data_cfg["lidar_min_range_m"]),
        lidar_max_range_m=float(data_cfg["lidar_max_range_m"]),
        ego_features=tuple(data_cfg["ego_features"]), trajectory_steps=int(model_cfg["trajectory_steps"]),
        camera_history_length=int(view["camera_history_length"]),
        ego_history_length=int(view["ego_history_length"]), batch_size=batch_size,
        max_batches=args.max_batches, behavior_view_root=args.behavior_view,
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
        (Path("schemas/model_batch_v3.schema.json").read_bytes()
         + Path("schemas/model_output_v3.schema.json").read_bytes())
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
            "behavior_class_weights": behavior_weights,
            "behavior_side_class_weights": side_weights,
        }))
        return EXIT_SUCCESS
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    model = build_full_control_model_v3(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    trainer = TrainerV3(
        model=model, batches=[move_batch_v3(batch, device) for batch in batches], optimizer=optimizer,
        identity=identity,
        loss_weights=LossWeightsV3(
            float(loss_cfg["trajectory"]), float(loss_cfg["speed_profile"]),
            float(loss_cfg["current_control"]), float(loss_cfg["behavior"]),
            float(loss_cfg["behavior_side"]), behavior_weights, side_weights,
        ),
    )
    checkpoint = output / "last.pt"
    if args.resume:
        if not checkpoint.is_file():
            raise FileNotFoundError(f"resume checkpoint missing: {checkpoint}")
        trainer.resume(checkpoint)
    target_steps = epochs * len(batches)
    trainer.train_steps(max(0, target_steps - trainer.global_step))
    trainer.save(checkpoint)
    runtime_artifact = output / "runtime_artifact.json"
    runtime_artifact.write_text(
        json.dumps(
            {
                "format": "aic_runtime_artifact_v3",
                "checkpoint_sha256": _sha256(checkpoint),
                "contract_hash": contract_hash,
                "capabilities": [
                    "trajectory", "speed_profile", "current_control", "behavior", "behavior_side",
                ],
                "model_kwargs": full_control_model_kwargs_v3(config),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    run_manifest = {
        "format": "aic_full_control_training_run_v3", "global_step": trainer.global_step,
        "epochs": epochs, "batches_per_epoch": len(batches), "identity": identity.__dict__,
        "config_sha256": _sha256(Path(args.config)), "device": str(device), "last_log": trainer.logs[-1] if trainer.logs else None,
        "behavior_ontology": "aic_behavior_v1",
        "behavior_view_manifest_sha256": _sha256(Path(args.behavior_view) / "manifest.json"),
        "runtime_artifact_manifest_sha256": _sha256(runtime_artifact),
    }
    (output / "run_manifest.json").write_text(
        json.dumps(run_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "status": "COMPLETE", "checkpoint": str(checkpoint),
        "runtime_artifact": str(runtime_artifact), "global_step": trainer.global_step,
    }))
    return EXIT_SUCCESS


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
