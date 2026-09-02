import json
import csv
import hashlib
from pathlib import Path

import yaml
import torch

from aic_transfuser_lite.cli import EXIT_SUCCESS, main
from aic_transfuser_lite.data.canonical_converter_v3 import write_prepared_dataset_v3
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3, sha256_file_v3
from test_dataset_v3_converter import _convert


ROOT = Path(__file__).parents[1]


def _training_fixture(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    dataset = tmp_path / "dataset"
    write_prepared_dataset_v3(
        dataset, dataset_id="dataset01", topic_profile_id="default",
        runs=(_convert(),), jpeg_quality=90,
    )
    manifest = validate_complete_dataset(dataset)
    behavior_view = tmp_path / "behavior_view"
    behavior_view.mkdir()
    with (dataset / "samples.csv").open(newline="", encoding="utf-8") as stream:
        sample_rows = list(csv.DictReader(stream))
    fields = [
        "sample_id", "run_id", "grid_stamp_ns", "behavior_class", "behavior_label",
        "behavior_side", "behavior_side_label", "behavior_valid", "behavior_side_valid",
        "quality", "source_stamp_ns", "source_age_ms", "source", "authority", "target_vehicle",
        "invalid_reason",
    ]
    labels = behavior_view / "behavior_labels.csv"
    class_names = ["FORWARD_NORMAL", "FORWARD_FOLLOW", "FORWARD_AVOID", "FORWARD_RETURN", "RECOVERY"]
    side_names = ["NONE", "LEFT", "RIGHT"]
    with labels.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(sample_rows):
            behavior, side = index % 5, index % 3
            writer.writerow({
                "sample_id": row["sample_id"], "run_id": row["run_id"],
                "grid_stamp_ns": row["grid_stamp_ns"], "behavior_class": behavior,
                "behavior_label": class_names[behavior], "behavior_side": side,
                "behavior_side_label": side_names[side], "behavior_valid": True,
                "behavior_side_valid": True, "quality": 1.0,
                "source_stamp_ns": row["grid_stamp_ns"], "source_age_ms": 0.0,
                "source": "mpc_expert_autoware_log", "authority": "MPC",
                "target_vehicle": "", "invalid_reason": "",
            })
    labels_sha = hashlib.sha256(labels.read_bytes()).hexdigest()
    (behavior_view / "manifest.json").write_text(json.dumps({
        "format": "aic_behavior_view_v1", "ontology": "aic_behavior_v1",
        "class_names": class_names, "side_names": side_names,
        "dataset_manifest_sha256": manifest["manifest_sha256"], "labels_sha256": labels_sha,
        "sample_count": len(sample_rows), "valid_behavior_count": len(sample_rows),
        "valid_side_count": len(sample_rows),
    }), encoding="utf-8")
    split = tmp_path / "split.json"
    split.write_text(json.dumps({
        "dataset_manifest_sha256": manifest["manifest_sha256"],
        "assignments": [{"run_id": "run01", "split": "train"}],
    }), encoding="utf-8")
    config = yaml.safe_load((ROOT / "configs/models/full_control_lite_v3.yaml").read_text())
    config["model"].update({
        "hidden_dim": 16, "camera_tokens_hw": [1, 1], "lidar_tokens": 2,
        "fusion_depth": 1, "fusion_heads": 4,
    })
    config["data"].update({
        "image_height": 32, "image_width": 32, "lidar_points": 4,
        "ego_features": ["longitudinal_speed_mps", "lateral_speed_mps", "yaw_rate_rps"],
    })
    config_path = tmp_path / "full_control.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    output = tmp_path / "run"
    return dataset, split, config_path, behavior_view, output


def test_cli_full_control_one_epoch_and_resume(tmp_path: Path) -> None:
    dataset, split, config, behavior_view, output = _training_fixture(tmp_path)
    args = [
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split), "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--behavior-view", str(behavior_view),
        "--output", str(output), "--epochs", "1", "--batch-size", "2",
        "--max-batches", "1", "--device", "cpu",
    ]
    assert main(args) == EXIT_SUCCESS
    assert (output / "last.pt").is_file()
    artifact = json.loads((output / "runtime_artifact.json").read_text())
    assert artifact["capabilities"][-2:] == ["behavior", "behavior_side"]
    assert artifact["model_kwargs"]["behavior_head_enabled"] is True
    loaded = load_runtime_model_v3(
        output / "last.pt", output / "runtime_artifact.json", device=torch.device("cpu"),
        expected_checkpoint_sha256=artifact["checkpoint_sha256"],
        expected_manifest_sha256=sha256_file_v3(output / "runtime_artifact.json"),
        expected_contract_hash=artifact["contract_hash"],
    )
    assert loaded.model.behavior_head is not None
    run = json.loads((output / "run_manifest.json").read_text())
    assert run["global_step"] == 1
    assert main([*args, "--resume"]) == EXIT_SUCCESS
    resumed = json.loads((output / "run_manifest.json").read_text())
    assert resumed["global_step"] == 1


def test_cli_full_control_dry_run_writes_nothing(tmp_path: Path) -> None:
    dataset, split, config, behavior_view, output = _training_fixture(tmp_path)
    assert main([
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split), "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--behavior-view", str(behavior_view),
        "--output", str(output), "--epochs", "1", "--batch-size", "2",
        "--max-batches", "1", "--device", "cpu", "--dry-run",
    ]) == EXIT_SUCCESS
    assert not output.exists()
