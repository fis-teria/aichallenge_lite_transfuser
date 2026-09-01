import json
from pathlib import Path

import yaml

from aic_transfuser_lite.cli import EXIT_SUCCESS, main
from aic_transfuser_lite.data.canonical_converter_v3 import write_prepared_dataset_v3
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset
from test_dataset_v3_converter import _convert


ROOT = Path(__file__).parents[1]


def _training_fixture(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    dataset = tmp_path / "dataset"
    write_prepared_dataset_v3(
        dataset, dataset_id="dataset01", topic_profile_id="default",
        runs=(_convert(),), jpeg_quality=90,
    )
    manifest = validate_complete_dataset(dataset)
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
    return dataset, split, config_path, output


def test_cli_full_control_one_epoch_and_resume(tmp_path: Path) -> None:
    dataset, split, config, output = _training_fixture(tmp_path)
    args = [
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split), "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--output", str(output), "--epochs", "1", "--batch-size", "2",
        "--max-batches", "1", "--device", "cpu",
    ]
    assert main(args) == EXIT_SUCCESS
    assert (output / "last.pt").is_file()
    run = json.loads((output / "run_manifest.json").read_text())
    assert run["global_step"] == 1
    assert main([*args, "--resume"]) == EXIT_SUCCESS
    resumed = json.loads((output / "run_manifest.json").read_text())
    assert resumed["global_step"] == 1


def test_cli_full_control_dry_run_writes_nothing(tmp_path: Path) -> None:
    dataset, split, config, output = _training_fixture(tmp_path)
    assert main([
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split), "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--output", str(output), "--epochs", "1", "--batch-size", "2",
        "--max-batches", "1", "--device", "cpu", "--dry-run",
    ]) == EXIT_SUCCESS
    assert not output.exists()
