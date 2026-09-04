import json
import csv
import hashlib
from dataclasses import replace
from pathlib import Path

import pytest
import torch
import yaml

from aic_transfuser_lite.cli import EXIT_GATE, EXIT_SUCCESS, main
from aic_transfuser_lite.data.canonical_converter_v3 import write_prepared_dataset_v3
from aic_transfuser_lite.data.canonical_converter_v3 import PreparedRunV3
from aic_transfuser_lite.data.canonical_schema_v3 import make_sample_id
from aic_transfuser_lite.data.dataset_view_v3 import (
    ControlTargetBoundsV3,
    _ego_row,
    clip_control_target_v3,
    load_temporal_training_batches_v3,
    project_teacher_control_sequence_v3,
    select_control_sequence_row_indices_v3,
)
import aic_transfuser_lite.data.dataset_view_v3 as dataset_view_v3
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset
from aic_transfuser_lite.data.mcap_converter_v2 import TimedCommand
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3, sha256_file_v3
from aic_transfuser_lite.training.train_v3 import (
    TrainerV3,
    balanced_class_weights_v3,
    load_full_control_config_v3,
)
from test_dataset_v3_converter import _convert, _streams


ROOT = Path(__file__).parents[1]


def _target_bounds(**overrides: float) -> ControlTargetBoundsV3:
    values = {
        "max_steering_rad": 0.6,
        "max_steering_rate_radps": 0.8,
        "max_speed_mps": 12.0,
        "min_acceleration_mps2": -4.0,
        "max_acceleration_mps2": 2.0,
        "min_jerk_mps3": -8.0,
        "max_jerk_mps3": 4.0,
        "control_dt_sec": 0.1,
    }
    values.update(overrides)
    return ControlTargetBoundsV3(**values)


def _training_fixture(
    tmp_path: Path,
    *,
    prepared_run: PreparedRunV3 | None = None,
    include_validation: bool = False,
) -> tuple[Path, Path, Path, Path, Path]:
    dataset = tmp_path / "dataset"
    train_run = _convert() if prepared_run is None else prepared_run
    runs = (train_run,)
    if include_validation:
        validation_run_id = "run02"
        validation_run = PreparedRunV3(
            run=replace(
                train_run.run,
                run_id=validation_run_id,
                scenario_id="scenario02",
                source_uri="file:///validation/run02",
            ),
            samples=tuple(
                replace(
                    item,
                    sample=replace(
                        item.sample,
                        sample_id=make_sample_id(
                            validation_run_id,
                            item.sample.segment_id,
                            item.sample.grid_stamp_ns,
                        ),
                        run_id=validation_run_id,
                        scenario_id="scenario02",
                    ),
                )
                for item in train_run.samples
            ),
        )
        runs = (train_run, validation_run)
    write_prepared_dataset_v3(
        dataset, dataset_id="dataset01", topic_profile_id="default",
        runs=runs, jpeg_quality=90,
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
    assignments = [{"run_id": "run01", "split": "train"}]
    if include_validation:
        assignments.append({"run_id": "run02", "split": "validation"})
    split.write_text(json.dumps({
        "dataset_manifest_sha256": manifest["manifest_sha256"],
        "assignments": assignments,
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
    config["training"]["gradient_accumulation_steps"] = 1
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
    assert "control_sequence" in artifact["capabilities"]
    assert artifact["model_kwargs"]["behavior_head_enabled"] is True
    assert artifact["model_kwargs"]["command_history_alignment"] == (
        "causal_previous_only"
    )
    loaded = load_runtime_model_v3(
        output / "last.pt", output / "runtime_artifact.json", device=torch.device("cpu"),
        expected_checkpoint_sha256=artifact["checkpoint_sha256"],
        expected_manifest_sha256=sha256_file_v3(output / "runtime_artifact.json"),
        expected_contract_hash=artifact["contract_hash"],
    )
    assert loaded.model.behavior_head is not None
    run = json.loads((output / "run_manifest.json").read_text())
    assert run["global_step"] == 1
    assert main([
        *args,
        "--resume",
        "--resume-initialization-checkpoint",
        str(output / "last.pt"),
    ]) == EXIT_SUCCESS
    resumed = json.loads((output / "run_manifest.json").read_text())
    assert resumed["global_step"] == 1
    assert resumed["initialization"]["resume_provenance_only"] is True
    initialized_output = tmp_path / "initialized_run"
    initialized_args = [
        *args[: args.index("--output")],
        "--output", str(initialized_output),
        *args[args.index("--output") + 2 :],
        "--init-checkpoint", str(output / "last.pt"),
        "--freeze-migrated",
    ]
    assert main(initialized_args) == EXIT_SUCCESS
    initialized = json.loads((initialized_output / "run_manifest.json").read_text())
    assert initialized["initialization"]["freeze_migrated"] is True
    assert initialized["initialization"]["loaded_key_count"] > 0
    source_state = torch.load(output / "last.pt", map_location="cpu", weights_only=True)["model"]
    trained_state = torch.load(
        initialized_output / "last.pt", map_location="cpu", weights_only=True
    )["model"]
    for name, source_value in source_state.items():
        if not name.startswith("control_sequence_head."):
            torch.testing.assert_close(trained_state[name], source_value, rtol=0.0, atol=0.0)


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


def test_cli_full_control_saves_periodic_resumable_checkpoints(
    tmp_path: Path, monkeypatch,
) -> None:
    dataset, split, config, behavior_view, output = _training_fixture(tmp_path)
    raw = yaml.safe_load(config.read_text(encoding="utf-8"))
    raw["training"]["checkpoint_every_steps"] = 1
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    saved_steps: list[int] = []
    original_save = TrainerV3.save

    def tracked_save(trainer: TrainerV3, path: Path) -> None:
        original_save(trainer, path)
        saved_steps.append(trainer.global_step)

    monkeypatch.setattr(TrainerV3, "save", tracked_save)
    assert main([
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split),
        "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--behavior-view", str(behavior_view), "--output", str(output),
        "--epochs", "1", "--batch-size", "2", "--max-batches", "2",
        "--checkpoint-every-steps", "2",
        "--device", "cpu",
    ]) == EXIT_SUCCESS
    assert saved_steps == [2]
    assert torch.load(output / "last.pt", map_location="cpu", weights_only=True)[
        "global_step"
    ] == 2
    assert json.loads((output / "run_manifest.json").read_text())[
        "checkpoint_every_steps"
    ] == 2


def test_cli_promotes_best_trajectory_checkpoint_from_validation_split(
    tmp_path: Path,
) -> None:
    dataset, split, config, behavior_view, output = _training_fixture(
        tmp_path, include_validation=True
    )
    assert main([
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split),
        "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--behavior-view", str(behavior_view), "--output", str(output),
        "--epochs", "1", "--batch-size", "2", "--device", "cpu",
    ]) == EXIT_SUCCESS
    assert (output / "epoch_001.pt").is_file()
    assert (output / "best_trajectory.pt").is_file()
    history = json.loads((output / "validation_history.json").read_text())
    assert history["selection"] == [
        "trajectory_ade_m", "speed_profile_mae_mps"
    ]
    assert history["epochs"][0]["promoted"] is True
    manifest = json.loads((output / "run_manifest.json").read_text())
    assert manifest["selected_checkpoint"] == "best_trajectory.pt"
    artifact = json.loads((output / "runtime_artifact.json").read_text())
    assert artifact["checkpoint_sha256"] == sha256_file_v3(
        output / "best_trajectory.pt"
    )


def test_cli_launch_gate_blocks_runtime_artifact_promotion(tmp_path: Path) -> None:
    dataset, split, config, behavior_view, output = _training_fixture(
        tmp_path, include_validation=True
    )
    raw = yaml.safe_load(config.read_text(encoding="utf-8"))
    raw["validation"] = {
        "launch_gate": {
            "enabled": True,
            "current_speed_max_mps": 100.0,
            "commanded_speed_min_mps": 101.0,
            "minimum_forward_progress_m": 100.0,
            "minimum_samples": 1,
            "minimum_ready_fraction": 1.0,
        }
    }
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")

    result = main([
        "train", "--config", str(config), "--dataset-root", str(dataset),
        "--split-manifest", str(split),
        "--view-config", str(ROOT / "configs/data/view_temporal_v3.yaml"),
        "--behavior-view", str(behavior_view), "--output", str(output),
        "--epochs", "1", "--batch-size", "2", "--device", "cpu",
    ])

    assert result == EXIT_GATE
    assert (output / "last.pt").is_file()
    assert not (output / "runtime_artifact.json").exists()
    manifest = json.loads((output / "run_manifest.json").read_text())
    assert manifest["launch_readiness_gate_passed"] is False
    assert manifest["selected_checkpoint"] is None
    assert manifest["best_validation"] is None


def test_full_control_config_rejects_nonpositive_checkpoint_interval(tmp_path: Path) -> None:
    raw = yaml.safe_load((ROOT / "configs/models/full_control_lite_v3.yaml").read_text())
    raw["training"]["checkpoint_every_steps"] = 0
    config = tmp_path / "invalid_checkpoint_interval.yaml"
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="positive checkpoint_every_steps"):
        load_full_control_config_v3(config)


def test_full_control_config_rejects_invalid_plan_loss_and_accumulation(
    tmp_path: Path,
) -> None:
    raw = yaml.safe_load((ROOT / "configs/models/full_control_lite_v3.yaml").read_text())
    raw["loss"]["plan_step_sec"] = 0.0
    config = tmp_path / "invalid_plan_step.yaml"
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="plan_step_sec"):
        load_full_control_config_v3(config)

    raw["loss"]["plan_step_sec"] = 0.1
    raw["training"]["gradient_accumulation_steps"] = 0
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="gradient_accumulation_steps"):
        load_full_control_config_v3(config)


def test_temporal_training_loader_defers_asset_reads_until_batch_access(
    tmp_path: Path, monkeypatch,
) -> None:
    dataset, split, _, behavior_view, _ = _training_fixture(tmp_path)
    real_open = dataset_view_v3.Image.open
    opened: list[Path] = []

    def tracked_open(path, *args, **kwargs):
        opened.append(Path(path))
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr(dataset_view_v3.Image, "open", tracked_open)
    batches = load_temporal_training_batches_v3(
        dataset,
        split,
        split="train",
        image_height=32,
        image_width=32,
        lidar_points=4,
        lidar_min_range_m=0.0,
        lidar_max_range_m=25.0,
        ego_features=("longitudinal_speed_mps", "lateral_speed_mps", "yaw_rate_rps"),
        trajectory_steps=15,
        control_sequence_steps=10,
        camera_history_length=4,
        ego_history_length=10,
        command_history_length=10,
        control_target_bounds=_target_bounds(),
        batch_size=2,
        behavior_view_root=behavior_view,
    )
    assert len(batches) > 1
    assert opened == []
    weights = balanced_class_weights_v3(
        batches,
        target_name="behavior_class",
        mask_name="behavior_mask",
        class_count=5,
        require_all_classes=False,
    )
    assert len(weights) == 5
    assert opened == []
    assert all(
        int(batches.rows[anchor]["future_valid_count"]) > 0
        for anchor in batches.usable_anchors
    )
    first = batches[0]
    assert first.image.shape == (2, 4, 3, 32, 32)
    assert len(opened) == 8


def test_training_command_history_is_past_only_and_projection_uses_previous_state(
    tmp_path: Path,
) -> None:
    streams = _streams()
    commands = tuple(
        TimedCommand(
            item.timestamp_ns,
            1.0,
            -1.0 if index == 0 else 2.0,
            0.0,
        )
        for index, item in enumerate(streams.nominal_commands)
    )
    prepared = _convert(
        streams.__class__(
            **{
                **streams.__dict__,
                "nominal_commands": commands,
                "final_commands": commands,
            }
        )
    )
    dataset, split, _, behavior_view, _ = _training_fixture(
        tmp_path, prepared_run=prepared
    )
    batch = load_temporal_training_batches_v3(
        dataset,
        split,
        split="train",
        image_height=32,
        image_width=32,
        lidar_points=4,
        lidar_min_range_m=0.0,
        lidar_max_range_m=25.0,
        ego_features=(
            "longitudinal_speed_mps",
            "lateral_speed_mps",
            "yaw_rate_rps",
        ),
        trajectory_steps=15,
        control_sequence_steps=10,
        camera_history_length=4,
        ego_history_length=10,
        command_history_length=10,
        control_target_bounds=_target_bounds(),
        batch_size=2,
        max_batches=1,
        behavior_view_root=behavior_view,
    )[0]

    assert not bool(batch.command_mask[0].any())
    assert batch.command_mask[1].tolist() == [False] * 9 + [True]
    assert batch.command_history[1, -1, 2].item() == pytest.approx(-1.0)
    assert batch.targets is not None
    assert batch.targets.current_control[1, 2].item() == pytest.approx(2.0)
    assert batch.targets.control_sequence[1, 0, 2].item() == pytest.approx(-0.6)
    assert batch.targets.control_sequence_time_sec[1].tolist() == pytest.approx(
        [index * 0.1 for index in range(10)]
    )
    assert len(batch.targets.control_sequence_provenance[1]) == 10


def test_control_sequence_selection_uses_exact_grid_timestamps_without_compression() -> None:
    rows = [
        {"run_id": "run-a", "segment_id": "segment-0", "grid_stamp_ns": stamp}
        for stamp in (0, 100_000_000, 300_000_000)
    ]
    index = {
        (row["run_id"], row["segment_id"], int(row["grid_stamp_ns"])): offset
        for offset, row in enumerate(rows)
    }

    selected = select_control_sequence_row_indices_v3(
        rows,
        index,
        anchor_index=0,
        steps=4,
        control_dt_sec=0.1,
    )

    assert selected == (0, 1, None, 2)


def test_full_control_config_rejects_noncausal_command_history(tmp_path: Path) -> None:
    raw = yaml.safe_load((ROOT / "configs/models/full_control_lite_v3.yaml").read_text())
    raw["targets"]["command_history_alignment"] = "includes_current_target"
    config = tmp_path / "leaking_history.yaml"
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="causal command history"):
        load_full_control_config_v3(config)


def test_full_control_config_rejects_row_order_control_sequence_alignment(
    tmp_path: Path,
) -> None:
    raw = yaml.safe_load((ROOT / "configs/models/full_control_lite_v3.yaml").read_text())
    raw["targets"]["control_sequence_alignment"] = "next_row"
    config = tmp_path / "compressed_sequence.yaml"
    config.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="exact-grid control sequence"):
        load_full_control_config_v3(config)


def test_teacher_control_projection_matches_head_absolute_rate_and_jerk_limits() -> None:
    commands = torch.tensor(
        [[1.0, 20.0, 10.0], [-1.0, -2.0, -10.0], [0.2, 3.0, 0.0]]
    )
    mask = torch.tensor(
        [[True, True, True], [True, True, True], [False, False, False]]
    )

    projected = project_teacher_control_sequence_v3(
        commands,
        mask,
        initial_steering_rad=0.0,
        initial_acceleration_mps2=2.0,
        bounds=_target_bounds(),
    )

    torch.testing.assert_close(
        projected,
        torch.tensor([[0.08, 12.0, 2.0], [0.0, 0.0, 1.2], [0.0, 0.0, 0.0]]),
    )
    steering_steps = torch.diff(
        torch.cat((torch.tensor([0.0]), projected[:2, 0]))
    )
    assert torch.max(torch.abs(steering_steps)) <= 0.08
    acceleration_steps = torch.diff(
        torch.cat((torch.tensor([2.0]), projected[:2, 2]))
    )
    assert torch.min(acceleration_steps) >= -0.8
    assert torch.max(acceleration_steps) <= 0.4


def test_current_teacher_control_is_clipped_to_head_absolute_limits() -> None:
    clipped = clip_control_target_v3(
        torch.tensor([1.0, -2.0, 10.0]), bounds=_target_bounds()
    )
    torch.testing.assert_close(clipped, torch.tensor([0.6, 0.0, 2.0]))


def test_teacher_control_projection_rejects_invalid_bounds_shape_and_mask() -> None:
    commands = torch.zeros(2, 3)
    mask = torch.ones(2, 3, dtype=torch.bool)
    with pytest.raises(ValueError, match="speed and time step"):
        project_teacher_control_sequence_v3(
            commands,
            mask,
            initial_steering_rad=0.0,
            initial_acceleration_mps2=0.0,
            bounds=_target_bounds(max_speed_mps=0.0),
        )
    with pytest.raises(ValueError, match=r"\[H,3\]"):
        project_teacher_control_sequence_v3(
            torch.zeros(2, 2),
            mask,
            initial_steering_rad=0.0,
            initial_acceleration_mps2=0.0,
            bounds=_target_bounds(),
        )
    with pytest.raises(ValueError, match=r"\[3\]"):
        clip_control_target_v3(torch.zeros(1, 3), bounds=_target_bounds())
    with pytest.raises(ValueError, match="mask must be boolean"):
        project_teacher_control_sequence_v3(
            commands,
            torch.ones(2, 3),
            initial_steering_rad=0.0,
            initial_acceleration_mps2=0.0,
            bounds=_target_bounds(),
        )
    partial_mask = mask.clone()
    partial_mask[0, 0] = False
    with pytest.raises(ValueError, match="all-valid or all-invalid"):
        project_teacher_control_sequence_v3(
            commands,
            partial_mask,
            initial_steering_rad=0.0,
            initial_acceleration_mps2=0.0,
            bounds=_target_bounds(),
        )


def test_ego_abs_limit_masks_implausible_yaw_rate() -> None:
    row = {
        "velocity_longitudinal_mps": "0.4",
        "velocity_lateral_mps": "0.01",
        "yaw_rate_rps": "1077.0",
        "actual_steering_rad": "0.1",
        "actual_steering_valid": "true",
    }
    values, mask = _ego_row(
        row,
        (
            "longitudinal_speed_mps",
            "lateral_speed_mps",
            "yaw_rate_rps",
            "actual_steering_rad",
        ),
        abs_limits={
            "longitudinal_speed_mps": 20.0,
            "lateral_speed_mps": 10.0,
            "yaw_rate_rps": 5.0,
            "actual_steering_rad": 0.7,
        },
    )
    assert values.tolist() == pytest.approx([0.4, 0.01, 0.0, 0.1])
    assert mask.tolist() == [True, True, False, True]
