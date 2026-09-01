from __future__ import annotations

import copy
from pathlib import Path

import pytest

from aic_transfuser_lite.config import (
    ConfigValidationError,
    load_config,
    load_v1_config,
    validate_v1_config,
)


ROOT = Path(__file__).resolve().parents[1]
V1_CONFIG = ROOT / "configs/diagnostics/v1_training_stack_smoke.yaml"


def valid_config() -> dict:
    return load_v1_config(V1_CONFIG)


def test_legacy_loader_still_accepts_protected_v0_config() -> None:
    config = load_config(ROOT / "configs/transfuser_lite_v0.yaml")
    assert config["project"]["version"] == "v0.1"


def test_v1_loader_rejects_legacy_config_explicitly() -> None:
    with pytest.raises(ConfigValidationError, match="schema_version"):
        load_v1_config(ROOT / "configs/transfuser_lite_v0.yaml")


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (lambda value: value.update({"mystery": 1}), "Unknown root keys"),
        (
            lambda value: value["training"].update({"learning_rate": 3e-4}),
            "Unknown training keys",
        ),
        (
            lambda value: value["training"].update({"optimizer": "sgd"}),
            "training.optimizer",
        ),
        (
            lambda value: value["model"]["lidar"].update({"use_valid_mask": True}),
            "not implemented",
        ),
        (
            lambda value: value["training"].update({"epochs": 51}),
            "must be <= 50",
        ),
    ],
)
def test_v1_schema_rejects_unknown_or_noop_contracts(mutator, message: str) -> None:
    config = copy.deepcopy(valid_config())
    mutator(config)
    with pytest.raises(ConfigValidationError, match=message):
        validate_v1_config(config)


def test_disabled_optional_head_requires_zero_loss() -> None:
    config = copy.deepcopy(valid_config())
    config["loss_weights"]["stop"] = 0.1
    with pytest.raises(ConfigValidationError, match="must be zero"):
        validate_v1_config(config)


def test_ego_and_horizon_shapes_are_strict() -> None:
    config = copy.deepcopy(valid_config())
    config["data"]["ego_dim"] = 2
    with pytest.raises(ConfigValidationError, match="ego_dim"):
        validate_v1_config(config)

    config = copy.deepcopy(valid_config())
    config["training"]["waypoint_horizon_weights"] = [1.0]
    with pytest.raises(ConfigValidationError, match="horizon_weights"):
        validate_v1_config(config)


def test_zero_worker_contract_has_no_silent_prefetch_noop() -> None:
    config = copy.deepcopy(valid_config())
    config["training"]["num_workers"] = 0
    config["training"]["persistent_workers"] = False
    with pytest.raises(ConfigValidationError, match="prefetch_factor must be null"):
        validate_v1_config(config)
    config["training"]["prefetch_factor"] = None
    validate_v1_config(config)


@pytest.mark.parametrize("value", [-0.01, 1.01, True, "0.1"])
def test_v2_full_lidar_dropout_probability_is_strict(value: object) -> None:
    config = copy.deepcopy(
        load_v1_config(ROOT / "configs/transfuser_lite_v1_static.yaml")
    )
    config["data"]["augmentation"]["lidar"][
        "full_dropout_probability"
    ] = value
    with pytest.raises(
        ConfigValidationError,
        match="data.augmentation.lidar.full_dropout_probability",
    ):
        validate_v1_config(config)


def test_v2_full_lidar_dropout_is_optional_for_old_embedded_configs() -> None:
    config = load_v1_config(ROOT / "configs/transfuser_lite_v1_static.yaml")
    assert "full_dropout_probability" not in config["data"]["augmentation"]["lidar"]
    validate_v1_config(config)


def test_v2_full_lidar_dropout_candidate_config_is_valid() -> None:
    config = load_v1_config(
        ROOT / "configs/diagnostics/v1_static_lidar_dropout_p010_seed42.yaml"
    )
    assert config["data"]["augmentation"]["lidar"]["full_dropout_probability"] == 0.1


@pytest.mark.parametrize("value", [-0.01, 1.01, True, "0.1"])
def test_v2_full_camera_dropout_probability_is_strict(value: object) -> None:
    config = copy.deepcopy(
        load_v1_config(
            ROOT / "configs/diagnostics/v1_static_lidar_dropout_p010_seed42.yaml"
        )
    )
    config["data"]["augmentation"]["camera"][
        "full_dropout_probability"
    ] = value
    with pytest.raises(
        ConfigValidationError,
        match="data.augmentation.camera.full_dropout_probability",
    ):
        validate_v1_config(config)


def test_v2_balanced_dropout_candidate_config_is_valid() -> None:
    config = load_v1_config(
        ROOT / "configs/diagnostics/v1_static_balanced_dropout_p010_seed42.yaml"
    )
    augmentation = config["data"]["augmentation"]
    assert augmentation["camera"]["full_dropout_probability"] == 0.1
    assert augmentation["lidar"]["full_dropout_probability"] == 0.1


def test_v2_full_dropout_policy_is_optional_and_defaults_to_independent() -> None:
    config = load_v1_config(
        ROOT / "configs/diagnostics/v1_static_balanced_dropout_p020_seed42.yaml"
    )
    assert "full_dropout_policy" not in config["data"]["augmentation"]
    validate_v1_config(config)


@pytest.mark.parametrize("value", ["exclusive", "", 1, True, None])
def test_v2_full_dropout_policy_is_strict(value: object) -> None:
    config = copy.deepcopy(
        load_v1_config(
            ROOT / "configs/diagnostics/v1_static_balanced_dropout_p020_seed42.yaml"
        )
    )
    config["data"]["augmentation"]["full_dropout_policy"] = value
    with pytest.raises(
        ConfigValidationError,
        match="data.augmentation.full_dropout_policy",
    ):
        validate_v1_config(config)


def test_v2_mutually_exclusive_dropout_rejects_probability_sum_above_one() -> None:
    config = copy.deepcopy(
        load_v1_config(
            ROOT
            / "configs/diagnostics/v1_static_exclusive_dropout_p020_seed42.yaml"
        )
    )
    config["data"]["augmentation"]["camera"]["full_dropout_probability"] = 0.6
    config["data"]["augmentation"]["lidar"]["full_dropout_probability"] = 0.5
    with pytest.raises(ConfigValidationError, match="must sum to <= 1"):
        validate_v1_config(config)


def test_v2_independent_dropout_preserves_overlapping_probability_contract() -> None:
    config = copy.deepcopy(
        load_v1_config(
            ROOT / "configs/diagnostics/v1_static_balanced_dropout_p020_seed42.yaml"
        )
    )
    config["data"]["augmentation"]["camera"]["full_dropout_probability"] = 1.0
    config["data"]["augmentation"]["lidar"]["full_dropout_probability"] = 1.0
    validate_v1_config(config)


def test_v2_exclusive_dropout_candidate_config_is_valid() -> None:
    config = load_v1_config(
        ROOT / "configs/diagnostics/v1_static_exclusive_dropout_p020_seed42.yaml"
    )
    augmentation = config["data"]["augmentation"]
    assert augmentation["full_dropout_policy"] == "mutually_exclusive"
    assert augmentation["camera"]["full_dropout_probability"] == 0.2
    assert augmentation["lidar"]["full_dropout_probability"] == 0.2
