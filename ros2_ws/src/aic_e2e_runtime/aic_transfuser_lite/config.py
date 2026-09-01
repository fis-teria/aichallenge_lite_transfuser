from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


V1_SCHEMA_VERSION = "transfuser_lite_v1"

V1_DATA_REQUIRED_KEYS = {
    "sample_rate_hz",
    "sync_tolerance_ms",
    "image_height",
    "image_width",
    "lidar_points",
    "lidar_min_range_m",
    "lidar_max_range_m",
    "ego_dim",
    "ego_features",
    "prediction_horizon_sec",
    "num_waypoints",
    "mode_classes",
}

V2_DATA_EXTRA_KEYS = {
    "format_version",
    "lidar_angle_min_rad",
    "lidar_angle_increment_rad",
    "ego_speed_scale_mps",
    "augmentation",
}


class ConfigValidationError(ValueError):
    """Raised when a versioned v1 config violates the executable contract."""


def load_config(path: str | Path) -> dict[str, Any]:
    """Load a legacy-compatible YAML config and return a plain dictionary."""
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Config not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping: {config_path}")
    return data


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigValidationError(f"{path} must be a mapping")
    return value


def _keys(
    value: dict[str, Any],
    path: str,
    *,
    required: set[str],
    optional: set[str] | None = None,
) -> None:
    optional = optional or set()
    unknown = sorted(set(value) - required - optional)
    missing = sorted(required - set(value))
    if unknown:
        raise ConfigValidationError(f"Unknown {path} keys: {unknown}")
    if missing:
        raise ConfigValidationError(f"Missing {path} keys: {missing}")


def _bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigValidationError(f"{path} must be a boolean")
    return value


def _integer(value: Any, path: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigValidationError(f"{path} must be an integer")
    if minimum is not None and value < minimum:
        raise ConfigValidationError(f"{path} must be >= {minimum}")
    return value


def _number(
    value: Any,
    path: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    strict_minimum: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigValidationError(f"{path} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigValidationError(f"{path} must be finite")
    if minimum is not None:
        invalid = result <= minimum if strict_minimum else result < minimum
        if invalid:
            relation = ">" if strict_minimum else ">="
            raise ConfigValidationError(f"{path} must be {relation} {minimum}")
    if maximum is not None and result > maximum:
        raise ConfigValidationError(f"{path} must be <= {maximum}")
    return result


def _enum(value: Any, path: str, allowed: set[str]) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise ConfigValidationError(f"{path} must be one of {sorted(allowed)}")
    return value


def validate_v2_data_config(data: dict[str, Any]) -> None:
    """Validate the standalone Dataset v2 preprocessing contract.

    Model support is intentionally validated separately by ``validate_v1_config``
    so the dataset loader can be tested before the two-channel model is installed.
    """
    data = _mapping(data, "data")
    _keys(
        data,
        "data",
        required=V1_DATA_REQUIRED_KEYS | V2_DATA_EXTRA_KEYS,
    )
    if _integer(data["format_version"], "data.format_version", minimum=1) != 2:
        raise ConfigValidationError("data.format_version must be 2 for Dataset v2")
    _number(
        data["sample_rate_hz"],
        "data.sample_rate_hz",
        minimum=0.0,
        strict_minimum=True,
    )
    _number(data["sync_tolerance_ms"], "data.sync_tolerance_ms", minimum=0.0)
    _integer(data["image_height"], "data.image_height", minimum=1)
    _integer(data["image_width"], "data.image_width", minimum=1)
    _integer(data["lidar_points"], "data.lidar_points", minimum=2)
    _integer(data["num_waypoints"], "data.num_waypoints", minimum=1)
    lidar_min = _number(
        data["lidar_min_range_m"], "data.lidar_min_range_m", minimum=0.0
    )
    lidar_max = _number(
        data["lidar_max_range_m"],
        "data.lidar_max_range_m",
        minimum=0.0,
        strict_minimum=True,
    )
    if lidar_max <= lidar_min:
        raise ConfigValidationError(
            "data.lidar_max_range_m must be greater than data.lidar_min_range_m"
        )
    _number(data["lidar_angle_min_rad"], "data.lidar_angle_min_rad")
    _number(
        data["lidar_angle_increment_rad"],
        "data.lidar_angle_increment_rad",
        minimum=0.0,
        strict_minimum=True,
    )
    _number(
        data["ego_speed_scale_mps"],
        "data.ego_speed_scale_mps",
        minimum=0.0,
        strict_minimum=True,
    )
    if _integer(data["ego_dim"], "data.ego_dim", minimum=1) != 1:
        raise ConfigValidationError("Dataset v2 static requires data.ego_dim=1")
    if data["ego_features"] != ["speed_mps"]:
        raise ConfigValidationError(
            "Dataset v2 static requires data.ego_features=['speed_mps']"
        )

    augmentation = _mapping(data["augmentation"], "data.augmentation")
    _keys(
        augmentation,
        "data.augmentation",
        required={"enabled", "camera", "lidar"},
        optional={"full_dropout_policy"},
    )
    _bool(augmentation["enabled"], "data.augmentation.enabled")
    full_dropout_policy = _enum(
        augmentation.get("full_dropout_policy", "independent"),
        "data.augmentation.full_dropout_policy",
        {"independent", "mutually_exclusive"},
    )

    camera = _mapping(augmentation["camera"], "data.augmentation.camera")
    _keys(
        camera,
        "data.augmentation.camera",
        required={
            "brightness_delta",
            "contrast_delta",
            "gamma_min",
            "gamma_max",
            "blur_probability",
            "blur_radius_max_px",
            "noise_probability",
            "noise_std_fraction",
        },
        optional={"full_dropout_probability"},
    )
    _number(
        camera["brightness_delta"],
        "data.augmentation.camera.brightness_delta",
        minimum=0.0,
        maximum=1.0,
    )
    _number(
        camera["contrast_delta"],
        "data.augmentation.camera.contrast_delta",
        minimum=0.0,
        maximum=1.0,
    )
    gamma_min = _number(
        camera["gamma_min"],
        "data.augmentation.camera.gamma_min",
        minimum=0.0,
        strict_minimum=True,
    )
    gamma_max = _number(
        camera["gamma_max"],
        "data.augmentation.camera.gamma_max",
        minimum=0.0,
        strict_minimum=True,
    )
    if gamma_max < gamma_min:
        raise ConfigValidationError(
            "data.augmentation.camera.gamma_max must be >= gamma_min"
        )
    for name in ("blur_probability", "noise_probability"):
        _number(
            camera[name],
            f"data.augmentation.camera.{name}",
            minimum=0.0,
            maximum=1.0,
        )
    _number(
        camera["blur_radius_max_px"],
        "data.augmentation.camera.blur_radius_max_px",
        minimum=0.0,
    )
    _number(
        camera["noise_std_fraction"],
        "data.augmentation.camera.noise_std_fraction",
        minimum=0.0,
        maximum=1.0,
    )
    camera_full_dropout_probability = _number(
        camera.get("full_dropout_probability", 0.0),
        "data.augmentation.camera.full_dropout_probability",
        minimum=0.0,
        maximum=1.0,
    )

    lidar = _mapping(augmentation["lidar"], "data.augmentation.lidar")
    _keys(
        lidar,
        "data.augmentation.lidar",
        required={
            "range_noise_sigma_min_m",
            "range_noise_sigma_max_m",
            "beam_dropout_max_fraction",
            "sector_dropout_probability",
            "sector_dropout_max_degrees",
        },
        optional={"full_dropout_probability"},
    )
    sigma_min = _number(
        lidar["range_noise_sigma_min_m"],
        "data.augmentation.lidar.range_noise_sigma_min_m",
        minimum=0.0,
    )
    sigma_max = _number(
        lidar["range_noise_sigma_max_m"],
        "data.augmentation.lidar.range_noise_sigma_max_m",
        minimum=0.0,
    )
    if sigma_max < sigma_min:
        raise ConfigValidationError(
            "data.augmentation.lidar.range_noise_sigma_max_m must be >= sigma_min"
        )
    _number(
        lidar["beam_dropout_max_fraction"],
        "data.augmentation.lidar.beam_dropout_max_fraction",
        minimum=0.0,
        maximum=1.0,
    )
    _number(
        lidar["sector_dropout_probability"],
        "data.augmentation.lidar.sector_dropout_probability",
        minimum=0.0,
        maximum=1.0,
    )
    _number(
        lidar["sector_dropout_max_degrees"],
        "data.augmentation.lidar.sector_dropout_max_degrees",
        minimum=0.0,
        maximum=360.0,
    )
    lidar_full_dropout_probability = _number(
        lidar.get("full_dropout_probability", 0.0),
        "data.augmentation.lidar.full_dropout_probability",
        minimum=0.0,
        maximum=1.0,
    )
    if (
        full_dropout_policy == "mutually_exclusive"
        and camera_full_dropout_probability + lidar_full_dropout_probability > 1.0
    ):
        raise ConfigValidationError(
            "mutually-exclusive Camera/LiDAR full-dropout probabilities must sum to <= 1"
        )


def validate_v1_config(config: dict[str, Any]) -> None:
    """Validate every accepted v1 key against behavior implemented by train_v1."""
    if config.get("schema_version") != V1_SCHEMA_VERSION:
        raise ConfigValidationError(
            f"schema_version must be {V1_SCHEMA_VERSION!r}; legacy configs must use load_config"
        )
    _keys(
        config,
        "root",
        required={
            "schema_version",
            "project",
            "data",
            "model",
            "loss_weights",
            "training",
        },
    )
    project = _mapping(config["project"], "project")
    _keys(project, "project", required={"name", "version", "seed"})
    if not isinstance(project["name"], str) or not project["name"]:
        raise ConfigValidationError("project.name must be a non-empty string")
    if not isinstance(project["version"], str) or not project["version"]:
        raise ConfigValidationError("project.version must be a non-empty string")
    _integer(project["seed"], "project.seed", minimum=0)

    data = _mapping(config["data"], "data")
    _keys(
        data,
        "data",
        required=V1_DATA_REQUIRED_KEYS,
        optional=V2_DATA_EXTRA_KEYS,
    )
    data_format_version = _integer(
        data.get("format_version", 1), "data.format_version", minimum=1
    )
    if data_format_version not in {1, 2}:
        raise ConfigValidationError("data.format_version must be 1 or 2")
    if data_format_version == 2:
        validate_v2_data_config(data)
    else:
        unused_v2_keys = sorted((V2_DATA_EXTRA_KEYS - {"format_version"}) & set(data))
        if unused_v2_keys:
            raise ConfigValidationError(
                f"Dataset v2-only data keys require data.format_version=2: {unused_v2_keys}"
            )
    _number(data["sample_rate_hz"], "data.sample_rate_hz", minimum=0.0, strict_minimum=True)
    _number(data["sync_tolerance_ms"], "data.sync_tolerance_ms", minimum=0.0)
    _integer(data["image_height"], "data.image_height", minimum=1)
    _integer(data["image_width"], "data.image_width", minimum=1)
    _integer(data["lidar_points"], "data.lidar_points", minimum=1)
    lidar_min = _number(
        data["lidar_min_range_m"], "data.lidar_min_range_m", minimum=0.0
    )
    lidar_max = _number(
        data["lidar_max_range_m"],
        "data.lidar_max_range_m",
        minimum=0.0,
        strict_minimum=True,
    )
    if lidar_max <= lidar_min:
        raise ConfigValidationError(
            "data.lidar_max_range_m must be greater than data.lidar_min_range_m"
        )
    ego_dim = _integer(data["ego_dim"], "data.ego_dim", minimum=1)
    ego_features = data["ego_features"]
    if not isinstance(ego_features, list) or not ego_features or not all(
        isinstance(item, str) and item for item in ego_features
    ):
        raise ConfigValidationError("data.ego_features must be a non-empty string list")
    if len(set(ego_features)) != len(ego_features):
        raise ConfigValidationError("data.ego_features must not contain duplicates")
    if ego_dim != len(ego_features):
        raise ConfigValidationError("data.ego_dim must equal len(data.ego_features)")
    _number(
        data["prediction_horizon_sec"],
        "data.prediction_horizon_sec",
        minimum=0.0,
        strict_minimum=True,
    )
    num_waypoints = _integer(data["num_waypoints"], "data.num_waypoints", minimum=1)
    mode_classes = _mapping(data["mode_classes"], "data.mode_classes")
    if not mode_classes or not all(
        isinstance(name, str)
        and name
        and isinstance(index, int)
        and not isinstance(index, bool)
        and index >= 0
        for name, index in mode_classes.items()
    ):
        raise ConfigValidationError(
            "data.mode_classes must map non-empty names to non-negative integers"
        )
    if sorted(mode_classes.values()) != list(range(len(mode_classes))):
        raise ConfigValidationError("data.mode_classes indices must be contiguous from zero")

    model = _mapping(config["model"], "model")
    _keys(
        model,
        "model",
        required={
            "name",
            "initialization",
            "hidden_dim",
            "camera",
            "lidar",
            "ego",
            "fusion",
            "heads",
        },
    )
    _enum(model["name"], "model.name", {"transfuser_lite"})
    _enum(
        model["initialization"],
        "model.initialization",
        {"component_seeded_v1"},
    )
    _integer(model["hidden_dim"], "model.hidden_dim", minimum=1)

    camera = _mapping(model["camera"], "model.camera")
    _keys(
        camera,
        "model.camera",
        required={"backbone", "pretrained", "token_h", "token_w"},
    )
    _enum(camera["backbone"], "model.camera.backbone", {"resnet18"})
    _bool(camera["pretrained"], "model.camera.pretrained")
    camera_token_h = _integer(camera["token_h"], "model.camera.token_h", minimum=1)
    camera_token_w = _integer(camera["token_w"], "model.camera.token_w", minimum=1)
    if data_format_version == 2 and (camera_token_h, camera_token_w) != (6, 10):
        raise ConfigValidationError(
            "Dataset v2 static requires model.camera token geometry 6x10"
        )

    lidar = _mapping(model["lidar"], "model.lidar")
    _keys(
        lidar,
        "model.lidar",
        required={"encoder", "token_count", "use_valid_mask", "use_bev"},
    )
    _enum(lidar["encoder"], "model.lidar.encoder", {"cnn1d"})
    _integer(lidar["token_count"], "model.lidar.token_count", minimum=1)
    use_valid_mask = _bool(lidar["use_valid_mask"], "model.lidar.use_valid_mask")
    if data_format_version == 2 and not use_valid_mask:
        raise ConfigValidationError(
            "Dataset v2 requires model.lidar.use_valid_mask=true"
        )
    if use_valid_mask and data_format_version != 2:
        raise ConfigValidationError(
            "model.lidar.use_valid_mask=true is not implemented for Dataset format 1; "
            "use the explicit Dataset v2 model contract"
        )
    if _bool(lidar["use_bev"], "model.lidar.use_bev"):
        raise ConfigValidationError("model.lidar.use_bev=true is not implemented")

    ego = _mapping(model["ego"], "model.ego")
    _keys(ego, "model.ego", required={"hidden_dim"})
    _integer(ego["hidden_dim"], "model.ego.hidden_dim", minimum=1)

    fusion = _mapping(model["fusion"], "model.fusion")
    _keys(
        fusion,
        "model.fusion",
        required={"type", "depth", "heads", "mlp_ratio", "dropout", "pooling"},
    )
    _enum(fusion["type"], "model.fusion.type", {"transformer_encoder"})
    _integer(fusion["depth"], "model.fusion.depth", minimum=1)
    _integer(fusion["heads"], "model.fusion.heads", minimum=1)
    _integer(fusion["mlp_ratio"], "model.fusion.mlp_ratio", minimum=1)
    _number(fusion["dropout"], "model.fusion.dropout", minimum=0.0, maximum=1.0)
    _enum(fusion["pooling"], "model.fusion.pooling", {"learned_cls"})

    heads = _mapping(model["heads"], "model.heads")
    _keys(
        heads,
        "model.heads",
        required={
            "waypoints",
            "target_speed",
            "stop",
            "behavior_mode",
            "direct_control_aux",
        },
    )
    for name, enabled in heads.items():
        _bool(enabled, f"model.heads.{name}")
    if not heads["waypoints"] or not heads["target_speed"]:
        raise ConfigValidationError(
            "v1 static requires model.heads.waypoints and target_speed"
        )
    if data_format_version == 2 and any(
        heads[name] for name in ("stop", "behavior_mode", "direct_control_aux")
    ):
        raise ConfigValidationError(
            "Dataset v2 static requires stop, behavior_mode, and direct_control_aux Heads disabled"
        )

    weights = _mapping(config["loss_weights"], "loss_weights")
    _keys(
        weights,
        "loss_weights",
        required={"waypoint", "speed", "shape", "stop", "mode", "direct_control"},
    )
    for name, value in weights.items():
        _number(value, f"loss_weights.{name}", minimum=0.0)
    if float(weights["waypoint"]) <= 0.0:
        raise ConfigValidationError("loss_weights.waypoint must be > 0")
    optional_pairs = {
        "stop": "stop",
        "mode": "behavior_mode",
        "direct_control": "direct_control_aux",
    }
    for weight_name, head_name in optional_pairs.items():
        if not heads[head_name] and float(weights[weight_name]) != 0.0:
            raise ConfigValidationError(
                f"loss_weights.{weight_name} must be zero when "
                f"model.heads.{head_name} is disabled"
            )

    training = _mapping(config["training"], "training")
    _keys(
        training,
        "training",
        required={
            "batch_size",
            "epochs",
            "num_workers",
            "data_order_seed",
            "optimizer",
            "main_learning_rate",
            "backbone_learning_rate",
            "weight_decay",
            "mixed_precision",
            "grad_clip_norm",
            "warmup_fraction",
            "min_lr_ratio",
            "early_stopping_patience",
            "early_stopping_min_delta_m",
            "pin_memory",
            "persistent_workers",
            "prefetch_factor",
            "freeze_backbone_epochs",
            "waypoint_horizon_weights",
        },
        optional={"sampler"},
    )
    _integer(training["batch_size"], "training.batch_size", minimum=1)
    epochs = _integer(training["epochs"], "training.epochs", minimum=1)
    if epochs > 50:
        raise ConfigValidationError("training.epochs must be <= 50 for v1")
    num_workers = _integer(training["num_workers"], "training.num_workers", minimum=0)
    _integer(training["data_order_seed"], "training.data_order_seed", minimum=0)
    _enum(training["optimizer"], "training.optimizer", {"adamw"})
    _number(
        training["main_learning_rate"],
        "training.main_learning_rate",
        minimum=0.0,
        strict_minimum=True,
    )
    _number(
        training["backbone_learning_rate"],
        "training.backbone_learning_rate",
        minimum=0.0,
        strict_minimum=True,
    )
    _number(training["weight_decay"], "training.weight_decay", minimum=0.0)
    _bool(training["mixed_precision"], "training.mixed_precision")
    _number(
        training["grad_clip_norm"],
        "training.grad_clip_norm",
        minimum=0.0,
        strict_minimum=True,
    )
    warmup = _number(
        training["warmup_fraction"],
        "training.warmup_fraction",
        minimum=0.0,
        maximum=1.0,
    )
    if warmup >= 1.0:
        raise ConfigValidationError("training.warmup_fraction must be < 1")
    _number(
        training["min_lr_ratio"],
        "training.min_lr_ratio",
        minimum=0.0,
        maximum=1.0,
        strict_minimum=True,
    )
    _integer(
        training["early_stopping_patience"],
        "training.early_stopping_patience",
        minimum=1,
    )
    _number(
        training["early_stopping_min_delta_m"],
        "training.early_stopping_min_delta_m",
        minimum=0.0,
    )
    _bool(training["pin_memory"], "training.pin_memory")
    persistent = _bool(
        training["persistent_workers"], "training.persistent_workers"
    )
    prefetch = training["prefetch_factor"]
    if num_workers == 0:
        if persistent:
            raise ConfigValidationError(
                "training.persistent_workers must be false when num_workers is zero"
            )
        if prefetch is not None:
            raise ConfigValidationError(
                "training.prefetch_factor must be null when num_workers is zero"
            )
    else:
        _integer(prefetch, "training.prefetch_factor", minimum=1)
    freeze_epochs = _integer(
        training["freeze_backbone_epochs"],
        "training.freeze_backbone_epochs",
        minimum=0,
    )
    if freeze_epochs >= epochs:
        raise ConfigValidationError(
            "training.freeze_backbone_epochs must be less than training.epochs"
        )
    horizon_weights = training["waypoint_horizon_weights"]
    if not isinstance(horizon_weights, list) or len(horizon_weights) != num_waypoints:
        raise ConfigValidationError(
            "training.waypoint_horizon_weights length must equal data.num_waypoints"
        )
    for index, value in enumerate(horizon_weights):
        _number(
            value,
            f"training.waypoint_horizon_weights[{index}]",
            minimum=0.0,
            strict_minimum=True,
        )

    if data_format_version == 2:
        if "sampler" not in training:
            raise ConfigValidationError(
                "Dataset v2 requires an explicit training.sampler contract"
            )
        sampler = _mapping(training["sampler"], "training.sampler")
        _keys(
            sampler,
            "training.sampler",
            required={
                "type",
                "straight_threshold_per_m",
                "sharp_threshold_per_m",
                "max_weight",
                "recovery_weight",
            },
        )
        _enum(
            sampler["type"],
            "training.sampler.type",
            {"capped_inverse_frequency_curvature_recovery"},
        )
        straight_threshold = _number(
            sampler["straight_threshold_per_m"],
            "training.sampler.straight_threshold_per_m",
            minimum=0.0,
            strict_minimum=True,
        )
        sharp_threshold = _number(
            sampler["sharp_threshold_per_m"],
            "training.sampler.sharp_threshold_per_m",
            minimum=0.0,
            strict_minimum=True,
        )
        if sharp_threshold <= straight_threshold:
            raise ConfigValidationError(
                "training.sampler sharp threshold must be greater than straight threshold"
            )
        max_weight = _number(
            sampler["max_weight"],
            "training.sampler.max_weight",
            minimum=1.0,
        )
        recovery_weight = _number(
            sampler["recovery_weight"],
            "training.sampler.recovery_weight",
            minimum=1.0,
        )
        if recovery_weight > max_weight:
            raise ConfigValidationError(
                "training.sampler.recovery_weight must be <= max_weight"
            )
    elif "sampler" in training:
        raise ConfigValidationError(
            "training.sampler is only implemented for Dataset format 2"
        )


def load_v1_config(path: str | Path) -> dict[str, Any]:
    """Load and strictly validate a versioned TransFuser Lite v1 config."""
    config = load_config(path)
    validate_v1_config(config)
    return config
