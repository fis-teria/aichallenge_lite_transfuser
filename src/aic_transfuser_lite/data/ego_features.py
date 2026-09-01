from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any


LEGACY_EGO_FEATURES = (
    "longitudinal_speed_mps",
    "lateral_speed_mps",
    "yaw_rate_rps",
    "steering_rad",
    "gear",
)
SUPPORTED_EGO_FEATURES = frozenset(
    {
        "speed_mps",
        "longitudinal_speed_mps",
        "lateral_speed_mps",
        "yaw_rate_rps",
        "steering_rad",
        "gear",
    }
)


def configured_ego_features(data_config: Mapping[str, Any]) -> tuple[str, ...]:
    """Return the ordered ego input contract, preserving legacy v0 by default."""

    raw = data_config.get("ego_features", LEGACY_EGO_FEATURES)
    if not isinstance(raw, (list, tuple)) or not raw:
        raise ValueError("data.ego_features must be a non-empty list")
    features = tuple(str(value) for value in raw)
    unknown = sorted(set(features).difference(SUPPORTED_EGO_FEATURES))
    if unknown:
        raise ValueError(f"Unsupported ego features: {unknown}")
    if len(set(features)) != len(features):
        raise ValueError(f"Duplicate ego features are not allowed: {features}")
    ego_dim = int(data_config.get("ego_dim", len(features)))
    if ego_dim != len(features):
        raise ValueError(
            f"data.ego_dim={ego_dim} does not match ego_features={features}"
        )
    return features


def select_ego_features(
    features: Sequence[str], values: Mapping[str, float]
) -> tuple[float, ...]:
    """Select an ordered ego vector from explicitly named, unit-bearing values."""

    missing = [name for name in features if name not in values]
    if missing:
        raise ValueError(f"Missing ego feature values: {missing}")
    return tuple(float(values[name]) for name in features)
