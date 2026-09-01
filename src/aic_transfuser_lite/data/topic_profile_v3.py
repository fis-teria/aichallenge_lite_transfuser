from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from aic_transfuser_lite.contracts.capabilities import (
    CapabilityAssessment,
    CapabilityRule,
    assess_capabilities,
)


TOPIC_PROFILE_FORMAT_V3 = "aic_topic_profile_v3"
TIMESTAMP_POLICIES = frozenset(
    {"header_required", "stamp_required", "clock_value", "bag_stamp"}
)
SYNC_POLICIES = frozenset(
    {
        "regular_grid_master",
        "nearest",
        "causal_previous",
        "linear_interpolation",
        "angle_interpolation",
        "exact_event",
    }
)


@dataclass(frozen=True)
class TopicSpecV3:
    role: str
    name: str
    message_type: str
    required_for_recording: bool
    required_for_conversion: bool
    timestamp_policy: str
    sync_policy: str
    tolerance_ms: float | None

    def validate(self) -> None:
        if not self.role or not self.name.startswith("/") or not self.message_type:
            raise ValueError(f"invalid topic identity for role {self.role!r}")
        if self.timestamp_policy not in TIMESTAMP_POLICIES:
            raise ValueError(f"unknown timestamp policy: {self.timestamp_policy!r}")
        if self.sync_policy not in SYNC_POLICIES:
            raise ValueError(f"unknown synchronization policy: {self.sync_policy!r}")
        if self.sync_policy == "exact_event":
            if self.tolerance_ms is not None:
                raise ValueError("exact_event tolerance must be null")
        elif self.tolerance_ms is None or self.tolerance_ms <= 0.0:
            raise ValueError(f"{self.role} synchronization tolerance must be positive")


@dataclass(frozen=True)
class TopicProfileV3:
    profile_id: str
    roles: Mapping[str, TopicSpecV3]
    capabilities: tuple[CapabilityRule, ...]
    format_version: str = TOPIC_PROFILE_FORMAT_V3

    def validate(self) -> None:
        if self.format_version != TOPIC_PROFILE_FORMAT_V3:
            raise ValueError(f"unsupported topic profile format: {self.format_version!r}")
        if not self.profile_id or not self.roles:
            raise ValueError("topic profile id and roles must be non-empty")
        names: set[str] = set()
        for role, spec in self.roles.items():
            if role != spec.role:
                raise ValueError(f"topic role key/name mismatch: {role!r}/{spec.role!r}")
            spec.validate()
            if spec.name in names:
                raise ValueError(f"duplicate topic name: {spec.name!r}")
            names.add(spec.name)
        known_roles = frozenset(self.roles)
        capability_names: set[str] = set()
        for rule in self.capabilities:
            rule.validate(known_roles=known_roles)
            if rule.name in capability_names:
                raise ValueError(f"duplicate capability: {rule.name!r}")
            capability_names.add(rule.name)


@dataclass(frozen=True)
class TopicProfileAssessmentV3:
    available_roles: frozenset[str]
    missing_for_recording: tuple[str, ...]
    missing_for_conversion: tuple[str, ...]
    capabilities: CapabilityAssessment

    @property
    def conversion_accepted(self) -> bool:
        return not self.missing_for_conversion


def load_topic_profile_v3(path: str | Path) -> TopicProfileV3:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"Topic profile not found: {source}")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("topic profile root must be a mapping")
    allowed_root = {"format_version", "profile_id", "roles", "capabilities"}
    unknown_root = sorted(set(raw).difference(allowed_root))
    if unknown_root:
        raise ValueError(f"unknown topic profile fields: {unknown_root}")
    role_values = raw.get("roles")
    capability_values = raw.get("capabilities")
    if not isinstance(role_values, dict) or not isinstance(capability_values, dict):
        raise ValueError("topic profile roles and capabilities must be mappings")
    roles: dict[str, TopicSpecV3] = {}
    for role, value in role_values.items():
        if not isinstance(value, dict):
            raise ValueError(f"topic role {role!r} must be a mapping")
        sync = value.get("synchronization")
        if not isinstance(sync, dict):
            raise ValueError(f"topic role {role!r} lacks synchronization mapping")
        roles[str(role)] = TopicSpecV3(
            role=str(role),
            name=str(value.get("topic", "")),
            message_type=str(value.get("type", "")),
            required_for_recording=_strict_bool(
                value.get("required_for_recording"), f"{role}.required_for_recording"
            ),
            required_for_conversion=_strict_bool(
                value.get("required_for_conversion"), f"{role}.required_for_conversion"
            ),
            timestamp_policy=str(value.get("timestamp_policy", "")),
            sync_policy=str(sync.get("policy", "")),
            tolerance_ms=(
                None if sync.get("tolerance_ms") is None else float(sync["tolerance_ms"])
            ),
        )
    rules: list[CapabilityRule] = []
    for name, value in capability_values.items():
        if not isinstance(value, dict):
            raise ValueError(f"capability {name!r} must be a mapping")
        rules.append(
            CapabilityRule(
                name=str(name),
                all_roles=_string_tuple(value.get("all_roles", ()), f"{name}.all_roles"),
                any_roles=_string_tuple(value.get("any_roles", ()), f"{name}.any_roles"),
            )
        )
    profile = TopicProfileV3(
        profile_id=str(raw.get("profile_id", "")),
        roles=roles,
        capabilities=tuple(rules),
        format_version=str(raw.get("format_version", "")),
    )
    profile.validate()
    return profile


def assess_topic_profile_v3(
    profile: TopicProfileV3, observed_topics: Mapping[str, str]
) -> TopicProfileAssessmentV3:
    """Validate exact observed types and separately report three requirement axes."""

    profile.validate()
    available_roles: set[str] = set()
    for role, spec in profile.roles.items():
        observed_type = observed_topics.get(spec.name)
        if observed_type is None:
            continue
        if observed_type != spec.message_type:
            raise ValueError(
                f"topic type mismatch for {spec.name}: observed={observed_type!r}, "
                f"expected={spec.message_type!r}"
            )
        available_roles.add(role)
    missing_recording = tuple(
        role
        for role, spec in profile.roles.items()
        if spec.required_for_recording and role not in available_roles
    )
    missing_conversion = tuple(
        role
        for role, spec in profile.roles.items()
        if spec.required_for_conversion and role not in available_roles
    )
    capabilities = assess_capabilities(
        profile.capabilities, available_roles=frozenset(available_roles)
    )
    return TopicProfileAssessmentV3(
        available_roles=frozenset(available_roles),
        missing_for_recording=missing_recording,
        missing_for_conversion=missing_conversion,
        capabilities=capabilities,
    )


def _strict_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _string_tuple(value: Any, name: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        if value == ():
            return ()
        raise ValueError(f"{name} must be a list")
    result = tuple(str(item) for item in value)
    if any(not item for item in result):
        raise ValueError(f"{name} contains an empty role")
    return result
