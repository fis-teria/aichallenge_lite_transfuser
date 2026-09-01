from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


KNOWN_CAPABILITIES = frozenset(
    {
        "camera_input",
        "lidar_input",
        "trajectory_label",
        "v1_compatibility",
        "actual_steering",
        "lateral_calibration",
        "full_control_label",
        "event_label",
    }
)


@dataclass(frozen=True)
class CapabilityRule:
    name: str
    all_roles: tuple[str, ...] = ()
    any_roles: tuple[str, ...] = ()

    def validate(self, *, known_roles: frozenset[str]) -> None:
        if self.name not in KNOWN_CAPABILITIES:
            raise ValueError(f"unknown capability: {self.name!r}")
        if not self.all_roles and not self.any_roles:
            raise ValueError(f"capability {self.name!r} has no role requirements")
        roles = self.all_roles + self.any_roles
        if len(set(roles)) != len(roles):
            raise ValueError(f"capability {self.name!r} repeats a role")
        unknown = sorted(set(roles).difference(known_roles))
        if unknown:
            raise ValueError(f"capability {self.name!r} uses unknown roles: {unknown}")


@dataclass(frozen=True)
class CapabilityAssessment:
    available: frozenset[str]
    unavailable: Mapping[str, tuple[str, ...]]


def assess_capabilities(
    rules: tuple[CapabilityRule, ...], *, available_roles: frozenset[str]
) -> CapabilityAssessment:
    """Evaluate role-based capabilities without treating optional roles as fatal."""

    available: set[str] = set()
    unavailable: dict[str, tuple[str, ...]] = {}
    for rule in rules:
        missing_all = tuple(role for role in rule.all_roles if role not in available_roles)
        any_satisfied = not rule.any_roles or any(
            role in available_roles for role in rule.any_roles
        )
        if not missing_all and any_satisfied:
            available.add(rule.name)
            continue
        missing_any = tuple(
            f"any:{role}" for role in rule.any_roles if role not in available_roles
        )
        unavailable[rule.name] = missing_all + missing_any
    return CapabilityAssessment(frozenset(available), unavailable)
