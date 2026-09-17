"""ROS-free speed semantics used at control decision boundaries."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias, Union


class SpeedSource(str, Enum):
    """Origin whose availability and mutation semantics matter."""

    BASE_REFERENCE = "base_reference"
    CONSTRAINED = "constrained"


class VelocityConstraintOwner(str, Enum):
    """Subsystem that proposed a scalar upper-speed constraint."""

    COLLISION_BRAKING = "collision_braking"


@dataclass(frozen=True)
class BaseReferenceSpeed:
    """Non-negative speed from the immutable course reference profile."""

    value_mps: float

    def __post_init__(self) -> None:
        if isinstance(self.value_mps, bool):
            raise TypeError("base reference speed must be numeric, not bool")
        value_mps = float(self.value_mps)
        if not math.isfinite(value_mps) or value_mps < 0.0:
            raise ValueError(
                "base reference speed must be finite and non-negative"
            )
        object.__setattr__(self, "value_mps", value_mps)


@dataclass(frozen=True)
class ConstrainedSpeed:
    """Non-negative speed after mutable transient limits were applied."""

    value_mps: float

    def __post_init__(self) -> None:
        if isinstance(self.value_mps, bool):
            raise TypeError("constrained speed must be numeric, not bool")
        value_mps = float(self.value_mps)
        if not math.isfinite(value_mps) or value_mps < 0.0:
            raise ValueError(
                "constrained speed must be finite and non-negative"
            )
        object.__setattr__(self, "value_mps", value_mps)


@dataclass(frozen=True)
class SpeedUnavailable:
    """Explicit replacement for a non-finite legacy speed sample."""

    source: SpeedSource
    reason: str = "nonfinite_legacy_value"

    def __post_init__(self) -> None:
        if not isinstance(self.source, SpeedSource):
            raise TypeError("speed unavailable source must be a SpeedSource")
        if not self.reason:
            raise ValueError("speed unavailable reason must not be empty")


@dataclass(frozen=True)
class VelocityConstraint:
    """One owner-tagged non-negative upper speed constraint."""

    owner: VelocityConstraintOwner
    maximum_speed_mps: float

    def __post_init__(self) -> None:
        if not isinstance(self.owner, VelocityConstraintOwner):
            raise TypeError(
                "velocity constraint owner must be a VelocityConstraintOwner"
            )
        if isinstance(self.maximum_speed_mps, bool):
            raise TypeError(
                "velocity constraint maximum must be numeric, not bool"
            )
        maximum_speed_mps = float(self.maximum_speed_mps)
        if not math.isfinite(maximum_speed_mps) or maximum_speed_mps < 0.0:
            raise ValueError(
                "velocity constraint maximum must be finite and non-negative"
            )
        object.__setattr__(self, "maximum_speed_mps", maximum_speed_mps)


@dataclass(frozen=True)
class SpeedConstraintProposals:
    """All speed-limit proposals observed at one resolution boundary."""

    normal_speed_limit: ConstrainedSpeed
    constraints: tuple[VelocityConstraint, ...]
    relax_collision_braking_horizon: bool

    def __post_init__(self) -> None:
        if not isinstance(self.normal_speed_limit, ConstrainedSpeed):
            raise TypeError("normal_speed_limit must be a ConstrainedSpeed")
        if not isinstance(self.constraints, tuple):
            raise TypeError("speed constraints must be a tuple")
        if any(
            not isinstance(constraint, VelocityConstraint)
            for constraint in self.constraints
        ):
            raise TypeError(
                "speed constraints must contain VelocityConstraint values"
            )
        owners = tuple(constraint.owner for constraint in self.constraints)
        if len(set(owners)) != len(owners):
            raise ValueError("speed constraint owners must be unique")
        if not isinstance(self.relax_collision_braking_horizon, bool):
            raise TypeError("relax_collision_braking_horizon must be bool")

    def constraint_for(
        self,
        owner: VelocityConstraintOwner,
    ) -> Union[VelocityConstraint, None]:
        if not isinstance(owner, VelocityConstraintOwner):
            raise TypeError("constraint owner must be a VelocityConstraintOwner")
        return next(
            (
                constraint
                for constraint in self.constraints
                if constraint.owner is owner
            ),
            None,
        )


@dataclass(frozen=True)
class SpeedLimitResolution:
    """Reference and MPC-horizon limits derived in one resolver call."""

    reference_speed: ConstrainedSpeed
    horizon_speed: ConstrainedSpeed
    collision_braking_constraint: Union[VelocityConstraint, None]

    def __post_init__(self) -> None:
        if not isinstance(self.reference_speed, ConstrainedSpeed):
            raise TypeError("reference_speed must be a ConstrainedSpeed")
        if not isinstance(self.horizon_speed, ConstrainedSpeed):
            raise TypeError("horizon_speed must be a ConstrainedSpeed")
        if (
            self.collision_braking_constraint is not None
            and not isinstance(
                self.collision_braking_constraint,
                VelocityConstraint,
            )
        ):
            raise TypeError(
                "collision_braking_constraint must be a VelocityConstraint "
                "or None"
            )


BaseReferenceSpeedSample: TypeAlias = Union[
    BaseReferenceSpeed,
    SpeedUnavailable,
]
ConstrainedSpeedSample: TypeAlias = Union[
    ConstrainedSpeed,
    SpeedUnavailable,
]


def classify_legacy_base_reference_speed(
    value_mps: float,
) -> BaseReferenceSpeedSample:
    """Preserve the legacy finite-check and zero clamp explicitly."""

    if isinstance(value_mps, bool):
        raise TypeError("base reference speed must be numeric, not bool")
    value_mps = float(value_mps)
    if not math.isfinite(value_mps):
        return SpeedUnavailable(source=SpeedSource.BASE_REFERENCE)
    return BaseReferenceSpeed(max(0.0, value_mps))


def classify_legacy_constrained_speed(
    value_mps: float,
) -> ConstrainedSpeedSample:
    """Preserve the legacy finite-check and zero clamp explicitly."""

    if isinstance(value_mps, bool):
        raise TypeError("constrained speed must be numeric, not bool")
    value_mps = float(value_mps)
    if not math.isfinite(value_mps):
        return SpeedUnavailable(source=SpeedSource.CONSTRAINED)
    return ConstrainedSpeed(max(0.0, value_mps))


def resolve_legacy_speed_limits(
    *,
    proposals: SpeedConstraintProposals,
) -> SpeedLimitResolution:
    """Preserve the existing reference/horizon braking-cap split.

    Collision braking always caps the soft reference speed.  When the legacy
    relaxation flag is true, the MPC horizon retains the normal limit so the
    lateral plan is not shortened; otherwise both limits use the same cap.
    """

    if not isinstance(proposals, SpeedConstraintProposals):
        raise TypeError("proposals must be SpeedConstraintProposals")
    normal_speed_limit = proposals.normal_speed_limit
    collision_braking_constraint = proposals.constraint_for(
        VelocityConstraintOwner.COLLISION_BRAKING
    )
    if collision_braking_constraint is None:
        return SpeedLimitResolution(
            reference_speed=normal_speed_limit,
            horizon_speed=normal_speed_limit,
            collision_braking_constraint=None,
        )
    if not isinstance(collision_braking_constraint, VelocityConstraint):
        raise TypeError(
            "collision_braking_constraint must be a VelocityConstraint"
        )
    if (
        collision_braking_constraint.owner
        != VelocityConstraintOwner.COLLISION_BRAKING
    ):
        raise ValueError("unexpected collision braking constraint owner")

    limited_value_mps = min(
        normal_speed_limit.value_mps,
        collision_braking_constraint.maximum_speed_mps,
    )
    reference_speed = (
        normal_speed_limit
        if limited_value_mps == normal_speed_limit.value_mps
        else ConstrainedSpeed(limited_value_mps)
    )
    horizon_speed = (
        normal_speed_limit
        if proposals.relax_collision_braking_horizon
        else reference_speed
    )
    return SpeedLimitResolution(
        reference_speed=reference_speed,
        horizon_speed=horizon_speed,
        collision_braking_constraint=collision_braking_constraint,
    )
