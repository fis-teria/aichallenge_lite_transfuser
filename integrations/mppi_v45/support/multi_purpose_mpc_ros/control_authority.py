"""ROS-free authority selection for one Controller control tick."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class ControlAuthority(str, Enum):
    """The single subsystem whose lateral command is used this tick."""

    RECOVERY_REVERSE = "RECOVERY_REVERSE"
    RECOVERY_STOPPING = "RECOVERY_STOPPING"
    RECOVERY_FORWARD = "RECOVERY_FORWARD"
    RECOVERY_ESCAPE_TURN = "RECOVERY_ESCAPE_TURN"
    RETURN_PURE = "RETURN_PURE"
    AVOID_PURE = "AVOID_PURE"
    AVOID_MPC = "AVOID_MPC"
    CURVE_PURE = "CURVE_PURE"
    MPC = "MPC"

    @property
    def uses_pure_pursuit(self) -> bool:
        return self in (
            ControlAuthority.RETURN_PURE,
            ControlAuthority.AVOID_PURE,
            ControlAuthority.CURVE_PURE,
        )

    @property
    def uses_avoidance_pure_pursuit(self) -> bool:
        return self in (
            ControlAuthority.RETURN_PURE,
            ControlAuthority.AVOID_PURE,
        )

    @property
    def is_recovery(self) -> bool:
        return self in (
            ControlAuthority.RECOVERY_REVERSE,
            ControlAuthority.RECOVERY_STOPPING,
            ControlAuthority.RECOVERY_FORWARD,
            ControlAuthority.RECOVERY_ESCAPE_TURN,
        )

    @property
    def legacy_steering_mode(self) -> str:
        """Preserve the existing diagnostic mode during recovery.

        The legacy diagnostic reported ``MPC`` while a recovery branch owned
        the command because all avoidance/curve flags were suppressed.  Keep
        that output until diagnostics are migrated separately.
        """

        if self.is_recovery:
            return ControlAuthority.MPC.value
        return str(self.value)


@dataclass(frozen=True)
class ControlAuthorityProposals:
    """All lateral-control ownership requests observed in one control tick."""

    recovery_moving_reverse_requested: bool
    recovery_stopping_requested: bool
    recovery_moving_forward_requested: bool
    recovery_escape_turning_requested: bool
    avoid_return_requested: bool
    avoid_pure_requested: bool
    avoid_mpc_requested: bool
    curve_pure_requested: bool

    def __post_init__(self) -> None:
        if any(
            not isinstance(value, bool)
            for value in (
                self.recovery_moving_reverse_requested,
                self.recovery_stopping_requested,
                self.recovery_moving_forward_requested,
                self.recovery_escape_turning_requested,
                self.avoid_return_requested,
                self.avoid_pure_requested,
                self.avoid_mpc_requested,
                self.curve_pure_requested,
            )
        ):
            raise TypeError("control authority proposals must be bool")


class RecoveryAuthorityProposalView(Protocol):
    """Recovery ownership facts consumed by the final authority resolver."""

    @property
    def moving_reverse(self) -> bool: ...

    @property
    def stopping(self) -> bool: ...

    @property
    def moving_forward(self) -> bool: ...

    @property
    def escape_turning(self) -> bool: ...


class AvoidanceAuthorityProposalView(Protocol):
    """Avoidance ownership facts consumed by the final authority resolver."""

    return_pure_requested: bool
    avoid_pure_requested: bool
    avoid_mpc_requested: bool
    curve_pure_requested: bool


def compose_control_authority_proposals(
    *,
    recovery: RecoveryAuthorityProposalView,
    avoidance: AvoidanceAuthorityProposalView,
) -> ControlAuthorityProposals:
    """Combine validated domain proposals without re-deriving their policy."""

    return ControlAuthorityProposals(
        recovery_moving_reverse_requested=recovery.moving_reverse,
        recovery_stopping_requested=recovery.stopping,
        recovery_moving_forward_requested=recovery.moving_forward,
        recovery_escape_turning_requested=recovery.escape_turning,
        avoid_return_requested=avoidance.return_pure_requested,
        avoid_pure_requested=avoidance.avoid_pure_requested,
        avoid_mpc_requested=avoidance.avoid_mpc_requested,
        curve_pure_requested=avoidance.curve_pure_requested,
    )


@dataclass(frozen=True)
class ControlDecision:
    """Final command semantics shared by commit and publish consumers."""

    authority: ControlAuthority
    internal_speed_mps: float
    steering_rad: float
    acceleration_mps2: float
    publish_in_reverse_gear: bool
    bug_acc_enabled: bool

    def __post_init__(self) -> None:
        if not isinstance(self.authority, ControlAuthority):
            raise TypeError("control decision authority must be ControlAuthority")
        for field_name in (
            "internal_speed_mps",
            "steering_rad",
            "acceleration_mps2",
        ):
            value = getattr(self, field_name)
            if isinstance(value, bool):
                raise TypeError(f"control decision {field_name} must be numeric")
            object.__setattr__(self, field_name, float(value))
        if not isinstance(self.publish_in_reverse_gear, bool):
            raise TypeError("publish_in_reverse_gear must be bool")
        if not isinstance(self.bug_acc_enabled, bool):
            raise TypeError("bug_acc_enabled must be bool")
        if self.publish_in_reverse_gear and self.authority not in (
            ControlAuthority.RECOVERY_REVERSE,
            ControlAuthority.RECOVERY_STOPPING,
        ):
            raise ValueError(
                "reverse-gear publication requires reverse or stopping authority"
            )

    @property
    def published_speed_mps(self) -> float:
        return (
            -self.internal_speed_mps
            if self.publish_in_reverse_gear
            else self.internal_speed_mps
        )

    @property
    def published_acceleration_mps2(self) -> float:
        return (
            -self.acceleration_mps2
            if self.publish_in_reverse_gear
            else self.acceleration_mps2
        )


class PreviousControlDecisionState:
    """Retain the last committed decision values used by control filters."""

    def __init__(self) -> None:
        self._decision: ControlDecision | None = None

    @property
    def internal_speed_mps(self) -> float:
        return (
            0.0
            if self._decision is None
            else self._decision.internal_speed_mps
        )

    @property
    def steering_rad(self) -> float:
        return 0.0 if self._decision is None else self._decision.steering_rad

    @property
    def acceleration_mps2(self) -> float:
        return (
            0.0
            if self._decision is None
            else self._decision.acceleration_mps2
        )

    def record(self, *, decision: ControlDecision) -> None:
        self._decision = decision

    def reset(self) -> None:
        self._decision = None


def resolve_legacy_control_authority(
    *,
    proposals: ControlAuthorityProposals,
) -> ControlAuthority:
    """Select exactly one owner in the Controller's existing branch order."""

    if proposals.recovery_moving_reverse_requested:
        return ControlAuthority.RECOVERY_REVERSE
    if proposals.recovery_stopping_requested:
        return ControlAuthority.RECOVERY_STOPPING
    if proposals.recovery_moving_forward_requested:
        return ControlAuthority.RECOVERY_FORWARD
    if proposals.recovery_escape_turning_requested:
        return ControlAuthority.RECOVERY_ESCAPE_TURN
    if proposals.avoid_return_requested:
        return ControlAuthority.RETURN_PURE
    if proposals.avoid_pure_requested:
        return ControlAuthority.AVOID_PURE
    if proposals.avoid_mpc_requested:
        return ControlAuthority.AVOID_MPC
    if proposals.curve_pure_requested:
        return ControlAuthority.CURVE_PURE
    return ControlAuthority.MPC
