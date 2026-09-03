from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .control_projection import ProjectedControlSequence
from .residual_control import ExternalControllerCommand
from .rollout_consistency import ConsistencyMetrics


class ControlAuthorityMode(str, Enum):
    SHADOW = "shadow_control"
    BOUNDED_RESIDUAL = "bounded_residual"
    FULL_CONTROL = "full_control"


@dataclass(frozen=True)
class FullControlReadiness:
    capabilities: frozenset[str]
    calibration_state: str
    deployment_stage: str
    safety_supervisor_ready: bool
    evidence_sha256: str
    evidence_passed: bool
    trial_speed_cap_mps: float | None = None

    def validate(self) -> None:
        if not {"trajectory", "control_sequence"}.issubset(self.capabilities):
            raise ValueError("full-control requires trajectory and control_sequence capabilities")
        if not self.safety_supervisor_ready:
            raise ValueError("full-control requires a ready Safety Supervisor")
        if len(self.evidence_sha256) != 64 or any(
            char not in "0123456789abcdef" for char in self.evidence_sha256
        ):
            raise ValueError("full-control evidence SHA-256 is invalid")
        if not self.evidence_passed:
            raise ValueError("full-control evidence did not pass")
        if self.deployment_stage == "limited_odd_trial":
            if self.calibration_state != "shadow":
                raise ValueError("limited-ODD trial requires shadow calibration state")
            if (
                self.trial_speed_cap_mps is None
                or not math.isfinite(self.trial_speed_cap_mps)
                or not 0.0 < self.trial_speed_cap_mps <= 1.0
            ):
                raise ValueError("limited-ODD trial requires a speed cap in (0,1] m/s")
        elif self.deployment_stage == "promoted":
            if self.calibration_state != "promoted":
                raise ValueError("promoted full-control requires promoted calibration")
            if self.trial_speed_cap_mps is not None:
                raise ValueError("promoted stage must not reuse a trial speed cap")
        else:
            raise ValueError("unknown full-control deployment stage")


@dataclass(frozen=True)
class FullControlDecision:
    command: ExternalControllerCommand
    source: str
    selected_trajectory_id: str
    consistency_reasons: tuple[str, ...]
    requires_safety_supervisor: bool = True


def authority_change_allowed(
    current: ControlAuthorityMode,
    target: ControlAuthorityMode,
    *,
    lifecycle_inactive: bool,
    longitudinal_speed_mps: float,
    stopped_threshold_mps: float = 0.05,
) -> bool:
    """Permit actual authority changes only while inactive or measured stopped."""

    if not math.isfinite(longitudinal_speed_mps):
        raise ValueError("authority transition speed must be finite")
    if not math.isfinite(stopped_threshold_mps) or stopped_threshold_mps < 0.0:
        raise ValueError("stopped threshold must be finite and non-negative")
    if current is target:
        return True
    return lifecycle_inactive or abs(longitudinal_speed_mps) <= stopped_threshold_mps


def choose_full_control_or_same_trajectory_fallback(
    sequence: ProjectedControlSequence,
    consistency: ConsistencyMetrics,
    fallback: ExternalControllerCommand | None,
    *,
    readiness: FullControlReadiness,
    selected_trajectory_id: str,
    fallback_trajectory_id: str,
) -> FullControlDecision:
    """Use model control only when consistent, otherwise the same-trajectory controller."""

    readiness.validate()
    if not selected_trajectory_id:
        raise ValueError("full-control selected trajectory ID must not be empty")
    if sequence.commands.ndim != 2 or sequence.commands.shape[1] != 3:
        raise ValueError("validated model sequence must be [H,3]")
    if consistency.consistent:
        first = sequence.commands[0]
        speed = float(first[1])
        if readiness.trial_speed_cap_mps is not None:
            speed = min(speed, readiness.trial_speed_cap_mps)
        return FullControlDecision(
            command=ExternalControllerCommand(float(first[0]), speed, float(first[2])),
            source="model_control_sequence",
            selected_trajectory_id=selected_trajectory_id,
            consistency_reasons=(),
        )
    if fallback is None:
        raise ValueError("inconsistent model sequence requires a same-trajectory fallback")
    if fallback_trajectory_id != selected_trajectory_id:
        raise ValueError("full-control fallback must use the same selected trajectory")
    fallback_values = (
        fallback.steering_rad,
        fallback.speed_mps,
        fallback.acceleration_mps2,
    )
    if not all(math.isfinite(value) for value in fallback_values) or fallback.speed_mps < 0.0:
        raise ValueError("full-control fallback command is invalid")
    fallback_command = fallback
    if (
        readiness.trial_speed_cap_mps is not None
        and fallback.speed_mps > readiness.trial_speed_cap_mps
    ):
        fallback_command = ExternalControllerCommand(
            fallback.steering_rad,
            readiness.trial_speed_cap_mps,
            fallback.acceleration_mps2,
        )
    return FullControlDecision(
        command=fallback_command,
        source="same_trajectory_external_fallback",
        selected_trajectory_id=selected_trajectory_id,
        consistency_reasons=consistency.reasons or ("rollout_inconsistent",),
    )
