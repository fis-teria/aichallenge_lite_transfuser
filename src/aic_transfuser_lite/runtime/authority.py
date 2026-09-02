from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class AuthorityRole(str, Enum):
    DEBUG = "debug"
    NOMINAL = "nominal"
    FINAL = "final"


@dataclass(frozen=True)
class PublisherOwnership:
    owner: str
    topic: str
    role: AuthorityRole


@dataclass(frozen=True)
class ShadowAuthorityContract:
    """Unambiguous publisher ownership for V3 model-control shadowing."""

    publishers: tuple[PublisherOwnership, ...]

    def validate(self) -> None:
        if len({claim.topic for claim in self.publishers}) != len(self.publishers):
            raise ValueError("authority contract contains duplicate publisher topics")
        by_role = {
            role: tuple(claim for claim in self.publishers if claim.role is role)
            for role in AuthorityRole
        }
        if by_role[AuthorityRole.DEBUG] != (
            PublisherOwnership(
                owner="inference_node_v3",
                topic="shadow_model_control",
                role=AuthorityRole.DEBUG,
            ),
        ):
            raise ValueError("model control must have exactly one debug-only publisher")
        if by_role[AuthorityRole.NOMINAL] != (
            PublisherOwnership(
                owner="external_controller",
                topic="nominal_control_cmd",
                role=AuthorityRole.NOMINAL,
            ),
        ):
            raise ValueError("external controller must exclusively own nominal control")
        if by_role[AuthorityRole.FINAL] != (
            PublisherOwnership(
                owner="safety_supervisor",
                topic="control/command/control_cmd",
                role=AuthorityRole.FINAL,
            ),
        ):
            raise ValueError("Safety Supervisor must exclusively own final control")


@dataclass(frozen=True)
class DebugModelControl:
    """Candidate-zero debug proposal ordered steering, speed, acceleration."""

    steering_rad: float
    speed_mps: float
    acceleration_mps2: float
    authoritative: bool = False


def shadow_authority_contract() -> ShadowAuthorityContract:
    contract = ShadowAuthorityContract(
        publishers=(
            PublisherOwnership(
                owner="inference_node_v3",
                topic="shadow_model_control",
                role=AuthorityRole.DEBUG,
            ),
            PublisherOwnership(
                owner="external_controller",
                topic="nominal_control_cmd",
                role=AuthorityRole.NOMINAL,
            ),
            PublisherOwnership(
                owner="safety_supervisor",
                topic="control/command/control_cmd",
                role=AuthorityRole.FINAL,
            ),
        )
    )
    contract.validate()
    return contract


def model_control_debug_publication(current_control: np.ndarray) -> DebugModelControl:
    """Validate model ``[1,K,3]`` output and select candidate zero for debug only."""

    control = np.asarray(current_control, dtype=np.float64)
    if control.ndim != 3 or control.shape[0] != 1 or control.shape[1] < 1:
        raise ValueError(f"current_control must be [1,K,3], got {control.shape}")
    if control.shape[2] != 3:
        raise ValueError(f"current_control must be [1,K,3], got {control.shape}")
    selected = control[0, 0]
    if not np.isfinite(selected).all():
        raise ValueError("current_control candidate zero must be finite")
    if selected[1] < 0.0:
        raise ValueError("current_control speed must be non-negative")
    return DebugModelControl(
        steering_rad=float(selected[0]),
        speed_mps=float(selected[1]),
        acceleration_mps2=float(selected[2]),
        authoritative=False,
    )
