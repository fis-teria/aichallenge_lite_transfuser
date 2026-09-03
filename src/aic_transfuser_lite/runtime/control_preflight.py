from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControlPreflightV3:
    ready: bool
    reasons: tuple[str, ...]
    gear_report: int | None
    control_mode_report: int | None
    awsim_state: str | None


def evaluate_control_preflight_v3(
    *,
    gear_report: int | None,
    control_mode_report: int | None,
    awsim_state: str | None,
    gear_age_sec: float | None,
    control_mode_age_sec: float | None,
    awsim_state_age_sec: float | None,
    maximum_status_age_sec: float,
    expected_drive_gear: int,
    expected_autonomous_mode: int,
    required_awsim_state: str,
    nominal_publishers: int,
    nominal_subscribers: int,
    final_publishers: int,
    final_subscribers: int,
) -> ControlPreflightV3:
    """Check Drive/Autonomous/Start and unique nominal/final ROS routing."""

    if not math.isfinite(maximum_status_age_sec) or maximum_status_age_sec <= 0.0:
        raise ValueError("maximum_status_age_sec must be finite and positive")
    if not required_awsim_state:
        raise ValueError("required_awsim_state must not be empty")
    counts = (
        nominal_publishers,
        nominal_subscribers,
        final_publishers,
        final_subscribers,
    )
    if any(not isinstance(value, int) or value < 0 for value in counts):
        raise ValueError("ROS graph endpoint counts must be non-negative integers")
    reasons: list[str] = []
    status = (
        ("gear", gear_report, gear_age_sec),
        ("control_mode", control_mode_report, control_mode_age_sec),
    )
    for name, value, age_sec in status:
        if value is None or age_sec is None:
            reasons.append(f"{name}_missing")
        elif not math.isfinite(age_sec) or age_sec < 0.0:
            reasons.append(f"{name}_age_invalid")
        elif age_sec > maximum_status_age_sec:
            reasons.append(f"{name}_stale")
    if awsim_state is None or awsim_state_age_sec is None:
        reasons.append("awsim_state_missing")
    elif not math.isfinite(awsim_state_age_sec) or awsim_state_age_sec < 0.0:
        reasons.append("awsim_state_age_invalid")
    if gear_report is not None and gear_report != expected_drive_gear:
        reasons.append("gear_not_drive")
    if (
        control_mode_report is not None
        and control_mode_report != expected_autonomous_mode
    ):
        reasons.append("control_mode_not_autonomous")
    if awsim_state is not None and awsim_state != required_awsim_state:
        reasons.append("awsim_not_started")
    if nominal_publishers != 1:
        reasons.append("nominal_publisher_count")
    if nominal_subscribers != 1:
        reasons.append("nominal_subscriber_count")
    if final_publishers != 1:
        reasons.append("final_publisher_count")
    if final_subscribers != 1:
        reasons.append("final_subscriber_count")
    return ControlPreflightV3(
        ready=not reasons,
        reasons=tuple(reasons),
        gear_report=gear_report,
        control_mode_report=control_mode_report,
        awsim_state=awsim_state,
    )
