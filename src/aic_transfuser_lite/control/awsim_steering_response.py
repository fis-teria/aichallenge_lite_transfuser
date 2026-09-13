"""Causal bounded steering lead for the hash-pinned AWSIM actuator, in SI.

The physical PP request stays observable. Only the actuator target receives
lead = (70 ms transport delay + 20 ms response constant) * filtered request
rate. This is not an inverse of arbitrary future motion or a sensor predictor.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

from .awsim_steering import LEAD_POLICY, steering_response_gain


@dataclass(frozen=True)
class SteeringResponseState:
    """Last admitted nominal tire request and filtered rate at simulation time."""

    nominal_tire_rad: float
    filtered_rate_radps: float
    stamp_ns: int


def compensate_steering_response(
    nominal_tire_rad: float, stamp_ns: int, previous: SteeringResponseState | None,
    *, policy: str,
) -> tuple[float, SteeringResponseState | None, dict[str, Any]]:
    """Return (physical actuator target, candidate state, replayable diagnostics).

    Inputs are scalars: rad, simulation ns and rad/s; no wall/report-age lead.
    Caller commits candidate state only after admission and publication, and
    clears it on rejection/authority loss. A first, duplicate, reversed or
    >150 ms clock step primes without lead. Malformed values fail explicitly.
    The original nominal request must be feasible before compensation/clipping.
    """
    gain = steering_response_gain(policy)
    limit = .5 * gain
    if not math.isfinite(nominal_tire_rad) or type(stamp_ns) is not int or stamp_ns < 0:
        raise ValueError("STEERING_RESPONSE_CONTRACT")
    if abs(nominal_tire_rad) > limit:
        raise ValueError("STEERING_ACTUATOR_INFEASIBLE")
    if previous is not None and (
        not isinstance(previous, SteeringResponseState)
        or not all(math.isfinite(v) for v in (previous.nominal_tire_rad, previous.filtered_rate_radps))
        or abs(previous.nominal_tire_rad) > limit or abs(previous.filtered_rate_radps) > .48
        or type(previous.stamp_ns) is not int or previous.stamp_ns < 0
    ):
        raise ValueError("STEERING_RESPONSE_STATE")
    dt_s = None if previous is None else (stamp_ns - previous.stamp_ns) / 1e9
    active = policy == LEAD_POLICY
    reset = not active or dt_s is None or not 0. < dt_s <= .15
    raw_rate = filtered_rate = 0.
    if not reset:
        raw_rate = (nominal_tire_rad - previous.nominal_tire_rad) / dt_s
        bounded_rate = max(-.48, min(.48, raw_rate))
        alpha = dt_s / (.05 + dt_s)
        filtered_rate = previous.filtered_rate_radps + alpha * (bounded_rate - previous.filtered_rate_radps)
    correction = .09 * filtered_rate
    target = max(-limit, min(limit, nominal_tire_rad + correction))
    candidate = SteeringResponseState(nominal_tire_rad, filtered_rate, stamp_ns) if active else None
    return target, candidate, {
        "policy": policy, "nominal_tire_rad": nominal_tire_rad, "target_tire_rad": target,
        "stamp_ns": stamp_ns, "dt_s": dt_s, "reset": reset,
        "previous_state": asdict(previous) if previous is not None else None,
        "next_state": asdict(candidate) if candidate is not None else None,
        "raw_rate_radps": raw_rate, "filtered_rate_radps": filtered_rate,
        "lead_s": .09 if active else 0., "rate_filter_s": .05 if active else 0.,
        "requested_correction_rad": correction, "applied_correction_rad": target - nominal_tire_rad,
        "headroom_limited": abs(target - (nominal_tire_rad + correction)) > 1e-12,
    }
