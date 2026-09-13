"""Explicit AWSIM input-angle to physical tire-angle contract, in radians.

The calibrated profile is specific to the recorded simulator assets. It changes
the command interface only; it does not change simulator physics or PP targets.
"""
from __future__ import annotations

import math
from typing import Any


CALIBRATED_POLICY = "awsim_grip_0p6_v1"
CALIBRATED_ASSETS = {
    "AWSIM_Data/StreamingAssets/Vehicle/vehicle.yaml": "5b66e58691091c82d5511535ca458d4c89e85f3b59fa0d0c4a2c7eee47887244",
    "AWSIM_Data/Managed/Assembly-CSharp.dll": "859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13",
}


def steering_response_gain(policy: str) -> float:
    """Steady tire angle / ROS input angle; reports already use tire radians."""
    if policy == "identity_v1":
        return 1.
    if policy == CALIBRATED_POLICY:
        return .6
    raise ValueError("TRIAL_STEERING_POLICY")


def steering_asset_contract(config: dict[str, Any]) -> dict[str, str]:
    """Return required immutable asset hashes; reject unspecified calibration."""
    policy = config.get("steering_policy", "identity_v1")
    steering_response_gain(policy)
    expected = CALIBRATED_ASSETS if policy == CALIBRATED_POLICY else {}
    if config.get("steering_asset_sha256", {}) != expected:
        raise ValueError("TRIAL_STEERING_ASSET_IDENTITY")
    return dict(expected)


def command_steering(required_tire_rad: float, previous_input_rad: float,
                     dt_s: float, *, policy: str = "identity_v1") -> dict[str, Any]:
    """Map PP's physical angle to rate-limited ROS input; all scalars in SI.

    Input magnitude remains <=0.5 rad and rate <=0.8 rad/s. A tire-angle
    request beyond the calibrated reachable limit is rejected, never silently
    clipped. The returned tire targets are for the physical stopping sweep.
    """
    gain = steering_response_gain(policy)
    if (not all(math.isfinite(v) for v in (required_tire_rad, previous_input_rad, dt_s))
            or abs(previous_input_rad) > .5 or not 0. <= dt_s <= .1):
        raise ValueError("STEERING_ACTUATOR_CONTRACT")
    if abs(required_tire_rad) > .5 * gain:
        raise ValueError("STEERING_ACTUATOR_INFEASIBLE")
    requested_input = required_tire_rad / gain
    issued_input = max(previous_input_rad - .8*dt_s, min(previous_input_rad + .8*dt_s, requested_input))
    return {"policy": policy, "response_gain": gain, "dt_s": dt_s,
            "required_tire_rad": required_tire_rad, "requested_input_rad": requested_input,
            "previous_input_rad": previous_input_rad, "issued_input_rad": issued_input,
            "issued_tire_target_rad": gain * issued_input,
            "previous_tire_target_rad": gain * previous_input_rad,
            "maximum_reachable_tire_rad": .5 * gain}
