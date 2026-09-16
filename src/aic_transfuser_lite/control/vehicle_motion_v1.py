"""Shared physical-tire/body-curvature contract for finite AWSIM trials.

The experimental understeer family is fitted to a narrow low-speed population,
not a general vehicle calibration. Stopping containment is conditional on its
curvature interval and 0.03 m/s rear lateral-speed bound throughout braking.
Static wheelbase, speed-dependent response length and actuator gain are distinct.
"""
from __future__ import annotations

import math
from typing import Any

from .awsim_steering import CALIBRATED_POLICIES


IDEAL_POLICY = "ideal_bicycle_v1"
AWSIM_POLICY = "awsim_understeer_v1"
AWSIM_10KMH_POLICY = "awsim_understeer_10kmh_trial_v1"
AWSIM_POLICIES = (AWSIM_POLICY, AWSIM_10KMH_POLICY)
WHEELBASE_M = 1.087
MAX_SPEED_MPS = 6. / 3.6
TRIAL_10KMH_MAX_SPEED_MPS = 11. / 3.6
MAX_CURVATURE_PER_M = math.tan(.5) / WHEELBASE_M
NOMINAL_K_S2_PER_M = .045
MAX_K_S2_PER_M = .2
COM_FORWARD_OF_REAR_M = .17500001192092896
MAX_REAR_LATERAL_MPS = .03
SCENE_SHA256 = "9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b"


def vehicle_model_speed_limit(policy: str) -> float:
    """Finite measured-speed domain in m/s; the 10 km/h trial is extrapolated."""
    if policy == AWSIM_10KMH_POLICY:
        return TRIAL_10KMH_MAX_SPEED_MPS
    if policy not in (IDEAL_POLICY, AWSIM_POLICY):
        raise ValueError("VEHICLE_MODEL_POLICY")
    return MAX_SPEED_MPS


def effective_response_length(speed_mps: float, policy: str = IDEAL_POLICY) -> float:
    """Metres in tire=atan(curvature*length); speed in m/s, curvature in 1/m."""
    maximum = vehicle_model_speed_limit(policy)
    if not math.isfinite(speed_mps) or not 0 <= speed_mps <= maximum:
        raise ValueError("VEHICLE_MODEL_SPEED")
    return WHEELBASE_M + (NOMINAL_K_S2_PER_M*speed_mps**2 if policy in AWSIM_POLICIES else 0.)


def physical_tire_for_curvature(curvature_per_m: float, speed_mps: float,
                                policy: str = IDEAL_POLICY) -> float:
    """Inverse nominal body response. Physical tire angle in rad, no clamping."""
    length = effective_response_length(speed_mps, policy)
    if not math.isfinite(curvature_per_m):
        raise ValueError("VEHICLE_MODEL_CURVATURE")
    return math.atan(curvature_per_m*length)


def body_curvature_for_tire(tire_rad: float, speed_mps: float,
                            policy: str = IDEAL_POLICY) -> float:
    """Forward nominal body response (1/m), not ROS input steering."""
    length = effective_response_length(speed_mps, policy)
    if not math.isfinite(tire_rad) or abs(tire_rad) > .5:
        raise ValueError("VEHICLE_MODEL_TIRE")
    return math.tan(tire_rad)/length


def validate_vehicle_model_config(config: dict[str, Any]) -> str:
    policy = config.get("vehicle_model_policy", IDEAL_POLICY)
    effective_response_length(0., policy)
    if policy in AWSIM_POLICIES:
        geometry = config.get("geometry", {})
        speed_policy = "fixed_10kmh" if policy == AWSIM_10KMH_POLICY else "fixed_5kmh"
        if (config.get("steering_policy") not in CALIBRATED_POLICIES
                or config.get("obstacle_policy") != "steering_support_v2"
                or config.get("speed_policy") != speed_policy
                or geometry.get("wheelbase_m") != WHEELBASE_M
                or geometry.get("scene_sha256") != SCENE_SHA256):
            raise ValueError("VEHICLE_MODEL_ASSET_OR_POLICY_CONTRACT")
    if policy == AWSIM_10KMH_POLICY and (
            config.get("scope") != "BOUNDED_AWSIM_TRIAL_ONLY"
            or config.get("host") != "graneple@192.168.3.10"
            or config.get("execution_profile") != "one_lap"
            or config.get("record_vehicle_motion") is not True):
        raise ValueError("VEHICLE_MODEL_10KMH_TRIAL_SCOPE")
    return policy


def stopping_motion(speed_mps: float, measured_tire_rad: float, issued_tire_rad: float,
                    previous_tire_rad: float | None = None, *, policy: str = IDEAL_POLICY,
                    heading_rate_radps: float | None = None,
                    reported_lateral_mps: float | None = None) -> dict[str, Any]:
    """Bounds for forward travel [0,.4+v*.5+v²/2] in m, radians and seconds.

    Three physical angles remain in the interval. For AWSIM, union all speeds
    [0,v] and K in [0,.2] s²/m with measured yaw/v. At v<.2 m/s division is
    ill-conditioned, so use the full permitted curvature interval instead.
    VelocityReport is the CoM velocity in body axes in the pinned DLL/scene;
    transport its lateral component to the rear axle before checking it.
    """
    angles = [measured_tire_rad, issued_tire_rad,
              issued_tire_rad if previous_tire_rad is None else previous_tire_rad]
    if (not all(math.isfinite(x) for x in [speed_mps, *angles])
            or not -.03 <= speed_mps <= vehicle_model_speed_limit(policy) or max(map(abs, angles)) > .5):
        raise ValueError("SWEEP_VEHICLE_STATE")
    speed = max(0., speed_mps)
    length = effective_response_length(speed, policy)
    k = [math.tan(a)/WHEELBASE_M for a in angles]
    metadata: dict[str, Any] = {"policy": policy, "static_wheelbase_m": WHEELBASE_M,
        "nominal_response_length_m": length, "lateral_displacement_bound_m": 0.}
    if policy in AWSIM_POLICIES:
        if heading_rate_radps is None or reported_lateral_mps is None:
            raise ValueError("MOTION_MEASUREMENT_REQUIRED")
        if (not math.isfinite(heading_rate_radps)
                or abs(heading_rate_radps) > max(.2, speed)*MAX_CURVATURE_PER_M):
            raise ValueError("MOTION_YAW_RATE_INVALID")
        rear_lateral = reported_lateral_mps - COM_FORWARD_OF_REAR_M*heading_rate_radps
        if not math.isfinite(rear_lateral) or abs(rear_lateral) > MAX_REAR_LATERAL_MPS:
            raise ValueError("MOTION_REAR_LATERAL_INVALID")
        k += [math.tan(a)/(WHEELBASE_M+MAX_K_S2_PER_M*speed**2) for a in angles]
        measured_curvature = heading_rate_radps/speed if speed >= .2 else None
        k += ([measured_curvature] if measured_curvature is not None
              else [-MAX_CURVATURE_PER_M, MAX_CURVATURE_PER_M])
        metadata.update({"scope": "EMPIRICAL_AWSIM_ONLY_CONDITIONAL_BRAKING_BOUNDS",
            "gradient_interval_s2_per_m": [0., MAX_K_S2_PER_M],
            "braking_speed_interval_mps": [0., speed],
            "heading_rate_radps": heading_rate_radps, "reported_lateral_mps": reported_lateral_mps,
            "rear_lateral_mps": rear_lateral, "measured_curvature_per_m": measured_curvature,
            "low_speed_full_curvature_interval": speed < .2,
            "lateral_displacement_bound_m": MAX_REAR_LATERAL_MPS*(.5+speed)})
        if policy == AWSIM_10KMH_POLICY:
            metadata.update(scope="EXTRAPOLATED_10KMH_AWSIM_TRIAL_CONDITIONAL_BRAKING_BOUNDS",
                            calibrated_at_10kmh=False, maximum_trial_speed_mps=TRIAL_10KMH_MAX_SPEED_MPS)
    metadata["curvature_interval_per_m"] = [min(k), max(k)]
    return metadata
