"""Finite AWSIM speed policies over an unmodified 30-point time prediction."""
from __future__ import annotations

from dataclasses import replace
import math
from typing import Any, Sequence

import numpy as np

from .time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference, reference_control
from .time_geometry_v2 import validate_time_geometry
from .awsim_steering import CALIBRATED_POLICIES, steering_asset_contract
from .vehicle_motion_v1 import IDEAL_POLICY, AWSIM_FIXED_TRIAL_SPEED_POLICIES, AWSIM_TRIAL_SPEED_POLICIES, AWSIM_TRIAL_TARGETS_KMH, WHEELBASE_M, effective_response_length, validate_vehicle_model_config
from .curvature_speed_v1 import ADAPTIVE_SPEED_POLICIES, TIME_ADAPTIVE_SPEED_POLICIES, preview_speed_limit
from .time_preview_v1 import TIME_LOOKAHEAD_POLICY, time_preview_distance
from .waypoint_controller import ControllerConfig, select_lookahead, control_from_waypoints
from .polyline_lookahead_v1 import select_polyline_lookahead
from .curvature_support_v2 import ACTUAL_STOPPING_SPEED, DIAGNOSTIC_STOPPING_POLICIES, STANDARD_CLEARANCE, NEAR_LIMIT_CLEARANCE, ONE_METRE_STOPPING_TRAVEL, SCAN_STOP_POLICY, SCAN_LOG_ONLY_POLICY, clearance_dimensions
from ..runtime.awsim_trial_session import trial_duration_limits


FIXED_SPEED_POLICIES = ("fixed_5kmh", *AWSIM_FIXED_TRIAL_SPEED_POLICIES)
SPEED_POLICIES = ("source_capped_0p25", *FIXED_SPEED_POLICIES, *ADAPTIVE_SPEED_POLICIES)
SEGMENT_LOOKAHEAD_POLICY = "stopping_preview_segment_v1"
EXTENDED_LOOKAHEAD_POLICY = "stopping_preview_extended_v1"
LOOKAHEAD_POLICIES = ("fixed_1m_v1", "feasible_1_to_1p5m_v1", "stopping_preview_v1",
                     SEGMENT_LOOKAHEAD_POLICY, EXTENDED_LOOKAHEAD_POLICY, TIME_LOOKAHEAD_POLICY)


def trial_speed_limits(speed_policy: str) -> tuple[float, float]:
    """Return (normal target ceiling, measured overspeed limit), both m/s."""
    if speed_policy == "source_capped_0p25":
        return .25, .45
    if speed_policy == "fixed_5kmh":
        return 5. / 3.6, 6. / 3.6
    if speed_policy in AWSIM_TRIAL_SPEED_POLICIES:
        target = AWSIM_TRIAL_TARGETS_KMH[AWSIM_TRIAL_SPEED_POLICIES[speed_policy]]
        return target/3.6, (target+1)/3.6
    raise ValueError("TRIAL_SPEED_POLICY")


def validate_trial_config(config: dict[str, Any]) -> str:
    """Reject descriptive JSON settings that disagree with this bounded runtime."""
    policy = config.get("speed_policy", "source_capped_0p25")
    if type(config.get("record_vehicle_motion", False)) is not bool:
        raise ValueError("TRIAL_MOTION_RECORDING_FLAG")
    steering_asset_contract(config)
    validate_vehicle_model_config(config)
    if policy in AWSIM_TRIAL_SPEED_POLICIES and config.get("vehicle_model_policy") != AWSIM_TRIAL_SPEED_POLICIES[policy]:
        raise ValueError("TRIAL_SPEED_MODEL_CONTRACT")
    lookahead_policy = config.get("lookahead_policy", "fixed_1m_v1")
    if lookahead_policy not in LOOKAHEAD_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_POLICY")
    if (lookahead_policy == TIME_LOOKAHEAD_POLICY) != (policy in TIME_ADAPTIVE_SPEED_POLICIES):
        raise ValueError('TRIAL_TIME_PREVIEW_CONTRACT')
    if lookahead_policy != "fixed_1m_v1" and config.get("steering_policy") not in CALIBRATED_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_REQUIRES_CALIBRATION")
    if config.get("obstacle_policy", "straight_v1") not in ("straight_v1", "steering_sweep_v1", "steering_support_v2"):
        raise ValueError("TRIAL_OBSTACLE_POLICY")
    clearance_profile = config.get('diagnostic_clearance_profile', STANDARD_CLEARANCE)
    clearance_dimensions(clearance_profile)
    distance_policy = config.get('stopping_distance_policy', ACTUAL_STOPPING_SPEED)
    occupancy_policy = config.get('scan_occupancy_policy', SCAN_STOP_POLICY)
    if occupancy_policy not in (SCAN_STOP_POLICY, SCAN_LOG_ONLY_POLICY):
        raise ValueError('SCAN_OCCUPANCY_POLICY')
    if occupancy_policy == SCAN_LOG_ONLY_POLICY and (
            policy not in ADAPTIVE_SPEED_POLICIES or distance_policy != ONE_METRE_STOPPING_TRAVEL
            or clearance_profile != STANDARD_CLEARANCE):
        raise ValueError('SCAN_LOG_ONLY_REQUIRES_ADAPTIVE_DIAGNOSTIC')
    if distance_policy not in (ACTUAL_STOPPING_SPEED, *DIAGNOSTIC_STOPPING_POLICIES):
        raise ValueError('STOPPING_DISTANCE_POLICY')
    if distance_policy in DIAGNOSTIC_STOPPING_POLICIES and (
            config.get('diagnostic_only') is not True or type(config.get('maximum_diagnostic_trials')) is not int
            or config.get('maximum_diagnostic_trials') != 1 or policy not in AWSIM_TRIAL_SPEED_POLICIES
            or config.get('host') != 'graneple@192.168.3.10'
            or config.get('execution_profile') != 'one_lap'
            or config.get('obstacle_policy') != 'steering_support_v2'
            or config.get('vehicle_model_policy') != AWSIM_TRIAL_SPEED_POLICIES.get(policy)
            or clearance_profile != STANDARD_CLEARANCE):
        raise ValueError('FIVE_KMH_STOPPING_DIAGNOSTIC_SCOPE_REQUIRED')
    if clearance_profile == NEAR_LIMIT_CLEARANCE and (
            config.get('diagnostic_only') is not True or type(config.get('maximum_diagnostic_trials')) is not int
            or config.get('maximum_diagnostic_trials') != 1
            or config.get('host') != 'graneple@192.168.3.10'
            or config.get('execution_profile') != 'one_lap'
            or config.get('obstacle_policy') != 'steering_support_v2'
            or config.get('vehicle_model_policy') != 'awsim_understeer_v1'):
        raise ValueError('NEAR_LIMIT_DIAGNOSTIC_SCOPE_REQUIRED')
    ceiling, overspeed = trial_speed_limits(policy)
    drive_sim_s, drive_wall_s, outer_wall_s = trial_duration_limits(config.get("execution_profile", "bounded_10s"))
    expected = {"speed_cap_mps": ceiling, "overspeed_limit_mps": overspeed,
                "drive_limit_sim_s": drive_sim_s, "drive_limit_wall_s": drive_wall_s, "outer_limit_wall_s": outer_wall_s,
                "plan_max_age_s": .5, "controller_wall_period_s": .05,
                "stop_confirmation_speed_mps": .03, "stop_confirmation_duration_sim_s": 1.}
    for key, value in expected.items():
        actual = config.get(key)
        if type(actual) not in (int, float) or not math.isclose(actual, value, rel_tol=0., abs_tol=1e-12):
            raise ValueError("TRIAL_CONFIG_MISMATCH:" + key)
    if config.get("scope") != "BOUNDED_AWSIM_TRIAL_ONLY":
        raise ValueError("TRIAL_CONFIG_SCOPE")
    return policy


def interpolate_body_pose(poses: Sequence[TimedBodyPose], stamp_ns: int,
                          *, tolerance_ns: int = 50_000_000) -> TimedBodyPose:
    """Observation pose at its original capture, no nearest-pose relabeling."""
    rows = sorted(poses, key=lambda p: p.stamp_ns)
    for pose in rows:
        if pose.stamp_ns == stamp_ns:
            return pose
    for a, b in zip(rows, rows[1:]):
        if a.stamp_ns < stamp_ns < b.stamp_ns:
            if max(stamp_ns - a.stamp_ns, b.stamp_ns - stamp_ns) > tolerance_ns:
                break
            if (a.clock, a.epoch, a.world_frame, a.body_frame) != (b.clock, b.epoch, b.world_frame, b.body_frame):
                raise ValueError("POSE_ENDPOINT_IDENTITY")
            alpha = (stamp_ns - a.stamp_ns) / (b.stamp_ns - a.stamp_ns)
            yaw_delta = math.atan2(math.sin(b.yaw_rad - a.yaw_rad), math.cos(b.yaw_rad - a.yaw_rad))
            return replace(a, stamp_ns=stamp_ns, x_m=a.x_m + alpha * (b.x_m - a.x_m),
                           y_m=a.y_m + alpha * (b.y_m - a.y_m), yaw_rad=a.yaw_rad + alpha * yaw_delta)
    raise ValueError("OBSERVATION_POSE_MISSING")


def time_trial_control(plan: TimePlan, current: TimedBodyPose, *, speed_mps: float,
                       rear_axle_offset_m: tuple[float, float], speed_cap_mps: float | None = None,
                       speed_policy: str = "source_capped_0p25",
                       lookahead_policy: str = "fixed_1m_v1",
                       vehicle_model_policy: str = IDEAL_POLICY) -> dict[str, Any]:
    """SI units; age-aligned XY, explicit source-speed or fixed-speed target.

    The static selected-scene rear axle differs by about 1 mm from base_link.
    A larger offset requires a future body-heading contract, not this trial.
    Reject geometry failures instead of cutting/fixing the predicted trajectory.
    """
    ceiling, overspeed = trial_speed_limits(speed_policy)
    if (speed_policy in AWSIM_TRIAL_SPEED_POLICIES or vehicle_model_policy in AWSIM_TRIAL_TARGETS_KMH) and (
            AWSIM_TRIAL_SPEED_POLICIES.get(speed_policy) != vehicle_model_policy):
        raise ValueError("TRIAL_SPEED_MODEL_CONTRACT")
    if lookahead_policy not in LOOKAHEAD_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_POLICY")
    if (lookahead_policy == TIME_LOOKAHEAD_POLICY) != (speed_policy in TIME_ADAPTIVE_SPEED_POLICIES):
        raise ValueError('TRIAL_TIME_PREVIEW_CONTRACT')
    if speed_cap_mps is None:
        speed_cap_mps = ceiling
    if (not np.isfinite([speed_mps, speed_cap_mps]).all() or not -.03 <= speed_mps <= overspeed
            or not 0 < speed_cap_mps <= ceiling
            or (speed_policy in FIXED_SPEED_POLICIES and speed_cap_mps != ceiling)):
        raise ValueError("TRIAL_SPEED_CONTRACT")
    if len(rear_axle_offset_m) != 2 or not np.isfinite(rear_axle_offset_m).all() or np.linalg.norm(rear_axle_offset_m) > .002:
        raise ValueError("BODY_POINT_OFFSET_REQUIRES_FUTURE_HEADING")
    reference = prepare_time_reference(plan, current, rear_axle_offset_m=rear_axle_offset_m)
    geometry = validate_time_geometry(plan.xy_m)
    predicted_speed = reference.target_speed_mps
    target_speed = (speed_cap_mps if speed_policy in ADAPTIVE_SPEED_POLICIES else
                    ceiling if speed_policy in FIXED_SPEED_POLICIES else min(speed_cap_mps, predicted_speed))
    if (predicted_speed > 1e-6 or speed_policy in (*FIXED_SPEED_POLICIES, *ADAPTIVE_SPEED_POLICIES)) and not geometry["motion_resolved"]:
        raise ValueError("TIME_PATH_MOTION_UNRESOLVED")
    reference = replace(reference, target_speed_mps=target_speed)
    response_length = effective_response_length(max(0., speed_mps), vehicle_model_policy)
    config = ControllerConfig(wheelbase_m=response_length, min_lookahead_m=1., max_steer_rad=.5,
                              min_accel_mps2=-1., max_accel_mps2=1., speed_kp=4.)
    target = np.zeros(2)
    selection_details: dict[str, Any] = {}
    minimum_preview = (max(1., .4+max(0., speed_mps)*.5+speed_mps**2/2)
                       if lookahead_policy in ("stopping_preview_v1", SEGMENT_LOOKAHEAD_POLICY, EXTENDED_LOOKAHEAD_POLICY) else 1.)
    if lookahead_policy == TIME_LOOKAHEAD_POLICY:
        minimum_preview = time_preview_distance(max(0., speed_mps))
    if target_speed > 1e-6:
        points = reference.xy_current_m[1:]
        forward = points[points[:, 0] > 1e-6]
        if not len(forward):
            raise ValueError("NO_FORWARD_REFERENCE")
        target = select_lookahead(forward, config.min_lookahead_m)
        if lookahead_policy in (SEGMENT_LOOKAHEAD_POLICY, EXTENDED_LOOKAHEAD_POLICY, TIME_LOOKAHEAD_POLICY):
            maximum_preview = minimum_preview+.5
            try:
                selection = select_polyline_lookahead(reference.xy_current_m, reference.remaining_sec,
                    minimum_m=minimum_preview, maximum_m=maximum_preview,
                    response_length_m=response_length)
            except ValueError as exc:
                if (lookahead_policy not in (EXTENDED_LOOKAHEAD_POLICY, TIME_LOOKAHEAD_POLICY)
                        or str(exc) != 'STEERING_FEASIBLE_LOOKAHEAD_MISSING'):
                    raise
                # Search further on the original, age-aligned polyline only.
                # Required preview, physical tire limit and raw points stay fixed.
                maximum_preview = minimum_preview+1.
                selection = select_polyline_lookahead(reference.xy_current_m, reference.remaining_sec,
                    minimum_m=minimum_preview, maximum_m=maximum_preview,
                    response_length_m=response_length)
            if lookahead_policy in (EXTENDED_LOOKAHEAD_POLICY, TIME_LOOKAHEAD_POLICY):
                selection['search_band_m'] = [minimum_preview, maximum_preview]
                selection['extended_search'] = maximum_preview > minimum_preview+.5
            selection['observation_horizon_s'] = selection['remaining_s']+reference.age_sec
            selection['source_interval_s'] = [t+reference.age_sec for t in selection['reference_interval_s']]
            selection_details['lookahead_selection'] = selection
            target = np.asarray(selection['xy_m'], dtype=np.float32)
        elif lookahead_policy != "fixed_1m_v1":
            target = None
            # Use the same float32 target representation as existing PP. Keep
            # time order and original points, with a bounded 0.5 m search band.
            for point in np.asarray(forward, dtype=np.float32):
                x, y = map(float, point)
                squared = x*x + y*y
                angle = math.atan(config.wheelbase_m * (2*y / max(squared, 1e-6)))
                if minimum_preview <= math.sqrt(squared) <= minimum_preview+.5 and abs(angle) <= .3:
                    target = point
                    break
            if target is None:
                raise ValueError("STEERING_FEASIBLE_LOOKAHEAD_MISSING")
        required = math.atan(2 * config.wheelbase_m * target[1] / max(float(target @ target), 1e-6))
        if abs(required) > config.max_steer_rad:
            raise ValueError("STEERING_INFEASIBLE")
        if lookahead_policy == TIME_LOOKAHEAD_POLICY:
            if np.linalg.norm(forward[-1]) < minimum_preview:
                raise ValueError('REFERENCE_TIME_PREVIEW_DISTANCE')
        elif np.linalg.norm(forward[-1]) < .1 + max(0., speed_mps) * .5 + speed_mps ** 2 / 2:
            raise ValueError("REFERENCE_STOPPING_DISTANCE")
    if speed_policy in ADAPTIVE_SPEED_POLICIES:
        speed_plan = preview_speed_limit(reference.xy_current_m, measured_speed_mps=max(0., speed_mps),
            cruise_ceiling_mps=speed_cap_mps, policy=speed_policy,
            tracking_curvature_per_m=2*float(target[1])/max(float(target @ target), 1e-6))
        target_speed = speed_plan['target_speed_mps']
        reference = replace(reference, target_speed_mps=target_speed)
        config = replace(config, max_accel_mps2=speed_plan['acceleration_cap_mps2'],
                         speed_kp=speed_plan['config']['speed_gain_per_s'])
        selection_details['longitudinal_preview'] = speed_plan
    if lookahead_policy != "fixed_1m_v1" and target_speed > 1e-6:
        command = control_from_waypoints(np.asarray([target]), target_speed, max(0., speed_mps), config)
    else:
        command = reference_control(reference, current_speed_mps=max(0., speed_mps), config=config)
    return {"steer_rad": command.steering_rad, "acceleration_mps2": command.acceleration_mps2,
            "target_speed_mps": target_speed, "predicted_source_speed_mps": predicted_speed,
            "speed_policy": speed_policy, "overspeed_limit_mps": overspeed,
            "vehicle_model_policy": vehicle_model_policy, "static_wheelbase_m": WHEELBASE_M,
            "nominal_response_length_m": response_length,
            "lookahead_policy": lookahead_policy, "selected_lookahead_distance_m": float(np.linalg.norm(target)),
            "minimum_preview_distance_m": minimum_preview,
            "plan_age_sec": reference.age_sec, "lookahead_rear_m": target.tolist(),
            "reference_xy_rear_m": reference.xy_current_m.tolist(),
            "source_body_frame": reference.source_body_frame, "tracking_frame": reference.tracking_frame,
            "geometry": geometry, **selection_details}
