"""Finite AWSIM speed policies over an unmodified 30-point time prediction."""
from __future__ import annotations

from dataclasses import replace
import math
from typing import Any, Sequence

import numpy as np

from .time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference, reference_control
from .time_geometry_v2 import validate_time_geometry
from .awsim_steering import CALIBRATED_POLICIES, steering_asset_contract
from .waypoint_controller import ControllerConfig, select_lookahead, control_from_waypoints
from ..runtime.awsim_trial_session import trial_duration_limits


SPEED_POLICIES = ("source_capped_0p25", "fixed_5kmh")
LOOKAHEAD_POLICIES = ("fixed_1m_v1", "feasible_1_to_1p5m_v1", "stopping_preview_v1")


def trial_speed_limits(speed_policy: str) -> tuple[float, float]:
    """Return (normal target ceiling, measured overspeed limit), both m/s."""
    if speed_policy == "source_capped_0p25":
        return .25, .45
    if speed_policy == "fixed_5kmh":
        return 5. / 3.6, 6. / 3.6
    raise ValueError("TRIAL_SPEED_POLICY")


def validate_trial_config(config: dict[str, Any]) -> str:
    """Reject descriptive JSON settings that disagree with this bounded runtime."""
    policy = config.get("speed_policy", "source_capped_0p25")
    if type(config.get("record_vehicle_motion", False)) is not bool:
        raise ValueError("TRIAL_MOTION_RECORDING_FLAG")
    steering_asset_contract(config)
    lookahead_policy = config.get("lookahead_policy", "fixed_1m_v1")
    if lookahead_policy not in LOOKAHEAD_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_POLICY")
    if lookahead_policy != "fixed_1m_v1" and config.get("steering_policy") not in CALIBRATED_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_REQUIRES_CALIBRATION")
    if config.get("obstacle_policy", "straight_v1") not in ("straight_v1", "steering_sweep_v1", "steering_support_v2"):
        raise ValueError("TRIAL_OBSTACLE_POLICY")
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
                       lookahead_policy: str = "fixed_1m_v1") -> dict[str, Any]:
    """SI units; age-aligned XY, explicit source-speed or fixed-5-km/h target.

    The static selected-scene rear axle differs by about 1 mm from base_link.
    A larger offset requires a future body-heading contract, not this trial.
    Reject geometry failures instead of cutting/fixing the predicted trajectory.
    """
    ceiling, overspeed = trial_speed_limits(speed_policy)
    if lookahead_policy not in LOOKAHEAD_POLICIES:
        raise ValueError("TRIAL_LOOKAHEAD_POLICY")
    if speed_cap_mps is None:
        speed_cap_mps = ceiling
    if (not np.isfinite([speed_mps, speed_cap_mps]).all() or not -.03 <= speed_mps <= overspeed
            or not 0 < speed_cap_mps <= ceiling
            or (speed_policy == "fixed_5kmh" and speed_cap_mps != ceiling)):
        raise ValueError("TRIAL_SPEED_CONTRACT")
    if len(rear_axle_offset_m) != 2 or not np.isfinite(rear_axle_offset_m).all() or np.linalg.norm(rear_axle_offset_m) > .002:
        raise ValueError("BODY_POINT_OFFSET_REQUIRES_FUTURE_HEADING")
    reference = prepare_time_reference(plan, current, rear_axle_offset_m=rear_axle_offset_m)
    geometry = validate_time_geometry(plan.xy_m)
    predicted_speed = reference.target_speed_mps
    target_speed = ceiling if speed_policy == "fixed_5kmh" else min(speed_cap_mps, predicted_speed)
    if (predicted_speed > 1e-6 or speed_policy == "fixed_5kmh") and not geometry["motion_resolved"]:
        raise ValueError("TIME_PATH_MOTION_UNRESOLVED")
    reference = replace(reference, target_speed_mps=target_speed)
    config = ControllerConfig(wheelbase_m=1.087, min_lookahead_m=1., max_steer_rad=.5,
                              min_accel_mps2=-1., max_accel_mps2=1., speed_kp=4.)
    target = np.zeros(2)
    minimum_preview = (max(1., .4+max(0., speed_mps)*.5+speed_mps**2/2)
                       if lookahead_policy == "stopping_preview_v1" else 1.)
    if target_speed > 1e-6:
        points = reference.xy_current_m[1:]
        forward = points[points[:, 0] > 1e-6]
        if not len(forward):
            raise ValueError("NO_FORWARD_REFERENCE")
        target = select_lookahead(forward, config.min_lookahead_m)
        if lookahead_policy != "fixed_1m_v1":
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
        if np.linalg.norm(forward[-1]) < .1 + max(0., speed_mps) * .5 + speed_mps ** 2 / 2:
            raise ValueError("REFERENCE_STOPPING_DISTANCE")
    if lookahead_policy != "fixed_1m_v1" and target_speed > 1e-6:
        command = control_from_waypoints(np.asarray([target]), target_speed, max(0., speed_mps), config)
    else:
        command = reference_control(reference, current_speed_mps=max(0., speed_mps), config=config)
    return {"steer_rad": command.steering_rad, "acceleration_mps2": command.acceleration_mps2,
            "target_speed_mps": target_speed, "predicted_source_speed_mps": predicted_speed,
            "speed_policy": speed_policy, "overspeed_limit_mps": overspeed,
            "lookahead_policy": lookahead_policy, "selected_lookahead_distance_m": float(np.linalg.norm(target)),
            "minimum_preview_distance_m": minimum_preview,
            "plan_age_sec": reference.age_sec, "lookahead_rear_m": target.tolist(),
            "reference_xy_rear_m": reference.xy_current_m.tolist(),
            "source_body_frame": reference.source_body_frame, "tracking_frame": reference.tracking_frame,
            "geometry": geometry}
