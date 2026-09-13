"""Finite low-speed AWSIM policy over an unmodified 30-point time prediction."""
from __future__ import annotations

from dataclasses import replace
import math
from typing import Any, Sequence

import numpy as np

from .time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference, reference_control
from .time_geometry_v2 import validate_time_geometry
from .waypoint_controller import ControllerConfig, select_lookahead


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
                       rear_axle_offset_m: tuple[float, float], speed_cap_mps: float = .25) -> dict[str, Any]:
    """SI units; age-aligned source speed, capped trial speed, bounded PP.

    The static selected-scene rear axle differs by about 1 mm from base_link.
    A larger offset requires a future body-heading contract, not this trial.
    Reject geometry failures instead of cutting/fixing the predicted trajectory.
    """
    if (not np.isfinite([speed_mps, speed_cap_mps]).all() or not -.03 <= speed_mps <= .45
            or not 0 < speed_cap_mps <= .25):
        raise ValueError("TRIAL_SPEED_CONTRACT")
    if len(rear_axle_offset_m) != 2 or not np.isfinite(rear_axle_offset_m).all() or np.linalg.norm(rear_axle_offset_m) > .002:
        raise ValueError("BODY_POINT_OFFSET_REQUIRES_FUTURE_HEADING")
    reference = prepare_time_reference(plan, current, rear_axle_offset_m=rear_axle_offset_m)
    geometry = validate_time_geometry(plan.xy_m)
    predicted_speed = reference.target_speed_mps
    if predicted_speed > 1e-6 and not geometry["motion_resolved"]:
        raise ValueError("TIME_PATH_MOTION_UNRESOLVED")
    target_speed = min(speed_cap_mps, predicted_speed)
    reference = replace(reference, target_speed_mps=target_speed)
    config = ControllerConfig(wheelbase_m=1.087, min_lookahead_m=1., max_steer_rad=.5,
                              min_accel_mps2=-1., max_accel_mps2=1., speed_kp=4.)
    target = np.zeros(2)
    if target_speed > 1e-6:
        points = reference.xy_current_m[1:]
        forward = points[points[:, 0] > 1e-6]
        if not len(forward):
            raise ValueError("NO_FORWARD_REFERENCE")
        target = select_lookahead(forward, config.min_lookahead_m)
        required = math.atan(2 * config.wheelbase_m * target[1] / max(float(target @ target), 1e-6))
        if abs(required) > config.max_steer_rad:
            raise ValueError("STEERING_INFEASIBLE")
        if np.linalg.norm(forward[-1]) < .1 + max(0., speed_mps) * .5 + speed_mps ** 2 / 2:
            raise ValueError("REFERENCE_STOPPING_DISTANCE")
    command = reference_control(reference, current_speed_mps=max(0., speed_mps), config=config)
    return {"steer_rad": command.steering_rad, "acceleration_mps2": command.acceleration_mps2,
            "target_speed_mps": target_speed, "predicted_source_speed_mps": predicted_speed,
            "plan_age_sec": reference.age_sec, "lookahead_rear_m": target.tolist(),
            "reference_xy_rear_m": reference.xy_current_m.tolist(),
            "source_body_frame": reference.source_body_frame, "tracking_frame": reference.tracking_frame,
            "geometry": geometry}
