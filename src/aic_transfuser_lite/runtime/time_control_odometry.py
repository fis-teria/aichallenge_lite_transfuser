"""Wheel-speed/steering pose for short-horizon TimePath control, without ROS.

No global initialization, GNSS, IMU, reported lateral velocity or reported yaw
rate enters this estimator. This is dead reckoning, not LiDAR SLAM. The pinned
vehicle response model is a nominal estimate; slip is not measured by it.
"""
from __future__ import annotations

import math
from collections import deque

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.vehicle_motion_v1 import (
    IDEAL_POLICY, WHEELBASE_M, body_curvature_for_tire,
)
from .local_odometry_v4 import LocalOdometry
from .steering_odometry_v4 import SteeringOdometry


CONTROL_ODOMETRY_FRAME = "time_wheel_odom"
CONTROL_ODOMETRY_POLICY = "wheel_speed_steering_v1"


class ClockAlignedControlInputs:
    """Bounded callback queue; a sensor may arrive before its /clock message.

    Entries retain capture ns and wall receipt ns. No entry is released outside
    the controller's [-20 ms, +150 ms] capture-age window or after 300 ms wall.
    A future entry waits, never gets restamped or integrated early.
    """

    def __init__(self) -> None:
        self.pending: deque[tuple[str, int, float, int]] = deque()

    def clear(self) -> None:
        self.pending.clear()

    def add(self, role: str, captured_ns: int, value: float, receipt_ns: int) -> None:
        if (role not in ('velocity', 'steering') or type(captured_ns) is not int or captured_ns < 0
                or type(receipt_ns) is not int or receipt_ns < 0 or not math.isfinite(value)):
            raise ValueError('CONTROL_ODOMETRY_INPUT')
        if len(self.pending) >= 32:
            raise ValueError('CONTROL_ODOMETRY_INPUT_OVERFLOW')
        self.pending.append((role, captured_ns, value, receipt_ns))

    def ready(self, clock_ns: int, wall_ns: int) -> list[tuple[str, int, float]]:
        if type(clock_ns) is not int or clock_ns < 0 or type(wall_ns) is not int or wall_ns < 0:
            raise ValueError('CONTROL_ODOMETRY_CLOCK')
        result = []
        # Keep arrival order, including within each sensor stream. Reordered or
        # conflicting captures still reach the odometry's latched-fault checks.
        while self.pending:
            role, captured, value, receipt = self.pending[0]
            if not 0 <= wall_ns-receipt <= 300_000_000 or clock_ns-captured > 150_000_000:
                raise ValueError('CONTROL_ODOMETRY_INPUT_STALE')
            if captured-clock_ns > 20_000_000:
                break
            self.pending.popleft()
            result.append((role, captured, value))
        return result


class TimeControlOdometry:
    """Join source stamps in ns; integrate speed [m/s], physical steering [rad].

    Speed is longitudinal speed along the vehicle centreline. Rear axle has
    zero nominal lateral speed. Returned poses describe base_link, with its
    explicit rear-axle offset [m], in a first-sample local frame. No TF lookup.
    """

    def __init__(self, rear_axle_forward_m: float, vehicle_model_policy: str = IDEAL_POLICY) -> None:
        if not math.isfinite(rear_axle_forward_m) or abs(rear_axle_forward_m) > .002:
            raise ValueError("UNSUPPORTED_BODY_POINT_CALIBRATION")
        body_curvature_for_tire(0., 0., vehicle_model_policy)
        self.rear_axle_forward_m = rear_axle_forward_m
        self.vehicle_model_policy = vehicle_model_policy
        self.join = SteeringOdometry(wheelbase_m=WHEELBASE_M, reference_left_offset_m=0., max_gap_s=.15)
        self.integration = LocalOdometry(max_gap_s=.15, max_speed_mps=10., max_yaw_rate_rps=2.)
        self.fault: str | None = None

    def reset(self) -> None:
        self.join.reset()
        self.integration.reset()
        self.fault = None

    def add_steering(self, stamp_ns: int, physical_tire_rad: float) -> None:
        if self.fault:
            raise ValueError(self.fault)
        try:
            if not math.isfinite(physical_tire_rad) or abs(physical_tire_rad) > .5:
                raise ValueError("ODOMETRY_STEERING_RANGE")
            self.join.add_steering(stamp_ns, physical_tire_rad)
        except ValueError as exc:
            self.fault = str(exc)
            raise

    def add_speed(self, stamp_ns: int, speed_mps: float) -> None:
        if self.fault:
            raise ValueError(self.fault)
        try:
            if not math.isfinite(speed_mps) or speed_mps < -.03:
                raise ValueError("ODOMETRY_SPEED_RANGE")
            body_curvature_for_tire(0., max(0., speed_mps), self.vehicle_model_policy)
            # SteeringOdometry supplies bounded joining only. Its ideal-bicycle
            # pose is not consumed; integrate the configured response below.
            self.join.add_velocity(stamp_ns, speed_mps, 0., 0.)
        except ValueError as exc:
            self.fault = str(exc)
            raise

    def drain(self, now_ns: int, epoch: str) -> list[tuple[TimedBodyPose, dict]]:
        if self.fault:
            raise ValueError(self.fault)
        if type(now_ns) is not int or now_ns < 0 or not isinstance(epoch, str) or not epoch:
            self.fault = "ODOMETRY_CLOCK"
            raise ValueError(self.fault)
        result = []
        try:
            for _, joined in self.join.drain(now_ns):
                if joined['stamp_ns'] > now_ns + 20_000_000:
                    raise ValueError("ODOMETRY_FUTURE_STAMP")
                speed = joined['vx_mps']
                yaw_rate = speed * body_curvature_for_tire(
                    joined['steering_rad'], max(0., speed), self.vehicle_model_policy)
                rear = self.integration.update(joined['stamp_ns'], speed, 0., yaw_rate)
                if rear is None:
                    continue
                offset = self.rear_axle_forward_m
                pose = TimedBodyPose(rear.stamp_ns, 'sim', epoch, CONTROL_ODOMETRY_FRAME, 'base_link',
                    rear.x_m + offset*(1-math.cos(rear.yaw_rad)),
                    rear.y_m - offset*math.sin(rear.yaw_rad), rear.yaw_rad)
                result.append((pose, dict(policy=CONTROL_ODOMETRY_POLICY,
                    vehicle_model_policy=self.vehicle_model_policy, speed_mps=speed,
                    steering_rad=joined['steering_rad'], steering_stamps_ns=joined['steering_stamps_ns'],
                    estimated_yaw_rate_radps=yaw_rate, rear_axle_forward_m=offset)))
        except ValueError as exc:
            self.fault = str(exc)
            raise
        return result
