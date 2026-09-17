"""Close real throttle after conversion, retaining steering and braking."""

from copy import deepcopy
import json
import math

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tier4_vehicle_msgs.msg import ActuationCommandStamped

from .geometry import yaw_from_quaternion
from .propulsion_guard import PropulsionGuard, PropulsionGuardConfig


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class RealActuationFilter(Node):
    def __init__(self):
        super().__init__('mppi_real_actuation_filter')
        self.guard = PropulsionGuard(PropulsionGuardConfig(
            float(self.declare_parameter('no_progress_timeout_sec', 0.4).value),
            float(self.declare_parameter('minimum_progress_m', 0.02).value)))
        self.timeout = float(self.declare_parameter('input_timeout_sec', 0.25).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('input_timeout_sec must be finite and positive')
        self.actuation = self.control = self.odometry = None
        self.autonomous = None
        self.gear = None
        self.mode_stamp = self.gear_stamp = -1
        self.direction_changed_ns = 0
        self.last_reason = None
        self.output_propelling = False
        self.pub = self.create_publisher(ActuationCommandStamped, 'output/actuation_cmd', 1)
        self.status_pub = self.create_publisher(String, 'debug/status', 10)
        self.create_subscription(ActuationCommandStamped, 'input/actuation_cmd', self.on_actuation, 1)
        self.create_subscription(AckermannControlCommand, 'input/control_cmd', self.on_control, 1)
        self.create_subscription(Odometry, 'input/odometry', self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(ControlModeReport, 'input/control_mode', self.on_mode, qos_profile_sensor_data)
        self.create_subscription(GearReport, 'input/gear_status', self.on_gear, qos_profile_sensor_data)
        self.create_timer(0.025, self.on_timer)

    def on_mode(self, message):
        stamp = stamp_ns(message.stamp)
        if stamp < self.mode_stamp:
            return
        self.mode_stamp = stamp
        autonomous = message.mode == ControlModeReport.AUTONOMOUS
        if autonomous != self.autonomous:
            self.guard.reset()
            self.direction_changed_ns = max(self.direction_changed_ns, stamp)
        self.autonomous = autonomous
        self.publish_filtered(False)

    def on_gear(self, message):
        stamp = stamp_ns(message.stamp)
        if stamp < self.gear_stamp:
            return
        self.gear_stamp = stamp
        if message.report != self.gear:
            self.direction_changed_ns = max(self.direction_changed_ns, stamp)
        self.gear = message.report
        self.publish_filtered(False)

    def on_odometry(self, message):
        if self.odometry is None or stamp_ns(message.header.stamp) >= stamp_ns(self.odometry.header.stamp):
            self.odometry = message

    def on_control(self, message):
        if self.control is None or stamp_ns(message.stamp) >= stamp_ns(self.control.stamp):
            self.control = message
        self.publish_filtered(False)

    def on_actuation(self, message):
        if self.actuation is not None and stamp_ns(message.header.stamp) < stamp_ns(self.actuation.header.stamp):
            return
        self.actuation = message
        self.publish_filtered(True)

    def on_timer(self):
        self.publish_filtered(False)

    def publish_filtered(self, allow_positive):
        if self.actuation is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        direction = {GearReport.DRIVE: 1, GearReport.REVERSE: -1}.get(self.gear, 0)
        stamps = [stamp_ns(self.actuation.header.stamp)]
        if self.control is not None:
            stamps.append(stamp_ns(self.control.stamp))
        if self.odometry is not None:
            stamps.append(stamp_ns(self.odometry.header.stamp))
        fresh = len(stamps) == 3 and all(0 <= now_ns - s <= self.timeout * 1e9 for s in stamps)
        reason = 'tracking'
        if self.autonomous is not True:
            reason = 'not_autonomous'
        elif not direction:
            reason = 'gear_not_confirmed'
        elif not fresh:
            reason = 'input_stale'
        elif min(stamps[:2]) < self.direction_changed_ns:
            reason = 'waiting_command_after_gear_change'
        elif (not math.isfinite(self.control.longitudinal.speed) or
              not math.isfinite(self.actuation.actuation.accel_cmd)):
            reason = 'invalid_command'
        elif self.control.longitudinal.speed <= 0.0:
            # The converter may map zero acceleration to nonzero throttle.
            # A stop or steering-settle command must still close the throttle.
            reason = 'stop_command'
        x = y = yaw = 0.0
        if self.odometry is not None:
            pose = self.odometry.pose.pose
            x, y, yaw = pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            reason = 'invalid_odometry'
        propelling = (reason == 'tracking' and self.actuation.actuation.accel_cmd > 0.0
                      and (allow_positive or self.output_propelling))
        blocked = self.guard.update(now_ns * 1e-9, direction, x, y, yaw, propelling)
        if reason == 'tracking' and blocked:
            reason = 'no_progress'
        cut = reason != 'tracking'
        if allow_positive or cut:
            output = deepcopy(self.actuation)
            if cut:
                output.header.stamp = now.to_msg()
                output.actuation.accel_cmd = 0.0
            self.pub.publish(output)
            self.output_propelling = output.actuation.accel_cmd > 0.0
        if reason != self.last_reason:
            self.status_pub.publish(String(data=json.dumps({
                'stamp': now_ns * 1e-9, 'reason': reason,
                'blocked_directions': sorted(self.guard.blocked_directions),
                'direction': direction, 'requested_throttle': self.actuation.actuation.accel_cmd,
                'published_throttle': 0.0 if cut else self.actuation.actuation.accel_cmd,
                'exposure_sec': self.guard.exposure.get(direction, 0.0)})))
            self.get_logger().info(f'[MPPI_REAL_PROPULSION] reason={reason} direction={direction}')
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = RealActuationFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.autonomous = False
        node.publish_filtered(False)
        node.destroy_node()
        rclpy.shutdown()
