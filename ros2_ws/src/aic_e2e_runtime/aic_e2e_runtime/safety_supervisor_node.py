from __future__ import annotations

"""Independent safety supervisor for AWSIM control commands."""

import math

from autoware_auto_control_msgs.msg import AckermannControlCommand
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String

from aic_transfuser_lite.control.safety_supervisor import (
    SafetyConfig,
    SensorStamps,
    apply_safety,
    clamp_command_envelope,
)
from aic_transfuser_lite.control.waypoint_controller import ControlCommand

from .runtime_adapter import strict_message_stamp_to_seconds


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("aic_safety_supervisor")
        defaults = SafetyConfig()
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)
        self.declare_parameter("publish_hz", 20.0)
        self.config = SafetyConfig(
            **{
                name: type(value)(self.get_parameter(name).value)
                for name, value in defaults.__dict__.items()
            }
        )
        self.image_stamp_sec = -math.inf
        self.scan_stamp_sec = -math.inf
        self.ego_stamp_sec = -math.inf
        self.nominal_stamp_sec = -math.inf
        self.nominal_valid_until_sec = -math.inf
        self.scan: LaserScan | None = None
        self.speed_mps = 0.0
        self.stop_probability: float | None = None
        self.nominal = ControlCommand(0.0, self.config.min_accel_mps2)
        self.nominal_speed_mps = 0.0
        self.last_reason = "startup"

        self.create_subscription(Image, "image", self._on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odometry", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Float32, "stop_probability", self._on_stop, 1)
        self.create_subscription(AckermannControlCommand, "nominal_control_cmd", self._on_nominal, 1)
        self.control_pub = self.create_publisher(AckermannControlCommand, "control_cmd", 1)
        self.reason_pub = self.create_publisher(String, "safety_reason", 1)
        hz = float(self.get_parameter("publish_hz").value)
        if hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info(f"Safety supervisor ready at {hz:.1f} Hz")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_image(self, message: Image) -> None:
        try:
            self.image_stamp_sec = strict_message_stamp_to_seconds(message)
        except ValueError:
            self.image_stamp_sec = -math.inf

    def _on_scan(self, message: LaserScan) -> None:
        self.scan = message
        try:
            self.scan_stamp_sec = strict_message_stamp_to_seconds(message)
        except ValueError:
            self.scan_stamp_sec = -math.inf

    def _on_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        self.speed_mps = math.hypot(float(linear.x), float(linear.y))
        try:
            self.ego_stamp_sec = strict_message_stamp_to_seconds(message)
        except ValueError:
            self.ego_stamp_sec = -math.inf

    def _on_stop(self, message: Float32) -> None:
        self.stop_probability = float(message.data)

    def _on_nominal(self, message: AckermannControlCommand) -> None:
        now = self._now_sec()
        try:
            source_stamp = strict_message_stamp_to_seconds(message)
            envelope = clamp_command_envelope(
                proposed_speed_mps=float(message.longitudinal.speed),
                source_observation_stamp_sec=source_stamp,
                generated_stamp_sec=source_stamp,
                requested_valid_until_sec=source_stamp + self.config.nominal_timeout_sec,
                now_sec=now,
                config=self.config,
            )
            self.nominal_speed_mps = envelope.speed_mps
            self.nominal = ControlCommand(
                steering_rad=float(message.lateral.steering_tire_angle),
                acceleration_mps2=float(message.longitudinal.acceleration),
            )
            self.nominal_stamp_sec = source_stamp
            self.nominal_valid_until_sec = envelope.valid_until_sec
        except (ValueError, TimeoutError) as error:
            self.nominal_stamp_sec = -math.inf
            self.nominal_valid_until_sec = -math.inf
            self.last_reason = f"nominal_rejected:{error}"

    def _publish(
        self, command: ControlCommand, reason: str, *, commanded_speed_mps: float
    ) -> None:
        message = AckermannControlCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.longitudinal.speed = float(commanded_speed_mps)
        message.longitudinal.acceleration = float(command.acceleration_mps2)
        message.lateral.steering_tire_angle = float(command.steering_rad)
        self.control_pub.publish(message)
        self.reason_pub.publish(String(data=reason))
        if reason != self.last_reason:
            if reason == "normal":
                self.get_logger().info(f"Safety state: {reason}")
            else:
                self.get_logger().warning(f"Safety state: {reason}")
            self.last_reason = reason

    def _on_timer(self) -> None:
        now = self._now_sec()
        brake = ControlCommand(0.0, self.config.min_accel_mps2)
        if self.scan is None:
            self._publish(brake, "scan_missing", commanded_speed_mps=0.0)
            return
        if now > self.nominal_valid_until_sec:
            self._publish(brake, "nominal_command_timeout", commanded_speed_mps=0.0)
            return
        if not math.isfinite(self.nominal_speed_mps):
            self._publish(brake, "nominal_speed_non_finite", commanded_speed_mps=0.0)
            return
        try:
            decision = apply_safety(
                self.nominal,
                speed_mps=self.speed_mps,
                lidar_ranges_m=np.asarray(self.scan.ranges, dtype=np.float32),
                angle_min_rad=float(self.scan.angle_min),
                angle_increment_rad=float(self.scan.angle_increment),
                stop_probability=self.stop_probability,
                confidence=None,
                stamps=SensorStamps(
                    camera_sec=self.image_stamp_sec,
                    lidar_sec=self.scan_stamp_sec,
                    ego_sec=self.ego_stamp_sec,
                ),
                now_sec=now,
                config=self.config,
            )
        except Exception as error:
            self.get_logger().error(f"Safety evaluation failed: {error}")
            self._publish(brake, "safety_exception", commanded_speed_mps=0.0)
            return
        pass_through_speed = decision.reason in {"normal", "command_clamped"}
        self._publish(
            decision.command,
            decision.reason,
            commanded_speed_mps=(self.nominal_speed_mps if pass_through_speed else 0.0),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
