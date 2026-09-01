from __future__ import annotations

"""AWSIM-facing TransFuser Lite inference node."""

from pathlib import Path
import math
import time

from autoware_auto_control_msgs.msg import AckermannControlCommand
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, Int32
import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.control.waypoint_controller import (
    ControllerConfig,
    control_from_waypoints,
)
from aic_transfuser_lite.data.ego_features import configured_ego_features
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.data.lidar_preprocess import (
    LidarPreprocessConfig,
    sanitize_lidar,
)
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.runtime.inference_core import infer

from .runtime_adapter import (
    image_message_to_rgb,
    odometry_to_ego,
    resample_laser_ranges,
    stamp_to_seconds,
)


class InferenceNode(Node):
    """Run batch-size-one inference from AWSIM sensor topics.

    Input units are metres, seconds, radians and metres/second. Model tensors are
    image ``[1,3,H,W]``, lidar ``[1,P]`` and config-defined ego ``[1,D]``.
    """

    def __init__(self) -> None:
        super().__init__("aic_transfuser_inference")
        self.declare_parameter("model_path", "")
        self.declare_parameter("config_path", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("inference_hz", 10.0)
        self.declare_parameter("input_timeout_sec", 0.35)
        self.declare_parameter("wheelbase_m", 1.087)
        self.declare_parameter("min_lookahead_m", 1.0)
        self.declare_parameter("max_steer_rad", 0.6)
        self.declare_parameter("min_accel_mps2", -4.0)
        self.declare_parameter("max_accel_mps2", 2.0)
        self.declare_parameter("speed_kp", 1.0)

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"model_path does not exist: {model_path}")
        checkpoint = torch.load(model_path, map_location="cpu")
        if not isinstance(checkpoint, dict) or "model" not in checkpoint:
            raise ValueError("checkpoint must contain a 'model' state dictionary")
        config_path = str(self.get_parameter("config_path").value).strip()
        self.config = load_config(config_path) if config_path else checkpoint.get("config")
        if not isinstance(self.config, dict):
            raise ValueError("checkpoint has no config; set config_path")

        requested_device = str(self.get_parameter("device").value)
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(requested_device)
        self.model = build_model(self.config).to(self.device)
        self.model.load_state_dict(checkpoint["model"], strict=True)
        self.model.eval()

        data = self.config["data"]
        self.image_height = int(data["image_height"])
        self.image_width = int(data["image_width"])
        self.lidar_points = int(data["lidar_points"])
        self.ego_features = configured_ego_features(data)
        self.lidar_config = LidarPreprocessConfig(
            min_range_m=float(data.get("lidar_min_range_m", 0.05)),
            max_range_m=float(data.get("lidar_max_range_m", 30.0)),
        )
        self.model_name = str(self.config["model"]["name"])
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.controller_config = ControllerConfig(
            wheelbase_m=float(self.get_parameter("wheelbase_m").value),
            min_lookahead_m=float(self.get_parameter("min_lookahead_m").value),
            max_steer_rad=float(self.get_parameter("max_steer_rad").value),
            min_accel_mps2=float(self.get_parameter("min_accel_mps2").value),
            max_accel_mps2=float(self.get_parameter("max_accel_mps2").value),
            speed_kp=float(self.get_parameter("speed_kp").value),
        )

        self.image: Image | None = None
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.last_steering_rad = 0.0
        self.last_warning_sec = -math.inf

        self.create_subscription(Image, "image", self._on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odometry", self._on_odom, qos_profile_sensor_data)
        self.command_pub = self.create_publisher(AckermannControlCommand, "nominal_control_cmd", 1)
        self.stop_pub = self.create_publisher(Float32, "stop_probability", 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, "predicted_waypoints", 1)
        self.mode_pub = self.create_publisher(Int32, "behavior_mode", 1)

        hz = float(self.get_parameter("inference_hz").value)
        if hz <= 0.0:
            raise ValueError("inference_hz must be positive")
        self.timer = self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info(
            f"Loaded {model_path} on {self.device}; image={self.image_width}x{self.image_height}, "
            f"lidar={self.lidar_points}, ego={self.ego_features}, "
            f"rate={hz:.1f} Hz"
        )

    def _on_image(self, message: Image) -> None:
        self.image = message

    def _on_scan(self, message: LaserScan) -> None:
        self.scan = message

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _inputs_ready(self, now_sec: float) -> bool:
        messages = (self.image, self.scan, self.odom)
        if any(message is None for message in messages):
            self._warn_throttled("Waiting for image, scan and odometry")
            return False
        assert self.image is not None and self.scan is not None and self.odom is not None
        ages = {
            "image": now_sec - stamp_to_seconds(self.image.header.stamp, now_sec),
            "scan": now_sec - stamp_to_seconds(self.scan.header.stamp, now_sec),
            "odometry": now_sec - stamp_to_seconds(self.odom.header.stamp, now_sec),
        }
        stale = {name: age for name, age in ages.items() if age > self.input_timeout_sec or age < -0.1}
        if stale:
            self._warn_throttled(f"Stale inputs: {stale}")
            return False
        return True

    def _warn_throttled(self, text: str) -> None:
        now = self._now_sec()
        if now - self.last_warning_sec >= 2.0:
            self.get_logger().warning(text)
            self.last_warning_sec = now

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        if not self._inputs_ready(now_sec):
            return
        assert self.image is not None and self.scan is not None and self.odom is not None
        started = time.perf_counter()
        try:
            rgb = image_message_to_rgb(self.image)
            image = preprocess_image(
                rgb, height=self.image_height, width=self.image_width
            ).unsqueeze(0).to(self.device)
            raw_lidar = resample_laser_ranges(
                np.asarray(self.scan.ranges, dtype=np.float32), self.lidar_points
            )
            normalized_lidar, _ = sanitize_lidar(raw_lidar, self.lidar_config)
            lidar = torch.from_numpy(normalized_lidar).unsqueeze(0).to(self.device)
            ego_array, speed_mps = odometry_to_ego(
                self.odom,
                previous_steering_rad=self.last_steering_rad,
                ego_features=self.ego_features,
            )
            ego = torch.from_numpy(ego_array).unsqueeze(0).to(self.device)
            output = infer(
                self.model,
                image=image,
                lidar=lidar,
                ego=ego,
                model_name=self.model_name,
            )
            waypoints = output["waypoints"][0].numpy().astype(np.float32)
            target_speed_mps = float(output["target_speed"][0, 0])
            stop_probability = (
                float(output["stop_probability"][0, 0])
                if "stop_probability" in output
                else None
            )
            behavior_mode = (
                int(torch.argmax(output["mode_logits"][0]).item())
                if "mode_logits" in output
                else None
            )
            command = control_from_waypoints(
                waypoints,
                target_speed_mps,
                speed_mps,
                self.controller_config,
            )
            finite_values = [command.steering_rad, command.acceleration_mps2]
            if stop_probability is not None:
                finite_values.append(stop_probability)
            if not all(math.isfinite(value) for value in finite_values):
                raise ValueError("model produced a non-finite control value")
        except Exception as error:
            self.get_logger().error(f"Inference failed: {error}")
            return

        self.last_steering_rad = command.steering_rad
        if stop_probability is not None:
            self.stop_pub.publish(Float32(data=stop_probability))
        self.waypoint_pub.publish(Float32MultiArray(data=waypoints.reshape(-1).tolist()))
        if behavior_mode is not None:
            self.mode_pub.publish(Int32(data=behavior_mode))
        message = AckermannControlCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.longitudinal.acceleration = command.acceleration_mps2
        message.lateral.steering_tire_angle = command.steering_rad
        self.command_pub.publish(message)
        latency_ms = (time.perf_counter() - started) * 1000.0
        if latency_ms > 100.0:
            self._warn_throttled(f"Inference deadline miss: {latency_ms:.1f} ms")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
