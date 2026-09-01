from __future__ import annotations

"""Trajectory-only V3 ROS adapter. This node never creates a control publisher."""

import math
from pathlib import Path
from typing import Any

from autoware_auto_vehicle_msgs.msg import SteeringReport
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray, String
import torch

from .canonical_source import prefer_canonical_source

prefer_canonical_source()

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.data.normalization import normalize_lidar_range_and_validity
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3
from aic_transfuser_lite.runtime.output_profiles import output_profile, validate_observation_timing

from .runtime_adapter import image_message_to_rgb, strict_message_stamp_to_seconds


class InferenceNodeV3(Node):
    """Strict-stamp, fail-closed trajectory-only V3 inference."""

    def __init__(self) -> None:
        super().__init__("aic_transfuser_inference_v3")
        parameters = {
            "model_path": "", "artifact_manifest_path": "", "expected_checkpoint_sha256": "",
            "expected_manifest_sha256": "", "expected_contract_hash": "", "device": "auto",
            "input_timeout_sec": 0.35, "max_sensor_skew_ms": 30.0,
            "lidar_min_range_m": 0.0, "lidar_max_range_m": 25.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        if output_profile("trajectory_only").nominal_control_authority:
            raise RuntimeError("trajectory_only profile unexpectedly has control authority")
        requested = str(self.get_parameter("device").value)
        requested = ("cuda" if torch.cuda.is_available() else "cpu") if requested == "auto" else requested
        self.device = torch.device(requested)
        loaded = load_runtime_model_v3(
            Path(str(self.get_parameter("model_path").value)).expanduser(),
            Path(str(self.get_parameter("artifact_manifest_path").value)).expanduser(),
            device=self.device,
            expected_checkpoint_sha256=str(self.get_parameter("expected_checkpoint_sha256").value),
            expected_manifest_sha256=str(self.get_parameter("expected_manifest_sha256").value),
            expected_contract_hash=str(self.get_parameter("expected_contract_hash").value),
        )
        self.model = loaded.model
        self.model_kwargs = {
            "image_height": self.model.image_height, "image_width": self.model.image_width,
            "lidar_points": self.model.lidar_points, "ego_dim": self.model.ego_dim,
        }
        if self.model.ego_dim != 4:
            raise ValueError("trajectory runtime requires ego_dim=4: vx, vy, yaw_rate, steering")
        self.timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.max_skew_sec = float(self.get_parameter("max_sensor_skew_ms").value) / 1000.0
        self.latest: dict[str, Any] = {}
        self.trajectory_pub = self.create_publisher(Float32MultiArray, "predicted_trajectory", 1)
        self.status_pub = self.create_publisher(String, "runtime_status", 1)
        self.create_subscription(LaserScan, "scan", lambda msg: self._remember("lidar", msg), 10)
        self.create_subscription(Odometry, "odometry", lambda msg: self._remember("odometry", msg), 10)
        self.create_subscription(SteeringReport, "steering_status", lambda msg: self._remember("steering", msg), 10)
        self.create_subscription(Image, "image", self._on_image, 10)

    def _remember(self, role: str, message: Any) -> None:
        try:
            strict_message_stamp_to_seconds(message)
        except ValueError as error:
            self.status_pub.publish(String(data=f"invalid_{role}_timestamp:{error}"))
            return
        self.latest[role] = message

    def _on_image(self, image: Image) -> None:
        try:
            missing = sorted({"lidar", "odometry", "steering"}.difference(self.latest))
            if missing:
                raise ValueError("missing:" + ",".join(missing))
            camera_stamp = strict_message_stamp_to_seconds(image)
            stamps = {name: strict_message_stamp_to_seconds(msg) for name, msg in self.latest.items()}
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            validate_observation_timing(
                now_sec=now_sec, camera_stamp_sec=camera_stamp, role_stamps_sec=stamps,
                timeout_sec=self.timeout_sec, max_skew_sec=self.max_skew_sec,
            )
            batch = self._make_batch(image, self.latest["lidar"], self.latest["odometry"], self.latest["steering"], stamps)
            with torch.inference_mode():
                output = self.model(batch)
            xy = output.trajectory_xy[0, 0].detach().cpu().reshape(-1).tolist()
            self.trajectory_pub.publish(Float32MultiArray(data=xy))
            self.status_pub.publish(String(data="trajectory_published"))
        except Exception as error:
            self.status_pub.publish(String(data=f"inference_rejected:{error}"))

    def _make_batch(
        self, image: Image, scan: LaserScan, odometry: Odometry, steering: SteeringReport,
        stamps: dict[str, float],
    ) -> ModelBatchV3:
        rgb = image_message_to_rgb(image)
        image_tensor = preprocess_image(
            rgb, height=self.model.image_height, width=self.model.image_width
        ).to(self.device)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        if ranges.shape != (self.model.lidar_points,):
            raise ValueError(f"lidar_point_count:{ranges.size}")
        valid = np.isfinite(ranges) & (ranges >= float(scan.range_min)) & (ranges <= float(scan.range_max))
        if not bool(valid.any()):
            raise ValueError("lidar_zero_valid_beams")
        lidar = torch.from_numpy(normalize_lidar_range_and_validity(
            ranges, valid,
            min_range_m=float(self.get_parameter("lidar_min_range_m").value),
            max_range_m=float(self.get_parameter("lidar_max_range_m").value),
        )).to(self.device)
        twist = odometry.twist.twist
        ego = torch.tensor([
            float(twist.linear.x), float(twist.linear.y), float(twist.angular.z),
            float(steering.steering_tire_angle),
        ], dtype=torch.float32, device=self.device)
        if not torch.isfinite(ego).all():
            raise ValueError("non_finite_ego")
        dt = torch.tensor([[[stamps["lidar"] - stamps["odometry"], stamps["steering"] - stamps["odometry"]]]], device=self.device)
        return ModelBatchV3(
            image=image_tensor[None, None], image_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            lidar=lidar[None, None], lidar_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            ego=ego[None, None], ego_feature_mask=torch.ones(1, 1, 4, dtype=torch.bool, device=self.device),
            command_history=torch.zeros(1, 1, 3, device=self.device),
            command_mask=torch.zeros(1, 1, dtype=torch.bool, device=self.device),
            sensor_dt_sec=dt, requested_outputs=frozenset({"trajectory", "speed_profile"}),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InferenceNodeV3()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
