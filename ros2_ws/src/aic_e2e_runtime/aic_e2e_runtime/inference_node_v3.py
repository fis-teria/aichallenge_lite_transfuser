from __future__ import annotations

"""Trajectory-authority V3 ROS adapter with optional behavior diagnostics."""

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    Float64MultiArray,
    Int32,
    String,
)
import torch

from .canonical_source import prefer_canonical_source

prefer_canonical_source()

from aic_transfuser_lite.contracts.behavior_v1 import BEHAVIOR_ONTOLOGY_V1
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.data.normalization import normalize_lidar_range_and_validity
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3
from aic_transfuser_lite.runtime.behavior_decode_v1 import decode_behavior_logits_v1
from aic_transfuser_lite.runtime.output_profiles import (
    output_profile,
    runtime_clock_has_reached_observation,
    trajectory_speed_publication,
    validate_observation_timing,
)
from aic_transfuser_lite.runtime.sensor_sync import (
    SettledCameraSynchronizer,
    SyncDecision,
)

from .runtime_adapter import image_message_to_rgb, strict_message_stamp_to_seconds


@dataclass(frozen=True)
class ReadyObservation:
    image: Image
    scan: LaserScan
    velocity: VelocityReport
    steering: SteeringReport
    role_stamps_sec: dict[str, float]


class InferenceNodeV3(Node):
    """Strict-stamp, fail-closed V3 inference without a control publisher."""

    SYNC_ROLES = ("lidar", "velocity", "steering")

    def __init__(self) -> None:
        super().__init__("aic_transfuser_inference_v3")
        parameters = {
            "model_path": "", "artifact_manifest_path": "", "expected_checkpoint_sha256": "",
            "expected_manifest_sha256": "", "expected_contract_hash": "", "device": "auto",
            "input_timeout_sec": 0.35, "sync_queue_size": 10,
            "sync_clock_poll_sec": 0.005,
            "max_sensor_skew_ms": 30.0,
            "lidar_min_range_m": 0.0, "lidar_max_range_m": 25.0,
            "behavior_confidence_threshold": 0.5, "behavior_temperature": 1.0,
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
        self.behavior_enabled = "behavior" in loaded.capabilities
        if self.behavior_enabled and "behavior_side" not in loaded.capabilities:
            raise ValueError("behavior runtime capability requires behavior_side")
        self.behavior_threshold = float(self.get_parameter("behavior_confidence_threshold").value)
        self.behavior_temperature = float(self.get_parameter("behavior_temperature").value)
        # The pure decoder performs the authoritative finite/range validation.
        decode_behavior_logits_v1(
            [0.0] * 5,
            [0.0] * 3,
            confidence_threshold=self.behavior_threshold,
            temperature=self.behavior_temperature,
        )
        self.model_kwargs = {
            "image_height": self.model.image_height, "image_width": self.model.image_width,
            "lidar_points": self.model.lidar_points, "ego_dim": self.model.ego_dim,
        }
        if self.model.ego_dim != 4:
            raise ValueError(
                "trajectory runtime requires ego_dim=4: longitudinal_velocity, "
                "lateral_velocity, heading_rate, steering_tire_angle"
            )
        self.timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.max_skew_sec = float(self.get_parameter("max_sensor_skew_ms").value) / 1000.0
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_clock_poll_sec = float(self.get_parameter("sync_clock_poll_sec").value)
        if sync_clock_poll_sec <= 0.0:
            raise ValueError("sync_clock_poll_sec must be positive")
        self.synchronizer: SettledCameraSynchronizer[Image, Any] = (
            SettledCameraSynchronizer(
                required_roles=self.SYNC_ROLES,
                queue_size=sync_queue_size,
                max_skew_sec=self.max_skew_sec,
            )
        )
        self.ready_observations: deque[ReadyObservation] = deque()
        self.ready_queue_size = sync_queue_size
        self.trajectory_pub = self.create_publisher(Float32MultiArray, "predicted_trajectory", 1)
        self.speed_profile_pub = self.create_publisher(
            Float32MultiArray, "predicted_speed_profile", 1
        )
        self.status_pub = self.create_publisher(String, "runtime_status", 1)
        self.sync_debug_pub = self.create_publisher(
            Float64MultiArray, "runtime_sync_debug", 1
        )
        self.behavior_mode_pub = None
        self.behavior_label_pub = None
        self.behavior_confidence_pub = None
        self.behavior_side_pub = None
        if self.behavior_enabled:
            self.behavior_mode_pub = self.create_publisher(Int32, "behavior_mode", 1)
            self.behavior_label_pub = self.create_publisher(String, "behavior_label", 1)
            self.behavior_confidence_pub = self.create_publisher(
                Float32, "behavior_confidence", 1
            )
            self.behavior_side_pub = self.create_publisher(Int32, "behavior_side", 1)
        self.create_subscription(
            LaserScan,
            "scan",
            lambda msg: self._add_sensor("lidar", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VelocityReport,
            "velocity_status",
            lambda msg: self._add_sensor("velocity", msg),
            10,
        )
        self.create_subscription(
            SteeringReport,
            "steering_status",
            lambda msg: self._add_sensor("steering", msg),
            10,
        )
        self.create_subscription(
            Image,
            "image",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.sync_clock_timer = self.create_timer(
            sync_clock_poll_sec, self._drain_ready_observations
        )

    def _add_sensor(self, role: str, message: Any) -> None:
        try:
            self.synchronizer.add_sensor(
                role, strict_message_stamp_to_seconds(message), message
            )
        except ValueError as error:
            self.status_pub.publish(String(data=f"invalid_{role}_sample:{error}"))
            return
        self._drain_camera_queue()

    def _on_image(self, image: Image) -> None:
        try:
            camera_stamp = strict_message_stamp_to_seconds(image)
            dropped = self.synchronizer.add_camera(camera_stamp, image)
            if dropped is not None:
                self.status_pub.publish(
                    String(data=f"camera_sync_queue_overflow:{dropped.stamp_sec:.9f}")
                )
        except ValueError as error:
            self.status_pub.publish(String(data=f"invalid_camera_sample:{error}"))
            return
        self._drain_camera_queue()

    def _drain_camera_queue(self) -> None:
        while True:
            settled = self.synchronizer.pop_ready()
            if settled is None:
                self._drain_ready_observations()
                return
            decision = settled.decision
            self._publish_sync_debug(decision)
            if not decision.accepted:
                self.status_pub.publish(
                    String(
                        data=(
                            "inference_rejected:sensor_skew:"
                            f"{decision.max_skew_sec:.6f}"
                        )
                    )
                )
                continue
            stamps = {
                role: decision.camera_stamp_sec + decision.deltas_sec[role]
                for role in self.SYNC_ROLES
            }
            self._queue_ready_observation(
                ReadyObservation(
                    image=settled.camera,
                    scan=decision.samples["lidar"],
                    velocity=decision.samples["velocity"],
                    steering=decision.samples["steering"],
                    role_stamps_sec=stamps,
                )
            )

    def _queue_ready_observation(self, observation: ReadyObservation) -> None:
        if len(self.ready_observations) >= self.ready_queue_size:
            dropped = self.ready_observations.popleft()
            dropped_stamp = strict_message_stamp_to_seconds(dropped.image)
            self.status_pub.publish(
                String(
                    data=(
                        "inference_rejected:runtime_clock_queue_overflow:"
                        f"{dropped_stamp:.9f}"
                    )
                )
            )
        self.ready_observations.append(observation)
        self._drain_ready_observations()

    def _drain_ready_observations(self) -> None:
        while self.ready_observations:
            observation = self.ready_observations[0]
            camera_stamp = strict_message_stamp_to_seconds(observation.image)
            source_stamps = {
                "camera": camera_stamp,
                **observation.role_stamps_sec,
            }
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec <= 0.0:
                return
            if not runtime_clock_has_reached_observation(
                now_sec=now_sec,
                source_stamps_sec=source_stamps,
            ):
                return
            self.ready_observations.popleft()
            self._process_observation(
                observation.image,
                observation.scan,
                observation.velocity,
                observation.steering,
                observation.role_stamps_sec,
            )

    def _publish_sync_debug(self, decision: SyncDecision[Any]) -> None:
        self.sync_debug_pub.publish(
            Float64MultiArray(
                data=[
                    decision.camera_stamp_sec,
                    *(
                        decision.deltas_sec.get(role, float("nan")) * 1000.0
                        for role in self.SYNC_ROLES
                    ),
                    decision.max_skew_sec * 1000.0,
                    1.0 if decision.accepted else 0.0,
                ]
            )
        )

    def _process_observation(
        self,
        image: Image,
        scan: LaserScan,
        velocity: VelocityReport,
        steering: SteeringReport,
        stamps: dict[str, float],
    ) -> None:
        try:
            camera_stamp = strict_message_stamp_to_seconds(image)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            validate_observation_timing(
                now_sec=now_sec,
                camera_stamp_sec=camera_stamp,
                role_stamps_sec=stamps,
                timeout_sec=self.timeout_sec,
                max_skew_sec=self.max_skew_sec,
            )
            batch = self._make_batch(image, scan, velocity, steering, stamps)
            with torch.inference_mode():
                output = self.model(batch)
            publication = trajectory_speed_publication(
                output.trajectory_xy.detach().cpu().numpy(),
                output.trajectory_speed_mps.detach().cpu().numpy(),
            )
            self.trajectory_pub.publish(
                Float32MultiArray(data=list(publication.trajectory_xy_m))
            )
            self.speed_profile_pub.publish(
                Float32MultiArray(data=list(publication.speed_profile_mps))
            )
            if self.behavior_enabled:
                self._publish_behavior(output)
            self.status_pub.publish(String(data="trajectory_published"))
        except Exception as error:
            self.status_pub.publish(String(data=f"inference_rejected:{error}"))

    def _make_batch(
        self,
        image: Image,
        scan: LaserScan,
        velocity: VelocityReport,
        steering: SteeringReport,
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
        # Competition-legal ego inputs: Wheel Odometry (VelocityReport) and Steer Angle.
        ego = torch.tensor([
            float(velocity.longitudinal_velocity),
            float(velocity.lateral_velocity),
            float(velocity.heading_rate),
            float(steering.steering_tire_angle),
        ], dtype=torch.float32, device=self.device)
        if not torch.isfinite(ego).all():
            raise ValueError("non_finite_ego")
        dt = torch.tensor(
            [[[
                stamps["lidar"] - stamps["velocity"],
                stamps["steering"] - stamps["velocity"],
            ]]],
            device=self.device,
        )
        requested = {"trajectory", "speed_profile"}
        if self.behavior_enabled:
            requested.update({"behavior", "behavior_side"})
        return ModelBatchV3(
            image=image_tensor[None, None], image_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            lidar=lidar[None, None], lidar_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            ego=ego[None, None], ego_feature_mask=torch.ones(1, 1, 4, dtype=torch.bool, device=self.device),
            command_history=torch.zeros(1, 1, 3, device=self.device),
            command_mask=torch.zeros(1, 1, dtype=torch.bool, device=self.device),
            sensor_dt_sec=dt, requested_outputs=frozenset(requested),
        )

    def _publish_behavior(self, output: Any) -> None:
        if output.behavior_logits is None or output.behavior_side_logits is None:
            raise ValueError("behavior-capable artifact returned no behavior logits")
        prediction = decode_behavior_logits_v1(
            output.behavior_logits[0].detach().cpu().tolist(),
            output.behavior_side_logits[0].detach().cpu().tolist(),
            confidence_threshold=self.behavior_threshold,
            temperature=self.behavior_temperature,
        )
        payload = {
            "ontology": BEHAVIOR_ONTOLOGY_V1,
            "label": prediction.behavior_label,
            "side": prediction.behavior_side_label,
            "confidence": prediction.behavior_confidence,
            "side_confidence": prediction.behavior_side_confidence,
            "source": "e2e_model",
        }
        if any(
            publisher is None
            for publisher in (
                self.behavior_mode_pub,
                self.behavior_label_pub,
                self.behavior_confidence_pub,
                self.behavior_side_pub,
            )
        ):
            raise RuntimeError("behavior publishers are not initialized")
        self.behavior_mode_pub.publish(Int32(data=prediction.behavior_class))
        self.behavior_side_pub.publish(Int32(data=prediction.behavior_side))
        self.behavior_confidence_pub.publish(
            Float32(data=prediction.behavior_confidence)
        )
        self.behavior_label_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InferenceNodeV3()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
