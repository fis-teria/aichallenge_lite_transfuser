from __future__ import annotations

"""Strict Dataset-v2 static TransFuser runtime for AWSIM."""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import math
import time
from typing import Any

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
import torch

from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
    control_from_waypoints_delay_aware,
)
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.runtime.inference_core import infer_v1
from aic_transfuser_lite.runtime.model_loader_v1 import load_runtime_model_v1
from aic_transfuser_lite.runtime.preprocessing_v1 import (
    V1LidarContract,
    prepare_native_lidar_input,
    prepare_speed_only_ego,
)
from aic_transfuser_lite.runtime.sensor_sync import (
    CameraMasterSynchronizer,
    SyncDecision,
)

from .runtime_adapter import (
    image_message_to_rgb,
    steering_report_to_angle,
    strict_message_stamp_to_seconds,
    velocity_report_to_state,
)


@dataclass(frozen=True)
class SynchronizedObservation:
    camera_stamp_sec: float
    image: Image
    scan: LaserScan
    velocity: VelocityReport
    steering: SteeringReport
    deltas_sec: dict[str, float]
    max_skew_sec: float


@dataclass(frozen=True)
class CameraObservation:
    stamp_sec: float
    image: Image


class InferenceNodeV1(Node):
    """Camera-master, fail-closed batch-size-one static-v1 inference."""

    SYNC_ROLES = ("lidar", "velocity", "steering")

    def __init__(self) -> None:
        super().__init__("aic_transfuser_inference_v1")
        self.declare_parameter("model_path", "")
        self.declare_parameter("expected_checkpoint_sha256", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("use_amp", True)
        self.declare_parameter("warmup_runs", 10)
        self.declare_parameter("inference_hz", 10.0)
        self.declare_parameter("input_timeout_sec", 0.35)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("max_sensor_skew_ms", 30.0)
        self.declare_parameter("expected_lidar_frame", "lidar")
        self.declare_parameter("estimated_delay_sec", 0.0)
        self.declare_parameter("base_preview_sec", 0.35)
        self.declare_parameter("min_preview_sec", 0.5)
        self.declare_parameter("max_preview_sec", 1.2)
        self.declare_parameter("wheelbase_m", 1.087)
        self.declare_parameter("max_steer_rad", 0.6)
        self.declare_parameter("min_accel_mps2", -4.0)
        self.declare_parameter("max_accel_mps2", 2.0)
        self.declare_parameter("speed_kp", 1.0)
        self.declare_parameter("max_steering_rate_radps", 0.0)

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        requested_device = str(self.get_parameter("device").value)
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(requested_device)
        loaded = load_runtime_model_v1(
            model_path,
            device=self.device,
            expected_checkpoint_sha256=str(
                self.get_parameter("expected_checkpoint_sha256").value
            ),
        )
        self.model = loaded.model
        self.config = loaded.config
        self.use_amp = bool(self.get_parameter("use_amp").value)

        data = self.config["data"]
        self.image_height = int(data["image_height"])
        self.image_width = int(data["image_width"])
        self.lidar_contract = V1LidarContract(
            points=int(data["lidar_points"]),
            angle_min_rad=float(data["lidar_angle_min_rad"]),
            angle_increment_rad=float(data["lidar_angle_increment_rad"]),
            range_min_m=float(data["lidar_min_range_m"]),
            range_max_m=float(data["lidar_max_range_m"]),
            frame_id=str(self.get_parameter("expected_lidar_frame").value),
        )
        self.ego_speed_scale_mps = float(data["ego_speed_scale_mps"])
        hz = float(self.get_parameter("inference_hz").value)
        if not math.isclose(hz, float(data["sample_rate_hz"]), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("inference_hz must equal the checkpoint sample_rate_hz")
        max_skew_ms = float(self.get_parameter("max_sensor_skew_ms").value)
        if max_skew_ms > float(data["sync_tolerance_ms"]):
            raise ValueError("runtime sensor skew may not exceed checkpoint tolerance")
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        if self.input_timeout_sec <= 0.0:
            raise ValueError("input_timeout_sec must be positive")
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.synchronizer: CameraMasterSynchronizer[Any] = CameraMasterSynchronizer(
            required_roles=self.SYNC_ROLES,
            queue_size=sync_queue_size,
            max_skew_sec=max_skew_ms / 1000.0,
        )
        waypoint_step = float(data["prediction_horizon_sec"]) / int(data["num_waypoints"])
        waypoint_times = tuple(
            waypoint_step * (index + 1) for index in range(int(data["num_waypoints"]))
        )
        self.controller_config = DelayAwareControllerConfig(
            waypoint_times_sec=waypoint_times,
            estimated_delay_sec=float(self.get_parameter("estimated_delay_sec").value),
            base_preview_sec=float(self.get_parameter("base_preview_sec").value),
            min_preview_sec=float(self.get_parameter("min_preview_sec").value),
            max_preview_sec=float(self.get_parameter("max_preview_sec").value),
            wheelbase_m=float(self.get_parameter("wheelbase_m").value),
            max_steer_rad=float(self.get_parameter("max_steer_rad").value),
            min_accel_mps2=float(self.get_parameter("min_accel_mps2").value),
            max_accel_mps2=float(self.get_parameter("max_accel_mps2").value),
            speed_kp=float(self.get_parameter("speed_kp").value),
            max_steering_rate_radps=float(
                self.get_parameter("max_steering_rate_radps").value
            ),
            control_period_sec=1.0 / hz,
        )
        self.camera_queue: deque[CameraObservation] = deque(
            maxlen=sync_queue_size
        )
        self.last_camera_stamp_sec = -math.inf
        self.last_warning_sec = -math.inf

        sensor_qos = QoSProfile(
            depth=sync_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, "image", self._on_image, sensor_qos)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_subscription(
            VelocityReport, "velocity_status", self._on_velocity, sensor_qos
        )
        self.create_subscription(
            SteeringReport, "steering_status", self._on_steering, sensor_qos
        )
        self.command_pub = self.create_publisher(
            AckermannControlCommand, "nominal_control_cmd", 1
        )
        self.waypoint_pub = self.create_publisher(
            Float32MultiArray, "predicted_waypoints", 1
        )
        self.sync_debug_pub = self.create_publisher(
            Float64MultiArray, "runtime_sync_debug", 1
        )
        self.control_debug_pub = self.create_publisher(
            Float64MultiArray, "runtime_control_debug", 1
        )
        self.status_pub = self.create_publisher(String, "runtime_status", 1)
        self._warmup(int(self.get_parameter("warmup_runs").value))
        self.get_logger().info(
            f"Loaded strict v1 epoch={loaded.checkpoint_epoch} sha={loaded.checkpoint_sha256} "
            f"on {self.device}; image={self.image_width}x{self.image_height}, "
            f"lidar=[2,{self.lidar_contract.points}], ego=[1], rate={hz:.1f}Hz, "
            f"skew<={max_skew_ms:.1f}ms, delay={self.controller_config.estimated_delay_sec:.3f}s"
        )

    def _warmup(self, runs: int) -> None:
        if runs < 0:
            raise ValueError("warmup_runs must be non-negative")
        image = torch.zeros(
            1, 3, self.image_height, self.image_width, device=self.device
        )
        lidar = torch.ones(
            1, 2, self.lidar_contract.points, device=self.device
        )
        ego = torch.zeros(1, 1, device=self.device)
        for _ in range(runs):
            infer_v1(
                self.model,
                image=image,
                lidar=lidar,
                ego=ego,
                use_amp=self.use_amp,
            )
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, text: str) -> None:
        now = self._now_sec()
        if now - self.last_warning_sec >= 2.0:
            self.get_logger().warning(text)
            self.last_warning_sec = now

    def _add_sample(self, role: str, message: Any) -> None:
        try:
            self.synchronizer.add(role, strict_message_stamp_to_seconds(message), message)
        except Exception as error:
            self._warn_throttled(f"Rejected {role} sample: {error}")
            return
        self._drain_camera_queue()

    def _on_scan(self, message: LaserScan) -> None:
        self._add_sample("lidar", message)

    def _on_velocity(self, message: VelocityReport) -> None:
        self._add_sample("velocity", message)

    def _on_steering(self, message: SteeringReport) -> None:
        self._add_sample("steering", message)

    def _publish_sync_debug(self, decision: SyncDecision[Any]) -> None:
        deltas_ms = [
            decision.deltas_sec.get(role, math.nan) * 1000.0
            for role in self.SYNC_ROLES
        ]
        self.sync_debug_pub.publish(
            Float64MultiArray(
                data=[
                    decision.camera_stamp_sec,
                    *deltas_ms,
                    decision.max_skew_sec * 1000.0,
                    1.0 if decision.accepted else 0.0,
                ]
            )
        )
        self.status_pub.publish(String(data=decision.reason))

    def _on_image(self, message: Image) -> None:
        try:
            stamp = strict_message_stamp_to_seconds(message)
        except Exception as error:
            self._warn_throttled(f"Rejected Camera sample: {error}")
            self.status_pub.publish(String(data="camera_timestamp_invalid"))
            return
        if stamp <= self.last_camera_stamp_sec:
            self._warn_throttled("Rejected non-increasing Camera timestamp")
            self.status_pub.publish(String(data="camera_timestamp_not_increasing"))
            return
        self.last_camera_stamp_sec = stamp
        if len(self.camera_queue) == self.camera_queue.maxlen:
            dropped = self.camera_queue.popleft()
            self._warn_throttled(
                f"Camera sync queue overflow; dropped stamp {dropped.stamp_sec:.6f}"
            )
            self.status_pub.publish(String(data="camera_sync_queue_overflow"))
        self.camera_queue.append(CameraObservation(stamp, message))
        self._drain_camera_queue()

    def _drain_camera_queue(self) -> None:
        while self.camera_queue:
            camera = self.camera_queue[0]
            decision = self.synchronizer.match(camera.stamp_sec)
            if decision.accepted:
                self.camera_queue.popleft()
                self._publish_sync_debug(decision)
                self._process_observation(
                    SynchronizedObservation(
                        camera_stamp_sec=camera.stamp_sec,
                        image=camera.image,
                        scan=decision.samples["lidar"],
                        velocity=decision.samples["velocity"],
                        steering=decision.samples["steering"],
                        deltas_sec=dict(decision.deltas_sec),
                        max_skew_sec=decision.max_skew_sec,
                    )
                )
                continue
            if (
                decision.reason == "sensor_skew"
                and self.synchronizer.all_streams_reached(camera.stamp_sec)
            ):
                self.camera_queue.popleft()
                self._publish_sync_debug(decision)
                self._warn_throttled(
                    f"No nominal command for Camera stamp {camera.stamp_sec:.6f}: "
                    f"sensor_skew={decision.max_skew_sec * 1000.0:.3f}ms"
                )
                continue
            break

    def _process_observation(self, observation: SynchronizedObservation) -> None:
        if observation is None:
            return
        now_sec = self._now_sec()
        age = now_sec - observation.camera_stamp_sec
        if age > self.input_timeout_sec or age < -0.1:
            self._warn_throttled(f"Synchronized observation is stale: age={age:.3f}s")
            self.status_pub.publish(String(data="synchronized_observation_stale"))
            return

        started = time.perf_counter()
        try:
            rgb = image_message_to_rgb(observation.image)
            image = preprocess_image(
                rgb,
                height=self.image_height,
                width=self.image_width,
            ).unsqueeze(0).to(self.device)
            scan = observation.scan
            lidar_array = prepare_native_lidar_input(
                np.asarray(scan.ranges, dtype=np.float32),
                angle_min_rad=float(scan.angle_min),
                angle_increment_rad=float(scan.angle_increment),
                range_min_m=float(scan.range_min),
                range_max_m=float(scan.range_max),
                frame_id=str(scan.header.frame_id),
                contract=self.lidar_contract,
            )
            lidar = torch.from_numpy(lidar_array).unsqueeze(0).to(self.device)
            longitudinal_speed_mps, yaw_rate_rps = velocity_report_to_state(
                observation.velocity
            )
            actual_steering_rad = steering_report_to_angle(observation.steering)
            ego_array = prepare_speed_only_ego(
                longitudinal_speed_mps,
                scale_mps=self.ego_speed_scale_mps,
            )
            ego = torch.from_numpy(ego_array).unsqueeze(0).to(self.device)
            output = infer_v1(
                self.model,
                image=image,
                lidar=lidar,
                ego=ego,
                use_amp=self.use_amp,
            )
            waypoints = output["waypoints"][0].numpy()
            target_speed_mps = float(output["target_speed"][0, 0])
            control = control_from_waypoints_delay_aware(
                waypoints,
                target_speed_mps=target_speed_mps,
                current_longitudinal_speed_mps=longitudinal_speed_mps,
                yaw_rate_rps=yaw_rate_rps,
                actual_steering_rad=actual_steering_rad,
                config=self.controller_config,
            )
        except Exception as error:
            self.get_logger().error(f"V1 inference/control failed closed: {error}")
            self.status_pub.publish(String(data="inference_or_control_error"))
            return

        message = AckermannControlCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.longitudinal.speed = control.commanded_speed_mps
        message.longitudinal.acceleration = control.command.acceleration_mps2
        message.lateral.steering_tire_angle = control.command.steering_rad
        self.command_pub.publish(message)
        self.waypoint_pub.publish(
            Float32MultiArray(data=waypoints.astype(np.float32).reshape(-1).tolist())
        )
        latency_ms = (time.perf_counter() - started) * 1000.0
        observation_age_ms = (self._now_sec() - observation.camera_stamp_sec) * 1000.0
        self.control_debug_pub.publish(
            Float64MultiArray(
                data=[
                    observation.camera_stamp_sec,
                    latency_ms,
                    observation_age_ms,
                    observation.max_skew_sec * 1000.0,
                    control.delay_sec,
                    control.preview_time_sec,
                    control.curvature_per_m,
                    control.unlimited_steering_rad,
                    control.command.steering_rad,
                    actual_steering_rad,
                    control.commanded_speed_mps,
                    longitudinal_speed_mps,
                ]
            )
        )
        self.status_pub.publish(String(data="nominal_published"))
        if latency_ms > 20.0:
            self._warn_throttled(f"V1 inference p95 gate sample exceeded: {latency_ms:.1f}ms")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNodeV1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
