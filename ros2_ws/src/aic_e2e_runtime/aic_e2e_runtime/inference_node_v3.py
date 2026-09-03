from __future__ import annotations

"""Trajectory-authority V3 ROS adapter with optional behavior diagnostics."""

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
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
from aic_transfuser_lite.runtime.authority import model_control_debug_publication
from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
)
from aic_transfuser_lite.control.shadow_trajectory_controller import (
    shadow_control_from_trajectory_speed_profile,
)
from aic_transfuser_lite.data.image_preprocess import preprocess_image
from aic_transfuser_lite.data.normalization import normalize_lidar_range_and_validity
from aic_transfuser_lite.data.calibration.artifact import load_calibration_artifact
from aic_transfuser_lite.runtime.control_projection import (
    ControlLimits,
    PreviousControlState,
    ProjectionTiming,
    apply_stopped_launch_acceleration_floor,
    project_model_control_sequence,
)
from aic_transfuser_lite.runtime.full_control_gate import (
    ControlAuthorityMode,
    FullControlReadiness,
    authority_change_allowed,
    choose_full_control_or_same_trajectory_fallback,
    nominal_command_history,
)
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3, sha256_file_v3
from aic_transfuser_lite.runtime.residual_control import ExternalControllerCommand
from aic_transfuser_lite.runtime.rollout_consistency import (
    ConsistencyThresholds,
    RolloutInitialState,
    evaluate_rollout_consistency,
    rollout_actuator_bicycle,
)
from aic_transfuser_lite.runtime.behavior_decode_v1 import decode_behavior_logits_v1
from aic_transfuser_lite.runtime.output_profiles import (
    RuntimeProfile,
    output_profile,
    runtime_clock_has_reached_observation,
    trajectory_path_publication,
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
            "runtime_profile": "trajectory_only",
            "trajectory_frame_id": "base_link",
            "input_timeout_sec": 0.35, "sync_queue_size": 10,
            "sync_clock_poll_sec": 0.005,
            "max_sensor_skew_ms": 30.0,
            "lidar_min_range_m": 0.0, "lidar_max_range_m": 25.0,
            "behavior_confidence_threshold": 0.5, "behavior_temperature": 1.0,
            "controller_calibration_status": "unverified",
            "trajectory_step_sec": 0.1,
            "estimated_delay_sec": 0.0,
            "base_preview_sec": 0.35,
            "min_preview_sec": 0.5,
            "max_preview_sec": 1.2,
            "wheelbase_m": 1.087,
            "max_steer_rad": 0.6,
            "min_accel_mps2": -4.0,
            "max_accel_mps2": 2.0,
            "speed_kp": 1.0,
            "max_steering_rate_radps": 0.0,
            "control_period_sec": 0.1,
            "calibration_artifact_path": "",
            "full_control_deployment_stage": "limited_odd_trial",
            "full_control_evidence_sha256": "",
            "full_control_evidence_path": "",
            "full_control_evidence_passed": False,
            "safety_supervisor_ready": False,
            "trial_speed_cap_mps": 0.8,
            "command_valid_for_sec": 0.2,
            "max_observation_age_sec": 0.15,
            "min_jerk_mps3": -8.0,
            "max_jerk_mps3": 4.0,
            "consistency_max_position_error_m": 0.75,
            "consistency_max_lateral_error_m": 0.5,
            "consistency_max_heading_error_rad": 0.7,
            "consistency_max_speed_error_mps": 1.5,
            "consistency_max_endpoint_error_m": 0.75,
            "consistency_min_heading_speed_mps": 0.2,
            "launch_assist_enabled": False,
            "launch_assist_stopped_speed_mps": 0.1,
            "launch_assist_min_commanded_speed_mps": 0.2,
            "launch_assist_acceleration_floor_mps2": 0.5,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        self.runtime_profile = RuntimeProfile(
            str(self.get_parameter("runtime_profile").value)
        )
        if self.runtime_profile not in {
            RuntimeProfile.TRAJECTORY_ONLY,
            RuntimeProfile.EXTERNAL_CONTROLLER,
            RuntimeProfile.SHADOW_CONTROL,
            RuntimeProfile.FULL_CONTROL,
        }:
            raise ValueError(
                "inference_node_v3 supports only trajectory_only, external_controller, "
                "shadow_control, or full_control"
            )
        selected_profile = output_profile(self.runtime_profile)
        if (
            selected_profile.nominal_control_authority
            and self.runtime_profile is not RuntimeProfile.FULL_CONTROL
        ):
            raise RuntimeError("V3 trajectory adapter unexpectedly has control authority")
        self.trajectory_frame_id = str(
            self.get_parameter("trajectory_frame_id").value
        )
        trajectory_path_publication((0.0, 0.0), frame_id=self.trajectory_frame_id)
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
        self.model_capabilities = loaded.capabilities
        if (
            self.runtime_profile is RuntimeProfile.SHADOW_CONTROL
            and "current_control" not in self.model_capabilities
        ):
            raise ValueError("shadow_control requires current_control artifact capability")
        if self.runtime_profile is RuntimeProfile.FULL_CONTROL and not {
            "trajectory", "control_sequence"
        }.issubset(self.model_capabilities):
            raise ValueError("full_control requires trajectory and control_sequence capabilities")
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
        self.controller_config: DelayAwareControllerConfig | None = None
        if self.runtime_profile in {
            RuntimeProfile.EXTERNAL_CONTROLLER,
            RuntimeProfile.FULL_CONTROL,
        }:
            calibration_status = str(
                self.get_parameter("controller_calibration_status").value
            )
            if (
                self.runtime_profile is RuntimeProfile.EXTERNAL_CONTROLLER
                and calibration_status != "unverified"
            ):
                raise ValueError(
                    "external controller shadow requires calibration_status=unverified "
                    "until a verified V3 calibration artifact is implemented"
                )
            trajectory_step_sec = float(
                self.get_parameter("trajectory_step_sec").value
            )
            if not np.isfinite(trajectory_step_sec) or trajectory_step_sec <= 0.0:
                raise ValueError("trajectory_step_sec must be finite and positive")
            waypoint_times = tuple(
                trajectory_step_sec * (index + 1)
                for index in range(self.model.trajectory_steps)
            )
            self.controller_config = DelayAwareControllerConfig(
                waypoint_times_sec=waypoint_times,
                estimated_delay_sec=float(
                    self.get_parameter("estimated_delay_sec").value
                ),
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
                control_period_sec=float(
                    self.get_parameter("control_period_sec").value
                ),
            )
        self.full_control_readiness = None
        self.full_control_limits = None
        self.full_control_calibration = None
        self.consistency_thresholds = None
        self.previous_nominal_command: ExternalControllerCommand | None = None
        self.nominal_command_history: deque[ExternalControllerCommand] = deque(
            maxlen=self.model.max_ego_history
        )
        self.launch_assist_completed = False
        if self.runtime_profile is RuntimeProfile.FULL_CONTROL:
            calibration_path = Path(
                str(self.get_parameter("calibration_artifact_path").value)
            ).expanduser()
            self.full_control_calibration = load_calibration_artifact(calibration_path)
            stage = str(self.get_parameter("full_control_deployment_stage").value)
            trial_cap = float(self.get_parameter("trial_speed_cap_mps").value)
            self.full_control_readiness = FullControlReadiness(
                capabilities=self.model_capabilities,
                calibration_state=self.full_control_calibration.promotion.state,
                deployment_stage=stage,
                safety_supervisor_ready=bool(
                    self.get_parameter("safety_supervisor_ready").value
                ),
                evidence_sha256=str(
                    self.get_parameter("full_control_evidence_sha256").value
                ),
                evidence_passed=bool(
                    self.get_parameter("full_control_evidence_passed").value
                ),
                trial_speed_cap_mps=trial_cap if stage == "limited_odd_trial" else None,
            )
            self.full_control_readiness.validate()
            evidence_path = Path(
                str(self.get_parameter("full_control_evidence_path").value)
            ).expanduser()
            if sha256_file_v3(evidence_path) != self.full_control_readiness.evidence_sha256:
                raise ValueError("full-control evidence file SHA-256 mismatch")
            if not authority_change_allowed(
                ControlAuthorityMode.SHADOW,
                ControlAuthorityMode.FULL_CONTROL,
                lifecycle_inactive=True,
                longitudinal_speed_mps=0.0,
            ):
                raise RuntimeError("full-control authority transition was rejected")
            self.full_control_limits = ControlLimits(
                max_abs_steering_rad=float(self.get_parameter("max_steer_rad").value),
                max_steering_rate_radps=float(
                    self.get_parameter("max_steering_rate_radps").value
                ),
                min_acceleration_mps2=float(self.get_parameter("min_accel_mps2").value),
                max_acceleration_mps2=float(self.get_parameter("max_accel_mps2").value),
                min_jerk_mps3=float(self.get_parameter("min_jerk_mps3").value),
                max_jerk_mps3=float(self.get_parameter("max_jerk_mps3").value),
                max_speed_mps=trial_cap,
                dt_sec=float(self.get_parameter("control_period_sec").value),
                authoritative=True,
                source=self.full_control_calibration.vehicle_profile_sha256,
            )
            self.consistency_thresholds = ConsistencyThresholds(
                max_position_error_m=float(self.get_parameter("consistency_max_position_error_m").value),
                max_lateral_error_m=float(self.get_parameter("consistency_max_lateral_error_m").value),
                max_heading_error_rad=float(self.get_parameter("consistency_max_heading_error_rad").value),
                max_speed_error_mps=float(self.get_parameter("consistency_max_speed_error_mps").value),
                max_endpoint_error_m=float(self.get_parameter("consistency_max_endpoint_error_m").value),
                min_heading_speed_mps=float(
                    self.get_parameter("consistency_min_heading_speed_mps").value
                ),
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
        self.trajectory_path_pub = self.create_publisher(
            PathMessage, "predicted_trajectory_path", 1
        )
        self.speed_profile_pub = self.create_publisher(
            Float32MultiArray, "predicted_speed_profile", 1
        )
        self.shadow_control_pub = None
        if self.runtime_profile is RuntimeProfile.EXTERNAL_CONTROLLER:
            self.shadow_control_pub = self.create_publisher(
                AckermannControlCommand, "shadow_external_control", 1
            )
        self.shadow_model_control_pub = None
        if self.runtime_profile is RuntimeProfile.SHADOW_CONTROL:
            self.shadow_model_control_pub = self.create_publisher(
                AckermannControlCommand, "shadow_model_control", 1
            )
        self.shadow_model_sequence_pub = None
        if (
            self.runtime_profile is RuntimeProfile.SHADOW_CONTROL
            and "control_sequence" in self.model_capabilities
        ):
            self.shadow_model_sequence_pub = self.create_publisher(
                Float32MultiArray, "shadow_model_control_sequence", 1
            )
        self.full_control_pub = None
        if self.runtime_profile is RuntimeProfile.FULL_CONTROL:
            self.full_control_pub = self.create_publisher(
                AckermannControlCommand, "nominal_control_cmd", 1
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
            path_publication = trajectory_path_publication(
                publication.trajectory_xy_m,
                frame_id=self.trajectory_frame_id,
            )
            self.trajectory_pub.publish(
                Float32MultiArray(data=list(publication.trajectory_xy_m))
            )
            self.speed_profile_pub.publish(
                Float32MultiArray(data=list(publication.speed_profile_mps))
            )
            path_message = PathMessage()
            path_message.header.stamp = image.header.stamp
            path_message.header.frame_id = path_publication.frame_id
            for x_m, y_m in path_publication.points_xy_m:
                pose = PoseStamped()
                pose.header.stamp = image.header.stamp
                pose.header.frame_id = path_publication.frame_id
                pose.pose.position.x = x_m
                pose.pose.position.y = y_m
                pose.pose.orientation.w = 1.0
                path_message.poses.append(pose)
            self.trajectory_path_pub.publish(path_message)
            if self.shadow_control_pub is not None:
                try:
                    self._publish_shadow_external_control(
                        publication.trajectory_xy_m,
                        publication.speed_profile_mps,
                        image,
                        velocity,
                        steering,
                    )
                except Exception as error:
                    self.status_pub.publish(
                        String(data=f"shadow_control_rejected:{error}")
                    )
            if self.shadow_model_control_pub is not None:
                try:
                    self._publish_shadow_model_control(output.current_control, image)
                except Exception as error:
                    self.status_pub.publish(
                        String(data=f"shadow_model_control_rejected:{error}")
                    )
            if self.shadow_model_sequence_pub is not None:
                if output.control_sequence is None:
                    self.status_pub.publish(String(data="shadow_model_sequence_rejected:absent"))
                else:
                    sequence = output.control_sequence.detach().cpu().numpy()
                    if sequence.shape != (1, 1, self.model.control_sequence_steps, 3):
                        self.status_pub.publish(
                            String(data=f"shadow_model_sequence_rejected:shape:{sequence.shape}")
                        )
                    elif not np.isfinite(sequence).all():
                        self.status_pub.publish(
                            String(data="shadow_model_sequence_rejected:non_finite")
                        )
                    else:
                        self.shadow_model_sequence_pub.publish(
                            Float32MultiArray(data=sequence[0, 0].reshape(-1).tolist())
                        )
            if self.full_control_pub is not None:
                self._publish_gated_full_control(
                    output,
                    publication.trajectory_xy_m,
                    publication.speed_profile_mps,
                    image,
                    velocity,
                    steering,
                )
            if self.behavior_enabled:
                self._publish_behavior(output)
            self.status_pub.publish(String(data="trajectory_published"))
        except Exception as error:
            self.status_pub.publish(String(data=f"inference_rejected:{error}"))

    def _publish_shadow_external_control(
        self,
        trajectory_xy_m: tuple[float, ...],
        speed_profile_mps: tuple[float, ...],
        image: Image,
        velocity: VelocityReport,
        steering: SteeringReport,
    ) -> None:
        if self.controller_config is None or self.shadow_control_pub is None:
            raise RuntimeError("external controller shadow is not initialized")
        result = self._external_control_from_trajectory(
            trajectory_xy_m,
            speed_profile_mps,
            current_longitudinal_speed_mps=float(velocity.longitudinal_velocity),
            yaw_rate_rps=float(velocity.heading_rate),
            actual_steering_rad=float(steering.steering_tire_angle),
        )
        if result.nominal_control_eligible:
            raise RuntimeError("unverified shadow proposal became authority-eligible")
        message = AckermannControlCommand()
        message.stamp = image.header.stamp
        message.longitudinal.speed = result.control.commanded_speed_mps
        message.longitudinal.acceleration = result.control.command.acceleration_mps2
        message.lateral.steering_tire_angle = result.control.command.steering_rad
        self.shadow_control_pub.publish(message)
        self.status_pub.publish(String(data="shadow_control_published:unverified"))

    def _external_control_from_trajectory(
        self,
        trajectory_xy_m: tuple[float, ...],
        speed_profile_mps: tuple[float, ...],
        *,
        current_longitudinal_speed_mps: float,
        yaw_rate_rps: float,
        actual_steering_rad: float,
    ) -> Any:
        if self.controller_config is None:
            raise RuntimeError("same-trajectory fallback controller is not initialized")
        return shadow_control_from_trajectory_speed_profile(
            np.asarray(trajectory_xy_m, dtype=np.float32).reshape(-1, 2),
            np.asarray(speed_profile_mps, dtype=np.float32),
            current_longitudinal_speed_mps=current_longitudinal_speed_mps,
            yaw_rate_rps=yaw_rate_rps,
            actual_steering_rad=actual_steering_rad,
            config=self.controller_config,
        )

    def _publish_gated_full_control(
        self,
        output: Any,
        trajectory_xy_m: tuple[float, ...],
        speed_profile_mps: tuple[float, ...],
        image: Image,
        velocity: VelocityReport,
        steering: SteeringReport,
    ) -> None:
        if any(
            item is None
            for item in (
                self.full_control_pub,
                self.full_control_readiness,
                self.full_control_limits,
                self.full_control_calibration,
                self.consistency_thresholds,
            )
        ):
            raise RuntimeError("full-control runtime is not initialized")
        if output.control_sequence is None:
            raise ValueError("full-control model returned no control_sequence")
        sequence_values = output.control_sequence.detach().cpu().numpy()
        if sequence_values.ndim != 4 or sequence_values.shape[:2] != (1, 1):
            raise ValueError("full-control sequence must be [1,1,H,3]")
        camera_stamp = strict_message_stamp_to_seconds(image)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        previous_acceleration_mps2 = (
            0.0
            if self.previous_nominal_command is None
            else self.previous_nominal_command.acceleration_mps2
        )
        previous = PreviousControlState(
            steering_rad=float(steering.steering_tire_angle),
            speed_mps=float(velocity.longitudinal_velocity),
            acceleration_mps2=previous_acceleration_mps2,
        )
        proposals = sequence_values[0, 0]
        launch_assist_applied = False
        if bool(self.get_parameter("launch_assist_enabled").value):
            stopped_threshold = float(
                self.get_parameter("launch_assist_stopped_speed_mps").value
            )
            if previous.speed_mps > stopped_threshold:
                self.launch_assist_completed = True
            if not self.launch_assist_completed:
                proposals, launch_assist_applied = (
                    apply_stopped_launch_acceleration_floor(
                        proposals,
                        previous=previous,
                        limits=self.full_control_limits,
                        stopped_speed_threshold_mps=stopped_threshold,
                        minimum_commanded_speed_mps=float(
                            self.get_parameter(
                                "launch_assist_min_commanded_speed_mps"
                            ).value
                        ),
                        acceleration_floor_mps2=float(
                            self.get_parameter(
                                "launch_assist_acceleration_floor_mps2"
                            ).value
                        ),
                    )
                )
        projected = project_model_control_sequence(
            proposals,
            previous=previous,
            limits=self.full_control_limits,
            timing=ProjectionTiming(
                observation_stamp_sec=camera_stamp,
                now_sec=now_sec,
                valid_for_sec=float(self.get_parameter("command_valid_for_sec").value),
                max_observation_age_sec=float(
                    self.get_parameter("max_observation_age_sec").value
                ),
            ),
        )
        rollout = rollout_actuator_bicycle(
            projected,
            calibration=self.full_control_calibration,
            wheelbase_m=float(self.get_parameter("wheelbase_m").value),
            initial=RolloutInitialState(
                actual_steering_rad=float(steering.steering_tire_angle),
                actual_acceleration_mps2=previous_acceleration_mps2,
                longitudinal_mode=(
                    "brake" if previous_acceleration_mps2 < -0.1 else "drive"
                ),
            ),
        )
        horizon = projected.commands.shape[0]
        trajectory = np.asarray(trajectory_xy_m, dtype=np.float64).reshape(-1, 2)
        speeds = np.asarray(speed_profile_mps, dtype=np.float64)
        if trajectory.shape[0] < horizon:
            raise ValueError("model trajectory is shorter than control sequence")
        consistency = evaluate_rollout_consistency(
            trajectory[:horizon],
            speeds[:horizon],
            rollout,
            thresholds=self.consistency_thresholds,
        )
        fallback = None
        if not consistency.consistent:
            fallback_result = self._external_control_from_trajectory(
                trajectory_xy_m,
                speed_profile_mps,
                current_longitudinal_speed_mps=float(velocity.longitudinal_velocity),
                yaw_rate_rps=float(velocity.heading_rate),
                actual_steering_rad=float(steering.steering_tire_angle),
            )
            fallback = ExternalControllerCommand(
                fallback_result.control.command.steering_rad,
                fallback_result.control.commanded_speed_mps,
                fallback_result.control.command.acceleration_mps2,
            )
        decision = choose_full_control_or_same_trajectory_fallback(
            projected,
            consistency,
            fallback,
            readiness=self.full_control_readiness,
            selected_trajectory_id="candidate0",
            fallback_trajectory_id="candidate0",
        )
        message = AckermannControlCommand()
        message.stamp = image.header.stamp
        message.lateral.steering_tire_angle = decision.command.steering_rad
        message.longitudinal.speed = decision.command.speed_mps
        message.longitudinal.acceleration = decision.command.acceleration_mps2
        self.full_control_pub.publish(message)
        self.previous_nominal_command = decision.command
        self.nominal_command_history.append(decision.command)
        reasons = ",".join(decision.consistency_reasons) or "consistent"
        if launch_assist_applied and decision.source == "model_control_sequence":
            reasons = f"launch_assist,{reasons}"
        self.status_pub.publish(String(data=f"full_control_published:{decision.source}:{reasons}"))

    def _publish_shadow_model_control(self, current_control: Any, image: Image) -> None:
        if self.shadow_model_control_pub is None:
            raise RuntimeError("model-control shadow publisher is not initialized")
        if current_control is None:
            raise ValueError("model returned no current_control output")
        proposal = model_control_debug_publication(
            current_control.detach().cpu().numpy()
        )
        if proposal.authoritative:
            raise RuntimeError("model-control shadow became authority-eligible")
        message = AckermannControlCommand()
        message.stamp = image.header.stamp
        message.lateral.steering_tire_angle = proposal.steering_rad
        message.longitudinal.speed = proposal.speed_mps
        message.longitudinal.acceleration = proposal.acceleration_mps2
        self.shadow_model_control_pub.publish(message)
        self.status_pub.publish(String(data="shadow_model_control_published:debug_only"))

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
        if self.runtime_profile in {RuntimeProfile.SHADOW_CONTROL, RuntimeProfile.FULL_CONTROL}:
            requested.add("current_control")
        if (
            self.runtime_profile is RuntimeProfile.SHADOW_CONTROL
            and "control_sequence" in self.model_capabilities
        ):
            requested.add("control_sequence")
        if self.runtime_profile is RuntimeProfile.FULL_CONTROL:
            requested.add("control_sequence")
        if self.behavior_enabled:
            requested.update({"behavior", "behavior_side"})
        command_values, command_valid = nominal_command_history(
            tuple(self.nominal_command_history),
            length=self.model.max_ego_history,
        )
        return ModelBatchV3(
            image=image_tensor[None, None], image_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            lidar=lidar[None, None], lidar_mask=torch.ones(1, 1, dtype=torch.bool, device=self.device),
            ego=ego[None, None], ego_feature_mask=torch.ones(1, 1, 4, dtype=torch.bool, device=self.device),
            command_history=torch.tensor(
                command_values, dtype=torch.float32, device=self.device
            )[None],
            command_mask=torch.tensor(
                [command_valid], dtype=torch.bool, device=self.device
            ),
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
