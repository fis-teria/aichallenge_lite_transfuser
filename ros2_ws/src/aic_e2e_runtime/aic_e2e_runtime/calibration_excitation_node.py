from __future__ import annotations

"""Fail-closed scripted AWSIM excitation for actuator-calibration recording."""

from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .canonical_source import prefer_canonical_source

prefer_canonical_source()

from aic_transfuser_lite.data.calibration.excitation import (
    ExcitationCommand,
    excitation_plan_sha256,
    load_excitation_plan,
    runtime_guard_reasons,
)


class CalibrationExcitationNode(Node):
    def __init__(self) -> None:
        super().__init__("aic_calibration_excitation_v3")
        self.declare_parameter("plan_path", "")
        self.declare_parameter("arm_token", "")
        self.declare_parameter("result_path", "")
        self.declare_parameter("preflight_timeout_sec", 15.0)
        self.declare_parameter("post_stop_timeout_sec", 15.0)
        self.declare_parameter("abort_stop_hold_sec", 2.0)

        plan_path = Path(str(self.get_parameter("plan_path").value)).expanduser()
        self.plan = load_excitation_plan(plan_path)
        self.plan_sha256 = excitation_plan_sha256(self.plan)
        arm_token = str(self.get_parameter("arm_token").value)
        if arm_token != self.plan_sha256:
            raise ValueError("arm_token must exactly match the validated excitation plan SHA-256")
        result_value = str(self.get_parameter("result_path").value)
        if not result_value:
            raise ValueError("result_path is required")
        self.result_path = Path(result_value).expanduser()
        for name in (
            "preflight_timeout_sec",
            "post_stop_timeout_sec",
            "abort_stop_hold_sec",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.preflight_timeout_sec = float(
            self.get_parameter("preflight_timeout_sec").value
        )
        self.post_stop_timeout_sec = float(
            self.get_parameter("post_stop_timeout_sec").value
        )
        self.abort_stop_hold_sec = float(
            self.get_parameter("abort_stop_hold_sec").value
        )

        self.nominal_pub = self.create_publisher(
            AckermannControlCommand, "nominal_control_cmd", 1
        )
        self.status_pub = self.create_publisher(String, "calibration_status", 10)
        self.create_subscription(
            VelocityReport,
            "velocity_status",
            self._on_velocity,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "safety_reason", self._on_safety_reason, 10)

        self.state = "preflight"
        self.exit_code = 1
        self.abort_reason: str | None = None
        self.latest_speed_mps: float | None = None
        self.velocity_arrival_wall_sec: float | None = None
        self.safety_reason: str | None = None
        self.safety_arrival_wall_sec: float | None = None
        self.preflight_started_wall_sec = time.monotonic()
        self.preflight_stable_since_wall_sec: float | None = None
        self.plan_started_ros_sec: float | None = None
        self.last_ros_sec: float | None = None
        self.stop_started_wall_sec: float | None = None
        self.stopped_since_wall_sec: float | None = None
        self.abort_started_wall_sec: float | None = None
        self.last_segment_id: str | None = None
        self.samples_published = 0
        self.max_abs_observed_speed_mps = 0.0
        self.safety_reason_counts: Counter[str] = Counter()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self._result_written = False
        self._timer = self.create_timer(1.0 / self.plan.publish_hz, self._on_timer)
        self.get_logger().warning(
            "Calibration excitation armed by exact plan hash; waiting for exclusive "
            "publisher ownership, fresh telemetry, Safety, and stationary vehicle"
        )

    def _on_velocity(self, message: VelocityReport) -> None:
        self.latest_speed_mps = math.hypot(
            float(message.longitudinal_velocity), float(message.lateral_velocity)
        )
        self.velocity_arrival_wall_sec = time.monotonic()
        if math.isfinite(self.latest_speed_mps):
            self.max_abs_observed_speed_mps = max(
                self.max_abs_observed_speed_mps, abs(self.latest_speed_mps)
            )

    def _on_safety_reason(self, message: String) -> None:
        self.safety_reason = str(message.data)
        self.safety_arrival_wall_sec = time.monotonic()
        self.safety_reason_counts[self.safety_reason] += 1

    def _ros_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_command(self, command: ExcitationCommand) -> None:
        message = AckermannControlCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.lateral.steering_tire_angle = float(command.steering_rad)
        message.longitudinal.speed = float(command.speed_mps)
        message.longitudinal.acceleration = float(command.acceleration_mps2)
        self.nominal_pub.publish(message)
        self.samples_published += 1

    def _stop_command(self) -> ExcitationCommand:
        return self.plan.segments[-1].command

    def _topic_counts(self) -> tuple[int, int, int, int]:
        nominal = self.resolve_topic_name("nominal_control_cmd")
        final = self.resolve_topic_name("control_cmd")
        return (
            self.count_publishers(nominal),
            self.count_publishers(final),
            self.count_subscribers(nominal),
            self.count_subscribers(final),
        )

    def _guard_reasons(self, now_wall_sec: float) -> tuple[str, ...]:
        nominal_publishers, final_publishers, nominal_subscribers, final_subscribers = (
            self._topic_counts()
        )
        velocity_age = (
            None
            if self.velocity_arrival_wall_sec is None
            else now_wall_sec - self.velocity_arrival_wall_sec
        )
        safety_age = (
            None
            if self.safety_arrival_wall_sec is None
            else now_wall_sec - self.safety_arrival_wall_sec
        )
        reasons = list(
            runtime_guard_reasons(
                speed_mps=self.latest_speed_mps,
                telemetry_age_sec=velocity_age,
                safety_reason=self.safety_reason,
                nominal_publisher_count=nominal_publishers,
                final_publisher_count=final_publishers,
                nominal_subscriber_count=nominal_subscribers,
                final_subscriber_count=final_subscribers,
                plan=self.plan,
            )
        )
        if safety_age is None or safety_age > self.plan.telemetry_timeout_sec:
            reasons.append("safety_reason_stale")
        return tuple(reasons)

    def _on_timer(self) -> None:
        now_wall = time.monotonic()
        now_ros = self._ros_now_sec()
        if self.state == "preflight":
            self._run_preflight(now_wall)
        elif self.state == "running":
            self._run_plan(now_wall, now_ros)
        elif self.state == "stopping":
            self._run_stopping(now_wall)
        elif self.state == "aborting":
            self._run_abort(now_wall)

    def _run_preflight(self, now_wall: float) -> None:
        self._publish_command(self._stop_command())
        reasons = self._guard_reasons(now_wall)
        ownership = tuple(
            reason
            for reason in reasons
            if "publisher_count" in reason
            or reason.endswith("subscriber_missing")
        )
        if ownership and now_wall - self.preflight_started_wall_sec >= 1.0:
            self._begin_abort("authority_preflight:" + ",".join(ownership))
            return
        stationary = (
            self.latest_speed_mps is not None
            and math.isfinite(self.latest_speed_mps)
            and abs(self.latest_speed_mps) <= self.plan.stop_speed_threshold_mps
        )
        if not reasons and stationary:
            if self.preflight_stable_since_wall_sec is None:
                self.preflight_stable_since_wall_sec = now_wall
            elif now_wall - self.preflight_stable_since_wall_sec >= self.plan.preflight_hold_sec:
                self.state = "running"
                self.plan_started_ros_sec = self._ros_now_sec()
                self.last_ros_sec = self.plan_started_ros_sec
                self._publish_status("running", "preflight_passed")
                self.get_logger().warning("Preflight passed; starting bounded excitation")
                return
        else:
            self.preflight_stable_since_wall_sec = None
        if now_wall - self.preflight_started_wall_sec > self.preflight_timeout_sec:
            detail = ",".join(reasons) if reasons else "vehicle_not_stationary"
            self._begin_abort("preflight_timeout:" + detail)

    def _run_plan(self, now_wall: float, now_ros: float) -> None:
        reasons = self._guard_reasons(now_wall)
        if reasons:
            self._begin_abort("runtime_guard:" + ",".join(reasons))
            return
        if self.last_ros_sec is not None and now_ros < self.last_ros_sec:
            self._begin_abort("ros_clock_moved_backwards")
            return
        self.last_ros_sec = now_ros
        assert self.plan_started_ros_sec is not None
        elapsed = now_ros - self.plan_started_ros_sec
        if not math.isfinite(elapsed) or elapsed < 0.0:
            self._begin_abort("invalid_plan_elapsed_time")
            return
        if elapsed >= self.plan.total_duration_sec:
            self.state = "stopping"
            self.stop_started_wall_sec = now_wall
            self._publish_status("stopping", "plan_complete")
            self._publish_command(self._stop_command())
            return
        segment, _ = self.plan.command_at(elapsed)
        if segment.segment_id != self.last_segment_id:
            self.last_segment_id = segment.segment_id
            self._publish_status("running", f"segment:{segment.segment_id}")
            self.get_logger().info(
                f"Excitation segment={segment.segment_id} mode={segment.mode}"
            )
        self._publish_command(segment.command)

    def _run_stopping(self, now_wall: float) -> None:
        self._publish_command(self._stop_command())
        stopped = (
            self.latest_speed_mps is not None
            and math.isfinite(self.latest_speed_mps)
            and abs(self.latest_speed_mps) <= self.plan.stop_speed_threshold_mps
        )
        if stopped:
            if self.stopped_since_wall_sec is None:
                self.stopped_since_wall_sec = now_wall
            elif now_wall - self.stopped_since_wall_sec >= self.plan.stop_hold_sec:
                self._finish("complete", 0, None)
                return
        else:
            self.stopped_since_wall_sec = None
        assert self.stop_started_wall_sec is not None
        if now_wall - self.stop_started_wall_sec > self.post_stop_timeout_sec:
            self._begin_abort("post_stop_timeout")

    def _begin_abort(self, reason: str) -> None:
        if self.state == "aborting":
            return
        self.state = "aborting"
        self.abort_reason = reason
        self.abort_started_wall_sec = time.monotonic()
        self._publish_status("aborting", reason)
        self.get_logger().error(f"Calibration excitation aborted: {reason}")
        self._publish_command(self._stop_command())

    def _run_abort(self, now_wall: float) -> None:
        self._publish_command(self._stop_command())
        assert self.abort_started_wall_sec is not None
        if now_wall - self.abort_started_wall_sec >= self.abort_stop_hold_sec:
            self._finish("aborted", 2, self.abort_reason)

    def _publish_status(self, state: str, detail: str) -> None:
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {"state": state, "detail": detail, "plan_id": self.plan.plan_id},
                    sort_keys=True,
                )
            )
        )

    def _finish(self, status: str, exit_code: int, reason: str | None) -> None:
        if self._result_written:
            return
        self.state = status
        self.exit_code = exit_code
        payload = {
            "format_version": "aic_calibration_capture_result_v1",
            "status": status,
            "reason": reason,
            "plan_id": self.plan.plan_id,
            "plan_sha256": self.plan_sha256,
            "started_utc": self.started_utc,
            "finished_utc": datetime.now(timezone.utc).isoformat(),
            "samples_published": self.samples_published,
            "max_abs_observed_speed_mps": self.max_abs_observed_speed_mps,
            "safety_reason_counts": dict(sorted(self.safety_reason_counts.items())),
        }
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.result_path.with_name(self.result_path.name + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.result_path)
        self._result_written = True
        self._publish_status(status, reason or "complete")
        rclpy.shutdown()


def main(args=None) -> int:
    rclpy.init(args=args)
    node: CalibrationExcitationNode | None = None
    try:
        node = CalibrationExcitationNode()
        rclpy.spin(node)
        return node.exit_code
    except KeyboardInterrupt:
        if node is not None:
            node._begin_abort("keyboard_interrupt")
            deadline = time.monotonic() + node.abort_stop_hold_sec
            while rclpy.ok() and time.monotonic() < deadline:
                node._publish_command(node._stop_command())
                rclpy.spin_once(node, timeout_sec=0.05)
            node._finish("aborted", 130, "keyboard_interrupt")
            return node.exit_code
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
