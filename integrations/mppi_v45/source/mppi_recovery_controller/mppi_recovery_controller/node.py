"""One final command publisher for normal CMA tracking and MPC wall recovery."""

from pathlib import Path
from dataclasses import replace
import math

from ament_index_python.packages import get_package_share_directory
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport, SteeringReport
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool
from v2x_msgs.msg import V2XVehiclePositionArray
import yaml

from multi_purpose_mpc_ros.boost_logic import update_race_start_gate
from multi_purpose_mpc_ros.v2x_vehicle_tracker import V2XVehicleTracker
from .core import Ego, RecoveryAdapter, RecoveryConfig
from .collection_intent import fresh_forward_speed_mps
from .geometry import reference_samples, wall_snapshot, yaw_from_quaternion
from .reference_builder import ReferenceBuilder
from .real_start import RealVehicleRecoveryGate


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class MppiRecoveryController(Node):
    def __init__(self):
        super().__init__("mppi_recovery_controller")
        self.simulation = bool(self.declare_parameter("simulation", True).value)
        config_path = self.declare_parameter("mpc_config_path", str(
            Path(get_package_share_directory("multi_purpose_mpc_ros")) / "config/config.yaml"
        )).value
        with open(config_path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.config = RecoveryConfig.from_mpc_config(config)
        if not self.simulation:
            propulsion = float(self.declare_parameter('maximum_recovery_propulsion_mps2', 0.5).value)
            recovery_speed = float(self.declare_parameter('recovery_speed_mps', 0.5).value)
            ratio = float(self.declare_parameter('steering_command_to_tire_angle_ratio', 0.60).value)
            tire_limit = float(self.declare_parameter('maximum_tire_steering_angle_rad', math.pi / 10).value)
            tolerance = float(self.declare_parameter('steering_settle_tolerance_rad', 0.03).value)
            if not all(math.isfinite(v) and v > 0 for v in (propulsion, recovery_speed, ratio, tire_limit, tolerance)):
                raise ValueError('real recovery limits must be finite and positive')
            self.config = replace(self.config, maximum_recovery_propulsion_mps2=propulsion,
                reverse_speed_mps=-recovery_speed, forward_speed_mps=recovery_speed,
                short_speed_mps=min(self.config.short_speed_mps, recovery_speed),
                require_steering_feedback=True, tire_to_command_ratio=ratio,
                maximum_tire_angle_rad=tire_limit, steering_settle_tolerance_rad=tolerance)
        self.adapter = RecoveryAdapter(self.config, drive_gear=GearCommand.DRIVE,
                                       reverse_gear=GearCommand.REVERSE)
        self.own_vehicle_id = self.declare_parameter("own_vehicle_id", "").value
        self.input_timeout = float(self.declare_parameter("input_timeout_sec", 0.25).value)
        self.collection_planned_stop_gate = bool(
            self.declare_parameter("collection_planned_stop_gate", False).value)
        self._last_auto_speed_mps = None
        self.real_start = RealVehicleRecoveryGate(
            self.config.min_forward_command_speed, self.config.stop_speed_threshold,
            int(self.input_timeout * 1e9))
        self._last_mode_report_ns = None
        self._last_gear_report_ns = None
        self._real_gear = None
        v2x_config = config["v2x_obstacle_avoidance"]
        self.tracker = V2XVehicleTracker(
            v2x_config["v_max_safety"], v2x_config["position_jump_threshold"],
            warn_callback=self.get_logger().warn)
        self.odometry = None
        self.steering = None
        self.samples = None
        self.wall_map = None
        self.reference = None
        self.reference_builder = ReferenceBuilder(self.config.max_width_m)
        self.race_started = self.ready_seen = False
        self.vehicle_state = ""
        self.control_enabled = True
        self._wanted_pause = False
        self._applied_pause = None
        self._sync = None
        self._normal_after_ns = 0
        self._last_auto_ns = None
        self._last_timer_ns = None
        self._last_service_warning_sec = -10.0
        self.command_pub = self.create_publisher(AckermannControlCommand, "output/control_cmd", 1)
        self.gear_pub = self.create_publisher(GearCommand, "output/gear_cmd", 10)
        self.status_pub = self.create_publisher(String, "debug/status", 10)
        self.pause_clients = [self.create_client(SetBool, name) for name in
                              ("planner/set_recovery_active", "tracker/set_recovery_active")]
        self.create_subscription(AckermannControlCommand, "input/auto_cmd", self.on_auto, 1)
        self.create_subscription(Odometry, "input/odometry", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(Trajectory, "input/reference", self.on_reference, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "input/wall_map", self.on_wall_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(V2XVehiclePositionArray, "input/vehicles", self.tracker.update,
                                 qos_profile_sensor_data)
        if self.simulation:
            self.create_subscription(String, "input/race_state", self.on_race_state, 10)
        else:
            self.create_subscription(SteeringReport, 'input/steering_status', self.on_steering, qos_profile_sensor_data)
            self.create_subscription(ControlModeReport, "input/control_mode", self.on_control_mode,
                                     qos_profile_sensor_data)
            self.create_subscription(GearReport, "input/gear_status", self.on_gear_status,
                                     qos_profile_sensor_data)
        self.create_subscription(Empty, "input/stop_request", self.on_stop, 1)
        self.create_timer(0.025, self.on_timer)
        self.get_logger().info(
            f"MPC recovery enabled={self.config.enabled}, stop={self.config.stuck_detection_sec}s, "
            f"reverse={self.config.reverse_speed_mps}m/s for <= {self.config.backing_up_timeout_sec}s")

    def on_odometry(self, message):
        self.odometry = message
        if not self.simulation and self.real_start.observe_speed(
                message.twist.twist.linear.x, stamp_ns(message.header.stamp),
                self.get_clock().now().nanoseconds):
            self.get_logger().info("[MPPI_REAL_START] recovery armed after autonomous forward motion")

    def on_steering(self, message):
        if self.steering is None or stamp_ns(message.stamp) >= stamp_ns(self.steering.stamp):
            self.steering = message

    def on_control_mode(self, message):
        report_ns = stamp_ns(message.stamp)
        if self._last_mode_report_ns is not None and report_ns < self._last_mode_report_ns:
            return
        self._last_mode_report_ns = report_ns
        now = self.get_clock().now()
        if not self.real_start.set_autonomous(message.mode == ControlModeReport.AUTONOMOUS,
                                              now.nanoseconds):
            return
        # Clear the whole recovery episode, including queued handback state.
        # Lap/rank tracking belongs to the reference node and is untouched.
        self.adapter = RecoveryAdapter(self.config, drive_gear=GearCommand.DRIVE,
                                       reverse_gear=GearCommand.REVERSE)
        self._wanted_pause = False
        self._applied_pause = None
        self._normal_after_ns = now.nanoseconds
        self._last_auto_ns = None
        self.publish_stop(now.to_msg())
        if self.real_start.autonomous:
            self.publish_drive(now.to_msg())
        self.get_logger().info(
            f"[MPPI_REAL_START] autonomous={int(self.real_start.autonomous)} recovery disarmed")

    def on_gear_status(self, message):
        report_ns = stamp_ns(message.stamp)
        if self._last_gear_report_ns is not None and report_ns < self._last_gear_report_ns:
            return
        self._last_gear_report_ns = report_ns
        self._real_gear = message.report
        self.real_start.observe_gear(message.report == GearReport.DRIVE, report_ns)

    def publish_drive(self, stamp):
        message = GearCommand()
        message.stamp, message.command = stamp, GearCommand.DRIVE
        self.gear_pub.publish(message)

    def on_reference(self, message):
        samples = reference_samples(message)
        if samples != self.samples:
            self.samples = samples
            self.rebuild_reference()

    def on_wall_map(self, message):
        snapshot = wall_snapshot(message)
        if snapshot == self.wall_map:
            return
        self.wall_map = snapshot
        self.rebuild_reference()

    def rebuild_reference(self):
        self.reference = self.reference_builder.request(self.samples, self.wall_map)
        self.log_reference_build()

    def log_reference_build(self):
        builder = self.reference_builder
        self.get_logger().info(
            f"[MPPI_REFERENCE_CACHE] map={builder.map_generation} ready={int(self.reference is not None)} "
            f"submitted={builder.submitted} completed={builder.completed} hits={builder.cache_hits} "
            f"worker={builder.worker_pid} build_ms={builder.build_ms}")

    def destroy_node(self):
        self.reference_builder.close()
        return super().destroy_node()

    def on_race_state(self, message):
        if not self.simulation:
            return
        state = message.data.strip().lower()
        if state != self.vehicle_state:
            self.race_started, self.ready_seen = update_race_start_gate(
                self.race_started, self.ready_seen, state, self.vehicle_state)
            self.vehicle_state = state

    def on_stop(self, _message):
        self.control_enabled = False

    @property
    def normal_ready(self):
        return (not self.adapter.active and not self._wanted_pause
                and self._applied_pause is False and self._sync is None
                and self.control_enabled
                and (self.simulation or self.real_start.can_drive))

    def on_auto(self, message):
        now_ns = self.get_clock().now().nanoseconds
        command_ns = stamp_ns(message.stamp)
        if (self.normal_ready and command_ns >= self._normal_after_ns
                and 0 <= now_ns - command_ns <= self.input_timeout * 1e9):
            self.command_pub.publish(message)
            self._last_auto_ns = command_ns
            self._last_auto_speed_mps = message.longitudinal.speed
            if not self.simulation:
                self.real_start.forwarded_command(message.longitudinal.speed, command_ns)

    def sync_controllers(self, now_ns):
        if self._sync is not None:
            requested, futures = self._sync
            if not all(future.done() for future in futures):
                return
            self._sync = None
            try:
                responses = [future.result() for future in futures]
                if not all(response.success for response in responses):
                    raise RuntimeError("; ".join(response.message for response in responses))
            except Exception as error:
                self.get_logger().error(f"Recovery controller reset failed: {error}")
                self._applied_pause = None
                return
            self._applied_pause = requested
            if not requested:
                # CMA's service also rejects pre-reset trajectory stamps. This
                # excludes its already queued commands at the final publisher.
                self._normal_after_ns = now_ns
                self._last_auto_ns = None
        if self._applied_pause != self._wanted_pause:
            if not all(client.service_is_ready() for client in self.pause_clients):
                now_sec = now_ns * 1e-9
                if now_sec - self._last_service_warning_sec >= 5.0:
                    self.get_logger().warn("Waiting for MPPI/CMA recovery reset services")
                    self._last_service_warning_sec = now_sec
                return
            requested = self._wanted_pause
            self._sync = (requested, [client.call_async(SetBool.Request(data=requested))
                                      for client in self.pause_clients])

    def publish_stop(self, stamp):
        command = AckermannControlCommand()
        command.stamp = command.lateral.stamp = command.longitudinal.stamp = stamp
        command.longitudinal.acceleration = float(self.config.acceleration_min_mps2)
        self.command_pub.publish(command)

    def on_timer(self):
        reference = self.reference_builder.poll()
        if reference is not self.reference:
            self.reference = reference
            self.log_reference_build()
        if self.reference_builder.error is not None:
            self.get_logger().error(f"Reference build failed: {self.reference_builder.error}")
            self.reference_builder.error = None
        now = self.get_clock().now()
        if self._last_timer_ns is not None and now.nanoseconds < self._last_timer_ns:
            self._applied_pause = None
            self._normal_after_ns = 0
            self._last_auto_ns = None
            if not self.simulation:
                self.real_start.set_autonomous(None, now.nanoseconds)
                self._last_mode_report_ns = self._last_gear_report_ns = None
                self.adapter = RecoveryAdapter(self.config, drive_gear=GearCommand.DRIVE,
                                               reverse_gear=GearCommand.REVERSE)
                self._wanted_pause = False
        self._last_timer_ns = now.nanoseconds
        self.sync_controllers(now.nanoseconds)
        if (not self.simulation and self.real_start.autonomous and
                not self.real_start.drive_confirmed and not self.adapter.active):
            self.publish_drive(now.to_msg())
        fresh = (self.odometry is not None and
                 0 <= now.nanoseconds - stamp_ns(self.odometry.header.stamp) <= self.input_timeout * 1e9)
        if (fresh and self.reference is not None and
                (self.simulation or self.real_start.armed)):
            pose = self.odometry.pose.pose
            ego = Ego(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation),
                      self.odometry.twist.twist.linear.x)
            opponents = []
            for vehicle_id in self.tracker.active_vehicle_ids():
                if vehicle_id != self.own_vehicle_id:
                    opponents.extend(self.tracker.predict_positions(vehicle_id, [0.0]))
            tick = self.adapter.prepare(
                now_sec=now.nanoseconds * 1e-9, ego=ego, reference=self.reference, opponents=opponents,
                measured_tire_angle_rad=(self.steering.steering_tire_angle if self.steering is not None
                    and 0 <= now.nanoseconds - stamp_ns(self.steering.stamp) <= self.input_timeout * 1e9 else None),
                recovery_allowed=((self.race_started if self.simulation else self.real_start.armed)
                                  and self.control_enabled
                                  and (self.adapter.active or self.normal_ready)),
                commanded_forward_speed_mps=(fresh_forward_speed_mps(
                    now_sec=now.nanoseconds * 1e-9,
                    command_sec=(self._last_auto_ns * 1e-9 if self._last_auto_ns is not None else None),
                    speed_mps=self._last_auto_speed_mps, timeout_sec=self.input_timeout)
                    if self.collection_planned_stop_gate else None),
            )
            if tick.decision is not None:
                if tick.execution.gear_command is not None:
                    gear = GearCommand()
                    gear.stamp, gear.command = now.to_msg(), tick.execution.gear_command
                    self.gear_pub.publish(gear)
                command = AckermannControlCommand()
                command.stamp = command.lateral.stamp = command.longitudinal.stamp = now.to_msg()
                command.longitudinal.speed = tick.decision.published_speed_mps
                command.longitudinal.acceleration = tick.decision.published_acceleration_mps2
                command.lateral.steering_tire_angle = (
                    tick.decision.steering_rad * self.config.steering_publish_gain)
                if not self.simulation and self._real_gear != tick.execution.after.active_gear:
                    # Gear and control are independent on the real vehicle.
                    # Do not apply a gear-relative movement command in the old gear.
                    gear = GearCommand()
                    gear.stamp, gear.command = now.to_msg(), tick.execution.after.active_gear
                    self.gear_pub.publish(gear)
                    self.publish_stop(now.to_msg())
                else:
                    self.command_pub.publish(command)
            self.adapter.commit(tick)
            self._wanted_pause = tick.active
            if tick.update.before.state != tick.update.after.state:
                status = (f"{tick.update.before.state}->{tick.update.after.state} "
                          f"speed={ego.speed:.3f} rear_wall={tick.rear_wall.status} "
                          f"wall_violation={tick.rear_wall.current_violation_m:.3f}->"
                          f"{tick.rear_wall.probe_violation_m:.3f} short={int(tick.short_mode)}")
                self.get_logger().info(f"[MPPI_STUCK_RECOVERY] {status}")
                self.status_pub.publish(String(data=status))
            if tick.motion is not None:
                self.get_logger().info(
                    f"[MPPI_SHORT_RECOVERY] state={tick.update.after.state} "
                    f"available_m={tick.motion.available_m:.3f} "
                    f"speed_limit={tick.motion.speed_limit_mps:.3f} "
                    f"tire_steer={tick.motion.tire_steer_rad:.3f}",
                    throttle_duration_sec=1.0)
            if tick.decision is not None:
                self.sync_controllers(now.nanoseconds)
                return
        if (not fresh or not self.normal_ready or self._last_auto_ns is None
                or now.nanoseconds - self._last_auto_ns > self.input_timeout * 1e9):
            self.publish_stop(now.to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = MppiRecoveryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop(node.get_clock().now().to_msg())
        node.destroy_node()
        rclpy.shutdown()
