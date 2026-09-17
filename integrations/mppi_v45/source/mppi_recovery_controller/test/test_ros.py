"""ROS transport checks, including the real CMA controller reset service."""

from dataclasses import replace
from pathlib import Path
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_auto_control_msgs")
from ament_index_python.packages import get_package_prefix
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport, SteeringReport
from multi_purpose_mpc_ros_msgs.msg import StateLatticeDirectTrajectory
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray

from mppi_recovery_controller.core import RecoveryConfig
from mppi_recovery_controller.node import MppiRecoveryController, stamp_ns


def spin(executor, duration, publish=lambda: None, until=lambda: False):
    end, next_publish = time.monotonic() + duration, 0.0
    while time.monotonic() < end:
        if time.monotonic() >= next_publish:
            publish()
            next_publish = time.monotonic() + 0.02
        executor.spin_once(timeout_sec=0.005)
        if until():
            return True
    return until()


def trajectory(stamp):
    message = Trajectory()
    message.header.stamp, message.header.frame_id = stamp, "map"
    for x in range(80):
        point = TrajectoryPoint()
        point.pose.position.x = float(x)
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 3.0
        message.points.append(point)
    return message


def odometry(stamp, speed=0.0):
    message = Odometry()
    message.header.stamp, message.header.frame_id = stamp, "map"
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = speed
    return message


def occupancy(stamp):
    message = OccupancyGrid()
    message.header.stamp, message.header.frame_id = stamp, "map"
    message.info.resolution = 0.5
    message.info.width, message.info.height = 240, 40
    message.info.origin.position.x, message.info.origin.position.y = -20.0, -10.0
    message.info.origin.orientation.w = 1.0
    message.data = [100 if row in (0, 39) else 0 for row in range(40) for col in range(240)]
    return message


def test_commands_continue_while_initial_reference_build_is_pending(monkeypatch):
    from concurrent.futures import Future
    from mppi_recovery_controller.reference_builder import ReferenceBuilder
    import mppi_recovery_controller.node as recovery_node

    class PendingExecutor:
        def submit(self, *_args):
            return Future()

        def shutdown(self, **_kwargs):
            pass

    monkeypatch.setattr(recovery_node, 'ReferenceBuilder',
                        lambda width: ReferenceBuilder(width, executor=PendingExecutor()))
    rclpy.init()
    node = MppiRecoveryController()
    probe = rclpy.create_node('pending_reference_transport_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    received, sent = [], set()

    def reset(_request, response):
        response.success = True
        return response

    services = [probe.create_service(SetBool, name, reset) for name in
                ('planner/set_recovery_active', 'tracker/set_recovery_active')]
    probe.create_subscription(AckermannControlCommand, 'output/control_cmd',
        lambda command: received.append((time.monotonic(), serialize_message(command))), 10)
    auto = probe.create_publisher(AckermannControlCommand, 'input/auto_cmd', 1)
    odom = probe.create_publisher(Odometry, 'input/odometry', 1)

    def publish():
        stamp = probe.get_clock().now().to_msg()
        odom.publish(odometry(stamp, 1.))
        command = AckermannControlCommand()
        command.stamp = stamp
        command.longitudinal.speed = 3.
        command.lateral.steering_tire_angle = .1
        sent.add(serialize_message(command))
        auto.publish(command)

    try:
        assert spin(executor, 2, publish, lambda: node.normal_ready and len(received) > 3)
        node.on_wall_map(occupancy(probe.get_clock().now().to_msg()))
        node.on_reference(trajectory(probe.get_clock().now().to_msg()))
        received.clear()
        spin(executor, .7, publish)
        assert node.reference is None and node.reference_builder.pending is not None
        forwarded = [(t, b) for t, b in received if b in sent]
        assert len(forwarded) >= 15
        assert max(b[0] - a[0] for a, b in zip(forwarded, forwarded[1:])) < .15
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def test_adapter_real_pubsub_gear_handoff_and_queued_command_exclusion(monkeypatch):
    load_config = RecoveryConfig.from_mpc_config
    monkeypatch.setattr(RecoveryConfig, "from_mpc_config", classmethod(
        lambda cls, raw: replace(load_config(raw), stuck_detection_sec=0.15,
                                 backing_up_timeout_sec=0.20, reverse_stop_hold_sec=0.05)))
    rclpy.init()
    node = MppiRecoveryController()
    probe = rclpy.create_node("recovery_transport_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    output, gears, resets, sent = [], [], [], set()

    def reset(request, response):
        resets.append(request.data)
        response.success = True
        return response

    services = [probe.create_service(SetBool, name, reset) for name in
                ("planner/set_recovery_active", "tracker/set_recovery_active")]
    probe.create_subscription(AckermannControlCommand, "output/control_cmd", output.append, 10)
    probe.create_subscription(GearCommand, "output/gear_cmd", gears.append, 10)
    auto = probe.create_publisher(AckermannControlCommand, "input/auto_cmd", 1)
    odom = probe.create_publisher(Odometry, "input/odometry", 1)
    ref = probe.create_publisher(Trajectory, "input/reference", 1)
    wall = probe.create_publisher(OccupancyGrid, "input/wall_map",
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    race = probe.create_publisher(String, "input/race_state", 10)
    stop = probe.create_publisher(Empty, "input/stop_request", 1)
    speed = 1.0
    auto_stamp_override = None

    def publish():
        stamp = probe.get_clock().now().to_msg()
        odom.publish(odometry(stamp, speed))
        command = AckermannControlCommand()
        command.stamp = auto_stamp_override or stamp
        command.longitudinal.speed = 1.5
        command.longitudinal.acceleration = 0.7
        command.lateral.steering_tire_angle = 0.035
        sent.add(serialize_message(command))
        auto.publish(command)

    try:
        wall.publish(occupancy(probe.get_clock().now().to_msg()))
        assert spin(executor, 4, lambda: (publish(), ref.publish(trajectory(probe.get_clock().now().to_msg()))),
                    lambda: node.reference is not None and any(abs(c.longitudinal.acceleration - 0.7) < 1e-5 for c in output))
        forwarded = [c for c in output if abs(c.longitudinal.acceleration - 0.7) < 1e-5]
        assert all(serialize_message(c) in sent for c in forwarded)
        assert not gears
        race.publish(String(data="Ready"))
        assert spin(executor, 1, publish, lambda: node.ready_seen)
        race.publish(String(data="Start"))
        assert spin(executor, 1, publish, lambda: node.race_started)
        speed = 0.0
        assert spin(executor, 2, publish, lambda: node.adapter.recovery.state == "BACKING_UP")
        takeover_ns = probe.get_clock().now().nanoseconds
        spin(executor, 0.10, publish)
        assert any(g.command == GearCommand.REVERSE for g in gears)
        assert any(c.longitudinal.speed == 2.5 and
                   abs(c.longitudinal.acceleration - 1.3) < 1e-5 for c in output)
        assert all(c.longitudinal.acceleration <= 1.3 + 1e-5 for c in output)
        assert not any(stamp_ns(c.stamp) >= takeover_ns and abs(c.longitudinal.acceleration - 0.7) < 1e-5
                       for c in output)
        auto_stamp_override = probe.get_clock().now().to_msg()
        assert spin(executor, 2, publish, lambda: not node.adapter.active and node.normal_ready)
        speed = 1.0
        output.clear()
        spin(executor, 0.10, publish)
        assert not any(abs(c.longitudinal.acceleration - 0.7) < 1e-5 for c in output)
        assert any(g.command == GearCommand.DRIVE for g in gears)
        assert resets.count(True) == 2 and resets.count(False) == 4
        auto_stamp_override = None
        assert spin(executor, 1, publish, lambda: any(abs(c.longitudinal.acceleration - 0.7) < 1e-5 for c in output))
        stop.publish(Empty())
        assert spin(executor, 1, publish, lambda: not node.control_enabled)
        stopped_ns = probe.get_clock().now().nanoseconds
        output.clear()
        spin(executor, 0.10, publish)
        after_stop = [c for c in output if stamp_ns(c.stamp) >= stopped_ns]
        assert after_stop and all(c.longitudinal.speed == 0 for c in after_stop)
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def test_real_mode_motion_gate_and_manual_interruption(monkeypatch):
    load_config = RecoveryConfig.from_mpc_config
    monkeypatch.setattr(RecoveryConfig, "from_mpc_config", classmethod(
        lambda cls, raw: replace(load_config(raw), stuck_detection_sec=0.15,
                                 backing_up_timeout_sec=2.0)))
    rclpy.init(args=["--ros-args", "-p", "simulation:=false",
                     "-r", "input/race_state:=/awsim/state"])
    node = MppiRecoveryController()
    probe = rclpy.create_node("real_recovery_probe", use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    received, gears, sent = [], [], set()
    probe.create_subscription(AckermannControlCommand, "output/control_cmd", received.append, 10)
    probe.create_subscription(GearCommand, "output/gear_cmd", gears.append, 10)
    auto = probe.create_publisher(AckermannControlCommand, "input/auto_cmd", 1)
    odom = probe.create_publisher(Odometry, "input/odometry", 1)
    ref = probe.create_publisher(Trajectory, "input/reference", 1)
    wall = probe.create_publisher(OccupancyGrid, "input/wall_map",
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    modes = probe.create_publisher(ControlModeReport, "input/control_mode", 10)
    reports = probe.create_publisher(GearReport, "input/gear_status", 10)
    steering_reports = probe.create_publisher(SteeringReport, "input/steering_status", 10)
    mode, gear, speed = ControlModeReport.MANUAL, GearReport.DRIVE, 1.0

    def publish():
        stamp = probe.get_clock().now().to_msg()
        modes.publish(ControlModeReport(stamp=stamp, mode=mode))
        reports.publish(GearReport(stamp=stamp, report=gear))
        steering_reports.publish(SteeringReport(stamp=stamp, steering_tire_angle=0.0))
        odom.publish(odometry(stamp, speed))
        ref.publish(trajectory(stamp))
        command = AckermannControlCommand()
        command.stamp = stamp
        command.longitudinal.speed = 1.5
        command.longitudinal.acceleration = 0.7
        sent.add(serialize_message(command))
        auto.publish(command)

    def reset(_request, response):
        response.success = True
        return response

    services = [probe.create_service(SetBool, name, reset) for name in
                ("planner/set_recovery_active", "tracker/set_recovery_active")]
    try:
        assert not node.simulation
        assert all(not subscription.topic_name.startswith("/awsim/")
                   for subscription in node.subscriptions)
        node.on_race_state(String(data="ready"))
        node.on_race_state(String(data="start"))
        assert not node.race_started
        assert not node.normal_ready  # Mode is unknown at startup.
        wall.publish(occupancy(probe.get_clock().now().to_msg()))
        assert spin(executor, 3, publish, lambda: node.reference is not None and
                    node.real_start.autonomous is False)
        spin(executor, 0.8, publish)
        assert not node.real_start.armed and not node.adapter.active
        assert not gears and all(msg.longitudinal.speed == 0 for msg in received)

        mode, gear, speed = ControlModeReport.AUTONOMOUS, GearReport.REVERSE, 0.0
        assert spin(executor, 2, publish, lambda: bool(gears))
        spin(executor, 0.3, publish)
        assert all(msg.command == GearCommand.DRIVE for msg in gears)
        assert not node.normal_ready and not node.real_start.armed
        assert all(msg.longitudinal.speed == 0 for msg in received)

        gear = GearReport.DRIVE
        assert spin(executor, 2, publish, lambda: node.normal_ready and
                    any(msg.longitudinal.speed == 1.5 for msg in received))
        spin(executor, 0.8, publish)
        assert not node.real_start.armed and not node.adapter.active
        assert all(serialize_message(msg) in sent for msg in received
                   if msg.longitudinal.speed == 1.5)

        speed = 1.0
        assert spin(executor, 1, publish, lambda: node.real_start.armed)
        speed = 0.0
        assert spin(executor, 2, publish, lambda: node.adapter.recovery.state == "BACKING_UP")
        assert spin(executor, 1, publish, lambda: any(msg.command == GearCommand.REVERSE for msg in gears))
        waiting_gear_ns = probe.get_clock().now().nanoseconds
        spin(executor, 0.1, publish)
        assert all(msg.longitudinal.speed == 0 for msg in received
                   if stamp_ns(msg.stamp) >= waiting_gear_ns)
        gear = GearReport.REVERSE
        assert spin(executor, 1, publish, lambda: any(
            stamp_ns(msg.stamp) >= waiting_gear_ns and msg.longitudinal.speed == 0.5
            for msg in received))

        mode, gear = ControlModeReport.MANUAL, GearReport.REVERSE
        assert spin(executor, 1, publish, lambda: node.real_start.autonomous is False)
        interrupted_ns = node.real_start.transition_ns
        spin(executor, 0.3, publish)
        assert not node.adapter.active and not node.real_start.armed
        assert not any(stamp_ns(msg.stamp) >= interrupted_ns for msg in gears)
        assert all(msg.longitudinal.speed == 0 for msg in received
                   if stamp_ns(msg.stamp) >= interrupted_ns)

        # Restart at a standstill must not inherit the previous armed latch/timer.
        mode = ControlModeReport.AUTONOMOUS
        assert spin(executor, 1, publish, lambda: node.real_start.autonomous is True)
        gear = GearReport.DRIVE
        assert spin(executor, 1, publish, lambda: node.normal_ready)
        spin(executor, 0.8, publish)
        assert not node.adapter.active and not node.real_start.armed
        speed = 1.0
        assert spin(executor, 1, publish, lambda: node.real_start.armed)
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def test_real_cma_reset_rejects_old_trajectory_and_resumes_fresh(tmp_path):
    rclpy.init()
    probe = rclpy.create_node("cma_recovery_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    binary = Path(get_package_prefix("cma_pure_pursuit")) / "lib/cma_pure_pursuit/cma_pure_pursuit"
    log = (tmp_path / "cma.log").open("w")
    process = subprocess.Popen([str(binary), "--ros-args",
        "-r", "__node:=recovery_cma_test", "-p", "use_atomic_direct_trajectory_command:=true",
        "-p", "recovery_service_enabled:=true", "-p", "use_external_target_vel:=false",
        "-p", "odometry_timeout_sec:=0.25", "-p", "trajectory_timeout_sec:=0.25",
        "-r", "input/kinematics:=/cma_test/odom",
        "-r", "input/direct_trajectory_command:=/cma_test/trajectory",
        "-r", "output/control_cmd:=/cma_test/control"], stdout=log, stderr=subprocess.STDOUT)
    commands = []
    probe.create_subscription(AckermannControlCommand, "/cma_test/control", commands.append, 10)
    odom = probe.create_publisher(Odometry, "/cma_test/odom", 1)
    paths = probe.create_publisher(StateLatticeDirectTrajectory, "/cma_test/trajectory", 1)
    client = probe.create_client(SetBool, "/recovery_cma_test/set_recovery_active")
    generation = 1
    override = None

    def publish():
        now = probe.get_clock().now()
        odom.publish(odometry(now.to_msg()))
        message = StateLatticeDirectTrajectory()
        message.header.stamp = override or now.to_msg()
        message.header.frame_id = "map"
        message.generation = generation
        message.mode = "FREE_RUN"
        message.valid_until_sec = now.nanoseconds * 1e-9 + 0.25
        message.trajectory = trajectory(message.header.stamp)
        paths.publish(message)

    def pause(value):
        future = client.call_async(SetBool.Request(data=value))
        assert spin(executor, 3, publish, future.done)
        assert future.result().success

    try:
        assert spin(executor, 5, publish, lambda: client.service_is_ready() and
                    any(c.longitudinal.speed > 0 for c in commands)), (tmp_path / "cma.log").read_text()[-3000:]
        old_stamp = probe.get_clock().now().to_msg()
        pause(True)
        spin(executor, 0.10, publish)
        commands.clear()
        spin(executor, 0.15, publish)
        assert commands and all(c.longitudinal.speed == 0 for c in commands)
        override, generation = old_stamp, 100
        pause(False)
        spin(executor, 0.10, publish)
        commands.clear()
        spin(executor, 0.20, publish)
        assert commands and all(c.longitudinal.speed == 0 and c.longitudinal.acceleration < 0 for c in commands)
        override, generation = None, 101
        assert spin(executor, 3, publish, lambda: any(c.longitudinal.speed > 0 for c in commands))
    finally:
        process.terminate()
        process.wait(timeout=10)
        log.close()
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()


def test_real_mppi_pause_discards_queued_work_and_restarts(tmp_path):
    rclpy.init()
    probe = rclpy.create_node("mppi_recovery_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    binary = Path(get_package_prefix("reference_space_mppi_planner")) / "lib/reference_space_mppi_planner/reference_space_mppi_node"
    log_path = tmp_path / "mppi.log"
    log = log_path.open("w")
    process = subprocess.Popen([str(binary), "--ros-args",
        "-r", "__node:=recovery_mppi_test", "-p", "enabled:=true",
        "-p", "brain_mode:=true", "-p", "recovery_service_enabled:=true",
        "-p", "brain.internal_timer_enabled:=false",
        "-r", "input/odometry:=/mppi_test/odom",
        "-r", "input/base_reference:=/mppi_test/reference",
        "-r", "input/wall_map:=/mppi_test/map",
        "-r", "input/vehicle_positions:=/mppi_test/vehicles",
        "-r", "input/trajectory_command:=/mppi_test/tick",
        "-r", "output/trajectory_command:=/mppi_test/output"], stdout=log, stderr=subprocess.STDOUT)
    commands = []
    probe.create_subscription(StateLatticeDirectTrajectory, "/mppi_test/output", commands.append, 10)
    odom = probe.create_publisher(Odometry, "/mppi_test/odom", 1)
    ref = probe.create_publisher(Trajectory, "/mppi_test/reference", 1)
    wall = probe.create_publisher(OccupancyGrid, "/mppi_test/map",
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    paths = probe.create_publisher(StateLatticeDirectTrajectory, "/mppi_test/tick", 1)
    vehicles = probe.create_publisher(V2XVehiclePositionArray, "/mppi_test/vehicles", 1)
    client = probe.create_client(SetBool, "/recovery_mppi_test/set_recovery_active")
    generation = 0
    with_opponent = False
    ego_x = 0.0

    def publish():
        nonlocal generation
        now = probe.get_clock().now()
        state = odometry(now.to_msg(), 2.0)
        state.pose.pose.position.x = ego_x
        odom.publish(state)
        ref.publish(trajectory(now.to_msg()))
        message = StateLatticeDirectTrajectory()
        message.header.stamp, message.header.frame_id = now.to_msg(), "map"
        generation += 1
        message.generation = generation
        message.mode = "FREE_RUN"
        message.valid_until_sec = now.nanoseconds * 1e-9 + 1.0
        message.trajectory = trajectory(now.to_msg())
        traffic = V2XVehiclePositionArray()
        if with_opponent:
            car = V2XVehiclePosition()
            car.header.stamp = now.to_msg()
            car.vehicle_id = "opponent"
            car.position.x = 8.0
            traffic.vehicles.append(car)
        vehicles.publish(traffic)
        paths.publish(message)

    def pause(value):
        future = client.call_async(SetBool.Request(data=value))
        assert spin(executor, 5, publish, future.done), log_path.read_text()[-3000:]
        assert future.result().success

    try:
        # MPPI's external map subscription is volatile; wait for discovery
        # before sending its single map sample.
        assert spin(executor, 5, until=lambda: wall.get_subscription_count() > 0)
        wall.publish(occupancy(probe.get_clock().now().to_msg()))
        assert spin(executor, 5, publish, lambda: client.service_is_ready() and bool(commands)), log_path.read_text()[-3000:]
        with_opponent = True
        assert spin(executor, 5, publish, lambda: "[MPPI_TEMPLATE_SELECTION]" in log_path.read_text()), log_path.read_text()[-4000:]
        pause(True)
        spin(executor, 0.15, publish)  # drain output sent before the reset reply
        commands.clear()
        spin(executor, 0.40, publish)
        assert not commands, "paused MPPI must not publish queued or newly requested paths"
        ego_x = 1.0
        resume_ns = probe.get_clock().now().nanoseconds
        pause(False)
        assert spin(executor, 5, publish, lambda: bool(commands)), log_path.read_text()[-3000:]
        assert all(stamp_ns(command.header.stamp) >= resume_ns for command in commands)
        assert all(command.trajectory.points[0].pose.position.x >= 0.999 for command in commands)
        assert spin(executor, 5, publish, lambda: "[MPPI_EXECUTION_APPLIED]" in
                    log_path.read_text().split("[MPPI_STUCK_RECOVERY_RESET] active=0", 1)[-1])
        text = log_path.read_text()
        assert "[MPPI_STUCK_RECOVERY_RESET] active=1" in text
        assert "[MPPI_STUCK_RECOVERY_RESET] active=0" in text
        resumed = text.split("[MPPI_STUCK_RECOVERY_RESET] active=0", 1)[1]
        first_execution = next(line for line in resumed.splitlines() if "[MPPI_EXECUTION]" in line)
        assert "prior_source_generation=0" in first_execution and "action=new" in first_execution
    finally:
        process.terminate()
        process.wait(timeout=10)
        log.close()
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("simulation", [True, False])
def test_production_mppi_launch_has_one_final_publisher_and_connected_resets(tmp_path, simulation):
    rclpy.init()
    probe = rclpy.create_node("mppi_launch_recovery_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    commands = []
    probe.create_subscription(AckermannControlCommand, "/control/command/control_cmd", commands.append, 10)
    log_path = tmp_path / "launch.log"
    log = log_path.open("w")
    process = subprocess.Popen([
        "ros2", "launch", "aichallenge_submit_launch", "mppi.launch.xml",
        "use_sim_time:=false", "vehicle_id:=recovery_probe",
        f"simulation:={str(simulation).lower()}",
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        required = {"/reference_space_mppi_planner/set_recovery_active",
                    "/cma_pure_pursuit_node/set_recovery_active"}
        assert spin(executor, 15, until=lambda: bool(commands) and required.issubset(
            {name for name, _ in probe.get_service_names_and_types()})), log_path.read_text()[-4000:]
        publishers = probe.get_publishers_info_by_topic("/control/command/control_cmd")
        assert [p.node_name for p in publishers] == ["mppi_recovery_controller"]
        assert probe.count_publishers("/mppi/internal/cma_control_cmd") == 1
        assert probe.count_subscribers("/mppi/internal/cma_control_cmd") == 1
        assert probe.count_publishers("/control/command/gear_cmd") == 1
        subscriptions = dict(probe.get_subscriber_names_and_types_by_node("mppi_recovery_controller", "/"))
        assert ("/awsim/state" in subscriptions) == simulation
        assert ("/vehicle/status/control_mode" in subscriptions) == (not simulation)
        assert ("/vehicle/status/gear_status" in subscriptions) == (not simulation)
        assert all(c.longitudinal.speed == 0 for c in commands)
        assert spin(executor, 3, until=lambda: "[MPPI_STUCK_RECOVERY_RESET] active=0" in log_path.read_text())
    finally:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
        log.close()
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()
