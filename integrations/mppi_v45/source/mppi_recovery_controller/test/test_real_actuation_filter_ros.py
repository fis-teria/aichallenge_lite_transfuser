"""Real ROS transport and installed acceleration-converter integration."""

from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tier4_vehicle_msgs')
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from tier4_vehicle_msgs.msg import ActuationCommandStamped

from mppi_recovery_controller.real_actuation_filter import RealActuationFilter


@contextmanager
def fixture():
    rclpy.init()
    node = RealActuationFilter()
    probe = rclpy.create_node('real_actuation_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    output, status, upstream = [], [], []
    probe.create_subscription(ActuationCommandStamped, 'output/actuation_cmd', output.append, 50)
    probe.create_subscription(ActuationCommandStamped, 'input/actuation_cmd', upstream.append, 50)
    probe.create_subscription(String, 'debug/status', lambda m: status.append(json.loads(m.data)), 50)
    pubs = {
        'control': probe.create_publisher(AckermannControlCommand, 'input/control_cmd', 1),
        'actuation': probe.create_publisher(ActuationCommandStamped, 'input/actuation_cmd', 1),
        'odom': probe.create_publisher(Odometry, 'input/odometry', 1),
        'mode': probe.create_publisher(ControlModeReport, 'input/control_mode', 10),
        'gear': probe.create_publisher(GearReport, 'input/gear_status', 10),
        'steer': probe.create_publisher(SteeringReport, 'input/steering_status', 10),
    }
    def publish(x=0.0, speed=1.0, throttle=0.1, gear=GearReport.DRIVE,
                raw=True, mode=ControlModeReport.AUTONOMOUS, control_stamp=None):
        stamp = probe.get_clock().now().to_msg()
        pubs['mode'].publish(ControlModeReport(stamp=stamp, mode=mode))
        pubs['gear'].publish(GearReport(stamp=stamp, report=gear))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.w = 1.0
        pubs['odom'].publish(odom)
        pubs['steer'].publish(SteeringReport(stamp=stamp, steering_tire_angle=0.12))
        control = AckermannControlCommand()
        control.stamp = control_stamp or stamp
        control.longitudinal.speed = speed
        control.longitudinal.acceleration = 0.0
        control.lateral.steering_tire_angle = 0.12
        pubs['control'].publish(control)
        if raw:
            actuation = ActuationCommandStamped()
            actuation.header.stamp = stamp
            actuation.actuation.accel_cmd = throttle
            actuation.actuation.brake_cmd = 0.25
            actuation.actuation.steer_cmd = 0.12
            pubs['actuation'].publish(actuation)
    def spin(duration, callback=publish, until=lambda: False):
        end, next_publish = time.monotonic() + duration, 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                callback()
                next_publish = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.003)
            if until():
                return True
        return until()
    try:
        yield node, probe, output, status, upstream, publish, spin
    finally:
        executor.remove_node(node)
        executor.remove_node(probe)
        node.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_stationary_start_cuts_converted_throttle_preserving_steer_and_brake(record_property):
    with fixture() as (node, probe, output, status, upstream, publish, spin):
        assert spin(3, until=lambda: any(m.actuation.accel_cmd > 0 for m in output))
        first = next(m for m in output if m.actuation.accel_cmd > 0)
        t0 = first.header.stamp.sec + first.header.stamp.nanosec * 1e-9
        assert spin(1, until=lambda: any(s['reason'] == 'no_progress' for s in status))
        cut = next(s for s in status if s['reason'] == 'no_progress')
        record_property('cutoff_latency_sec', cut['stamp'] - t0)
        assert cut['stamp'] - t0 <= 0.7
        start = len(output)
        spin(0.2)
        assert output[start:] and all(m.actuation.accel_cmd == 0 for m in output[start:])
        assert all(m.actuation.brake_cmd == pytest.approx(0.25) and
                   m.actuation.steer_cmd == pytest.approx(0.12) for m in output[start:])
        assert node.guard.blocked_directions == {1}


def test_stop_command_closes_map_creep_and_reverse_requires_new_commands():
    with fixture() as (node, probe, output, status, upstream, publish, spin):
        assert spin(3, callback=lambda: publish(speed=0.0), until=lambda: len(output) >= 3)
        assert all(m.actuation.accel_cmd == 0 for m in output)
        assert spin(1, until=lambda: any(s['reason'] == 'no_progress' for s in status))
        old_stamp = probe.get_clock().now().to_msg()
        time.sleep(0.02)
        begin = len(output)
        spin(0.1, lambda: publish(gear=GearReport.REVERSE, control_stamp=old_stamp))
        assert all(m.actuation.accel_cmd == 0 for m in output[begin:])
        begin = len(output)
        assert spin(0.3, lambda: publish(gear=GearReport.REVERSE),
                    until=lambda: any(m.actuation.accel_cmd > 0 for m in output[begin:]))


def test_slow_directional_progress_remains_enabled_and_stale_control_closes():
    with fixture() as (node, probe, output, status, upstream, publish, spin):
        begin = time.monotonic()
        spin(1.0, lambda: publish(x=0.1 * (time.monotonic() - begin)))
        assert any(m.actuation.accel_cmd > 0 for m in output)
        assert not node.guard.blocked_directions
        old_stamp = probe.get_clock().now().to_msg()
        assert spin(0.6, lambda: publish(x=0.1 * (time.monotonic() - begin), control_stamp=old_stamp),
                    until=lambda: status and status[-1]['reason'] == 'input_stale')
        assert output[-1].actuation.accel_cmd == 0


def test_installed_converter_zero_acceleration_maps_to_throttle_then_is_cut(tmp_path, record_property):
    submit = Path(__file__).resolve().parents[2]
    launch = submit / 'aichallenge_submit_launch'
    command = ['ros2', 'launch', 'raw_vehicle_cmd_converter', 'raw_vehicle_converter.launch.xml',
        f'converter_param_path:={launch}/config/converter.param.yaml',
        f'csv_path_accel_map:={launch}/data/accel_map.csv',
        f'csv_path_brake_map:={launch}/data/brake_map.csv',
        'max_throttle:=1.0', 'max_brake:=1.0', 'convert_accel_cmd:=true',
        'convert_brake_cmd:=true', 'convert_steer_cmd:=false',
        'input_control_cmd:=/input/control_cmd', 'input_odometry:=/input/odometry',
        'input_steering:=/input/steering_status', 'output_actuation_cmd:=/input/actuation_cmd']
    with (tmp_path / 'converter.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            with fixture() as (node, probe, output, status, upstream, publish, spin):
                assert spin(8, lambda: publish(raw=False), until=lambda: len(upstream) >= 3), (tmp_path / 'converter.log').read_text()
                assert process.poll() is None, (tmp_path / 'converter.log').read_text()
                assert any(m.actuation.accel_cmd == pytest.approx(0.1) for m in upstream)
                assert spin(2, lambda: publish(raw=False), until=lambda: any(s['reason'] == 'no_progress' for s in status))
                first = next(m for m in output if m.actuation.accel_cmd > 0.0)
                cut = next(s for s in status if s['reason'] == 'no_progress')
                first_time = first.header.stamp.sec + first.header.stamp.nanosec * 1e-9
                record_property('converter_throttle_at_zero_acceleration', first.actuation.accel_cmd)
                record_property('converter_cutoff_latency_sec', cut['stamp'] - first_time)
                assert cut['stamp'] - first_time <= 0.7
                begin = len(output)
                spin(0.15, lambda: publish(raw=False))
                assert output[begin:] and all(m.actuation.accel_cmd == 0 for m in output[begin:])
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
