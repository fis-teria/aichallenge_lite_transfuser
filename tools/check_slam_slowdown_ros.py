"""Isolated real Cartographer + production controller test, shadow commands only."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main() -> None:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    if os.environ.get('ROS_DOMAIN_ID') != '93':
        raise ValueError('ISOLATED_DOMAIN_93_REQUIRED')
    source = Path(__file__).resolve().parents[1]
    config = json.loads((source/'configs/control/time_path_slam_slowdown.json').read_text())
    config['checkpoint_sha256'] = '0'*64
    config_path = args.output/'fixture_config.json'; config_path.write_text(json.dumps(config))
    rclpy.init(); sensor = rclpy.create_node('awsim_d1')
    inference = rclpy.create_node('time_path_inference'); observer = rclpy.create_node('rviz_slowdown_smoke')
    scan_pub = sensor.create_publisher(LaserScan, '/sensing/lidar/scan', qos_profile_sensor_data)
    clock_pub = sensor.create_publisher(Clock, '/clock', 10)
    velocity_pub = sensor.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
    steering_pub = sensor.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
    plan_pub = inference.create_publisher(String, '/time_path/plan', 10)
    commands = []
    observer.create_subscription(AckermannControlCommand, '/time_path/shadow/control_cmd',
                                 lambda m: commands.append(m), 10)
    observer.create_subscription(MarkerArray, '/time_path/slam/markers', lambda m: None, 10)
    processes = []; logs = []
    def launch(command: list[str], name: str):
        log = (args.output/(name+'.log')).open('x'); logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child); return child
    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try: child.wait(timeout=4)
            except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=3)
    next_pub = 0.; t = 1_000_000_027; count = 0
    def cycle(seconds: float, box_range: float | None) -> None:
        nonlocal next_pub, t, count
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                next_pub = time.monotonic()+.02; t += 20_000_000; count += 1
                clock = Clock(); clock.clock.sec, clock.clock.nanosec = divmod(t, 10**9); clock_pub.publish(clock)
                velocity = VelocityReport(); velocity.header.stamp = clock.clock
                velocity.header.frame_id = 'base_link'; velocity.longitudinal_velocity = 2.
                velocity_pub.publish(velocity)
                steering = SteeringReport(); steering.stamp = clock.clock; steering_pub.publish(steering)
                if count % 5 == 0:
                    scan = LaserScan(); scan.header.stamp = clock.clock; scan.header.frame_id = 'lidar'
                    scan.angle_min = -math.pi; scan.angle_increment = 2*math.pi/750
                    scan.angle_max = scan.angle_min+749*scan.angle_increment
                    scan.range_min = .1; scan.range_max = 25.; scan.time_increment = .1/750
                    ranges = []
                    for i in range(750):
                        c, s = math.cos(scan.angle_min+i*scan.angle_increment), math.sin(scan.angle_min+i*scan.angle_increment)
                        ray = min((18. if c >= 0 else 6.)/max(abs(c), 1e-9), 3./max(abs(s), 1e-9))
                        if box_range is not None and c > 0 and abs(box_range*s/c) <= .6:
                            ray = min(ray, box_range/c)
                        ranges.append(ray)
                    scan.ranges = ranges; scan_pub.publish(scan)
                    distances = [(i+1)*.4 for i in range(30)]
                    plan_pub.publish(String(data=json.dumps(dict(event='PLAN', run_id=args.output.name,
                        epoch='0', plan_id=str(t), observation_ns=t, clock='sim', frame='base_link',
                        dt_s=.1, checkpoint_sha256='0'*64, precision='float32', producer_kind='SYNTHETIC_ROS_FIXTURE',
                        raw_xy_m=[[math.sin(.01*d)/.01, (1-math.cos(.01*d))/.01] for d in distances]))))
            for node in (sensor, inference, observer): rclpy.spin_once(node, timeout_sec=.001)
            assert not observer.get_publishers_info_by_topic('/control/command/control_cmd')
    def records():
        return [json.loads(l) for l in (args.output/'control.jsonl').read_text().splitlines()
                if json.loads(l).get('event') == 'COMMAND_SENT']
    try:
        launch(['ros2', 'run', 'aic_e2e_runtime', 'time_trial_controller_node', '--output', str(args.output),
                '--run-id', args.output.name, '--checkpoint-sha256', '0'*64, '--rear-axle-forward-m',
                '.0010000169277191162', '--trial-config', str(config_path), '--synthetic-shadow-fixture'], 'controller')
        slam = launch(['python3', str(source/'tools/run_slam_obstacle_sidecar.py'), '--output', str(args.output)], 'slam')
        cycle(8., None)
        clear = [r for r in records() if r['details'].get('slam_slowdown', {}).get('valid')]
        assert clear and any(r['target_speed_mps'] > 2. for r in clear), 'NO_CLEAR_SHADOW_COMMAND'
        begin = time.monotonic_ns(); cycle(4., 3.)
        slowed = [r for r in records() if r['monotonic_ns'] > begin and
                  r['details'].get('slam_slowdown', {}).get('reason') == 'OBSTACLE_SPEED_CAP']
        assert slowed and any(r['acceleration_mps2'] < 0 for r in slowed), 'NO_OBSTACLE_DECELERATION'
        begin = time.monotonic_ns(); cycle(3., 1.)
        stopped = [r for r in records() if r['monotonic_ns'] > begin and
                   r['details'].get('slam_slowdown', {}).get('reason') == 'OBSTACLE_STOP']
        assert stopped and all(r['target_speed_mps'] == 0 for r in stopped), 'NO_CLOSE_OBSTACLE_STOP'
        begin = time.monotonic_ns(); cycle(4., None)
        released = [r for r in records() if r['monotonic_ns'] > begin and
                    r['details'].get('slam_slowdown', {}).get('reason') in ('CLEAR_RELEASE', 'CLEAR')]
        assert released and released[-1]['target_speed_mps'] > .5, 'NO_CLEAR_RELEASE'
        stop(slam); begin = time.monotonic_ns(); cycle(1.5, None)
        stale = [r for r in records() if r['monotonic_ns'] > begin+500_000_000 and
                 r['details'].get('slam_slowdown')]
        assert stale and all(r['target_speed_mps'] == 0 and r['acceleration_mps2'] < 0 for r in stale)
        guarded = [r for r in records() if r['details'].get('slam_slowdown')]
        for row in guarded:
            g = row['details']['slam_slowdown']
            assert abs(row['steer_rad']-g['issued_steer_before_rad']) < 1e-12
            assert row['target_speed_mps'] <= g['nominal_target_mps']
            assert row['acceleration_mps2'] <= g['nominal_acceleration_mps2']
        result = dict(status='PASS', scope='SYNTHETIC_REAL_CARTOGRAPHER_REAL_CONTROLLER_SHADOW_ONLY',
                      guarded_commands=len(guarded), slowed_commands=len(slowed), stop_commands=len(stopped),
                      release_commands=len(released), stale_brake_commands=len(stale),
                      steering_identical=True, vehicle_command_publishers=0)
        (args.output/'summary.json').write_text(json.dumps(result, indent=2)); print(json.dumps(result))
    finally:
        for child in reversed(processes): stop(child)
        for log in logs: log.close()
        for node in (sensor, inference, observer): node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
