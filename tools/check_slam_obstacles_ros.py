"""Finite real-Cartographer synthetic test: stopped obstacle, removal, timeout."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time


def main() -> None:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); args.output.mkdir(exist_ok=False, parents=True)
    rclpy.init()
    sensor = rclpy.create_node('awsim_d1'); wheel = rclpy.create_node('time_path_controller')
    inference = rclpy.create_node('time_path_inference'); observer = rclpy.create_node('rviz_slam_smoke')
    scan_pub = sensor.create_publisher(LaserScan, '/sensing/lidar/scan', qos_profile_sensor_data)
    clock_pub = sensor.create_publisher(Clock, '/clock', 10)
    wheel_pub = wheel.create_publisher(Odometry, '/time_path/wheel_odometry', 10)
    plan_pub = inference.create_publisher(String, '/time_path/plan', 10)
    records = []; statuses = []
    observer.create_subscription(String, '/time_path/slam/obstacles', lambda m: records.append(json.loads(m.data)), 10)
    observer.create_subscription(String, '/time_path/slam/status', lambda m: statuses.append(json.loads(m.data)), 10)
    observer.create_subscription(MarkerArray, '/time_path/slam/markers', lambda m: None, 10)
    t = 1.000000027; next_pub = 0.  # Exercise non-100-ns AWSIM stamps.
    def cycle(duration: float, box: bool, *, scans: bool = True, clock_running: bool = True) -> None:
        nonlocal t, next_pub
        end = time.monotonic()+duration
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_pub:
                next_pub = now+.05
                if clock_running: t += .05
                clock = Clock(); clock.clock.sec = int(t); clock.clock.nanosec = round((t-int(t))*1e9)
                clock_pub.publish(clock)
                if clock_running:
                    odom = Odometry(); odom.header.stamp = clock.clock
                    odom.header.frame_id = 'time_wheel_odom'; odom.child_frame_id = 'base_link'
                    odom.pose.pose.orientation.w = 1.; wheel_pub.publish(odom)
                    packet = dict(event='PLAN', frame='base_link', observation_ns=round(t*1e9),
                                  raw_xy_m=[[i*10/29, 0.] for i in range(30)])
                    plan_pub.publish(String(data=json.dumps(packet)))
                    if scans:
                        msg = LaserScan(); msg.header.stamp = clock.clock; msg.header.frame_id = 'lidar'
                        msg.angle_min = -1.5; msg.angle_increment = 3/749; msg.angle_max = 1.5
                        msg.range_min = 0.; msg.range_max = 25.; msg.scan_time = .05
                        msg.time_increment = .05/750
                        ranges = []
                        for i in range(750):
                            angle = -1.5+i*3/749; c, s = math.cos(angle), math.sin(angle)
                            distance = min(18/c, 3/max(abs(s), 1e-9))
                            if box and abs(5/c*s) <= .6: distance = min(distance, 5/c)
                            ranges.append(float(distance))
                        msg.ranges = ranges; scan_pub.publish(msg)
            for node in (sensor, wheel, inference, observer): rclpy.spin_once(node, timeout_sec=.001)
    log = (args.output/'sidecar.log').open('x')
    process = subprocess.Popen(['python3', str(Path(__file__).with_name('run_slam_obstacle_sidecar.py')),
                                '--output', str(args.output)], stdout=log, stderr=subprocess.STDOUT)
    try:
        cycle(7., False)
        assert process.poll() is None, (args.output/'sidecar.log').read_text()
        assert len(records) >= 10, statuses[-1:] or 'NO_SLAM_OUTPUT'
        assert any(r['path_valid'] and not r['path_blocked'] for r in records[-15:]), records[-1]
        before = len(records); cycle(4., True)
        box_records = records[before:]
        assert any(r['path_blocked'] and any(s['confirmed'] and s['path_overlap'] for s in r['surfaces'])
                   for r in box_records), box_records[-1:]
        before = len(records); cycle(2., False)
        assert records[before:] and any(r['path_valid'] and not r['path_blocked'] for r in records[-8:])
        cycle(1.2, False, scans=False)
        assert not statuses[-1]['input_valid'], statuses[-1]
        count = len(records); cycle(.4, False, scans=False); assert len(records) == count
        cycle(1.5, False); assert statuses[-1]['input_valid'], statuses[-1]
        cycle(.8, False, clock_running=False)
        assert not statuses[-1]['input_valid'] and statuses[-1]['reason'] == 'CLOCK_STALE', statuses[-1]
        assert not observer.get_publishers_info_by_topic('/control/command/control_cmd')
        ready = json.loads((args.output/'slam_obstacles_ready.json').read_text())
        result = dict(status='PASS', real_cartographer=True, processed=len(records),
                      stopped_obstacle_path_overlap_scans=sum(bool(r['path_blocked']) for r in box_records),
                      controller_publishers=0, graph=ready['graph'],
                      checks=['real_scan_wheel_slam', 'stationary_obstacle', 'removal_no_ghost',
                              'scan_timeout', 'clock_timeout', 'no_gnss_imu_global_pose', 'no_commands'])
        (args.output/'summary.json').write_text(json.dumps(result, indent=2)); print(json.dumps(result))
    finally:
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close()
        for node in (sensor, wheel, inference, observer): node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
