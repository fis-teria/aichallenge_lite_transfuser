"""Trial-only map localizer plus read-only evaluation journal, never control.

Run inside the trial's isolated official ROS container. Ground truth and the
existing Autoware pose are recorded only here; neither is sent to localization.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--initial-pose', type=float, nargs=3, required=True)
    args = parser.parse_args()
    course = Path('/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm')
    command = [sys.executable, '-m', 'aic_e2e_runtime.lidar_map_localization_node',
               '--map', str(course), '--initial-pose', *map(str, args.initial_pose),
               '--correction-mode', 'simulation_aggressive',
               '--output', str(args.output/'localization_status.jsonl')]
    journal = (args.output/'localization_observations.jsonl').open('x', buffering=1)
    localizer_log = (args.output/'localizer.log').open('x')
    localizer = subprocess.Popen(command, stdout=localizer_log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = Node('time_lidar_map_evaluation_observer')
    counts: dict[str, int] = {}
    subscribed: set[str] = set()
    status: dict = {}
    topics = ('/time_path/localization/pose', '/awsim/ground_truth/vehicle/pose', '/localization/pose')

    def write(row: dict) -> None:
        journal.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(), **row), allow_nan=False)+'\n')

    def on_pose(msg, topic: str) -> None:
        counts[topic] = counts.get(topic, 0)+1
        pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
        p, q = pose.position, pose.orientation
        xyzq = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in xyzq):
            write(dict(kind='invalid_pose', topic=topic)); return
        write(dict(kind='pose', topic=topic, frame=msg.header.frame_id,
                   stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                   position_m=xyzq[:3], quaternion_xyzw=xyzq[3:]))

    def on_scan(msg) -> None:
        counts['corrected_scan'] = counts.get('corrected_scan', 0)+1
        write(dict(kind='corrected_scan', stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                   frame=msg.header.frame_id, beams=len(msg.ranges)))

    def on_status(msg) -> None:
        nonlocal status
        status = json.loads(msg.data)

    node.create_subscription(LaserScan, '/time_path/localization/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(String, '/time_path/localization/status', on_status, 10)

    def snapshot() -> None:
        if localizer.poll() is not None:
            raise RuntimeError('LOCALIZER_EXIT:'+str(localizer.returncode))
        available = dict(node.get_topic_names_and_types())
        for topic in topics:
            if topic not in subscribed and topic in available:
                types = available[topic]
                if len(types) != 1 or types[0] not in (
                    'geometry_msgs/msg/PoseStamped', 'geometry_msgs/msg/PoseWithCovarianceStamped'):
                    raise ValueError('UNSUPPORTED_EVALUATION_POSE_TYPE:'+topic+':'+repr(types))
                node.create_subscription(get_message(types[0]), topic,
                    lambda msg, topic=topic: on_pose(msg, topic), qos_profile_sensor_data)
                subscribed.add(topic)
        graph = {}
        for name in ('time_path_controller', 'time_path_inference', 'time_lidar_map_localizer'):
            try:
                graph[name] = dict(subscriptions=node.get_subscriber_names_and_types_by_node(name, '/'),
                                   publishers=node.get_publisher_names_and_types_by_node(name, '/'))
            except rclpy.node.NodeNameNonExistentError:
                continue
        rviz = [entry.node_name for entry in node.get_subscriptions_info_by_topic('/time_path/localization/scan')
                if entry.node_name.startswith('rviz')]
        value = dict(counts=counts, status=status, graph=graph, rviz_corrected_scan_subscribers=rviz,
                     available_evaluation_topics={t: available.get(t) for t in topics},
                     correction_scope='DISPLAY_AND_EVALUATION_ONLY_NO_CONTROL_INPUT')
        write(dict(kind='graph', **value))
        pending = args.output/'localization_observer.pending'
        pending.write_text(json.dumps(value, indent=2))
        pending.replace(args.output/'localization_observer.json')
        ready = args.output/'localization_ready.json'
        if not ready.exists() and rviz and counts.get('corrected_scan', 0) >= 3 and status.get('valid'):
            expected = {
                'time_path_controller': {'/clock', '/sensing/lidar/scan', '/time_path/plan',
                    '/vehicle/status/steering_status', '/vehicle/status/velocity_status'},
                'time_lidar_map_localizer': {'/clock', '/sensing/lidar/scan', '/time_path/wheel_odometry',
                    '/time_path/localization/initialpose'},
            }
            for name, allowed in expected.items():
                actual = {t for t, _ in graph.get(name, {}).get('subscriptions', [])}
                if actual != allowed:
                    raise RuntimeError('UNEXPECTED_INPUT_GRAPH:'+name+':'+repr(actual))
            ready.write_text(json.dumps(value, indent=2))

    node.create_timer(1., snapshot)

    def stop(signum, frame) -> None:
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        localizer.terminate()
        try:
            localizer.wait(timeout=1)
        except subprocess.TimeoutExpired:
            localizer.kill(); localizer.wait()
        journal.close(); localizer_log.close()
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
