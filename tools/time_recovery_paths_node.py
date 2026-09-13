"""Publish ordinary RViz Paths in a separate process from collection control."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from aic_transfuser_lite.runtime.path_trace_v1 import ObservedPathTrace


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args()
    reference = json.loads(args.reference.read_text())
    fixed = {'baseline': reference['baseline_xy_m'], 'reference': reference['reference_xy_m']}
    for points in fixed.values():
        if len(points) < 20 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('DISPLAY_REFERENCE_SHAPE_FINITE')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = Node('time_recovery_paths', enable_rosout=False, start_parameter_services=False)
    publishers = {name: node.create_publisher(RosPath, '/recovery_teacher/'+name+'_path', 1)
                  for name in (*fixed, 'observed')}
    trace = ObservedPathTrace()
    published = 0

    def receive(message) -> None:
        if message.header.frame_id != 'map' or message.child_frame_id != 'base_link':
            raise ValueError('DISPLAY_POSE_FRAME')
        stamp = message.header.stamp
        p = message.pose.pose.position
        trace.add(int(stamp.sec)*10**9+int(stamp.nanosec), (p.x, p.y))

    def publish() -> None:
        nonlocal published
        if trace.last_stamp_ns is None:
            return
        counts = {}
        for name, points in (*fixed.items(), ('observed', trace.points)):
            path = RosPath(); path.header.frame_id = 'map'
            path.header.stamp.sec = trace.last_stamp_ns//10**9
            path.header.stamp.nanosec = trace.last_stamp_ns%10**9
            for x, y in points:
                p = PoseStamped(); p.header = path.header
                p.pose.position.x = float(x); p.pose.position.y = float(y); p.pose.orientation.w = 1.
                path.poses.append(p)
            publishers[name].publish(path); counts[name] = len(path.poses)
        published += 1
        pending = args.output/'path_heartbeat.pending'
        pending.write_text(json.dumps(dict(monotonic_ns=time.monotonic_ns(), published=published,
            point_counts=counts, frame_id='map', observed_spacing_m=trace.spacing_m)))
        pending.replace(args.output/'path_heartbeat.json')

    node.create_subscription(Odometry, '/localization/kinematic_state', receive, qos_profile_sensor_data)
    node.create_timer(.5, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
