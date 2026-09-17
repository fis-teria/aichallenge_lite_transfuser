"""Finite ROS-message/TF smoke; no simulator, driving command or native V2X."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from v2x_msgs.msg import V2XVehiclePositionArray

from aic_lidar_v2x.node import LidarV2XNode, fill_stamp


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="lidar_v2x_smoke_") as folder:
        root = Path(folder)
        Image.fromarray(np.full((400, 400), 255, dtype=np.uint8)).save(root / "map.pgm")
        (root / "map.yaml").write_text("image: map.pgm\nresolution: 0.1\norigin: [-20, -20, 0]\nnegate: 0\nfree_thresh: 0.196\noccupied_thresh: 0.65\n")
        rclpy.init(args=["--ros-args", "-p", f"map_yaml:={root / 'map.yaml'}", "-p", "motion_model:=snapshot"])
        detector, probe = LidarV2XNode(), Node("lidar_v2x_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(detector)
        executor.add_node(probe)
        scan_pub = probe.create_publisher(LaserScan, "/sensing/lidar/scan", qos_profile_sensor_data)
        tf_pub, static_pub = TransformBroadcaster(probe), StaticTransformBroadcaster(probe)
        messages, statuses = [], []
        probe.create_subscription(V2XVehiclePositionArray, "/collection/lidar_v2x/vehicle_positions", messages.append, 10)
        probe.create_subscription(String, "/collection/lidar_v2x/status", lambda m: statuses.append(json.loads(m.data)), 100)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = "base_link", "lidar"
        mount.transform.rotation.w = 1.0
        static_pub.sendTransform(mount)
        start = time.monotonic()
        invalid_phase_count = None
        missing_tf_count = None
        try:
            while (elapsed := time.monotonic() - start) < 5.0:
                now = probe.get_clock().now().nanoseconds * 1e-9
                if elapsed >= .8:
                    tf = TransformStamped()
                    tf.header.frame_id, tf.child_frame_id = "map", "base_link"
                    fill_stamp(tf.header.stamp, now)
                    tf.transform.rotation.w = 1.0
                    tf_pub.sendTransform(tf)
                if missing_tf_count is None and elapsed >= .7:
                    missing_tf_count = len(messages)
                if elapsed < 3.8:
                    scan = LaserScan()
                    scan.header.frame_id = "lidar"
                    fill_stamp(scan.header.stamp, now - .06)
                    scan.angle_min, scan.angle_max, scan.angle_increment = -1.5, 1.5, 3.0/749
                    scan.range_min, scan.range_max, scan.scan_time = 0.0, 25.0, .02
                    angles = scan.angle_min + np.arange(750) * scan.angle_increment
                    ranges = np.full(750, np.inf)
                    visible = np.abs(angles) < .1
                    ranges[visible] = 5. / np.cos(angles[visible])
                    if elapsed >= 3.0:
                        ranges[:] = np.nan
                    scan.ranges = ranges.astype(np.float32).tolist()
                    scan_pub.publish(scan)
                if invalid_phase_count is None and elapsed >= 3.3:
                    invalid_phase_count = len(messages)
                executor.spin_once(timeout_sec=.01)
                time.sleep(.01)
            for _ in range(20):
                executor.spin_once(timeout_sec=.01)
            nonempty = [m for m in messages if m.vehicles]
            assert missing_tf_count == 0, "Published before map TF was available"
            assert len(nonempty) >= 5, "No confirmed object delivered through ROS"
            assert len(messages) == invalid_phase_count, "Invalid/stale scans published fresh empty arrays"
            assert all(m.header.frame_id == "map" for m in messages)
            ids = {v.vehicle_id for m in nonempty for v in m.vehicles}
            assert len(ids) == 1 and next(iter(ids)).startswith("lidar_")
            assert abs(nonempty[-1].vehicles[0].position.x - 5.) < .05
            assert nonempty[-1].vehicles[0].covariance.x == .15
            reasons = sorted({s["reason"] for s in statuses})
            assert "invalid_perception_input" in reasons and "scan_timeout" in reasons
            assert all(s["teacher_ready"] is False for s in statuses)
            assert probe.count_publishers("/v2x/vehicle_positions") == 0
            print(json.dumps(dict(passed=True, arrays=len(messages), nonempty_arrays=len(nonempty),
                track_ids=sorted(ids), reasons=reasons, native_v2x_publishers=0, driving_publishers=0)), flush=True)
        finally:
            executor.shutdown()
            detector.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
