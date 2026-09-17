"""Timestamped TF -> LiDAR detections -> dedicated, non-authoritative V2X stream."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray
from visualization_msgs.msg import Marker, MarkerArray

from .core import Config, Detector, Scan, Tracker, v2x_payload
from .io import load_map, load_reference, planar_pose


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fill_stamp(stamp, seconds: float) -> None:
    ns = round(seconds * 1e9)
    stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)


class LidarV2XNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_v2x")
        values = {key: self.declare_parameter(key, value).value for key, value in asdict(Config()).items()}
        self.config = Config(**values)
        map_path = self.declare_parameter("map_yaml", "").value
        reference_path = self.declare_parameter("reference_csv", "").value
        if not map_path:
            raise ValueError("map_yaml is required; missing map must not turn walls into vehicles")
        self.detector = Detector(self.config, load_map(map_path, self.config.wall_margin_m),
                                 load_reference(reference_path) if reference_path else None)
        self.tracker = Tracker(self.config)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        scan_topic = self.declare_parameter("scan_topic", "/sensing/lidar/scan").value
        prefix = self.declare_parameter("output_prefix", "/collection/lidar_v2x").value.rstrip("/")
        if not prefix.startswith("/collection/"):
            raise ValueError("Use a dedicated /collection/ namespace; never publish onto native V2X")
        self.max_age_s = float(self.declare_parameter("max_scan_age_s", 0.30).value)
        self.wall_timeout_s = float(self.declare_parameter("scan_wall_timeout_s", 0.75).value)
        if not all(math.isfinite(x) and x > 0 for x in [self.max_age_s, self.wall_timeout_s]):
            raise ValueError("Scan timeouts must be positive seconds")
        self.v2x_pub = self.create_publisher(V2XVehiclePositionArray, prefix + "/vehicle_positions", 10)
        self.objects_pub = self.create_publisher(String, prefix + "/objects", 10)
        self.status_pub = self.create_publisher(String, prefix + "/status", 10)
        self.markers_pub = self.create_publisher(MarkerArray, prefix + "/markers", 10)
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.pending: deque[tuple[LaserScan, float]] = deque()
        self.last_receive_wall = time.monotonic()
        self.last_processed_wall: float | None = None
        self.last_clock_s: float | None = None
        self.last_status = ""
        self.create_subscription(LaserScan, scan_topic, self.receive_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status(False, "waiting_for_scan")

    def publish_status(self, valid: bool, reason: str, **extra) -> None:
        status = dict(schema_version=1, input_valid=valid, reason=reason, teacher_ready=False,
                      role="SHADOW_UNVALIDATED_FOR_CONTROL", object_model=self.config.object_model,
                      stamp_s=self.get_clock().now().nanoseconds * 1e-9, **extra)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        if reason != self.last_status:
            self.get_logger().info(f"LiDAR V2X: {reason}; teacher_ready=false")
            self.last_status = reason

    def receive_scan(self, msg: LaserScan) -> None:
        self.last_receive_wall = time.monotonic()
        if len(self.pending) >= 8:
            self.pending.popleft()
            self.publish_status(False, "scan_queue_overflow")
        self.pending.append((msg, self.last_receive_wall))

    def lookup_pose(self, source: str, stamp_s: float):
        # Exact measurement time, never Time() / latest TF.
        tr = self.tf.lookup_transform(self.map_frame, source, Time(nanoseconds=round(stamp_s * 1e9)))
        p, q = tr.transform.translation, tr.transform.rotation
        return planar_pose(p.x, p.y, (q.x, q.y, q.z, q.w))

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if self.last_clock_s is not None and now < self.last_clock_s - 0.001:
            self.pending.clear()
            self.tracker.reset()
            self.last_processed_wall = None
            self.publish_status(False, "clock_reset_waiting_for_new_scan")
        self.last_clock_s = now
        if wall - self.last_receive_wall > self.wall_timeout_s:
            self.publish_status(False, "scan_timeout")
            return
        if not self.pending:
            return
        msg, received_wall = self.pending[0]
        start_s = stamp_seconds(msg.header.stamp)
        if now - start_s > self.max_age_s or wall - received_wall > self.wall_timeout_s:
            self.pending.popleft()
            self.publish_status(False, "stale_scan_or_missing_tf")
            return
        if start_s > now + 0.05:
            self.pending.popleft()
            self.publish_status(False, "future_scan")
            return
        if self.tracker.last_stamp_s is not None and start_s <= self.tracker.last_stamp_s:
            self.pending.popleft()
            self.publish_status(False, "nonincreasing_scan")
            return
        try:
            scan = Scan(start_s, np.asarray(msg.ranges), msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, msg.time_increment)
            end_s = start_s + (scan.duration_s if self.config.motion_model == "rolling" else 0.0)
            a = self.lookup_pose(msg.header.frame_id, start_s)
            b = self.lookup_pose(msg.header.frame_id, end_s)
            base = self.lookup_pose(self.base_frame, start_s)
        except TransformException:
            self.publish_status(False, "waiting_for_timestamped_tf")
            return  # Bounded queue/age limit; never substitute identity or latest pose.
        except ValueError as error:
            self.pending.popleft()
            self.publish_status(False, "invalid_scan_or_tf", detail=str(error))
            return
        self.pending.popleft()
        try:
            detections, stats = self.detector.detect(scan, a, b, base)
            tracks = self.tracker.update(detections, start_s)
            payload = v2x_payload(tracks, start_s, self.map_frame)
        except ValueError as error:
            self.publish_status(False, "invalid_perception_input", detail=str(error))
            return
        self.last_processed_wall = wall
        array = V2XVehiclePositionArray()
        fill_stamp(array.header.stamp, start_s)
        array.header.frame_id = self.map_frame
        for item in payload["vehicles"]:
            vehicle = V2XVehiclePosition()
            vehicle.header.frame_id = self.map_frame
            fill_stamp(vehicle.header.stamp, item["stamp_s"])
            vehicle.vehicle_id = item["vehicle_id"]
            for axis in ("x", "y", "z"):
                setattr(vehicle.position, axis, item["position"][axis])
                setattr(vehicle.covariance, axis, item["covariance"][axis])
            array.vehicles.append(vehicle)
        self.v2x_pub.publish(array)
        self.objects_pub.publish(String(data=json.dumps(dict(schema_version=1, stamp_s=start_s,
            frame_id=self.map_frame, source="lidar", teacher_ready=False,
            object_model=self.config.object_model, tracks=[asdict(t) for t in tracks], stats=stats), allow_nan=False)))
        markers = MarkerArray()
        for i, track in enumerate(tracks):
            marker = Marker()
            marker.header = array.header
            marker.ns, marker.id, marker.type, marker.action = "lidar_surface", i, Marker.CUBE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = track.detection.surface_xy_m
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y = [max(0.08, x) for x in track.detection.observed_size_xy_m]
            marker.scale.z, marker.color.a, marker.color.g, marker.color.b = 0.5, 0.6, 1.0, 1.0
            marker.lifetime = Duration(seconds=0.15).to_msg()
            markers.markers.append(marker)
        self.markers_pub.publish(markers)
        self.publish_status(True, "ok", confirmed_tracks=len(tracks), stats=stats)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LidarV2XNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
