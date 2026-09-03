#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import threading
import time
from typing import Sequence

from aic_transfuser_lite.data.collection_reference_v3 import (
    route_points_from_arrows,
    write_route_reference_v3,
)


REFERENCE_TOPIC = "/heading_pose_initializer/raceline_markers"
REFERENCE_TYPE = "visualization_msgs/msg/MarkerArray"


def parse_marker_array(message: object) -> tuple:
    """Parse ROS MarkerArray heading arrows without leaking ROS types downstream."""

    arrows = []
    for marker in message.markers:
        if str(marker.ns) != "heading_arrows" or int(marker.type) != 0:
            continue
        if len(marker.points) < 2:
            raise ValueError(f"heading arrow {marker.id} has fewer than two points")
        frame_id = str(marker.header.frame_id)
        arrows.append(
            (
                int(marker.id),
                frame_id,
                float(marker.points[0].x),
                float(marker.points[0].y),
                float(marker.points[1].x),
                float(marker.points[1].y),
            )
        )
    return route_points_from_arrows(arrows)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a teacher-only course Reference from transient raceline arrows."
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default=REFERENCE_TOPIC)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.timeout_sec <= 0.0:
        raise ValueError("timeout-sec must be positive")
    if args.output.exists() or args.output.with_suffix(".manifest.yaml").exists():
        raise FileExistsError(f"refusing to overwrite existing Reference: {args.output}")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    node = Node("aic_capture_raceline_reference_v3")
    received = threading.Event()
    result: dict[str, object] = {}

    def callback(message: MarkerArray) -> None:
        try:
            result["points"] = parse_marker_array(message)
        except Exception as exc:  # surfaced after spin; never silently ignored.
            result["error"] = exc
        finally:
            received.set()

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscription = node.create_subscription(MarkerArray, args.topic, callback, qos)
    try:
        deadline = time.monotonic() + args.timeout_sec
        while not received.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if not received.is_set():
        raise TimeoutError(f"no transient MarkerArray received from {args.topic}")
    if "error" in result:
        raise result["error"]  # type: ignore[misc]
    points = result["points"]
    write_route_reference_v3(
        args.output,
        points,  # type: ignore[arg-type]
        source_topic=args.topic,
        source_type=REFERENCE_TYPE,
        captured_utc=datetime.now(timezone.utc).isoformat(),
    )
    print(f"captured {len(points)} Reference points to {args.output}")  # type: ignore[arg-type]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
