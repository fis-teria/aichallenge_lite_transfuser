#!/usr/bin/env python3
"""Deterministic test-only ego/V2X publisher and Planner metrics collector."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray


DT = 0.05
FRAME = "map"
START_NS = 100_000_000_000
SCENARIOS = (
    ("WP25-L", 25, -0.25, 0.25, 0.15, 8.0, 2.8, "none"),
    ("WP30-R", 30, 0.00, 0.00, 0.50, 12.0, 1.0, "none"),
    ("WP40-B", 40, 0.25, -0.25, 0.15, 4.0, 4.0, "none"),
    ("WP50-B", 50, 0.00, 0.25, 0.50, 8.0, 2.8, "none"),
    ("WP55-DIST", 55, 0.00, 0.00, 0.15, 12.0, 1.0, "none"),
    ("WP60-DIST", 60, 0.00, 0.00, 0.15, 8.0, 2.8, "none"),
    ("WP63-DIST", 63, 0.00, 0.00, 0.15, 4.0, 4.0, "none"),
    ("BLOCK-L", 40, 0.00, 0.00, 0.15, 8.0, 2.8, "LEFT"),
    ("BLOCK-R", 40, 0.00, 0.00, 0.15, 8.0, 2.8, "RIGHT"),
    ("ZONE-EXIT-PRE", 62, 0.00, 0.00, 0.50, 12.0, 1.0, "none"),
)


class Reference:
    def __init__(self, path: Path):
        with path.open() as stream:
            self.points = [
                {key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)
            ]
        self.length = self.points[-1]["s_m"]

    def at(self, s: float, d: float = 0.0) -> tuple[float, float, float]:
        s %= self.length
        upper = next((i for i, p in enumerate(self.points) if p["s_m"] >= s), 0)
        lower = max(0, upper - 1)
        a, b = self.points[lower], self.points[upper]
        span = b["s_m"] - a["s_m"]
        ratio = 0.0 if span <= 1e-9 else (s - a["s_m"]) / span
        yaw_delta = math.atan2(math.sin(b["psi_rad"] - a["psi_rad"]), math.cos(b["psi_rad"] - a["psi_rad"]))
        yaw = a["psi_rad"] + ratio * yaw_delta
        x = a["x_m"] + ratio * (b["x_m"] - a["x_m"]) - math.sin(yaw) * d
        y = a["y_m"] + ratio * (b["y_m"] - a["y_m"]) + math.cos(yaw) * d
        return x, y, yaw


def stamp(message_stamp, nanoseconds: int) -> None:
    message_stamp.sec = nanoseconds // 1_000_000_000
    message_stamp.nanosec = nanoseconds % 1_000_000_000


class Harness(Node):
    def __init__(self, reference: Reference, output: Path, collect_only: bool, duration: float):
        super().__init__("test_only_attack_zone_scenario")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.reference = reference
        self.output = output
        self.collect_only = collect_only
        self.duration = duration
        self.clock_pub = self.create_publisher(Clock, "/clock", qos)
        self.odom_pub = self.create_publisher(Odometry, "/localization/kinematic_state", qos)
        self.v2x_pub = self.create_publisher(V2XVehiclePositionArray, "/v2x/vehicle_positions", qos)
        self.scenario_pub = self.create_publisher(String, "/test_only/attack_zone/scenario", qos)
        self.create_subscription(String, "/test_only/attack_zone/scenario", self.on_scenario, qos)
        self.create_subscription(String, "/debug/overtake/metrics", self.on_metrics, qos)
        self.current_scenario = "warmup"
        self.metrics: list[dict] = []
        self.start_wall = time.monotonic()

    def on_scenario(self, message: String) -> None:
        self.current_scenario = message.data

    def on_metrics(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        payload["harness_scenario"] = self.current_scenario
        self.metrics.append(payload)

    def publish_tick(self, scenario, tick: int, sim_ns: int) -> None:
        name, wp, ego_d, target_d, uncertainty, gap, relative_speed, blocked = scenario
        elapsed = tick * DT
        ego_speed = 8.0
        target_speed = max(0.0, ego_speed - relative_speed)
        ego_s = self.reference.points[wp]["s_m"] + ego_speed * elapsed
        target_s = self.reference.points[wp]["s_m"] + gap + target_speed * elapsed
        clock = Clock(); stamp(clock.clock, sim_ns); self.clock_pub.publish(clock)
        marker = String(); marker.data = name; self.scenario_pub.publish(marker)
        x, y, yaw = self.reference.at(ego_s, ego_d)
        odom = Odometry(); stamp(odom.header.stamp, sim_ns); odom.header.frame_id = FRAME
        odom.child_frame_id = "base_link"; odom.pose.pose.position.x = x; odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0); odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Odometry twist is expressed in child_frame_id; Planner consumes its
        # longitudinal x component directly.
        odom.twist.twist.linear.x = ego_speed
        self.odom_pub.publish(odom)
        array = V2XVehiclePositionArray(); stamp(array.header.stamp, sim_ns); array.header.frame_id = FRAME
        array.vehicles.append(self.vehicle("target", target_s, target_d, uncertainty, sim_ns))
        if blocked != "none":
            block_d = 1.75 if blocked == "LEFT" else -1.75
            array.vehicles.append(self.vehicle("blocker", ego_s + 9.0, block_d, 0.15, sim_ns))
        self.v2x_pub.publish(array)

    def vehicle(self, vehicle_id: str, s: float, d: float, uncertainty: float, sim_ns: int):
        x, y, _ = self.reference.at(s, d)
        vehicle = V2XVehiclePosition(); stamp(vehicle.header.stamp, sim_ns); vehicle.header.frame_id = FRAME
        vehicle.vehicle_id = vehicle_id; vehicle.position.x = x; vehicle.position.y = y
        vehicle.covariance.x = uncertainty; vehicle.covariance.y = uncertainty
        return vehicle

    def run(self) -> None:
        if self.collect_only:
            deadline = time.monotonic() + self.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
        else:
            sim_ns = START_NS
            # Discovery and velocity-history warm-up use the same deterministic clock.
            for index, scenario in enumerate(SCENARIOS):
                for tick in range(30):
                    self.publish_tick(scenario, tick, sim_ns)
                    sim_ns += int(DT * 1e9)
                    time.sleep(DT)
                    for _ in range(16):
                        rclpy.spin_once(self, timeout_sec=0.0)
                # Force stale/reacquisition boundaries without resetting Planner code.
                sim_ns += 600_000_000
                clock = Clock(); stamp(clock.clock, sim_ns); self.clock_pub.publish(clock)
                time.sleep(0.08)
                for _ in range(16):
                    rclpy.spin_once(self, timeout_sec=0.0)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.metrics, indent=2, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--collect-only", action="store_true")
    parser.add_argument("--duration", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = Harness(Reference(Path(args.reference)), Path(args.output), args.collect_only, args.duration)
    try:
        node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
