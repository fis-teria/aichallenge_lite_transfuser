#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from aic_transfuser_lite.runtime.m3_acceptance import (
    TimedPlanDiagnosticV3,
    TimedScalarV3,
    summarize_m3_interval_v3,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze an M3 ROS bag without replaying control topics."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--speed-cap-mps", type=float, default=0.75)
    parser.add_argument("--speed-tolerance-mps", type=float, default=0.10)
    return parser.parse_args()


def _position(message: Any) -> tuple[float, float]:
    position = message.pose.pose.position
    return float(position.x), float(position.y)


def main() -> int:
    args = _arguments()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_classes = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }
    velocity: list[TimedScalarV3] = []
    yaw_rate: list[TimedScalarV3] = []
    plans: list[TimedPlanDiagnosticV3] = []
    safety: list[tuple[float, str]] = []
    arm_events: list[tuple[float, bool]] = []
    positions: list[tuple[float, float, float]] = []
    collision_true_count = 0
    while reader.has_next():
        topic, encoded, time_ns = reader.read_next()
        time_sec = time_ns * 1e-9
        message = deserialize_message(encoded, message_classes[topic])
        if topic == "/vehicle/status/velocity_status":
            velocity.append(TimedScalarV3(time_sec, float(message.longitudinal_velocity)))
            yaw_rate.append(TimedScalarV3(time_sec, float(message.heading_rate)))
        elif topic == "/plan_diagnostics":
            payload = json.loads(str(message.data))
            control = payload.get("external_controller")
            if isinstance(control, dict):
                longitudinal = control.get("longitudinal", {})
                decision = payload.get("decision", {})
                plans.append(TimedPlanDiagnosticV3(
                    time_sec=time_sec,
                    preflight_ready=bool(control.get("preflight_ready", False)),
                    commanded_speed_mps=float(control["commanded_speed_mps"]),
                    acceleration_mps2=float(control["acceleration_mps2"]),
                    controller_state=str(longitudinal.get("state", "missing")),
                    fault_reason=(
                        None if longitudinal.get("fault_reason") is None
                        else str(longitudinal["fault_reason"])
                    ),
                    stop_required=bool(decision.get("stop_required", False)),
                    decision_reasons=tuple(
                        str(reason) for reason in decision.get("reasons", [])
                    ),
                    preflight_reasons=tuple(
                        str(reason) for reason in control.get("preflight_reasons", [])
                    ),
                ))
        elif topic == "/safety_reason":
            safety.append((time_sec, str(message.data)))
        elif topic == "/overtake/race_armed":
            arm_events.append((time_sec, bool(message.data)))
        elif topic == "/localization/kinematic_state":
            x_m, y_m = _position(message)
            positions.append((time_sec, x_m, y_m))
        elif topic == "/awsim/ground_truth/on_collision" and bool(message.data):
            collision_true_count += 1

    arm_start = next((time_sec for time_sec, armed in arm_events if armed), None)
    if arm_start is None:
        raise RuntimeError("bag does not contain race_armed=true")
    arm_end = next(
        (time_sec for time_sec, armed in arm_events if time_sec > arm_start and not armed),
        None,
    )
    if arm_end is None:
        candidates = [sample.time_sec for sample in velocity if sample.time_sec >= arm_start]
        if not candidates:
            raise RuntimeError("bag has no velocity after race arm")
        arm_end = candidates[-1]
    interval_positions = [
        sample for sample in positions if arm_start <= sample[0] <= arm_end
    ]
    displacement_m = None
    if len(interval_positions) >= 2:
        displacement_m = math.hypot(
            interval_positions[-1][1] - interval_positions[0][1],
            interval_positions[-1][2] - interval_positions[0][2],
        )
    result = summarize_m3_interval_v3(
        arm_start_sec=arm_start,
        arm_end_sec=arm_end,
        velocity_mps=velocity,
        yaw_rate_rps=yaw_rate,
        plans=plans,
        safety_reasons=[
            reason for time_sec, reason in safety
            if arm_start <= time_sec <= arm_end
        ],
        displacement_m=displacement_m,
        collision_topic_present="/awsim/ground_truth/on_collision" in topic_types,
        collision_true_count=collision_true_count,
        speed_cap_mps=args.speed_cap_mps,
        speed_tolerance_mps=args.speed_tolerance_mps,
    )
    result["bag"] = str(args.bag.resolve())
    result["arm_start_sec"] = arm_start
    result["arm_end_sec"] = arm_end
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
