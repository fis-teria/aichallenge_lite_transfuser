"""TimePath camera-anchored inference and ordinary RViz Path; no actuation."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import numpy as np
import torch

# Standard source/ament install resolution, shared with the existing package.
from . import spatial_path_shadow_node_v4 as _source_layout

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.mcap_converter_v2 import TimedImage, TimedLidar, TimedSteering, TimedVelocity
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeInputBuffer, TimeRuntimeModel, decode_ros_image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--sensor-source", default="/awsim_d1")
    args, ros_args = parser.parse_known_args()
    args.output.mkdir(parents=True, exist_ok=True)
    log = (args.output / "inference.jsonl").open("x")

    def record(value: dict) -> None:
        log.write(json.dumps({"monotonic_ns": time.monotonic_ns(), **value}, allow_nan=False) + "\n")
        log.flush()

    torch.set_num_threads(2)
    model = TimeRuntimeModel(args.checkpoint, expected_sha256=args.checkpoint_sha256, device=args.device)
    config = model.config
    warm = ModelBatchV3(
        torch.zeros(1, config.image_history_length, 3, config.image_height, config.image_width),
        torch.ones(1, config.image_history_length, dtype=torch.bool),
        torch.zeros(1, config.lidar_history_length, 2, config.lidar_points),
        torch.ones(1, config.lidar_history_length, dtype=torch.bool),
        torch.zeros(1, config.ego_history_length, 4), torch.ones(1, config.ego_history_length, 4, dtype=torch.bool),
        torch.zeros(1, config.command_history_length, 3), torch.zeros(1, config.command_history_length, dtype=torch.bool),
        torch.zeros(1, config.image_history_length, 2), targets=None, requested_outputs=frozenset({"trajectory"}))
    model.predict(warm)
    record({"event": "MODEL_READY", "checkpoint_sha256": model.sha256, "epoch": model.epoch,
            "precision": "float32", "device": args.device, "torch_version": torch.__version__,
            "config": config.to_dict(), "receipt_clock": "monotonic", "freeze_delay_ns": 50_000_000})

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped

    rclpy.init(args=ros_args)
    node = Node("time_path_inference", enable_rosout=False, start_parameter_services=False)
    buffer = TimeInputBuffer(config, run_id=args.run_id)
    publisher = node.create_publisher(String, "/time_path/plan", 10)
    path_publisher = node.create_publisher(RosPath, "/visualization/time_path/raw_path", 10)
    pool = ThreadPoolExecutor(max_workers=1)
    future = None
    state = {"plans": 0, "rejected": 0, "fault": None, "sources_ok": False}
    clock_receipt = 0
    last_graph_check = 0
    frame_ids: dict[str, str] = {}

    def stamp(value) -> int:
        return int(value.sec) * 10**9 + int(value.nanosec)

    def on_clock(message) -> None:
        nonlocal clock_receipt
        clock_receipt = time.monotonic_ns()
        if buffer.on_clock(stamp(message.clock)):
            record({"event": "CLOCK_RESET", "epoch": buffer.epoch})

    node.create_subscription(Clock, "/clock", on_clock, 10)
    topics = {"camera": ("/sensing/camera/image_raw", Image), "lidar": ("/sensing/lidar/scan", LaserScan),
              "velocity": ("/vehicle/status/velocity_status", VelocityReport),
              "actual_steering": ("/vehicle/status/steering_status", SteeringReport)}

    def receive(role, message) -> None:
        received = time.monotonic_ns()  # Receipt before decode, explicit proxy, not preprocessing completion.
        t = stamp(message.stamp if role == "actual_steering" else message.header.stamp)
        try:
            if role in {"camera", "lidar"}:
                frame = message.header.frame_id
                if not frame or role in frame_ids and frame_ids[role] != frame:
                    raise ValueError("SENSOR_FRAME_CHANGED_OR_EMPTY")
                frame_ids[role] = frame
            if role == "camera":
                payload = TimedImage(t, decode_ros_image(message))
            elif role == "lidar":
                payload = TimedLidar(t, np.asarray(message.ranges, dtype=np.float32).copy(),
                                     float(message.angle_min), float(message.angle_increment),
                                     float(message.range_min), float(message.range_max), message.header.frame_id)
            elif role == "velocity":
                payload = TimedVelocity(t, float(message.longitudinal_velocity),
                                        float(message.lateral_velocity), float(message.heading_rate))
            else:
                payload = TimedSteering(t, float(message.steering_tire_angle))
            buffer.add(role, payload, received_ns=received)
        except ValueError as exc:
            state["rejected"] += 1
            record({"event": "INPUT_REJECTED", "role": role, "reason": str(exc), "capture_ns": t})

    for role, (topic, kind) in topics.items():
        node.create_subscription(kind, topic, lambda m, role=role: receive(role, m), qos_profile_sensor_data)

    def infer(snapshot):
        started = time.monotonic_ns()
        batch, provenance = snapshot.assemble(buffer.config)
        xy = model.predict(batch)
        return snapshot, xy, provenance, time.monotonic_ns() - started

    def tick() -> None:
        nonlocal future, last_graph_check
        now = time.monotonic_ns()
        if now - last_graph_check > 500_000_000:
            graph = {role: [e.node_namespace.rstrip("/") + "/" + e.node_name
                            for e in node.get_publishers_info_by_topic(topic)] for role, (topic, _) in topics.items()}
            state["sources_ok"] = all(names == [args.sensor_source] for names in graph.values())
            state["sources"] = graph
            state["path_subscribers"] = [e.node_name for e in node.get_subscriptions_info_by_topic("/visualization/time_path/raw_path")]
            last_graph_check = now
        if future is not None and future.done():
            try:
                snapshot, xy, provenance, inference_ns = future.result()
                anchor = snapshot.anchor
                if (anchor.epoch != buffer.epoch or buffer.clock_ns is None
                        or not 0 <= buffer.clock_ns - anchor.capture_ns <= 500_000_000
                        or now - snapshot.freeze_ns > 500_000_000
                        or now - clock_receipt > 500_000_000 or not state["sources_ok"]):
                    raise ValueError("RESULT_EXPIRED_OR_SOURCE_CHANGED")
                value = {"event": "PLAN", "run_id": args.run_id, "epoch": anchor.epoch,
                         "plan_id": f"{args.run_id}:{anchor.epoch}:{anchor.capture_ns}:{anchor.sequence}",
                         "observation_ns": anchor.capture_ns, "freeze_monotonic_ns": snapshot.freeze_ns,
                         "published_monotonic_ns": now, "clock": "sim", "frame": "base_link",
                         "dt_s": .1, "checkpoint_sha256": model.sha256, "precision": "float32",
                         "producer_kind": "LEARNED_TIME_MODEL",
                         "raw_xy_m": xy.tolist(), "inference_ns": inference_ns,
                         "provenance": provenance, "sensor_frames": dict(frame_ids)}
                message = String(); message.data = json.dumps(value, allow_nan=False)
                publisher.publish(message)
                path = RosPath(); path.header.frame_id = "base_link"
                path.header.stamp.sec = anchor.capture_ns // 10**9
                path.header.stamp.nanosec = anchor.capture_ns % 10**9
                for x, y in xy:
                    pose = PoseStamped(); pose.header = path.header
                    pose.pose.position.x = float(x); pose.pose.position.y = float(y)
                    pose.pose.orientation.w = 1.
                    path.poses.append(pose)
                path_publisher.publish(path)
                record(value); state["plans"] += 1
            except ValueError as exc:
                state["rejected"] += 1
                record({"event": "ANCHOR_REJECTED", "reason": str(exc)})
            finally:
                future = None
        if future is None and state["sources_ok"] and now - clock_receipt <= 500_000_000:
            snapshot = buffer.freeze_latest(now)
            if snapshot is not None:
                future = pool.submit(infer, snapshot)
        heartbeat = {**state, "monotonic_ns": now, "sim_ns": buffer.clock_ns,
                     "epoch": buffer.epoch, "dropped_anchors": buffer.dropped_anchors}
        temporary = args.output / "inference_heartbeat.pending"
        temporary.write_text(json.dumps(heartbeat, allow_nan=False))
        temporary.replace(args.output / "inference_heartbeat.json")

    node.create_timer(.01, tick)  # Wall timer: node never enables use_sim_time.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
        record({"event": "INFERENCE_END", **state})
        log.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
