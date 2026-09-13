"""Bounded Humble DDS smoke: real model Path, then labelled oracle shadow PP.

No vehicle command publisher. Run with isolated ROS_DOMAIN_ID and --network none.
The oracle phase is explicitly separate from learned-model behavior evidence.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--checkpoint-sha256", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--speed-policy", choices=("source_capped_0p25", "fixed_5kmh"), default="source_capped_0p25")
    ap.add_argument("--trial-config", type=Path)
    args = ap.parse_args()
    fixture_config = json.loads(args.trial_config.read_text()) if args.trial_config else None
    if fixture_config is not None:
        args.speed_policy = fixture_config["speed_policy"]
    if os.environ.get("ROS_DOMAIN_ID") != "93":
        raise ValueError("ISOLATED_DOMAIN_93_REQUIRED")
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry, Path as RosPath
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand

    rclpy.init(); node = Node("time_ros_fixture", enable_rosout=False)
    kinds = {"clock": ("/clock", Clock), "image": ("/sensing/camera/image_raw", Image),
             "scan": ("/sensing/lidar/scan", LaserScan), "velocity": ("/vehicle/status/velocity_status", VelocityReport),
             "steering": ("/vehicle/status/steering_status", SteeringReport), "pose": ("/localization/kinematic_state", Odometry)}
    pubs = {name: node.create_publisher(kind, topic, 10) for name, (topic, kind) in kinds.items()}
    paths = {}; plans = {}; commands = []
    def ns(stamp):
        return int(stamp.sec) * 10**9 + stamp.nanosec
    def on_path(m):
        paths[ns(m.header.stamp)] = {"frame": m.header.frame_id,
            "xy": [[p.pose.position.x, p.pose.position.y] for p in m.poses]}
    def on_plan(m):
        p = json.loads(m.data); plans[p["observation_ns"]] = p
    node.create_subscription(RosPath, "/visualization/time_path/raw_path", on_path, 10)
    node.create_subscription(String, "/time_path/plan", on_plan, 10)
    node.create_subscription(AckermannControlCommand, "/time_path/shadow/control_cmd",
        lambda m: commands.append({"wall": time.monotonic(), "speed": m.longitudinal.speed,
                                   "accel": m.longitudinal.acceleration, "steer": m.lateral.steering_tire_angle}), 10)
    processes = []; streams = []; result = {"status": "FAILED", "scope": "ISOLATED_SYNTHETIC_ROS_NOT_AWSIM"}
    start = time.monotonic(); last_sensor = 0.; counter = 0
    fixture_speed_mps = .1
    fixture_scan_offset_ns = 0

    def launch(module, extra, name):
        stream = (args.output / (name + ".log")).open("x"); streams.append(stream)
        process = subprocess.Popen([sys.executable, "-m", "aic_e2e_runtime." + module, *extra],
                                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)

    def set_stamp(target, stamp):
        target.sec = stamp // 10**9; target.nanosec = stamp % 10**9

    def sensors():
        nonlocal counter, last_sensor
        wall = time.monotonic()
        if wall - last_sensor < .02:
            return None
        last_sensor = wall; counter += 1
        t = round((100 + wall - start) * 1e9)
        clock = Clock(); set_stamp(clock.clock, t); pubs["clock"].publish(clock)
        velocity = VelocityReport(); set_stamp(velocity.header.stamp, t); velocity.header.frame_id = "base_link"
        velocity.longitudinal_velocity = fixture_speed_mps; pubs["velocity"].publish(velocity)
        steering = SteeringReport(); set_stamp(steering.stamp, t); pubs["steering"].publish(steering)
        pose = Odometry(); set_stamp(pose.header.stamp, t); pose.header.frame_id = "map"; pose.child_frame_id = "base_link"
        pose.pose.pose.orientation.w = 1.; pose.pose.pose.position.x = .1 * (wall - start); pubs["pose"].publish(pose)
        if counter % 5 == 0:
            scan = LaserScan(); set_stamp(scan.header.stamp, t+fixture_scan_offset_ns); scan.header.frame_id = "lidar" if fixture_config else "lidar_link"
            scan.angle_min = -float(np.pi); scan.angle_increment = float(2*np.pi/750)
            scan.angle_max = scan.angle_min + 749 * scan.angle_increment
            scan.range_min = .1; scan.range_max = 25.; scan.ranges = [25.] * 750; pubs["scan"].publish(scan)
            image = Image(); set_stamp(image.header.stamp, t); image.header.frame_id = "camera_link"
            image.height = 72; image.width = 128; image.step = 384; image.encoding = "rgb8"
            image.data = bytes([128]) * (72*128*3); pubs["image"].publish(image)
        return t

    def spin_for(seconds, extra=None, publish=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if time.monotonic() - start > 100:
                raise RuntimeError("SMOKE_WALL_LIMIT")
            t = sensors() if publish else None
            if extra and t is not None:
                extra(t)
            rclpy.spin_once(node, timeout_sec=.002)
            if node.get_publishers_info_by_topic("/control/command/control_cmd"):
                raise RuntimeError("UNEXPECTED_VEHICLE_COMMAND_PUBLISHER")

    try:
        model_process = launch("time_path_node", ["--checkpoint", str(args.checkpoint), "--checkpoint-sha256", args.checkpoint_sha256,
            "--output", str(args.output / "learned"), "--run-id", "ros-smoke-learned", "--device", "cpu",
            "--sensor-source", "/time_ros_fixture"], "model")
        # Publishing throughout startup also exercises initially missing history.
        deadline = time.monotonic() + 40
        while len(paths) < 6 and time.monotonic() < deadline:
            if model_process.poll() is not None:
                raise RuntimeError("MODEL_NODE_EXIT")
            spin_for(.2)
        joined = sorted(set(paths) & set(plans))
        if len(joined) < 6:
            raise RuntimeError("INSUFFICIENT_LEARNED_PATHS")
        for t in joined:
            if paths[t]["frame"] != "base_link" or plans[t]["producer_kind"] != "LEARNED_TIME_MODEL":
                raise RuntimeError("PATH_FRAME_OR_PRODUCER")
            xy = np.asarray(paths[t]["xy"])
            if xy.shape != (30, 2) or not np.isfinite(xy).all() or not np.array_equal(xy, plans[t]["raw_xy_m"]):
                raise RuntimeError("PATH_RAW_VALUE_MISMATCH")
        result["learned_path_matches"] = len(joined)
        result["checkpoint_sha256"] = args.checkpoint_sha256
        stop(model_process)
        oracle = Node("time_path_inference", enable_rosout=False)
        oracle_publisher = oracle.create_publisher(String, "/time_path/plan", 10)
        controller_settings = ["--speed-policy", args.speed_policy]
        if fixture_config is not None:
            # Real scan/pose callbacks are asynchronous: the newest scan may
            # not yet have a following pose endpoint at the control tick.
            fixture_scan_offset_ns = 5_000_000
            fixture_config["checkpoint_sha256"] = "0"*64
            fixture_path = args.output/"oracle_trial_config.json"
            fixture_path.write_text(json.dumps(fixture_config))
            controller_settings = ["--trial-config", str(fixture_path)]
        controller = launch("time_trial_controller_node", ["--output", str(args.output / "oracle"),
            "--run-id", "ros-smoke-oracle", "--checkpoint-sha256", "0"*64,
            "--rear-axle-forward-m", ".0010000169277191162", "--pose-source", "/time_ros_fixture",
            *controller_settings,
            "--sensor-source", "/time_ros_fixture", "--synthetic-shadow-fixture"], "controller")
        oracle_source_speed = .8 if args.speed_policy == "fixed_5kmh" else .15
        expected_target = 5./3.6 if args.speed_policy == "fixed_5kmh" else .15
        oracle_curvature = 0.
        def publish_oracle(t):
            distances = np.arange(1, 31)*oracle_source_speed*.1
            xy = (np.column_stack((np.sin(oracle_curvature*distances)/oracle_curvature,
                                  (1-np.cos(oracle_curvature*distances))/oracle_curvature))
                  if oracle_curvature else np.column_stack((distances, np.zeros(30))))
            value = {"event": "PLAN", "run_id": "ros-smoke-oracle", "epoch": "0", "plan_id": str(t),
                     "observation_ns": t, "clock": "sim", "frame": "base_link", "dt_s": .1,
                     "checkpoint_sha256": "0"*64, "precision": "float32", "producer_kind": "SYNTHETIC_ROS_FIXTURE",
                     "raw_xy_m": xy.tolist()}
            message = String(); message.data = json.dumps(value); oracle_publisher.publish(message)
        spin_for(7, publish_oracle)
        positive = [c for c in commands if c["accel"] > 0 and abs(c["speed"]-expected_target) < 1e-5 and abs(c["steer"]) < 1e-8]
        if not positive:
            raise RuntimeError("ORACLE_DID_NOT_REACH_SHADOW_PP")
        if fixture_config is not None and fixture_config.get("obstacle_policy") in ("steering_sweep_v1", "steering_support_v2"):
            records = [json.loads(line) for line in (args.output/"oracle/control.jsonl").read_text().splitlines()]
            expected_guard = ("CURVATURE_INTERVAL_SUPPORT_V2" if fixture_config["obstacle_policy"] == "steering_support_v2"
                              else "STEERING_INTERVAL_SWEEP_V1")
            verified = [r for r in records if r.get("details", {}).get("obstacle_guard", {}).get("policy") == expected_guard]
            if not verified:
                raise RuntimeError("SWEEP_GUARD_NOT_EXECUTED")
            result["sweep_guard_commands"] = len(verified)
            result["sweep_guard_policy"] = expected_guard
            result["scan_ahead_of_pose_ns"] = fixture_scan_offset_ns
        if fixture_config is not None and fixture_config.get("steering_policy") in ("awsim_grip_0p6_v1", "awsim_grip_0p6_lead_v1"):
            if fixture_config.get("lookahead_policy") == "stopping_preview_v1":
                fixture_speed_mps = 1.2  # Exercise the moving-speed preview, not just startup.
            mapping_count = 0
            for direction in (-1., 1.):
                oracle_curvature = direction*.08
                turn_started = time.monotonic()
                spin_for(1.5, publish_oracle)
                records = [json.loads(line) for line in (args.output/"oracle/control.jsonl").read_text().splitlines()]
                turning = [r for r in records if r.get("reason") == "SHADOW_CONTROL"
                           and r["monotonic_ns"]/1e9 > turn_started+.8]
                if not turning:
                    raise RuntimeError("CALIBRATED_TURN_NOT_EXECUTED")
                for row in turning:
                    required = row["details"]["steer_rad"]
                    actuator_target = row["details"].get("steering_response", {}).get("target_tire_rad", required)
                    guard = row["details"]["obstacle_guard"]
                    if (direction*required < .05 or abs(.6*row["steer_rad"]-actuator_target) > 1e-5
                            or abs(guard["issued_steer_rad"]-.6*row["steer_rad"]) > 1e-9):
                        raise RuntimeError("CALIBRATED_TIRE_AND_INPUT_ANGLE_MISMATCH")
                    if (fixture_config.get("lookahead_policy") == "stopping_preview_v1"
                            and row["details"]["selected_lookahead_distance_m"] < 1.72):
                        raise RuntimeError("STOPPING_PREVIEW_NOT_APPLIED_AT_SPEED")
                mapping_count += len(turning)
            result["calibrated_left_right_shadow_commands"] = mapping_count
            if fixture_config["steering_policy"] == "awsim_grip_0p6_lead_v1":
                lead = [r for r in records if r.get("reason") == "SHADOW_CONTROL"
                        and abs(r["details"]["steering_response"]["applied_correction_rad"]) > .001]
                if not lead or any(r["steering_observation"] is None for r in lead):
                    raise RuntimeError("STEERING_RESPONSE_LEAD_NOT_EXECUTED")
                result["response_lead_shadow_commands"] = len(lead)
            oracle_curvature = .4
            infeasible_started = time.monotonic()
            spin_for(1., publish_oracle)
            records = [json.loads(line) for line in (args.output/"oracle/control.jsonl").read_text().splitlines()]
            rejected = [r for r in records if r.get("event") == "COMMAND_SENT"
                        and r["monotonic_ns"]/1e9 > infeasible_started+.5]
            infeasible_reason = ("STEERING_FEASIBLE_LOOKAHEAD_MISSING"
                                 if fixture_config.get("lookahead_policy") in ("feasible_1_to_1p5m_v1", "stopping_preview_v1")
                                 else "STEERING_ACTUATOR_INFEASIBLE")
            if not rejected or any(r["reason"] != infeasible_reason
                                   or r["acceleration_mps2"] >= 0 or r["target_speed_mps"] != 0 for r in rejected):
                raise RuntimeError("INFEASIBLE_ACTUATOR_DID_NOT_BRAKE")
            result["actuator_infeasible_brake_commands"] = len(rejected)
            result["infeasible_curve_reason"] = infeasible_reason
            oracle_curvature = 0.
        stale_started = time.monotonic()
        spin_for(1.2)  # Sensors continue; stop sending plans.
        stale = [c for c in commands if c["wall"] > stale_started + .65]
        if not stale or any(c["accel"] >= 0 or c["speed"] != 0 for c in stale):
            raise RuntimeError("STALE_PLAN_DID_NOT_BRAKE")
        pause_started = time.monotonic()
        spin_for(1.0, publish=False)  # Wall watchdog must continue with a stopped /clock.
        paused = [c for c in commands if c["wall"] > pause_started + .65]
        if not paused or any(c["accel"] >= 0 for c in paused):
            raise RuntimeError("PAUSED_CLOCK_DID_NOT_BRAKE")
        fixture_speed_mps = (6./3.6 if args.speed_policy == "fixed_5kmh" else .45) + .05
        overspeed_started = time.monotonic()
        spin_for(1., publish_oracle)
        overspeed = [c for c in commands if c["wall"] > overspeed_started + .4]
        heartbeat = json.loads((args.output / "oracle/control_heartbeat.json").read_text())
        if (not overspeed or any(c["accel"] >= 0 or c["speed"] != 0 for c in overspeed)
                or heartbeat["fault"] != "OVERSPEED_OR_REVERSE"):
            raise RuntimeError("OVERSPEED_DID_NOT_BRAKE")
        result.update(status="PASS", oracle_positive_shadow_commands=len(positive),
                      stale_plan_brake_commands=len(stale), paused_clock_brake_commands=len(paused),
                      speed_policy=args.speed_policy, expected_target_speed_mps=expected_target,
                      overspeed_brake_commands=len(overspeed),
                      vehicle_command_publishers=0)
        oracle.destroy_node()
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        for process in processes:
            stop(process)
        for stream in streams:
            stream.close()
        result["wall_s"] = time.monotonic() - start
        result["child_exit_codes"] = [p.returncode for p in processes]
        (args.output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False))
        print(json.dumps(result))
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
