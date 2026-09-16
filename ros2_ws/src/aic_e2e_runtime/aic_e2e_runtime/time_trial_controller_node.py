"""Independent finite AWSIM trial controller; default publishes shadow only."""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from . import spatial_path_shadow_node_v4 as _source_layout
from aic_transfuser_lite.control.long_sim_tracking_v4 import check_scan
from aic_transfuser_lite.control.awsim_steering import command_steering, steering_response_gain
from aic_transfuser_lite.control.awsim_steering_response import SteeringResponseState, compensate_steering_response
from aic_transfuser_lite.control.vehicle_motion_v1 import IDEAL_POLICY, AWSIM_POLICY, AWSIM_POLICIES
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan, scan_pose_in_rear, select_aligned_scan
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import (
    SPEED_POLICIES, interpolate_body_pose, time_trial_control, trial_speed_limits, validate_trial_config,
)
from aic_transfuser_lite.runtime.awsim_trial_session import requested_stop, trial_duration_limits, trial_brake_reason, encode_scan_values
from aic_transfuser_lite.runtime.time_recovery_takeover_v1 import (
    RecoveryTakeoverState, propose_recovery_takeover, validate_recovery_reference,
)
from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    TARGET_MPS, bounded_collection_command, check_collection_input_time, project_course, validate_nominal,
)
from aic_transfuser_lite.data.time_steering_pulse_v1 import nominal_recovery_errors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--rear-axle-forward-m", type=float, required=True)
    parser.add_argument("--pose-source", required=True)
    parser.add_argument("--sensor-source", default="/awsim_d1")
    parser.add_argument("--authorize-awsim-only", action="store_true")
    parser.add_argument("--synthetic-shadow-fixture", action="store_true")
    parser.add_argument("--recovery-reference", type=Path,
                        help="Optional teacher bootstrap and finite pulse before one-way E2E takeover")
    settings = parser.add_mutually_exclusive_group()
    settings.add_argument("--trial-config", type=Path)
    settings.add_argument("--speed-policy", choices=SPEED_POLICIES)
    args, ros_args = parser.parse_known_args()
    config_sha = None
    execution_profile = "bounded_10s"
    obstacle_policy = "straight_v1"
    steering_policy = "identity_v1"
    lookahead_policy = "fixed_1m_v1"
    vehicle_model_policy = IDEAL_POLICY
    record_vehicle_motion = False
    clearance_profile = 'standard_v1'
    stopping_distance_policy = 'measured_speed_v1'
    speed_policy = args.speed_policy or "source_capped_0p25"
    if args.trial_config is not None:
        config_bytes = args.trial_config.read_bytes()
        config = json.loads(config_bytes)
        speed_policy = validate_trial_config(config)
        execution_profile = config.get("execution_profile", "bounded_10s")
        obstacle_policy = config.get("obstacle_policy", "straight_v1")
        steering_policy = config.get("steering_policy", "identity_v1")
        lookahead_policy = config.get("lookahead_policy", "fixed_1m_v1")
        vehicle_model_policy = config.get("vehicle_model_policy", IDEAL_POLICY)
        record_vehicle_motion = config.get("record_vehicle_motion", False)
        clearance_profile = config.get('diagnostic_clearance_profile', 'standard_v1')
        stopping_distance_policy = config.get('stopping_distance_policy', 'measured_speed_v1')
        if (config["checkpoint_sha256"] != args.checkpoint_sha256
                or config["geometry"]["rear_axle_forward_in_base_link_m"] != args.rear_axle_forward_m):
            raise ValueError("TRIAL_CONFIG_IDENTITY")
        config_sha = hashlib.sha256(config_bytes).hexdigest()
    _, overspeed_limit_mps = trial_speed_limits(speed_policy)
    steering_gain = steering_response_gain(steering_policy)
    drive_sim_s, drive_wall_s, _ = trial_duration_limits(execution_profile)
    live = args.authorize_awsim_only
    if live and args.synthetic_shadow_fixture:
        raise ValueError("SYNTHETIC_FIXTURE_CANNOT_ACTUATE")
    recovery_state = RecoveryTakeoverState()
    recovery_config = recovery_baseline = recovery_guide = None
    if args.recovery_reference is not None:
        if (not live or speed_policy != 'fixed_5kmh' or execution_profile != 'one_lap'
                or lookahead_policy != 'stopping_preview_segment_v1'
                or steering_policy != 'awsim_grip_0p6_lead_v1' or vehicle_model_policy != AWSIM_POLICY
                or obstacle_policy != 'steering_support_v2'):
            raise ValueError('RECOVERY_TRIAL_POLICIES_REQUIRED')
        recovery_config, recovery_baseline, recovery_guide = validate_recovery_reference(
            json.loads(args.recovery_reference.read_text()))
    if not math.isfinite(args.rear_axle_forward_m) or abs(args.rear_axle_forward_m) > .002:
        raise ValueError("UNSUPPORTED_BODY_POINT_CALIBRATION")
    if live and (os.environ.get("ROS_DOMAIN_ID") != "1" or args.sensor_source != "/awsim_d1"
                 or any(Path(p).exists() for p in ("/dev/vcu", "/dev/gnss", "/dev/ttyUSB0"))):
        raise ValueError("AWSIM_ONLY_REQUIRED")
    args.output.mkdir(parents=True, exist_ok=True)
    log = (args.output / "control.jsonl").open("x")
    motion_log = (args.output / "vehicle_observations.jsonl").open("x", buffering=65536) if record_vehicle_motion else None
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_planning_msgs.msg import Trajectory

    rclpy.init(args=ros_args)
    node = Node("time_path_controller", enable_rosout=False, start_parameter_services=False)
    topic = "/control/command/control_cmd" if live else "/time_path/shadow/control_cmd"
    publisher = None
    clock_ns = None
    clock_receipt = 0
    epoch = 0
    cache = {}
    poses: deque[TimedBodyPose] = deque(maxlen=256)
    scans: deque[tuple[LaserScan, int]] = deque(maxlen=4)
    previous = [0., time.monotonic()]
    response_state: SteeringResponseState | None = None
    state = {"fault": None, "armed_ns": None, "armed_wall": None, "stop_since_ns": None,
             "stop_confirmed": False, "positive_count": 0, "commands": 0, "max_speed_mps": 0.,
             "requested_stop_reason": None, "speed_mps": None, "current_pose": None,
             "scan_rejection_recorded": False}

    def stamp(t) -> int:
        return int(t.sec) * 10**9 + int(t.nanosec)

    def names(t) -> list[str]:
        return [e.node_namespace.rstrip("/") + "/" + e.node_name for e in node.get_publishers_info_by_topic(t)]

    def record(value) -> None:
        log.write(json.dumps({"monotonic_ns": time.monotonic_ns(), "sim_ns": clock_ns, **value}, allow_nan=False) + "\n")
        log.flush()

    def observe_motion(role: str, message) -> None:
        """Diagnostic observations only; never enter model/control admission."""
        if motion_log is None:
            return
        row = {"event": "VEHICLE_OBSERVATION", "role": role, "epoch": str(epoch),
               "receipt_monotonic_ns": time.monotonic_ns(), "received_sim_ns": clock_ns,
               "stamp_ns": stamp(message.stamp if role == "steering" else message.header.stamp)}
        if role != "steering":
            row["frame"] = message.header.frame_id
        if role == "velocity":
            row["longitudinal_lateral_mps_heading_radps"] = encode_scan_values(
                [message.longitudinal_velocity, message.lateral_velocity, message.heading_rate])
        elif role == "steering":
            row["tire_rad"] = encode_scan_values([message.steering_tire_angle])[0]
        elif role == "imu":
            a = message.angular_velocity
            row["angular_xyz_radps"] = encode_scan_values([a.x, a.y, a.z])
        elif role == "pose":
            p, q = message.pose.pose.position, message.pose.pose.orientation
            row["child_frame"] = message.child_frame_id
            row["position_xyz_m"] = encode_scan_values([p.x, p.y, p.z])
            row["quaternion_xyzw"] = encode_scan_values([q.x, q.y, q.z, q.w])
        motion_log.write(json.dumps(row, allow_nan=False) + "\n")

    def on_clock(message) -> None:
        nonlocal clock_ns, clock_receipt, epoch, response_state
        t = stamp(message.clock)
        if clock_ns is not None and t < clock_ns:
            state["fault"] = "CLOCK_RESET"; epoch += 1; poses.clear(); scans.clear(); cache.clear()
            response_state = None
        clock_ns = t; clock_receipt = time.monotonic_ns()

    node.create_subscription(Clock, "/clock", on_clock, 10)
    topics = {"plan": ("/time_path/plan", String, "/time_path_inference"),
              "pose": ("/localization/kinematic_state", Odometry, args.pose_source),
              "velocity": ("/vehicle/status/velocity_status", VelocityReport, args.sensor_source),
              "steering": ("/vehicle/status/steering_status", SteeringReport, args.sensor_source),
              "scan": ("/sensing/lidar/scan", LaserScan, args.sensor_source)}
    if recovery_config is not None:
        topics.update(nominal=("/recovery_teacher/nominal_control_cmd", AckermannControlCommand,
                               "/recovery_teacher_pure_pursuit"),
                      trajectory=("/recovery_teacher/trajectory", Trajectory, "/recovery_teacher_trajectory"))

    def receive(role, message) -> None:
        cache[role] = (message, time.monotonic_ns())
        if role in ("velocity", "steering", "pose"):
            observe_motion(role, message)
        if role == "scan":
            scans.append(cache[role])
        if role == "pose":
            p = message.pose.pose; q = p.orientation
            if (message.header.frame_id != "map" or message.child_frame_id != "base_link"
                    or not np.isfinite([p.position.x, p.position.y, q.x, q.y, q.z, q.w]).all()
                    or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .01):
                state["fault"] = "POSE_FRAME_OR_QUATERNION"
                return
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            poses.append(TimedBodyPose(stamp(message.header.stamp), "sim", str(epoch), "map", "base_link",
                                       float(p.position.x), float(p.position.y), yaw))

    for role, (input_topic, kind, _) in topics.items():
        node.create_subscription(kind, input_topic, lambda m, role=role: receive(role, m), qos_profile_sensor_data)
    if motion_log is not None:
        node.create_subscription(Imu, "/sensing/imu/imu_raw", lambda m: observe_motion("imu", m), qos_profile_sensor_data)
        def record_motion_sources() -> None:
            sources = {role: names(topics[role][0]) for role in ("velocity", "steering", "pose")}
            sources["imu"] = names("/sensing/imu/imu_raw")
            motion_log.write(json.dumps({"event": "MOTION_SOURCES", "sim_ns": clock_ns,
                "monotonic_ns": time.monotonic_ns(), "epoch": str(epoch), "sources": sources}) + "\n")
            motion_log.flush()
        node.create_timer(1., record_motion_sources)

    def tick() -> None:
        nonlocal publisher, response_state, recovery_state
        wall = time.monotonic(); now = time.monotonic_ns()
        actual = names(topic)
        if publisher is None:
            if actual:
                state["fault"] = "COMPETING_CONTROLLER"
            elif not live or any(e.node_name == "awsim_d1" for e in node.get_subscriptions_info_by_topic(topic)):
                publisher = node.create_publisher(AckermannControlCommand, topic, 10)
                record({"event": "PUBLISHER_CREATED", "topic": topic, "live": live,
                        "speed_policy": speed_policy, "overspeed_limit_mps": overspeed_limit_mps,
                        "trial_config_sha256": config_sha,
                        "obstacle_policy": obstacle_policy,
                        "steering_policy": steering_policy, "steering_response_gain": steering_gain,
                        "lookahead_policy": lookahead_policy,
                        "vehicle_model_policy": vehicle_model_policy,
                        "stopping_distance_policy": stopping_distance_policy,
                        "execution_profile": execution_profile, "drive_limit_sim_s": drive_sim_s,
                        "rear_axle_forward_m": args.rear_axle_forward_m})
        reason = "WAIT_AUTHORIZATION" if live else "SHADOW_ONLY"
        steer, accel, target = previous[0], -1., 0.
        details = {}; plan_id = None
        velocity_fresh = False
        speed = None
        measured_steer = None
        steering_observation = None
        motion_observation = None
        heading_rate = reported_lateral = None
        candidate_response_state = None
        proposed_recovery = None
        recovery_trace = None
        control_owner = 'E2E' if recovery_config is None or recovery_state.takeover_ns is not None else 'TEACHER_BOOTSTRAP'
        checking_scan = False
        scan_alignment = None
        dt = min(.1, max(0., wall - previous[1]))
        try:
            if state["fault"]:
                raise ValueError(state["fault"])
            if publisher is None or actual not in ([], ["/time_path_controller"]):
                raise ValueError("COMMAND_AUTHORITY")
            if clock_ns is None or now - clock_receipt > 500_000_000:
                raise ValueError("CLOCK_STALE")
            for role in ("velocity", "steering", "scan", "pose"):
                m, received = cache[role]
                t = stamp(m.stamp if role == "steering" else m.header.stamp)
                if now - received > 300_000_000 or not -20_000_000 <= clock_ns - t <= 150_000_000:
                    raise ValueError("STALE_" + role)
                if names(topics[role][0]) != [topics[role][2]]:
                    raise ValueError("SOURCE_" + role)
            speed = float(cache["velocity"][0].longitudinal_velocity)
            measured_steer = float(cache["steering"][0].steering_tire_angle)
            steering_observation = {
                "stamp_ns": stamp(cache["steering"][0].stamp),
                "age_sim_s": (clock_ns-stamp(cache["steering"][0].stamp))/1e9,
                "receipt_monotonic_ns": cache["steering"][1],
                "age_wall_s": (now-cache["steering"][1])/1e9,
            }
            if vehicle_model_policy in AWSIM_POLICIES:
                velocity = cache["velocity"][0]
                heading_rate, reported_lateral = float(velocity.heading_rate), float(velocity.lateral_velocity)
                motion_observation = {"stamp_ns": stamp(velocity.header.stamp), "frame": velocity.header.frame_id,
                    "receipt_monotonic_ns": cache["velocity"][1],
                    "age_sim_s": (clock_ns-stamp(velocity.header.stamp))/1e9,
                    "heading_rate_radps": encode_scan_values([heading_rate])[0],
                    "reported_lateral_mps": encode_scan_values([reported_lateral])[0]}
                if (velocity.header.frame_id != "base_link"
                        or abs(stamp(velocity.header.stamp)-steering_observation["stamp_ns"]) > 50_000_000):
                    raise ValueError("MOTION_FRAME_OR_CAPTURE_SKEW")
            if not np.isfinite([speed, float(cache["steering"][0].steering_tire_angle)]).all():
                speed = None
                measured_steer = None
                raise ValueError("NONFINITE_VEHICLE_STATE")
            velocity_fresh = True
            state["speed_mps"] = speed
            state["current_pose"] = max(poses, key=lambda p: p.stamp_ns).__dict__
            state["max_speed_mps"] = max(state["max_speed_mps"], abs(speed))
            if not -.03 <= speed <= overspeed_limit_mps:
                state["fault"] = "OVERSPEED_OR_REVERSE"; raise ValueError(state["fault"])
            if recovery_config is not None and recovery_state.takeover_ns is None:
                # Readiness is checked before authorization. These topics never
                # enter inference, and cannot gate/fallback after model takeover.
                for role in ('nominal', 'trajectory'):
                    m, received = cache[role]
                    if names(topics[role][0]) != [topics[role][2]]:
                        raise ValueError('SOURCE_' + role)
                    check_collection_input_time(role,
                        capture_ns=stamp(m.stamp if role == 'nominal' else m.header.stamp),
                        receipt_ns=received, now_sim_ns=clock_ns, now_wall_ns=now)
                nominal, received = cache['nominal']
                validate_nominal(stamp_ns=stamp(nominal.stamp), now_ns=clock_ns, received_ns=received,
                    now_wall_ns=now, target_mps=float(nominal.longitudinal.speed),
                    acceleration_mps2=float(nominal.longitudinal.acceleration),
                    steering_input_rad=float(nominal.lateral.steering_tire_angle), measured_speed_mps=speed)
                trajectory = cache['trajectory'][0]
                if (trajectory.header.frame_id != 'map' or len(trajectory.points) < 20
                        or any(not math.isclose(p.longitudinal_velocity_mps, TARGET_MPS, abs_tol=1e-5)
                               for p in trajectory.points)):
                    raise ValueError('REFERENCE_SPEED_OR_FRAME')
            auth = args.output / "drive_authorized.json"
            if live and auth.exists() and state["armed_ns"] is None:
                value = json.loads(auth.read_text())
                if (value.get("scope") != "TIME_PATH_AWSIM_TRIAL" or value.get("run_id") != args.run_id
                        or wall > value["expires_monotonic_s"]):
                    state["fault"] = "AUTHORIZATION_INVALID"; raise ValueError(state["fault"])
                state["armed_ns"] = clock_ns; state["armed_wall"] = wall
                record({"event": "ARMED", "authorization": value})
            armed = state["armed_ns"] is not None
            if live and not armed:
                raise ValueError("WAIT_AUTHORIZATION")
            stop_file = args.output / "stop_request.json"
            if live and stop_file.exists() and state["requested_stop_reason"] is None:
                state["requested_stop_reason"] = requested_stop(json.loads(stop_file.read_text()), args.run_id)
                record({"event": "STOP_REQUESTED", "reason": state["requested_stop_reason"]})
            if live:
                brake_reason = trial_brake_reason(execution_profile, (clock_ns-state["armed_ns"])/1e9,
                    wall-state["armed_wall"], state["requested_stop_reason"] is not None)
                if brake_reason is not None:
                    raise ValueError(brake_reason)
            laser = cache["scan"][0]
            clearance = None
            if obstacle_policy == "straight_v1":
                checking_scan = True
                clearance = check_scan(laser.ranges, laser.angle_min, laser.angle_increment,
                                       laser.range_min, laser.range_max, speed)
                checking_scan = False
            message, received = cache["plan"]
            value = json.loads(message.data)
            if (names(topics["plan"][0]) != [topics["plan"][2]] or value.get("event") != "PLAN"
                    or value.get("run_id") != args.run_id or value.get("epoch") != str(epoch)
                    or value.get("checkpoint_sha256") != args.checkpoint_sha256
                    or value.get("clock") != "sim" or value.get("frame") != "base_link"
                    or value.get("producer_kind") != ("SYNTHETIC_ROS_FIXTURE" if args.synthetic_shadow_fixture else "LEARNED_TIME_MODEL")
                    or value.get("dt_s") != .1 or value.get("precision") != "float32"):
                raise ValueError("PLAN_IDENTITY")
            obs_ns = value["observation_ns"]
            if (type(obs_ns) is not int or not 0 <= clock_ns - obs_ns <= 500_000_000
                    or now - received > 500_000_000):
                raise ValueError("PLAN_STALE")
            current = max(poses, key=lambda p: p.stamp_ns)
            if abs(current.stamp_ns - stamp(cache["velocity"][0].header.stamp)) > 50_000_000:
                raise ValueError("POSE_VELOCITY_SKEW")
            observed = interpolate_body_pose(poses, obs_ns)
            plan = TimePlan(value["plan_id"], observed, np.asarray(value["raw_xy_m"], dtype=float))
            plan_id = plan.plan_id
            details = {"clearance_m": clearance, "current_pose": current.__dict__,
                       "observation_pose": observed.__dict__}
            if recovery_config is not None:
                projection = None
                errors = None
                diagnostic_error = None
                try:
                    projection = project_course(recovery_baseline, [current.x_m, current.y_m], current.yaw_rad)
                    if recovery_guide[0, 0] <= projection['s_m'] <= recovery_guide[-1, 0]:
                        errors = nominal_recovery_errors(recovery_guide, s_m=projection['s_m'],
                            offset_m=projection['offset_m'], yaw_rad=current.yaw_rad)
                    elif recovery_state.takeover_ns is None and recovery_state.pulse.stage in ('active', 'releasing', 'recovery'):
                        raise ValueError('RECOVERY_GUIDE_SUPPORT_LOST')
                except ValueError as exc:
                    if recovery_state.takeover_ns is None:
                        raise
                    diagnostic_error = str(exc)  # Debug geometry never gates E2E.
                perturbation = 0.
                if recovery_state.takeover_ns is None:
                    proposed_recovery, perturbation, control_owner = propose_recovery_takeover(
                        recovery_config, recovery_state, sim_ns=clock_ns, wall_ns=now, observation_ns=obs_ns,
                        s_m=projection['s_m'], speed_mps=speed,
                        lateral_m=errors[0] if errors else 0., heading_rad=errors[1] if errors else 0.)
                recovery_trace = dict(projection=projection, lateral_error_m=errors[0] if errors else None,
                    heading_error_rad=errors[1] if errors else None, perturbation_rad=perturbation,
                    observation_ns=obs_ns, current_pose=current.__dict__, diagnostic_error=diagnostic_error)
                if control_owner == 'E2E' and recovery_state.takeover_ns is None:
                    # Commit ownership BEFORE PP admission; no quality-based
                    # selection and no teacher fallback on a rejected model plan.
                    recovery_state = proposed_recovery
                    response_state = None
                    record(dict(event='RECOVERY_E2E_TAKEOVER', plan_id=plan_id,
                                recovery=recovery_trace, recovery_state=asdict(recovery_state)))
            if control_owner == 'TEACHER_BOOTSTRAP':
                steer, accel = bounded_collection_command(
                    float(nominal.lateral.steering_tire_angle) + perturbation,
                    float(nominal.longitudinal.acceleration), previous[0], dt)
                target = TARGET_MPS
                mapping = dict(issued_input_rad=steer, issued_tire_target_rad=steering_gain*steer,
                               previous_tire_target_rad=steering_gain*previous[0])
                details['teacher_nominal'] = dict(stamp_ns=stamp(nominal.stamp),
                    steering_input_rad=float(nominal.lateral.steering_tire_angle),
                    acceleration_mps2=float(nominal.longitudinal.acceleration))
                details['steering_actuator'] = mapping
            else:
                details.update(time_trial_control(plan, current, speed_mps=speed,
                                                  speed_policy=speed_policy,
                                                  lookahead_policy=lookahead_policy,
                                                  vehicle_model_policy=vehicle_model_policy,
                                                  rear_axle_offset_m=(args.rear_axle_forward_m, 0.)))
                steer = details["steer_rad"]; accel = details["acceleration_mps2"]; target = details["target_speed_mps"]
                response_target, candidate_response_state, response = compensate_steering_response(
                    steer, clock_ns, response_state, policy=steering_policy)
                details["steering_response"] = response
                mapping = command_steering(response_target, previous[0], dt, policy=steering_policy)
                details["steering_actuator"] = mapping
                steer = mapping["issued_input_rad"]
            if obstacle_policy in ("steering_sweep_v1", "steering_support_v2"):
                checking_scan = True
                selected, captured = select_aligned_scan([(stamp(m.header.stamp), receipt) for m, receipt in scans],
                    poses, current, now_sim_ns=clock_ns, now_receipt_ns=now)
                laser = scans[selected][0]
                if laser.header.frame_id != "lidar":
                    raise ValueError("SCAN_FRAME")
                scan_alignment = scan_pose_in_rear(captured, current, args.rear_axle_forward_m)
                guard = check_turning_scan(laser.ranges, laser.angle_min, laser.angle_increment,
                    laser.range_min, laser.range_max, speed_mps=speed, measured_steer_rad=measured_steer,
                    issued_steer_rad=mapping["issued_tire_target_rad"], scan_in_current_rear=scan_alignment,
                    previous_steer_rad=mapping["previous_tire_target_rad"],
                    vehicle_model_policy=vehicle_model_policy,
                    heading_rate_radps=heading_rate, reported_lateral_mps=reported_lateral,
                    clearance_profile=clearance_profile,
                    stopping_distance_policy=stopping_distance_policy,
                    envelope_policy="curvature_support_v2" if obstacle_policy == "steering_support_v2" else "isotropic_v1")
                details["obstacle_guard"] = guard
                details["scan_in_current_rear"] = scan_alignment
                details["scan_stamp_ns"] = captured.stamp_ns
                details["clearance_m"] = guard["minimum_ray_margin_m"]
                checking_scan = False
            plan_id = plan.plan_id
            reason = ('RECOVERY_TEACHER_BOOTSTRAP' if control_owner == 'TEACHER_BOOTSTRAP'
                      else "TIME_PATH_TRACKING" if live else "SHADOW_CONTROL")
        except (ValueError, KeyError, TypeError) as exc:
            if checking_scan and str(exc) in ("STOPPING_CORRIDOR_OCCUPIED", "STOPPING_SWEEP_OCCUPIED") and not state["scan_rejection_recorded"]:
                scalar_names = ("angle_min", "angle_increment", "range_min", "range_max")
                record({"event": "SCAN_GUARD_REJECTED", "reason": str(exc), "speed_mps": speed,
                        "scan_stamp_ns": stamp(laser.header.stamp), "scan_frame": laser.header.frame_id,
                        "scan": {**dict(zip(scalar_names, encode_scan_values(getattr(laser, k) for k in scalar_names))),
                                 "ranges": encode_scan_values(laser.ranges)},
                        "current_pose": state["current_pose"], "pose_history": [p.__dict__ for p in poses],
                        "latest_plan_json": cache["plan"][0].data if "plan" in cache else None,
                        "obstacle_policy": obstacle_policy, "measured_steer_rad": measured_steer,
                        "clearance_profile": clearance_profile,
                        "steering_observation": steering_observation,
                        "motion_observation": motion_observation, "vehicle_model_policy": vehicle_model_policy,
                        "issued_steer_rad": steering_gain*steer, "previous_steer_rad": steering_gain*previous[0],
                        "issued_input_steer_rad": steer, "previous_input_steer_rad": previous[0],
                        "steering_policy": steering_policy, "scan_in_current_rear": scan_alignment,
                        "steering_response": details.get("steering_response"),
                        "plan_admission": "NOT_CHECKED_AFTER_SCAN_REJECTION" if obstacle_policy == "straight_v1" else "IDENTITY_AND_CONTROL_CHECKED_BEFORE_SCAN"})
                state["scan_rejection_recorded"] = True
            reason = str(exc); accel = -1.; target = 0.; steer = previous[0]
            response_state = candidate_response_state = None
            if reason in ("STOPPING_CORRIDOR_OCCUPIED", "STOPPING_SWEEP_OCCUPIED") and state["positive_count"]:
                state["fault"] = reason
            if reason.startswith('RECOVERY_'):
                state['fault'] = reason
        if reason in ("SCHEDULED_BRAKE", "REQUESTED_BRAKE") and velocity_fresh and speed is not None and abs(speed) < .03:
            if state["stop_since_ns"] is None:
                state["stop_since_ns"] = clock_ns
            state["stop_confirmed"] = clock_ns - state["stop_since_ns"] >= 1_000_000_000
        else:
            state["stop_since_ns"] = None
        steer = float(np.clip(steer, previous[0] - .8 * dt, previous[0] + .8 * dt))
        previous[:] = [steer, wall]
        # Do not fight a competing sender. The outer host supervisor terminates
        # this owned simulator on invalid authority; never commandeer a live graph.
        if publisher is not None and names(topic) == ["/time_path_controller"] and clock_ns is not None:
            command = AckermannControlCommand()
            command.stamp.sec = clock_ns // 10**9; command.stamp.nanosec = clock_ns % 10**9
            command.lateral.steering_tire_angle = steer
            command.longitudinal.acceleration = float(accel); command.longitudinal.speed = float(target)
            publisher.publish(command); state["commands"] += 1
            response_state = candidate_response_state
            if proposed_recovery is not None and control_owner == 'TEACHER_BOOTSTRAP' and target > 0:
                recovery_state = proposed_recovery
            if accel > 0:
                state["positive_count"] += 1
            record({"event": "COMMAND_SENT", "reason": reason, "plan_id": plan_id,
                    "steer_rad": steer, "acceleration_mps2": accel, "target_speed_mps": target,
                    "speed_mps": speed, "measured_steer_rad": measured_steer,
                    "steering_observation": steering_observation,
                    "motion_observation": motion_observation, "details": details,
                    **({'control_owner': control_owner, 'recovery': recovery_trace,
                        'recovery_state': asdict(recovery_state)} if recovery_config is not None else {})})
        else:
            response_state = None
            if actual and actual != ["/time_path_controller"]:
                state["fault"] = "COMPETING_CONTROLLER"
        temporary = args.output / "control_heartbeat.pending"
        if recovery_config is not None:
            state['recovery_state'] = asdict(recovery_state)
            state['control_owner'] = control_owner
        temporary.write_text(json.dumps({**state, "reason": reason, "monotonic_ns": now,
                                         "sim_ns": clock_ns, "topic": topic}, allow_nan=False))
        temporary.replace(args.output / "control_heartbeat.json")

    node.create_timer(.05, tick)  # Independent process, wall time, no model imports.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if motion_log is not None:
            motion_log.close()
        record({"event": "CONTROLLER_END", **state}); log.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
