"""Sole SIM-ONLY Tiny command publisher, independent of official model worker."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import time

import numpy as np

from aic_transfuser_lite.runtime.tiny_lidar_sim import (
    OfficialTiny, speed_acceleration, validate_scan, validate_operation, verify_container_contract, input_clock_ready,
    SimReadiness, validate_tiny_config, DRIVE_CUTOFF_UNIX_S, motion_limit_reason,
    host_arm_status, GUI_AUTHORIZATIONS,
)
from spatial_dev_host_v4 import atomic_json

CONTROL_TOPICS = ("/control/command/control_cmd", "/awsim/control_mode_request_topic",
                  "/control/command/gear_cmd")
ALTERNATE_TOPICS = ("/control/command/actuation_cmd", "/control/command/emergency_cmd")


def worker_main(inbox, outbox, stop, output: str, forward_limit: int) -> None:
    """No ROS/controller access. One official forward per accepted scan ID."""
    destination = Path(output)
    calls = 0
    reason = None
    trace = (destination / "tiny_worker.jsonl").open("x", buffering=1)
    try:
        model = OfficialTiny(Path("/official_tiny"))
        atomic_json(destination / "official_identity.json", model.identity)
        outbox.put(dict(event="READY", identity=model.identity), timeout=.5)
        while not stop.is_set() and calls < forward_limit:
            try:
                scan = inbox.get(timeout=.05)
            except queue.Empty:
                continue
            validate_scan(scan)
            started = time.monotonic_ns()
            if started-scan["received_ns"] > 500_000_000:
                outbox.put(dict(event="EXPIRED_INPUT", input_id=scan["input_id"]), timeout=.2)
                continue
            calls += 1  # Includes failed forward, never just successful results.
            steer = model.process(scan["ranges"])
            finished = time.monotonic_ns()
            result = {key: scan[key] for key in ("input_id", "source_ns", "received_ns", "scan_sequence", "frame", "ranges_sha256")}
            result.update(event="TINY_OUTPUT", tiny_forward_index=calls, started_ns=started,
                          finished_ns=finished, steering_rad=steer, acceleration_ai_used=False)
            trace.write(json.dumps(result, allow_nan=False)+"\n")
            outbox.put(result, timeout=.2)
    except BaseException as exc:
        reason = type(exc).__name__+": "+str(exc)
        try:
            outbox.put(dict(event="WORKER_EXCEPTION", reason=reason), timeout=.2)
        except queue.Full:
            pass  # Summary and process exit still expose this fault.
    finally:
        atomic_json(destination / "tiny_worker_summary.json", dict(tiny_forward_calls=calls,
            exception=reason, official_weight_forward_only=True, v4_forward_calls=0,
            forward_limit_reached=calls >= forward_limit, forward_limit=forward_limit))
        trace.close()
        # Never block interpreter exit on a queue whose supervisor has stopped.
        for transport in (inbox, outbox):
            transport.cancel_join_thread()
            transport.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--project", required=True)
    parser.add_argument("--authorize-sim-session", action="store_true")
    # ROS launch appends its own --ros-args; rclpy receives those unchanged.
    own_args = sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else None
    args = parser.parse_args(own_args)
    if not args.authorize_sim_session:
        raise ValueError("EXPLICIT_SIM_SESSION_REQUIRED")
    cfg = json.loads(args.config.read_text())
    validate_tiny_config(cfg)
    gui = cfg["authorization_profile"] in GUI_AUTHORIZATIONS
    if time.time() >= DRIVE_CUTOFF_UNIX_S-10:
        raise ValueError("CUTOFF_NO_NEW_RUNTIME")
    inspection = args.output / "instance_inspect.json"
    waiting = time.monotonic()
    while not inspection.exists() and time.monotonic()-waiting < 30:
        time.sleep(.05)
    items = json.loads(inspection.read_text())
    verify_container_contract(items, args.project, gui=gui)
    runtime_service = "autoware" if gui else "tiny"
    runtime = next(c for c in items if c["Config"]["Labels"]["com.docker.compose.service"] == runtime_service)
    expected_command = ["/v4/integrations/tiny_gui/runtime.sh"] if gui else ["/v4/integrations/tiny_dev/runtime.sh"]
    if (socket.gethostname() != runtime["Config"]["Hostname"]
            or runtime["Config"]["Cmd"] != expected_command
            or os.environ.get("ROS_DOMAIN_ID") != "1"):
        raise ValueError("WRONG_CURRENT_CONTAINER")
    if sorted(p.name for p in Path("/sys/class/net").iterdir()) != ["lo"]:
        raise ValueError("EXTERNAL_INTERFACE")
    if subprocess.run(["ip", "route"], capture_output=True, text=True, check=True).stdout.strip():
        raise ValueError("EXTERNAL_ROUTE")
    if any(p.name.startswith(("ttyUSB", "ttyACM", "can", "serial")) for p in Path("/dev").iterdir()):
        raise ValueError("PHYSICAL_DEVICE")

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport, GearCommand
    from autoware_auto_control_msgs.msg import AckermannControlCommand

    rclpy.init()
    node = rclpy.create_node("official_tiny_sim_supervisor", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    trace = (args.output / "tiny_supervisor.jsonl").open("x", buffering=1)
    counts: Counter = Counter()
    readiness_gate = SimReadiness()
    latest: dict = {}
    sim_ns = -1
    clock_rx = 0
    clock_change_rx = 0
    fault = None
    requested_stop = False
    ready = False
    frame = None
    result = None
    pending_id = None
    pubs = {}
    powered_start = None
    max_speed = 0.
    first_moving_ns = None
    stopped_start = None
    stop_fresh_samples = 0
    stopped_after_motion = False
    stopping = False
    start_ns = time.monotonic_ns()
    start_unix_ns = time.time_ns()
    last_heartbeat = last_send = last_graph = 0
    last_mode = 0
    last_steering_sent = 0.
    controls_with_tiny = set()
    last_inference_seen = -1
    natural_stop_sim_ns = None
    stop_reason = None
    logger_ok = True
    gui_verified = not gui
    context = mp.get_context("spawn")
    inbox, outbox = context.Queue(1), context.Queue(2)
    stop_event = context.Event()
    worker = context.Process(target=worker_main,
        args=(inbox, outbox, stop_event, str(args.output), cfg["forward_limit"]))

    def log(event: str, **value) -> None:
        nonlocal logger_ok
        try:
            if trace.tell() > 32*1024**2:
                raise RuntimeError("SUPERVISOR_LOG_LIMIT")
            trace.write(json.dumps(dict(event=event, monotonic_ns=time.monotonic_ns(),
                sim_ns=sim_ns, **value), allow_nan=False)+"\n")
        except Exception:
            logger_ok = False
            raise

    def heartbeat(phase: str) -> None:
        nonlocal last_heartbeat
        now = time.monotonic_ns()
        if now-last_heartbeat >= 100_000_000 or phase == "HOST_FREEZE_REQUESTED":
            atomic_json(args.output/"heartbeat.json", dict(token=args.project, monotonic_ns=now,
                logger_ok=logger_ok, phase=phase, powered=powered_start is not None))
            last_heartbeat = now

    def host_armed() -> bool:
        p = args.output / "host_armed.json"
        arm = json.loads(p.read_text()) if p.exists() else {}
        # Sampling BEFORE this read can reject a concurrent, fresher ARM as future.
        status = host_arm_status(arm, project=args.project, observed_ns=time.monotonic_ns())
        if not status["allowed"] and pubs:
            log("HOST_ARM_REJECTED", **status)
        return status["allowed"]

    def receive(role: str, msg) -> None:
        nonlocal sim_ns, clock_rx, clock_change_rx, fault, frame, max_speed
        nonlocal first_moving_ns, stopped_start, stop_fresh_samples, stopped_after_motion, natural_stop_sim_ns
        now = time.monotonic_ns()
        counts[role] += 1
        if role == "clock":
            stamp = msg.clock.sec*1_000_000_000+msg.clock.nanosec
            if sim_ns >= 0 and stamp < sim_ns:
                fault = "CLOCK_RESET"
            if stamp > sim_ns:
                clock_change_rx = now
            sim_ns, clock_rx = stamp, now
            return
        if role == "phase":
            readiness_gate.observe(msg.data, sim_ns, now, counts["scan"])
            log("AWSIM_PHASE", value=msg.data, received_ns=now,
                ready_sim_ns=readiness_gate.ready_sim_ns,
                ready_received_ns=readiness_gate.ready_received_ns,
                ready_scan_sequence=readiness_gate.ready_scan_sequence)
            return
        if role == "collision":
            log("COLLISION_MESSAGE", value=msg.data)
            if msg.data:
                fault = "COLLISION_REPORTED"
            return
        stamp = msg.header.stamp if hasattr(msg, "header") else msg.stamp
        source_ns = stamp.sec*1_000_000_000+stamp.nanosec
        if role in latest and source_ns <= latest[role]["source_ns"]:
            counts[role+"_duplicate_or_regressing"] += 1
            return
        record = dict(source_ns=source_ns, received_ns=now)
        if role == "scan":
            ranges = np.asarray(msg.ranges, dtype=np.float32).copy()
            record.update(ranges=ranges, frame=msg.header.frame_id, angle_min=msg.angle_min,
                angle_increment=msg.angle_increment, range_min=msg.range_min, range_max=msg.range_max,
                scan_time=msg.scan_time, input_id=f"scan:{source_ns}", scan_sequence=counts["scan"],
                ranges_sha256=hashlib.sha256(ranges.tobytes()).hexdigest())
            validate_scan(record)
            if frame is not None and frame != record["frame"]:
                raise ValueError("SCAN_FRAME_CHANGED")
            frame = record["frame"]
            log("SCAN_RECEIVED", **{k:v for k,v in record.items() if k != "ranges"},
                count=len(ranges), nan_count=int(np.isnan(ranges).sum()),
                inf_count=int(np.isinf(ranges).sum()), zero_count=int((ranges == 0).sum()),
                scan_clock_domain="ROS_SIM_TIME", receipt_clock_domain="HOST_MONOTONIC")
        elif role == "velocity":
            speed = float(msg.longitudinal_velocity)
            if msg.header.frame_id != "base_link" or not math.isfinite(speed):
                raise ValueError("VELOCITY_FRAME_OR_FINITE")
            record["speed_mps"] = speed
            max_speed = max(max_speed, abs(speed))
            if abs(speed) >= .1 and first_moving_ns is None:
                first_moving_ns = source_ns
            # Fresh source stamps, source-time persistence, and actual previous
            # motion distinguish observed braking from initial stationarity.
            if stopping and abs(speed) <= .03:
                if stopped_start is None:
                    stopped_start = source_ns
                stop_fresh_samples += 1
                if source_ns-stopped_start >= 500_000_000 and stop_fresh_samples >= 5:
                    stopped_after_motion = first_moving_ns is not None
                    natural_stop_sim_ns = source_ns
            elif stopping:
                stopped_start, stop_fresh_samples = None, 0
            log("VELOCITY_RECEIVED", **record, stopping=stopping)
        elif role == "steering":
            record["steering_rad"] = float(msg.steering_tire_angle)
            if not math.isfinite(record["steering_rad"]):
                raise ValueError("NONFINITE_OBSERVED_STEERING")
            log("STEERING_RECEIVED", **record)
        latest[role] = record

    subscriptions = []
    for role, topic, typ in (("scan", "/sensing/lidar/scan", LaserScan),
        ("velocity", "/vehicle/status/velocity_status", VelocityReport),
        ("steering", "/vehicle/status/steering_status", SteeringReport),
        ("clock", "/clock", Clock), ("phase", "/awsim/state", String),
        ("collision", "/awsim/ground_truth/on_collision", Bool)):
        subscriptions.append(node.create_subscription(typ, topic, lambda m, r=role: receive(r, m),
            latched if role == "phase" else qos))

    def graph_check(*, initializing: bool = False) -> bool:
        endpoints = {}
        for topic in CONTROL_TOPICS:
            subs = node.get_subscriptions_info_by_topic(topic)
            publishers = node.get_publishers_info_by_topic(topic)
            if initializing and (len(subs) != 1 or subs[0].node_name != "awsim_d1"):
                return False
            if len(subs) != 1 or subs[0].node_name != "awsim_d1":
                raise ValueError("CONSUMER_CHANGED:"+topic)
            if publishers and (initializing or any(p.node_name != node.get_name() for p in publishers)):
                raise ValueError("COMPETING_PUBLISHER:"+topic)
            endpoints[topic] = dict(subscriber=subs[0].node_name, gid=list(subs[0].endpoint_gid),
                qos=str(subs[0].qos_profile), publishers=[p.node_name for p in publishers])
        for topic in ALTERNATE_TOPICS:
            if node.count_publishers(topic):
                raise ValueError("ALTERNATE_CONTROL_PUBLISHER:"+topic)
        if initializing:
            log("CURRENT_CONSUMER_VERIFIED", endpoints=endpoints, topics=node.get_topic_names_and_types(),
                network_interfaces=["lo"], physical_device_found=False)
        return True

    def stamp(target) -> None:
        target.sec, target.nanosec = divmod(max(0, sim_ns), 1_000_000_000)

    def send(acceleration: float, steering: float, source: str, output: dict | None = None) -> None:
        nonlocal last_send, last_steering_sent, powered_start
        if not host_armed():
            raise RuntimeError("HOST_MONITOR_NOT_ARMED")
        msg = AckermannControlCommand()
        for field in (msg.stamp, msg.longitudinal.stamp, msg.lateral.stamp):
            stamp(field)
        msg.lateral.steering_tire_angle = float(steering)
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.longitudinal.acceleration = float(acceleration)
        msg.longitudinal.speed = 0.0  # Unused consumer field, NOT a speed limiter.
        # Record the request first; a subsequent send failure must not be called
        # delivered/applied. The next state report is observational, not an ack.
        operation_id = f"command:{counts['command_sent']+1}"
        value = dict(operation_id=operation_id, acceleration_mps2=acceleration, steering_rad=steering,
            source=source, input_id=output.get("input_id") if output else None,
            tiny_forward_index=output.get("tiny_forward_index") if output else None,
            output_source_ns=output.get("source_ns") if output else None,
            output_received_ns=output.get("received_ns") if output else None,
            output_scan_sequence=output.get("scan_sequence") if output else None,
            ready_sim_ns=readiness_gate.ready_sim_ns, ready_received_ns=readiness_gate.ready_received_ns,
            ready_scan_sequence=readiness_gate.ready_scan_sequence,
            first_positive_request=acceleration > 0 and powered_start is None,
            velocity_source_ns=latest.get("velocity", {}).get("source_ns"))
        if acceleration > 0 and not readiness_gate.allow_drive(output):
            raise RuntimeError("POSITIVE_REQUEST_WITHOUT_POST_READY_RECEIPT_SCAN")
        log("COMMAND_REQUEST", **value)
        if acceleration > 0 and powered_start is None:
            # Conservative: a publisher exception cannot prove no delivery.
            powered_start = sim_ns
        pubs["control"].publish(msg)
        last_send, last_steering_sent = time.monotonic_ns(), steering
        counts["command_sent"] += 1
        if source == "TINY_STEERING_WITH_SPEED_LIMITER":
            counts["tiny_steering_sent"] += 1
            controls_with_tiny.add(output["input_id"])
        log("COMMAND_SENT_NOT_APPLIED_ACK", **value)

    def shutdown_signal(signum, _frame) -> None:
        nonlocal requested_stop, stop_reason
        requested_stop, stop_reason = True, f"SIGNAL_{signum}"

    signal.signal(signal.SIGTERM, shutdown_signal)
    signal.signal(signal.SIGINT, shutdown_signal)
    worker.start()
    try:
        log("SESSION_START", source_commit=os.environ["TINY_SOURCE_COMMIT"], config=cfg, unix_ns=start_unix_ns)
        while time.monotonic_ns()-start_ns < cfg["wall_seconds"]*1e9:
            heartbeat("ACTIVE")
            rclpy.spin_once(node, timeout_sec=.002)
            now = time.monotonic_ns()
            if fault:
                raise RuntimeError(fault)
            if requested_stop:
                break
            if time.time() >= DRIVE_CUTOFF_UNIX_S-10:
                stop_reason = "DEADLINE_BRAKE_RESERVE"
                break
            request_path = args.output / "stop_request.json"
            if request_path.exists():
                request = json.loads(request_path.read_text())
                if request.get("token") != args.project:
                    raise ValueError("STOP_REQUEST_IDENTITY")
                stop_reason = request["reason"]
                break
            if not worker.is_alive():
                worker_final = args.output/"tiny_worker_summary.json"
                if worker_final.exists() and json.loads(worker_final.read_text()).get("forward_limit_reached") is True:
                    stop_reason = "FORWARD_LIMIT_REACHED"
                    break
                raise RuntimeError("MODEL_WORKER_EXITED")
            try:
                answer = outbox.get_nowait()
                log("WORKER_MESSAGE", answer=answer)
                if answer["event"] == "READY":
                    ready = True
                elif answer["event"] == "WORKER_EXCEPTION":
                    raise RuntimeError(answer["reason"])
                elif answer["event"] == "TINY_OUTPUT":
                    if answer["source_ns"] <= last_inference_seen:
                        raise ValueError("TINY_OUTPUT_ORDER")
                    result = answer
                    last_inference_seen = answer["source_ns"]
                    counts["tiny_outputs"] += 1
                pending_id = None
            except queue.Empty:
                pass
            if not pubs:
                if (ready and host_armed() and all(k in latest for k in ("scan", "velocity", "steering"))
                        and sim_ns >= 0 and graph_check(initializing=True)):
                    if abs(latest["velocity"]["speed_mps"]) > .03:
                        raise ValueError("START_NOT_STATIONARY")
                    pubs["control"] = node.create_publisher(AckermannControlCommand, CONTROL_TOPICS[0], 32)
                    pubs["mode"] = node.create_publisher(Bool, CONTROL_TOPICS[1], 1)
                    pubs["gear"] = node.create_publisher(GearCommand, CONTROL_TOPICS[2], 1)
                    send(0., 0., "INITIAL_STATIONARY_HOLD")
                elif now-start_ns > 40_000_000_000:
                    raise RuntimeError("STARTUP_MODEL_SCAN_CONSUMER_TIMEOUT")
                else:
                    continue
            if not host_armed():
                raise RuntimeError("HOST_MONITOR_STALE")
            if now-last_graph >= 250_000_000:
                graph_check()
                last_graph = now
            if now-clock_rx > 500_000_000 or now-clock_change_rx > 500_000_000:
                raise RuntimeError("SIM_CLOCK_STALE_OR_FROZEN")
            # Report callbacks and /clock are independent. Wait for the clock
            # watermark without changing stamps or extending a powered lease.
            if powered_start is not None and result is not None:
                validate_operation(result, now_ns=now, sim_ns=sim_ns,
                                   steering_limit_rad=cfg["steering_limit_rad"])
            readiness = [input_clock_ready(latest[role], now_ns=now, sim_ns=sim_ns)
                         for role in ("scan", "velocity", "steering")]
            if not all(readiness):
                counts["clock_watermark_wait"] += 1
                continue
            # Repeated mode/gear asserts only our declared selected consumer.
            if now-last_mode > 500_000_000:
                mode = Bool(); mode.data = True; pubs["mode"].publish(mode)
                gear = GearCommand(); stamp(gear.stamp); gear.command = 2; pubs["gear"].publish(gear)
                log("MODE_GEAR_SENT", autonomous=True, gear_command=2)
                last_mode = now
            scan = latest["scan"]
            if (pending_id is None and scan["source_ns"] > last_inference_seen
                    and readiness_gate.allow_inference(counts["tiny_outputs"], cfg["phase"] == "stationary")):
                try:
                    inbox.put_nowait(scan)
                    pending_id = scan["input_id"]
                    log("INFERENCE_INPUT_QUEUED", input_id=pending_id)
                except queue.Full:
                    counts["input_queue_full"] += 1
            if cfg["phase"] != "stationary" and not readiness_gate.allow_drive(result):
                if now-last_send >= cfg["command_period_s"]*1e9:
                    send(0., 0., "WAIT_AWSIM_READY_AND_FRESH_TINY")
                continue
            if result is None:
                continue
            validate_operation(result, now_ns=now, sim_ns=sim_ns,
                               steering_limit_rad=cfg["steering_limit_rad"])
            if cfg["phase"] == "stationary":
                if counts["tiny_outputs"] >= cfg["stationary_forwards"]:
                    stop_reason = "STATIONARY_TINY_CHECK_COMPLETE"
                    break
                continue
            # First three fresh results while stationary, then first actuation.
            if counts["tiny_outputs"] < 3:
                continue
            if not gui_verified:
                proof = args.output/"gui_ready.json"
                if proof.exists():
                    visible = json.loads(proof.read_text())
                    if visible.get("token") != args.project or set(visible.get("windows", {})) != {"awsim", "rviz"}:
                        raise ValueError("GUI_READY_IDENTITY")
                    gui_verified = True
                    log("GUI_READY_BEFORE_DRIVE", evidence=visible)
                else:
                    if now-last_send >= cfg["command_period_s"]*1e9:
                        send(0., 0., "WAIT_VISIBLE_AWSIM_AND_RVIZ")
                    continue
            if powered_start is not None:
                duration = (sim_ns-powered_start)/1e9
                limit_reason = motion_limit_reason(cfg, duration)
                if limit_reason:
                    stop_reason = limit_reason
                    break
            if now-last_send >= cfg["command_period_s"]*1e9:
                acceleration = speed_acceleration(latest["velocity"]["speed_mps"],
                    target_mps=cfg["target_speed_mps"], maximum_mps=cfg["maximum_speed_mps"])
                send(acceleration, result["steering_rad"], "TINY_STEERING_WITH_SPEED_LIMITER", result)
        else:
            stop_reason = "SESSION_WALL_LIMIT"
    except BaseException as exc:
        fault = type(exc).__name__+": "+str(exc)
        stop_reason = fault
        try:
            log("SUPERVISOR_EXCEPTION", reason=fault)
        except Exception:
            logger_ok = False
    finally:
        stopping = True
        stop_event.set()
        stop_start = time.monotonic_ns()
        try:
            log("STOP_BEGIN", reason=stop_reason, initially_stationary=first_moving_ns is None)
            while pubs and time.monotonic_ns()-stop_start < cfg["stop_wall_seconds"]*1e9:
                heartbeat("STOPPING")
                rclpy.spin_once(node, timeout_sec=.005)
                now = time.monotonic_ns()
                if not host_armed() or not logger_ok:
                    break
                if (now-clock_change_rx > 500_000_000 or "velocity" not in latest
                        or now-latest["velocity"]["received_ns"] > 500_000_000):
                    break  # Host freeze handles missing state, not guessed brake.
                graph_check()
                if powered_start is not None and (sim_ns-powered_start)/1e9 >= cfg["single_episode_sim_limit_s"]:
                    break
                if natural_stop_sim_ns is not None:
                    send(0., last_steering_sent, "OBSERVED_STOP_HOLD")
                    break
                if now-last_send >= cfg["command_period_s"]*1e9:
                    acceleration = speed_acceleration(latest["velocity"]["speed_mps"],
                        target_mps=cfg["target_speed_mps"], maximum_mps=cfg["maximum_speed_mps"], stop=True)
                    send(acceleration, last_steering_sent, "INDEPENDENT_SUPERVISOR_BRAKE")
        except BaseException as exc:
            fault = fault or type(exc).__name__+": "+str(exc)
        heartbeat("HOST_FREEZE_REQUESTED")
        # Host performs pause before expensive process finalization. No unpause.
        end_wait = time.monotonic()+3.
        while not (args.output/"finalization_freeze.json").exists() and time.monotonic() < end_wait:
            time.sleep(.05)
        worker.join(timeout=.3)
        worker_terminated = worker.is_alive()
        if worker_terminated:
            worker.terminate(); worker.join(timeout=.3)
        if worker.is_alive():
            worker.kill(); worker.join(timeout=.3)
        for transport in (inbox, outbox):
            transport.cancel_join_thread(); transport.close()
        summary = dict(source_commit=os.environ["TINY_SOURCE_COMMIT"], phase=cfg["phase"],
            stop_reason=stop_reason, exception=fault, counters=dict(counts),
            powered_episode_count=int(powered_start is not None), powered_start_sim_ns=powered_start,
            powered_sim_seconds=(sim_ns-powered_start)/1e9 if powered_start is not None else 0.,
            max_abs_speed_mps=max_speed, observed_motion=first_moving_ns is not None,
            observed_braking_stop=stopped_after_motion, natural_stop_sim_ns=natural_stop_sim_ns,
            distinct_tiny_inputs_sent=len(controls_with_tiny), worker_terminated=worker_terminated,
            logger_ok=logger_ok, source_frame=frame, full_sensor_saved=False,
            awsim_ready_sim_ns=readiness_gate.ready_sim_ns,
            awsim_ready_received_ns=readiness_gate.ready_received_ns,
            awsim_ready_scan_sequence=readiness_gate.ready_scan_sequence,
            start_unix_ns=start_unix_ns, finish_unix_ns=time.time_ns(),
            hash_does_not_reproduce_sensor=True, host_pause_not_braking_proof=True,
            lap_result="HOST_JUDGE_LOG_REQUIRED", v4_forwards=0, mpc_calls=0,
            learned_acceleration_used=False, physical_device_control=False)
        atomic_json(args.output/"tiny_supervisor_summary.json", summary)
        trace.close()
        node.destroy_node()
        rclpy.shutdown()
    return int(fault is not None)


if __name__ == "__main__":
    raise SystemExit(main())
