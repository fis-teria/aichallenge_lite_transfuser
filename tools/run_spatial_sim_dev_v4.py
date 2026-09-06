"""Finite make-dev ROS supervisor. The ONLY command publisher, separate from ML/MPC.

No ROS imports or publisher creation until actual container isolation is checked.
This is a new SIM_E2E_CONTROLLED_TEST entry; the private shadow is unchanged.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict
import importlib.metadata
import json
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
import yaml

from aic_transfuser_lite.control.spatial_sim_guard_v4 import OperationLease
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import plain
from aic_transfuser_lite.runtime.spatial_sim_adapter_v4 import Sample, base_pose_from_gnss_imu, interpolate


def verify_dev_isolation(items: list[dict], project: str) -> None:
    """Actual docker inspect plus in-container network checks, not domain alone."""
    if len(items) != 2 or not project.startswith('codex-v4-dev-'):
        raise ValueError('UNEXPECTED_COMPOSE_SCOPE')
    indexed = {c['Config']['Labels']['com.docker.compose.service']: c for c in items}
    if set(indexed) != {'simulator', 'autoware'}:
        raise ValueError('UNEXPECTED_SERVICES')
    sim = indexed['simulator']
    for name, c in indexed.items():
        h = c['HostConfig']
        if (c['Config']['Labels'].get('com.docker.compose.project') != project or h['Privileged']
                or h['Devices'] or h['CapAdd'] or h['CapDrop'] != ['ALL'] or h['PidMode']
                or h['IpcMode'] != 'private' or 'no-new-privileges:true' not in h['SecurityOpt']):
            raise ValueError('UNSAFE_CONTAINER')
        mode = 'none' if name == 'simulator' else 'container:'+sim['Id']
        if h['NetworkMode'] != mode:
            raise ValueError('UNSAFE_NETWORK')
        if c['Image'] != 'sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7':
            raise ValueError('UNEXPECTED_RUNTIME_IMAGE')
        for mount in c['Mounts']:
            if mount['Destination'] not in ('/v4', '/evidence', '/aichallenge/simulator/AWSIM',
                    '/aichallenge/run_simulator.bash', '/xvfb', '/tmp/.X11-unix', '/usr/bin/xkbcomp',
                    '/home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197/checkpoints/final.pt'):
                raise ValueError('UNEXPECTED_MOUNT')
            if mount['Destination'] not in ('/evidence', '/tmp/.X11-unix') and mount['RW']:
                raise ValueError('WRITABLE_SOURCE_OR_SIMULATOR')
    current = indexed['autoware']
    # Shared-network Docker services can inherit the simulator's hostname.
    # Compare the actual inspected setting, not an assumed ID prefix. This
    # check is supplementary to the complete namespace/mount checks above.
    if (socket.gethostname() != current['Config']['Hostname']
            or current['Config']['Cmd'] != ['/v4/integrations/awsim_dev_v4/runtime.sh']):
        raise ValueError('WRONG_CURRENT_CONTAINER')
    if [p.name for p in Path('/sys/class/net').iterdir()] != ['lo']:
        raise ValueError('EXTERNAL_INTERFACE')
    if subprocess.run(['ip', 'route'], capture_output=True, check=True, text=True).stdout.strip():
        raise ValueError('EXTERNAL_ROUTE')
    if any(p.name.startswith(('ttyUSB', 'ttyACM', 'can', 'serial')) for p in Path('/dev').iterdir()):
        raise ValueError('PHYSICAL_DEVICE')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--project', required=True)
    ap.add_argument('--authorize-sim-session', action='store_true')
    args = ap.parse_args()
    if not args.authorize_sim_session:
        raise ValueError('EXPLICIT_SIM_SESSION_REQUIRED')
    cfg = yaml.safe_load(args.config.read_text())
    if not (0 < cfg['dev']['wall_limit_s'] <= 120 and 0 < cfg['dev']['forward_limit'] <= 60):
        raise ValueError('FINITE_SESSION_REQUIRED')
    inspection = args.output/'instance_inspect.json'
    waiting = time.monotonic()
    while not inspection.exists() and time.monotonic()-waiting < 30:
        time.sleep(.10)
    verify_dev_isolation(json.loads(inspection.read_text()), args.project)
    if os.environ.get('ROS_DOMAIN_ID') != '1':
        raise ValueError('UNEXPECTED_DOMAIN')
    # Only now resolve ROS and launch the read-only fixed model worker.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, LaserScan, NavSatFix, Imu
    from std_msgs.msg import Bool, String
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport, GearCommand
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from pyproj import Transformer
    from aic_transfuser_lite.runtime.spatial_sim_worker_v4 import worker_main

    rclpy.init()
    node = rclpy.create_node('lite_transfuser_mpc_supervisor', enable_rosout=False, start_parameter_services=False)
    sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
    latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    buffers = {role: deque(maxlen=11 if role == 'camera' else 64)
               for role in ('camera', 'lidar', 'velocity', 'steering', 'imu', 'fix', 'pose')}
    counts: Counter = Counter()
    clocks: deque = deque(maxlen=64)
    commands: deque = deque(maxlen=64)
    epoch = '0'
    sim_ns = -1
    phase = None
    requested_stop = False
    first_error = None
    grid_phase = None
    trace = (args.output/'supervisor.jsonl').open('x', buffering=1)
    lease = OperationLease(cfg['maximum_speed_mps'])
    context = mp.get_context('spawn')
    inbox, outbox = context.Queue(1), context.Queue(2)
    stop = context.Event()
    worker = context.Process(target=worker_main, args=(inbox, outbox, stop, str(args.config), str(args.output)))
    start_ns = time.monotonic_ns()
    worker.start()
    ready = False
    pub = mode_pub = gear_pub = None
    last_packet = last_sent_sim = -1
    last_accepted_ns = start_ns
    last_send_ns = 0
    last_heartbeat_ns = 0
    accepted_deadline_ns = 0
    latest_input_id = ''
    current_result = None
    logger_ok = True
    pose_origin = None
    pose_points: list = []
    converter = Transformer.from_crs('EPSG:4326', 'EPSG:32654', always_xy=True)
    max_speed = 0.
    distance = 0.
    motion_first_sim_ns = None
    stopped_since = None

    def log(event: dict) -> None:
        nonlocal logger_ok
        try:
            if trace.tell() > 64*1024*1024:
                raise RuntimeError('SUPERVISOR_LOG_BUDGET')
            trace.write(json.dumps(plain(dict(monotonic_ns=time.monotonic_ns(), sim_ns=sim_ns,
                                             epoch=epoch, **event)), separators=(',', ':'), allow_nan=False)+'\n')
            trace.flush()
        except Exception:
            logger_ok = False
            raise

    def receive(role: str, message: object) -> None:
        nonlocal sim_ns, epoch, phase, grid_phase, first_error, max_speed, pose_origin, distance
        now = time.monotonic_ns()
        counts[role] += 1
        if role == 'clock':
            ns = message.clock.sec*1_000_000_000+message.clock.nanosec
            if sim_ns >= 0 and ns < sim_ns:
                epoch = str(int(epoch)+1)
                for b in buffers.values(): b.clear()
                commands.clear()
                grid_phase = pose_origin = None
                lease.fault = 'CLOCK_RESET'
            sim_ns = ns
            clocks.append((ns, now))
            return
        if role == 'phase':
            phase = message.data
            log(dict(event='AWSIM_PHASE', phase=phase))
            return
        header = getattr(message, 'header', None)
        stamp = header.stamp if header is not None else message.stamp
        ns = stamp.sec*1_000_000_000+stamp.nanosec
        frame = header.frame_id if header is not None else 'steering_tire_angle'
        if role == 'camera':
            if message.encoding not in ('bgr8', 'rgb8') or message.height != 256 or message.width != 384:
                raise ValueError('UNSUPPORTED_CAMERA')
            value = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width*3].reshape(message.height, message.width, 3)
            value = value[..., ::-1].copy() if message.encoding == 'bgr8' else value.copy()
            if grid_phase is None: grid_phase = ns
        elif role == 'lidar':
            value = dict(ranges=np.asarray(message.ranges, dtype=np.float32).copy(), angle_min=message.angle_min,
                         angle_increment=message.angle_increment, range_min=message.range_min,
                         range_max=message.range_max, scan_time=message.scan_time)
        elif role == 'velocity':
            if frame != 'base_link': raise ValueError('VELOCITY_FRAME')
            value = [message.longitudinal_velocity, message.lateral_velocity, message.heading_rate]
            max_speed = max(max_speed, abs(value[0]))
            if counts[role] % 3 == 0: log(dict(event='OBSERVED_VELOCITY', source_ns=ns, value=value))
        elif role == 'steering': value = [message.steering_tire_angle]
        elif role == 'imu':
            if frame != 'imu_link': raise ValueError('IMU_FRAME')
            q = message.orientation
            value = [q.x, q.y, q.z, q.w]
        elif role == 'fix':
            if frame != 'gnss_link' or message.status.status < 0: return
            value = list(converter.transform(message.longitude, message.latitude))
        else: return
        b = buffers[role]
        if b and ns <= b[-1].ns:
            counts[role+'_duplicate_or_out_of_order'] += 1
            return
        b.append(Sample(ns, now, value, frame, epoch))

    def update_pose() -> None:
        nonlocal pose_origin, distance
        # Retry pending GNSS after the right-hand IMU bracket actually arrives.
        # Sensor source stamps remain unchanged; availability includes both.
        for fix in list(buffers['fix']):
            ns, value, now = fix.ns, fix.value, fix.received_ns
            if buffers['pose'] and ns <= buffers['pose'][-1].ns: continue
            try:
                orientations = [Sample(s.ns, s.received_ns,
                    base_pose_from_gnss_imu(np.zeros(2), s.value)[2:3], 'base_link_yaw', epoch)
                    for s in buffers['imu']]
                yaw, basis = interpolate(orientations, ns, angle_columns=(0,))
                p = np.r_[np.asarray(value)+.26*np.array([np.cos(yaw[0]), np.sin(yaw[0])]), yaw[0]]
                if pose_origin is None: pose_origin = p[:2].copy()
                p[:2] -= pose_origin
                # Duplicate or regressing source stamps never contribute travel.
                if buffers['pose']:
                    delta = float(np.linalg.norm(p[:2]-buffers['pose'][-1].value[:2]))
                    if delta > .5:
                        lease.fault = 'POSE_JUMP'
                    # GNSS-noise-contaminated cumulative distance is diagnostic,
                    # never used as success; net displacement is separately saved.
                    distance += delta
                buffers['pose'].append(Sample(ns, max(now, basis['available_ns']), p, 'utm_local_base', epoch))
                if len(pose_points) < 4000: pose_points.append([ns, *p.tolist()])
                log(dict(event='GNSS_IMU_POSE', source_ns=ns, pose=p, basis=basis,
                         source='CURRENT_GNSS_IMU_NO_ROUTE', timing_verified=False))
            except ValueError:
                counts['pose_alignment_missing'] += 1

    subscriptions = []
    for role, topic, typ in (
        ('camera', '/sensing/camera/image_raw', Image), ('lidar', '/sensing/lidar/scan', LaserScan),
        ('velocity', '/vehicle/status/velocity_status', VelocityReport),
        ('steering', '/vehicle/status/steering_status', SteeringReport),
        ('fix', '/sensing/gnss/nav_sat_fix', NavSatFix), ('imu', '/sensing/imu/imu_raw', Imu),
        ('clock', '/clock', Clock), ('phase', '/awsim/state', String)):
        subscriptions.append(node.create_subscription(typ, topic, lambda m, r=role:receive(r, m),
                             latched_qos if role == 'phase' else sensor_qos))

    def requested_signal(_signum: int, _frame: object) -> None:
        nonlocal requested_stop
        requested_stop = True
    signal.signal(signal.SIGTERM, requested_signal)
    signal.signal(signal.SIGINT, requested_signal)

    def stamp_message(target: object, ns: int) -> None:
        target.sec, target.nanosec = divmod(max(ns, 0), 1_000_000_000)

    def publish(a: float, delta: float, speed: float, operation_id: str, source: str) -> None:
        nonlocal last_sent_sim, last_send_ns
        msg = AckermannControlCommand()
        for field in (msg.stamp, msg.lateral.stamp, msg.longitudinal.stamp): stamp_message(field, sim_ns)
        msg.lateral.steering_tire_angle = float(delta)
        msg.longitudinal.acceleration = float(a)
        msg.longitudinal.speed = float(speed)  # desired reference; AWSIM ignores it
        pub.publish(msg)
        sent = time.monotonic_ns()
        counts['control_publish_calls'] += 1
        counts['mpc_publish_calls'] += int(source == 'MPC')
        receipt = dict(operation_id=operation_id, sim_ns=sim_ns, monotonic_ns=sent, epoch=epoch,
                       steering_rad=delta, acceleration_mps2=a, desired_speed_reference_mps=speed,
                       policy='SIM_ONLY_POLICY_CHANGED', status='SENT_NOT_APPLIED_CONFIRMED', source=source)
        commands.append(receipt)
        last_sent_sim, last_send_ns = sim_ns, sent
        log(dict(event='COMMAND_SENT', receipt=receipt))

    try:
        log(dict(event='SESSION_START', scope='SIM_E2E_CONTROLLED_TEST', config=cfg,
                 source_commit=os.environ.get('V4_SOURCE_COMMIT'), dependencies={
                     k:importlib.metadata.version(k) for k in ('torch', 'numpy', 'scipy', 'pyproj')}))
        deadline = start_ns+int(cfg['dev']['wall_limit_s']*1e9)
        while not requested_stop and time.monotonic_ns() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            now = time.monotonic_ns()
            update_pose()
            if now-last_heartbeat_ns >= 100_000_000:
                heartbeat_tmp = args.output/'heartbeat.tmp'
                heartbeat_tmp.write_text(json.dumps(dict(monotonic_ns=now, powered=lease.powered,
                                                         logger_ok=logger_ok, phase='RUNNING')))
                heartbeat_tmp.replace(args.output/'heartbeat.json')
                last_heartbeat_ns = now
            try:
                answer = outbox.get_nowait()
                if answer['event'] == 'READY': ready = True
                elif answer['event'] in ('WORKER_EXCEPTION', 'WORKER_FINISHED'):
                    log(answer)
                    first_error = answer.get('reason')
                    break
                elif answer['event'] == 'CYCLE': current_result = answer
                counts['worker_results'] += 1
            except queue.Empty:
                pass
            if pub is None:
                endpoints = node.get_subscriptions_info_by_topic('/control/command/control_cmd')
                publisher_count = node.count_publishers('/control/command/control_cmd')
                if (ready and len(endpoints) == 1 and endpoints[0].node_name == 'awsim_d1'
                        and publisher_count == 0 and buffers['velocity'] and buffers['steering'] and sim_ns >= 0):
                    for topic in ('/awsim/control_mode_request_topic', '/control/command/gear_cmd'):
                        e = node.get_subscriptions_info_by_topic(topic)
                        if len(e) != 1 or e[0].node_name != 'awsim_d1' or node.count_publishers(topic):
                            raise ValueError('UNVERIFIED_MODE_OR_GEAR_CONSUMER')
                    log(dict(event='CONSUMER_VERIFIED', control_subscriber=endpoints[0].node_name,
                             endpoint_gid=list(endpoints[0].endpoint_gid), qos=str(endpoints[0].qos_profile)))
                    pub = node.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 32)
                    mode_pub = node.create_publisher(Bool, '/awsim/control_mode_request_topic', 1)
                    gear_pub = node.create_publisher(GearCommand, '/control/command/gear_cmd', 1)
                    # Actual stationary HOLD establishes sent history, not fake zeros.
                    publish(0., buffers['steering'][-1].value[0], 0., 'initial_hold', 'SUPERVISOR_HOLD')
                    mode = Bool(); mode.data = True; mode_pub.publish(mode); counts['mode_publish_calls'] += 1
                    gear = GearCommand(); stamp_message(gear.stamp, sim_ns); gear.command = 2
                    gear_pub.publish(gear); counts['gear_publish_calls'] += 1
                elif now-start_ns > 40_000_000_000:
                    raise RuntimeError('STARTUP_CONSUMER_OR_MODEL_TIMEOUT')
                else: continue
            if node.count_publishers('/control/command/control_cmd') != 1:
                raise RuntimeError('COMPETING_CONTROLLER')
            if not worker.is_alive():
                if worker.exitcode == 0 and (args.output/'worker_summary.json').exists(): break
                raise RuntimeError('WORKER_EXITED')
            if buffers['camera'] and buffers['camera'][-1].ns != last_packet and buffers['lidar']:
                cam = buffers['camera'][-1]
                # Delay selection until ego brackets have had a bounded opportunity.
                if now-cam.received_ns >= 35_000_000:
                    input_id = f'{epoch}:camera:{cam.ns}'
                    bundle = {name:list(b) for name, b in buffers.items()}
                    bundle.update(epoch=epoch, cutoff_ns=now, grid_phase_ns=grid_phase, input_id=input_id,
                        sent_commands=list(commands), isolation=True, consumer=True,
                        pose_timing_verified=False, collision_monitor_healthy=False,
                        run_requested=cfg['enabled'], previous_acceleration=commands[-1]['acceleration_mps2'],
                        control_dt_s=.1 if last_sent_sim < 0 else (sim_ns-last_sent_sim)*1e-9)
                    try:
                        inbox.put_nowait(bundle)
                        latest_input_id = input_id
                        last_packet = cam.ns
                        counts['input_bundles_submitted'] += 1
                    except queue.Full:
                        counts['input_queue_full'] += 1
            velocity = buffers['velocity'][-1]
            speed = float(velocity.value[0])
            why = lease.watchdog(now_ns=now, last_accepted_ns=last_accepted_ns,
                                 state_received_ns=velocity.received_ns, speed_mps=speed,
                                 worker_alive=worker.is_alive(), logger_ok=logger_ok)
            if lease.fault: break
            if current_result is not None:
                rejection = lease.reject(current_result, now_ns=now, now_sim_ns=sim_ns, epoch=epoch,
                    current_input_id=latest_input_id, current_state_ns=velocity.ns, healthy=logger_ok)
                log(dict(event='DISPATCH_DECISION', input_id=current_result['input_id'], rejection=rejection,
                         worker_reason=current_result.get('reason'), motion_rejection=current_result.get('motion_rejection')))
                if rejection is None:
                    request = current_result['request']
                    # Recheck expiry immediately before sole-authority publication.
                    if time.monotonic_ns() >= current_result['deadline_monotonic_ns']:
                        lease.fault = 'EXPIRED_BEFORE_PUBLISH'
                        break
                    publish(request['acceleration_mps2'], request['steering_tire_angle_rad'],
                            request['desired_speed_reference_mps'], request['operation_id'], 'MPC')
                    lease.sent(request); last_accepted_ns = time.monotonic_ns()
                    accepted_deadline_ns = current_result['deadline_monotonic_ns']
                    if lease.powered and motion_first_sim_ns is None: motion_first_sim_ns = sim_ns
                current_result = None
            if now-last_send_ns >= 50_000_000 and (not lease.powered or why or now >= accepted_deadline_ns):
                # Never replay a worker's positive control. On expiry the independent
                # publisher immediately sends braking, while the worker may block.
                acceleration = -cfg['braking_max_mps2'] if speed > .01 or why == 'STATE_STALE' else 0.
                publish(acceleration, buffers['steering'][-1].value[0], 0.,
                        f'hold:{counts["control_publish_calls"]}', 'SUPERVISOR_HOLD')
            if motion_first_sim_ns is not None and sim_ns-motion_first_sim_ns >= 60_000_000_000: break
        log(dict(event='STOP_REQUESTED', reason=lease.fault or first_error or 'FINITE_SESSION_END'))
    except BaseException as exc:
        first_error = type(exc).__name__+': '+str(exc)
        try: log(dict(event='SUPERVISOR_EXCEPTION', reason=first_error))
        except Exception: pass
    finally:
        # Solver is not awaited before this independent stop loop.
        stop.set()
        stop_deadline = time.monotonic()+3.
        try:
            while pub is not None and time.monotonic() < stop_deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if not buffers['velocity']: break
                v = float(buffers['velocity'][-1].value[0])
                publish(-cfg['braking_max_mps2'] if v > .01 else 0., buffers['steering'][-1].value[0],
                        0., f'stop:{counts["control_publish_calls"]}', 'SUPERVISOR_STOP')
                fresh = time.monotonic_ns()-buffers['velocity'][-1].received_ns < 250_000_000
                if fresh and abs(v) <= .03:
                    if stopped_since is None: stopped_since = sim_ns
                    if sim_ns-stopped_since >= 1_000_000_000: break
                else: stopped_since = None
        except Exception as exc:
            first_error = first_error or 'STOP_LOOP:'+str(exc)
        worker.join(timeout=8)
        if worker.is_alive():
            # Stop confirmation unavailable or blocked worker: host runner stops
            # only the owned simulator container. Do not pretend the vehicle stopped.
            first_error = first_error or 'WORKER_DID_NOT_EXIT'
            worker.terminate(); worker.join(timeout=2)
        stopped = stopped_since is not None and sim_ns-stopped_since >= 1_000_000_000
        observed_displacement = float(np.linalg.norm(buffers['pose'][-1].value[:2])) if buffers['pose'] else None
        summary = dict(scope='SIM_E2E_CONTROLLED_TEST', counts=dict(counts), first_error=first_error,
            watchdog_fault=lease.fault, worker_exitcode=worker.exitcode, maximum_observed_speed_mps=max_speed,
            observed_net_displacement_m=observed_displacement, gnss_noisy_accumulated_distance_m=distance,
            pose_points=pose_points, stationary_stop_confirmed=stopped, powered_episode_count=int(lease.powered),
            powered_sim_seconds=0 if motion_first_sim_ns is None else (sim_ns-motion_first_sim_ns)*1e-9,
            wall_seconds=(time.monotonic_ns()-start_ns)*1e-9, clocks=list(clocks),
            E2E_SIM_CLOSED_LOOP_EXECUTED=False, LOW_SPEED_RUN_AND_STOP_PASSED=False,
            REALTIME_AT_1X='NOT_ESTABLISHED', collision_status='UNKNOWN',
            note='Stationary HOLD is not a powered E2E run; live state timing and footprint gates remain explicit')
        (args.output/'supervisor_summary.json').write_text(json.dumps(plain(summary), indent=2, allow_nan=False))
        trace.close(); node.destroy_node(); rclpy.shutdown()
    return 1 if first_error else 0


if __name__ == '__main__':
    raise SystemExit(main())
