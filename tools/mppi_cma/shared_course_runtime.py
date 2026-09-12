"""Four MPPI stacks and one AWSIM, with a single synchronized finite race."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_msgs.msg import Bool, String, Float32MultiArray
from v2x_msgs.msg import V2XVehiclePositionArray
from visualization_msgs.msg import Marker, MarkerArray

from runtime import stamp
from shared_course_state import all_vehicles_ready


def main() -> int:
    out = Path('/eval')
    cfg = json.loads((out / 'config.json').read_text())
    if cfg['vehicle_count'] != 4:
        raise ValueError('This runtime requires exactly four vehicles')
    contexts, nodes, executors, processes, logs, controllers = [], [], [], [], [], []
    best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'admin': '', 'started': False, 'finish': False, 'summary': None}
    cars = [dict(vehicle_number=d, ego=None, gnss=None, reference=None, command=None,
                 status=None, debug=None, observed_vehicle_ids=[]) for d in range(1, 5)]
    begin = time.monotonic()
    stopped = False
    samples = (out / 'samples.jsonl').open('w', buffering=1)
    events = (out / 'events.jsonl').open('w', buffering=1)
    summaries = (out / 'admin.jsonl').open('w', buffering=1)

    def stop(_number: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def event(kind: str, data: object) -> None:
        events.write(json.dumps({'wall_s': time.monotonic() - begin, 'kind': kind, 'data': data}) + '\n')

    for domain in range(5):
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        node = rclpy.create_node(f'shared_course_observer_{domain}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        contexts.append(context); nodes.append(node); executors.append(executor)

    def admin(msg: String) -> None:
        if state['admin'] != msg.data:
            event('admin_state', msg.data)
        state['admin'] = msg.data
        if msg.data.lower() in {'finishall', 'finishedall', 'terminate', 'terminated'}:
            state['finish'] = True

    def summary(msg: String) -> None:
        state['summary'] = json.loads(msg.data)
        summaries.write(json.dumps({'wall_s': time.monotonic() - begin, 'data': state['summary']}) + '\n')

    nodes[0].create_subscription(String, '/admin/awsim/state', admin, latched)
    nodes[0].create_subscription(String, '/admin/summary', summary, latched)
    start_pub = nodes[0].create_publisher(Bool, '/admin/awsim/start', latched)
    brakes, empty_v2x = [], []

    def observe(domain: int) -> None:
        car, node = cars[domain - 1], nodes[domain]

        def odom(msg: Odometry) -> None:
            q = msg.pose.pose.orientation
            car['ego'] = {'stamp_s': stamp(msg), 'arrival_s': time.monotonic(),
                          'x_m': msg.pose.pose.position.x, 'y_m': msg.pose.pose.position.y,
                          'speed_mps': msg.twist.twist.linear.x,
                          'yaw_rad': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}

        def gnss(msg: PoseWithCovarianceStamped) -> None:
            car['gnss'] = {'stamp_s': stamp(msg), 'arrival_s': time.monotonic(),
                           'x_m': msg.pose.pose.position.x, 'y_m': msg.pose.pose.position.y}

        def reference(msg: Trajectory) -> None:
            if msg.points:
                car['reference'] = {'stamp_s': stamp(msg), 'points': len(msg.points),
                                    'x_m': msg.points[0].pose.position.x,
                                    'y_m': msg.points[0].pose.position.y}

        def command(msg: AckermannControlCommand) -> None:
            car['command'] = {'stamp_s': stamp(msg), 'speed_mps': msg.longitudinal.speed,
                              'acceleration_mps2': msg.longitudinal.acceleration,
                              'steering_rad': msg.lateral.steering_tire_angle}

        def status(msg: Float32MultiArray) -> None:
            car['status'] = list(msg.data)

        def debug(msg: String) -> None:
            car['debug'] = msg.data

        def v2x(msg: V2XVehiclePositionArray) -> None:
            car['observed_vehicle_ids'] = sorted({v.vehicle_id for v in msg.vehicles})

        node.create_subscription(Odometry, '/localization/kinematic_state', odom, best)
        node.create_subscription(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', gnss, best)
        node.create_subscription(Trajectory, '/planning/scenario_planning/trajectory', reference, best)
        node.create_subscription(AckermannControlCommand, '/control/command/control_cmd', command, best)
        node.create_subscription(Float32MultiArray, '/awsim/status', status, best)
        node.create_subscription(String, '/pure_pursuit/debug', debug, best)
        node.create_subscription(V2XVehiclePositionArray, '/v2x/vehicle_positions', v2x, best)
        brakes.append(node.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10))
        if cfg['ignore_other_vehicles']:
            empty_v2x.append(node.create_publisher(V2XVehiclePositionArray, '/cma/ghost/vehicle_positions', 10))

    for domain in range(1, 5):
        observe(domain)
    marker_pub = nodes[1].create_publisher(MarkerArray, '/cma/shared/vehicles', 10)

    def display_cars() -> None:
        markers = MarkerArray()
        colors = [(0.5, .69, 1.), (1., .81, .45), (.76, .61, 1.), (.41, .9, .72)]
        for car, color in zip(cars, colors):
            ego = car['ego']
            if ego is None:
                continue
            for label in (False, True):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.id = car['vehicle_number'] * 2 + int(label)
                marker.ns = 'shared_course'
                marker.type = Marker.TEXT_VIEW_FACING if label else Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = ego['x_m']
                marker.pose.position.y = ego['y_m'] + (1.5 if label else 0.)
                marker.pose.position.z = 6.7
                marker.pose.orientation.z = math.sin(ego['yaw_rad'] / 2) if not label else 0.
                marker.pose.orientation.w = math.cos(ego['yaw_rad'] / 2) if not label else 1.
                marker.scale.x = 2.2; marker.scale.y = .8; marker.scale.z = 1.1 if label else .3
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.
                marker.text = f"D{car['vehicle_number']} {ego['speed_mps']:.2f} m/s"
                markers.markers.append(marker)
        marker_pub.publish(markers)

    def launch(args: list[str], domain: int, filename: str) -> subprocess.Popen:
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        directory = out if domain == 0 else out / f'd{domain}'
        directory.mkdir(exist_ok=True)
        if domain:
            env.update(ROS_HOME=str(directory / 'ros-home'), ROS_LOG_DIR=str(directory / 'ros'),
                       SIM_MODE='h2h-race', VEHICLE_ID=f'd{domain}')
        log = (out / filename).open('w'); logs.append(log)
        process = subprocess.Popen(args, env=env, cwd=directory, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        event('launch', {'domain': domain, 'args': args})
        return process

    def spin() -> None:
        for _ in range(3):
            for executor in executors:
                executor.spin_once(timeout_sec=0)

    result = {'ok': False, 'reason': 'not_started'}
    try:
        sim_args = ['/aichallenge/simulator/AWSIM/AWSIM.x86_64', '-batchmode', '-nographics',
                      '--venue', 'citycircuit', '--vehicles', '4', '--npcs', '0', '--boosts', '0',
                      '--camera', 'off', '--lidar', 'off', '--start-mode', 'sync',
                      '--start-count-seconds', '3', '--laps', '2', '--timeout', str(cfg['sim_timeout_s']),
                      '--steer-source', 'ackermann', '--sound', 'off', '--collisions',
                      'on' if cfg['vehicle_collisions'] else 'off', '--handicap',
                      'on' if cfg['handicap'] else 'off', '--ranking', 'on',
                      '--overtaking-lane', 'on', '--wall-recovery', 'on',
                      '-logFile', '/eval/awsim-player.log']
        if cfg.get('same_start'):
            sim_args += ['--scenario', '/eval/scenario.yaml']
        sim = launch(sim_args, 0, 'awsim-console.log')
        for domain in range(1, 5):
            controllers.append(launch([
                'ros2', 'launch', 'aichallenge_system_launch', 'aichallenge_system.launch.xml',
                'simulation:=true', 'use_sim_time:=true', 'run_rviz:=false', f'domain_id:={domain}',
                'control_method_override:=mppi', f"reference_execution_speed_cap_mps:={cfg['target_mps']}",
                'rosbag:=false', 'capture:=false'], domain, f'autoware-d{domain}.log'))
        next_sample, next_report, next_empty, finished_at = 0., 0., 0., None
        while not stopped:
            spin()
            now = time.monotonic(); elapsed = now - begin
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A simulator/controller process exited early')
            if elapsed > cfg['wall_timeout_s']:
                raise RuntimeError('Shared-course wall timeout')
            if not state['started'] and elapsed > 90:
                raise RuntimeError('Four-vehicle initialization timeout')
            if elapsed >= next_empty:
                for car, publisher in zip(cars, empty_v2x):
                    msg = V2XVehiclePositionArray()
                    value = car['ego']['stamp_s'] if car['ego'] else 0.
                    msg.header.stamp.sec = int(value)
                    msg.header.stamp.nanosec = int((value % 1) * 1e9)
                    msg.header.frame_id = 'map'
                    publisher.publish(msg)
                next_empty = elapsed + .05
            if elapsed >= next_sample:
                samples.write(json.dumps({'wall_s': elapsed, **state, **cars[0], 'vehicles': cars}) + '\n')
                display_cars()
                next_sample = elapsed + .05
            if elapsed >= next_report:
                print(json.dumps({'wall_s': elapsed, 'admin': state['admin'], 'vehicles': [
                    {k: c[k] for k in ('vehicle_number', 'ego', 'status', 'observed_vehicle_ids')} for c in cars]}), flush=True)
                next_report = elapsed + 10
            if not state['started'] and state['admin'].lower() in {'ready', 'waitstart'}:
                if all_vehicles_ready(cars, now, cfg['reference_first_xy_m']):
                    if cfg['ignore_other_vehicles']:
                        required = {'reference_space_mppi_planner', 'mppi_recovery_controller'}
                        proof = {}
                        for domain in range(1, 5):
                            node = nodes[domain]
                            real = {s.node_name for s in node.get_subscriptions_info_by_topic('/v2x/vehicle_positions')}
                            ghost = {s.node_name for s in node.get_subscriptions_info_by_topic('/cma/ghost/vehicle_positions')}
                            proof[domain] = {'raw_subscribers': sorted(real), 'empty_subscribers': sorted(ghost)}
                            if real & required:
                                raise RuntimeError('A controller still receives real other-vehicle information')
                        if not all(required <= set(p['empty_subscribers']) for p in proof.values()):
                            continue
                        (out / 'ghost_subscription_proof.json').write_text(json.dumps(proof, indent=2))
                    if cfg.get('same_start'):
                        expected = cfg['start_pose_source']['observed_d1_gnss_xy_m']
                        errors = [math.hypot(car['gnss']['x_m']-expected[0], car['gnss']['y_m']-expected[1])
                                  for car in cars]
                        if max(errors) > .05:
                            raise RuntimeError(f'A car is not at the native D1 start: errors_m={errors}')
                        (out / 'same_start_measurement.json').write_text(json.dumps({
                            'expected_d1_map_xy_m': expected, 'maximum_allowed_error_m': .05,
                            'errors_m': errors, 'vehicles': [{k: c[k] for k in ('vehicle_number','ego','gnss')}
                                                           for c in cars]}, indent=2))
                    start_pub.publish(Bool(data=True)); state['started'] = True
                    event('start_after_all_four_ready', cars)
            if state['finish']:
                if finished_at is None:
                    finished_at = now
                if now - finished_at >= 3:
                    result = {'ok': True, 'reason': 'native_finish', 'started': state['started'],
                              'summary': state['summary'], 'vehicle_count': 4}
                    break
            time.sleep(.002)
        if stopped:
            result = {'ok': False, 'reason': 'interrupted', 'started': state['started']}
    except Exception as exc:
        result = {'ok': False, 'reason': str(exc), 'started': state['started']}
        event('error', str(exc))
    finally:
        for process in controllers:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        deadline = time.monotonic() + 6
        while any(p.poll() is None for p in controllers) and time.monotonic() < deadline:
            spin(); time.sleep(.02)
        for process in controllers:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            for car, publisher in zip(cars, brakes):
                msg = AckermannControlCommand()
                value = car['ego']['stamp_s'] if car['ego'] else 0.
                msg.stamp.sec = int(value); msg.stamp.nanosec = int((value % 1) * 1e9)
                msg.longitudinal.stamp = msg.stamp
                msg.longitudinal.speed = 0.; msg.longitudinal.acceleration = -3.
                publisher.publish(msg)
            spin(); time.sleep(.02)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)
        result['simulator_process_stopped'] = all(p.poll() is not None for p in processes)
        (out / 'runtime_result.json').write_text(json.dumps(result, indent=2))
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            rclpy.shutdown(context=context)
        for log in logs + [samples, events, summaries]:
            log.close()
    print(json.dumps(result), flush=True)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
