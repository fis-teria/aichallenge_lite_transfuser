"""Bounded ROS/AWSIM episode inside a network-isolated disposable container."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_msgs.msg import Bool, String, Float32MultiArray, Int32
from v2x_msgs.msg import V2XVehiclePositionArray

OUT = Path('/eval')


def stamp(message: object) -> float:
    value = getattr(message, 'stamp', None)
    if value is None:
        value = message.header.stamp
    return float(value.sec) + float(value.nanosec) * 1e-9


def main() -> int:
    cfg = json.loads((OUT / 'config.json').read_text())
    calibration = cfg.get('mode') == 'calibration'
    contexts, nodes, executors, processes, logs = [], [], [], [], []
    best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state: dict = {'admin': '', 'ego': None, 'gnss': None, 'status': None, 'reference': None,
                   'command': None, 'condition': None, 'summary': None, 'started': False,
                   'finish': False, 'debug': None}
    observed: dict[str, list[list[float]]] = {}
    nodes_by_domain = {}
    begin = time.monotonic()
    stop = False

    def signal_stop(_number, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, signal_stop)
    signal.signal(signal.SIGINT, signal_stop)
    events = (OUT / 'events.jsonl').open('w', buffering=1)
    samples = (OUT / 'samples.jsonl').open('w', buffering=1)
    summaries = (OUT / 'admin.jsonl').open('w', buffering=1)

    def event(kind: str, data: object) -> None:
        events.write(json.dumps({'wall_s': time.monotonic() - begin, 'kind': kind, 'data': data}) + '\n')

    for domain in [0, 1, 2]:
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        node = rclpy.create_node(f'cma_episode_observer_{domain}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        contexts.append(context); nodes.append(node); executors.append(executor)
        nodes_by_domain[domain] = node

    def admin(msg):
        if msg.data != state['admin']:
            event('admin_state', msg.data)
        state['admin'] = msg.data
        if msg.data.lower() in {'finishall', 'finishedall', 'terminate', 'terminated'}:
            state['finish'] = True

    def summary(msg):
        value = json.loads(msg.data)
        state['summary'] = value
        summaries.write(json.dumps({'wall_s': time.monotonic() - begin, 'data': value}) + '\n')

    def v2x(msg):
        for vehicle in msg.vehicles:
            points = observed.setdefault(vehicle.vehicle_id, [])
            points.append([vehicle.position.x, vehicle.position.y])
            if len(points) > 100:
                del points[:-100]

    def odom(msg):
        q = msg.pose.pose.orientation
        state['ego'] = {'stamp_s': stamp(msg), 'x_m': msg.pose.pose.position.x,
                        'y_m': msg.pose.pose.position.y,
                        'yaw_rad': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                        'speed_mps': msg.twist.twist.linear.x, 'arrival_s': time.monotonic()}

    def gnss(msg):
        state['gnss'] = {'stamp_s': stamp(msg), 'x_m': msg.pose.pose.position.x,
                         'y_m': msg.pose.pose.position.y, 'arrival_s': time.monotonic()}

    def reference(msg):
        if msg.points:
            state['reference'] = {'points': len(msg.points), 'x_m': msg.points[0].pose.position.x,
                                  'y_m': msg.points[0].pose.position.y, 'stamp_s': stamp(msg)}

    def command(msg):
        state['command'] = {'stamp_s': stamp(msg), 'speed_mps': msg.longitudinal.speed,
                            'acceleration_mps2': msg.longitudinal.acceleration,
                            'steering_rad': msg.lateral.steering_tire_angle}

    def status(msg):
        state['status'] = list(msg.data)

    def condition(msg):
        if state['condition'] != msg.data:
            event('condition', msg.data)
        state['condition'] = msg.data

    def debug(msg):
        state['debug'] = msg.data

    nodes[0].create_subscription(String, '/admin/awsim/state', admin, latched)
    nodes[0].create_subscription(String, '/admin/summary', summary, latched)
    start_pub = nodes[0].create_publisher(Bool, '/admin/awsim/start', latched)
    nodes[1].create_subscription(Odometry, '/localization/kinematic_state', odom, best)
    nodes[1].create_subscription(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', gnss, best)
    nodes[1].create_subscription(Trajectory, '/planning/scenario_planning/trajectory', reference, best)
    nodes[1].create_subscription(AckermannControlCommand, '/control/command/control_cmd', command, best)
    nodes[1].create_subscription(Float32MultiArray, '/awsim/status', status, best)
    nodes[1].create_subscription(Int32, '/aichallenge/pitstop/condition', condition, best)
    nodes[1].create_subscription(String, '/pure_pursuit/debug', debug, best)
    brake_pub = nodes[1].create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)
    for node in nodes[1:]:
        node.create_subscription(V2XVehiclePositionArray, '/v2x/vehicle_positions', v2x, best)

    def launch(args: list[str], log_name: str, domain: int) -> subprocess.Popen:
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        if domain == 1:
            env.update(ROS_LOG_DIR='/eval/ros', ROS_HOME='/eval/ros-home', SIM_MODE='h2h-race')
        log = (OUT / log_name).open('w'); logs.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env=env,
                                   cwd=OUT, start_new_session=True)
        processes.append(process)
        event('launch', {'args': args, 'domain': domain})
        return process

    args = ['/aichallenge/simulator/AWSIM/AWSIM.x86_64', '-batchmode', '-nographics',
            '--venue', 'citycircuit', '--vehicles', '4' if calibration else '1',
            '--npcs', '0', '--boosts', '0', '--camera', 'off', '--lidar', 'off',
            '--start-mode', 'off' if calibration else 'sync', '--start-count-seconds', '3',
            '--laps', '2', '--timeout', str(cfg.get('sim_timeout_s', 260)),
            '--steer-source', 'ackermann', '--sound', 'off', '--collisions', 'on',
            '--handicap', 'on' if cfg.get('handicap') else 'off', '--ranking', 'on',
            '--overtaking-lane', 'on', '--wall-recovery', 'on', '-logFile', '/eval/awsim-player.log']
    if calibration:
        args += ['--scenario', '/eval/scenario.yaml']
    result = {'ok': False, 'reason': 'not_started'}
    try:
        sim = launch(args, 'awsim-console.log', 0)
        controller = None
        if not calibration:
            controller = launch(['ros2', 'launch', 'aichallenge_system_launch', 'aichallenge_system.launch.xml',
                                 'simulation:=true', 'use_sim_time:=true', 'run_rviz:=false', 'domain_id:=1',
                                 'control_method_override:=mppi', 'reference_execution_speed_cap_mps:=' + str(cfg['target_mps']),
                                 'rosbag:=false', 'capture:=false'], 'autoware.log', 1)
        next_report, next_sample, finished_at = 0.0, 0.0, None
        while not stop:
            for executor in executors:
                executor.spin_once(timeout_sec=0.005)
            now = time.monotonic(); elapsed = now-begin
            if sim.poll() is not None:
                raise RuntimeError(f'AWSIM exited early: {sim.returncode}')
            if controller is not None and controller.poll() is not None:
                raise RuntimeError(f'Controller exited early: {controller.returncode}')
            if elapsed > cfg.get('wall_timeout_s', 360):
                raise RuntimeError('Episode wall timeout')
            if not calibration and not state['started'] and elapsed > 70:
                raise RuntimeError('Initialization timeout: controller/reference/GNSS readiness not established')
            if elapsed >= next_report:
                print(json.dumps({'wall_s': round(elapsed, 1), 'admin': state['admin'],
                                  'status': state['status'], 'ego': state['ego'],
                                  'gnss': state['gnss'], 'observed': list(observed)}), flush=True)
                next_report = elapsed+10
                (OUT/'topics.json').write_text(json.dumps({str(d): nodes_by_domain[d].get_topic_names_and_types() for d in nodes_by_domain}))
            if elapsed >= next_sample:
                samples.write(json.dumps({'wall_s': elapsed, **state})+'\n')
                next_sample = elapsed+0.05
            if calibration:
                if elapsed >= 30 and len(observed) == 4 and min(map(len, observed.values())) >= 20:
                    result = {'ok': True, 'reason': 'calibration_observed', 'observations': observed}
                    break
                continue
            if state['admin'].lower() in {'ready','waitstart'} and not state['started']:
                ego, gps, ref = state['ego'], state['gnss'], state['reference']
                ready = bool(ego and gps and ref and now-ego['arrival_s']<0.3 and now-gps['arrival_s']<0.3
                             and abs(ego['speed_mps'])<0.15
                             and math.hypot(ego['x_m']-gps['x_m'],ego['y_m']-gps['y_m'])<0.5)
                if ready:
                    expected = cfg['reference_first_xy_m']
                    if math.hypot(ref['x_m']-expected[0],ref['y_m']-expected[1])>0.005:
                        raise RuntimeError('Loaded reference differs from requested candidate')
                    start_pub.publish(Bool(data=True)); state['started']=True
                    event('start_after_ready', {'reference':ref, 'ego':ego, 'gnss':gps})
            if state['finish']:
                if finished_at is None:
                    finished_at=now
                if now-finished_at>=3:
                    result={'ok':True,'reason':'native_finish','summary':state['summary'],
                            'status':state['status'],'started':state['started']}
                    break
            if state['started'] and state.get('summary') and state['summary'].get('vehicles'):
                penalties=state['summary']['vehicles'][0].get('penalty_by_kind',{})
                if any(penalties.get(k,{}).get('count',0)>0 for k in ['crash','wall','over']):
                    result={'ok':True,'reason':'candidate_penalty','started':True,
                            'summary':state['summary'],'status':state['status']}
                    event('candidate_rejected', penalties)
                    break
    except Exception as exc:
        result={'ok':False,'reason':str(exc)}
        event('error', str(exc))
    finally:
        if not calibration:
            if len(processes)>1 and processes[1].poll() is None:
                os.killpg(processes[1].pid, signal.SIGINT)
                try: processes[1].wait(timeout=6)
                except subprocess.TimeoutExpired: os.killpg(processes[1].pid, signal.SIGTERM)
            braking_until = time.monotonic()+4
            while time.monotonic()<braking_until:
                msg=AckermannControlCommand()
                ego=state['ego']
                sim_stamp=ego['stamp_s'] if ego else 0.0
                msg.stamp.sec=int(sim_stamp); msg.stamp.nanosec=int((sim_stamp%1)*1e9)
                msg.longitudinal.stamp=msg.stamp
                msg.longitudinal.speed=0.0; msg.longitudinal.acceleration=-3.0
                brake_pub.publish(msg)
                for executor in executors: executor.spin_once(timeout_sec=0.01)
            result['last_observed_speed_mps']=state['ego']['speed_mps'] if state['ego'] else None
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)
        result['simulator_process_stopped'] = all(p.poll() is not None for p in processes)
        (OUT/'runtime_result.json').write_text(json.dumps(result,indent=2))
        for node in nodes: node.destroy_node()
        for context in contexts: rclpy.shutdown(context=context)
        for log in logs+[events,samples,summaries]: log.close()
    print(json.dumps(result), flush=True)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
