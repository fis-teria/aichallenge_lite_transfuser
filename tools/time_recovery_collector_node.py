"""AWSIM-only measured recovery recorder gate; official PP supplies nominal input.

Reference paths and phase annotations are teacher/debug-only. No learned model
is loaded. The existing time-trial scan/motion monitor is reused unchanged.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan, select_aligned_scan, scan_pose_in_rear
from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    TARGET_MPS, bounded_collection_command, check_collection_input_time, phase_at_s, project_course,
    select_collection_input, select_collection_motion, validate_nominal,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--reference', type=Path, required=True)
    ap.add_argument('--run-id', required=True)
    args, ros_args = ap.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '1' or any(Path(p).exists() for p in ('/dev/vcu', '/dev/gnss', '/dev/ttyUSB0')):
        raise ValueError('AWSIM_ONLY_REQUIRED')
    reference = json.loads(args.reference.read_text())
    baseline = np.asarray(reference['baseline_xy_m'])
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import String, Float32MultiArray
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_planning_msgs.msg import Trajectory

    rclpy.init(args=ros_args)
    node = Node('time_recovery_collector', enable_rosout=False, start_parameter_services=False)
    cache = {}; poses = deque(maxlen=256); scans = deque(maxlen=4); histories = {}
    clock_ns = None; clock_receipt = 0; previous = [0., time.monotonic()]
    state = dict(run_id=args.run_id, fault=None, armed_ns=None, armed_wall=None,
                 stop_reason=None, stop_since_ns=None, stop_confirmed=False,
                 ready_ticks=0, commands=0, speed_mps=None, max_speed_mps=0.,
                 phase='invalid', nominal_count=0, trajectory_count=0)
    log = (args.output/'control.jsonl').open('x', buffering=1)
    final_topic = '/control/command/control_cmd'; publisher = None
    phase_pub = node.create_publisher(String, '/recovery_teacher/phase', 10)
    paths = {name: node.create_publisher(RosPath, '/recovery_teacher/'+name+'_path', 1)
             for name in ('baseline', 'reference', 'observed')}
    trace = deque(maxlen=10000); last_path_wall = 0.; last_pose_stamp = None

    def stamp(t) -> int:
        return int(t.sec)*10**9+int(t.nanosec)

    def names(topic: str) -> list[str]:
        return [e.node_namespace.rstrip('/')+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic)]

    def on_clock(m) -> None:
        nonlocal clock_ns, clock_receipt
        value = stamp(m.clock)
        if clock_ns is not None and value < clock_ns:
            state['fault'] = 'CLOCK_RESET'; poses.clear(); scans.clear(); cache.clear(); histories.clear()
        clock_ns = value; clock_receipt = time.monotonic_ns()

    # /clock is a latest-time sample, like the official PP ROS clock QoS.
    # A depth-10 callback queue can deliver old time after newer odometry and
    # falsely make a valid pose future-dated. Keep all original sensor stamps
    # and the existing [-20 ms, 150 ms] admission interval unchanged.
    node.create_subscription(Clock, '/clock', on_clock,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    topics = {
        'pose': ('/localization/kinematic_state', Odometry, '/localization/ekf_localizer'),
        'velocity': ('/vehicle/status/velocity_status', VelocityReport, '/awsim_d1'),
        'steering': ('/vehicle/status/steering_status', SteeringReport, '/awsim_d1'),
        'camera': ('/sensing/camera/image_raw', Image, '/awsim_d1'),
        'scan': ('/sensing/lidar/scan', LaserScan, '/awsim_d1'),
        'nominal': ('/recovery_teacher/nominal_control_cmd', AckermannControlCommand, '/recovery_teacher_pure_pursuit'),
        'trajectory': ('/recovery_teacher/trajectory', Trajectory, '/recovery_teacher_trajectory'),
    }

    def receive(role, m) -> None:
        cache[role] = (m, time.monotonic_ns())
        histories.setdefault(role, deque(maxlen=4 if role in ('camera','scan','trajectory') else 32)).append(cache[role])
        if role == 'scan':
            scans.append(cache[role])
        if role == 'nominal':
            state['nominal_count'] += 1
        if role == 'trajectory':
            state['trajectory_count'] += 1
        if role == 'pose':
            p = m.pose.pose; q = p.orientation
            if (m.header.frame_id != 'map' or m.child_frame_id != 'base_link'
                    or not np.isfinite([p.position.x, p.position.y, q.x, q.y, q.z, q.w]).all()
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                state['fault'] = 'POSE_CONTRACT'; return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            poses.append(TimedBodyPose(stamp(m.header.stamp), 'sim', '0', 'map', 'base_link',
                                      p.position.x, p.position.y, yaw))

    for role, (topic, kind, _) in topics.items():
        # A blocked callback must not replay five old motion reports against
        # the latest /clock. Retain history after receipt, not in the DDS queue.
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
               if role in ('pose', 'velocity', 'steering', 'nominal') else qos_profile_sensor_data)
        node.create_subscription(kind, topic, lambda m, role=role: receive(role, m), qos)

    def tick() -> None:
        nonlocal publisher, last_path_wall, last_pose_stamp
        now = time.monotonic_ns(); wall = time.monotonic(); dt = min(.1, max(0., wall-previous[1]))
        speed = None; fresh_velocity = False; details = {}; reason = 'STARTUP'; projection = None; current = None
        inputs = dict(cache); motion_selected = None; input_times = {}
        if clock_ns is not None:
            for role, history in histories.items():
                input_times[role] = [(stamp(m.stamp if role in ('nominal','steering') else m.header.stamp), receipt)
                                     for m,receipt in history]
                selected = select_collection_input(role, input_times[role],
                    now_sim_ns=clock_ns, now_wall_ns=now)
                if selected is not None:
                    inputs[role] = history[selected]
            motion_selected = select_collection_motion(input_times, now_sim_ns=clock_ns, now_wall_ns=now)
            if motion_selected is not None:
                for role, selected in motion_selected.items():
                    inputs[role] = histories[role][selected]
        angle, accel, target = previous[0], -1., 0.
        actual = names(final_topic)
        if publisher is None and not actual:
            publisher = node.create_publisher(AckermannControlCommand, final_topic, 10)
        if clock_ns is not None and 'velocity' in inputs:
            v, receipt = inputs['velocity']
            speed = float(v.longitudinal_velocity)
            fresh_velocity = (math.isfinite(speed) and 0 <= now-receipt <= 300_000_000
                              and -20_000_000 <= clock_ns-stamp(v.header.stamp) <= 150_000_000
                              and names(topics['velocity'][0]) == ['/awsim_d1'])
        state['speed_mps'] = speed if speed is not None and math.isfinite(speed) else None
        try:
            # Authorization and stop intent must be consumed even if a later
            # sensor/nominal check rejects; otherwise a post-start reject could
            # remain unarmed indefinitely and evade measured-stop handling.
            auth_path = args.output/'drive_authorized.json'
            if state['armed_ns'] is None and auth_path.exists() and clock_ns is not None:
                auth = json.loads(auth_path.read_text())
                if (auth.get('run_id') != args.run_id or auth.get('scope') != 'MEASURED_RECOVERY_AWSIM'
                        or wall > auth['expires_monotonic_s']):
                    state['fault'] = 'AUTHORIZATION_INVALID'
                    raise ValueError('AUTHORIZATION_INVALID')
                state['armed_ns'] = clock_ns; state['armed_wall'] = wall
            request = args.output/'stop_request.json'
            if request.exists() and state['stop_reason'] is None:
                value = json.loads(request.read_text())
                if value.get('run_id') != args.run_id:
                    state['fault'] = 'STOP_IDENTITY'
                    raise ValueError('STOP_IDENTITY')
                state['stop_reason'] = str(value['reason'])
            if state['fault']:
                raise ValueError(state['fault'])
            if publisher is None or actual not in ([], ['/time_recovery_collector']):
                raise ValueError('COMMAND_AUTHORITY')
            if not any(e.node_name == 'awsim_d1' for e in node.get_subscriptions_info_by_topic(final_topic)):
                raise ValueError('AWSIM_COMMAND_SUBSCRIBER_MISSING')
            if clock_ns is None or now-clock_receipt > 500_000_000:
                raise ValueError('CLOCK_STALE')
            for role in ('pose', 'velocity', 'steering', 'scan', 'camera', 'nominal', 'trajectory'):
                m, receipt = inputs[role]
                if names(topics[role][0]) != [topics[role][2]]:
                    raise ValueError('SOURCE_'+role)
                t = stamp(m.stamp if role in ('steering', 'nominal') else m.header.stamp)
                check_collection_input_time(role, capture_ns=t, receipt_ns=receipt, now_sim_ns=clock_ns, now_wall_ns=now)
            if not fresh_velocity:
                raise ValueError('VELOCITY_INVALID')
            pose_stamp = stamp(inputs['pose'][0].header.stamp)
            current = next(p for p in reversed(poses) if p.stamp_ns == pose_stamp)
            velocity = inputs['velocity'][0]; steering = inputs['steering'][0]; nominal, received = inputs['nominal']
            if (motion_selected is None or velocity.header.frame_id != 'base_link'
                    or abs(current.stamp_ns-stamp(velocity.header.stamp)) > 50_000_000
                    or abs(stamp(steering.stamp)-stamp(velocity.header.stamp)) > 50_000_000):
                raise ValueError('STATE_FRAME_OR_CAPTURE_SKEW')
            validate_nominal(stamp_ns=stamp(nominal.stamp), now_ns=clock_ns, received_ns=received,
                now_wall_ns=now, target_mps=float(nominal.longitudinal.speed),
                acceleration_mps2=float(nominal.longitudinal.acceleration),
                steering_input_rad=float(nominal.lateral.steering_tire_angle), measured_speed_mps=speed)
            trajectory = inputs['trajectory'][0]
            if (trajectory.header.frame_id != 'map' or len(trajectory.points) < 20
                    or any(not math.isclose(p.longitudinal_velocity_mps, TARGET_MPS, abs_tol=1e-5) for p in trajectory.points)):
                raise ValueError('REFERENCE_SPEED_OR_FRAME')
            projection = project_course(baseline, [current.x_m, current.y_m], current.yaw_rad)
            state['max_speed_mps'] = max(state['max_speed_mps'], abs(speed))
            if state['armed_ns'] is not None and (clock_ns-state['armed_ns'] >= 1800_000_000_000 or wall-state['armed_wall'] >= 1860):
                raise ValueError('COLLECTION_DURATION_LIMIT')
            if state['stop_reason']:
                raise ValueError('REQUESTED_BRAKE')
            angle, bounded_accel = bounded_collection_command(float(nominal.lateral.steering_tire_angle),
                float(nominal.longitudinal.acceleration), previous[0], dt)
            selected, captured = select_aligned_scan([(stamp(m.header.stamp), t) for m, t in scans],
                poses, current, now_sim_ns=clock_ns, now_receipt_ns=now)
            laser = scans[selected][0]
            if laser.header.frame_id != 'lidar':
                raise ValueError('SCAN_FRAME')
            alignment = scan_pose_in_rear(captured, current, .0010000169277191162)
            details = check_turning_scan(laser.ranges, laser.angle_min, laser.angle_increment, laser.range_min, laser.range_max,
                speed_mps=speed, measured_steer_rad=float(steering.steering_tire_angle),
                issued_steer_rad=.6*angle, previous_steer_rad=.6*previous[0], scan_in_current_rear=alignment,
                envelope_policy='curvature_support_v2', vehicle_model_policy='awsim_understeer_v1',
                heading_rate_radps=float(velocity.heading_rate), reported_lateral_mps=float(velocity.lateral_velocity))
            state['ready_ticks'] += 1
            if state['armed_ns'] is None:
                reason = 'READY'; angle = previous[0]
            else:
                reason = 'RECOVERY_TEACHER_TRACKING'
                accel = bounded_accel; target = TARGET_MPS
        except (ValueError, KeyError, TypeError) as exc:
            reason = str(exc); angle = previous[0]; accel = -1.; target = 0.; state['ready_ticks'] = 0
            if state['armed_ns'] is not None and reason != 'REQUESTED_BRAKE':
                state['fault'] = reason
        braking = state['fault'] is not None or state['stop_reason'] is not None
        if braking and fresh_velocity and abs(speed) < .03 and now-clock_receipt <= 500_000_000:
            if state['stop_since_ns'] is None:
                state['stop_since_ns'] = clock_ns
            state['stop_confirmed'] = clock_ns-state['stop_since_ns'] >= 3_000_000_000
        else:
            state['stop_since_ns'] = None; state['stop_confirmed'] = False
        phase = ('invalid' if state['fault'] or state['armed_ns'] is None else
                 'braking' if state['stop_reason'] else phase_at_s(projection['s_m'], reference['intervals']) if projection else 'invalid')
        state['phase'] = phase
        previous[:] = [angle, wall]
        if publisher is not None and names(final_topic) == ['/time_recovery_collector'] and clock_ns is not None:
            command = AckermannControlCommand()
            command.stamp.sec = clock_ns//10**9; command.stamp.nanosec = clock_ns%10**9
            command.lateral.stamp = command.stamp; command.longitudinal.stamp = command.stamp
            command.lateral.steering_tire_angle = angle
            command.longitudinal.speed = target; command.longitudinal.acceleration = accel
            publisher.publish(command); state['commands'] += 1
        elif actual and actual != ['/time_recovery_collector']:
            state['fault'] = 'COMPETING_CONTROLLER'
        row = dict(event='CONTROL_AND_PHASE', monotonic_ns=now, sim_ns=clock_ns, reason=reason,
                   phase=phase, projection=projection, speed_mps=state['speed_mps'],
                   issued_angle_rad=angle, target_speed_mps=target, acceleration_mps2=accel,
                   guard=details, current_pose=current.__dict__ if current else None,
                   nominal_angle_rad=float(inputs['nominal'][0].lateral.steering_tire_angle)
                       if 'nominal' in inputs and math.isfinite(inputs['nominal'][0].lateral.steering_tire_angle) else None,
                   nominal_stamp_ns=stamp(inputs['nominal'][0].stamp) if 'nominal' in inputs else None)
        row['input_timing_ns'] = {role: dict(capture_ns=stamp(m.stamp if role in ('nominal','steering') else m.header.stamp),
            receipt_monotonic_ns=receipt, receipt_age_ns=now-receipt) for role,(m,receipt) in inputs.items()}
        row['latest_received_capture_ns'] = {role:stamp(m.stamp if role in ('nominal','steering') else m.header.stamp)
                                            for role,(m,receipt) in cache.items()}
        if reason == 'STATE_FRAME_OR_CAPTURE_SKEW':
            row['motion_history_ns'] = {role:input_times.get(role, []) for role in ('pose','velocity','steering')}
        serialized = json.dumps(row, allow_nan=False); log.write(serialized+'\n')
        phase_pub.publish(String(data=serialized))
        if poses and poses[-1].stamp_ns != last_pose_stamp:
            last_pose_stamp = poses[-1].stamp_ns; trace.append([poses[-1].x_m, poses[-1].y_m])
        if clock_ns is not None and wall-last_path_wall >= .5:
            for name, points in [('baseline', baseline), ('reference', reference['reference_xy_m']), ('observed', trace)]:
                path = RosPath(); path.header.frame_id = 'map'
                path.header.stamp.sec = clock_ns//10**9; path.header.stamp.nanosec = clock_ns%10**9
                for x, y in points:
                    p = PoseStamped(); p.header = path.header; p.pose.position.x = float(x); p.pose.position.y = float(y)
                    p.pose.orientation.w = 1.; path.poses.append(p)
                paths[name].publish(path)
            last_path_wall = wall
        state['rviz_subscribers'] = [e.node_name for e in node.get_subscriptions_info_by_topic('/recovery_teacher/reference_path')]
        pending = args.output/'control_heartbeat.pending'
        pending.write_text(json.dumps(dict(state, monotonic_ns=now, sim_ns=clock_ns, reason=reason, projection=projection), allow_nan=False))
        pending.replace(args.output/'control_heartbeat.json')

    node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        log.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
