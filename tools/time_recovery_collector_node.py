"""AWSIM-only measured recovery recorder gate; official PP supplies nominal input.

Reference paths and phase annotations are teacher/debug-only. No learned model
is loaded. The existing time-trial scan/motion monitor is reused unchanged.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import gc
import json
import math
import os
from pathlib import Path
import resource
import sys
import threading
import time

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan, select_aligned_scan, scan_pose_in_rear
from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    TARGET_MPS, bounded_collection_command, check_collection_input_time, phase_at_s, project_course,
    select_collection_input, select_collection_motion, validate_nominal,
    check_collection_decision_age, collection_snapshot_retry_allowed,
    collection_snapshot_retry_wait_ns,
    validate_collection_imu_axes, collection_imu_yaw_rate,
)
from aic_transfuser_lite.data.time_steering_pulse_v1 import (
    SteeringPulseConfig, SteeringPulseState, nominal_recovery_errors, propose_steering_pulse,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--reference', type=Path, required=True)
    ap.add_argument('--run-id', required=True)
    args, ros_args = ap.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '1' or any(Path(p).exists() for p in ('/dev/vcu', '/dev/gnss', '/dev/ttyUSB0')):
        raise ValueError('AWSIM_ONLY_REQUIRED')
    # Small NumPy operations release the GIL repeatedly. The default 5 ms
    # Python thread handoff lets ROS reception delay a ~12 ms CPU calculation
    # beyond the sensor deadline. Bound this collector process's handoff only;
    # keep all sensor/computation deadlines and the numerical monitor intact.
    sys.setswitchinterval(.0005)
    (args.output/'collector_python_runtime.json').write_text(json.dumps(dict(
        python_thread_switch_interval_s=sys.getswitchinterval(),
        numerical_threads={key:os.environ.get(key) for key in
            ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}), indent=2))
    reference = json.loads(args.reference.read_text())
    baseline = np.asarray(reference['baseline_xy_m'])
    pulse_spec = reference.get('steering_pulse')
    pulse_config = None; pulse_guide = None; pulse_state = SteeringPulseState()
    if pulse_spec is not None:
        if (pulse_spec['schema'] != 'measured_steering_pulse_v1' or reference['intervals']
                or reference['signed_offset_m'] != 0.
                or reference['reference_xy_m'] != reference['baseline_xy_m']):
            raise ValueError('PULSE_REQUIRES_UNPERTURBED_REFERENCE')
        pulse_config = SteeringPulseConfig(**pulse_spec['config'])
        pulse_guide = np.asarray(pulse_spec['nominal_guide'], dtype=float)
        nominal_recovery_errors(pulse_guide, s_m=pulse_config.start_s_m, offset_m=0., yaw_rad=0.)
        if (pulse_guide[0, 0] > pulse_config.start_s_m-5.
                or pulse_guide[-1, 0] < pulse_config.start_s_m+pulse_config.start_window_m+3.+1.7*pulse_config.recovery_s):
            raise ValueError('PULSE_GUIDE_RECOVERY_COVERAGE')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan, Imu
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String, Float32MultiArray
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_planning_msgs.msg import Trajectory

    rclpy.init(args=ros_args)
    node = Node('time_recovery_collector', enable_rosout=False, start_parameter_services=False)
    receiver = Node('time_recovery_inputs', enable_rosout=False, start_parameter_services=False)
    sensor_lock = threading.Lock()
    cache = {}; poses = deque(maxlen=256); scans = deque(maxlen=4); histories = {}
    sensor_clock = [None, 0]; sensor_fault = [None]; sensor_generation = [0]
    imu_transforms = {}
    previous = [0., time.monotonic()]
    state = dict(run_id=args.run_id, fault=None, armed_ns=None, armed_wall=None,
                 stop_reason=None, stop_since_ns=None, stop_confirmed=False,
                 ready_ticks=0, commands=0, speed_mps=None, max_speed_mps=0.,
                 phase='invalid', nominal_count=0, trajectory_count=0)
    log = (args.output/'control.jsonl').open('x', buffering=1)
    final_topic = '/control/command/control_cmd'; publisher = None
    phase_pub = node.create_publisher(String, '/recovery_teacher/phase', 10)
    gc_events = deque(maxlen=32); gc_started = {}
    def observe_gc(phase, info):
        generation = info['generation']
        if phase == 'start':
            gc_started[generation] = time.monotonic_ns()
        elif generation in gc_started:
            ended = time.monotonic_ns(); began = gc_started.pop(generation)
            gc_events.append((began, ended, generation))
    gc.callbacks.append(observe_gc)
    gc_log = (args.output/'gc.jsonl').open('x', buffering=1)
    last_gc_wall = time.monotonic()

    def stamp(t) -> int:
        return int(t.sec)*10**9+int(t.nanosec)

    def names(topic: str) -> list[str]:
        return [e.node_namespace.rstrip('/')+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic)]

    def on_clock(m) -> None:
        value = stamp(m.clock)
        with sensor_lock:
            if sensor_clock[0] is not None and value < sensor_clock[0]:
                sensor_fault[0] = 'CLOCK_RESET'; sensor_generation[0] += 1
                poses.clear(); scans.clear(); cache.clear(); histories.clear()
            sensor_clock[:] = [value, time.monotonic_ns()]

    # /clock is a latest-time sample, like the official PP ROS clock QoS.
    # A depth-10 callback queue can deliver old time after newer odometry and
    # falsely make a valid pose future-dated. Keep all original sensor stamps
    # and the existing [-20 ms, 150 ms] admission interval unchanged.
    receiver.create_subscription(Clock, '/clock', on_clock,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    topics = {
        'pose': ('/localization/kinematic_state', Odometry, '/localization/ekf_localizer'),
        'velocity': ('/vehicle/status/velocity_status', VelocityReport, '/awsim_d1'),
        'steering': ('/vehicle/status/steering_status', SteeringReport, '/awsim_d1'),
        'imu': ('/sensing/imu/imu_raw', Imu, '/awsim_d1'),
        'camera': ('/sensing/camera/image_raw', Image, '/awsim_d1'),
        'scan': ('/sensing/lidar/scan', LaserScan, '/awsim_d1'),
        'nominal': ('/recovery_teacher/nominal_control_cmd', AckermannControlCommand, '/recovery_teacher_pure_pursuit'),
        'trajectory': ('/recovery_teacher/trajectory', Trajectory, '/recovery_teacher_trajectory'),
    }

    def receive(role, m) -> None:
        receipt = time.monotonic_ns()
        with sensor_lock:
            cache[role] = (m, receipt)
            histories.setdefault(role, deque(maxlen=4 if role in ('camera','scan','trajectory') else 32)).append(cache[role])
            if role == 'scan':
                scans.append(cache[role])
            if role == 'nominal':
                sensor_counts['nominal'] += 1
            if role == 'trajectory':
                sensor_counts['trajectory'] += 1
            if role == 'pose':
                p = m.pose.pose; q = p.orientation
                if (m.header.frame_id != 'map' or m.child_frame_id != 'base_link'
                        or not np.isfinite([p.position.x, p.position.y, q.x, q.y, q.z, q.w]).all()
                        or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                    sensor_fault[0] = 'POSE_CONTRACT'; return
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                poses.append(TimedBodyPose(stamp(m.header.stamp), 'sim', '0', 'map', 'base_link',
                                          p.position.x, p.position.y, yaw))

    sensor_counts = {'nominal': 0, 'trajectory': 0}
    def receive_static(m) -> None:
        with sensor_lock:
            for transform in m.transforms:
                if transform.child_frame_id in ('imu_link', 'sensor_kit_base_link'):
                    q = transform.transform.rotation
                    imu_transforms[transform.child_frame_id] = (transform.header.frame_id, (q.x,q.y,q.z,q.w))
            if len(imu_transforms) == 2:
                try:
                    validate_collection_imu_axes(imu_transforms)
                except ValueError as exc:
                    sensor_fault[0] = str(exc)
    receiver.create_subscription(TFMessage, '/tf_static', receive_static,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for role, (topic, kind, _) in topics.items():
        # Latest motion reports must not replay a DDS backlog against /clock.
        # Pose retains the sensor-data queue: the scan's original capture time
        # needs both measured interpolation endpoints, not just the latest pose.
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
               if role in ('velocity', 'steering', 'imu', 'nominal') else qos_profile_sensor_data)
        receiver.create_subscription(kind, topic, lambda m, role=role: receive(role, m), qos)

    def snapshot():
        # ROS messages are immutable after receipt; copy only bounded references.
        # The receiver never waits for scan geometry, ROS graph queries or disk IO.
        with sensor_lock:
            return (*sensor_clock, sensor_fault[0], sensor_generation[0], dict(cache),
                    {r:tuple(h) for r,h in histories.items()}, tuple(poses), tuple(scans), dict(sensor_counts), dict(imu_transforms))

    def tick(attempt: int = 0, started_ns: int | None = None, retry_reasons: tuple[str, ...] = ()) -> None:
        nonlocal publisher, last_gc_wall, pulse_state
        if started_ns is None:
            started_ns = time.monotonic_ns()
        stage_started = time.monotonic_ns(); thread_started = time.thread_time_ns(); stages = {}
        def stage(name):
            nonlocal stage_started
            ended = time.monotonic_ns(); stages[name] = (ended-stage_started)/1e6; stage_started = ended
        clock_ns, clock_receipt, fault, generation, cache, histories, poses, scans, counts, imu_axes = snapshot()
        if fault:
            state['fault'] = fault
        state['nominal_count'] = counts['nominal']; state['trajectory_count'] = counts['trajectory']
        now = time.monotonic_ns(); wall = time.monotonic(); dt = min(.1, max(0., wall-previous[1]))
        speed = None; fresh_velocity = False; details = {}; reason = 'STARTUP'; projection = None; current = None
        guard_yaw_rate = None
        inputs = dict(cache); motion_selected = None; input_times = {}
        pulse_decision = None; pulse_errors = None; nominal_bounded_angle = None
        if clock_ns is not None:
            for role, history in histories.items():
                input_times[role] = [(stamp(m.stamp if role in ('nominal','steering') else m.header.stamp), receipt)
                                     for m,receipt in history]
                selected = select_collection_input(role, input_times[role],
                    now_sim_ns=clock_ns, now_wall_ns=now)
                if selected is not None:
                    inputs[role] = history[selected]
            motion_selected = select_collection_motion(input_times, now_sim_ns=clock_ns, now_wall_ns=now, include_imu=True)
            if motion_selected is not None:
                for role, selected in motion_selected.items():
                    inputs[role] = histories[role][selected]
        angle, accel, target = previous[0], -1., 0.
        stage('snapshot_selection')
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
            stage('authority_clock')
            for role in ('pose', 'velocity', 'steering', 'imu', 'scan', 'camera', 'nominal', 'trajectory'):
                m, receipt = inputs[role]
                if names(topics[role][0]) != [topics[role][2]]:
                    raise ValueError('SOURCE_'+role)
                t = stamp(m.stamp if role in ('steering', 'nominal') else m.header.stamp)
                check_collection_input_time(role, capture_ns=t, receipt_ns=receipt, now_sim_ns=clock_ns, now_wall_ns=now)
            stage('source_input_validation')
            if not fresh_velocity:
                raise ValueError('VELOCITY_INVALID')
            pose_stamp = stamp(inputs['pose'][0].header.stamp)
            current = next(p for p in reversed(poses) if p.stamp_ns == pose_stamp)
            velocity = inputs['velocity'][0]; steering = inputs['steering'][0]; nominal, received = inputs['nominal']
            imu = inputs['imu'][0]
            if (motion_selected is None or velocity.header.frame_id != 'base_link'
                    or abs(current.stamp_ns-stamp(velocity.header.stamp)) > 50_000_000
                    or abs(stamp(steering.stamp)-stamp(velocity.header.stamp)) > 50_000_000
                    or abs(stamp(imu.header.stamp)-stamp(velocity.header.stamp)) > 50_000_000):
                raise ValueError('STATE_FRAME_OR_CAPTURE_SKEW')
            validate_collection_imu_axes(imu_axes)
            if not (args.output/'imu_axes.json').exists():
                (args.output/'imu_axes.json').write_text(json.dumps(dict(transforms=imu_axes,
                    tf_publishers=names('/tf_static'), heading_rate_source=topics['imu'][0],
                    angular_unit='rad/s', source='MEASURED_IMU_Z_WITH_VALIDATED_BODY_Z_AXES'),indent=2))
            w = imu.angular_velocity
            guard_yaw_rate = collection_imu_yaw_rate([w.x,w.y,w.z], imu.header.frame_id)
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
            requested_angle = float(nominal.lateral.steering_tire_angle)
            if pulse_config is not None and state['armed_ns'] is not None:
                if pulse_guide[0, 0] <= projection['s_m'] <= pulse_guide[-1, 0]:
                    pulse_errors = nominal_recovery_errors(pulse_guide, s_m=projection['s_m'],
                        offset_m=projection['offset_m'], yaw_rad=current.yaw_rad)
                elif pulse_state.stage in ('active', 'releasing', 'recovery'):
                    raise ValueError('PULSE_LEFT_GUIDE_SUPPORT')
                pulse_decision = propose_steering_pulse(pulse_config, pulse_state, sim_ns=clock_ns,
                    wall_ns=now, s_m=projection['s_m'], speed_mps=speed,
                    lateral_m=pulse_errors[0] if pulse_errors else 0.,
                    heading_rad=pulse_errors[1] if pulse_errors else 0.)
                nominal_bounded_angle, _ = bounded_collection_command(requested_angle,
                    float(nominal.longitudinal.acceleration), previous[0], dt)
                requested_angle += pulse_decision.perturbation_rad
            angle, bounded_accel = bounded_collection_command(requested_angle,
                float(nominal.longitudinal.acceleration), previous[0], dt)
            stage('motion_reference')
            selected, captured = select_aligned_scan([(stamp(m.header.stamp), t) for m, t in scans],
                poses, current, now_sim_ns=clock_ns, now_receipt_ns=now)
            laser = scans[selected][0]
            inputs['scan'] = scans[selected]  # Recheck the scan actually used by the guard at publish time.
            if laser.header.frame_id != 'lidar':
                raise ValueError('SCAN_FRAME')
            alignment = scan_pose_in_rear(captured, current, .0010000169277191162)
            stage('scan_alignment')
            details = check_turning_scan(laser.ranges, laser.angle_min, laser.angle_increment, laser.range_min, laser.range_max,
                speed_mps=speed, measured_steer_rad=float(steering.steering_tire_angle),
                issued_steer_rad=.6*angle, previous_steer_rad=.6*previous[0], scan_in_current_rear=alignment,
                envelope_policy='curvature_support_v2', vehicle_model_policy='awsim_understeer_v1',
                heading_rate_radps=guard_yaw_rate, reported_lateral_mps=float(velocity.lateral_velocity))
            stage('scan_guard')
            state['ready_ticks'] += 1
            if state['armed_ns'] is None:
                reason = 'READY'; angle = previous[0]
            else:
                reason = 'RECOVERY_TEACHER_TRACKING'
                accel = bounded_accel; target = TARGET_MPS
        except (ValueError, KeyError, TypeError) as exc:
            retry_elapsed = time.monotonic_ns()-started_ns
            if not state['fault'] and collection_snapshot_retry_allowed(str(exc), attempt=attempt,
                    elapsed_ns=retry_elapsed):
                time.sleep(collection_snapshot_retry_wait_ns(retry_elapsed)/1e9)
                return tick(1, started_ns, (*retry_reasons, str(exc)))
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
                 'braking' if state['stop_reason'] else pulse_decision.phase if pulse_decision is not None
                 else phase_at_s(projection['s_m'], reference['intervals']) if projection else 'invalid')
        state['phase'] = phase
        retry_after_snapshot = None
        publication = None
        if publisher is not None and names(final_topic) == ['/time_recovery_collector'] and clock_ns is not None:
            # Recheck the *same* selected samples before publishing. A clock
            # reset, bad pose or processing delay must not publish an old go
            # command merely because its snapshot passed earlier in the tick.
            with sensor_lock:
                publish_sim, publish_receipt = sensor_clock
                publish_wall = time.monotonic_ns()
                try:
                    if sensor_fault[0] or generation != sensor_generation[0]:
                        raise ValueError(sensor_fault[0] or 'CLOCK_RESET')
                    if publish_sim is None or publish_wall-publish_receipt > 500_000_000:
                        raise ValueError('CLOCK_STALE')
                    if target > 0:
                        check_collection_decision_age(started_ns=started_ns, now_ns=publish_wall)
                        for role, (m, receipt) in inputs.items():
                            check_collection_input_time(role,
                                capture_ns=stamp(m.stamp if role in ('nominal','steering') else m.header.stamp),
                                receipt_ns=receipt, now_sim_ns=publish_sim, now_wall_ns=publish_wall)
                except ValueError as exc:
                    if not state['fault'] and not sensor_fault[0] and collection_snapshot_retry_allowed(
                            str(exc), attempt=attempt, elapsed_ns=publish_wall-started_ns):
                        retry_after_snapshot = str(exc)
                    else:
                        reason = str(exc); angle = previous[0]; accel = -1.; target = 0.
                        phase = 'invalid'; state['phase'] = phase; state['ready_ticks'] = 0
                        if state['fault'] is None and (state['armed_ns'] is not None or sensor_fault[0]):
                            state['fault'] = reason
                if retry_after_snapshot is None:
                    command = AckermannControlCommand()
                    command_stamp = publish_sim if publish_sim is not None else clock_ns
                    command.stamp.sec = command_stamp//10**9; command.stamp.nanosec = command_stamp%10**9
                    command.lateral.stamp = command.stamp; command.longitudinal.stamp = command.stamp
                    command.lateral.steering_tire_angle = angle
                    command.longitudinal.speed = target; command.longitudinal.acceleration = accel
                    emitted_wall = time.monotonic_ns()
                    publisher.publish(command); state['commands'] += 1
                    publication = dict(sim_ns=command_stamp, monotonic_ns=emitted_wall,
                                       sequence=state['commands'])
                    if pulse_decision is not None and target > 0 and not state['fault']:
                        # No state change survives a rejected/unpublished proposal.
                        pulse_state = pulse_decision.state
        elif actual and actual != ['/time_recovery_collector']:
            state['fault'] = 'COMPETING_CONTROLLER'
        if retry_after_snapshot is not None:
            elapsed = time.monotonic_ns()-started_ns
            if elapsed <= 80_000_000:
                time.sleep(collection_snapshot_retry_wait_ns(elapsed)/1e9)
            return tick(1, started_ns, (*retry_reasons, retry_after_snapshot))
        previous[:] = [angle, wall]
        row = dict(event='CONTROL_AND_PHASE', monotonic_ns=now, sim_ns=clock_ns, reason=reason,
                   phase=phase, projection=projection, speed_mps=state['speed_mps'],
                   issued_angle_rad=angle, target_speed_mps=target, acceleration_mps2=accel,
                   guard=details, current_pose=current.__dict__ if current else None,
                   nominal_angle_rad=float(inputs['nominal'][0].lateral.steering_tire_angle)
                       if 'nominal' in inputs and math.isfinite(inputs['nominal'][0].lateral.steering_tire_angle) else None,
                   nominal_stamp_ns=stamp(inputs['nominal'][0].stamp) if 'nominal' in inputs else None)
        row['publication'] = publication  # ROS publication boundary, not simulator application time.
        if pulse_config is not None:
            applied = publication is not None and target > 0 and not state['fault'] and pulse_decision is not None
            row['annotation_schema'] = 'measured_steering_pulse_v1'
            row['pulse'] = dict(state=asdict(pulse_state), applied=bool(applied),
                requested_rad=pulse_decision.perturbation_rad if pulse_decision else 0.,
                effective_rad=angle-nominal_bounded_angle if applied and nominal_bounded_angle is not None else 0.,
                lateral_error_m=pulse_errors[0] if pulse_errors else None,
                heading_error_rad=pulse_errors[1] if pulse_errors else None)
            state['pulse'] = row['pulse']
        row['nominal_acceleration_mps2'] = (float(inputs['nominal'][0].longitudinal.acceleration)
            if 'nominal' in inputs and math.isfinite(inputs['nominal'][0].longitudinal.acceleration) else None)
        row['odometry_speed_mps'] = (float(inputs['pose'][0].twist.twist.linear.x)
            if 'pose' in inputs and math.isfinite(inputs['pose'][0].twist.twist.linear.x) else None)
        row['measured_steering_rad'] = (float(inputs['steering'][0].steering_tire_angle)
            if 'steering' in inputs and math.isfinite(inputs['steering'][0].steering_tire_angle) else None)
        row['input_timing_ns'] = {role: dict(capture_ns=stamp(m.stamp if role in ('nominal','steering') else m.header.stamp),
            receipt_monotonic_ns=receipt, receipt_age_ns=now-receipt) for role,(m,receipt) in inputs.items()}
        row['latest_received_capture_ns'] = {role:stamp(m.stamp if role in ('nominal','steering') else m.header.stamp)
                                            for role,(m,receipt) in cache.items()}
        row['guard_heading_rate_radps'] = guard_yaw_rate
        row['raw_velocity_heading_rate_radps'] = (float(inputs['velocity'][0].heading_rate)
            if 'velocity' in inputs and math.isfinite(inputs['velocity'][0].heading_rate) else None)
        row['guard_heading_rate_source'] = topics['imu'][0]
        if reason == 'STATE_FRAME_OR_CAPTURE_SKEW':
            row['motion_history_ns'] = {role:input_times.get(role, []) for role in ('pose','velocity','steering','imu')}
        if reason == 'FRESH_ALIGNED_SCAN_MISSING':
            row['pose_history_ns'] = [p.stamp_ns for p in poses]
            row['scan_history_ns'] = [(stamp(m.header.stamp), t) for m,t in scans]
        row['processing_ms'] = (time.monotonic_ns()-now)/1e6
        row['snapshot_retry_reasons'] = retry_reasons
        row['decision_wall_ms'] = (time.monotonic_ns()-started_ns)/1e6
        row['thread_cpu_ms'] = (time.thread_time_ns()-thread_started)/1e6
        row['stage_wall_ms'] = stages
        row['gc_pauses'] = [dict(generation=g, duration_ms=(end-begin)/1e6)
                            for begin,end,g in tuple(gc_events) if end >= started_ns]
        serialized = json.dumps(row, allow_nan=False); log.write(serialized+'\n')
        phase_pub.publish(String(data=serialized))
        state['rviz_subscribers'] = [e.node_name for e in node.get_subscriptions_info_by_topic('/recovery_teacher/reference_path')]
        pending = args.output/'control_heartbeat.pending'
        pending.write_text(json.dumps(dict(state, monotonic_ns=now, sim_ns=clock_ns, reason=reason, projection=projection), allow_nan=False))
        pending.replace(args.output/'control_heartbeat.json')
        # Keep cyclic collection out of scan selection/calculation/publication.
        # Refcounts still release ordinary message/array objects immediately.
        if time.monotonic()-last_gc_wall >= 5.:
            began = time.monotonic_ns(); collected = gc.collect()
            last_gc_wall = time.monotonic()
            gc_log.write(json.dumps(dict(monotonic_ns=began, collected=collected,
                duration_ms=(time.monotonic_ns()-began)/1e6,
                peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))+'\n')

    node.create_timer(.05, tick)
    automatic_gc_enabled = gc.isenabled()
    gc.collect(); gc.freeze(); gc.disable()
    (args.output/'gc_policy.json').write_text(json.dumps(dict(
        policy='FROZEN_STARTUP_GRAPH_AND_EXPLICIT_COLLECTION_AFTER_PUBLICATION',
        collect_interval_s=5., frozen_objects=gc.get_freeze_count(),
        automatic_enabled=gc.isenabled()), indent=2))
    receiver_executor = SingleThreadedExecutor()
    receiver_executor.add_node(receiver)
    def spin_receiver():
        try:
            receiver_executor.spin()
        except Exception:
            # SIGINT may invalidate the shared ROS context before this thread
            # wakes. A live-context receiver failure remains explicit and stops.
            if rclpy.ok():
                with sensor_lock:
                    sensor_fault[0] = 'RECEIVER_EXECUTOR_FAILED'
                raise
    receiver_thread = threading.Thread(target=spin_receiver, name='recovery-inputs', daemon=True)
    receiver_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        gc.callbacks.remove(observe_gc)
        receiver_executor.shutdown(timeout_sec=2.)
        receiver_thread.join(timeout=2.)
        receiver.destroy_node()
        log.close(); gc_log.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        gc.unfreeze()
        if automatic_gc_enabled:
            gc.enable()


if __name__ == '__main__':
    main()
