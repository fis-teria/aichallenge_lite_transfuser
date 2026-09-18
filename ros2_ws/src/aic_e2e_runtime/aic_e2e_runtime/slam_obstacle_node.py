"""AWSIM-only scan/wheel bridge and local SLAM obstacle diagnostics. No commands."""
from __future__ import annotations

import argparse
from collections import deque
import copy
import json
import math
from pathlib import Path
import time

from .canonical_source import prefer_canonical_source
prefer_canonical_source()
import numpy as np
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.slam_mppi import AvoidancePlanner, validate_slam_mppi_policy
from aic_transfuser_lite.control.time_geometry_v2 import validate_time_geometry
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose
from aic_transfuser_lite.runtime.lidar_map_localization import transform
from aic_transfuser_lite.runtime.slam_obstacles import (
    SlamObstacleDetector, cartographer_stamp_ns, scan_geometry, slam_scan_pose,
)


def main(argv: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import remove_ros_args
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--mppi', action='store_true', help='Plan low-speed local avoidance for TimePath')
    args = ap.parse_args(remove_ros_args(argv)[1:])
    args.output.mkdir(exist_ok=True, parents=True)
    journal = (args.output/'slam_obstacles.jsonl').open('x', buffering=1)
    rclpy.init(args=argv); node = Node('time_slam_obstacles')
    detector = SlamObstacleDetector()
    avoidance = AvoidancePlanner() if args.mppi else None
    mppi_config = json.loads((args.output/'trial_config.json').read_text()) if args.mppi else None
    if args.mppi and validate_slam_mppi_policy(mppi_config) == 'off':
        raise ValueError('MPPI_SIDECAR_CONFIG_REQUIRED')
    clock = None; clock_wall = 0; wheel_ns = None; wheel_wall = 0
    fatal = None; last_feed = -1; last_process_wall = 0; last_process_ns = None
    poses: deque[TimedBodyPose] = deque(maxlen=1024)
    pending: deque = deque(maxlen=24)
    waiting: deque = deque(maxlen=24)
    plans: deque = deque(maxlen=32)
    processed = 0; rejected = 0; dropped = 0
    state = dict(input_valid=False, reason='WAIT_INPUT', motion_authority=False)
    pubs = {
        'scan_input': node.create_publisher(LaserScan, '/time_path/slam/input_scan', qos_profile_sensor_data),
        'wheel_input': node.create_publisher(Odometry, '/time_path/slam/input_odom', 10),
        'scan': node.create_publisher(LaserScan, '/time_path/slam/scan', qos_profile_sensor_data),
        'pose': node.create_publisher(PoseStamped, '/time_path/slam/pose', 10),
        'map': node.create_publisher(OccupancyGrid, '/time_path/slam/recent_occupancy', qos_profile_sensor_data),
        'objects': node.create_publisher(String, '/time_path/slam/obstacles', 10),
        'status': node.create_publisher(String, '/time_path/slam/status', 10),
        'markers': node.create_publisher(MarkerArray, '/time_path/slam/markers', 10),
        'path': node.create_publisher(RosPath, '/time_path/slam/path', 10),
    }
    tf = TransformBroadcaster(node); static_tf = StaticTransformBroadcaster(node)
    mount = TransformStamped(); mount.header.frame_id = 'time_slam_base_link'
    mount.child_frame_id = 'time_slam_lidar'; mount.transform.translation.x = 1.65
    mount.transform.translation.z = .0377; mount.transform.rotation.w = 1.
    static_tf.sendTransform(mount)

    def ns(stamp) -> int:
        return int(stamp.sec)*10**9+int(stamp.nanosec)

    def names(topic: str) -> list[str]:
        return sorted(e.node_namespace.rstrip('/')+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic))

    def yaw(q) -> float:
        if (not np.isfinite([q.x, q.y, q.z, q.w]).all()
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01
                or abs(q.x) > .04 or abs(q.y) > .04):
            raise ValueError('SLAM_PLANAR_QUATERNION')
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def on_clock(msg) -> None:
        nonlocal clock, clock_wall, fatal
        stamp = ns(msg.clock)
        if clock is not None and stamp < clock:
            fatal = 'CLOCK_RESET_RESTART_SLAM_REQUIRED'; pending.clear(); waiting.clear(); poses.clear()
        if stamp != clock:
            clock_wall = time.monotonic_ns()
        clock = stamp

    def on_wheel(msg) -> None:
        nonlocal wheel_ns, wheel_wall, fatal
        if fatal or names('/time_path/wheel_odometry') != ['/time_path_controller']:
            return
        try:
            if msg.header.frame_id != 'time_wheel_odom' or msg.child_frame_id != 'base_link':
                raise ValueError('WHEEL_FRAME')
            angle = yaw(msg.pose.pose.orientation); stamp = ns(msg.header.stamp)
            if wheel_ns is not None and stamp <= wheel_ns:
                raise ValueError('WHEEL_STAMP_ORDER')
            if not np.isfinite([msg.pose.pose.position.x, msg.pose.pose.position.y]).all():
                raise ValueError('WHEEL_POSITION')
            wheel_ns = stamp; wheel_wall = time.monotonic_ns()
            out = copy.deepcopy(msg); out.header.frame_id = 'time_slam_wheel_origin'
            out.child_frame_id = 'time_slam_lidar'
            out.pose.pose.position.x += 1.65*math.cos(angle)
            out.pose.pose.position.y += 1.65*math.sin(angle)
            out.pose.pose.position.z = 0.
            # Cartographer uses pose only; do not mislabel base-origin twist as sensor twist.
            out.twist.twist.linear.x = out.twist.twist.linear.y = out.twist.twist.angular.z = 0.
            pubs['wheel_input'].publish(out)
        except ValueError as exc:
            fatal = str(exc)

    def on_scan(msg) -> None:
        nonlocal dropped
        if len(pending) == pending.maxlen:
            dropped += 1
        pending.append((msg, time.monotonic_ns()))

    def on_pose(msg) -> None:
        nonlocal fatal
        if fatal or names('/time_slam/tracked_pose') != ['/time_slam/cartographer']:
            return
        try:
            if msg.header.frame_id != 'time_slam_map':
                raise ValueError('SLAM_POSE_FRAME')
            stamp = ns(msg.header.stamp)
            if poses and stamp <= poses[-1].stamp_ns:
                if stamp == poses[-1].stamp_ns:
                    return  # The installed Cartographer fork normally suppresses these.
                raise ValueError('SLAM_POSE_STAMP_ORDER')
            poses.append(TimedBodyPose(stamp, 'sim', '0', 'time_slam_map', 'time_slam_lidar',
                msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)))
        except ValueError as exc:
            fatal = str(exc)

    def on_plan(msg) -> None:
        try:
            packet = json.loads(msg.data)
            xy = np.asarray(packet['raw_xy_m'], dtype=float)
            if (packet['event'] != 'PLAN' or packet['frame'] != 'base_link'
                    or xy.shape != (30, 2) or not np.isfinite(xy).all()
                    or type(packet.get('observation_ns')) is not int or packet['observation_ns'] < 0):
                raise ValueError('PLAN_CONTRACT')
            if args.mppi and (packet.get('run_id') != args.output.name or packet.get('epoch') != '0'
                    or packet.get('checkpoint_sha256') != mppi_config['checkpoint_sha256']
                    or packet.get('producer_kind') != 'LEARNED_TIME_MODEL'
                    or packet.get('clock') != 'sim' or packet.get('dt_s') != .1):
                raise ValueError('MPPI_MODEL_IDENTITY')
            if args.mppi:
                validate_time_geometry(xy)
            plans.append((packet, time.monotonic_ns()))
        except (ValueError, KeyError, TypeError) as exc:
            plans.clear()
            journal.write(json.dumps(dict(event='PLAN_REJECTED', reason=str(exc)))+'\n')

    node.create_subscription(Clock, '/clock', on_clock, 10)
    node.create_subscription(Odometry, '/time_path/wheel_odometry', on_wheel, 10)
    node.create_subscription(LaserScan, '/sensing/lidar/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/time_slam/tracked_pose', on_pose, 10)
    node.create_subscription(String, '/time_path/plan', on_plan, 10)

    def clear_display() -> None:
        delete = Marker(); delete.action = Marker.DELETEALL
        pubs['markers'].publish(MarkerArray(markers=[delete]))
        path = RosPath(); path.header.frame_id = 'time_slam_map'; pubs['path'].publish(path)

    def process(scan, receipt: int) -> None:
        nonlocal processed, last_process_ns, last_process_wall, state
        stamp = ns(scan.header.stamp)
        pose = slam_scan_pose(poses, stamp)
        p = np.array([pose.x_m, pose.y_m, pose.yaw_rad])
        path_world = None
        plan_observation_ns = None
        if names('/time_path/plan') == ['/time_path_inference']:
            # SLAM can trail the newest prediction. Select a received plan whose
            # observation pose is already available, without relabeling its time.
            for plan, plan_wall in reversed(plans):
                if (plan['observation_ns'] > stamp or not 0 <= clock-plan['observation_ns'] <= 500_000_000
                        or time.monotonic_ns()-plan_wall > 750_000_000):
                    continue
                try:
                    anchor = interpolate_body_pose(poses, plan['observation_ns'], tolerance_ns=100_000_000)
                except ValueError:
                    continue  # Explicitly unavailable until a bracketed observation exists.
                body_pose = np.array([anchor.x_m-1.65*math.cos(anchor.yaw_rad),
                                      anchor.y_m-1.65*math.sin(anchor.yaw_rad), anchor.yaw_rad])
                xy = np.asarray(plan['raw_xy_m'])
                if np.linalg.norm(np.diff(np.vstack(([0., 0.], xy)), axis=0), axis=1).max() > .01:
                    path_world = transform(xy, body_pose); plan_observation_ns = plan['observation_ns']
                break
        state = detector.update(stamp, scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max, p, path_world)
        state.update(event='OBSTACLES', run_id=args.output.name, reason='OK', sim_ns=clock, monotonic_ns=time.monotonic_ns(),
                     gnss_imu_map_inputs=False, pose_source='Cartographer scan+wheel_odometry',
                     plan_observation_ns=plan_observation_ns,
                     scan_age_s=(clock-stamp)/1e9)
        if avoidance is not None:
            started = time.monotonic()
            state['mppi'] = avoidance.update(state, path_world, detector.grid)
            state['mppi']['compute_wall_s'] = time.monotonic()-started
            if state['mppi']['compute_wall_s'] > .15:
                state['mppi'].update(mode='STOP', reason='MPPI_COMPUTE_TIMEOUT', target_speed_mps=0.)
            state.update(epoch='0', checkpoint_sha256=mppi_config['checkpoint_sha256'])
        processed += 1; last_process_ns = stamp; last_process_wall = time.monotonic_ns()
        journal.write(json.dumps(state, allow_nan=False)+'\n')
        pubs['objects'].publish(String(data=json.dumps(state, allow_nan=False)))
        base = state['base_pose_xyyaw']
        tr = TransformStamped(); tr.header.stamp = scan.header.stamp
        tr.header.frame_id = 'time_slam_map'; tr.child_frame_id = 'time_slam_base_link'
        tr.transform.translation.x = base[0]; tr.transform.translation.y = base[1]
        tr.transform.rotation.z = math.sin(base[2]/2); tr.transform.rotation.w = math.cos(base[2]/2)
        tf.sendTransform(tr)
        out = copy.deepcopy(scan); out.header.frame_id = 'time_slam_lidar'
        out.time_increment = 0.  # AWSIM snapshot; RViz must not deskew into future TF.
        pubs['scan'].publish(out)
        pose_msg = PoseStamped(); pose_msg.header = tr.header
        pose_msg.pose.position.x = base[0]; pose_msg.pose.position.y = base[1]
        pose_msg.pose.orientation = tr.transform.rotation; pubs['pose'].publish(pose_msg)
        grid = OccupancyGrid(); grid.header = tr.header; grid.info.resolution = detector.grid.resolution_m
        grid.info.width = grid.info.height = detector.grid.size
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, detector.grid.origin*detector.grid.resolution_m)
        grid.info.origin.orientation.w = 1.; grid.data = detector.grid.values.ravel().tolist()
        pubs['map'].publish(grid)
        delete = Marker(); delete.action = Marker.DELETEALL; markers = [delete]
        for surface in state['surfaces']:
            marker = Marker(); marker.header = tr.header; marker.ns = 'occupied_surface'; marker.id = surface['id']
            marker.type = Marker.CUBE; marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = (
                (surface['min_xy_m'][i]+surface['max_xy_m'][i])/2 for i in range(2))
            marker.pose.position.z = .15; marker.pose.orientation.w = 1.
            marker.scale.x = max(.15, surface['max_xy_m'][0]-surface['min_xy_m'][0])
            marker.scale.y = max(.15, surface['max_xy_m'][1]-surface['min_xy_m'][1]); marker.scale.z = .3
            marker.color.a = .55; marker.color.r = 1.
            marker.color.g = .05 if surface['path_overlap'] else .8; marker.color.b = .05
            if surface['extent_m'] > 3.:
                # A long curved wall's bounding rectangle is not occupied solid.
                # Draw its observed points instead of filling across the road.
                marker.type = Marker.POINTS; marker.pose.position.x = marker.pose.position.y = 0.
                marker.scale.x = marker.scale.y = .15
                for x, y in surface['points_xy_m']:
                    marker.points.append(Point(x=float(x), y=float(y), z=0.))
            marker.lifetime.nanosec = 400_000_000; markers.append(marker)
        pubs['markers'].publish(MarkerArray(markers=markers))
        path = RosPath(); path.header = tr.header
        if path_world is not None:
            displayed = (state['mppi']['path_world_xy_m'] if avoidance is not None
                         and state['mppi']['mode'] == 'AVOID' else path_world)
            for x, y in displayed:
                point = PoseStamped(); point.header = tr.header
                point.pose.position.x = float(x); point.pose.position.y = float(y); point.pose.orientation.w = 1.
                path.poses.append(point)
        pubs['path'].publish(path)

    def tick() -> None:
        nonlocal last_feed, rejected, dropped, state
        now = time.monotonic_ns()
        reason = fatal
        if not reason and (clock is None or now-clock_wall > 500_000_000): reason = 'CLOCK_STALE'
        if not reason and (wheel_ns is None or now-wheel_wall > 500_000_000): reason = 'WHEEL_STALE'
        if not reason and (names('/sensing/lidar/scan') != ['/awsim_d1']
                           or names('/time_path/wheel_odometry') != ['/time_path_controller']): reason = 'SOURCE_INVALID'
        if reason:
            state = dict(input_valid=False, reason=reason, motion_authority=False)
            clear_display(); return
        while pending:
            scan, receipt = pending[0]; stamp = ns(scan.header.stamp)
            if (stamp > clock or stamp > wheel_ns) and now-receipt < 500_000_000:
                break
            pending.popleft()
            if stamp-last_feed < 100_000_000: continue
            if not 0 <= clock-stamp <= 500_000_000 or now-receipt > 500_000_000:
                dropped += 1; continue
            try:
                if scan.header.frame_id != 'lidar' or len(scan.ranges) != 750:
                    raise ValueError('AWSIM_SCAN_FRAME_OR_SHAPE')
                scan_geometry(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
                out = copy.deepcopy(scan); out.header.frame_id = 'time_slam_lidar'; out.time_increment = 0.
                r = np.asarray(out.ranges, dtype=float)
                r[~np.isfinite(r) | (r <= scan.range_min) | (r >= scan.range_max-.01)] = np.inf
                out.ranges = r.astype(np.float32).tolist(); pubs['scan_input'].publish(out)
                if len(waiting) == waiting.maxlen: dropped += 1
                waiting.append((scan, receipt)); last_feed = stamp
            except ValueError as exc:
                rejected += 1; state = dict(input_valid=False, reason=str(exc), motion_authority=False)
        while waiting and poses:
            scan, receipt = waiting[0]; stamp = ns(scan.header.stamp)
            if cartographer_stamp_ns(stamp) > poses[-1].stamp_ns and now-receipt < 750_000_000: break
            waiting.popleft()
            if not 0 <= clock-stamp <= 500_000_000 or now-receipt > 750_000_000:
                dropped += 1; continue
            try:
                process(scan, receipt)
            except ValueError as exc:
                rejected += 1; state = dict(input_valid=False, reason=str(exc), motion_authority=False)
                journal.write(json.dumps(dict(event='SCAN_REJECTED', stamp_ns=stamp, reason=str(exc)))+'\n')
            break
        if last_process_ns is None or now-last_process_wall > 750_000_000 or clock-last_process_ns > 500_000_000:
            state = dict(input_valid=False, reason='SCAN_OR_SLAM_STALE', motion_authority=False)
            clear_display()

    def heartbeat() -> None:
        graph = {}
        for name, namespace in [('time_slam_obstacles', '/'), ('cartographer', '/time_slam'),
                                 ('time_path_controller', '/'), ('time_path_inference', '/')]:
            try:
                graph[namespace.rstrip('/')+'/'+name] = dict(subscriptions=node.get_subscriber_names_and_types_by_node(name, namespace),
                    publishers=node.get_publisher_names_and_types_by_node(name, namespace))
            except rclpy.node.NodeNameNonExistentError:
                pass
        snapshot = dict(state, processed=processed, rejected=rejected, dropped=dropped, graph=graph,
                        monotonic_ns=time.monotonic_ns(), sim_ns=clock)
        pubs['status'].publish(String(data=json.dumps(snapshot, allow_nan=False)))
        pending_file = args.output/'slam_obstacles.pending'
        pending_file.write_text(json.dumps(snapshot, allow_nan=False)); pending_file.replace(args.output/'slam_obstacles_status.json')
        ready = args.output/'slam_obstacles_ready.json'
        rviz = [e.node_name for e in node.get_subscriptions_info_by_topic('/time_path/slam/markers')
                if e.node_name.startswith('rviz')]
        if not ready.exists() and state['input_valid'] and processed >= 3 and rviz:
            for description in graph.values():
                if any('gnss' in t or 'imu' in t or t.startswith('/localization/')
                       for t, _ in description['subscriptions']):
                    raise RuntimeError('UNEXPECTED_GLOBAL_POSE_OR_GNSS_IMU_INPUT')
            ready.write_text(json.dumps(snapshot, indent=2))

    node.create_timer(.02, tick); node.create_timer(.25, heartbeat)
    try:
        rclpy.spin(node)
    finally:
        clear_display(); journal.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
