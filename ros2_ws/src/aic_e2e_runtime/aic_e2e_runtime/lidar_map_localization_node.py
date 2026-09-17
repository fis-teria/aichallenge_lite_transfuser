"""GNSS/IMU-free map alignment. Publishes only localization and display data."""
from __future__ import annotations

import argparse
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import time

from .canonical_source import prefer_canonical_source
prefer_canonical_source()
import numpy as np
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose
from aic_transfuser_lite.runtime.lidar_map_localization import (
    CORRECTION_MODES, CORRECTION_SCHEDULES, BoundaryMap, MapLocalizer, compose, scan_points,
)


def main(argv: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import remove_ros_args
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X_M','Y_M','YAW_RAD'))
    parser.add_argument('--wheel-source', default='/time_path_controller')
    parser.add_argument('--scan-source', default='/awsim_d1')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--correction-mode', choices=CORRECTION_MODES, default='bounded')
    parser.add_argument('--correction-schedule', choices=CORRECTION_SCHEDULES, default='all')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    course = BoundaryMap.from_lanelet(args.map)
    tracker = MapLocalizer(course, correction_mode=args.correction_mode,
                           correction_schedule=args.correction_schedule)
    if args.initial_pose is not None:
        tracker.initialize(np.asarray(args.initial_pose))
    map_sha = hashlib.sha256(args.map.read_bytes()).hexdigest()
    log = args.output.open('x', encoding='utf-8') if args.output else None
    rclpy.init(args=argv)
    node = Node('time_lidar_map_localizer')
    tf = TransformBroadcaster(node); static_tf = StaticTransformBroadcaster(node)
    scan_pub = node.create_publisher(LaserScan, '/time_path/localization/scan', qos_profile_sensor_data)
    pose_pub = node.create_publisher(PoseStamped, '/time_path/localization/pose', 10)
    status_pub = node.create_publisher(String, '/time_path/localization/status', 10)
    wheel_topic = '/time_path/wheel_odometry'
    scans: deque[tuple[LaserScan, int]] = deque(maxlen=4)
    wheels: deque[TimedBodyPose] = deque(maxlen=512)
    clock: int | None = None
    clock_wall = 0
    last_scan_ns: int | None = None
    last_processed_wall = 0
    epoch = 0
    detail: dict = {}

    def stamp(t) -> int:
        return int(t.sec)*10**9+int(t.nanosec)

    def quaternion(q) -> float:
        v = [q.x,q.y,q.z,q.w]
        if not np.isfinite(v).all() or abs(sum(x*x for x in v)-1) > .01:
            raise ValueError('QUATERNION_INVALID')
        if abs(q.x) > .04 or abs(q.y) > .04:
            raise ValueError('PLANAR_POSE_REQUIRED')
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def tf_message(parent: str, child: str, xy_yaw: np.ndarray, ns: int, z: float = 0.):
        msg = TransformStamped(); msg.header.frame_id = parent; msg.child_frame_id = child
        msg.header.stamp.sec = ns//10**9; msg.header.stamp.nanosec = ns%10**9
        msg.transform.translation.x = float(xy_yaw[0]); msg.transform.translation.y = float(xy_yaw[1])
        msg.transform.translation.z = float(z)
        msg.transform.rotation.z = math.sin(float(xy_yaw[2])/2)
        msg.transform.rotation.w = math.cos(float(xy_yaw[2])/2)
        return msg

    static_tf.sendTransform(tf_message('time_localized_base_link','time_localized_lidar',
                                      np.array([1.65,0.,0.]),0,.0377))

    def names(topic: str) -> list[str]:
        return [e.node_namespace.rstrip('/')+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic)]

    def invalidate(reason: str, *, reset: bool = False) -> None:
        if reset:
            tracker.reset(); wheels.clear(); scans.clear()
        tracker.status = reason

    def on_clock(msg) -> None:
        nonlocal clock, clock_wall, epoch, last_scan_ns
        ns = stamp(msg.clock)
        if clock is not None and ns < clock:
            invalidate('CLOCK_RESET', reset=True); epoch += 1; last_scan_ns = None
        if clock != ns:
            clock_wall = time.monotonic_ns()
        clock = ns

    def on_wheel(msg) -> None:
        try:
            if msg.header.frame_id != 'time_wheel_odom' or msg.child_frame_id != 'base_link':
                raise ValueError('WHEEL_FRAME_INVALID')
            p = msg.pose.pose.position; ns = stamp(msg.header.stamp)
            pose = TimedBodyPose(ns,'sim',str(epoch),'time_wheel_odom','base_link',p.x,p.y,
                                 quaternion(msg.pose.pose.orientation))
            if wheels and ns <= wheels[-1].stamp_ns:
                raise ValueError('WHEEL_STAMP_ORDER')
            wheels.append(pose)
        except ValueError as exc:
            invalidate(str(exc), reset=True)

    def on_seed(msg) -> None:
        nonlocal last_scan_ns, detail
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('INITIAL_POSE_MAP_FRAME_REQUIRED')
            p = msg.pose.pose.position
            tracker.initialize(np.array([p.x,p.y,quaternion(msg.pose.pose.orientation)]))
            scans.clear(); last_scan_ns = None; detail = {}
        except ValueError as exc:
            invalidate(str(exc), reset=True)

    node.create_subscription(Clock,'/clock',on_clock,10)
    node.create_subscription(Odometry,wheel_topic,on_wheel,10)
    node.create_subscription(LaserScan,'/sensing/lidar/scan',
                             lambda msg: scans.append((msg,time.monotonic_ns())),qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,'/time_path/localization/initialpose',on_seed,10)

    def tick() -> None:
        nonlocal last_scan_ns, last_processed_wall, detail
        now = time.monotonic_ns()
        if clock is None or now-clock_wall > 500_000_000:
            invalidate('CLOCK_STALE')
        elif names(wheel_topic) != [args.wheel_source] or names('/sensing/lidar/scan') != [args.scan_source]:
            invalidate('SOURCE_INVALID')
        elif not wheels or not -20_000_000 <= clock-wheels[-1].stamp_ns <= 200_000_000:
            invalidate('WHEEL_STALE')
        else:
            while scans:
                scan, receipt = scans[0]; ns = stamp(scan.header.stamp)
                if ns > clock or ns > wheels[-1].stamp_ns:
                    if now-receipt <= 350_000_000:
                        break
                scans.popleft()
                if last_scan_ns is not None and ns-last_scan_ns < 190_000_000:
                    continue
                if not 0 <= clock-ns <= 350_000_000 or now-receipt > 350_000_000:
                    invalidate('SCAN_STALE'); continue
                try:
                    if scan.header.frame_id != 'lidar':
                        raise ValueError('SCAN_FRAME_INVALID')
                    wheel = interpolate_body_pose(wheels,ns)
                    xy = np.array([wheel.x_m,wheel.y_m,wheel.yaw_rad])
                    points = scan_points(np.asarray(scan.ranges),scan.angle_min,scan.angle_increment,
                                         scan.range_min,scan.range_max)
                    result = tracker.update(ns,xy,points)
                    last_scan_ns = ns; last_processed_wall = now
                    detail = dict(scan_ns=ns,points=len(points),map_match_attempted=result is not None,
                                  map_match_applied=result is not None and result.accepted)
                    if result is not None:
                        detail.update(reason=result.reason,rank=result.rank,inliers=result.inliers,
                            fraction=result.fraction,mean_distance_m=result.mean_distance_m if math.isfinite(result.mean_distance_m) else None,
                            p90_distance_m=result.p90_distance_m if math.isfinite(result.p90_distance_m) else None,
                            all_mean_distance_m=result.all_mean_distance_m if math.isfinite(result.all_mean_distance_m) else None,
                            all_p90_distance_m=result.all_p90_distance_m if math.isfinite(result.all_p90_distance_m) else None,
                            correction_m=result.correction_m,correction_rad=result.correction_rad)
                    if tracker.valid(clock):
                        estimated_pose = compose(tracker.map_to_odom, xy)
                        tf.sendTransform([tf_message('map','time_wheel_odom',tracker.map_to_odom,ns,course.display_z_m),
                                          tf_message('time_wheel_odom','time_localized_base_link',xy,ns)])
                        out = copy.deepcopy(scan); out.header.frame_id = 'time_localized_lidar'; scan_pub.publish(out)
                        pose_msg = PoseStamped(); pose_msg.header = out.header; pose_msg.header.frame_id = 'map'
                        pose_msg.pose.position.x = float(estimated_pose[0]); pose_msg.pose.position.y = float(estimated_pose[1])
                        pose_msg.pose.position.z = course.display_z_m
                        pose_msg.pose.orientation.z = math.sin(estimated_pose[2]/2); pose_msg.pose.orientation.w = math.cos(estimated_pose[2]/2)
                        pose_pub.publish(pose_msg)
                except ValueError as exc:
                    invalidate('INPUT_'+str(exc))
                break
        valid = clock is not None and tracker.valid(clock) and now-last_processed_wall <= 500_000_000
        status = dict(valid=valid,mode=tracker.status,map_sha256=map_sha,
                      correction_mode=args.correction_mode,
                      correction_schedule=args.correction_schedule,
                      correction_enabled=(args.correction_schedule == 'all' or tracker.straight_gate.allowed),
                      wheel_yaw_rate_radps=tracker.straight_gate.yaw_rate_radps,
                      wheel_curvature_inv_m=tracker.straight_gate.curvature_inv_m,
                      pose_source='wheel_prediction' if tracker.status == 'ODOMETRY_ONLY' else 'scan_map_match',
                      last_map_match_ns=tracker.last_stamp_ns,
                      map_to_odom=None if not valid else tracker.map_to_odom.tolist(),
                      correction_frame='map->time_wheel_odom',gnss_imu_inputs=False,
                      motion_authority=False,planar_display_z_m=course.display_z_m,**detail)
        message = String(); message.data = json.dumps(status,allow_nan=False); status_pub.publish(message)
        if log:
            log.write(message.data+'\n'); log.flush()

    node.create_timer(.05,tick)
    try:
        rclpy.spin(node)
    finally:
        if log: log.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
