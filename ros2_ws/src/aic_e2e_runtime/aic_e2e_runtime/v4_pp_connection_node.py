"""V4 records + current extrapolated pose/status -> isolated existing-PP inputs.

Hard-coded /shadow outputs, no command/engage publisher. Wall watchdog clears
reference even when simulation clock pauses. This is not a vehicle stop proof.
"""
import json
import math
import time
# Resolve canonical installed math code without importing model/torch bootstrap.
from pathlib import Path
import sys
prefix = Path(__file__).resolve().parents[4]
for source in (prefix/'src', prefix/'share/aic_e2e_runtime/python_src'):
    if (source/'aic_transfuser_lite/control/v4_pp_connection.py').is_file():
        sys.path.insert(0, str(source)); break
else:
    raise RuntimeError('V4 canonical Python source is not installed')
from aic_transfuser_lite.control.path_control_bridge import Limits, Vehicle
from aic_transfuser_lite.control.v4_pp_connection import ShadowPPConnection, pp_center_pose
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_to_root


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
    from pathlib import Path
    rclpy.init(args=args)
    node = Node('v4_pp_connection', use_global_arguments=True)
    node.declare_parameter('config_file', '')
    config = json.loads(Path(node.get_parameter('config_file').value).read_text())
    if config.get('mode') != 'SHADOW_ONLY':
        raise ValueError('ONLY_SHADOW_AUTHORIZED')
    limits = Limits(**config['limits'])
    conn = ShadowPPConnection(limits, config['fixed_frame'])
    if not conn.adapter._bridge._limits_valid():
        raise ValueError('INVALID_LIMITS')
    trajectory = node.create_publisher(Trajectory, '/shadow/v4/pp/trajectory', 1)
    odometry = node.create_publisher(Odometry, '/shadow/v4/pp/odometry', 1)
    status = node.create_publisher(String, '/shadow/v4/pp/status', 1)
    cache = {}; clock = [None, 0.]; last_tick = [None]
    def ns(t): return t.sec * 10**9 + t.nanosec
    def emit(reason, reference=None):
        msg = String(); msg.data = json.dumps(dict(reason=reason,
            plan_id=None if reference is None else reference.plan_id,
            source_s=None if reference is None else reference.source_s,
            expires_s=None if reference is None else reference.expires_s,
            actuator_connected=False)); status.publish(msg)
    def clear(reason):
        conn.invalidate(reason); cache.clear()
        trajectory.publish(Trajectory()); emit(reason)
    def on_clock(msg):
        stamp = ns(msg.clock) * 1e-9
        if clock[0] is not None and stamp < clock[0]:
            clear('CLOCK_RESET'); last_tick[0] = None
        clock[:] = [stamp, time.monotonic()]
    def receive(role, msg):
        ends = node.get_publishers_info_by_topic(config['topics'][role])
        ids = [e.node_namespace.rstrip('/')+'/'+e.node_name for e in ends]
        if ids != [config['expected_nodes'][role]]:
            clear('SOURCE_MISMATCH_'+role); return
        if role == 'plan':
            if clock[0] is None: return
            try: packet = json.loads(msg.data)
            except ValueError: clear('PLAN_JSON_INVALID'); return
            if not conn.accept(packet, clock[0]):
                trajectory.publish(Trajectory()); emit(conn.reason)
        else:
            cache[role] = (msg, time.monotonic())
    node.create_subscription(Clock, '/clock', on_clock, 10)
    for role, kind in [('plan', String), ('pose', PoseStamped),
                       ('velocity', VelocityReport), ('steering', SteeringReport)]:
        node.create_subscription(kind, config['topics'][role],
            lambda msg, role=role: receive(role, msg), qos_profile_sensor_data)
    def tick():
        now = clock[0]
        if now is None or time.monotonic()-clock[1] > .5:
            clear('CLOCK_WALL_STALE'); return
        if last_tick[0] == now: return
        last_tick[0] = now
        try:
            pose, velocity, steering = [cache[k][0] for k in ('pose', 'velocity', 'steering')]
            stamps = [ns(pose.header.stamp)*1e-9, ns(velocity.header.stamp)*1e-9,
                      ns(steering.stamp)*1e-9]
            if (any(not -limits.future_tolerance_s <= now-t <= limits.state_ttl_s for t in stamps)
                or max(stamps)-min(stamps) > config['state_alignment_s']
                or any(time.monotonic()-cache[k][1] > .5 for k in ('pose','velocity','steering'))):
                raise ValueError('STATE_STALE_OR_UNALIGNED')
            if pose.header.frame_id != config['fixed_frame']: raise ValueError('POSE_FRAME')
            q = pose.pose.orientation; p = pose.pose.position
            if abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1) > .01: raise ValueError('POSE_QUATERNION')
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            root = lidar_to_root(p.x,p.y,yaw,config['lidar_forward_m'])
            if conn.context is None: raise ValueError('NO_PLAN')
            state = Vehicle(str(stamps[0]), min(stamps), conn.context[1], conn.context[2],
                root, velocity.longitudinal_velocity, steering.steering_tire_angle)
            ref = conn.tick(now, state)
            if ref is None:
                trajectory.publish(Trajectory()); emit(conn.reason); return
            out = Trajectory(); out.header.frame_id = ref.frame
            stamp_ns = round(ref.source_s*1e9)
            out.header.stamp.sec = stamp_ns//10**9; out.header.stamp.nanosec = stamp_ns%10**9
            for xy, angle, speed in zip(ref.xy_m, ref.yaw_rad, ref.speed_mps):
                point = TrajectoryPoint(); point.pose.position.x=float(xy[0]); point.pose.position.y=float(xy[1])
                point.pose.orientation.z=math.sin(angle/2); point.pose.orientation.w=math.cos(angle/2)
                point.longitudinal_velocity_mps=float(speed); out.points.append(point)
            odom = Odometry(); odom.header = pose.header; odom.child_frame_id='v4_axle_midpoint'
            center = pp_center_pose(root, limits.wheelbase_m, limits.rear_x_in_base_m)
            odom.pose.pose.position.x=center[0]; odom.pose.pose.position.y=center[1]
            odom.pose.pose.orientation.z=math.sin(yaw/2); odom.pose.pose.orientation.w=math.cos(yaw/2)
            odom.twist.twist.linear.x=float(state.speed_mps)
            odometry.publish(odom); trajectory.publish(out); emit(conn.reason, ref)
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            clear('STATE_INVALID:'+str(exc))
    # Independent of plan arrival; no inference in this callback.
    node.create_timer(limits.period_s, tick)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        # SIGINT may have already shut down the middleware context. Consumer's
        # existing wall-time input timeout handles that case; no invalid publish.
        if rclpy.ok(): clear('CONNECTION_CLOSED')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
