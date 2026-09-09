"""Velocity/steering local odometry; isolated TF, no actuator interface."""
import math
import json
from . import spatial_path_shadow_node_v4 as _source_layout
from aic_transfuser_lite.runtime.steering_odometry_v4 import SteeringOdometry


def create_local_odometry_node(geometry=None):
    """Build a separate ROS node; caller owns executor and ROS context."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    # The parent may have __node:=v4_shadow in global argv. Do not inherit that
    # remap or unrelated vehicle parameters into this independently named node.
    node=Node('v4_local_odometry',start_parameter_services=False,use_global_arguments=False)
    node.declare_parameter('expected_velocity_node','/awsim_d1')
    node.declare_parameter('max_gap_s',0.25)
    # Missing physical reference-point evidence must not silently become y=0.
    if not geometry or not geometry.get('evidence'):
        node.get_logger().error('LOCAL_ODOMETRY_DISABLED REFERENCE_GEOMETRY_REQUIRED')
        return node
    joined=SteeringOdometry(wheelbase_m=geometry['wheelbase_m'],
                           reference_left_offset_m=geometry['reference_left_offset_m'],
                           max_gap_s=node.get_parameter('max_gap_s').value)
    core=joined.core
    # Dedicated tree: do not give the existing base_link a second TF parent.
    parent,child='v4_odom','v4_base_link'
    topic='/vehicle/status/velocity_status'
    steering_topic='/vehicle/status/steering_status'
    publisher=node.create_publisher(Odometry,'/v4/local_odometry',10)
    tf=TransformBroadcaster(node)
    clock_ns=None

    def on_clock(message):
        nonlocal clock_ns
        now=message.clock.sec*1_000_000_000+message.clock.nanosec
        if clock_ns is not None and now<clock_ns:
            joined.reset()
            node.get_logger().warning('LOCAL_ODOMETRY_CLOCK_RESET epoch='+str(core.epoch))
        clock_ns=now
        flush()

    def validate_stamp(stamp):
        if stamp.sec<0 or not 0<=stamp.nanosec<1_000_000_000:
            core.fail('INVALID_STAMP')
        ns=stamp.sec*1_000_000_000+stamp.nanosec
        if not -50_000_000<=clock_ns-ns<=int(core.max_gap_s*1e9):
            core.fail('INPUT_CLOCK_AGE')
        return ns

    def validate_source(input_topic):
        endpoints=node.get_publishers_info_by_topic(input_topic)
        expected=node.get_parameter('expected_velocity_node').value
        if len(endpoints)!=1 or endpoints[0].node_namespace.rstrip('/')+'/'+endpoints[0].node_name!=expected:
            core.fail('INPUT_SOURCE')

    def on_steering(message):
        if clock_ns is None or core.fault: return
        try:
            validate_source(steering_topic)
            joined.add_steering(validate_stamp(message.stamp),message.steering_tire_angle)
            flush()
        except ValueError as exc:
            node.get_logger().error('LOCAL_ODOMETRY_FAULT '+str(exc))

    def on_velocity(message):
        if clock_ns is None or core.fault: return
        try:
            validate_source(topic)
            if message.header.frame_id!='base_link': core.fail('VELOCITY_FRAME')
            ns=validate_stamp(message.header.stamp)
            joined.add_velocity(ns,message.longitudinal_velocity,message.lateral_velocity,message.heading_rate)
            flush()
        except ValueError as exc:
            node.get_logger().error('LOCAL_ODOMETRY_FAULT '+str(exc))

    def flush():
        if clock_ns is None or core.fault: return
        try:
            records=joined.drain(clock_ns)
        except ValueError as exc:
            node.get_logger().error('LOCAL_ODOMETRY_FAULT '+str(exc))
            return
        for pose,trace in records:
            node.get_logger().info('LOCAL_ODOMETRY_INPUT '+json.dumps(trace,allow_nan=False))
            if trace['warning'] is not None:
                node.get_logger().warning(json.dumps(trace['warning'],allow_nan=False))
            odom=Odometry()
            odom.header.stamp.sec=pose.stamp_ns//1_000_000_000
            odom.header.stamp.nanosec=pose.stamp_ns%1_000_000_000
            odom.header.frame_id=parent;odom.child_frame_id=child
            odom.pose.pose.position.x=pose.x_m;odom.pose.pose.position.y=pose.y_m
            odom.pose.pose.orientation.z=math.sin(pose.yaw_rad*.5)
            odom.pose.pose.orientation.w=math.cos(pose.yaw_rad*.5)
            odom.twist.twist.linear.x=trace['vx_mps']
            odom.twist.twist.linear.y=trace['vy_mps']
            odom.twist.twist.angular.z=trace['estimated_yaw_rate_rps']
            # Covariance is not estimated. Conservative sentinel, not calibrated accuracy.
            for i in range(6):
                odom.pose.covariance[i*7]=1e6;odom.twist.covariance[i*7]=1e6
            transform=TransformStamped()
            transform.header=odom.header;transform.child_frame_id=child
            transform.transform.translation.x=pose.x_m;transform.transform.translation.y=pose.y_m
            transform.transform.rotation=odom.pose.pose.orientation
            publisher.publish(odom);tf.sendTransform(transform)

    subscriptions=[node.create_subscription(Clock,'/clock',on_clock,10),
                   node.create_subscription(SteeringReport,steering_topic,on_steering,qos_profile_sensor_data),
                   node.create_subscription(VelocityReport,topic,on_velocity,qos_profile_sensor_data)]
    node.get_logger().info('LOCAL_ODOMETRY_READY velocity_steering '+json.dumps(geometry))
    return node


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node=create_local_odometry_node()
    try: rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()
