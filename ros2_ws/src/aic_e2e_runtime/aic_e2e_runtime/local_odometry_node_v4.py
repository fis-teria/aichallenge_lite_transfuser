"""VelocityReport-only local odometry; isolated TF, no actuator interface."""
import math
import json
from . import spatial_path_shadow_node_v4 as _source_layout
from aic_transfuser_lite.runtime.local_odometry_v4 import LocalOdometry


def create_local_odometry_node():
    """Build a separate ROS node; caller owns executor and ROS context."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from autoware_auto_vehicle_msgs.msg import VelocityReport
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    # The parent may have __node:=v4_shadow in global argv. Do not inherit that
    # remap or unrelated vehicle parameters into this independently named node.
    node=Node('v4_local_odometry',start_parameter_services=False,use_global_arguments=False)
    node.declare_parameter('expected_velocity_node','/awsim_d1')
    node.declare_parameter('max_gap_s',0.25)
    node.declare_parameter('max_speed_mps',10.0)
    node.declare_parameter('max_yaw_rate_rps',2.0)
    core=LocalOdometry(**{k:node.get_parameter(k).value for k in
                        ('max_gap_s','max_speed_mps','max_yaw_rate_rps')})
    # Dedicated tree: do not give the existing base_link a second TF parent.
    parent,child='v4_odom','v4_base_link'
    topic='/vehicle/status/velocity_status'
    publisher=node.create_publisher(Odometry,'/v4/local_odometry',10)
    tf=TransformBroadcaster(node)
    clock_ns=None

    def on_clock(message):
        nonlocal clock_ns
        now=message.clock.sec*1_000_000_000+message.clock.nanosec
        if clock_ns is not None and now<clock_ns:
            core.reset()
            node.get_logger().warning('LOCAL_ODOMETRY_CLOCK_RESET epoch='+str(core.epoch))
        clock_ns=now

    def on_velocity(message):
        if clock_ns is None or core.fault: return
        try:
            endpoints=node.get_publishers_info_by_topic(topic)
            expected=node.get_parameter('expected_velocity_node').value
            if len(endpoints)!=1 or endpoints[0].node_namespace.rstrip('/')+'/'+endpoints[0].node_name!=expected:
                core.fail('VELOCITY_SOURCE')
            if message.header.frame_id!='base_link': core.fail('VELOCITY_FRAME')
            stamp=message.header.stamp
            if stamp.sec<0 or not 0<=stamp.nanosec<1_000_000_000: core.fail('INVALID_STAMP')
            ns=stamp.sec*1_000_000_000+stamp.nanosec
            if not -50_000_000<=clock_ns-ns<=int(core.max_gap_s*1e9):
                core.fail('VELOCITY_CLOCK_AGE')
            pose=core.update(ns,message.longitudinal_velocity,message.lateral_velocity,message.heading_rate)
            if pose is None: return
            if core.warning is not None:
                node.get_logger().warning(json.dumps(core.warning,allow_nan=False))
            odom=Odometry()
            odom.header.stamp=stamp;odom.header.frame_id=parent;odom.child_frame_id=child
            odom.pose.pose.position.x=pose.x_m;odom.pose.pose.position.y=pose.y_m
            odom.pose.pose.orientation.z=math.sin(pose.yaw_rad*.5)
            odom.pose.pose.orientation.w=math.cos(pose.yaw_rad*.5)
            odom.twist.twist.linear.x=message.longitudinal_velocity
            odom.twist.twist.linear.y=message.lateral_velocity
            odom.twist.twist.angular.z=message.heading_rate
            # Covariance is not estimated. Conservative sentinel, not calibrated accuracy.
            for i in range(6):
                odom.pose.covariance[i*7]=1e6;odom.twist.covariance[i*7]=1e6
            transform=TransformStamped()
            transform.header=odom.header;transform.child_frame_id=child
            transform.transform.translation.x=pose.x_m;transform.transform.translation.y=pose.y_m
            transform.transform.rotation=odom.pose.pose.orientation
            publisher.publish(odom);tf.sendTransform(transform)
        except ValueError as exc:
            node.get_logger().error('LOCAL_ODOMETRY_FAULT '+str(exc))

    subscriptions=[node.create_subscription(Clock,'/clock',on_clock,10),
                   node.create_subscription(VelocityReport,topic,on_velocity,qos_profile_sensor_data)]
    node.get_logger().info('LOCAL_ODOMETRY_READY velocity_only /v4/local_odometry v4_odom->v4_base_link')
    return node


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node=create_local_odometry_node()
    try: rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()
