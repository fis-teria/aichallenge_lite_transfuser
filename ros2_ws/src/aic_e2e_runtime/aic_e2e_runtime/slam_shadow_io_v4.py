"""Optional SLAM input bridge and RViz-only display, no vehicle command APIs."""
import math
import json
import time
from . import spatial_path_shadow_node_v4 as _source_layout
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_to_root,display_points


class SlamShadowIO:
    def __init__(self, config, emit):
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry,Path
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        self.node=Node('v4_slam_pose_adapter',enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
        self.emit=emit;self.clock=None;self.last=None;self.display_stamp=None;self.display_wall=None
        self.frame='cartographer_v4_local';self.odom_type=Odometry;self.path_type=Path;self.pose_type=PoseStamped
        self.offset=config['lidar_forward_m']
        self.string_type=String
        self.plan_pub=self.node.create_publisher(String,'/shadow/v4/plan_record',1) if config.get('plan_transport',False) else None
        self.odom=self.node.create_publisher(Odometry,'/v4/slam_odometry',10)
        self.path=self.node.create_publisher(Path,'/shadow/v4/path',1) if config.get('rviz_path',False) else None
        self.node.create_subscription(Clock,'/clock',self.on_clock,10)
        self.node.create_subscription(PoseStamped,'/cartographer_v4/tracked_pose',self.on_pose,10)
        self.node.create_timer(.1,self.expire)

    def clear(self):
        if self.path is not None:
            msg=self.path_type();msg.header.frame_id=self.frame;self.path.publish(msg)
        self.display_stamp=None;self.display_wall=None

    def on_clock(self,msg):
        now=msg.clock.sec*10**9+msg.clock.nanosec
        if self.clock is not None and now<self.clock:
            self.last=None;self.clear();self.emit(dict(event='SLAM_CLOCK_RESET'))
        self.clock=now

    def on_pose(self,msg):
        ends=self.node.get_publishers_info_by_topic('/cartographer_v4/tracked_pose')
        if len(ends)!=1 or ends[0].node_namespace.rstrip('/')+'/'+ends[0].node_name!='/cartographer_v4/cartographer':
            self.clear();return
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.clock is None or not -50_000_000<=self.clock-ns<=250_000_000:return
        if msg.header.frame_id!=self.frame or (self.last is not None and ns<=self.last):return
        p,q=msg.pose.position,msg.pose.orientation
        if abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>.01:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x,y,yaw=lidar_to_root(p.x,p.y,yaw,self.offset)
        out=self.odom_type();out.header=msg.header;out.child_frame_id='v4_base_link'
        out.pose.pose.position.x=x;out.pose.pose.position.y=y
        out.pose.pose.orientation.z=math.sin(yaw/2);out.pose.pose.orientation.w=math.cos(yaw/2)
        for i in range(6):out.pose.covariance[i*7]=1e6
        self.odom.publish(out);self.last=ns
        self.emit(dict(event='SLAM_POSE_ADAPTED',stamp_ns=ns,source='/cartographer_v4/tracked_pose',frame=self.frame))

    def on_record(self,record):
        if self.plan_pub is not None and record.get('event') in ('PLAN','SESSION_END'):
            # Same forward, not a reconstructed RViz Path. Zero TTL remains zero;
            # this diagnostic transport does not authorize a controller.
            packet=dict(record,pose_frame=self.frame,transport_kind='DIAGNOSTIC_NOT_CONTROL')
            msg=self.string_type();msg.data=json.dumps(packet,allow_nan=False)
            self.plan_pub.publish(msg)
            self.emit(dict(event='PLAN_RECORD_PUBLISHED',output_id=record.get('output_id'),control_publish=False))
        if self.path is None:return
        if record.get('event')=='SESSION_END':self.clear();return
        if record.get('event')!='PLAN':return
        ns=round(record['source_s']*10**9)
        if self.clock is None or not 0<=self.clock-ns<=500_000_000:return
        points=display_points(record)
        msg=self.path_type();msg.header.frame_id=self.frame
        msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
        for x,y in points:
            p=self.pose_type();p.header=msg.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.
            msg.poses.append(p)
        self.path.publish(msg);self.display_stamp=ns;self.display_wall=time.monotonic()
        self.emit(dict(event='RVIZ_PATH_PUBLISHED',output_id=record['output_id'],source_ns=ns,points=len(points),control_publish=False))

    def expire(self):
        if self.display_stamp is not None and (self.clock-self.display_stamp>500_000_000 or time.monotonic()-self.display_wall>1.):self.clear()

    def close(self):
        self.clear();self.node.destroy_node()
