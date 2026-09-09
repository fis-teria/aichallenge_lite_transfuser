"""Finite read-only stationary-test observer; no publishers."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_control_msgs.msg import AckermannControlCommand

rclpy.init()
n=Node('v4_stationary_observer',enable_rosout=False,start_parameter_services=False)
out=Path('/v4/evidence/guard.json')
s={'monotonic':time.monotonic(),'count':{},'fault':None,'ready':False}
last={}
trace=Path('/v4/evidence/motion.jsonl').open('x')
def receive(role,m):
    s['count'][role]=s['count'].get(role,0)+1
    if role=='command' and s['count'][role]>=60000: s['fault']='COMMAND_COUNT_LIMIT'
    last[role]=time.monotonic()
    if role=='clock':
        s['sim_s']=m.clock.sec+m.clock.nanosec*1e-9
        if s['sim_s']>110: s['fault']='TOTAL_SIM_TIME_LIMIT'
    elif role=='ego':
        s['speed_mps']=m.longitudinal_velocity
        s['max_abs_speed_mps']=max(s.get('max_abs_speed_mps',0),abs(m.longitudinal_velocity))
        permitted=Path('/v4/evidence/armed.json').exists()
        if not abs(m.longitudinal_velocity)<=(10. if permitted else .05): s['fault']='SPEED_LIMIT'
    elif role=='odom':
        s['pose_frame']=m.header.frame_id;s['child_frame']=m.child_frame_id
        p=m.pose.pose.position;s['xy_m']=[p.x,p.y]
        trace.write(json.dumps(dict(event='STATE',monotonic=time.monotonic(),
            stamp_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xy_m=s['xy_m'],
            speed_mps=s.get('speed_mps'),raw_command=s.get('command'),final_command=s.get('final_command')))+'\n')
        trace.flush()
        if (m.header.frame_id,m.child_frame_id)!=('map','base_link'): s['fault']='POSE_FRAME'
    elif role=='image':
        s['image']=[m.height,m.width,m.encoding]
        if (m.height,m.width)!=(256,384) or m.encoding not in ('rgb8','bgr8'): s['fault']='IMAGE_CONTRACT'
    elif role=='command':
        s['command']=[m.lateral.steering_tire_angle,m.longitudinal.speed,m.longitudinal.acceleration]
    elif role=='final':
        s['final_command']=[m.lateral.steering_tire_angle,m.longitudinal.speed,m.longitudinal.acceleration]
    s['ready']=all(k in s['count'] for k in ('clock','ego','odom','image','command','final')) and s['fault'] is None

spec=[('clock','/clock',Clock),('ego','/vehicle/status/velocity_status',VelocityReport),
      ('odom','/localization/kinematic_state',Odometry),('image','/sensing/camera/image_raw',Image),
      ('command','/control/command/control_cmd_raw',AckermannControlCommand),
      ('final','/control/command/control_cmd',AckermannControlCommand)]
subs=[]
for role,topic,kind in spec:
    def callback(m,role=role): receive(role,m)
    # One positional argument for the installed Humble callback classifier.
    def factory(role):
        def cb(m): receive(role,m)
        return cb
    subs.append(n.create_subscription(kind,topic,factory(role),qos_profile_sensor_data))
state_log=Path('/v4/evidence/vehicle_states.jsonl').open('x')
seen_start=False
def state_callback(message):
    global seen_start
    value=message.data.strip().lower()
    state_log.write(json.dumps(dict(state=value,monotonic=time.monotonic(),sim_s=s.get('sim_s')))+'\n')
    state_log.flush()
    if value=='start': seen_start=True
    if seen_start and value in ('finish','finished','finishedall'):
        s['finish_seen']=True
    if value in ('error','fault'): s['fault']='SIMULATOR_STATE_'+value
subs.append(n.create_subscription(String,'/awsim/state',state_callback,
    QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
start=time.monotonic()
try:
    while time.monotonic()-start<112 and s['fault'] is None:
        rclpy.spin_once(n,timeout_sec=.02)
        s['monotonic']=time.monotonic()
        if s['ready']:
            if any(time.monotonic()-last[k]>1 for k in last): s['fault']='INPUT_STALE'
            endpoints=n.get_publishers_info_by_topic('/control/command/control_cmd')
            if len(endpoints)!=1 or endpoints[0].node_namespace.rstrip('/')+'/'+endpoints[0].node_name!='/mpc_controller':
                s['fault']='FINAL_COMMAND_AUTHORITY'
        tmp=out.with_suffix('.pending');tmp.write_text(json.dumps(s));tmp.replace(out)
finally:
    state_log.close();trace.close();n.destroy_node();rclpy.shutdown()
