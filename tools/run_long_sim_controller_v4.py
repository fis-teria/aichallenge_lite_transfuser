"""Explicit, bounded AWSIM-only V4-20 controller with independent sensor watchdog.

Run only in the dedicated trial container. Host inspects instance ownership and
writes drive_authorized.json after official Start. No teacher/map/route input.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from aic_transfuser_lite.control.long_sim_tracking_v4 import tracking_command, check_scan
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_to_root


def main() -> None:
    ap=argparse.ArgumentParser()
    ap.add_argument('--authorize-awsim-only',action='store_true')
    args=ap.parse_args()
    if not args.authorize_awsim_only or os.environ.get('ROS_DOMAIN_ID')!='1':
        raise ValueError('EXPLICIT_AWSIM_TRIAL_REQUIRED')
    if any(Path(p).exists() for p in ('/dev/vcu','/dev/gnss','/dev/ttyUSB0')):
        raise ValueError('PHYSICAL_DEVICE_PRESENT')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    rclpy.init()
    node=Node('v4_20_controller',enable_rosout=False,start_parameter_services=False)
    root=Path('/probe/evidence')
    log=(root/'tracking.jsonl').open('x')
    cache={}; clock=[None,0.]; previous=[0.,time.monotonic()]
    state=dict(powered=False,armed_at=None,stopped_since=None,stop_confirmed=False,
               fault=None,publish_count=0,positive_count=0,plan_ids=[],max_speed_mps=0.)
    started=time.monotonic()
    publisher=None
    def stamp(t):return t.sec+t.nanosec*1e-9
    def names(topic):
        return [e.node_namespace.rstrip('/')+'/'+e.node_name for e in node.get_publishers_info_by_topic(topic)]
    def record(value):
        log.write(json.dumps(dict(monotonic_s=time.monotonic(),sim_s=clock[0],**value),allow_nan=False)+'\n');log.flush()
    def heartbeat(reason):
        p=root/'tracking.pending'
        p.write_text(json.dumps(dict(state,reason=reason,monotonic_s=time.monotonic())))
        p.replace(root/'tracking_heartbeat.json')
    def on_clock(m):
        now=stamp(m.clock)
        if clock[0] is not None and now<clock[0]:state['fault']='CLOCK_RESET'
        clock[:]=[now,time.monotonic()]
    node.create_subscription(Clock,'/clock',on_clock,10)
    topics=dict(plan=('/shadow/v4/plan_record',String,'/v4_slam_pose_adapter'),
        pose=('/cartographer_v4/extrapolated_pose',PoseStamped,'/cartographer_v4/cartographer'),
        velocity=('/vehicle/status/velocity_status',VelocityReport,'/awsim_d1'),
        steering=('/vehicle/status/steering_status',SteeringReport,'/awsim_d1'),
        scan=('/sensing/lidar/scan',LaserScan,'/awsim_d1'))
    def receive(role,msg):
        cache[role]=(msg,time.monotonic())
    for role,(topic,kind,_) in topics.items():
        node.create_subscription(kind,topic,lambda m,role=role:receive(role,m),qos_profile_sensor_data)
    def tick():
        nonlocal publisher
        now=clock[0];wall=time.monotonic()
        if wall-started>150:state['fault']='CONTROLLER_WALL_LIMIT'
        actual=names('/control/command/control_cmd')
        if publisher is None:
            ends=node.get_subscriptions_info_by_topic('/control/command/control_cmd')
            if actual:state['fault']='COMPETING_CONTROLLER'
            elif any(e.node_name=='awsim_d1' for e in ends) and now is not None:
                publisher=node.create_publisher(AckermannControlCommand,'/control/command/control_cmd',10)
                record(dict(event='CONTROL_PUBLISHER_CREATED'))
            heartbeat('WAIT_CONSUMER');return
        if actual!=['/v4_20_controller']:
            state['fault']='COMPETING_CONTROLLER';heartbeat(state['fault']);return
        reason='HOLD_BEFORE_AUTHORIZATION';accel=-1.;steer=previous[0];target=0.;plan_id=None;details={}
        auth=root/'drive_authorized.json'
        if auth.exists() and state['armed_at'] is None:
            value=json.loads(auth.read_text())
            if value.get('scope')!='V4_20_AWSIM_TRACKING' or wall>value['expires_monotonic_s']:
                state['fault']='AUTHORIZATION_INVALID'
            else:state['armed_at']=now;record(dict(event='ARMED',authorization=value))
        elapsed=None if state['armed_at'] is None or now is None else now-state['armed_at']
        try:
            if state['fault']:raise ValueError(state['fault'])
            if now is None or wall-clock[1]>.5:raise ValueError('CLOCK_STALE')
            for role in ('velocity','steering','scan','pose'):
                msg,received=cache[role]
                t=stamp(msg.stamp if role=='steering' else msg.header.stamp)
                if wall-received>.3 or not -.02<=now-t<=.3:raise ValueError('STALE_'+role)
                if names(topics[role][0])!=[topics[role][2]]:raise ValueError('SOURCE_'+role)
            speed=float(cache['velocity'][0].longitudinal_velocity)
            if not math.isfinite(speed):raise ValueError('SPEED_NONFINITE')
            state['max_speed_mps']=max(state['max_speed_mps'],abs(speed))
            if abs(speed)>.45:state['fault']='OVERSPEED';raise ValueError('OVERSPEED')
            if elapsed is not None and elapsed>=10:
                reason='SCHEDULED_BRAKE'
                if abs(speed)<.03:
                    if state['stopped_since'] is None:state['stopped_since']=now
                    state['stop_confirmed']=now-state['stopped_since']>=1.
                else:state['stopped_since']=None
            elif elapsed is not None:
                laser=cache['scan'][0]
                clearance=check_scan(laser.ranges,laser.angle_min,laser.angle_increment,laser.range_min,laser.range_max,speed)
                msg,received=cache['plan']
                if names(topics['plan'][0])!=[topics['plan'][2]]:raise ValueError('SOURCE_PLAN')
                p=json.loads(msg.data)
                if p.get('event')!='PLAN' or p.get('source')!='LONG_V4_20M_EPOCH12_UNCORRECTED':raise ValueError('PLAN_IDENTITY')
                if wall-received>.5 or not 0<=now-p['source_s']<=.5:raise ValueError('PLAN_STALE')
                if p['pose_frame']!='cartographer_v4_local' or p['clock']!='AWSIM_ROS':raise ValueError('PLAN_FRAME_CLOCK')
                pose=cache['pose'][0]
                if pose.header.frame_id!='cartographer_v4_local':raise ValueError('POSE_FRAME')
                q=pose.pose.orientation
                if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:raise ValueError('POSE_QUATERNION')
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                current=lidar_to_root(pose.pose.position.x,pose.pose.position.y,yaw,1.1649999618530273)
                details=tracking_command(p['raw_xy_m'],tuple(p['observation_pose_xyyaw']),current,speed)
                steer=details['steer_rad'];accel=details['acceleration_mps2'];target=details['target_speed_mps']
                plan_id=p['output_id'];details.update(clearance_m=clearance,current_pose=current,speed_mps=speed)
                reason='V4_20_TRACKING'
        except (ValueError,KeyError,TypeError) as exc:
            reason=str(exc);accel=-1.;target=0.
            if reason in ('STOPPING_CORRIDOR_OCCUPIED','OVERSPEED') and state['powered']:state['fault']=reason
        # Limit actual requested steering rate, including transitions between plans.
        dt=min(.1,max(0.,wall-previous[1]))
        steer=float(np.clip(steer,previous[0]-.8*dt,previous[0]+.8*dt))
        previous[:]=[steer,wall]
        if now is not None:
            m=AckermannControlCommand();ns=round(now*1e9)
            m.stamp.sec=ns//10**9;m.stamp.nanosec=ns%10**9
            m.lateral.steering_tire_angle=steer;m.longitudinal.acceleration=float(accel);m.longitudinal.speed=target
            publisher.publish(m);state['publish_count']+=1
            if accel>0:
                state['powered']=True;state['positive_count']+=1
                if plan_id not in state['plan_ids']:state['plan_ids'].append(plan_id)
            record(dict(event='COMMAND_SENT',reason=reason,plan_id=plan_id,steer_rad=steer,
                        acceleration_mps2=accel,target_speed_mps=target,details=details))
        heartbeat(reason)
    node.create_timer(.05,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        record(dict(event='CONTROLLER_END',state=state));log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
