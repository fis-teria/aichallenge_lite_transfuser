"""Official ROS wiring smoke with prescribed synthetic poses, NOT AWSIM driving.

Run ONLY in a Docker container with --network none, ROS_DOMAIN_ID=1 and no
physical devices. Sensors deliberately follow prescribed states, not commands.
The result validates two PP processes, collector selection, bag and markers;
it provides no recovery dynamics evidence and no eligible training data.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--reference-root', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if not Path('/.dockerenv').is_file() or os.environ.get('LARGE_RECOVERY_ISOLATED_SYNTHETIC_SMOKE') != '1':
        raise ValueError('ISOLATED_SYNTHETIC_CONTAINER_REQUIRED')
    interfaces = {p.name for p in Path('/sys/class/net').iterdir()}
    if interfaces != {'lo'}:
        raise ValueError('NETWORK_NONE_REQUIRED')
    args.output.mkdir(parents=True, exist_ok=False)
    ref = json.loads((args.reference_root/'left.json').read_bytes())
    if len(ref['large_recovery']['config']['sites']) != 1:
        raise ValueError('SYNTHETIC_SINGLE_EVENT_FIXTURE_REQUIRED')
    site = ref['large_recovery']['config']['sites'][0]
    baseline = np.asarray(ref['baseline_xy_m']); guide = np.asarray(ref['large_recovery']['nominal_guide'])
    progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(baseline, axis=0), axis=1))]
    run_id = 'codex-time-recovery-large-synthetic-smoke'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import Image, Imu, LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    sensor = Node('awsim_d1'); localization = Node('ekf_localizer', namespace='/localization')
    display = Node('rviz_synthetic_smoke')
    executor = SingleThreadedExecutor()
    for node in (sensor, localization, display): executor.add_node(node)
    pubs = {name:sensor.create_publisher(kind, topic, 10) for name, kind, topic in (
        ('clock', Clock, '/clock'), ('image', Image, '/sensing/camera/image_raw'),
        ('scan', LaserScan, '/sensing/lidar/scan'), ('imu', Imu, '/sensing/imu/imu_raw'),
        ('velocity', VelocityReport, '/vehicle/status/velocity_status'), ('steering', SteeringReport, '/vehicle/status/steering_status'))}
    pose_pub = localization.create_publisher(Odometry, '/localization/kinematic_state', 10)
    tf_pub = sensor.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sensor.create_subscription(AckermannControlCommand, '/control/command/control_cmd', lambda m:None, 10)
    path_counts = {}; marker_count = [0]
    for name in ('baseline', 'reference', 'preparation', 'observed'):
        display.create_subscription(RosPath, '/recovery_teacher/'+name+'_path',
            lambda msg, name=name:path_counts.update({name:len(msg.poses)}), 10)
    display.create_subscription(MarkerArray, '/recovery_teacher/disturbance_markers',
        lambda msg:marker_count.__setitem__(0,len(msg.markers)), 10)
    transforms = TFMessage()
    for child, parent in (('imu_link','sensor_kit_base_link'), ('sensor_kit_base_link','base_link')):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.
        transforms.transforms.append(t)
    started=time.monotonic(); last_publish=0.; last_tf=0.; drive_start=None; stop_at=None
    child=None; stream=(args.output/'wrapper.log').open('x'); result={}
    try:
        child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('run_time_recovery_nodes.py')),
            '--output',str(args.output),'--reference-root',str(args.reference_root),'--side','left',
            '--run-id',run_id,'--speed-policy','aligned_gain4_v1'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        control={}; current_s=site['release_s_m']-14.
        while time.monotonic()-started < 90.:
            executor.spin_once(timeout_sec=.001)
            now=time.monotonic()
            if child.poll() is not None: raise RuntimeError('SMOKE_WRAPPER_EARLY_EXIT')
            heartbeat=args.output/'control_heartbeat.json'
            if heartbeat.exists(): control=json.loads(heartbeat.read_bytes())
            if control.get('fault'): raise RuntimeError('SMOKE_COLLECTOR:'+control['fault'])
            if drive_start is None and control.get('ready_ticks',0)>=5 and len(path_counts)==4:
                drive_start=now
                (args.output/'drive_authorized.json').write_text(json.dumps(dict(run_id=run_id,scope='MEASURED_RECOVERY_AWSIM',expires_monotonic_s=now+75.)))
            if stop_at is None and control.get('large_recovery',{}).get('state',{}).get('completed_events')==1:
                stop_at=now
                (args.output/'stop_request.json').write_text(json.dumps(dict(run_id=run_id,reason='SYNTHETIC_SMOKE_COMPLETE')))
            if control.get('stop_confirmed') and stop_at is not None:
                result=dict(status='SYNTHETIC_ROS_WIRING_PASS',awsim_driving=False,new_teacher_samples=0,
                    source_commit=os.environ.get('SOURCE_COMMIT'),path_counts=path_counts,marker_count=marker_count[0],
                    last_control=control,elapsed_s=now-started)
                break
            if now-last_publish < .025: continue
            last_publish=now; ns=round((now-started+1.)*1e9)
            stamp=Clock();stamp.clock.sec=ns//10**9;stamp.clock.nanosec=ns%10**9;pubs['clock'].publish(stamp)
            if now-last_tf>1.: tf_pub.publish(transforms);last_tf=now
            if drive_start is not None and stop_at is None:
                current_s=site['release_s_m']-14.+1.25*(now-drive_start)
            st=control.get('large_recovery',{}).get('state',{})
            deviation=0.
            if st.get('stage') in ('preparing','handover'):
                fraction=float(np.clip((current_s-(site['release_s_m']-10.))/8.,0.,1.))
                deviation=site['target_offset_m']*fraction*fraction*(3.-2.*fraction)
            elif st.get('stage')=='recovery':
                deviation=site['target_offset_m']*max(0.,1.-(ns-st['release_ns'])/4e9)
            i=min(len(baseline)-2,max(0,int(np.searchsorted(progress,current_s)-1)))
            tangent=math.atan2(*(baseline[i+1]-baseline[i])[::-1])
            offset=float(np.interp(current_s,guide[:,0],guide[:,1]))+deviation
            yaw=float(np.interp(current_s,guide[:,0],np.unwrap(guide[:,2])))
            x=float(np.interp(current_s,progress,baseline[:,0]))-math.sin(tangent)*offset
            y=float(np.interp(current_s,progress,baseline[:,1]))+math.cos(tangent)*offset
            speed=0. if stop_at is not None else 1.25
            pose=Odometry();pose.header.stamp=stamp.clock;pose.header.frame_id='map';pose.child_frame_id='base_link'
            pose.pose.pose.position.x=x;pose.pose.pose.position.y=y
            pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2)
            pose.twist.twist.linear.x=speed;pose_pub.publish(pose)
            velocity=VelocityReport();velocity.header.stamp=stamp.clock;velocity.header.frame_id='base_link';velocity.longitudinal_velocity=speed
            pubs['velocity'].publish(velocity)
            steering=SteeringReport();steering.stamp=stamp.clock;pubs['steering'].publish(steering)
            imu=Imu();imu.header.stamp=stamp.clock;imu.header.frame_id='imu_link';pubs['imu'].publish(imu)
            laser=LaserScan();laser.header.stamp=stamp.clock;laser.header.frame_id='lidar'
            laser.angle_min=-math.pi;laser.angle_max=math.pi;laser.angle_increment=2*math.pi/720
            laser.range_min=.05;laser.range_max=50.;laser.ranges=[30.]*721;pubs['scan'].publish(laser)
            image=Image();image.header.stamp=stamp.clock;image.header.frame_id='camera_link'
            image.height=2;image.width=2;image.encoding='rgb8';image.step=6;image.data=[0]*12;pubs['image'].publish(image)
        else: raise RuntimeError('SYNTHETIC_SMOKE_TIMEOUT')
    finally:
        (args.output/'finish_nodes.json').write_text('{}')
        if child is not None:
            try: child.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        stream.close()
        for node in (sensor,localization,display): node.destroy_node()
        rclpy.shutdown()
    nodes=json.loads((args.output/'nodes_result.json').read_bytes())
    if not nodes['closed_bag'] or nodes['error']: raise RuntimeError('SYNTHETIC_BAG_SHUTDOWN_FAILED')
    from aic_transfuser_lite.data.time_large_recovery_v1 import large_recovery_events
    rows=[json.loads(line) for line in (args.output/'control.jsonl').read_bytes().splitlines()]
    result['events']=large_recovery_events(rows);result['nodes']=nodes
    if len(result['events'])!=1 or not result['events'][0]['recovery_confirmed'] or marker_count[0]!=3:
        raise RuntimeError('SYNTHETIC_EVENT_OR_MARKER_FAILED')
    (args.output/'synthetic_smoke_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='last_control'}))


if __name__=='__main__': main()
