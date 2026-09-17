"""Exercise the compiled CMA node through planner lease expiry and recovery."""
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from multi_purpose_mpc_ros_msgs.msg import StateLatticeDirectTrajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray


def path(stamp,y,speed):
    out=Trajectory();out.header.stamp=stamp;out.header.frame_id='map'
    for x in range(-5,81):
        p=TrajectoryPoint();p.pose.position.x=float(x);p.pose.position.y=y
        p.pose.orientation.w=1.;p.longitudinal_velocity_mps=speed
        out.points.append(p)
    return out


@pytest.mark.parametrize("mode,side,tracking_mode,path_y,hold", [
    ("FREE_RUN",0,"FREE_RUN",0.,True),
    ("OVERTAKE",-1,"OVERTAKE_RIGHT",-2.,True),
    ("AVOID",1,"OVERTAKE_LEFT",2.,True),
    ("OVERTAKE",-1,"OVERTAKE_RIGHT",-2.,False),
])
def test_expiry_keeps_trajectory_and_speed(tmp_path,monkeypatch,mode,side,tracking_mode,path_y,hold):
    monkeypatch.setenv('ROS_DOMAIN_ID',str(100+os.getpid()%100))
    rclpy.init();probe=rclpy.create_node('reference_delay_probe')
    binary=Path(os.environ.get('CMA_BINARY',str(Path(get_package_prefix('cma_pure_pursuit'))/'lib/cma_pure_pursuit/cma_pure_pursuit')))
    args=[str(binary),'--ros-args','-r','__node:=delay_cma_test']
    params={'hold_last_direct_trajectory_enabled':hold,
      'use_atomic_direct_trajectory_command':True,'recovery_service_enabled':True,
      'odometry_timeout_sec':.25,'trajectory_timeout_sec':.25,'overtake_mode_timeout_sec':.25,
      'lookahead_gain':.2,'lookahead_min_distance':2.,'curvature_lookahead_min_distance':2.,
      'continuous_preview_interpolation_enabled':True,'wheel_base':1.087,
      'corner_speed_retention':1.,'steering_command_to_tire_angle_ratio':.6,
      'steering_tire_angle_gain':1.,'hard_steering_angle_limit_rad':.5235987756,
      'hard_steering_rate_limit_radps':4.,'maximum_tire_steering_angle_rad':.3141592654,
      'longitudinal_acceleration_limit':2.,'longitudinal_deceleration_limit':2.,
      'steering_demand_acceleration_hold_enabled':False,'wall_edge_tracking_speed_enabled':False,
      'steering_tracking_speed_gate_enabled':False,'curvature_feedforward_enabled':False}
    for k,v in params.items():args+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
    remaps={'input/kinematics':'/delay/odom','input/direct_trajectory_command':'/delay/direct',
      'input/delay_reference':'/delay/reference','input/delay_vehicles':'/delay/vehicles',
      'output/control_cmd':'/delay/control','/pure_pursuit/debug':'/delay/debug'}
    for a,b in remaps.items():args+=['-r',a+':='+b]
    log=(tmp_path/'cma.log').open('w');process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
    commands=[];debug=[]
    probe.create_subscription(AckermannControlCommand,'/delay/control',commands.append,100)
    probe.create_subscription(String,'/delay/debug',lambda m:debug.append(json.loads(re.sub(r'\b(-?inf|nan)\b','null',m.data))),100)
    odom=probe.create_publisher(Odometry,'/delay/odom',1)
    direct=probe.create_publisher(StateLatticeDirectTrajectory,'/delay/direct',1)
    reference=probe.create_publisher(Trajectory,'/delay/reference',1)
    vehicles=probe.create_publisher(V2XVehiclePositionArray,'/delay/vehicles',10)
    client=probe.create_client(SetBool,'/delay_cma_test/set_recovery_active')
    state=dict(direct=False,old=False,odom=True,speed=4.,y=path_y,ego_y=path_y,x=0.,
               gradient=.1,empty=False,leader=None,leader_started=0.)
    generation=0
    def publish():
        nonlocal generation
        now=probe.get_clock().now();sec=now.nanoseconds*1e-9;stamp=now.to_msg()
        if state['odom']:
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='map'
            m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=4.
            m.pose.pose.position.x=state['x'];m.pose.pose.position.y=state['ego_y'];odom.publish(m)
        reference.publish(path(stamp,2.,8.))
        if state['direct']:
            m=StateLatticeDirectTrajectory();m.header.stamp=stamp;m.header.frame_id='map'
            generation+=1;m.generation=generation;m.owner_commit_id=1;m.geometry_revision=generation
            m.mode=mode;m.corridor_side=side;m.corridor_clearance_profile=side!=0
            m.valid_until_sec=sec+(.15 if not state['old'] else -.2)
            m.trajectory=path(stamp,state['y'],state['speed'])
            for point in m.trajectory.points:
                point.longitudinal_velocity_mps+=max(0.,point.pose.position.x)*state['gradient']
            if state['empty']:m.trajectory.points.clear()
            direct.publish(m)
        array=V2XVehiclePositionArray()
        for vid,x,y in [('self',0.,0.),('rear',-2.,2.),('adjacent',3.,6.)]:
            v=V2XVehiclePosition();v.vehicle_id=vid;v.header.stamp=stamp;v.position.x=x;v.position.y=y;array.vehicles.append(v)
        if state['leader'] is not None:
            v=V2XVehiclePosition();v.vehicle_id='lead';v.header.stamp=stamp
            v.position.x=10.+state['leader']*(time.monotonic()-state['leader_started'])
            v.position.y=2.;array.vehicles.append(v)
        vehicles.publish(array)
    def spin(duration):
        end=time.monotonic()+duration;next_pub=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_pub:publish();next_pub=time.monotonic()+.02
            rclpy.spin_once(probe,timeout_sec=.005)
        assert process.poll() is None,(tmp_path/'cma.log').read_text()[-2000:]
    def recent(duration=.2):
        commands.clear();debug.clear();spin(duration);assert commands
        return commands[:],debug[:]
    def pause(value):
        future=client.call_async(SetBool.Request(data=value));end=time.monotonic()+2
        while not future.done() and time.monotonic()<end:spin(.02)
        assert future.done() and future.result().success
    try:
        spin(.8);assert client.service_is_ready()
        cs,_=recent();assert all(c.longitudinal.speed==0 for c in cs)
        state['direct']=True;spin(.5)
        cs,ds=recent();assert all(abs(c.longitudinal.speed-4.)<.01 for c in cs)
        state['direct']=False;spin(.5)
        cs,ds=recent()
        if not hold:
            assert all(c.longitudinal.speed==0 for c in cs)
            return
        assert ds and all(d['direct_trajectory_held'] for d in ds)
        generation=ds[-1]['direct_trajectory_generation']
        geometry=ds[-1]['direct_geometry_revision']
        assert all(d['overtake_mode']==tracking_mode and d['overtake_mode_fresh'] for d in ds)
        assert all(d['corridor_clearance_profile']==(side!=0) for d in ds)
        assert all(abs(c.longitudinal.speed-4.)<.01 and abs(c.lateral.steering_tire_angle)<.01 for c in cs)
        # A stopped vehicle on the other Reference must not replace MPPI's speed.
        state['leader']=0.;state['leader_started']=time.monotonic();spin(.5)
        cs,ds=recent();assert all(abs(c.longitudinal.speed-4.)<.01 for c in cs)
        # Holding follows the spatial profile, not a frozen speed or steering command.
        state['x']=10.;spin(.15)
        cs,ds=recent();assert all(abs(c.longitudinal.speed-5.)<.01 for c in cs)
        assert all(d['direct_trajectory_generation']==generation and d['direct_geometry_revision']==geometry for d in ds)
        state['ego_y']=path_y-1.;spin(.3)
        cs,_=recent();assert all(c.lateral.steering_tire_angle>.05 for c in cs)
        state['ego_y']=path_y;state['x']=80.;spin(.3)
        cs,ds=recent();assert all(abs(c.longitudinal.speed-12.)<.01 for c in cs)
        assert all(d['lookahead_endpoint_fallback'] for d in ds)
        # The planner can replace the held path and speed normally.
        state.update(direct=True,speed=7.,gradient=0.,x=0.,y=path_y-2.)
        spin(.5);cs,ds=recent()
        assert ds and all(not d['direct_trajectory_held'] for d in ds)
        assert all(abs(c.longitudinal.speed-7.)<.01 and c.lateral.steering_tire_angle<-.05 for c in cs)
        state['direct']=False;spin(.4)
        cs,ds=recent();assert all(abs(c.longitudinal.speed-7.)<.01 for c in cs)
        state['odom']=False;spin(.4)
        cs,_=recent();assert all(c.longitudinal.speed==0 and c.lateral.steering_tire_angle==0 for c in cs)
        state['odom']=True;spin(.1);pause(True)
        cs,_=recent();assert all(c.longitudinal.speed==0 for c in cs)
        pause(False);spin(.1)
        cs,_=recent();assert all(c.longitudinal.speed==0 and c.longitudinal.acceleration<0 for c in cs)
        state.update(direct=True,speed=3.);spin(.4)
        cs,_=recent();assert all(abs(c.longitudinal.speed-3.)<.01 for c in cs)
        state['empty']=True;spin(.4)
        cs,_=recent();assert all(c.longitudinal.speed==0 for c in cs)
    finally:
        process.terminate();process.wait(timeout=10);log.close()
        probe.destroy_node();rclpy.shutdown()
