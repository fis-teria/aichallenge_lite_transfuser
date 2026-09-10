"""Finite synthetic ROS test with installed existing PP; no AWSIM or actuation.

Run in a network-isolated container with official ROS/PP overlays sourced.
All created processes are owned and terminated. Success requires a real PP
command and its expiry stop, not merely an Adapter result.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from nav_msgs.msg import Odometry
    from autoware_auto_planning_msgs.msg import Trajectory
    rclpy.init()
    sensor=Node('awsim_d1')
    planner=Node('v4_slam_pose_adapter')
    slam=Node('cartographer',namespace='/cartographer_v4')
    clock=sensor.create_publisher(Clock,'/clock',10)
    velocity=sensor.create_publisher(VelocityReport,'/vehicle/status/velocity_status',10)
    steering=sensor.create_publisher(SteeringReport,'/vehicle/status/steering_status',10)
    pose=slam.create_publisher(PoseStamped,'/cartographer_v4/extrapolated_pose',10)
    plan=planner.create_publisher(String,'/shadow/v4/plan_record',10)
    commands=[]; trajectories=[]; odoms=[]; statuses=[]
    sensor.create_subscription(AckermannControlCommand,'/shadow/v4/pp/output/control_cmd',
        lambda m:commands.append((time.monotonic(),m.longitudinal.acceleration)),10)
    sensor.create_subscription(Trajectory,'/shadow/v4/pp/trajectory',
        lambda m:trajectories.append((time.monotonic(),len(m.points))),10)
    sensor.create_subscription(Odometry,'/shadow/v4/pp/odometry',
        lambda m:odoms.append(m.pose.pose.position.x),10)
    sensor.create_subscription(String,'/shadow/v4/pp/status',
        lambda m:statuses.append(json.loads(m.data)),10)
    process=subprocess.Popen(['ros2','launch','aic_e2e_runtime','v4_pp_shadow_connection.launch.py'],
                             start_new_session=True)
    began=time.monotonic(); phase_end=began+7
    try:
        while time.monotonic()-began<10:
            elapsed=time.monotonic()-began; t=10.+elapsed
            stamp=round(t*1e9)
            c=Clock(); c.clock.sec=stamp//10**9; c.clock.nanosec=stamp%10**9; clock.publish(c)
            v=VelocityReport();v.header.stamp=c.clock;v.header.frame_id='base_link';velocity.publish(v)
            s=SteeringReport();s.stamp=c.clock;steering.publish(s)
            p=PoseStamped();p.header.stamp=c.clock;p.header.frame_id='cartographer_v4_local'
            p.pose.position.x=1.1649999618530273;p.pose.orientation.w=1.;pose.publish(p)
            if time.monotonic()<phase_end:
                r=dict(event='PLAN',session_id='ROS_SYNTHETIC',clock='AWSIM_ROS',epoch='0',
                    source='FIXED_V4_UNCORRECTED',source_s=t,generated_monotonic_s=time.monotonic(),
                    expires_s=t,accepted=False,frame='base_link',reference_point='BASE_LINK_ORIGIN',
                    output_id=str(stamp),raw_xy_m=[[i*.1,0.] for i in range(20)],
                    observation_pose_xyyaw=[0.,0.,0.],observation_pose_evidence='SYNTHETIC_ONLY',
                    pose_frame='cartographer_v4_local',transport_kind='DIAGNOSTIC_NOT_CONTROL')
                msg=String();msg.data=json.dumps(r);plan.publish(msg)
            for n in (sensor,planner,slam):rclpy.spin_once(n,timeout_sec=.002)
            if process.poll() is not None: raise RuntimeError('LAUNCH_EXIT')
            time.sleep(.04)
        result=dict(synthetic_only=True,command_count=len(commands),
            positive_commands=sum(a>0 for ts,a in commands if ts<phase_end),
            expired_stop_commands=sum(a<0 for ts,a in commands if ts>phase_end+1),
            populated_trajectories=sum(n>0 for ts,n in trajectories),
            expired_empty_trajectories=sum(n==0 for ts,n in trajectories if ts>phase_end+1),
            midpoint_x=odoms[-1] if odoms else None,
            actuator_publishers=len(sensor.get_publishers_info_by_topic('/control/command/control_cmd')),
            status_reasons=sorted(set(s['reason'] for s in statuses)))
        print('CONNECTION_RESULT '+json.dumps(result),flush=True)
        assert result['positive_commands']>0 and result['expired_stop_commands']>0
        assert result['populated_trajectories']>0 and result['expired_empty_trajectories']>0
        assert abs(result['midpoint_x']-.0595)<1e-9
        assert result['actuator_publishers']==0
    finally:
        # ros2 launch forwards SIGINT to children itself; signalling the entire
        # group here would deliver it twice and interrupt middleware teardown.
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for n in (sensor,planner,slam):n.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()
