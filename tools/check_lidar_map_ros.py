"""Finite isolated ROS smoke, synthetic scan/wheel inputs, no vehicle commands."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time


def main() -> None:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(exist_ok=False,parents=True)
    nodes=[(0,-3),(12,-3),(12,4),(0,4),(0,-3)]
    xml='<osm>'+''.join(f'<node id="{i}"><tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/></node>' for i,(x,y) in enumerate(nodes))
    xml+='<way id="10">'+''.join(f'<nd ref="{i}"/>' for i in range(5))+'</way><relation><member type="way" role="left" ref="10"/></relation></osm>'
    mapfile=args.output/'map.osm';mapfile.write_text(xml)
    rclpy.init();sensor=rclpy.create_node('awsim_d1');wheel=rclpy.create_node('time_path_controller')
    scan_pub=sensor.create_publisher(LaserScan,'/sensing/lidar/scan',qos_profile_sensor_data)
    clock_pub=sensor.create_publisher(Clock,'/clock',10)
    wheel_pub=wheel.create_publisher(Odometry,'/time_path/wheel_odometry',10)
    seed_pub=wheel.create_publisher(PoseWithCovarianceStamped,'/time_path/localization/initialpose',10)
    statuses=[]; aligned=[]; transforms=[]
    wheel.create_subscription(String,'/time_path/localization/status',lambda m:statuses.append(json.loads(m.data)),10)
    wheel.create_subscription(LaserScan,'/time_path/localization/scan',aligned.append,qos_profile_sensor_data)
    wheel.create_subscription(TFMessage,'/tf',lambda m:transforms.extend(m.transforms),10)
    t=1.; step=0; next_pub=0.
    def cycle(duration: float, *, scans: bool = True, clock_running: bool = True, bad_frame: bool = False) -> None:
        nonlocal t,step,next_pub
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_pub:
                next_pub=now+.05
                if clock_running:t+=.05
                seconds=int(t);nanos=round((t-seconds)*1e9)
                clock=Clock();clock.clock.sec=seconds;clock.clock.nanosec=nanos;clock_pub.publish(clock)
                odom=Odometry();odom.header.frame_id='time_wheel_odom';odom.child_frame_id='base_link'
                odom.header.stamp=clock.clock;odom.pose.pose.orientation.w=1.
                if clock_running:wheel_pub.publish(odom)
                if scans and clock_running:
                    scan=LaserScan();scan.header.stamp=clock.clock;scan.header.frame_id='incorrect' if bad_frame else 'lidar'
                    scan.angle_min=-1.5;scan.angle_increment=3./749;scan.angle_max=1.5
                    scan.range_min=0.;scan.range_max=25.;scan.scan_time=.05
                    ranges=[]
                    for i in range(750):
                        a=scan.angle_min+i*scan.angle_increment;c,s=math.cos(a),math.sin(a)
                        hits=[(12-4.65)/c]
                        if s>1e-9:hits.append((4-1)/s)
                        elif s < -1e-9:hits.append((-3-1)/s)
                        ranges.append(float(min(v for v in hits if v>0)))
                    scan.ranges=ranges;scan_pub.publish(scan)
                step+=1
            rclpy.spin_once(wheel,timeout_sec=.005);rclpy.spin_once(sensor,timeout_sec=.001)
    log=(args.output/'node.log').open('x')
    process=subprocess.Popen(['ros2','run','aic_e2e_runtime','lidar_map_localization_node',
        '--map',str(mapfile),'--initial-pose','3.2','.8','.02','--output',str(args.output/'status.jsonl')],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        cycle(5.)
        assert process.poll() is None,(args.output/'node.log').read_text()
        assert statuses and statuses[-1]['valid'],statuses[-1:] or 'No status'
        assert len(aligned)>5 and all(s.header.frame_id=='time_localized_lidar' for s in aligned)
        map_transforms=[m for m in transforms if m.header.frame_id=='map']
        assert map_transforms and all(m.child_frame_id=='time_wheel_odom' for m in map_transforms)
        last=map_transforms[-1].transform.translation
        assert abs(last.x-3)<.04 and abs(last.y-1)<.04,(last.x,last.y)
        subs=wheel.get_subscriber_names_and_types_by_node('time_lidar_map_localizer','/')
        assert {name for name,_ in subs}=={'/clock','/sensing/lidar/scan','/time_path/wheel_odometry','/time_path/localization/initialpose'},subs
        assert not wheel.get_publishers_info_by_topic('/control/command/control_cmd')
        cycle(1.2,scans=False)
        assert not statuses[-1]['valid']
        count=len(aligned);cycle(.4,scans=False);assert len(aligned)==count
        seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.pose.pose.position.x=3.;seed.pose.pose.position.y=1.;seed.pose.pose.orientation.w=1.
        seed_pub.publish(seed);cycle(1.5);assert statuses[-1]['valid'],statuses[-1]
        cycle(.6,bad_frame=True);assert not statuses[-1]['valid']
        seed_pub.publish(seed);cycle(1.5);assert statuses[-1]['valid']
        cycle(.8,clock_running=False);assert not statuses[-1]['valid'] and statuses[-1]['mode']=='CLOCK_STALE'
        result=dict(status='PASS',aligned_scans=len(aligned),subscriptions=subs,estimated_xy=[last.x,last.y],
                    gnss_imu_pose_tf_publishers=0,vehicle_command_publishers=0,
                    checks=['map_to_wheel_transform','fixed_mount_alias','scan_timeout','bad_scan_frame','clock_pause','manual_reinitialize'])
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        import os
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        log.close();wheel.destroy_node();sensor.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
