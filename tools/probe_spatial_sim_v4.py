"""Finite, NO-CONTROL probe inside the selected network-none AWSIM container.

No model/checkpoint/Dataset reader, no publisher/mode service. Reports graph,
actual source headers/geometry and timing, not parity or permission. Sensor
payloads are not saved. An unresolved binding MUST NOT be patched with defaults.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import time


def verify_isolation(inspection: list, expected_id: str) -> None:
    c, = inspection
    h=c['HostConfig']
    if c['Id']!=expected_id or h['NetworkMode']!='none' or h['Privileged'] or h['PidMode'] or h['IpcMode']!='private':
        raise ValueError('BLOCKED_SIM_ISOLATION')
    if h['Devices'] or h['CapAdd'] or h['CapDrop']!=['ALL']:
        raise ValueError('BLOCKED_DEVICE_OR_CAPABILITY')
    if c['Config']['Labels'].get('codex.task')!='spatial_sim_e2e_20260906':
        raise ValueError('NOT_OWNED_CONTAINER')
    if {m['Destination'] for m in c['Mounts']}!={'/sim','/evidence'}:
        raise ValueError('UNEXPECTED_MOUNT')
    if any(m['RW'] for m in c['Mounts'] if m['Destination']=='/sim'):
        raise ValueError('SIMULATOR_WRITABLE')
    if sorted(p.name for p in Path('/sys/class/net').iterdir())!=['lo']:
        raise ValueError('NONLOCAL_INTERFACE')
    routes=subprocess.run(['ip','route'],capture_output=True,text=True,check=True).stdout
    if routes.strip(): raise ValueError('EXTERNAL_ROUTE')
    for prefix in ('ttyUSB','ttyACM','can','serial'):
        if any(p.name.startswith(prefix) for p in Path('/dev').iterdir()):
            raise ValueError('HARDWARE_DEVICE')


def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--inspection',type=Path,required=True)
    ap.add_argument('--container-id',required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seconds',type=float,default=12.)
    args=ap.parse_args()
    if not 0<args.seconds<=20: raise ValueError('FINITE_PROBE_REQUIRED')
    if args.output.exists(): raise FileExistsError(args.output)
    blob=args.inspection.read_bytes()
    verify_isolation(json.loads(blob),args.container_id)
    if os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_LOCALHOST_ONLY')!='0':
        raise ValueError('FROZEN_ROS_ENVIRONMENT')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport,SteeringReport
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool,String
    rclpy.init(); node=None
    counts=Counter(); rows=[]; stamp_rows={}; first_error=None; graph={}
    start=time.monotonic_ns()
    def callback(role: str, msg: object) -> None:
        now=time.monotonic_ns(); counts[role]+=1
        source=getattr(getattr(msg,'header',None),'stamp',getattr(msg,'stamp',None))
        if role=='clock': source=msg.clock
        ns=None if source is None else int(source.sec)*1_000_000_000+int(source.nanosec)
        stamp_rows.setdefault(role,[])
        if len(stamp_rows[role])<256: stamp_rows[role].append([ns,now])
        if counts[role]>4: return
        row=dict(role=role,received_monotonic_ns=now,source_stamp_ns=ns,
                 source_clock_domain='ROS_SIMULATOR; provenance beyond simulator timestamp UNKNOWN',
                 frame=getattr(getattr(msg,'header',None),'frame_id',None))
        if role=='camera':
            row.update(height=msg.height,width=msg.width,encoding=msg.encoding,step=msg.step,payload_bytes=len(msg.data))
        elif role=='lidar':
            row.update(beams=len(msg.ranges),angle_min=msg.angle_min,angle_max=msg.angle_max,
                       angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max,
                       scan_time=msg.scan_time,time_increment=msg.time_increment)
        elif role=='velocity':
            row.update(v_long_mps=msg.longitudinal_velocity,v_lat_mps=msg.lateral_velocity,yaw_rate_rps=msg.heading_rate)
        elif role=='steering': row['steering_tire_angle_rad']=msg.steering_tire_angle
        elif role=='pose':
            row.update(position=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z],
                       orientation=[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        elif role in ('collision','state'): row['data']=msg.data
        rows.append(row)
    try:
        node=rclpy.create_node('v4_sim_no_control_probe',enable_rosout=False,start_parameter_services=False)
        roles={'camera':('/sensing/camera/image_raw',Image),'lidar':('/sensing/lidar/scan',LaserScan),
               'velocity':('/vehicle/status/velocity_status',VelocityReport),'steering':('/vehicle/status/steering_status',SteeringReport),
               'pose':('/awsim/ground_truth/vehicle/pose',PoseStamped),'clock':('/clock',Clock),
               'collision':('/awsim/ground_truth/on_collision',Bool),'state':('/awsim/state',String)}
        subscriptions=[node.create_subscription(typ,topic,lambda m,r=role:callback(r,m),qos_profile_sensor_data)
                       for role,(topic,typ) in roles.items()]
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=min(.05,max(0,deadline-time.monotonic())))
        for topic,types in node.get_topic_names_and_types():
            if topic in [v[0] for v in roles.values()]+['/control/command/control_cmd','/awsim/control_mode_request_topic']:
                def endpoint(e: object) -> dict:
                    return dict(node_name=e.node_name,namespace=e.node_namespace,type=e.topic_type,
                                gid=list(e.endpoint_gid),qos=str(e.qos_profile))
                graph[topic]=dict(types=types,publishers=[endpoint(e) for e in node.get_publishers_info_by_topic(topic)],
                                 subscribers=[endpoint(e) for e in node.get_subscriptions_info_by_topic(topic)])
    except Exception as exc:
        first_error=repr(exc)
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
    result=dict(scope='SIM_NO_CONTROL_METADATA_PROBE',code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                inspection_sha256=hashlib.sha256(blob).hexdigest(),container_id=args.container_id,
                source_commit=os.environ.get('V4_SOURCE_COMMIT','UNKNOWN'),counts=dict(counts),first_messages=rows,
                bounded_timestamps=stamp_rows,graph=graph,first_error=first_error,
                wall_seconds=(time.monotonic_ns()-start)*1e-9,
                control_publish_calls=0,mode_calls=0,forward_calls=0,sensor_snapshots_saved=0,
                dependencies={k:importlib.metadata.version(k) for k in ['numpy','torch','scipy']},
                physical_frames_verified=False,training_geometry_parity='UNKNOWN',
                warning='Graph metadata and headers are NOT evidence of safe motion or model parity')
    with args.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({'counts':dict(counts),'error':first_error,'output':str(args.output)}))
    return 1 if first_error else 0


if __name__=='__main__': raise SystemExit(main())
