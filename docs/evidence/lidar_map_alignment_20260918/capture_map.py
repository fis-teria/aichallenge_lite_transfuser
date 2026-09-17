"""Isolated official map publishers only; no simulator or vehicle commands."""
import json
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

out=Path('/proof')
map_path='/aichallenge/workspace/install/aichallenge_submit_launch/share/aichallenge_submit_launch/map/lanelet2_map.osm'
assert Path(map_path).exists(),map_path
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
rclpy.init(); node=rclpy.create_node('isolated_map_audit')
messages=[]
sub=node.create_subscription(MarkerArray,'/map/vector_map_marker',messages.append,qos)
processes=[];logs=[]
try:
    for exe in ['lanelet2_map_loader','lanelet2_map_visualization']:
        log=(out/(exe+'.log')).open('x');logs.append(log)
        args=['ros2','run','map_loader',exe,'--ros-args','-p','lanelet2_map_path:='+map_path,
              '-p','lanelet2_map_projector_type:=MGRS','-p','center_line_resolution:=5.0',
              '-r','output/lanelet2_map:=/map/vector_map','-r','input/lanelet2_map:=/map/vector_map',
              '-r','output/lanelet2_map_marker:=/map/vector_map_marker']
        processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    deadline=time.monotonic()+25
    while not messages and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.2)
    assert messages,'Map marker timeout'
    records=[]
    for m in messages[-1].markers:
        records.append(dict(ns=m.ns,id=m.id,type=m.type,frame=m.header.frame_id,
                            pose=dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z,
                                      q=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]),
                            xyz=[[p.x,p.y,p.z] for p in m.points]))
    (out/'markers.json').write_text(json.dumps(records))
    print(json.dumps(dict(status='PASS',markers=len(records),namespaces=sorted({m['ns'] for m in records}),
                         vehicle_command_publishers=len(node.get_publishers_info_by_topic('/control/command/control_cmd')))))
finally:
    import os
    for p in processes:
        if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
    for p in processes:
        try:p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=4)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
