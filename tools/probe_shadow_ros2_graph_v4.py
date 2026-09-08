"""Finite input-only live ROS graph audit; no model or control endpoints."""
import json
import time
from pathlib import Path


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    types={'sensor_msgs/msg/Image':Image,'sensor_msgs/msg/LaserScan':LaserScan,
           'autoware_auto_vehicle_msgs/msg/VelocityReport':VelocityReport,
           'autoware_auto_vehicle_msgs/msg/SteeringReport':SteeringReport,
           'autoware_auto_control_msgs/msg/AckermannControlCommand':AckermannControlCommand,
           'nav_msgs/msg/Odometry':Odometry,'rosgraph_msgs/msg/Clock':Clock}
    rclpy.init(args=[])
    node=Node('v4_input_graph_probe',enable_rosout=False,start_parameter_services=False,
              use_global_arguments=False)
    result={'kind':'ACTUAL_ROS_GRAPH_AND_RECEIPT','control_publish':False,'topics':{},'receipts':{}}
    subscriptions={};start=time.monotonic()
    def receive(topic,msg,info):
        d=result['receipts'].setdefault(topic,{'count':0,'gids':[]})
        d['count']+=1
        gid=bytes(info.publisher_gid).hex()
        if gid not in d['gids']: d['gids'].append(gid)
        stamp=getattr(getattr(msg,'header',None),'stamp',None) or getattr(msg,'stamp',None) or getattr(msg,'clock',None)
        if stamp is not None: d['last_stamp_ns']=stamp.sec*1_000_000_000+stamp.nanosec
        d['last_received_monotonic_ns']=time.monotonic_ns()
        if hasattr(msg,'header'): d['frame']=msg.header.frame_id
    def make_callback(topic):
        def cb(msg,info): receive(topic,msg,info)
        return cb
    try:
        while time.monotonic()-start<25:
            for topic,names in node.get_topic_names_and_types():
                if len(names)!=1 or names[0] not in types: continue
                endpoints=node.get_publishers_info_by_topic(topic)
                result['topics'][topic]={'type':names[0],'publishers':[
                    dict(node=e.node_name,namespace=e.node_namespace,gid=bytes(e.endpoint_gid).hex(),
                         reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability),
                         history=str(e.qos_profile.history),depth=e.qos_profile.depth) for e in endpoints]}
                if topic not in subscriptions:
                    subscriptions[topic]=node.create_subscription(types[names[0]],topic,make_callback(topic),qos_profile_sensor_data)
            rclpy.spin_once(node,timeout_sec=.1)
        result['wall_s']=time.monotonic()-start
    finally:
        Path('/evidence/graph_probe.json').write_text(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
