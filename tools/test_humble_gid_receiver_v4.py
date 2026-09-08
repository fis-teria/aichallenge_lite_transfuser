"""Synthetic ROS publishers on /fixture only, isolated container required."""
import json
import select
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main():
    rclpy.init(args=[])
    node=Node('fixture_gid_sources',enable_rosout=False,start_parameter_services=False)
    pubs=[node.create_publisher(String,'/fixture/gid',10) for _ in range(2)]
    child=subprocess.Popen([sys.argv[1],'command','/fixture/gid','std_msgs/msg/String'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    received={};started=time.monotonic()
    try:
        while time.monotonic()-started<12 and len(received)<2:
            for i,pub in enumerate(pubs): pub.publish(String(data='fixture'+str(i)))
            if not select.select([child.stdout],[],[],.05)[0]: continue
            line=child.stdout.readline(2_097_408)
            if not line: raise RuntimeError('RECEIVER_EOF:'+child.stderr.read().decode())
            role,gid,ns,payload=line.decode().strip().split(' ')
            msg=deserialize_message(bytes.fromhex(payload),String)
            assert role=='command' and 0<int(ns)<=time.monotonic_ns()
            received[msg.data]=gid
        graph={bytes(e.endpoint_gid).hex() for e in node.get_publishers_info_by_topic('/fixture/gid')}
        print(json.dumps(dict(observed=received,graph_gids=sorted(graph))),flush=True)
        assert len(received)==2 and len(set(received.values()))==2 and set(received.values())==graph
        print(json.dumps(dict(result='PASS',synthetic_only=True,received=received,graph_gids=sorted(graph))))
    finally:
        child.terminate()
        try: child.wait(timeout=2)
        except subprocess.TimeoutExpired: child.kill();child.wait(timeout=2)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
