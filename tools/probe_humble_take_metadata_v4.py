"""Isolated synthetic ROS test; only /fixture/v4_metadata String publications."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init(args=[])
    node=Node('synthetic_metadata_probe',enable_rosout=False,start_parameter_services=False)
    sub=node.create_subscription(String,'/fixture/v4_metadata',lambda msg:None,10)
    pub=node.create_publisher(String,'/fixture/v4_metadata',10)
    started=time.monotonic()
    try:
        while time.monotonic()-started<8:
            pub.publish(String(data='SYNTHETIC_ONLY'))
            with sub.handle:
                taken=sub.handle.take_message(sub.msg_type,sub.raw)
            if taken is not None:
                print(json.dumps({'metadata_type':str(type(taken[1])), 'metadata':repr(taken[1]),
                                  'graph_gids':[bytes(e.endpoint_gid).hex() for e in node.get_publishers_info_by_topic('/fixture/v4_metadata')]}))
                return
            time.sleep(.05)
        raise RuntimeError('SYNTHETIC_RECEIPT_TIMEOUT')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
