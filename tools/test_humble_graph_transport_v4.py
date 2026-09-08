"""Finite isolated ROS test, /fixture only. No AWSIM, model, or control topics."""
import json
import time
import rclpy
from rclpy.node import Node
from aic_transfuser_lite.runtime.shadow_ros2_transport_v4 import ShadowROS2Transport, ros2_types
from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding


def main():
    rclpy.init(args=[])
    source=Node('external',namespace='/fixture',enable_rosout=False,start_parameter_services=False)
    sink=Node('shadow',namespace='/fixture',enable_rosout=False,start_parameter_services=False)
    types=ros2_types();topics={r:'/fixture/'+r for r in types}
    pubs={r:source.create_publisher(t,topics[r],10) for r,t in types.items()}
    events=[];resets=[];transport=None
    try:
        deadline=time.monotonic()+4
        while not all(len(sink.get_publishers_info_by_topic(t))==1 for t in topics.values()):
            if time.monotonic()>deadline: raise RuntimeError('GRAPH_DISCOVERY_TIMEOUT')
            rclpy.spin_once(sink,timeout_sec=.02)
        binding=ControllerCommandBinding(topics['command'],'/fixture/external','nominal','SYNTHETIC_ONLY',
                                        'TIRE_RAD_TARGET_MPS_ACCEL_MPS2')
        transport=ShadowROS2Transport(sink,types,topics,dict.fromkeys(types,10),binding,
                    dict.fromkeys(types,'/fixture/external'),lambda *a:None,
                    lambda *a:resets.append(a),events.append,clock_id='fixture',monotonic_id='fixture')
        clock=types['clock']();clock.clock.sec=2
        command=types['command']();command.stamp.sec=1;command.longitudinal.speed=.2
        deadline=time.monotonic()+4
        while not transport.command_snapshot():
            if time.monotonic()>deadline: raise RuntimeError('COMMAND_RECEIPT_TIMEOUT')
            pubs['clock'].publish(clock);pubs['command'].publish(command)
            rclpy.spin_once(sink,timeout_sec=.02)
        received=len(transport.command_snapshot())
        extra=source.create_publisher(types['command'],topics['command'],10)
        deadline=time.monotonic()+4
        while transport.fault is None:
            if time.monotonic()>deadline: raise RuntimeError('DUPLICATE_NOT_DETECTED')
            rclpy.spin_once(sink,timeout_sec=.02)
        assert transport.fault=='PUBLISHER_COUNT:command' and not transport.command_snapshot()
        source.destroy_publisher(extra)
        assert not transport.check_graph()  # latched, no silent re-admission
        print(json.dumps(dict(result='PASS',synthetic_only=True,received=received,
                              reason=transport.fault,reset_count=len(resets))))
    finally:
        if transport: transport.close()
        sink.destroy_node();source.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
