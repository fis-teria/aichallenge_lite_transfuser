"""Synthetic ROS-only smoke test. Run in --network none container, no AWSIM.

No sensor assets, model, command publisher, actuator or global localization.
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from autoware_auto_vehicle_msgs.msg import VelocityReport
from aic_e2e_runtime.local_odometry_node_v4 import create_local_odometry_node


def main() -> None:
    rclpy.init()
    fixture=Node('awsim_d1')  # Artificial source identity in an isolated DDS domain.
    local=create_local_odometry_node()
    executor=rclpy.get_global_executor();executor.add_node(local);executor.add_node(fixture)
    outputs=[]
    subscription=fixture.create_subscription(Odometry,'/v4/local_odometry',outputs.append,10)
    clock=fixture.create_publisher(Clock,'/clock',10)
    velocity=fixture.create_publisher(VelocityReport,'/vehicle/status/velocity_status',10)

    def spin(seconds: float) -> None:
        until=time.monotonic()+seconds
        while time.monotonic()<until: executor.spin_once(timeout_sec=.01)

    try:
        spin(.5)
        for i in range(11):
            c=Clock();c.clock.sec=1;c.clock.nanosec=i*35_000_000
            clock.publish(c);spin(.02)
            v=VelocityReport();v.header.stamp=c.clock;v.header.frame_id='base_link'
            v.longitudinal_velocity=2.;v.heading_rate=1.
            velocity.publish(v);spin(.04)
        assert len(outputs)==11, len(outputs)
        p=outputs[-1].pose.pose.position
        assert abs(p.x-2*math.sin(.35))<1e-6
        assert abs(p.y-2*(1-math.cos(.35)))<1e-6
        assert outputs[-1].child_frame_id=='v4_base_link'
        velocity.publish(v);spin(.05)
        assert len(outputs)==11
        print(json.dumps(dict(synthetic=True,count=len(outputs),x_m=p.x,y_m=p.y,
                              gnss_imu_inputs=False,control_publish=False)))
    finally:
        executor.remove_node(local);executor.remove_node(fixture)
        local.destroy_node();fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
