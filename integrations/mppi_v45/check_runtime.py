"""Read-only ROS graph and parameter check for the pinned V45 teacher."""
import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

rclpy.init()
probe=Node('v45_collection_graph_check')
topic='/collection/lidar_v2x/vehicle_positions'
expected={'reference_space_mppi_planner','simple_trajectory_generator','mppi_recovery_controller'}
try:
    deadline=time.monotonic()+35
    while time.monotonic()<deadline:
        rclpy.spin_once(probe,timeout_sec=.1)
        subscriptions=probe.get_subscriptions_info_by_topic(topic)
        publishers=probe.get_publishers_info_by_topic(topic)
        if expected.issubset({s.node_name for s in subscriptions}) and len(publishers)==1:
            break
    assert expected.issubset({s.node_name for s in subscriptions}), json.dumps({
        'nodes': probe.get_node_names_and_namespaces(),
        'dedicated_consumers': [s.node_name for s in subscriptions],
        'dedicated_publishers': [p.node_name for p in publishers],
        'native_consumers': [s.node_name for s in probe.get_subscriptions_info_by_topic('/v2x/vehicle_positions')]})
    assert len(publishers)==1 and publishers[0].node_name=='lidar_v2x'
    assert not expected.intersection(s.node_name for s in probe.get_subscriptions_info_by_topic('/v2x/vehicle_positions'))
    checked={}
    margins=dict(obstacle_longitudinal_inflation_m=1.10,obstacle_lateral_inflation_m=1.15,
        clearance_target_m=.35,**{'brain.footprint_radius_m':.65,'brain.footprint_front_m':1.06,'brain.footprint_rear_m':1.10})
    for name,parameters in [('reference_space_mppi_planner',margins),
            ('simple_trajectory_generator',{'execution_profile.max_speed_mps':float(os.environ['TEACHER_SPEED_CAP_MPS'])})]:
        endpoint=next(s for s in subscriptions if s.node_name==name)
        fqn=endpoint.node_namespace.rstrip('/')+'/'+name
        client=probe.create_client(GetParameters,fqn+'/get_parameters')
        assert client.wait_for_service(timeout_sec=5),fqn
        request=GetParameters.Request();request.names=list(parameters)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(probe,future,timeout_sec=5)
        assert future.done() and future.result() is not None
        values={k:v.double_value for k,v in zip(parameters,future.result().values)}
        assert all(math.isclose(values[k],v,abs_tol=1e-8) for k,v in parameters.items()),(fqn,values)
        checked[fqn]=values
    client=probe.create_client(GetParameters, '/lidar_v2x/get_parameters')
    assert client.wait_for_service(timeout_sec=5)
    request=GetParameters.Request();request.names=['teacher_version', 'mode', 'object_model']
    future=client.call_async(request)
    rclpy.spin_until_future_complete(probe,future,timeout_sec=5)
    assert future.done() and future.result() is not None
    adapter=[v.string_value for v in future.result().values]
    assert adapter==['V45','teacher_existing_margin','surface'], adapter
    print(json.dumps(dict(passed=True,teacher='MPPI_SIM_V45',topic=topic,domain_id=os.environ.get('ROS_DOMAIN_ID'),
        consumers=sorted(expected),native_teacher_consumers=[],parameters=checked)))
finally:
    probe.destroy_node();rclpy.shutdown()
