"""Inspect a running teacher.launch.py in an isolated, non-driving ROS domain."""
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from aic_lidar_v2x.teacher import V2X_TOPIC


def main() -> None:
    rclpy.init()
    probe = Node("lidar_v2x_teacher_graph_probe")
    expected = {"reference_space_mppi_planner", "simple_trajectory_generator", "mppi_recovery_controller"}
    try:
        deadline = time.monotonic() + 35.0
        subscriptions = []
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
            subscriptions = probe.get_subscriptions_info_by_topic(V2X_TOPIC)
            if expected.issubset({s.node_name for s in subscriptions}):
                break
        actual = {s.node_name for s in subscriptions}
        assert expected.issubset(actual), f"Missing teacher subscriptions: {expected-actual}; got {actual}"
        native_subs = probe.get_subscriptions_info_by_topic("/v2x/vehicle_positions")
        assert not expected.intersection(s.node_name for s in native_subs), "Teacher still subscribes to native V2X"
        publishers = probe.get_publishers_info_by_topic(V2X_TOPIC)
        assert len(publishers) == 1 and publishers[0].node_name == "lidar_v2x"
        planner = next(s for s in subscriptions if s.node_name == "reference_space_mppi_planner")
        service = planner.node_namespace.rstrip("/") + "/" + planner.node_name + "/get_parameters"
        client = probe.create_client(GetParameters, service)
        assert client.wait_for_service(timeout_sec=5.0)
        original = dict(obstacle_longitudinal_inflation_m=1.10, obstacle_lateral_inflation_m=1.15,
                        clearance_target_m=0.35,
                        **{"brain.footprint_radius_m": 0.65, "brain.footprint_front_m": 1.06,
                           "brain.footprint_rear_m": 1.10})
        request = GetParameters.Request()
        request.names = list(original)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(probe, future, timeout_sec=5.0)
        assert future.done() and future.result() is not None
        values = {key: value.double_value for key, value in zip(request.names, future.result().values)}
        assert all(math.isclose(values[k], v, abs_tol=1e-9) for k, v in original.items()), values
        print(json.dumps(dict(passed=True, teacher_v2x_topic=V2X_TOPIC,
            consumers=sorted(actual), native_teacher_consumers=[],
            publishers=[p.node_name for p in publishers], unchanged_margin_parameters=values,
            driving_test=False)), flush=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
