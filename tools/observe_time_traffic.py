"""Read-only, per-domain AWSIM judge and motion journals; no model inputs."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


def main() -> None:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domains', type=int, nargs='+', required=True)
    args = parser.parse_args()
    if args.domains != list(range(1, len(args.domains)+1)) or not 2 <= len(args.domains) <= 4:
        raise ValueError('ORDERED_VEHICLE_DOMAINS_REQUIRED')
    contexts = []
    executors = []
    nodes = []
    journals = []
    try:
        for domain in args.domains:
            context = Context()
            context.init(args=[], domain_id=domain)
            contexts.append(context)
            node = Node('time_traffic_observer', context=context)
            nodes.append(node)
            journal = (args.output/f'traffic_d{domain}.jsonl').open('x', buffering=1)
            journals.append(journal)
            state = dict(domain_id=domain, status_messages=0, odometry_messages=0, state=None,
                         speed_mps=None, status_monotonic_ns=None, odometry_monotonic_ns=None)

            def write(row: dict, *, journal=journal, domain=domain) -> None:
                journal.write(json.dumps(dict(domain_id=domain, monotonic_ns=time.monotonic_ns(), **row),
                                         allow_nan=False)+'\n')

            def status(msg, *, state=state, node=node, write=write) -> None:
                state['status_messages'] += 1
                state['status_monotonic_ns'] = time.monotonic_ns()
                state['data'] = list(msg.data)
                write(dict(kind='status', data=list(msg.data), publisher_count=node.count_publishers('/awsim/status')))

            def odometry(msg, *, state=state, write=write) -> None:
                state['odometry_messages'] += 1
                state['odometry_monotonic_ns'] = time.monotonic_ns()
                state['speed_mps'] = msg.twist.twist.linear.x
                write(dict(kind='odometry', speed_mps=state['speed_mps'],
                           sim_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           x_m=msg.pose.pose.position.x, y_m=msg.pose.pose.position.y))

            def vehicle_state(msg, *, state=state, write=write) -> None:
                state['state'] = msg.data
                write(dict(kind='state', value=msg.data))

            def heartbeat(*, state=state, node=node, domain=domain) -> None:
                snapshot = dict(state, monotonic_ns=time.monotonic_ns(),
                    status_publishers=node.count_publishers('/awsim/status'),
                    command_publishers=node.count_publishers('/control/command/control_cmd'))
                pending = args.output/f'traffic_d{domain}.pending'
                pending.write_text(json.dumps(snapshot, allow_nan=False))
                pending.replace(args.output/f'traffic_d{domain}.json')

            node.create_subscription(Float32MultiArray, '/awsim/status', status, qos_profile_sensor_data)
            node.create_subscription(Odometry, '/localization/kinematic_state', odometry, qos_profile_sensor_data)
            node.create_subscription(String, '/awsim/state', vehicle_state,
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_timer(.2, heartbeat)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)
        while all(context.ok() for context in contexts):
            for executor in executors:
                executor.spin_once(timeout_sec=0)
            time.sleep(.001)
    finally:
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            context.try_shutdown()
        for journal in journals:
            journal.close()


if __name__ == '__main__':
    main()
