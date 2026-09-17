"""Isolated ROS smoke of the installed launch and immutable speed parameters.

Run in a network-none container, ROS_DOMAIN_ID=93. No sensor fixture, official
Start or actual-control publisher is created. Separate connection smoke covers
synthetic sensor/control math.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '93':
        raise ValueError('ISOLATED_DOMAIN_93_REQUIRED')
    args.output.mkdir(exist_ok=False)
    command = ['ros2', 'launch', 'aic_e2e_runtime', 'time_path_awsim.launch.py',
        'checkpoint:='+str(args.checkpoint), 'output:='+str(args.output/'run'),
        'run_id:=codex-time-launch-smoke', 'device:=cpu',
        'max_speed_kmh:=12.0', 'corner_max_speed_kmh:=8.0']
    import rclpy
    from rcl_interfaces.srv import GetParameters, DescribeParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    rclpy.init()
    node = rclpy.create_node('time_dev_launch_check')
    result = dict(status='FAILED', command=command)
    with (args.output/'launch.log').open('x') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def call(service_type, name, request):
                client = node.create_client(service_type, '/time_path_controller/'+name)
                if not client.wait_for_service(timeout_sec=15.):
                    raise RuntimeError('PARAMETER_SERVICE_MISSING:'+name)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
                if not future.done() or future.exception():
                    raise RuntimeError('PARAMETER_SERVICE_FAILED:'+name)
                return future.result()
            names = ['max_speed_kmh', 'corner_max_speed_kmh']
            values = call(GetParameters, 'get_parameters', GetParameters.Request(names=names))
            assert [v.double_value for v in values.values] == [12., 8.]
            description = call(DescribeParameters, 'describe_parameters', DescribeParameters.Request(names=names))
            assert all(p.read_only for p in description.descriptors)
            changed = call(SetParameters, 'set_parameters', SetParameters.Request(parameters=[
                Parameter(name=names[0], value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=18.))]))
            assert not changed.results[0].successful
            deadline = time.monotonic()+25.
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('LAUNCH_EXITED_EARLY')
                if all((args.output/'run'/name).is_file() for name in ('control_heartbeat.json', 'inference_heartbeat.json')):
                    break
                rclpy.spin_once(node, timeout_sec=.1)
            else:
                raise RuntimeError('NODE_HEARTBEAT_MISSING')
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.1)
            assert not node.get_publishers_info_by_topic('/control/command/control_cmd')
            assert node.get_publishers_info_by_topic('/time_path/shadow/control_cmd')
            config = json.loads((args.output/'run/trial_config.json').read_text())
            assert config['speed_parameters'] == dict(max_speed_kmh=12., corner_max_speed_kmh=8.)
            assert config['speed_cap_mps'] == 12/3.6 and config['overspeed_limit_mps'] == 13/3.6
            result.update(status='PASS', parameters=config['speed_parameters'], hot_changes_rejected=True,
                          actual_control_publishers=0, both_node_heartbeats=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3.)
                result['status'] = 'FAILED_SHUTDOWN_TIMEOUT'
            node.destroy_node(); rclpy.shutdown()
            (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    if result['status'] != 'PASS':
        raise RuntimeError(result['status'])
    print(json.dumps(result))


if __name__ == '__main__':
    main()
