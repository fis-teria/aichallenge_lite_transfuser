"""TimePath inference + Pure Pursuit with recorded launch-time speed ceilings.

Use make dev DEV_CONTROLLER=time for AWSIM, official Start, ordinary RViz,
lap supervision and cleanup. This ROS launch alone defaults to shadow output.
"""
from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    from aic_e2e_runtime.canonical_source import prefer_canonical_source
    prefer_canonical_source()
    from aic_transfuser_lite.control.time_dev_v1 import configure_dev_speeds
    from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config

    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    parameters = {name: float(value(name)) for name in ('max_speed_kmh', 'corner_max_speed_kmh')}
    config = configure_dev_speeds(json.loads(Path(value('config')).read_text()), **parameters)
    validate_trial_config(config)
    live = value('authorize_awsim_only')
    if live not in ('true', 'false'):
        raise ValueError('authorize_awsim_only must be true or false')
    output = Path(value('output'))
    output.mkdir(parents=True, exist_ok=True)
    effective = output / 'trial_config.json'
    encoded = (json.dumps(config, indent=2, allow_nan=False)+'\n').encode()
    if effective.exists():
        if effective.read_bytes() != encoded:
            raise ValueError('EFFECTIVE_CONFIG_ALREADY_EXISTS_WITH_DIFFERENT_PARAMETERS')
    else:
        with effective.open('xb') as stream:
            stream.write(encoded)
    common = ['--output', str(output), '--run-id', value('run_id'),
              '--checkpoint-sha256', config['checkpoint_sha256']]
    inference = Node(package='aic_e2e_runtime', executable='time_path_node', output='screen',
        arguments=[*common, '--checkpoint', value('checkpoint'), '--device', value('device')])
    controller = Node(package='aic_e2e_runtime', executable='time_trial_controller_node', output='screen',
        parameters=[parameters], arguments=[*common, '--trial-config', str(effective),
            '--rear-axle-forward-m', str(config['geometry']['rear_axle_forward_in_base_link_m']),
            *(['--authorize-awsim-only'] if live == 'true' else [])])
    # Parent host independently checks heartbeat and freezes its AWSIM before
    # shutting down control. Never leave a surviving half of this pair running.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='TIME_PATH_NODE_EXIT'))])) for node in (inference, controller)]
    return [*handlers, inference, controller, TimerAction(period=config['outer_limit_wall_s']-20.,
        actions=[EmitEvent(event=Shutdown(reason='TIME_PATH_LAUNCH_WALL_LIMIT'))])]


def generate_launch_description():
    config = str(Path(get_package_share_directory('aic_e2e_runtime'))/'config/time_path_dev.json')
    required = [DeclareLaunchArgument(name) for name in ('checkpoint', 'output', 'run_id')]
    defaults = dict(config=config, max_speed_kmh='20.0', corner_max_speed_kmh='10.0',
                    device='cuda', authorize_awsim_only='false')
    return LaunchDescription([*required,
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=launch_nodes)])
