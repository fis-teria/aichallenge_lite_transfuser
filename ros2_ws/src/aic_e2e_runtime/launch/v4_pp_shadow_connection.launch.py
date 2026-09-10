"""Existing PP executable, entirely namespaced shadow output; no live remap arg."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('aic_e2e_runtime'),
                          'config', 'v4_pp_shadow_connection.json')
    return LaunchDescription([
        # Explicit /clock input; wall timer remains alive when sim clock pauses.
        Node(package='aic_e2e_runtime', executable='v4_pp_connection_node',
             parameters=[{'config_file': config, 'use_sim_time': False}]),
        Node(package='simple_pure_pursuit', executable='simple_pure_pursuit',
             namespace='/shadow/v4/pp', name='controller',
             parameters=[{'use_sim_time': True, 'wheel_base': 1.087,
                 'lookahead_gain': 0.0, 'lookahead_min_distance': 0.5,
                 'curvature_adaptive_lookahead_enabled': False,
                 'use_external_target_vel': False, 'steering_tire_angle_gain': 1.0,
                 'use_overtake_reference_override': False, 'use_mpc_predicted_horizon': False,
                 'recovery_mode': False, 'stop_on_stale_input': True,
                 'max_odom_age_sec': 0.15, 'max_trajectory_age_sec': 0.15,
                 'aw2_shadow_transport_enabled': False}],
             remappings=[('input/kinematics', '/shadow/v4/pp/odometry'),
                         ('input/trajectory', '/shadow/v4/pp/trajectory'),
                         ('/control/debug/lookahead_point', '/shadow/v4/pp/lookahead'),
                         ('/pure_pursuit/debug', '/shadow/v4/pp/debug')])])
