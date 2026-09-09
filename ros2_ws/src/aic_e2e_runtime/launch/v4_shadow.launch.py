"""Include alongside the existing controller launch; no controller replacement."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file',description='Reviewed V4 shadow JSON config (required)'),
        DeclareLaunchArgument('local_odometry',default_value='false'),
        Node(package='aic_e2e_runtime',executable='local_odometry_node_v4',name='v4_local_odometry',
             output='screen',parameters=[{'use_sim_time':True}],
             condition=IfCondition(LaunchConfiguration('local_odometry'))),
        Node(package='aic_e2e_runtime',executable='v4_shadow_node',name='v4_shadow',
             output='screen',parameters=[{'config_file':LaunchConfiguration('config_file')}]),
    ])
