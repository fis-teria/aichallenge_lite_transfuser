"""Include alongside the existing controller launch; no controller replacement."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file',description='Reviewed V4 shadow JSON config (required)'),
        Node(package='aic_e2e_runtime',executable='v4_shadow_node',name='v4_shadow',
             output='screen',parameters=[{'config_file':LaunchConfiguration('config_file')}]),
    ])
