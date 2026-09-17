"""Independent scan/map alignment alongside TimePath wheel odometry."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Verified Lanelet2 OSM with local_x/local_y in map frame'),
        DeclareLaunchArgument('correction_mode', default_value='bounded',
                              choices=['bounded','unlimited','simulation_aggressive']),
        Node(package='aic_e2e_runtime',executable='lidar_map_localization_node',output='screen',
             arguments=['--map',LaunchConfiguration('map'),
                        '--correction-mode',LaunchConfiguration('correction_mode')]),
    ])
