from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('enable_v4_input_only',default_value='false',description='Opt-in is not live authorization; current bootstrap remains blocked'),
        Node(package='aic_e2e_runtime',executable='spatial_path_shadow_node_v4',name='spatial_path_shadow_v4',
             condition=IfCondition(LaunchConfiguration('enable_v4_input_only')),
             parameters=[PathJoinSubstitution([FindPackageShare('aic_e2e_runtime'),'config','spatial_path_shadow_v4.param.yaml'])])
    ])
