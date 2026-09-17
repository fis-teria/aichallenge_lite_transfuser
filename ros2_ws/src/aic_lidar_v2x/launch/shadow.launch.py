"""Start the observer only. Does not launch or remap a driving controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("reference_csv", default_value=""),
        DeclareLaunchArgument("object_model", default_value="surface"),
        DeclareLaunchArgument("motion_model", default_value="rolling"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="aic_lidar_v2x", executable="lidar_v2x_node", output="screen", parameters=[{
            "map_yaml": LaunchConfiguration("map_yaml"),
            "reference_csv": ParameterValue(LaunchConfiguration("reference_csv"), value_type=str),
            "object_model": LaunchConfiguration("object_model"),
            "motion_model": LaunchConfiguration("motion_model"),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }]),
    ])
