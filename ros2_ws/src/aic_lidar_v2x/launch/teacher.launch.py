"""MPPI V44 collection using LiDAR positions and its unchanged collision margins.

Requires the pinned V44 overlay to have been sourced. Does not launch AWSIM or
E2E inference; the existing scenario harness owns those processes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from aic_lidar_v2x.teacher import V44_INPUT_REMAPS


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("domain_id", default_value="1"),
        DeclareLaunchArgument("run_rviz", default_value="false"),
        DeclareLaunchArgument("speed_cap_mps", default_value="2.7777777777777777"),
        DeclareLaunchArgument("motion_model", default_value="rolling"),
    ]
    teacher = IncludeLaunchDescription(AnyLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("aichallenge_system_launch"), "launch", "aichallenge_system.launch.xml"])),
        launch_arguments={
            "simulation": "true", "use_sim_time": LaunchConfiguration("use_sim_time"),
            "run_rviz": LaunchConfiguration("run_rviz"), "launch_vehicle_interface": "false",
            "domain_id": LaunchConfiguration("domain_id"), "capture": "false", "rosbag": "false",
            "control_method_override": "mppi", "mppi_reference_profile": "race",
            "reference_execution_speed_cap_mps": LaunchConfiguration("speed_cap_mps"),
        }.items())
    adapter = Node(package="aic_lidar_v2x", executable="lidar_v2x_node", output="screen", parameters=[{
        "map_yaml": LaunchConfiguration("map_yaml"), "object_model": "surface",
        "mode": "teacher_existing_margin", "motion_model": LaunchConfiguration("motion_model"),
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    }])
    # Adapter stays outside the scoped remaps; simulator/native publishers and
    # all perception inputs retain their original topic names.
    return LaunchDescription(arguments + [
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        adapter,
        GroupAction(actions=[*[SetRemap(src=src, dst=dst) for src, dst in V44_INPUT_REMAPS], teacher]),
    ])
