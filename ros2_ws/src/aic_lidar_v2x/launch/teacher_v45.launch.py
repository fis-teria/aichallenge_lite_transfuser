"""AWSIM-only V45 teacher using the pinned submit overlay and LiDAR V2X.

Include the submit launch directly: older host system launches do not forward
the MPPI profile/speed arguments. Simulation adapters and Start authority stay
owned by the installed official system package. No real-vehicle launch exists.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from aic_lidar_v2x.teacher import V44_INPUT_REMAPS


def include(package: str, path: list[str], arguments: dict):
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), *path])), launch_arguments=arguments.items())


def generate_launch_description():
    arguments = [DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("domain_id", default_value="1"),
        DeclareLaunchArgument("speed_cap_mps", default_value="1.3888888888888888"),
        DeclareLaunchArgument("run_rviz", default_value="false")]
    teacher = include("aichallenge_submit_launch", ["launch", "aichallenge_submit.launch.xml"], {
        "simulation": "true", "use_sim_time": "true", "sensor_model": "racing_kart_sensor_kit",
        "launch_vehicle_interface": "false", "control_method": "mppi", "mppi_reference_profile": "race",
        "reference_execution_speed_cap_mps": LaunchConfiguration("speed_cap_mps")})
    adapter = Node(package="aic_lidar_v2x", executable="lidar_v2x_node", output="screen", parameters=[{
        "map_yaml": LaunchConfiguration("map_yaml"), "object_model": "surface",
        "mode": "teacher_existing_margin", "teacher_version": "V45", "motion_model": "rolling",
        "use_sim_time": True}])
    simulation = include("aichallenge_system_launch", ["launch", "mode", "awsim.launch.xml"], {
        "capture": "false", "rosbag": "false", "race_arm_on_vehicle_state": "Start"})
    relay = Node(package="topic_tools", executable="relay", name="initial_pose_relay", parameters=[{
        "input_topic": "/initialpose", "output_topic": "/localization/initial_pose3d"}])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", condition=IfCondition(LaunchConfiguration("run_rviz")),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("aichallenge_system_launch"), "config", "autoware.rviz"])],
        parameters=[{"use_sim_time": True}])
    return LaunchDescription(arguments + [
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")), adapter,
        GroupAction(actions=[*[SetRemap(src=src, dst=dst) for src, dst in V44_INPUT_REMAPS], teacher]),
        relay, simulation, rviz])
