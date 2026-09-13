"""Teacher-free TimePath inference and standard Path display topic; no control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = ("checkpoint", "checkpoint_sha256", "output", "run_id")
    declarations = [DeclareLaunchArgument(name) for name in names]
    declarations.append(DeclareLaunchArgument("device", default_value="cuda"))
    arguments = []
    for name in (*names, "device"):
        arguments.extend(["--" + name.replace("_", "-"), LaunchConfiguration(name)])
    return LaunchDescription(declarations + [Node(package="aic_e2e_runtime", executable="time_path_node",
                                                 arguments=arguments, output="screen")])
