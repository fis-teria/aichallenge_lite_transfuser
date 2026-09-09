"""Opt-in entry point for the existing make-dev V4_SHADOW_LAUNCH hook."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name('v4_shadow.launch.py'))),
            launch_arguments={'config_file':LaunchConfiguration('config_file'),
                              'local_odometry':'true'}.items()),
    ])
