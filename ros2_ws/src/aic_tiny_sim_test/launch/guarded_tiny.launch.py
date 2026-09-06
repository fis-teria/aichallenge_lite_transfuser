"""Explicit simulator-only CONTROL_METHOD; RViz subscribes to scan only."""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    if os.environ.get("CONTROL_METHOD") != "tiny_lidar_net_guarded":
        raise RuntimeError("EXPLICIT_GUARDED_TINY_CONTROL_METHOD_REQUIRED")
    project = os.environ["TINY_PROJECT"]
    supervisor = Node(package="aic_tiny_sim_test", executable="tiny_sim_supervisor", output="screen",
        arguments=["--config", "/evidence/resolved_config.json", "--output", "/evidence",
                   "--project", project, "--authorize-sim-session"])
    rviz = Node(package="rviz2", executable="rviz2", name="tiny_test_rviz", output="screen",
        arguments=["-d", get_package_share_directory("aic_tiny_sim_test")+"/config/tiny_scan.rviz"],
        parameters=[{"use_sim_time": True}])
    return LaunchDescription([supervisor, rviz,
        RegisterEventHandler(OnProcessExit(target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason="Finite Tiny supervisor finished"))])),
        RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="Required RViz window exited"))]))])
