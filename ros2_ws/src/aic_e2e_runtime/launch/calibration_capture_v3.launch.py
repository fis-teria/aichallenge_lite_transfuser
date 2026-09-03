from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("plan_path"),
        DeclareLaunchArgument("arm_token"),
        DeclareLaunchArgument("result_path"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/sensing/camera/image_raw"),
        DeclareLaunchArgument("scan_topic", default_value="/sensing/lidar/scan"),
        DeclareLaunchArgument(
            "velocity_topic", default_value="/vehicle/status/velocity_status"
        ),
        DeclareLaunchArgument(
            "nominal_topic", default_value="/nominal_control_cmd"
        ),
        DeclareLaunchArgument(
            "final_topic", default_value="/control/command/control_cmd"
        ),
        DeclareLaunchArgument(
            "safety_reason_topic", default_value="/calibration/safety_reason"
        ),
        DeclareLaunchArgument(
            "status_topic", default_value="/calibration/excitation_status"
        ),
    ]
    safety = Node(
        package="aic_e2e_runtime",
        executable="safety_supervisor_node",
        name="aic_safety_supervisor_calibration_v3",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ego_speed_source": "velocity_report",
                "publish_hz": 20.0,
                "max_steer_rad": 0.25,
                "max_speed_mps": 3.0,
                "max_accel_mps2": 1.0,
                "min_accel_mps2": -2.0,
                "enable_model_stop": False,
            }
        ],
        remappings=[
            ("image", LaunchConfiguration("image_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
            ("velocity_status", LaunchConfiguration("velocity_topic")),
            ("nominal_control_cmd", LaunchConfiguration("nominal_topic")),
            ("control_cmd", LaunchConfiguration("final_topic")),
            ("safety_reason", LaunchConfiguration("safety_reason_topic")),
        ],
    )
    excitation = Node(
        package="aic_e2e_runtime",
        executable="calibration_excitation_node",
        name="aic_calibration_excitation_v3",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "plan_path": LaunchConfiguration("plan_path"),
                "arm_token": LaunchConfiguration("arm_token"),
                "result_path": LaunchConfiguration("result_path"),
            }
        ],
        remappings=[
            ("velocity_status", LaunchConfiguration("velocity_topic")),
            ("nominal_control_cmd", LaunchConfiguration("nominal_topic")),
            ("control_cmd", LaunchConfiguration("final_topic")),
            ("safety_reason", LaunchConfiguration("safety_reason_topic")),
            ("calibration_status", LaunchConfiguration("status_topic")),
        ],
    )
    stop_launch_when_excitation_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=excitation,
            on_exit=[EmitEvent(event=Shutdown(reason="calibration excitation finished"))],
        )
    )
    return LaunchDescription(
        arguments + [safety, excitation, stop_launch_when_excitation_finishes]
    )
