from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Run the final-authority safety node for external teacher collection."""

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/sensing/camera/image_raw"),
        DeclareLaunchArgument("scan_topic", default_value="/sensing/lidar/scan"),
        DeclareLaunchArgument(
            "velocity_topic", default_value="/vehicle/status/velocity_status"
        ),
        DeclareLaunchArgument("nominal_topic", default_value="/nominal_control_cmd"),
        DeclareLaunchArgument(
            "final_topic", default_value="/control/command/control_cmd"
        ),
        DeclareLaunchArgument("safety_reason_topic", default_value="/safety_reason"),
        DeclareLaunchArgument("maximum_speed_mps", default_value="0.75"),
    ]
    safety = Node(
        package="aic_e2e_runtime",
        executable="safety_supervisor_node",
        name="aic_safety_supervisor_teacher_v3",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ego_speed_source": "velocity_report",
                "publish_hz": 20.0,
                "max_speed_mps": ParameterValue(
                    LaunchConfiguration("maximum_speed_mps"), value_type=float
                ),
                "max_steer_rad": 0.6,
                "max_accel_mps2": 2.0,
                "min_accel_mps2": -4.0,
                "camera_timeout_sec": 0.3,
                "lidar_timeout_sec": 0.2,
                "ego_timeout_sec": 0.2,
                "future_tolerance_sec": 0.001,
                "max_command_validity_sec": 0.45,
                "nominal_timeout_sec": 0.45,
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
    return LaunchDescription(arguments + [safety])
