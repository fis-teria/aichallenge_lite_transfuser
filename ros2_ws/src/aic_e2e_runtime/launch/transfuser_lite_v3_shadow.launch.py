from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare("aic_e2e_runtime"),
        "config",
        "runtime.v3.shadow.param.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("aic_e2e_runtime"),
        "config",
        "v3_external_controller_shadow.rviz",
    ])
    arguments = [
        DeclareLaunchArgument("param_file", default_value=default_params),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("artifact_manifest_path"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument(
            "velocity_topic", default_value="/vehicle/status/velocity_status"
        ),
        DeclareLaunchArgument(
            "steering_topic", default_value="/vehicle/status/steering_status"
        ),
    ]
    inference = Node(
        package="aic_e2e_runtime",
        executable="inference_node_v3",
        name="aic_transfuser_inference_v3_shadow",
        output="screen",
        parameters=[LaunchConfiguration("param_file"), {
            "model_path": LaunchConfiguration("model_path"),
            "artifact_manifest_path": LaunchConfiguration("artifact_manifest_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[
            ("image", "/sensing/camera/image_raw"),
            ("scan", "/sensing/lidar/scan"),
            ("velocity_status", LaunchConfiguration("velocity_topic")),
            ("steering_status", LaunchConfiguration("steering_topic")),
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="v3_model_control_shadow_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    # This launch owns debug outputs only. The external controller and Safety
    # Supervisor remain separate authority-bearing processes.
    return LaunchDescription(arguments + [inference, rviz])
