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
        "runtime.v3.full_control_trial.param.yaml",
    ])
    default_rviz = PathJoinSubstitution([
        FindPackageShare("aic_e2e_runtime"),
        "config",
        "v3_external_controller_shadow.rviz",
    ])
    arguments = [
        DeclareLaunchArgument("param_file", default_value=default_params),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("artifact_manifest_path"),
        DeclareLaunchArgument("calibration_artifact_path"),
        DeclareLaunchArgument("full_control_evidence_path"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("velocity_topic", default_value="/vehicle/status/velocity_status"),
        DeclareLaunchArgument("steering_topic", default_value="/vehicle/status/steering_status"),
    ]
    remappings = [
        ("image", "/sensing/camera/image_raw"),
        ("scan", "/sensing/lidar/scan"),
        ("velocity_status", LaunchConfiguration("velocity_topic")),
        ("steering_status", LaunchConfiguration("steering_topic")),
    ]
    inference = Node(
        package="aic_e2e_runtime",
        executable="inference_node_v3",
        name="aic_transfuser_inference_v3_full_control",
        output="screen",
        parameters=[LaunchConfiguration("param_file"), {
            "model_path": LaunchConfiguration("model_path"),
            "artifact_manifest_path": LaunchConfiguration("artifact_manifest_path"),
            "calibration_artifact_path": LaunchConfiguration("calibration_artifact_path"),
            "full_control_evidence_path": LaunchConfiguration("full_control_evidence_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=remappings,
    )
    safety = Node(
        package="aic_e2e_runtime",
        executable="safety_supervisor_node",
        name="aic_safety_supervisor_v3_full_control",
        output="screen",
        parameters=[LaunchConfiguration("param_file"), {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=remappings + [
            ("nominal_control_cmd", "nominal_control_cmd"),
            ("control_cmd", "/control/command/control_cmd"),
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="v3_full_control_trial_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    return LaunchDescription(arguments + [inference, safety, rviz])
