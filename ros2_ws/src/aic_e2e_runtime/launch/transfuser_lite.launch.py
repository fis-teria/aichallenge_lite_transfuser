from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_model = PathJoinSubstitution(
        [FindPackageShare("aic_e2e_runtime"), "ckpt", "best.pt"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("aic_e2e_runtime"), "config", "runtime.param.yaml"]
    )
    arguments = [
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("param_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/sensing/camera/image_raw"),
        DeclareLaunchArgument("scan_topic", default_value="/sensing/lidar/scan"),
        DeclareLaunchArgument("odometry_topic", default_value="/localization/kinematic_state"),
        DeclareLaunchArgument("control_cmd_topic", default_value="/control/command/control_cmd"),
    ]
    shared_remaps = [
        ("image", LaunchConfiguration("image_topic")),
        ("scan", LaunchConfiguration("scan_topic")),
        ("odometry", LaunchConfiguration("odometry_topic")),
    ]
    inference = Node(
        package="aic_e2e_runtime",
        executable="inference_node",
        name="aic_transfuser_inference",
        output="screen",
        parameters=[
            LaunchConfiguration("param_file"),
            {
                "model_path": LaunchConfiguration("model_path"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=shared_remaps,
    )
    safety = Node(
        package="aic_e2e_runtime",
        executable="safety_supervisor_node",
        name="aic_safety_supervisor",
        output="screen",
        parameters=[
            LaunchConfiguration("param_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=shared_remaps
        + [("control_cmd", LaunchConfiguration("control_cmd_topic"))],
    )
    return LaunchDescription(arguments + [inference, safety])
