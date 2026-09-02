import ast
from pathlib import Path


def test_safety_node_uses_strict_stamps_and_deadline() -> None:
    source = (Path(__file__).parents[1] / "aic_e2e_runtime" / "safety_supervisor_node.py").read_text()
    assert "from .runtime_adapter import stamp_to_seconds" not in source
    assert "strict_message_stamp_to_seconds" in source
    assert "now > self.nominal_valid_until_sec" in source


def test_v3_uses_velocity_report_without_map_odometry() -> None:
    root = Path(__file__).parents[1]
    inference = (root / "aic_e2e_runtime" / "inference_node_v3.py").read_text()
    launch = (root / "launch" / "transfuser_lite_v3_trajectory.launch.py").read_text()

    assert "VelocityReport" in inference
    assert "from nav_msgs.msg import Odometry" not in inference
    assert '"velocity_status"' in inference
    assert "/localization/kinematic_state" not in launch
    assert '"odometry"' not in launch


def test_safety_keeps_v1_odometry_default_and_allows_v3_velocity_report() -> None:
    root = Path(__file__).parents[1]
    safety = (root / "aic_e2e_runtime" / "safety_supervisor_node.py").read_text()
    v1_launch = (root / "launch" / "transfuser_lite_v1.launch.py").read_text()
    v3_params = (root / "config" / "runtime.v3.trajectory.param.yaml").read_text()

    assert 'declare_parameter("ego_speed_source", "odometry")' in safety
    assert 'ego_speed_source == "velocity_report"' in safety
    assert '("odometry", LaunchConfiguration("odometry_topic"))' in v1_launch
    assert "ego_speed_source: velocity_report" in v3_params


def test_v3_ego_features_map_to_wheel_odometry_and_steer_angle() -> None:
    root = Path(__file__).parents[1]
    source = (root / "aic_e2e_runtime" / "inference_node_v3.py").read_text()
    assert "velocity.longitudinal_velocity" in source
    assert "velocity.lateral_velocity" in source
    assert "velocity.heading_rate" in source
    assert "steering.steering_tire_angle" in source


def test_ros_package_installs_canonical_v3_source() -> None:
    source = (Path(__file__).parents[1] / "setup.py").read_text()
    assert 'rglob("*.py")' in source
    assert '"/python_src/"' in source
    helper = (Path(__file__).parents[1] / "aic_e2e_runtime" / "canonical_source.py").read_text()
    assert "sys.path.insert(0, value)" in helper


def _v3_subscription_qos_arguments() -> dict[str, ast.expr]:
    source = (
        Path(__file__).parents[1] / "aic_e2e_runtime" / "inference_node_v3.py"
    ).read_text()
    tree = ast.parse(source)
    qos_by_message: dict[str, ast.expr] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 4:
            continue
        function = node.func
        if not isinstance(function, ast.Attribute) or function.attr != "create_subscription":
            continue
        message_type = node.args[0]
        if isinstance(message_type, ast.Name):
            qos_by_message[message_type.id] = node.args[3]
    return qos_by_message


def test_v3_camera_and_lidar_use_sensor_data_qos() -> None:
    qos_by_message = _v3_subscription_qos_arguments()
    for message_type in ("Image", "LaserScan"):
        qos = qos_by_message[message_type]
        assert isinstance(qos, ast.Name)
        assert qos.id == "qos_profile_sensor_data"


def test_v3_sensor_streams_do_not_regress_to_implicit_reliable_depth() -> None:
    qos_by_message = _v3_subscription_qos_arguments()
    for message_type in ("Image", "LaserScan"):
        assert not isinstance(qos_by_message[message_type], ast.Constant)

    # Wheel Odometry and Steer Angle were compatible with the official graph;
    # do not broaden this sensor-stream compatibility change to vehicle state.
    for message_type in ("VelocityReport", "SteeringReport"):
        qos = qos_by_message[message_type]
        assert isinstance(qos, ast.Constant)
        assert qos.value == 10


def test_v3_uses_settled_camera_buffer_instead_of_callback_latest_values() -> None:
    source = (
        Path(__file__).parents[1] / "aic_e2e_runtime" / "inference_node_v3.py"
    ).read_text()
    assert "SettledCameraSynchronizer" in source
    assert "self.synchronizer.add_camera" in source
    assert "self.synchronizer.add_sensor" in source
    assert "self.synchronizer.pop_ready" in source
    assert "self.latest" not in source
    assert '"runtime_sync_debug"' in source
    assert "runtime_clock_has_reached_observation" in source
    assert "self.ready_observations" in source


def test_v3_publishes_matching_trajectory_and_speed_without_control_authority() -> None:
    source = (
        Path(__file__).parents[1] / "aic_e2e_runtime" / "inference_node_v3.py"
    ).read_text()
    assert '"predicted_trajectory"' in source
    assert '"predicted_speed_profile"' in source
    assert "trajectory_speed_publication(" in source
    assert "output.trajectory_speed_mps" in source
    assert "self.speed_profile_pub.publish" in source
    assert '"nominal_control_cmd"' not in source


def test_v3_external_controller_profile_is_explicitly_shadow_only() -> None:
    root = Path(__file__).parents[1]
    source = (root / "aic_e2e_runtime" / "inference_node_v3.py").read_text()
    launch = (
        root / "launch" / "transfuser_lite_v3_external_controller_shadow.launch.py"
    ).read_text()
    params = (
        root / "config" / "runtime.v3.external_controller_shadow.param.yaml"
    ).read_text()

    assert '"shadow_external_control"' in source
    assert "shadow_control_from_trajectory_speed_profile(" in source
    assert "if result.nominal_control_eligible" in source
    assert '"nominal_control_cmd"' not in source
    assert "runtime.v3.external_controller_shadow.param.yaml" in launch
    assert "runtime_profile: external_controller" in params
    assert "controller_calibration_status: unverified" in params
    assert "trajectory_step_sec: 0.1" in params
    assert "max_steering_rate_radps: 0.0" in params


def test_trajectory_only_launch_does_not_select_external_controller_profile() -> None:
    root = Path(__file__).parents[1]
    launch = (root / "launch" / "transfuser_lite_v3_trajectory.launch.py").read_text()
    params = (root / "config" / "runtime.v3.trajectory.param.yaml").read_text()

    assert "external_controller_shadow" not in launch
    assert "runtime_profile: external_controller" not in params
