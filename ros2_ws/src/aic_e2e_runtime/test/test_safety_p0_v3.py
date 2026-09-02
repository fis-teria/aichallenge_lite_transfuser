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
