from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = (
    ROOT
    / "ros2_ws"
    / "src"
    / "aic_e2e_runtime"
    / "launch"
    / "teacher_capture_safety_v3.launch.py"
)
NODE = (
    ROOT
    / "ros2_ws"
    / "src"
    / "aic_e2e_runtime"
    / "aic_e2e_runtime"
    / "safety_supervisor_node.py"
)


def test_teacher_capture_safety_uses_sim_time_and_single_final_authority() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in source
    assert '"use_sim_time": LaunchConfiguration("use_sim_time")' in source
    assert 'name="aic_safety_supervisor_teacher_v3"' in source
    assert '("nominal_control_cmd", LaunchConfiguration("nominal_topic"))' in source
    assert '("control_cmd", LaunchConfiguration("final_topic"))' in source
    assert 'DeclareLaunchArgument("maximum_speed_mps", default_value="0.75")' in source
    assert 'LaunchConfiguration("maximum_speed_mps"), value_type=float' in source


def test_teacher_capture_safety_keeps_strict_freshness_and_sensor_timeouts() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"future_tolerance_sec": 0.001' in source
    assert '"nominal_timeout_sec": 0.45' in source
    assert '"camera_timeout_sec": 0.3' in source
    assert '"lidar_timeout_sec": 0.2' in source
    assert '"ego_timeout_sec": 0.2' in source


def test_runtime_preserves_speed_cap_while_braking_overspeed() -> None:
    source = NODE.read_text(encoding="utf-8")

    assert '"speed_limit_exceeded", "speed_limit_guard"' in source
    assert "commanded_speed_mps = self.config.max_speed_mps" in source
