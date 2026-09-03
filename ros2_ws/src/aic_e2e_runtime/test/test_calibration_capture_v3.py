from __future__ import annotations

import ast
from pathlib import Path


def test_calibration_node_is_hash_armed_and_fail_closed() -> None:
    root = Path(__file__).parents[1]
    source = (root / "aic_e2e_runtime/calibration_excitation_node.py").read_text()
    ast.parse(source)
    assert "arm_token != self.plan_sha256" in source
    assert "runtime_guard_reasons(" in source
    assert "count_publishers" in source
    assert "observed_speed_exceeds_plan_limit" not in source  # owned by ROS-free core
    assert "_publish_command(self._stop_command())" in source
    assert '"aborted"' in source


def test_calibration_completion_leaves_executor_outside_timer_callback() -> None:
    root = Path(__file__).parents[1]
    source = (root / "aic_e2e_runtime/calibration_excitation_node.py").read_text()
    tree = ast.parse(source)
    node_class = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "CalibrationExcitationNode"
    )
    finish = next(
        item
        for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name == "_finish"
    )
    called_attributes = {
        call.func.attr
        for call in ast.walk(finish)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
    }
    assert "shutdown" not in called_attributes
    assert "cancel" in called_attributes
    assert "while rclpy.ok() and not node.finished" in source


def test_calibration_launch_uses_independent_safety_and_exclusive_topics() -> None:
    root = Path(__file__).parents[1]
    launch = (root / "launch/calibration_capture_v3.launch.py").read_text()
    ast.parse(launch)
    assert 'executable="safety_supervisor_node"' in launch
    assert 'executable="calibration_excitation_node"' in launch
    assert '"max_steer_rad": 0.25' in launch
    assert '"max_speed_mps": 3.0' in launch
    assert '"nominal_control_cmd", LaunchConfiguration("nominal_topic")' in launch
    assert '"control_cmd", LaunchConfiguration("final_topic")' in launch
    assert "OnProcessExit" in launch and "Shutdown" in launch


def test_ros_package_installs_calibration_entrypoint_and_yaml_dependency() -> None:
    root = Path(__file__).parents[1]
    setup = (root / "setup.py").read_text()
    package = (root / "package.xml").read_text()
    assert "calibration_excitation_node =" in setup
    assert "<exec_depend>python3-yaml</exec_depend>" in package
