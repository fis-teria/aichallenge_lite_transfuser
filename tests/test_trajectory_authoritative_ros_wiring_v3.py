from pathlib import Path

import yaml

from aic_transfuser_lite.runtime.output_profiles import output_profile


def test_trajectory_authoritative_profile_demotes_model_sequence_to_shadow() -> None:
    profile = output_profile("trajectory_authoritative")
    assert profile.nominal_control_authority
    assert {"trajectory", "speed_profile"}.issubset(profile.requested_outputs)
    assert "nominal_control_cmd" in profile.publisher_topics
    assert "shadow_model_control_sequence" in profile.publisher_topics
    assert "control_sequence" in profile.requested_outputs


def test_trajectory_authoritative_launch_keeps_safety_as_sole_final_publisher() -> None:
    root = Path(__file__).parents[1]
    package = root / "ros2_ws/src/aic_e2e_runtime"
    source = (package / "aic_e2e_runtime/inference_node_v3.py").read_text()
    launch = (
        package / "launch/transfuser_lite_v3_trajectory_authoritative.launch.py"
    ).read_text()
    params = yaml.safe_load(
        (package / "config/runtime.v3.trajectory_authoritative.param.yaml").read_text()
    )["/**"]["ros__parameters"]

    assert params["runtime_profile"] == "trajectory_authoritative"
    assert params["executable_reference_odd_speed_cap_mps"] == 0.75
    assert params["max_speed_mps"] == 0.75
    assert params["speed_limit_guard_margin_mps"] == 0.10
    assert params["minimum_lookahead_distance_m"] == 1.0
    assert params["executable_reference_require_stop_probability"] is False
    assert params["expected_drive_gear"] == 2
    assert params["expected_autonomous_mode"] == 1
    assert params["allowed_awsim_states"] == ["Start", "Ready"]
    assert params["race_arm_topic"] == "/overtake/race_armed"
    assert params["launch_assist_acceleration_floor_mps2"] == 0.5
    assert params["speed_ki"] > 0.0
    assert 'executable="inference_node_v3"' in launch
    assert 'executable="safety_supervisor_node"' in launch
    assert '("control_cmd", "/control/command/control_cmd")' in launch
    assert 'executable="rviz2"' in launch
    assert "build_executable_reference_v3(" in source
    assert "control_from_executable_reference_v3(" in source
    assert "fail_closed_stop_control_v3(" in source
    assert "evaluate_control_preflight_v3(" in source
    assert "self._on_race_armed" in source
    assert "LongitudinalControllerV3(" in source
    assert "self.count_publishers(final_topic)" in source
    assert "self.count_subscribers(final_topic)" in source
    assert "full_control authority is disabled" in source
