from pathlib import Path

import yaml


def test_full_control_launch_keeps_safety_as_only_final_publisher() -> None:
    root = Path(__file__).parents[1]
    package = root / "ros2_ws/src/aic_e2e_runtime"
    source = (package / "aic_e2e_runtime/inference_node_v3.py").read_text()
    launch = (package / "launch/transfuser_lite_v3_full_control_trial.launch.py").read_text()
    params = yaml.safe_load(
        (package / "config/runtime.v3.full_control_trial.param.yaml").read_text()
    )["/**"]["ros__parameters"]

    assert 'RuntimeProfile.FULL_CONTROL' in source
    assert '"nominal_control_cmd"' in source
    assert '"control_sequence"' in source
    assert "choose_full_control_or_same_trajectory_fallback(" in source
    assert "project_model_control_sequence(" in source
    assert "rollout_actuator_bicycle(" in source
    assert 'executable="safety_supervisor_node"' in launch
    assert '("control_cmd", "/control/command/control_cmd")' in launch
    assert 'executable="inference_node_v3"' in launch
    assert 'condition=IfCondition(LaunchConfiguration("launch_rviz"))' in launch
    assert params["runtime_profile"] == "full_control"
    assert params["max_speed_mps"] == 0.8
    assert params["trial_speed_cap_mps"] == 0.8
    assert params["safety_supervisor_ready"] is True


def test_trial_authorization_is_explicitly_limited_to_awsim() -> None:
    root = Path(__file__).parents[1]
    authorization = yaml.safe_load(
        (root / "configs/runtime/v3_full_control_trial_authorization.yaml").read_text()
    )
    assert authorization["scope"] == {
        "environment": "AWSIM",
        "host": "graneple@192.168.3.10",
        "deployment_stage": "limited_odd_trial",
        "maximum_speed_mps": 0.8,
        "route": "d1",
    }
    assert authorization["requirements"]["safety_supervisor_final_authority"] is True
