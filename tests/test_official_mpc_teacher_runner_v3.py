from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "run_official_mpc_teacher_v3.sh"


def test_official_mpc_teacher_runner_uses_sim_time_and_safety_input() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert "-p use_sim_time:=true" in source
    assert "-r /control/command/control_cmd:=/nominal_control_cmd" in source
    assert "-r /control/command/control_cmd_raw:=/teacher_control_cmd_raw" in source


def test_official_mpc_teacher_runner_remaps_control_enable_request() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert (
        "-r control/control_mode_request_topic:=/awsim/control_mode_request_topic"
        in source
    )
