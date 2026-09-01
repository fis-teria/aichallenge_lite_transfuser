from __future__ import annotations

from pathlib import Path

import yaml


EXPECTED_CHECKPOINT_SHA256 = (
    "1b82e33aa676ccc433a66781658ba9a919d88de34df6c0bc6948738e130dbb84"
)


def test_v1_runtime_parameters_are_explicit_and_fail_closed() -> None:
    root = Path(__file__).resolve().parents[1]
    parameters = yaml.safe_load((root / "config" / "runtime.v1.param.yaml").read_text())["/**"][
        "ros__parameters"
    ]
    assert parameters["expected_checkpoint_sha256"] == EXPECTED_CHECKPOINT_SHA256
    assert parameters["inference_hz"] == 10.0
    assert parameters["sync_queue_size"] == 10
    assert parameters["max_sensor_skew_ms"] == 30.0
    assert parameters["expected_lidar_frame"] == "lidar"
    assert parameters["estimated_delay_sec"] == 0.0
    assert parameters["enable_model_stop"] is False


def test_v1_launch_is_additive_and_never_uses_legacy_checkpoint_name() -> None:
    root = Path(__file__).resolve().parents[1]
    launch = (root / "launch" / "transfuser_lite_v1.launch.py").read_text()
    assert "inference_node_v1" in launch
    assert "transfuser_lite_v1_best_ade.pt" in launch
    assert '"best.pt"' not in launch
    assert "velocity_status" in launch
    assert "steering_status" in launch
