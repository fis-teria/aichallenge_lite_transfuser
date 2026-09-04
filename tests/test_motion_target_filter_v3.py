from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from aic_transfuser_lite.data.dataset_view_v3 import (
    MotionTargetFilterConfigV3,
    stationary_commanded_motion_target_v3,
)
from aic_transfuser_lite.training.train_v3 import (
    load_full_control_config_v3,
    motion_target_filter_config_v3,
)


ROOT = Path(__file__).parents[1]


def _future() -> np.ndarray:
    result = np.zeros((30, 8), dtype=np.float32)
    result[:, 0] = np.arange(30, dtype=np.float32) * 0.1
    result[:, 7] = 1.0
    return result


def _enabled() -> MotionTargetFilterConfigV3:
    return MotionTargetFilterConfigV3(enabled=True)


def test_stationary_commanded_motion_target_is_rejected() -> None:
    assert stationary_commanded_motion_target_v3(
        _future(),
        current_speed_mps=0.0,
        commanded_speed_mps=0.75,
        config=_enabled(),
    )


@pytest.mark.parametrize("signal", ["speed", "displacement"])
def test_observed_launch_motion_keeps_target(signal: str) -> None:
    future = _future()
    if signal == "speed":
        future[10, 4] = 0.25
    else:
        future[10, 1] = 0.15
    assert not stationary_commanded_motion_target_v3(
        future,
        current_speed_mps=0.0,
        commanded_speed_mps=0.75,
        config=_enabled(),
    )


def test_genuine_stop_and_moving_anchors_are_not_filtered() -> None:
    future = _future()
    assert not stationary_commanded_motion_target_v3(
        future,
        current_speed_mps=0.0,
        commanded_speed_mps=0.0,
        config=_enabled(),
    )
    assert not stationary_commanded_motion_target_v3(
        future,
        current_speed_mps=0.2,
        commanded_speed_mps=0.75,
        config=_enabled(),
    )


def test_motion_target_filter_config_is_explicit_and_validated(tmp_path: Path) -> None:
    raw = yaml.safe_load(
        (ROOT / "configs/models/trajectory_authoritative_finetune_v3.yaml").read_text()
    )
    parsed = motion_target_filter_config_v3(raw)
    assert parsed.enabled is True
    assert parsed.horizon_steps == 15

    raw["targets"]["motion_target_filter"]["minimum_future_speed_mps"] = 0.0
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="minimum future speed"):
        load_full_control_config_v3(path)


def test_motion_target_filter_rejects_invalid_future_shape() -> None:
    with pytest.raises(ValueError, match=r"\[H,8\]"):
        stationary_commanded_motion_target_v3(
            np.zeros((15, 7), dtype=np.float32),
            current_speed_mps=0.0,
            commanded_speed_mps=0.75,
            config=_enabled(),
        )
