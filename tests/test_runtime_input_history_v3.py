from __future__ import annotations

import pytest
import torch

from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3
from aic_transfuser_lite.runtime.input_history_v3 import (
    RuntimeObservationHistoryV3,
    RuntimeObservationTensorV3,
    build_runtime_temporal_batch_v3,
)
from aic_transfuser_lite.runtime.residual_control import ExternalControllerCommand


def _observation(stamp_sec: float, value: float) -> RuntimeObservationTensorV3:
    return RuntimeObservationTensorV3(
        stamp_sec=stamp_sec,
        image=torch.full((3, 16, 16), value),
        lidar=torch.full((2, 16), value),
        ego=torch.tensor([value, 0.0, 0.0, 0.0]),
        sensor_dt_sec=torch.tensor([0.0, value * 0.001]),
    )


def _model() -> FullControlLiteV3:
    return FullControlLiteV3(
        image_height=16,
        image_width=16,
        lidar_points=16,
        ego_dim=4,
        hidden_dim=16,
        camera_tokens_hw=(1, 1),
        lidar_tokens=2,
        fusion_depth=1,
        fusion_heads=4,
        max_sensor_history=4,
        max_ego_history=10,
    )


def test_runtime_warmup_matches_training_left_padding_and_model_output() -> None:
    observations = tuple(_observation(1.0 + index * 0.1, float(index + 1)) for index in range(3))
    commands = (
        ExternalControllerCommand(0.1, 0.75, 0.2),
        ExternalControllerCommand(0.2, 0.75, 0.3),
    )
    runtime = build_runtime_temporal_batch_v3(
        observations,
        commands,
        sensor_history_length=4,
        ego_history_length=10,
        command_history_length=10,
        requested_outputs=frozenset({"trajectory", "speed_profile"}),
    )

    assert runtime.image.shape == (1, 4, 3, 16, 16)
    assert runtime.lidar.shape == (1, 4, 2, 16)
    assert runtime.ego.shape == (1, 10, 4)
    assert runtime.command_history.shape == (1, 10, 3)
    assert runtime.image_mask.tolist() == [[False, True, True, True]]
    assert runtime.lidar_mask.tolist() == [[False, True, True, True]]
    assert runtime.ego_feature_mask[:, :, 0].tolist() == [
        [False] * 7 + [True, True, True]
    ]
    assert runtime.command_mask.tolist() == [[False] * 8 + [True, True]]
    torch.testing.assert_close(runtime.image[0, 0], observations[0].image)
    torch.testing.assert_close(runtime.ego[0, 0], observations[0].ego)

    # This independently assembled batch is the Dataset V3 selection contract:
    # current-inclusive sensor/ego history and commands strictly before anchor.
    expected = runtime.__class__(
        image=torch.stack(
            [observations[0].image, *(item.image for item in observations)]
        )[None],
        image_mask=torch.tensor([[False, True, True, True]]),
        lidar=torch.stack(
            [observations[0].lidar, *(item.lidar for item in observations)]
        )[None],
        lidar_mask=torch.tensor([[False, True, True, True]]),
        ego=torch.stack([observations[0].ego] * 7 + [item.ego for item in observations])[None],
        ego_feature_mask=torch.tensor(
            [[[False] * 4] * 7 + [[True] * 4] * 3], dtype=torch.bool
        ),
        command_history=torch.tensor(
            [[[0.0, 0.0, 0.0]] * 8 + [[0.1, 0.75, 0.2], [0.2, 0.75, 0.3]]]
        ),
        command_mask=torch.tensor([[False] * 8 + [True, True]]),
        sensor_dt_sec=torch.stack(
            [
                observations[0].sensor_dt_sec,
                *(item.sensor_dt_sec for item in observations),
            ]
        )[None],
        requested_outputs=frozenset({"trajectory", "speed_profile"}),
    )
    for name in (
        "image",
        "image_mask",
        "lidar",
        "lidar_mask",
        "ego",
        "ego_feature_mask",
        "command_history",
        "command_mask",
        "sensor_dt_sec",
    ):
        torch.testing.assert_close(getattr(runtime, name), getattr(expected, name))

    model = _model().eval()
    with torch.no_grad():
        runtime_output = model(runtime)
        expected_output = model(expected)
    torch.testing.assert_close(
        runtime_output.trajectory_xy, expected_output.trajectory_xy
    )
    torch.testing.assert_close(
        runtime_output.trajectory_speed_mps,
        expected_output.trajectory_speed_mps,
    )


def test_runtime_history_resets_on_gap_and_timestamp_regression() -> None:
    history = RuntimeObservationHistoryV3(maximum_length=10, maximum_gap_sec=0.5)
    assert history.append(_observation(1.0, 1.0)) is None
    assert history.append(_observation(1.1, 2.0)) is None
    assert history.append(_observation(2.0, 3.0)) == "timestamp_gap"
    assert len(history.values) == 1
    assert history.values[0].stamp_sec == 2.0
    assert history.append(_observation(1.9, 4.0)) == "timestamp_non_monotonic"
    assert len(history.values) == 1
    assert history.values[0].stamp_sec == 1.9


def test_runtime_history_rejects_missing_or_nonfinite_input() -> None:
    with pytest.raises(ValueError, match="current observation"):
        build_runtime_temporal_batch_v3(
            (),
            (),
            sensor_history_length=4,
            ego_history_length=10,
            command_history_length=10,
            requested_outputs=frozenset({"trajectory", "speed_profile"}),
        )
    bad = _observation(1.0, 1.0)
    bad.image[0, 0, 0] = float("nan")
    with pytest.raises(ValueError, match="image must be finite"):
        bad.validate()


def test_runtime_command_history_is_strictly_past_only() -> None:
    commands_before_anchor = [ExternalControllerCommand(0.1, 0.75, 0.2)]
    first = build_runtime_temporal_batch_v3(
        (_observation(1.0, 1.0),),
        commands_before_anchor,
        sensor_history_length=4,
        ego_history_length=10,
        command_history_length=10,
        requested_outputs=frozenset({"trajectory", "speed_profile"}),
    )
    assert first.command_mask.sum().item() == 1
    torch.testing.assert_close(
        first.command_history[0, -1], torch.tensor([0.1, 0.75, 0.2])
    )
    # A command produced for this anchor is appended only after inference and
    # can first appear in the next anchor's input.
    commands_before_anchor.append(ExternalControllerCommand(0.2, 0.75, 0.3))
    second = build_runtime_temporal_batch_v3(
        (_observation(1.0, 1.0), _observation(1.1, 2.0)),
        commands_before_anchor,
        sensor_history_length=4,
        ego_history_length=10,
        command_history_length=10,
        requested_outputs=frozenset({"trajectory", "speed_profile"}),
    )
    assert second.command_mask.sum().item() == 2
