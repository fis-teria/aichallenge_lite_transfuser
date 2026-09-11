from __future__ import annotations

import random

import numpy as np
import pytest
import torch

from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TimeCheckpointIdentity,
    load_time_checkpoint,
    save_time_checkpoint,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig
from aic_transfuser_lite.training.time_config_v1 import build_time_model


def identity() -> TimeCheckpointIdentity:
    return TimeCheckpointIdentity("a" * 64, "b" * 64, "c" * 64, "lineage-v1")


def test_config_defaults_and_typed_failures() -> None:
    config = TimeModelConfig()
    assert config.use_command_history is False
    assert config.future_steps == 30 and config.future_dt_sec == 0.1
    with pytest.raises(TypeError):
        TimeModelConfig(use_command_history=1).validate()  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        TimeModelConfig(future_steps=15).validate()
    with pytest.raises(ValueError):
        TimeCheckpointIdentity("x", "b" * 64, "c" * 64, "l").validate()


def test_checkpoint_roundtrip_and_finetune_separation(tmp_path) -> None:
    config = TimeModelConfig(image_height=32, image_width=32, lidar_points=32,
                             hidden_dim=16, camera_tokens_hw=(2, 2), lidar_tokens=4,
                             fusion_depth=1, fusion_heads=4)
    model = build_time_model(config)
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.01)
    from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
    mask = torch.ones(1, 4, dtype=torch.bool)
    batch = ModelBatchV3(torch.randn(1, 4, 3, 32, 32), mask,
                         torch.rand(1, 4, 2, 32), mask,
                         torch.zeros(1, 10, 4), torch.ones(1, 10, 4, dtype=torch.bool),
                         torch.zeros(1, 10, 3), torch.zeros(1, 10, dtype=torch.bool),
                         torch.zeros(1, 4, 2), requested_outputs=frozenset({"trajectory"}))
    model(batch).sum().backward(); optimizer.step()
    before = model.state_dict()["delta_head.weight"].clone()
    random.seed(4); np.random.seed(4); torch.manual_seed(4)
    path = tmp_path / "time.pt"
    save_time_checkpoint(path, config=config, identity=identity(), model=model,
                          optimizer=optimizer, scheduler=None, epoch=2, global_step=7)
    expected_random = (random.random(), float(np.random.rand()), torch.rand(1))
    epoch, step = load_time_checkpoint(path, config=config, identity=identity(), model=model,
                                       optimizer=optimizer, mode="resume")
    assert (epoch, step) == (2, 7)
    assert torch.equal(before, model.state_dict()["delta_head.weight"])
    actual_random = (random.random(), float(np.random.rand()), torch.rand(1))
    assert actual_random[0] == expected_random[0]
    assert actual_random[1] == expected_random[1]
    torch.testing.assert_close(actual_random[2], expected_random[2])
    assert load_time_checkpoint(path, config=config, identity=identity(), model=model,
                                mode="finetune") == (0, 0)
    # The checkpoint marks an optimizer boundary: the next identical update
    # must match a fresh model resumed from that boundary.
    resumed = build_time_model(config)
    resumed_optimizer = torch.optim.AdamW(resumed.parameters(), lr=0.01)
    load_time_checkpoint(path, config=config, identity=identity(), model=resumed,
                         optimizer=resumed_optimizer, mode="resume")
    model.eval(); resumed.eval()
    optimizer.zero_grad(); resumed_optimizer.zero_grad()
    model(batch).sum().backward(); resumed(batch).sum().backward()
    optimizer.step(); resumed_optimizer.step()
    for left, right in zip(model.parameters(), resumed.parameters()):
        torch.testing.assert_close(left, right, rtol=0, atol=0)
    with pytest.raises(ValueError):
        load_time_checkpoint(path, config=TimeModelConfig(command_history_length=9),
                             identity=identity(), model=model, mode="finetune")
