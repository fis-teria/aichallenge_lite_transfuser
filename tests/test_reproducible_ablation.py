from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import torch

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.training.train import order_sha256, seeded_random_sampler


ROOT = Path(__file__).resolve().parents[1]


def component_seeded_config(ego_dim: int) -> dict:
    config = deepcopy(load_config(ROOT / "configs/transfuser_lite_v0.yaml"))
    config["model"]["initialization"] = "component_seeded_v1"
    config["data"]["ego_dim"] = ego_dim
    return config


def test_component_seeded_model_is_independent_of_global_rng() -> None:
    config = component_seeded_config(5)
    torch.manual_seed(1)
    first = build_model(config).state_dict()
    torch.manual_seed(999)
    second = build_model(config).state_dict()
    assert first.keys() == second.keys()
    assert all(torch.equal(first[key], second[key]) for key in first)


def test_shared_components_match_when_ego_input_dimension_changes() -> None:
    five = build_model(component_seeded_config(5)).state_dict()
    one = build_model(component_seeded_config(1)).state_dict()
    assert five.keys() == one.keys()
    changed = {"ego.network.0.weight", "ego.network.0.bias"}
    for key in five:
        if key in changed:
            continue
        assert five[key].shape == one[key].shape, key
        assert torch.equal(five[key], one[key]), key
    assert five["ego.network.0.weight"].shape[1] == 5
    assert one["ego.network.0.weight"].shape[1] == 1


def test_seeded_sampler_preview_is_replayed_and_rng_independent() -> None:
    dataset = list(range(100))
    first_sampler, preview = seeded_random_sampler(dataset, 42)
    assert [int(index) for index in first_sampler] == preview
    torch.manual_seed(999)
    torch.rand(1000)
    second_sampler, second_preview = seeded_random_sampler(dataset, 42)
    assert second_preview == preview
    assert [int(index) for index in second_sampler] == preview
    assert order_sha256(second_preview) == order_sha256(preview)


def test_legacy_initialization_still_loads_v0_checkpoint_strictly() -> None:
    checkpoint = torch.load(
        ROOT / "runs/transfuser_lite_v0/best.pt",
        map_location="cpu",
        weights_only=False,
    )
    model = build_model(checkpoint["config"])
    model.load_state_dict(checkpoint["model"], strict=True)


def test_unknown_initialization_strategy_is_rejected() -> None:
    config = component_seeded_config(5)
    config["model"]["initialization"] = "mystery"
    try:
        build_model(config)
    except ValueError as error:
        assert "model.initialization" in str(error)
    else:
        raise AssertionError("unknown initialization strategy was accepted")
