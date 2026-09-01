from __future__ import annotations

from pathlib import Path

import pytest

from aic_transfuser_lite.config import load_v1_config
from aic_transfuser_lite.models.registry import Registry, register_v1_factory
from aic_transfuser_lite.models.transfuser_lite_v1 import AICTransFuserLiteV1


ROOT = Path(__file__).parents[1]


def test_registry_builds_and_lists_without_import_side_effects() -> None:
    registry: Registry[dict] = Registry("test")
    registry.register("b", lambda value: {"value": value})
    registry.register("a", lambda value: {"value": value + 1})
    assert registry.names() == ("a", "b")
    assert registry.build("b", value=3) == {"value": 3}


def test_registry_rejects_empty_duplicate_and_unknown_names() -> None:
    registry: Registry[object] = Registry("test")
    with pytest.raises(ValueError, match="non-empty"):
        registry.register("", object)
    registry.register("one", object)
    with pytest.raises(ValueError, match="duplicate"):
        registry.register("one", object)
    with pytest.raises(KeyError, match="available=.*one"):
        registry.build("missing")


def test_v1_factory_path_remains_supported() -> None:
    config = load_v1_config(ROOT / "configs/transfuser_lite_v1_static.yaml")
    config["model"]["camera"]["pretrained"] = False
    registry = register_v1_factory(Registry("model"))
    model = registry.build("transfuser_lite_v1", config=config)
    assert isinstance(model, AICTransFuserLiteV1)
