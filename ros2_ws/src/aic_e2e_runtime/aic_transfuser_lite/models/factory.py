from __future__ import annotations

from typing import Any

from torch import nn

from .late_fusion import LateFusionModel
from .lidar_only import LidarOnlyModel
from .transfuser_lite import AICTransFuserLite
from .transfuser_lite_v1 import AICTransFuserLiteV1


def build_model(config: dict[str, Any]) -> nn.Module:
    name = str(config["model"]["name"])
    if name == "transfuser_lite":
        if (
            config.get("schema_version") == "transfuser_lite_v1"
            and int(config.get("data", {}).get("format_version", 1)) == 2
        ):
            return AICTransFuserLiteV1(config)
        return AICTransFuserLite(config)
    if name == "lidar_only":
        return LidarOnlyModel(config)
    if name == "late_fusion":
        return LateFusionModel(config)
    raise ValueError(f"Unsupported model name={name!r}")
