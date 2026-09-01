from __future__ import annotations

import pytest
import torch

from aic_transfuser_lite.runtime.inference_core import infer_v1


class DummyStaticV1(torch.nn.Module):
    def __init__(self, extra_head: bool = False) -> None:
        super().__init__()
        self.extra_head = extra_head

    def forward(self, image, lidar, ego):
        batch = image.shape[0]
        output = {
            "waypoints": torch.zeros(batch, 6, 2, device=image.device),
            "target_speed": torch.ones(batch, 1, device=image.device),
        }
        if self.extra_head:
            output["stop_logit"] = torch.zeros(batch, 1, device=image.device)
        return output


def tensors() -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    return (
        torch.zeros(1, 3, 180, 320),
        torch.ones(1, 2, 750),
        torch.zeros(1, 1),
    )


def test_static_v1_inference_returns_only_finite_float32_cpu_outputs() -> None:
    image, lidar, ego = tensors()
    result = infer_v1(
        DummyStaticV1(),
        image=image,
        lidar=lidar,
        ego=ego,
        use_amp=True,
    )
    assert set(result) == {"waypoints", "target_speed"}
    assert result["waypoints"].shape == (1, 6, 2)
    assert all(value.device.type == "cpu" and value.dtype == torch.float32 for value in result.values())


def test_static_v1_inference_rejects_optional_head_drift() -> None:
    image, lidar, ego = tensors()
    with pytest.raises(RuntimeError, match="output contract drifted"):
        infer_v1(
            DummyStaticV1(extra_head=True),
            image=image,
            lidar=lidar,
            ego=ego,
            use_amp=False,
        )
