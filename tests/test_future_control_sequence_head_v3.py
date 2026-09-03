import pytest
import torch

from aic_transfuser_lite.models.heads.control_sequence import FutureControlSequenceHead


def test_future_control_sequence_has_physical_shape_and_rate_bounds() -> None:
    head = FutureControlSequenceHead(
        8,
        steps=10,
        control_dt_sec=0.1,
        max_steering_rad=0.6,
        max_steering_rate_radps=0.8,
        max_speed_mps=3.0,
        min_acceleration_mps2=-4.0,
        max_acceleration_mps2=2.0,
        min_jerk_mps3=-8.0,
        max_jerk_mps3=4.0,
    )
    with torch.no_grad():
        head.projection.weight.zero_()
        head.projection.bias.fill_(100.0)
    initial = torch.tensor([[0.59, 2.9, 1.9], [-0.59, 0.0, -3.9]])
    output = head(torch.zeros(2, 8), initial)
    assert output.shape == (2, 1, 10, 3)
    assert torch.isfinite(output).all()
    assert (output[..., 0].abs() <= 0.6).all()
    assert ((output[..., 1] >= 0.0) & (output[..., 1] <= 3.0)).all()
    assert ((output[..., 2] >= -4.0) & (output[..., 2] <= 2.0)).all()
    steering_with_initial = torch.cat((initial[:, None, None, 0], output[..., 0]), dim=2)
    accel_with_initial = torch.cat((initial[:, None, None, 2], output[..., 2]), dim=2)
    assert (torch.diff(steering_with_initial, dim=2).abs() <= 0.080001).all()
    assert (torch.diff(accel_with_initial, dim=2) <= 0.400001).all()
    assert (torch.diff(accel_with_initial, dim=2) >= -0.800001).all()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"control_dt_sec": 0.0},
        {"max_steering_rate_radps": 0.0},
        {"min_jerk_mps3": 0.0},
        {"max_acceleration_mps2": float("inf")},
    ],
)
def test_future_control_sequence_rejects_non_authoritative_limits(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        FutureControlSequenceHead(8, **kwargs)


def test_future_control_sequence_rejects_nonfinite_initial_state() -> None:
    head = FutureControlSequenceHead(8)
    with pytest.raises(ValueError, match="initial control"):
        head(torch.zeros(1, 8), torch.tensor([[0.0, float("nan"), 0.0]]))
