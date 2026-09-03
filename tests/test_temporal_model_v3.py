import pytest
import torch

from aic_transfuser_lite.models.encoders.ego_history import EgoHistoryEncoder
from aic_transfuser_lite.models.temporal.gru import (
    MaskedGRUTemporalEncoder,
    select_epoch_history,
    select_epoch_history_before_anchor,
)
from test_full_control_lite_v3_shape import _batch, _model


def test_temporal_lengths_four_and_ten_are_supported() -> None:
    model = _model().eval()
    source = _batch(temporal=4)
    batch = source.__class__(
        image=source.image,
        image_mask=source.image_mask,
        lidar=source.lidar,
        lidar_mask=source.lidar_mask,
        ego=torch.randn(2, 10, 8),
        ego_feature_mask=torch.ones(2, 10, 8, dtype=torch.bool),
        command_history=torch.zeros(2, 10, 3),
        command_mask=torch.ones(2, 10, dtype=torch.bool),
        sensor_dt_sec=source.sensor_dt_sec,
    )
    with torch.no_grad():
        output = model(batch)
    assert output.trajectory_xy.shape == (2, 1, 15, 2)


def test_t1_mode_remains_available() -> None:
    with torch.no_grad():
        assert _model().eval()(_batch()).trajectory_xy.shape == (2, 1, 15, 2)


def test_clock_epoch_boundary_is_left_padded_not_crossed() -> None:
    selected = select_epoch_history([0, 0, 1, 1], anchor_index=2, length=4)
    assert selected.indices == (2, 2, 2, 2)
    assert selected.mask == (False, False, False, True)


def test_causal_command_history_excludes_anchor_and_epoch_boundary() -> None:
    selected = select_epoch_history_before_anchor(
        [0, 0, 1, 1, 1], anchor_index=4, length=4
    )
    assert selected.indices == (2, 2, 2, 3)
    assert selected.mask == (False, False, True, True)
    at_boundary = select_epoch_history_before_anchor(
        [0, 0, 1, 1], anchor_index=2, length=3
    )
    assert at_boundary.indices == (2, 2, 2)
    assert at_boundary.mask == (False, False, False)


@pytest.mark.parametrize(
    ("anchor", "length"), [(-1, 2), (4, 2), (0, 0)]
)
def test_causal_command_history_rejects_invalid_selection(
    anchor: int, length: int
) -> None:
    with pytest.raises(ValueError, match="invalid history selection"):
        select_epoch_history_before_anchor(
            [0, 0, 1, 1], anchor_index=anchor, length=length
        )


def test_masked_steps_do_not_change_temporal_state() -> None:
    torch.manual_seed(1)
    encoder = MaskedGRUTemporalEncoder(3, 5)
    values = torch.randn(1, 4, 3)
    mask = torch.tensor([[True, False, True, True]])
    changed = values.clone()
    changed[:, 1] = 1e6
    torch.testing.assert_close(encoder(values, mask), encoder(changed, mask))


def test_temporal_encoder_rejects_all_invalid_history() -> None:
    with pytest.raises(ValueError, match="at least one"):
        MaskedGRUTemporalEncoder(3, 4)(
            torch.zeros(2, 4, 3), torch.zeros(2, 4, dtype=torch.bool)
        )


def test_ego_history_requires_complete_feature_mask_for_a_step() -> None:
    encoder = EgoHistoryEncoder(2, 4)
    ego = torch.zeros(1, 2, 2)
    feature_mask = torch.tensor([[[True, True], [True, False]]])
    command = torch.zeros(1, 2, 3)
    command_mask = torch.ones(1, 2, dtype=torch.bool)
    output = encoder(ego, feature_mask, command, command_mask)
    assert output.shape == (1, 4)
