from __future__ import annotations

import pytest

from aic_transfuser_lite.data.clock_segments import (
    ClockSample,
    TimestampSource,
    epoch_ids_by_sample,
    require_window_within_epoch,
    resolve_timestamp,
    segment_clock_epochs,
)


def _samples(sim_stamps: list[int], bag_stamps: list[int] | None = None):
    bags = bag_stamps or list(range(100, 100 + len(sim_stamps)))
    return [ClockSample(bag, sim) for bag, sim in zip(bags, sim_stamps)]


@pytest.mark.parametrize(
    ("samples", "reason"),
    [
        (_samples([10, 20, 15]), "clock_backward"),
        (_samples([10, 20, 0]), "clock_reset_zero"),
        (_samples([10, 20, 200]), "clock_forward_jump"),
        (_samples([10, 20, 30], [100, 101, 101]), "bag_timestamp_non_monotonic"),
    ],
)
def test_clock_epoch_split_reasons(samples, reason: str) -> None:
    epochs = segment_clock_epochs(samples, max_forward_jump_ns=100)
    assert len(epochs) == 2
    assert epochs[1].reset_reason == reason
    assert epochs[0].last_index + 1 == epochs[1].first_index


def test_temporal_and_future_windows_cannot_cross_epochs() -> None:
    samples = _samples([10, 20, 0, 10])
    ids = epoch_ids_by_sample(samples, max_forward_jump_ns=100)
    assert require_window_within_epoch(ids, start_index=0, end_index=1) == "epoch0000"
    with pytest.raises(ValueError, match="crosses a clock epoch"):
        require_window_within_epoch(ids, start_index=1, end_index=2)


def test_timestamp_resolution_is_ordered_and_never_uses_current_time() -> None:
    header = resolve_timestamp(
        bag_stamp_ns=30,
        header_stamp_ns=10,
        message_stamp_ns=20,
        allow_bag_fallback=True,
    )
    assert header.source is TimestampSource.HEADER
    message = resolve_timestamp(
        bag_stamp_ns=30,
        header_stamp_ns=0,
        message_stamp_ns=20,
        allow_bag_fallback=True,
    )
    assert message.source is TimestampSource.MESSAGE_STAMP
    bag = resolve_timestamp(
        bag_stamp_ns=30,
        header_stamp_ns=None,
        message_stamp_ns=None,
        allow_bag_fallback=True,
    )
    assert bag.source is TimestampSource.BAG_FALLBACK and bag.fallback_used
    invalid = resolve_timestamp(
        bag_stamp_ns=30,
        header_stamp_ns=None,
        message_stamp_ns=None,
        allow_bag_fallback=False,
    )
    assert invalid.source is TimestampSource.INVALID
    assert invalid.semantic_stamp_ns is None


@pytest.mark.parametrize("threshold", [0, -1])
def test_clock_threshold_must_be_positive(threshold: int) -> None:
    with pytest.raises(ValueError, match="positive"):
        segment_clock_epochs(_samples([1]), max_forward_jump_ns=threshold)


def test_window_indices_are_validated() -> None:
    with pytest.raises(IndexError):
        require_window_within_epoch(("epoch0000",), start_index=0, end_index=1)
