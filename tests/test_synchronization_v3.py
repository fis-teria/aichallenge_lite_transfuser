from __future__ import annotations

import math

import pytest

from aic_transfuser_lite.data.synchronization_v3 import (
    IndexedTimedValues,
    TimedValue,
    angle_interpolate,
    causal_previous,
    exact_events,
    linear_interpolate,
    nearest,
)


def test_nearest_returns_signed_delta_and_prefers_past_on_tie() -> None:
    stream = [TimedValue(90, "past"), TimedValue(110, "future")]
    result = nearest(stream, target_ns=100, tolerance_ns=10)
    assert result.valid and result.value == "past"
    assert result.source_stamps_ns == (90,)
    assert result.signed_delta_ns == -10
    assert result.age_ns == 10


def test_nearest_tolerance_boundary_is_inclusive() -> None:
    stream = [TimedValue(90, 1)]
    assert nearest(stream, target_ns=100, tolerance_ns=10).valid
    rejected = nearest(stream, target_ns=101, tolerance_ns=10)
    assert not rejected.valid and rejected.reason == "outside_tolerance"


def test_causal_previous_never_uses_future_command() -> None:
    future_only = causal_previous(
        [TimedValue(101, "future")], target_ns=100, max_age_ns=50
    )
    assert not future_only.valid and future_only.reason == "future_only"
    stream = [TimedValue(90, "old"), TimedValue(100, "current"), TimedValue(101, "future")]
    selected = causal_previous(stream, target_ns=100, max_age_ns=10)
    assert selected.value == "current" and selected.signed_delta_ns == 0


def test_linear_interpolation_requires_bracket_and_finite_values() -> None:
    result = linear_interpolate(
        [TimedValue(0, 0.0), TimedValue(10, 20.0)],
        target_ns=5,
        tolerance_ns=5,
    )
    assert result.valid and result.value == pytest.approx(10.0)
    assert result.source_stamps_ns == (0, 10)
    assert result.signed_delta_ns == -5 and result.age_ns == 5
    assert not linear_interpolate(
        [TimedValue(0, 0.0)], target_ns=5, tolerance_ns=5
    ).valid
    assert not linear_interpolate(
        [TimedValue(0, float("nan")), TimedValue(10, 1.0)],
        target_ns=5,
        tolerance_ns=5,
    ).valid


def test_angle_interpolation_uses_shortest_wrap() -> None:
    result = angle_interpolate(
        [TimedValue(0, math.radians(179)), TimedValue(10, math.radians(-179))],
        target_ns=5,
        tolerance_ns=5,
    )
    assert result.valid
    assert abs(abs(float(result.value)) - math.pi) < 1e-6


def test_exact_events_preserves_all_events_and_interval_semantics() -> None:
    stream = [TimedValue(10, "a"), TimedValue(20, "b"), TimedValue(30, "c")]
    assert exact_events(stream, start_exclusive_ns=10, end_inclusive_ns=30) == (
        TimedValue(20, "b"),
        TimedValue(30, "c"),
    )
    assert exact_events(stream, start_exclusive_ns=30, end_inclusive_ns=30) == ()


def test_indexed_stream_matches_sequence_synchronization_results() -> None:
    values = [TimedValue(0, 0.0), TimedValue(10, 20.0), TimedValue(20, 40.0)]
    indexed = IndexedTimedValues.from_values(values)
    assert indexed.stamps_ns == (0, 10, 20)
    assert linear_interpolate(indexed, target_ns=5, tolerance_ns=5) == linear_interpolate(
        values, target_ns=5, tolerance_ns=5
    )
    assert nearest(indexed, target_ns=14, tolerance_ns=6) == nearest(
        values, target_ns=14, tolerance_ns=6
    )
    assert causal_previous(indexed, target_ns=14, max_age_ns=5) == causal_previous(
        values, target_ns=14, max_age_ns=5
    )


def test_indexed_stream_rejects_duplicate_timestamps_at_construction() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        IndexedTimedValues.from_values([TimedValue(10, 1), TimedValue(10, 2)])
    with pytest.raises(ValueError, match="must match"):
        IndexedTimedValues((TimedValue(10, 1),), (11,))


@pytest.mark.parametrize(
    "stream",
    [
        [TimedValue(10, 1), TimedValue(10, 2)],
        [TimedValue(10, 1), TimedValue(9, 2)],
    ],
)
def test_all_policies_reject_non_monotonic_streams(stream) -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        nearest(stream, target_ns=10, tolerance_ns=1)
