from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.data.mcap_converter_v2 import select_regular_grid


def test_regular_grid_holds_ten_hz_under_camera_jitter_without_reuse() -> None:
    jitter_ms = np.array([0, 1, -1, 1, 0, -1, 1, -1, 0, 1] * 5)
    timestamps_ns = [
        int(index * 100_000_000 + jitter_ms[index] * 1_000_000)
        for index in range(50)
    ]

    matches = select_regular_grid(
        timestamps_ns,
        sample_rate_hz=10.0,
        tolerance_ms=40.0,
        origin_ns=0,
    )

    assert len(matches) == 50
    assert len({match.source_index for match in matches}) == len(matches)
    assert all(
        right.target_timestamp_ns - left.target_timestamp_ns == 100_000_000
        for left, right in zip(matches, matches[1:])
    )
    assert max(abs(match.delta_ns) for match in matches) <= 1_000_000
    effective_hz = (len(matches) - 1) / (
        (matches[-1].target_timestamp_ns - matches[0].target_timestamp_ns) / 1e9
    )
    assert effective_hz == pytest.approx(10.0)


def test_regular_grid_never_reuses_one_camera_for_two_targets() -> None:
    matches = select_regular_grid(
        [0, 149_000_000, 300_000_000],
        sample_rate_hz=10.0,
        tolerance_ms=60.0,
        origin_ns=0,
    )

    assert [match.source_index for match in matches] == [0, 1, 2]
    assert [match.target_timestamp_ns for match in matches] == [0, 100_000_000, 300_000_000]


def test_regular_grid_rejects_non_monotonic_timestamps() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        select_regular_grid([0, 100, 100], sample_rate_hz=10.0, tolerance_ms=40.0)
