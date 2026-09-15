"""Offline duration proof from published commands; no actuator-time inference."""
from __future__ import annotations

from typing import Sequence


def longest_published_hold_s(samples: Sequence[tuple[int, int, int, bool]]) -> float:
    """Samples [N,4]: (sim ns, monotonic publication ns, sequence, full hold).

    The last command lasts until the next evidenced publication. Equal sim
    stamps add zero time, and do not interrupt unchanged commands. Both clocks
    must be continuous within 150 ms, with no missing publication sequence.
    No duration is extrapolated after the last sample or across a telemetry gap.
    """
    previous = None
    since = None
    longest_ns = 0
    for sample in samples:
        if (len(sample) != 4 or any(type(v) is not int or v < 0 for v in sample[:3])
                or sample[2] < 1 or type(sample[3]) is not bool):
            raise ValueError('PUBLISHED_HOLD_SAMPLE_SHAPE_UNITS')
        sim, wall, sequence, full = sample
        if previous is not None:
            psim, pwall, psequence, pfull = previous
            if sim < psim or wall <= pwall or sequence <= psequence:
                raise ValueError('PUBLISHED_HOLD_ORDER')
            continuous = (sim-psim <= 150_000_000 and wall-pwall <= 150_000_000
                          and sequence == psequence+1)
            if continuous and pfull and since is not None:
                longest_ns = max(longest_ns, sim-since)
            else:
                since = None
        if full:
            if since is None:
                since = sim
        else:
            since = None
        previous = sample
    return longest_ns/1e9
