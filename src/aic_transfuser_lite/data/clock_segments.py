from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence


class TimestampSource(str, Enum):
    HEADER = "header"
    MESSAGE_STAMP = "message_stamp"
    BAG_FALLBACK = "bag_fallback_explicit"
    INVALID = "invalid"


@dataclass(frozen=True)
class ResolvedTimestamp:
    semantic_stamp_ns: int | None
    bag_stamp_ns: int
    source: TimestampSource
    fallback_used: bool


@dataclass(frozen=True)
class ClockSample:
    bag_stamp_ns: int
    sim_stamp_ns: int


@dataclass(frozen=True)
class ClockEpoch:
    epoch_id: str
    first_index: int
    last_index: int
    first_bag_stamp_ns: int
    last_bag_stamp_ns: int
    first_sim_stamp_ns: int
    last_sim_stamp_ns: int
    reset_reason: str | None

    def contains_index(self, index: int) -> bool:
        return self.first_index <= int(index) <= self.last_index


def resolve_timestamp(
    *,
    bag_stamp_ns: int,
    header_stamp_ns: int | None,
    message_stamp_ns: int | None,
    allow_bag_fallback: bool,
) -> ResolvedTimestamp:
    """Resolve a semantic timestamp without substituting wall/current time."""

    if bag_stamp_ns < 0:
        raise ValueError("bag_stamp_ns must be non-negative")
    if _valid_stamp(header_stamp_ns):
        return ResolvedTimestamp(
            int(header_stamp_ns), bag_stamp_ns, TimestampSource.HEADER, False
        )
    if _valid_stamp(message_stamp_ns):
        return ResolvedTimestamp(
            int(message_stamp_ns), bag_stamp_ns, TimestampSource.MESSAGE_STAMP, False
        )
    if allow_bag_fallback and bag_stamp_ns > 0:
        return ResolvedTimestamp(
            bag_stamp_ns, bag_stamp_ns, TimestampSource.BAG_FALLBACK, True
        )
    return ResolvedTimestamp(None, bag_stamp_ns, TimestampSource.INVALID, False)


def segment_clock_epochs(
    samples: Sequence[ClockSample], *, max_forward_jump_ns: int
) -> tuple[ClockEpoch, ...]:
    """Split ordered clock samples on reset, jump, or bag-time regression.

    Both timestamps are integer nanoseconds. Epoch IDs are deterministic and
    no returned epoch shares a sample index with another epoch.
    """

    if max_forward_jump_ns <= 0:
        raise ValueError("max_forward_jump_ns must be positive")
    if not samples:
        return ()
    for sample in samples:
        if sample.bag_stamp_ns < 0 or sample.sim_stamp_ns < 0:
            raise ValueError("clock timestamps must be non-negative")

    epochs: list[ClockEpoch] = []
    start_index = 0
    start_reason: str | None = None
    for index in range(1, len(samples)):
        previous = samples[index - 1]
        current = samples[index]
        delta_sim = current.sim_stamp_ns - previous.sim_stamp_ns
        delta_bag = current.bag_stamp_ns - previous.bag_stamp_ns
        reason: str | None = None
        if current.sim_stamp_ns == 0 and previous.sim_stamp_ns > 0:
            reason = "clock_reset_zero"
        elif delta_sim < 0:
            reason = "clock_backward"
        elif delta_sim > max_forward_jump_ns:
            reason = "clock_forward_jump"
        elif delta_bag <= 0:
            reason = "bag_timestamp_non_monotonic"
        if reason is None:
            continue
        epochs.append(_epoch(samples, start_index, index - 1, start_reason, len(epochs)))
        start_index = index
        start_reason = reason
    epochs.append(_epoch(samples, start_index, len(samples) - 1, start_reason, len(epochs)))
    return tuple(epochs)


def epoch_ids_by_sample(
    samples: Sequence[ClockSample], *, max_forward_jump_ns: int
) -> tuple[str, ...]:
    epochs = segment_clock_epochs(samples, max_forward_jump_ns=max_forward_jump_ns)
    result = [""] * len(samples)
    for epoch in epochs:
        for index in range(epoch.first_index, epoch.last_index + 1):
            result[index] = epoch.epoch_id
    return tuple(result)


def require_window_within_epoch(
    epoch_ids: Sequence[str], *, start_index: int, end_index: int
) -> str:
    """Return the window epoch or reject a temporal/future cross-epoch window."""

    if start_index < 0 or end_index < start_index or end_index >= len(epoch_ids):
        raise IndexError("window indices are outside the epoch-id sequence")
    selected = set(epoch_ids[start_index : end_index + 1])
    if "" in selected:
        raise ValueError("window contains an unassigned epoch")
    if len(selected) != 1:
        raise ValueError("temporal or future window crosses a clock epoch")
    return next(iter(selected))


def _epoch(
    samples: Sequence[ClockSample],
    first_index: int,
    last_index: int,
    reason: str | None,
    sequence: int,
) -> ClockEpoch:
    first = samples[first_index]
    last = samples[last_index]
    return ClockEpoch(
        epoch_id=f"epoch{sequence:04d}",
        first_index=first_index,
        last_index=last_index,
        first_bag_stamp_ns=first.bag_stamp_ns,
        last_bag_stamp_ns=last.bag_stamp_ns,
        first_sim_stamp_ns=first.sim_stamp_ns,
        last_sim_stamp_ns=last.sim_stamp_ns,
        reset_reason=reason,
    )


def _valid_stamp(value: int | None) -> bool:
    return value is not None and not isinstance(value, bool) and int(value) > 0 and math.isfinite(float(value))
