"""Raw event preservation and shared offline/runtime causal slot selection."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence
import numpy as np
from .clock_segments import ClockEpoch
from .mcap_converter_v2 import read_run_messages_v2

@dataclass(frozen=True)
class TimeEvent:
    role: str
    run: str
    epoch: str
    capture_clock: str
    available_clock: str
    capture_ns: int
    available_ns: int
    sequence: int
    payload: Any
    availability_source: str = "explicit"

    def __post_init__(self) -> None:
        if not all((self.role, self.run, self.epoch, self.capture_clock, self.available_clock)):
            raise ValueError("event identity required")
        if any(type(v) is not int or v < 0 for v in (self.capture_ns, self.available_ns, self.sequence)):
            raise ValueError("event times/sequence must be nonnegative integer ns")


def read_time_events(bag: Path, *, run: str, epochs: Sequence[ClockEpoch],
                     capture_clock: str) -> tuple[TimeEvent, ...]:
    """Preserve all decoded events before legacy dedup. Receipt is an explicit proxy.

    Epoch assignment uses bag receipt intervals, not overlapping reset sensor times.
    Production availability requires additional processing-delay evidence.
    """
    result = []
    def sink(role: str, item: Any, sequence: int) -> None:
        matches = [e for e in epochs if e.first_bag_stamp_ns <= item.bag_timestamp_ns <= e.last_bag_stamp_ns]
        if len(matches) != 1:
            raise ValueError("event receipt has no unique epoch")
        result.append(TimeEvent(role, run, matches[0].epoch_id, capture_clock, "bag_receipt",
            int(item.timestamp_ns), int(item.bag_timestamp_ns), sequence, item, "bag_receipt_proxy"))
    read_run_messages_v2(bag, event_sink=sink,
        optional_roles=frozenset({"velocity", "nominal_command", "final_command", "gear", "actual_steering"}))
    return tuple(result)


def select_time_history(events: Sequence[TimeEvent], *, role: str, run: str, epoch: str,
                        capture_clock: str, available_clock: str, freeze_ns: int,
                        observation_ns: int, length: int, tolerance_ns: int,
                        previous_only: bool = False) -> tuple[TimeEvent | None, ...]:
    """100 ms fixed slots; eligibility BEFORE last-arrival duplicate resolution.

    Past-only acquisition per slot. Missing slots stay missing; no time compression.
    Offline and online callers use this same function and clock-domain cut.
    """
    if (type(length) is not int or not 1 <= length <= 10 or
        any(type(v) is not int or v < 0 for v in (freeze_ns, observation_ns, tolerance_ns))):
        raise ValueError("invalid history settings")
    eligible = {}
    for event in events:
        if (event.role, event.run, event.epoch, event.capture_clock, event.available_clock) != (
                role, run, epoch, capture_clock, available_clock):
            continue
        if event.available_ns > freeze_ns:
            continue
        previous = eligible.get(event.capture_ns)
        if previous is None or (event.available_ns, event.sequence) > (previous.available_ns, previous.sequence):
            eligible[event.capture_ns] = event
    slots = []
    for i in range(length):
        slot = observation_ns - (length - 1 - i + int(previous_only)) * 100_000_000
        candidates = [e for t,e in eligible.items() if 0 <= slot-t <= tolerance_ns]
        slots.append(max(candidates, key=lambda e:e.capture_ns) if candidates else None)
    return tuple(slots)


def materialize_history(slots: Sequence[TimeEvent | None], encode: Callable[[Any], np.ndarray],
                        *, shape: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray]:
    """Shared tensor-value construction [T,*shape], mask[T]; never encode missing data."""
    values = np.zeros((len(slots), *shape), dtype=np.float32)
    mask = np.zeros(len(slots), dtype=bool)
    for i,event in enumerate(slots):
        if event is not None:
            value = np.asarray(encode(event.payload), dtype=np.float32)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError("invalid encoded history payload")
            values[i] = value
            mask[i] = True
    return values, mask
