from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from typing import Generic, Sequence, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class TimedValue(Generic[T]):
    stamp_ns: int
    value: T


@dataclass(frozen=True)
class SyncResult(Generic[T]):
    value: T | None
    valid: bool
    source_stamps_ns: tuple[int, ...]
    signed_delta_ns: int | None
    age_ns: int | None
    reason: str


def nearest(
    stream: Sequence[TimedValue[T]], *, target_ns: int, tolerance_ns: int
) -> SyncResult[T]:
    """Select the nearest sample; equal-distance ties prefer the past sample."""

    _validate(stream, target_ns=target_ns, tolerance_ns=tolerance_ns)
    if not stream:
        return _invalid("stream_empty")
    index = bisect_left([item.stamp_ns for item in stream], target_ns)
    candidates = [item for item in stream[max(0, index - 1) : min(len(stream), index + 1)]]
    selected = min(
        candidates,
        key=lambda item: (abs(item.stamp_ns - target_ns), item.stamp_ns > target_ns),
    )
    delta = selected.stamp_ns - target_ns
    if abs(delta) > tolerance_ns:
        return _invalid("outside_tolerance", source_stamps=(selected.stamp_ns,), delta=delta)
    return SyncResult(
        selected.value,
        True,
        (selected.stamp_ns,),
        delta,
        target_ns - selected.stamp_ns,
        "matched_nearest",
    )


def causal_previous(
    stream: Sequence[TimedValue[T]], *, target_ns: int, max_age_ns: int
) -> SyncResult[T]:
    """Select the newest sample at or before target; future values never leak."""

    _validate(stream, target_ns=target_ns, tolerance_ns=max_age_ns)
    if not stream:
        return _invalid("stream_empty")
    index = bisect_right([item.stamp_ns for item in stream], target_ns) - 1
    if index < 0:
        return _invalid("future_only")
    selected = stream[index]
    age = target_ns - selected.stamp_ns
    if age > max_age_ns:
        return _invalid(
            "outside_tolerance",
            source_stamps=(selected.stamp_ns,),
            delta=-age,
            age=age,
        )
    return SyncResult(
        selected.value,
        True,
        (selected.stamp_ns,),
        -age,
        age,
        "matched_causal_previous",
    )


def linear_interpolate(
    stream: Sequence[TimedValue[float]], *, target_ns: int, tolerance_ns: int
) -> SyncResult[float]:
    """Linearly interpolate a finite scalar from bracketing samples."""

    return _interpolate(stream, target_ns=target_ns, tolerance_ns=tolerance_ns, angle=False)


def angle_interpolate(
    stream: Sequence[TimedValue[float]], *, target_ns: int, tolerance_ns: int
) -> SyncResult[float]:
    """Interpolate radians along the shortest wrapped arc in ``[-pi, pi)``."""

    return _interpolate(stream, target_ns=target_ns, tolerance_ns=tolerance_ns, angle=True)


def exact_events(
    stream: Sequence[TimedValue[T]], *, start_exclusive_ns: int, end_inclusive_ns: int
) -> tuple[TimedValue[T], ...]:
    """Collect every event in ``(start, end]`` without synthesizing absence."""

    if start_exclusive_ns < 0 or end_inclusive_ns < start_exclusive_ns:
        raise ValueError("event interval must satisfy 0 <= start <= end")
    _validate_order(stream)
    stamps = [item.stamp_ns for item in stream]
    first = bisect_right(stamps, start_exclusive_ns)
    last = bisect_right(stamps, end_inclusive_ns)
    return tuple(stream[first:last])


def _interpolate(
    stream: Sequence[TimedValue[float]],
    *,
    target_ns: int,
    tolerance_ns: int,
    angle: bool,
) -> SyncResult[float]:
    _validate(stream, target_ns=target_ns, tolerance_ns=tolerance_ns)
    if not stream:
        return _invalid("stream_empty")
    stamps = [item.stamp_ns for item in stream]
    right_index = bisect_left(stamps, target_ns)
    if right_index < len(stream) and stream[right_index].stamp_ns == target_ns:
        value = float(stream[right_index].value)
        if not math.isfinite(value):
            return _invalid("non_finite_value", source_stamps=(target_ns,), delta=0, age=0)
        return SyncResult(value, True, (target_ns,), 0, 0, "matched_exact")
    if right_index == 0 or right_index == len(stream):
        return _invalid("missing_bracket")
    left = stream[right_index - 1]
    right = stream[right_index]
    left_delta = target_ns - left.stamp_ns
    right_delta = right.stamp_ns - target_ns
    if left_delta > tolerance_ns or right_delta > tolerance_ns:
        return _invalid(
            "outside_tolerance",
            source_stamps=(left.stamp_ns, right.stamp_ns),
            delta=-left_delta,
            age=left_delta,
        )
    left_value = float(left.value)
    right_value = float(right.value)
    if not math.isfinite(left_value) or not math.isfinite(right_value):
        return _invalid(
            "non_finite_value", source_stamps=(left.stamp_ns, right.stamp_ns)
        )
    alpha = left_delta / (right.stamp_ns - left.stamp_ns)
    if angle:
        difference = _wrap_angle(right_value - left_value)
        value = _wrap_angle(left_value + alpha * difference)
    else:
        value = left_value + alpha * (right_value - left_value)
    return SyncResult(
        value,
        True,
        (left.stamp_ns, right.stamp_ns),
        -left_delta,
        left_delta,
        "interpolated_angle" if angle else "interpolated_linear",
    )


def _validate(
    stream: Sequence[TimedValue[T]], *, target_ns: int, tolerance_ns: int
) -> None:
    if target_ns < 0:
        raise ValueError("target_ns must be non-negative")
    if tolerance_ns < 0:
        raise ValueError("tolerance_ns must be non-negative")
    _validate_order(stream)


def _validate_order(stream: Sequence[TimedValue[T]]) -> None:
    if any(item.stamp_ns < 0 for item in stream):
        raise ValueError("stream timestamps must be non-negative")
    if any(right.stamp_ns <= left.stamp_ns for left, right in zip(stream, stream[1:])):
        raise ValueError("stream timestamps must be strictly increasing")


def _invalid(
    reason: str,
    *,
    source_stamps: tuple[int, ...] = (),
    delta: int | None = None,
    age: int | None = None,
) -> SyncResult[T]:
    return SyncResult(None, False, source_stamps, delta, age, reason)


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
