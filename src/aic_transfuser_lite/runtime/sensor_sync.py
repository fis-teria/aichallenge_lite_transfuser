from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Generic, Mapping, Sequence, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class TimedSample(Generic[T]):
    stamp_sec: float
    value: T


@dataclass(frozen=True)
class SyncDecision(Generic[T]):
    accepted: bool
    reason: str
    camera_stamp_sec: float
    samples: Mapping[str, T]
    deltas_sec: Mapping[str, float]
    max_skew_sec: float


class CameraMasterSynchronizer(Generic[T]):
    """Bounded nearest-sample synchronization with Camera as the master.

    Selected samples are consumed exactly once. A failed skew check never emits
    samples, allowing the caller to fail closed without publishing a command.
    """

    def __init__(
        self,
        *,
        required_roles: Sequence[str],
        queue_size: int,
        max_skew_sec: float,
    ) -> None:
        roles = tuple(str(role) for role in required_roles)
        if not roles or any(not role for role in roles) or len(set(roles)) != len(roles):
            raise ValueError("required_roles must be unique non-empty names")
        if int(queue_size) < 1:
            raise ValueError("queue_size must be at least one")
        if not math.isfinite(float(max_skew_sec)) or float(max_skew_sec) <= 0.0:
            raise ValueError("max_skew_sec must be finite and positive")
        self.required_roles = roles
        self.queue_size = int(queue_size)
        self.max_skew_sec = float(max_skew_sec)
        self._buffers: dict[str, Deque[TimedSample[T]]] = {
            role: deque(maxlen=self.queue_size) for role in roles
        }

    @staticmethod
    def _valid_stamp(stamp_sec: float) -> float:
        stamp = float(stamp_sec)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("timestamp must be finite and positive")
        return stamp

    def add(self, role: str, stamp_sec: float, value: T) -> None:
        if role not in self._buffers:
            raise KeyError(f"unknown sync role: {role!r}")
        stamp = self._valid_stamp(stamp_sec)
        buffer = self._buffers[role]
        if buffer and stamp <= buffer[-1].stamp_sec:
            raise ValueError(f"{role} timestamps must be strictly increasing")
        buffer.append(TimedSample(stamp, value))

    def all_streams_reached(self, camera_stamp_sec: float) -> bool:
        """Return whether every stream has observed ``camera_stamp_sec`` or later.

        Once this is true, monotonic input timestamps guarantee that a later
        arrival cannot become a nearer sample on the past side of the Camera
        observation.  The ROS adapter uses this only to make a failed skew
        decision final; an already-valid match can be emitted immediately.
        """

        camera_stamp = self._valid_stamp(camera_stamp_sec)
        return all(
            buffer and buffer[-1].stamp_sec >= camera_stamp
            for buffer in self._buffers.values()
        )

    def match(self, camera_stamp_sec: float) -> SyncDecision[T]:
        camera_stamp = self._valid_stamp(camera_stamp_sec)
        missing = [role for role in self.required_roles if not self._buffers[role]]
        if missing:
            return SyncDecision(
                False,
                "missing:" + ",".join(missing),
                camera_stamp,
                {},
                {},
                math.inf,
            )

        selected: dict[str, tuple[int, TimedSample[T]]] = {}
        for role in self.required_roles:
            values = self._buffers[role]
            index = min(
                range(len(values)),
                key=lambda item: (abs(values[item].stamp_sec - camera_stamp), item),
            )
            selected[role] = (index, values[index])

        stamps = [camera_stamp] + [item.stamp_sec for _, item in selected.values()]
        max_skew = max(stamps) - min(stamps)
        deltas = {
            role: item.stamp_sec - camera_stamp
            for role, (_, item) in selected.items()
        }
        if max_skew - self.max_skew_sec > 1e-12:
            return SyncDecision(
                False,
                "sensor_skew",
                camera_stamp,
                {},
                deltas,
                max_skew,
            )

        samples = {role: item.value for role, (_, item) in selected.items()}
        for role, (index, _) in selected.items():
            for _ in range(index + 1):
                self._buffers[role].popleft()
        return SyncDecision(
            True,
            "synchronized",
            camera_stamp,
            samples,
            deltas,
            max_skew,
        )
