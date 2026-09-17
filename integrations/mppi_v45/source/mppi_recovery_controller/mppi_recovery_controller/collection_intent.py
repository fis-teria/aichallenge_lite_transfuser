"""Use the fresh normal controller request to distinguish waiting from stuck."""
from __future__ import annotations

import math


def fresh_forward_speed_mps(*, now_sec: float, command_sec: float | None,
                            speed_mps: float | None, timeout_sec: float) -> float:
    """Return a finite, fresh forward request in m/s; missing/stale means wait.

    This only gates entry into recovery. An active reverse/handback sequence
    remains governed by the existing recovery controller and its watchdogs.
    """
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError("command timeout must be positive finite seconds")
    if command_sec is None or speed_mps is None:
        return 0.0
    if not all(math.isfinite(value) for value in (now_sec, command_sec, speed_mps)):
        return 0.0
    if not 0 <= now_sec - command_sec <= timeout_sec:
        return 0.0
    return max(0.0, speed_mps)
