"""Pure command checks for the read-only, finite simulator observer."""
import math
from collections.abc import Sequence


def command_fault(values: Sequence[float], raw_limit_rad: float,
                  gain: float, *, final: bool) -> str | None:
    """Check [steering rad, speed m/s, acceleration m/s^2]; never sends control.

    Limits must be read from the exact loaded MPC config before arming. Final
    steering uses the existing gain domain, not measured physical tire angle.
    The 1e-6 rad allowance covers Float32 representation only.
    """
    if (not math.isfinite(raw_limit_rad) or raw_limit_rad <= 0
            or not math.isfinite(gain) or gain <= 0):
        return 'COMMAND_LIMITS_INVALID'
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        return 'COMMAND_NONFINITE_OR_SHAPE'
    limit = raw_limit_rad * (gain if final else 1.0)
    if abs(values[0]) > limit + 1e-6:
        return 'FINAL_STEERING_LIMIT' if final else 'RAW_STEERING_LIMIT'
    return None
