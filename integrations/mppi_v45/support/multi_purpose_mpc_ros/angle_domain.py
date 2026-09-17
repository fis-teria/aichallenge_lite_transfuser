"""ROS-free angle semantics used at Controller decision boundaries.

Only angle kinds connected to a real consumer belong here.  Additional
heading/error/turn contracts are introduced with their consumers so this
module does not become a collection of unused type aliases.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class UnsignedPathAngleRad:
    """Unsigned forward path angle in radians, constrained to ``[0, pi/2)``.

    This is the acute geometric angle of a forward connection, not an
    absolute heading, signed heading error, or accumulated turn.
    """

    value_rad: float

    def __post_init__(self) -> None:
        if isinstance(self.value_rad, bool):
            raise TypeError("unsigned path angle must be numeric, not bool")
        value_rad = float(self.value_rad)
        if not math.isfinite(value_rad):
            raise ValueError("unsigned path angle must be finite")
        if not 0.0 <= value_rad < math.pi / 2.0:
            raise ValueError(
                "unsigned path angle must be in [0, pi/2)"
            )
        object.__setattr__(self, "value_rad", value_rad)


@dataclass(frozen=True)
class WrappedHeadingErrorRad:
    """Shortest signed heading error using legacy ``(-pi, pi]`` wrapping.

    The endpoint convention is intentionally part of this contract.  The
    recovery normalizer maps both ``-pi`` and ``+pi`` to ``+pi``; changing
    that sign at the boundary could reverse a steering correction.
    """

    value_rad: float

    def __post_init__(self) -> None:
        if isinstance(self.value_rad, bool):
            raise TypeError("wrapped heading error must be numeric, not bool")
        value_rad = float(self.value_rad)
        if not math.isfinite(value_rad):
            raise ValueError("wrapped heading error must be finite")
        if not -math.pi < value_rad <= math.pi:
            raise ValueError(
                "wrapped heading error must be in (-pi, pi]"
            )
        object.__setattr__(self, "value_rad", value_rad)
