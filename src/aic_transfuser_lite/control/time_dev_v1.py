"""Launch-time speed parameters for the tested AWSIM time-preview profile.

Public values are km/h; control values are m/s. These are ceilings, not
constant speed requests. The 20 km/h profile remains the vehicle model domain.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from typing import Any


DEV_SPEED_POLICY = 'curvature_time_preview_20kmh_v1'
CORNER_CURVATURE_PER_M = .05  # Full corner cap at radius <= 20 m.
CORNER_TRANSITION_PER_M = .025  # Continuous blend from radius 40 m to 20 m.


@dataclass(frozen=True)
class TimeDevSpeeds:
    max_speed_kmh: float = 20.
    corner_max_speed_kmh: float = 10.

    def __post_init__(self) -> None:
        values = (self.max_speed_kmh, self.corner_max_speed_kmh)
        if (any(type(v) not in (float, int) or not math.isfinite(v) for v in values)
                or not 0 < self.corner_max_speed_kmh <= self.max_speed_kmh <= 20.):
            raise ValueError('TIME_DEV_SPEEDS_REQUIRE_0_LT_CORNER_LE_MAX_LE_20_KMH')

    @property
    def max_mps(self) -> float:
        return self.max_speed_kmh / 3.6

    @property
    def corner_mps(self) -> float:
        return self.corner_max_speed_kmh / 3.6

    @property
    def overspeed_mps(self) -> float:
        return (self.max_speed_kmh + 1.) / 3.6


def configured_dev_speeds(config: dict[str, Any]) -> TimeDevSpeeds | None:
    """Absent key means the exact legacy profile; null/unknown keys are errors."""
    if 'speed_parameters' not in config:
        return None
    values = config['speed_parameters']
    if (config.get('speed_policy') != DEV_SPEED_POLICY or not isinstance(values, dict)
            or set(values) != {'max_speed_kmh', 'corner_max_speed_kmh'}):
        raise ValueError('TIME_DEV_SPEED_PARAMETER_CONTRACT')
    return TimeDevSpeeds(**values)


def configure_dev_speeds(config: dict[str, Any], *, max_speed_kmh: float,
                         corner_max_speed_kmh: float) -> dict[str, Any]:
    """Make a new effective config; the input and other controller settings stay intact."""
    if config.get('speed_policy') != DEV_SPEED_POLICY:
        raise ValueError('TIME_DEV_REQUIRES_20_KMH_TIME_PREVIEW_PROFILE')
    speeds = TimeDevSpeeds(max_speed_kmh, corner_max_speed_kmh)
    result = deepcopy(config)
    result.update(speed_parameters=asdict(speeds), speed_cap_mps=speeds.max_mps,
                  overspeed_limit_mps=speeds.overspeed_mps)
    return result
