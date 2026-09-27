"""Offline adapter to an existing controller intent contract; no actuator I/O."""
from __future__ import annotations

import math

import numpy as np

from aic_transfuser_lite.contracts.common_il_v1 import CommonILPrediction, common_il_contract
from .executable_reference import AuthoritativePlanV3


def adapt_common_il_output(prediction: CommonILPrediction, *, observation_stamp_sec: float,
                          speed_supervised: bool, stop_supervised: bool,
                          speed_cap_mps: float, index: int = 0) -> AuthoritativePlanV3:
    """Six timed XY points plus a scalar-speed hold for the external controller.

    Repeating the scalar is an explicit constant-speed reference assumption,
    not six independently learned speed predictions. Forward-only control clips
    signed speed to [0, cap]. Stop remains None without supervised labels.
    The controller must handle frame/origin transforms and staleness, and the
    external Safety Supervisor still owns timeout/finite/braking decisions.
    This is an offline intent adapter, not a vehicle-ready checkpoint gate.
    """
    prediction.validate()
    if type(speed_supervised) is not bool or type(stop_supervised) is not bool or not speed_supervised:
        raise ValueError("learned-speed supervision required; explicit bool head status required")
    if not math.isfinite(speed_cap_mps) or speed_cap_mps <= 0:
        raise ValueError("speed cap must be positive finite m/s")
    if type(index) is not int or not 0 <= index < prediction.waypoints_m.shape[0]:
        raise ValueError("invalid batch index")
    speed = max(0.0, min(speed_cap_mps, float(prediction.target_speed_mps[index, 0])))
    stop = float(prediction.stop_probability[index, 0]) if stop_supervised else None
    plan = AuthoritativePlanV3(
        trajectory_xy_m=prediction.waypoints_m[index].detach().float().cpu().numpy().copy(),
        speed_profile_mps=np.full(6, speed),
        waypoint_times_sec=np.asarray(common_il_contract()["waypoint_time_sec"]),
        observation_stamp_sec=observation_stamp_sec, frame_id="base_link", stop_probability=stop,
    )
    plan.validate(require_stop_probability=stop_supervised)
    return plan
