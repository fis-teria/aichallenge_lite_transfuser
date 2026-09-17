"""Preview speed planning on an unchanged, age-aligned time-model polyline.

This is a longitudinal limit, not a collision or road-boundary certificate.
Geometry and measured-speed PP/scan admission remain separate requirements.
Only curvature measurement groups points; steering sees the original path.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import numpy as np

from .time_dev_v1 import CORNER_CURVATURE_PER_M, CORNER_TRANSITION_PER_M

from .time_preview_v1 import (TIME_LOOKAHEAD_POLICY, PREVIEW_TIME_S, PREVIEW_OFFSET_M,
                              MINIMUM_PREVIEW_M, time_preview_distance, time_horizon_speed_cap)


ADAPTIVE_SPEED_POLICY = "curvature_preview_15kmh_v1"
TIME_ADAPTIVE_SPEED_POLICIES = {'curvature_time_preview_15kmh_v1': 15,
                                'curvature_time_preview_20kmh_v1': 20}
ADAPTIVE_SPEED_POLICIES = {ADAPTIVE_SPEED_POLICY: 15, **TIME_ADAPTIVE_SPEED_POLICIES}


@dataclass(frozen=True)
class CurvatureSpeedConfig:
    lateral_acceleration_mps2: float = 1.
    planning_deceleration_mps2: float = .7
    response_delay_s: float = .5
    curve_distance_reserve_m: float = .5
    curvature_half_span_m: float = .5
    horizon_reserve_m: float = .5
    horizon_extra_delay_s: float = .3
    maximum_acceleration_mps2: float = 1.
    minimum_acceleration_mps2: float = -1.
    speed_gain_per_s: float = 4.

    def __post_init__(self) -> None:
        values = asdict(self)
        if (not all(math.isfinite(v) for v in values.values())
                or any(v <= 0 for k, v in values.items() if k != "minimum_acceleration_mps2")
                or not -1. <= self.minimum_acceleration_mps2 < 0
                or self.planning_deceleration_mps2 > -self.minimum_acceleration_mps2
                or self.maximum_acceleration_mps2 > 1.):
            raise ValueError("CURVATURE_SPEED_CONFIG")


def preview_speed_limit(xy_m: np.ndarray, *, measured_speed_mps: float,
                        cruise_ceiling_mps: float, tracking_curvature_per_m: float,
                        policy: str = ADAPTIVE_SPEED_POLICY,
                        corner_max_speed_mps: float | None = None,
                        config: CurvatureSpeedConfig = CurvatureSpeedConfig()) -> dict[str, Any]:
    """Return a speed ceiling in m/s from finite [N,2] rear-frame metres.

    N is 2..31. Repeated adjacent points are removed only for measurement.
    Three-point circumcircle curvature is measured over at least a 1 m span,
    avoiding the unstable heading of very short 0.1 s prediction segments.
    Each bend limits current speed by v^2 <= v_bend^2 + 2*b*usable_distance;
    usable distance excludes measured-speed delay travel and a fixed reserve.
    The legacy endpoint bound uses stopping preview; explicit time policies
    use the shared linear PP preview with additional anticipation. No
    minimum-speed floor can override a short horizon or a bend.
    """
    points = np.asarray(xy_m, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (2,) or not 2 <= len(points) <= 31
            or not np.isfinite(points).all()
            or not np.isfinite([measured_speed_mps, cruise_ceiling_mps, tracking_curvature_per_m]).all()
            or policy not in ADAPTIVE_SPEED_POLICIES
            or not 0 <= measured_speed_mps <= (ADAPTIVE_SPEED_POLICIES[policy]+1)/3.6
            or not 0 < cruise_ceiling_mps <= ADAPTIVE_SPEED_POLICIES[policy]/3.6):
        raise ValueError("CURVATURE_SPEED_INPUT")
    if corner_max_speed_mps is not None and (
            policy not in TIME_ADAPTIVE_SPEED_POLICIES
            or type(corner_max_speed_mps) not in (int, float)
            or not math.isfinite(corner_max_speed_mps)
            or not 0 < corner_max_speed_mps <= cruise_ceiling_mps):
        raise ValueError('CORNER_SPEED_INPUT')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.r_[True, lengths > 1e-9]]
    if len(points) < 2:
        raise ValueError("CURVATURE_SPEED_PATH_UNRESOLVED")
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if arc[-1] < 2*config.curvature_half_span_m:
        raise ValueError("CURVATURE_SPEED_PATH_UNRESOLVED")
    half = config.curvature_half_span_m
    center_indices = np.flatnonzero((arc >= half) & (arc <= arc[-1]-half))
    if not len(center_indices):
        raise ValueError("CURVATURE_SPEED_PATH_UNRESOLVED")
    centers = arc[center_indices]
    before_indices = np.maximum(0, np.searchsorted(arc, centers-half, side='right')-1)
    after_indices = np.minimum(len(points)-1, np.searchsorted(arc, centers+half))
    # Using actual, sufficiently separated vertices avoids curvature spikes
    # introduced by interpolating different positions within polyline chords.
    before, middle, after = points[before_indices], points[center_indices], points[after_indices]
    ab, bc, ac = middle-before, after-middle, after-before
    denominator = np.linalg.norm(ab, axis=1)*np.linalg.norm(bc, axis=1)*np.linalg.norm(ac, axis=1)
    if np.any(denominator < 1e-9):
        raise ValueError("CURVATURE_SPEED_PATH_UNRESOLVED")
    curvature = 2*np.abs(ab[:, 0]*bc[:, 1]-ab[:, 1]*bc[:, 0])/denominator
    bend_cap = np.minimum(cruise_ceiling_mps,
        np.sqrt(config.lateral_acceleration_mps2/np.maximum(curvature, 1e-12)))
    # Apply each finite-span estimate at the BEGINNING of its support, including
    # the unresolved last half span, rather than waiting to reach its center.
    segments = np.diff(points, axis=0)
    squared = np.sum(segments**2, axis=1)
    fraction = np.clip(-np.sum(points[:-1]*segments, axis=1)/squared, 0., 1.)
    projected = points[:-1]+fraction[:, None]*segments
    nearest = int(np.argmin(np.sum(projected**2, axis=1)))
    origin_arc = float(arc[nearest]+fraction[nearest]*math.sqrt(squared[nearest]))
    support_start = np.maximum(0., arc[before_indices]-origin_arc)
    usable = np.maximum(0., support_start-measured_speed_mps*config.response_delay_s
                        -config.curve_distance_reserve_m)
    upstream = np.sqrt(bend_cap**2 + 2*config.planning_deceleration_mps2*usable)
    index = int(np.argmin(upstream))
    curve_cap = min(cruise_ceiling_mps, float(upstream[index]))
    tracking_cap = min(cruise_ceiling_mps,
        math.sqrt(config.lateral_acceleration_mps2/max(abs(tracking_curvature_per_m), 1e-12)))
    forward = points[points[:, 0] > 1e-6]
    if not len(forward):
        raise ValueError("CURVATURE_SPEED_NO_FORWARD_PATH")
    endpoint_distance = float(np.linalg.norm(forward[-1]))
    # Invert .4 + (.5 + extra_delay)*v + v^2/2 <= endpoint - reserve.
    delay = .5+config.horizon_extra_delay_s
    horizon_cap = max(0., -delay + math.sqrt(delay**2 + 2*max(0.,
        endpoint_distance-config.horizon_reserve_m-.4)))
    horizon_details = {}
    if policy in TIME_ADAPTIVE_SPEED_POLICIES:
        horizon_cap = time_horizon_speed_cap(endpoint_distance,
            reserve_m=config.horizon_reserve_m, extra_delay_s=config.horizon_extra_delay_s)
        horizon_details['horizon_contract'] = dict(policy=TIME_LOOKAHEAD_POLICY,
            preview_time_s=PREVIEW_TIME_S, offset_m=PREVIEW_OFFSET_M, minimum_m=MINIMUM_PREVIEW_M,
            required_at_measured_speed_m=time_preview_distance(measured_speed_mps),
            stopping_distance_used_for_pp=False)
    limits = dict(cruise=cruise_ceiling_mps, preview_curvature=curve_cap,
                  tracking_curvature=tracking_cap, prediction_horizon=horizon_cap)
    if corner_max_speed_mps is not None:
        # Blend only the longitudinal ceiling; never change prediction vertices.
        # Apply the same delay/reserve and braking preview as the physical cap.
        def corner_cap(curv):
            weight = np.clip((np.abs(curv)-CORNER_TRANSITION_PER_M)
                             /(CORNER_CURVATURE_PER_M-CORNER_TRANSITION_PER_M), 0., 1.)
            return cruise_ceiling_mps + weight*(corner_max_speed_mps-cruise_ceiling_mps)

        corner_local = corner_cap(curvature)
        corner_upstream = np.sqrt(corner_local**2 + 2*config.planning_deceleration_mps2*usable)
        corner_index = int(np.argmin(corner_upstream))
        limits.update(preview_corner=min(cruise_ceiling_mps, float(corner_upstream[corner_index])),
                      tracking_corner=float(corner_cap(tracking_curvature_per_m)))
        horizon_details['corner_speed_limit'] = dict(max_speed_mps=corner_max_speed_mps,
            full_curvature_per_m=CORNER_CURVATURE_PER_M, transition_curvature_per_m=CORNER_TRANSITION_PER_M,
            support_start_m=float(support_start[corner_index]),
            local_speed_mps=float(corner_local[corner_index]), usable_distance_m=float(usable[corner_index]))
    limiting_reason = min(limits, key=limits.get)
    target = limits[limiting_reason]
    # Retain the completed fixed-speed trial's P gain and actuator authority.
    # Lower gain/cap allowed speed and predicted horizon to decay together in
    # AWSIM despite positive commands. A command is not net acceleration.
    # Preview limits govern speed; no floor overrides a bend or short path.
    acceleration_cap = config.maximum_acceleration_mps2
    acceleration = float(np.clip(config.speed_gain_per_s*(target-measured_speed_mps),
        config.minimum_acceleration_mps2, acceleration_cap))
    return dict(policy=policy, target_speed_mps=target,
        acceleration_mps2=acceleration, acceleration_cap_mps2=acceleration_cap,
        limiting_reason=limiting_reason, limits_mps=limits,
        measured_speed_mps=measured_speed_mps, endpoint_distance_m=endpoint_distance,
        origin_projection_arc_m=origin_arc,
        tracking_curvature_per_m=tracking_curvature_per_m,
        maximum_path_curvature_per_m=float(np.max(curvature)),
        limiting_curve_support_start_m=float(support_start[index]),
        limiting_curve_curvature_per_m=float(curvature[index]),
        limiting_curve_local_speed_mps=float(bend_cap[index]),
        limiting_curve_usable_distance_m=float(usable[index]),
        samples=len(centers), config=asdict(config), **horizon_details,
        scope="LONGITUDINAL_PREVIEW_LIMIT_NOT_COLLISION_PROOF")
