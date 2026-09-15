"""Conservative support halfplanes for a bounded-curvature stopping body.

No constant-steering assumption: heading at distance s lies in [k_min*s,
k_max*s]. Each position projection integrates the maximum over that interval.
Midpoint integration adds a Lipschitz error bound; body support maximizes over
all four corners and every possible endpoint heading. Taking maxima over s
encloses the whole sweep in a convex polygon. Convex filling can over-reject.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
from .vehicle_motion_v1 import MAX_CURVATURE_PER_M, stopping_motion

STANDARD_CLEARANCE = 'standard_v1'
NEAR_LIMIT_CLEARANCE = 'awsim_near_limit_v1'


def clearance_dimensions(profile: str) -> tuple[float, float]:
    """Return (fixed stopping reserve m, half-width m), with 1.30 m kart width."""
    if profile == STANDARD_CLEARANCE:
        return .4, .85
    if profile == NEAR_LIMIT_CLEARANCE:
        return .1, .70
    raise ValueError('SCAN_CLEARANCE_PROFILE')


def curvature_support_envelope(k_min: float, k_max: float, travel_m: float,
                               angle_step_rad: float, sensor_xy_m: np.ndarray, *, lateral_padding_m: float = 0.,
                               clearance_profile: str = STANDARD_CLEARANCE
                               ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Return unit normals [64,2], support distances [64] in m, and bounds.

    Points x satisfying normals @ x <= support enclose all allowed body
    footprints. Curvatures are 1/m; standard body is [-.510,1.984] x ±.85 m.
    The explicit AWSIM diagnostic uses ±.70 m (physical half-width .65 + .05).
    The admitted domain keeps all heading intervals strictly inside (-pi,pi).
    """
    sensor = np.asarray(sensor_xy_m, dtype=float)
    reserve, half_width = clearance_dimensions(clearance_profile)
    if (sensor.shape != (2,) or not np.isfinite(sensor).all()
            or not np.isfinite([k_min, k_max, travel_m, angle_step_rad, lateral_padding_m]).all()
            or not -MAX_CURVATURE_PER_M <= k_min <= k_max <= MAX_CURVATURE_PER_M
            or not 0 <= lateral_padding_m <= .1
            or not reserve <= travel_m <= reserve+(6/3.6)*.5+(6/3.6)**2/2
            or not 0 < angle_step_rad <= .02):
        raise ValueError("SUPPORT_ENVELOPE_CONTRACT")
    k_bound = max(abs(k_min), abs(k_max))
    distances = np.linspace(0., travel_m, int(math.ceil(travel_m/.005))+1)
    ds = distances[1]-distances[0]
    midpoint = (distances[:-1]+distances[1:])/2
    phase = np.linspace(-math.pi, math.pi, 64, endpoint=False)
    normals = np.column_stack([np.cos(phase), np.sin(phase)])
    low, high = k_min*midpoint[:, None], k_max*midpoint[:, None]
    projected_speed = np.maximum(np.cos(low-phase), np.cos(high-phase))
    projected_speed = np.where((low <= phase) & (phase <= high), 1., projected_speed)
    # The maximum projection is k_bound-Lipschitz in distance. ds²/2 per
    # interval safely exceeds the midpoint integrated error bound ds²/4.
    position = np.vstack([np.zeros(64), np.cumsum(projected_speed*ds+k_bound*ds*ds/2, axis=0)])
    low, high = k_min*distances[:, None, None], k_max*distances[:, None, None]
    corners = np.array([[-.510, -half_width], [1.984, -half_width], [1.984, half_width], [-.510, half_width]])
    a = normals@corners.T
    b = normals@np.column_stack([-corners[:, 1], corners[:, 0]]).T
    peak = np.arctan2(b, a)
    body = np.maximum(a*np.cos(low)+b*np.sin(low), a*np.cos(high)+b*np.sin(high))
    body = np.where((low <= peak) & (peak <= high), np.hypot(a, b), body).max(axis=2)
    radius = math.hypot(1.984, half_width)
    integration_bound = k_bound*travel_m*ds/2
    between_samples = ds/2*(1+radius*k_bound)
    # Include sensor offset and polygon facets when covering angular gaps.
    facet_cos = math.cos(math.pi/64)
    reach = (travel_m+radius+integration_bound+between_samples+lateral_padding_m)/facet_cos + float(np.linalg.norm(sensor))
    angular_padding = reach*angle_step_rad/(1-angle_step_rad/facet_cos)
    padding = between_samples+angular_padding+lateral_padding_m
    support = (position+body).max(axis=0)+padding
    metadata = {"policy": "CURVATURE_INTERVAL_SUPPORT_V2", "support_directions": 64,
        "sweep_samples": len(distances), "integration_step_m": float(ds),
        "maximum_position_integration_error_m": float(integration_bound),
        "maximum_discretization_padding_m": float(padding), "stopping_travel_m": travel_m,
        "whole_sweep_convex_enclosure": True}
    if clearance_profile != STANDARD_CLEARANCE:
        metadata.update(clearance_profile=clearance_profile, diagnostic_only=True,
                        fixed_stopping_reserve_m=reserve, body_half_width_m=half_width)
    return normals, support, metadata


def check_support_ranges(ranges: np.ndarray, angles: np.ndarray, range_max: float,
                         angle_step_rad: float, sensor: np.ndarray, speed_mps: float,
                         measured_steer_rad: float, issued_steer_rad: float,
                         previous_steer_rad: float | None, *, motion: dict[str, Any] | None = None,
                         clearance_profile: str = STANDARD_CLEARANCE) -> dict[str, Any]:
    """Internal ray test after check_turning_scan validates scan/state/frame."""
    if motion is None:
        motion = stopping_motion(speed_mps, measured_steer_rad, issued_steer_rad, previous_steer_rad)
    k_min, k_max = motion["curvature_interval_per_m"]
    speed = max(0., speed_mps)
    reserve, _ = clearance_dimensions(clearance_profile)
    normals, support, metadata = curvature_support_envelope(k_min, k_max,
        reserve+speed*.5+speed**2/2, angle_step_rad, sensor[:2],
        lateral_padding_m=motion["lateral_displacement_bound_m"], clearance_profile=clearance_profile)
    remaining = support-normals@sensor[:2]
    directions = normals@np.column_stack([np.cos(angles+sensor[2]), np.sin(angles+sensor[2])]).T
    parallel = abs(directions) < 1e-12
    intersections = remaining[:, None]/np.where(parallel, 1., directions)
    near = np.where(directions < -1e-12, intersections, -np.inf).max(axis=0)
    far = np.where(directions > 1e-12, intersections, np.inf).min(axis=0)
    outside = np.any(parallel & (remaining[:, None] < 0), axis=0)
    intersects = ~outside & (far >= np.maximum(near, 0.))
    required = np.where(intersects, far, 0.)
    observed = required > 0.
    margins = np.minimum(ranges, range_max)-required
    if np.any(margins[observed] <= 0.):
        raise ValueError("STOPPING_SWEEP_OCCUPIED")
    return {**metadata, "vehicle_motion": motion, "scope": "FORWARD_SCAN_PROXIMITY_NOT_ALL_AROUND_FREE_SPACE",
        "minimum_ray_margin_m": float(margins[observed].min()) if observed.any() else float(range_max),
        "checked_rays": int(observed.sum()), "measured_steer_rad": measured_steer_rad,
        "issued_steer_rad": issued_steer_rad, "previous_steer_rad": previous_steer_rad,
        "full_body_free_space_verified": False}
