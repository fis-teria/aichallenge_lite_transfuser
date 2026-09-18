"""Offline XY/speed curation of observed native-object runs, without ROS.

Selection is a dataset quality screen, not physical-clearance certification.
Stop intent and behaviour class are not inferred from speed or planner mode.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class NativeCurationConfig:
    history_ns: int = 1_000_000_000
    horizon_ns: int = 3_000_000_000
    endpoint_guard_ns: int = 50_000_000
    max_evidence_gap_ns: int = 150_000_000
    minimum_anchor_spacing_ns: int = 200_000_000
    minimum_future_speed_mps: float = .2
    box_exclusion_radius_m: float = 6.
    cone_context_radius_m: float = 6.
    minimum_clearance_m: float = .30
    additional_projection_margin_m: float = .30
    maximum_map_median_m: float = .15
    map_inlier_distance_m: float = .25
    minimum_map_inlier_fraction: float = .70
    minimum_wall_points: int = 80
    minimum_scan_span_rad: float = math.pi / 3

    def __post_init__(self) -> None:
        for name in ('history_ns', 'horizon_ns', 'endpoint_guard_ns', 'max_evidence_gap_ns',
                     'minimum_anchor_spacing_ns', 'minimum_wall_points'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be a positive integer')
        for name, value in asdict(self).items():
            if name.endswith(('_m', '_mps', '_rad', '_fraction')):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{name} must be finite and positive')
        if self.minimum_map_inlier_fraction > 1 or self.minimum_scan_span_rad > 2 * math.pi:
            raise ValueError('invalid fraction or radians')
        if self.history_ns != 1_000_000_000 or self.horizon_ns != 3_000_000_000 or self.endpoint_guard_ns < 50_000_000:
            raise ValueError('fixed 1 s history / 30 x 0.1 s future with at least 50 ms support required')
        if self.minimum_clearance_m < .30:
            raise ValueError('existing 0.30 m clearance requirement must be preserved')


def convex_point_distance(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Finite points [N,2] and convex polygon [M,2] -> distances [N], metres."""
    p, q = np.asarray(points, float), np.asarray(polygon, float)
    if p.ndim != 2 or p.shape[1] != 2 or q.ndim != 2 or q.shape[1] != 2 or len(q) < 3:
        raise ValueError('POINT_POLYGON_SHAPE')
    if not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('POINT_POLYGON_NONFINITE')
    edges = np.roll(q, -1, axis=0) - q
    lengths = np.sum(edges ** 2, axis=1)
    turns = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if (lengths <= 1e-20).any() or not ((turns >= -1e-12).all() or (turns <= 1e-12).all()) or np.max(abs(turns)) < 1e-12:
        raise ValueError('DEGENERATE_OR_NONCONVEX_POLYGON')
    delta = p[:, None] - q[None]
    fraction = np.clip(np.sum(delta * edges, axis=2) / lengths, 0., 1.)
    distance = np.linalg.norm(delta - fraction[:, :, None] * edges, axis=2).min(axis=1)
    cross = edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]
    inside = (cross >= -1e-10).all(axis=1) | (cross <= 1e-10).all(axis=1)
    return np.where(inside, 0., distance)


def window_indices(stamps_ns: np.ndarray, low_ns: int, high_ns: int,
                   max_gap_ns: int) -> tuple[int, int] | None:
    """Bracket a closed window using adjacent evidence; never interpolate a gap.

    Strictly increasing nonnegative int64 [N], bounds and max gap in integer ns.
    Returns inclusive start / exclusive stop, or None for incomplete coverage.
    """
    stamps = np.asarray(stamps_ns)
    if (stamps.ndim != 1 or stamps.dtype != np.int64 or (stamps < 0).any()
            or (np.diff(stamps) <= 0).any()):
        raise ValueError('EVIDENCE_STAMPS_INT64_SORTED_UNIQUE')
    if (any(type(v) is not int for v in (low_ns, high_ns, max_gap_ns))
            or low_ns < 0 or high_ns < low_ns or max_gap_ns <= 0):
        raise ValueError('EVIDENCE_WINDOW_NS')
    left = int(np.searchsorted(stamps, low_ns, side='right')) - 1
    right = int(np.searchsorted(stamps, high_ns, side='left'))
    if left < 0 or right >= len(stamps):
        return None
    if (np.diff(stamps[left:right + 1]) > max_gap_ns).any():
        return None
    return left, right + 1


def classify_anchor(*, xy_m: np.ndarray, xy_mask: np.ndarray, velocity_mps: np.ndarray,
                    velocity_mask: np.ndarray, findings: list[str], evidence: dict[str, Any],
                    config: NativeCurationConfig = NativeCurationConfig()) -> dict[str, Any]:
    """Classify one observed [30,2] m / [30] m/s teacher using its full window.

    Missing evidence fails closed. `selected` permits only observed XY and speed
    targets. It never changes the original strict avoidance eligibility mask.
    """
    xy, xm, v, vm = map(np.asarray, (xy_m, xy_mask, velocity_mps, velocity_mask))
    if xy.shape != (30, 2) or xm.shape != (30,) or v.shape != (30,) or vm.shape != (30,):
        raise ValueError('TEACHER_SHAPE')
    if xm.dtype != np.bool_ or vm.dtype != np.bool_:
        raise ValueError('TEACHER_MASK_DTYPE')
    reasons: list[str] = []
    if not xm.all() or not vm.all() or not np.isfinite(xy).all() or not np.isfinite(v).all():
        reasons.append('INCOMPLETE_OR_NONFINITE_TEACHER')
    hard_findings = {'RECORDED_POSE_PROJECTED_CONE_GAP_BELOW_030', 'NOMINAL_BOX_GAP_BELOW_030',
                     'RECORDED_POSE_WALL_OVERLAP', 'LIDAR_OBSERVED_POINT_GAP_BELOW_030'}
    reasons.extend(sorted(hard_findings.intersection(findings)))
    if np.isfinite(v).all() and (v < -.05).any():
        reasons.append('REVERSE_FUTURE')
    if reasons:
        return dict(disposition='exclude', selected=False, reasons=reasons, use=None)
    if (v <= config.minimum_future_speed_mps).any():
        reasons.append('STOP_OR_CREEP_INTENT_UNVERIFIED')
    required = {'coverage_ok', 'teacher_ok', 'perception_ok', 'pose_geometry_ok',
                'box_distance_m', 'cone_gap_m', 'cone_distance_m', 'map_median_max_m',
                'map_inlier_fraction_min', 'wall_points_min', 'scan_span_min_rad', 'all_free_run'}
    if required - evidence.keys():
        reasons.append('MISSING_WINDOW_EVIDENCE')
    else:
        for key in ('coverage_ok', 'teacher_ok', 'perception_ok', 'pose_geometry_ok'):
            if evidence[key] is not True:
                reasons.append('WINDOW_' + key.upper())
        numeric = ('box_distance_m', 'cone_gap_m', 'cone_distance_m', 'map_median_max_m',
                   'map_inlier_fraction_min', 'wall_points_min', 'scan_span_min_rad')
        if (any(type(evidence[k]) not in (float, int) or not math.isfinite(evidence[k])
                or evidence[k] < 0 for k in numeric)
                or evidence['map_inlier_fraction_min'] > 1 or evidence['scan_span_min_rad'] > 2 * math.pi):
            reasons.append('NONFINITE_WINDOW_EVIDENCE')
        else:
            if (evidence['box_distance_m'] <= config.box_exclusion_radius_m
                    or 'DYNAMIC_BOX_NEARBY_UNVERIFIED' in findings):
                reasons.append('DYNAMIC_BOX_PROXIMITY_UNVERIFIED')
            if evidence['cone_gap_m'] < config.minimum_clearance_m + config.additional_projection_margin_m:
                reasons.append('STATIC_CONE_PROJECTION_MARGIN')
            if (evidence['map_median_max_m'] > config.maximum_map_median_m
                    or evidence['map_inlier_fraction_min'] < config.minimum_map_inlier_fraction
                    or evidence['wall_points_min'] < config.minimum_wall_points
                    or evidence['scan_span_min_rad'] < config.minimum_scan_span_rad):
                reasons.append('SCAN_MAP_ALIGNMENT_OR_SUPPORT')
            if (evidence['cone_distance_m'] > config.cone_context_radius_m
                    and evidence['all_free_run'] is not True):
                reasons.append('NONFREE_MODE_WITHOUT_STATIC_CONE_CONTEXT')
    if 'POSE_WINDOW_COVERAGE_UNKNOWN' in findings:
        reasons.append('MONITOR_POSE_COVERAGE_UNKNOWN')
    if reasons:
        return dict(disposition='hold', selected=False, reasons=reasons, use=None)
    use = ('static_cone_xy_speed' if evidence['cone_distance_m'] <= config.cone_context_radius_m
           else 'nominal_xy_speed')
    return dict(disposition='eligible', selected=True, reasons=[], use=use)


def spaced_indices(stamps_ns: np.ndarray, eligible: np.ndarray, minimum_spacing_ns: int) -> np.ndarray:
    """Deterministic temporal thinning inside one run/epoch; preserves source order."""
    stamps, mask = np.asarray(stamps_ns), np.asarray(eligible)
    if (stamps.dtype != np.int64 or stamps.ndim != 1 or mask.dtype != np.bool_
            or mask.shape != stamps.shape or (stamps < 0).any() or (np.diff(stamps) <= 0).any()
            or type(minimum_spacing_ns) is not int or minimum_spacing_ns <= 0):
        raise ValueError('THINNING_CONTRACT')
    chosen: list[int] = []
    for i in np.flatnonzero(mask):
        if not chosen or int(stamps[i]) - int(stamps[chosen[-1]]) >= minimum_spacing_ns:
            chosen.append(int(i))
    return np.asarray(chosen, dtype=np.int64)
