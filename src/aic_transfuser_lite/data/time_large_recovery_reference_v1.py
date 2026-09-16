"""Static preparation reference and measured nominal guide for large recovery.

Only preparation uses the second reference. Its artificial far return is hidden
>=10 m beyond the latest preparation command; it is never a training label.
Actual recovery uses the original continuously published nominal reference.
"""
from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .recovery_reference_v3 import MpcReferencePointV3, OccupancyMapV3, _recompute_geometry
from .time_large_recovery_v1 import SCHEMA, LargeRecoveryConfig, LargeRecoverySite


def validate_large_reference(reference: Mapping[str, Any], root: Path | None = None
                             ) -> tuple[LargeRecoveryConfig, np.ndarray]:
    """Validate teacher-only config, measured guide [N,3] and static CSV identity."""
    spec = reference['large_recovery']
    if (spec['schema'] != SCHEMA or reference.get('steering_pulse') is not None
            or reference['intervals'] or reference['signed_offset_m'] != 0.
            or reference['reference_xy_m'] != reference['baseline_xy_m']
            or spec['map_screen_pass'] is not True):
        raise ValueError('LARGE_REFERENCE_CONTRACT')
    config = LargeRecoveryConfig(**spec['config'])
    guide = np.asarray(spec['nominal_guide'], dtype=float)
    if (guide.ndim != 2 or guide.shape[1] != 3 or len(guide) < 2
            or not np.isfinite(guide).all() or not np.all(np.diff(guide[:, 0]) > 0.)
            or np.max(np.diff(guide[:, 0])) > .3
            or guide[0, 0] > config.sites[0].start_s_m-5.
            or guide[-1, 0] < config.sites[-1].release_s_m+24.):
        raise ValueError('LARGE_NOMINAL_GUIDE_COVERAGE')
    xy = np.asarray(spec['preparation_xy_m'], dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 20 or not np.isfinite(xy).all():
        raise ValueError('LARGE_PREPARATION_SHAPE')
    name = spec['preparation_csv']
    if name not in ('left_preparation.csv', 'right_preparation.csv'):
        raise ValueError('LARGE_PREPARATION_CSV_NAME')
    if root is not None:
        actual = hashlib.sha256((root/name).read_bytes()).hexdigest()
        if actual != spec['preparation_sha256']:
            raise ValueError('LARGE_PREPARATION_SHA_MISMATCH')
        with (root/name).open(newline='') as stream:
            reader = csv.reader(stream)
            if next(reader) != ['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2']:
                raise ValueError('LARGE_PREPARATION_CSV_COLUMNS')
            rows = np.asarray([[float(v) for v in row] for row in reader])
        if (rows.ndim != 2 or rows.shape[1] != 7 or len(rows) <= len(xy)
                or not np.isfinite(rows).all() or not np.all(np.diff(rows[:, 0]) > 0.)
                or not np.allclose(rows[:, 5], 5/3.6, rtol=0., atol=1e-6)
                or not np.array_equal(rows[:len(xy), 1:3], xy)):
            raise ValueError('LARGE_PREPARATION_CSV_GEOMETRY')
    return config, guide


def _smooth(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0., 1.)
    return value*value*(3.-2.*value)


def preparation_lateral(progress: np.ndarray, site: LargeRecoverySite) -> np.ndarray:
    """Command-path offset [N] m; actual goal requires measured state validation.

    Zero-heading legacy profiles are unchanged. Heading profiles use a cubic
    Hermite approach with zero start slope and tan(target heading) release
    slope. A bounded forward extension supplies PP preview; it is not a label.
    """
    progress = np.asarray(progress, dtype=float)
    if progress.ndim != 1 or not np.isfinite(progress).all():
        raise ValueError('LARGE_PREPARATION_PROGRESS_SHAPE')
    if site.target_heading_rad == 0.:
        return site.target_offset_m*_smooth((progress-site.start_s_m)/8.)
    length = site.release_s_m-site.start_s_m
    u = np.clip((progress-site.start_s_m)/length, 0., 1.)
    slope = math.tan(site.target_heading_rad)
    approach = site.target_offset_m*(3*u*u-2*u*u*u)+length*slope*(u*u*u-u*u)
    # C1 extension: linear through release+2 m, taper slope to zero by +6 m.
    d = np.maximum(progress-site.release_s_m, 0.)
    v = np.clip((d-2.)/4., 0., 1.)
    extension = np.minimum(d, 2.)+4.*(v-v**3+.5*v**4)
    return approach+slope*extension


def _map_free(occupancy: OccupancyMapV3, xy: np.ndarray) -> bool:
    """Check every source vertex and segments at <=one map cell, radius 1.4 m."""
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all():
        raise ValueError('LARGE_MAP_XY_SHAPE')
    step = min(.05, occupancy.resolution_m_per_px)
    for a, b in zip(xy[:-1], xy[1:]):
        parts = max(1, math.ceil(float(np.linalg.norm(b-a))/step))
        dense = a+(b-a)*np.linspace(0., 1., parts+1)[:, None]
        if not occupancy.footprint_is_free(dense[:, 0], dense[:, 1], 1.4):
            return False
    return True


def preparation_course(base: Sequence[MpcReferencePointV3], normal: np.ndarray,
                       config: LargeRecoveryConfig, occupancy: OccupancyMapV3
                       ) -> tuple[tuple[MpcReferencePointV3, ...], list[dict[str, Any]]]:
    """Return one static path and per-site map evidence, never observed teachers.

    normal [N,6] = base progress m, map x/y m, body yaw rad, speed m/s,
    signed offset from base m. Measured normal poses define the goal frame.
    Preparation may shift the original command path, so normal tracking error
    is not necessarily commanded a second time. Actual poses remain labels.
    The path is nominal outside finite preparation patches. Runtime command
    selection must end by release+2 m, before the artificial far return.
    """
    normal = np.asarray(normal, dtype=float)
    if (normal.ndim != 2 or normal.shape[1] != 6 or len(normal) < 20
            or not np.isfinite(normal).all() or np.any(np.diff(normal[:, 0]) <= 0.)
            or len(base) < 20):
        raise ValueError('LARGE_NORMAL_TRACE_SHAPE')
    bs = np.array([p.s_m for p in base])
    bx = np.array([p.x_m for p in base]); by = np.array([p.y_m for p in base])
    # Keep the closing segment out of this open interpolation. Existing CSV
    # wrap export restores the periodic boundary and supplies the PP tail.
    # The official generator already densifies ordinary segments. Densify only
    # changed patches here; a whole-course 0.1 m grid triples the ROS payload.
    patches = [np.arange(site.start_s_m-4., site.release_s_m+22.+.05, .1) for site in config.sites]
    s = np.unique(np.concatenate([bs, *patches]))
    s = s[np.r_[True, np.diff(s) > 1e-5]]
    x = np.interp(s, bs, bx); y = np.interp(s, bs, by)
    evidence = []
    for site in config.sites:
        lo, hi = site.start_s_m-4., site.release_s_m+22.
        if not normal[0, 0] <= lo < hi <= normal[-1, 0] or hi > bs[-1]:
            raise ValueError('LARGE_TRACE_PATCH_COVERAGE')
        mask = (s >= lo) & (s <= hi)
        progress = s[mask]
        if site.preparation_origin == 'nominal_path':
            nx, ny = np.interp(progress, bs, bx), np.interp(progress, bs, by)
        else:
            nx, ny = [np.interp(progress, normal[:, 0], normal[:, k]) for k in (1, 2)]
        yaw = np.interp(progress, normal[:, 0], np.unwrap(normal[:, 3]))
        envelope = _smooth((progress-lo)/4.)*(1.-_smooth((progress-(site.release_s_m+14.))/8.))
        lateral = preparation_lateral(progress, site)
        x[mask] += envelope*(nx-x[mask]-np.sin(yaw)*lateral)
        y[mask] += envelope*(ny-y[mask]+np.cos(yaw)*lateral)
        preview = (s >= site.release_s_m+2.) & (s <= site.release_s_m+14.)
        preview_arc_m = float(np.hypot(np.diff(x[preview]), np.diff(y[preview])).sum())
        if preview_arc_m < 10.:
            raise ValueError('LARGE_PREPARATION_PREVIEW_TOO_SHORT')
        active = (s >= site.start_s_m-1.) & (s <= site.release_s_m+2.)
        preparation_pass = _map_free(occupancy, np.column_stack((x[active], y[active])))
        # Screen a candidate measured-normal return separately; the actual
        # official PP response still requires an AWSIM pilot.
        rs = np.linspace(site.release_s_m, site.release_s_m+site.return_length_m+1.,
                         round((site.return_length_m+1.)/.05)+1)
        rx, ry = [np.interp(rs, normal[:, 0], normal[:, k]) for k in (1, 2)]
        ryaw = np.interp(rs, normal[:, 0], np.unwrap(normal[:, 3]))
        shift = site.target_offset_m*(1.-_smooth((rs-site.release_s_m)/site.return_length_m))
        return_pass = _map_free(occupancy, np.column_stack((rx-np.sin(ryaw)*shift, ry+np.cos(ryaw)*shift)))
        evidence.append(dict(site_id=site.site_id, preparation_map_pass=preparation_pass,
                             corner_id=site.corner_id, target_heading_rad=site.target_heading_rad,
                             candidate_return_map_pass=return_pass, map_radius_m=1.4,
                             hidden_return_preview_arc_m=preview_arc_m,
                             physical_dynamic_recovery_proven=False))
        if not preparation_pass or not return_pass:
            raise ValueError('LARGE_SITE_MAP_REJECTED:'+site.site_id)
    actual_s, yaw, kappa = _recompute_geometry(x, y)
    points = tuple(MpcReferencePointV3(float(a), float(b), float(c), float(d), float(e), 5/3.6, 0.)
                   for a, b, c, d, e in zip(actual_s, x, y, yaw, kappa))
    return points, evidence
