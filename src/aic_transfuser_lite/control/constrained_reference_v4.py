"""Separate, bounded SIM diagnostic reference; never edits model output.

One seven-knot steering shooting fit. Ordered raw arc-length correspondence,
fixed prefix selection, and union-breakpoint continuous polyline certificate.
Acceptance here is geometry only, NEVER a live permission or free-space claim.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.optimize import minimize

from .spatial_path_adapter_v4 import prepare, transform, wrap
from .spatial_tracking_contracts_v4 import PreparedPath, SpatialPathCandidate, sha


def ordered_polyline_error(reference_s: np.ndarray, reference_xy: np.ndarray,
                           target_s: np.ndarray, target_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return all breakpoints [K] and ordered XY errors [K,2], in metres.

    On each union interval both polylines are affine in the same parameter.
    Their difference is affine and its Euclidean norm is convex. Therefore its
    maximum occurs at an endpoint; no Lipschitz grid-gap penalty is needed.
    This certifies these interpolated polylines only, not a vehicle trajectory,
    time correspondence, teacher validity, or free space.
    """
    arrays = [np.asarray(a, dtype=float) for a in (reference_s, reference_xy, target_s, target_xy)]
    rs, rx, ts, tx = arrays
    for s, xy in ((rs, rx), (ts, tx)):
        if (s.ndim != 1 or len(s) < 2 or xy.shape != (len(s), 2) or
                not np.isfinite(s).all() or not np.isfinite(xy).all() or np.any(np.diff(s) <= 0)):
            raise ValueError('INVALID_ORDERED_POLYLINE')
    if rs[0] != ts[0] or rs[-1] != ts[-1]:
        raise ValueError('POLYLINE_DOMAIN_MISMATCH')
    check = np.unique(np.r_[rs, ts])
    diff = np.column_stack([np.interp(check, rs, rx[:, k])-np.interp(check, ts, tx[:, k]) for k in (0, 1)])
    return check, diff


def constrained_reference(candidate: SpatialPathCandidate, cfg: dict,
                          initial_rear_in_base: np.ndarray) -> PreparedPath:
    """Input raw float32[20,2], rear [x_m,y_m,yaw_rad,v_mps,delta_rad].

    Seven steering knots are integrated with midpoint bicycle geometry on a
    <= 0.02 m grid. The union of reference and target breakpoints certifies
    the maximum ordered polyline error, including corners and the connection.
    Clearance must subsequently be checked from a current sensor observation.
    """
    raw = candidate.raw_xy
    z = np.asarray(initial_rear_in_base, dtype=float)
    diag = dict(policy='SIM_BOUNDED_STEERING_SHOOTING_V2', raw_hash=candidate.raw_hash,
                raw_bits_hex=raw.tobytes().hex(), nominal_s_m=candidate.nominal_s.tolist(),
                raw_unchanged=True, clearance_verified=False, runtime_permission=False,
                fit_solve_count=0, origin_in_raw=False)

    def reject(reason: str) -> PreparedPath:
        return PreparedPath(np.empty((0, 2)), np.empty(0), np.empty(0, dtype=int),
                            np.empty(0), np.empty(0), diag, reason)

    if raw.shape != (20, 2) or raw.dtype != np.float32 or z.shape != (5,):
        return reject('SHAPE_DTYPE')
    if not np.isfinite(raw).all() or not np.isfinite(z).all():
        return reject('NONFINITE')
    if candidate.frame != 'base_link@t_obs' or candidate.units != 'm':
        return reject('FRAME_OR_UNITS')
    if candidate.nominal_s.shape != (20,) or not np.isfinite(candidate.nominal_s).all():
        return reject('NOMINAL_CONTRACT')
    deviation = float(cfg['reference_max_deviation_m'])
    if not 0 < deviation <= .10:
        return reject('DEVIATION_POLICY')
    diag['old_raw_gate'] = prepare(candidate, cfg, {'saved': True}).reason
    delta_max = cfg['steering_limit_rad']
    if abs(z[4]) > delta_max or z[3] < 0 or z[3] > cfg['maximum_speed_mps']:
        return reject('INITIAL_STATE_LIMIT')
    raw64 = raw.astype(float)
    ds_raw = np.linalg.norm(np.diff(raw64, axis=0), axis=1)
    if np.any(ds_raw < cfg['minimum_segment_m']):
        return reject('DUPLICATE_ORDER_UNDEFINED')
    if np.any(ds_raw > cfg['maximum_segment_m']):
        return reject('RAW_GAP')
    raw_s = np.r_[0., np.cumsum(ds_raw)]
    headings = np.arctan2(np.diff(raw64, axis=0)[:, 1], np.diff(raw64, axis=0)[:, 0])
    cusps = np.flatnonzero(np.abs(wrap(np.diff(headings))) > cfg['cusp_angle_rad'])
    count = int(cusps[0] + 2) if len(cusps) else 20
    diag.update(raw_actual_s_m=raw_s.tolist(), used_raw_indices=list(range(count)),
                unused_tail_indices=list(range(count, 20)),
                trim_reason='FIRST_CUSP' if count < 20 else None)
    # Touching / crossing branches before or after the prefix are ambiguous.
    def cross(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0]*b[1] - a[1]*b[0])
    for i in range(19):
        for j in range(i+2, 19):
            a, b = raw64[i:i+2]; c, d = raw64[j:j+2]
            overlap = np.all(np.maximum(np.minimum(a,b),np.minimum(c,d)) <=
                             np.minimum(np.maximum(a,b),np.maximum(c,d)))
            if overlap and cross(b-a,c-a)*cross(b-a,d-a) <= 0 and cross(d-c,a-c)*cross(d-c,b-c) <= 0:
                return reject('SELF_INTERSECTION')
    connection = float(np.linalg.norm(raw64[0] - z[:2]))
    diag.update(initial_connection_length_m=connection,
                initial_connection_raw_correspondence=False,
                initial_rear_in_base=z.tolist())
    if connection > cfg['reference_max_connection_m']:
        return reject('INITIAL_CONNECTION_TOO_LONG')
    if count < 4 or raw_s[count-1] < cfg['reference_min_support_m']:
        return reject('SHORT_PREFIX')
    target_s = np.r_[0., connection + raw_s[:count]]
    target_xy = np.vstack([z[:2], raw64[:count]])
    # Coincident first point is removed only from the computational connection.
    if connection < 1e-9:
        target_s = target_s[1:]; target_xy = target_xy[1:]
    total = target_s[-1]
    q = np.linspace(0, total, int(np.ceil(total/.02))+1)
    ds = np.diff(q); mid = (q[:-1]+q[1:])/2
    knots = np.linspace(0, total, 7)
    target = np.column_stack([np.interp(q, target_s, target_xy[:, k]) for k in (0,1)])
    check = np.unique(np.r_[q, target_s])
    target_check = np.column_stack([np.interp(check, target_s, target_xy[:, k]) for k in (0,1)])
    check_gap = float(np.diff(check).max())
    # Retain a small numerical margin, not an arbitrary discretization penalty.
    certificate_gap = 1e-9
    # SLSQP feasibility tolerance below is 1e-8. Its optimization margin must
    # exceed that tolerance plus the certificate rounding allowance; otherwise
    # a successful boundary solution can fail the strict final 0.10 m check.
    optimizer_margin = 1e-7
    allowed_at_samples = deviation - optimizer_margin
    if allowed_at_samples <= 0:
        return reject('CERTIFICATE_RESOLUTION')

    def shoot(free: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        delta = np.interp(mid, knots, np.r_[z[4], free])
        dyaw = ds*np.tan(delta)/cfg['wheelbase_m']
        yaw = z[2] + np.r_[0., np.cumsum(dyaw)]
        angle = yaw[:-1] + dyaw/2
        xy = z[:2] + np.vstack([np.zeros(2), np.cumsum(ds[:,None]*np.c_[np.cos(angle),np.sin(angle)], axis=0)])
        return xy, yaw, delta

    def errors(free: np.ndarray) -> np.ndarray:
        xy, _, _ = shoot(free)
        xy_check = np.column_stack([np.interp(check,q,xy[:, k]) for k in (0,1)])
        return np.linalg.norm(xy_check-target_check,axis=1)

    def constraints(free: np.ndarray) -> np.ndarray:
        # Largest possible test speed, not a claimed applied speed.
        rate = np.diff(np.r_[z[4], free])/np.diff(knots)*cfg['maximum_speed_mps']
        lateral = cfg['maximum_speed_mps']**2*np.tan(np.r_[z[4], free])/cfg['wheelbase_m']
        return np.r_[allowed_at_samples-errors(free),
                     cfg['steering_rate_limit_rad_s']-np.abs(rate),
                     cfg['lateral_acceleration_limit_mps2']-np.abs(lateral)]

    def objective(free: np.ndarray) -> float:
        xy, _, _ = shoot(free)
        return float(np.sum((xy-target)**2)+.001*np.sum(np.diff(np.r_[z[4],free])**2))

    start = time.monotonic(); diag['fit_solve_count'] = 1
    try:
        result = minimize(objective, np.full(6, z[4]), method='SLSQP',
                          bounds=[(-delta_max, delta_max)]*6,
                          constraints=[{'type':'ineq','fun':constraints}],
                          options={'maxiter':60,'ftol':1e-9})
        xy, yaw, delta = shoot(result.x)
        error = errors(result.x)
        residual = float(max(0., -np.min(constraints(result.x))))
        diag.update(fit_wall_s=time.monotonic()-start, fit_success=bool(result.success),
                    fit_message=str(result.message), fit_iterations=int(result.nit),
                    ordered_parameter_m=q.tolist(), raw_correspondence_s_m=(q-connection).tolist(),
                    raw_correspondence_mask=(q>=connection).tolist(),
                    ordered_error_samples_m=error.tolist(), certificate_gap_m=certificate_gap,
                    certificate_policy='UNION_BREAKPOINT_CONVEX_NORM_V2',
                    optimizer_margin_m=optimizer_margin,
                    check_parameter_m=check.tolist(), max_check_spacing_m=check_gap,
                    legacy_lipschitz_bound_m=float(error.max()+check_gap),
                    maximum_deviation_bound_m=float(error.max()+certificate_gap),
                    initial_connection_fit_error_m=float(error[check<=connection].max()),
                    constraint_residual=residual, steering_knots_rad=np.r_[z[4],result.x].tolist())
        if not np.isfinite(xy).all() or not np.isfinite(error).all():
            return reject('NONFINITE_REFERENCE')
        if not result.success or residual > 1e-8 or error.max()+certificate_gap > deviation:
            return reject('BOUNDED_FIT_INFEASIBLE')
        if diag['fit_wall_s'] > cfg['reference_wall_limit_s']:
            return reject('REFERENCE_DEADLINE')
    except Exception as exc:
        diag['fit_exception'] = repr(exc)
        return reject('REFERENCE_EXCEPTION')
    actual_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    diag.update(reference_actual_s_m=actual_s.tolist(), usable_prefix_s_m=float(actual_s[-1]),
                reference_id=sha(candidate.raw_xy.tobytes()+xy.tobytes()),
                reference_yaw_rad=yaw.tolist(), reference_frame='base_link@t_obs',
                raw_unchanged=candidate.raw_xy.tobytes().hex()==diag['raw_bits_hex'])
    source_indices = np.where(q < connection, -1,
                              np.minimum(np.searchsorted(raw_s[:count], q-connection, side='right')-1, count-1))
    curvature = np.r_[np.tan(delta)/cfg['wheelbase_m'], np.tan(delta[-1])/cfg['wheelbase_m']]
    return PreparedPath(xy, actual_s, source_indices, yaw[:-1], curvature, diag, None)


def reference_to_world(path: PreparedPath, world_base_at_observation: np.ndarray,
                       *, observation_epoch: str, control_epoch: str) -> PreparedPath:
    """Transform once from observation base. Caller supplies evidenced rear pose."""
    if path.reason or observation_epoch != control_epoch:
        raise ValueError('REJECTED_PATH_OR_EPOCH_MISMATCH')
    pose = np.asarray(world_base_at_observation, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('MISSING_FRAME_TRANSFORM')
    return PreparedPath(transform(path.world_xy, pose), path.actual_s.copy(), path.source_indices.copy(),
                        path.heading+pose[2], path.curvature.copy(),
                        dict(path.diagnostics, world_base_at_observation=pose.tolist(),
                             observation_epoch=observation_epoch, reference_frame='world'), None)
