"""Low-speed V4-20 simulation trial policy. No ROS; XY metres, angles rad.

Uses the contiguous near 3 m of the predicted path for Pure Pursuit. All 46 raw
points remain recorded. Speed is an explicit trial policy, not model output.
"""
from __future__ import annotations
import math
import numpy as np
from .waypoint_controller import ControllerConfig, control_from_waypoints


def tracking_command(raw: np.ndarray, observed_pose: tuple, current_pose: tuple,
                     speed_mps: float) -> dict:
    """Transform float XY[46,2] observation-root to current rear axle, then PP."""
    xy = np.asarray(raw, dtype=float)
    if xy.shape != (46, 2) or not np.isfinite(xy).all():
        raise ValueError('PATH_SHAPE_FINITE')
    if not np.isfinite((*observed_pose, *current_pose, speed_mps)).all():
        raise ValueError('STATE_FINITE')
    if not -.05 <= speed_mps <= .45:
        raise ValueError('OVERSPEED_OR_REVERSE')
    def rotation(a):
        return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    world = xy @ rotation(observed_pose[2]).T + observed_pose[:2]
    local = (world - current_pose[:2]) @ rotation(current_pose[2])
    # root -> rear axle x=-0.484 m, verified current AWSIM geometry.
    local[:, 0] += .484
    lengths = np.linalg.norm(np.diff(local, axis=0), axis=1)
    arc = np.r_[0., np.cumsum(lengths)]
    end = min(len(local), int(np.searchsorted(arc, 3.)) + 1)
    near = local[:end]
    if len(near) < 4 or np.linalg.norm(near[-1]) < 1.:
        raise ValueError('SHORT_PREFIX')
    d = np.diff(near, axis=0)
    discontinuities = np.flatnonzero(np.linalg.norm(d, axis=1) > 1.6)
    cutoff_reason = None
    if len(discontinuities):
        end = min(end, int(discontinuities[0]) + 1)
        cutoff_reason = 'DISCONTINUITY'
    # Only non-degenerate segments define headings; no point is altered.
    valid_indices = np.flatnonzero(np.linalg.norm(d, axis=1) > .01)
    valid = d[valid_indices]
    angles = np.arctan2(valid[:, 1], valid[:, 0])
    turns = np.arctan2(np.sin(np.diff(angles)), np.cos(np.diff(angles)))
    folds = np.flatnonzero(np.abs(turns) > 1.2)
    if len(folds):
        end = min(end, int(valid_indices[folds[0]+1]) + 1)
        cutoff_reason = 'FOLDBACK'
    near = local[:end]
    if len(near)<4 or np.linalg.norm(near[-1])<1.:
        raise ValueError('INSUFFICIENT_CONTIGUOUS_PREFIX')
    remaining_m = float(np.linalg.norm(near[-1])) - .484
    if remaining_m < .2 + max(0.,speed_mps)*.5 + speed_mps**2/2.:
        raise ValueError('PREFIX_STOPPING_DISTANCE')
    candidates = np.flatnonzero((np.linalg.norm(near, axis=1) >= 1.) & (near[:, 0] > .5))
    if not len(candidates):
        raise ValueError('NO_FORWARD_LOOKAHEAD')
    target = near[candidates[0]]
    required = math.atan(2 * 1.087 * target[1] / float(target @ target))
    if abs(required) > .5:
        raise ValueError('STEERING_INFEASIBLE')
    cmd = control_from_waypoints(target[None], .25, speed_mps,
        ControllerConfig(wheelbase_m=1.087, min_lookahead_m=1., max_steer_rad=.5,
                         min_accel_mps2=-1., max_accel_mps2=.5, speed_kp=2.))
    return dict(steer_rad=cmd.steering_rad, acceleration_mps2=cmd.acceleration_mps2,
                target_speed_mps=.25, lookahead_rear_m=target.tolist(), prefix_points=end,
                prefix_cutoff_reason=cutoff_reason, remaining_prefix_m=remaining_m)


def check_scan(ranges: np.ndarray, angle_min: float, angle_increment: float,
               range_min: float, range_max: float, speed_mps: float) -> float:
    """Forward low-speed stopping corridor; unknown rays reject, no map inputs.

    LiDAR root offset +1.165 m. Width +/-0.85 m includes kart plus margin.
    This is a trial proximity monitor, not proof of collision-free footprint.
    """
    r = np.asarray(ranges, dtype=float)
    if r.ndim != 1 or len(r) < 100 or not np.isfinite([angle_min,angle_increment,range_min,range_max,speed_mps]).all():
        raise ValueError('SCAN_CONTRACT')
    a = angle_min + np.arange(len(r)) * angle_increment
    sector = np.abs(a) <= 1.3
    if sector.sum() < 100 or a.min() > -1.3 or a.max() < 1.3:
        raise ValueError('SCAN_COVERAGE')
    if np.any(np.isnan(r[sector]) | (r[sector] < range_min) | np.isneginf(r[sector])):
        raise ValueError('SCAN_UNKNOWN')
    rr = np.minimum(r, range_max)
    x, y = 1.165 + rr*np.cos(a), rr*np.sin(a)
    in_lane = sector & (np.abs(y) <= .85) & (x >= 0.)
    clearance = float(np.min(x[in_lane], initial=range_max)) - 1.5
    required = .4 + max(0.,speed_mps)*.5 + speed_mps**2/2.
    if clearance < required:
        raise ValueError('STOPPING_CORRIDOR_OCCUPIED')
    return clearance
