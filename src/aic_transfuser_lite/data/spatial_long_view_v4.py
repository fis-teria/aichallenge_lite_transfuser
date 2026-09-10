"""Separate 20m observed-diagnostic candidate; never changes the fixed 2m teacher."""
import numpy as np
from .spatial_diagnostic_view_v4 import spatial_target

# Near 0.1m, middle 0.5m, far 1m; not an observed runtime validity mask.
LONG_GRID = np.r_[np.arange(1,21)/10, np.arange(5,21)/2, np.arange(11,21)].astype(np.float64)


def long_spatial_target(future: np.ndarray) -> dict:
    """Saved h30[30,8] -> observed XY[46,2], mask[46]; no extrapolation.

    Reuses fixed prefix/time/jump checks. This candidate remains diagnostic only;
    its observed path is not an obstacle-free or intended-route oracle.
    """
    base=spatial_target(future)
    reliable=[base['raw_prefix_xy'][0]]
    for point in base['raw_prefix_xy'][1:]:
        if np.linalg.norm(point-reliable[-1])>=.005: reliable.append(point)
    if 'stationary_jitter_provisional' in base['flags']: reliable=reliable[:1]
    xy=np.asarray(reliable)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    mask=LONG_GRID<=arc[-1]
    target=np.zeros((46,2),dtype=np.float32)
    for axis in (0,1):target[mask,axis]=np.interp(LONG_GRID[mask],arc,xy[:,axis])
    return dict(xy=target,mask=mask,grid_m=LONG_GRID.copy(),
        reliable_prefix_xy=xy, raw_length_m=base['raw_length_m'],
        distance_status=base['distance_status'], prefix_count=base['prefix_count'],
        maximum_hold_s=base['maximum_hold_s'],
        observed_length_m=float(arc[-1]),cut_reason=base['cut_reason'],flags=base['flags'],
        tier='LONG_HORIZON_OBSERVED_DIAGNOSTIC_CANDIDATE',extrapolation=False)
