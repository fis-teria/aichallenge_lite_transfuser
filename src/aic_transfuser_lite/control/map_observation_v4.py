"""Diagnostic-only map comparison, never clearance or motion permission."""
from __future__ import annotations
import numpy as np
from .static_course_map_v4 import StaticCourseMap, OCCUPIED, FREE, UNKNOWN


def compare_observation(course: StaticCourseMap, utm_base_pose: np.ndarray, scan: dict,
                        *, lidar_x_m: float, lidar_y_m: float) -> dict:
    """Pose [UTM54N east m,north m,yaw rad]; fixed 54SUE square hypothesis.

    Header-near scan/pose comparison only, not calibrated/deskewed clearance.
    """
    z=np.asarray(utm_base_pose,dtype=float)
    if z.shape!=(3,) or not np.isfinite(z).all(): raise ValueError('MAP_POSE')
    xy=z[:2]-np.array([300000.,3900000.])
    if not ((0<=xy)&(xy<100000)).all(): raise ValueError('OUTSIDE_FIXED_MGRS_SQUARE')
    r=np.asarray(scan['ranges'],dtype=float)
    if r.shape!=(750,) or not np.isfinite([scan['angle_min'],scan['angle_increment'],
            scan['range_min'],scan['range_max'],lidar_x_m,lidar_y_m]).all():
        raise ValueError('MAP_SCAN_GEOMETRY')
    if scan['angle_increment']<=0 or not 0<=scan['range_min']<scan['range_max']:
        raise ValueError('MAP_SCAN_LIMITS')
    a=scan['angle_min']+np.arange(750)*scan['angle_increment']
    valid=np.isfinite(r)&(r>scan['range_min'])&(r<scan['range_max'])
    local=np.c_[r[valid]*np.cos(a[valid])+lidar_x_m,r[valid]*np.sin(a[valid])+lidar_y_m]
    c,s=np.cos(z[2]),np.sin(z[2]);hits=local@np.array([[c,s],[-s,c]])+xy
    values=course.query(hits);near=np.zeros(len(hits),dtype=bool)
    for dx in (-.2,-.1,0.,.1,.2):
        for dy in (-.2,-.1,0.,.1,.2):
            if dx*dx+dy*dy<=.2**2+1e-12:
                near |= course.query(hits+np.array([dx,dy]))==OCCUPIED
    return dict(map_xy_m=xy.tolist(),map_yaw_rad=float(z[2]),
        centre_class=int(course.query(xy[None])[0]),valid_hit_count=int(valid.sum()),
        exact_hit_counts={k:int((values==v).sum()) for k,v in [('occupied',OCCUPIED),('free',FREE),('unknown',UNKNOWN)]},
        hits_near_occupied_020m=int(near.sum()),hit_map_xy_m=hits.tolist(),
        map_image_sha256=course.provenance.get('image_sha256'),
        timing='HEADER_NEAREST_WITHIN_50MS_NOT_DESKEWED',runtime_permission=False,
        geometry_binding_verified=False)
