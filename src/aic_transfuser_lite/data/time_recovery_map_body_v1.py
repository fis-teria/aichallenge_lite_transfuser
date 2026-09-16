"""Static map screening of the standard inflated AWSIM kart rectangle.

This screens a planned path, not the actual dynamic response or stopping sweep.
The runtime LiDAR/vehicle-motion guard remains independent. Map cells outside
the image or not explicitly free are occupied. Units are metres and radians.
"""
from __future__ import annotations

import math
import numpy as np

from .recovery_reference_v3 import OccupancyMapV3


POLICY = 'oriented_body_v1'
# Standard runtime body at rear axle: [-.510, 1.984] x +/-.85 m.
# The rear axle is +.001 m in base_link (verified AWSIM geometry).
REAR_M, FRONT_M, HALF_WIDTH_M = -.509, 1.985, .85
BODY_RADIUS_M = math.hypot(FRONT_M, HALF_WIDTH_M)


def body_path_is_free(occupancy: OccupancyMapV3, poses: np.ndarray) -> bool:
    """Check continuous piecewise-linear base_link poses [N,3] (x,y,yaw).

    Heading follows the shortest arc between vertices. Translation plus the
    farthest-corner angular travel bounds displacement between samples. Half
    that travel inflates each sampled rectangle. Adding each map cell's half
    diagonal conservatively covers cells intersected between their centres.
    Every occupied cell intersecting the body is rejected, not just its edge.
    """
    occupancy.validate()
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
        raise ValueError('MAP_BODY_POSES_SHAPE')
    resolution = occupancy.resolution_m_per_px
    height, width = occupancy.free.shape
    step = min(.05, resolution/2.)

    def free_at(pose: np.ndarray, displacement: float) -> bool:
        inflation = displacement + resolution/math.sqrt(2.)
        low_x, high_x = REAR_M-inflation, FRONT_M+inflation
        half_y = HALF_WIDTH_M+inflation
        local = np.array([[low_x,-half_y],[high_x,-half_y],[high_x,half_y],[low_x,half_y]])
        c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
        rotation = np.array([[c,-s],[s,c]])
        corners = local @ rotation.T + pose[:2]
        px = (corners[:,0]-occupancy.origin_x_m)/resolution
        py = (height-1)-(corners[:,1]-occupancy.origin_y_m)/resolution
        x0, x1 = math.floor(float(px.min())), math.ceil(float(px.max()))
        y0, y1 = math.floor(float(py.min())), math.ceil(float(py.max()))
        if x0 < 0 or y0 < 0 or x1 >= width or y1 >= height:
            return False
        rr, cc = np.where(~occupancy.free[y0:y1+1,x0:x1+1])
        dx = (cc+x0)*resolution+occupancy.origin_x_m-pose[0]
        dy = (height-1-(rr+y0))*resolution+occupancy.origin_y_m-pose[1]
        forward, left = dx*c+dy*s, -dx*s+dy*c
        return not bool(np.any((forward >= low_x) & (forward <= high_x) & (abs(left) <= half_y)))

    for a, b in zip(poses[:-1], poses[1:]):
        yaw_delta = math.atan2(math.sin(float(b[2]-a[2])), math.cos(float(b[2]-a[2])))
        travel_bound = float(np.linalg.norm(b[:2]-a[:2])) + BODY_RADIUS_M*abs(yaw_delta)
        count = max(1, math.ceil(travel_bound/step))
        delta = np.array([b[0]-a[0], b[1]-a[1], yaw_delta])
        for fraction in np.linspace(0.,1.,count+1):
            if not free_at(a+fraction*delta, travel_bound/(2.*count)):
                return False
    return True


def body_polyline_is_free(occupancy: OccupancyMapV3, xy: np.ndarray) -> bool:
    """Derive tangent headings from a planned polyline [N,2], then screen it."""
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all():
        raise ValueError('MAP_BODY_XY_SHAPE')
    if np.any(np.linalg.norm(np.diff(xy,axis=0),axis=1) < 1e-9):
        raise ValueError('MAP_BODY_DUPLICATE_VERTEX')
    tangent = np.gradient(xy,axis=0)
    if np.any(np.linalg.norm(tangent,axis=1) < 1e-9):
        raise ValueError('MAP_BODY_UNDEFINED_HEADING')
    yaw = np.arctan2(tangent[:,1],tangent[:,0])
    return body_path_is_free(occupancy,np.column_stack((xy,yaw)))
