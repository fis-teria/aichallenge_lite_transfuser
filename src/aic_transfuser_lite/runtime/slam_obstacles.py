"""Recent SLAM-aligned occupancy and observed obstacle surfaces, no ROS.

No preloaded map, simulator objects, GNSS, IMU or vehicle classification.
Stationary returns remain obstacles even after integration into the local map.
Poses are [x m, y m, yaw rad]; points/paths have shape [N,2], in metres.
"""
from __future__ import annotations

import math
import numpy as np

from .lidar_map_localization import inverse, pose_array, transform


def scan_geometry(ranges: np.ndarray, angle_min: float, angle_increment: float,
                  range_min: float, range_max: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return finite ray endpoints [N,2], hit mask [N], original beam IDs [N]."""
    r = np.asarray(ranges, dtype=float)
    if (r.ndim != 1 or not 3 <= len(r) <= 10000 or
            not np.isfinite([angle_min, angle_increment, range_min, range_max]).all()
            or angle_increment <= 0 or not 0 <= range_min < range_max):
        raise ValueError('OBSTACLE_SCAN_CONTRACT')
    bad = np.isnan(r) | np.isneginf(r) | (r <= range_min) | (np.isfinite(r) & (r > range_max))
    if np.mean(bad) > .2:
        raise ValueError('OBSTACLE_SCAN_MALFORMED')
    ids = np.flatnonzero(~bad)[::2]
    limit = min(20., range_max)
    hit = np.isfinite(r[ids]) & (r[ids] < limit-.01)
    distance = np.minimum(r[ids], limit)
    angle = angle_min+ids*angle_increment
    return distance[:, None]*np.column_stack((np.cos(angle), np.sin(angle))), hit, ids


def path_clearance(points_body: np.ndarray, path_body: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Swept circular 0.85 m half-width corridor; returns distance and arc length.

    An explicit 1.8 m front extension accounts for the body beyond the last
    predicted base position. This diagnostic is not a certified swept footprint.
    """
    p, path = np.asarray(points_body, dtype=float), np.asarray(path_body, dtype=float)
    if (p.ndim != 2 or p.shape[1:] != (2,) or path.ndim != 2 or path.shape[1:] != (2,)
            or not 2 <= len(path) <= 200 or not np.isfinite(p).all() or not np.isfinite(path).all()):
        raise ValueError('OBSTACLE_PATH_SHAPE_OR_FINITE')
    route = np.vstack(([0., 0.], path))
    delta = np.diff(route, axis=0); lengths = np.linalg.norm(delta, axis=1)
    starts, delta, lengths = route[:-1][lengths > .01], delta[lengths > .01], lengths[lengths > .01]
    if not len(lengths):
        raise ValueError('OBSTACLE_PATH_TOO_SHORT')
    starts = np.vstack((starts, starts[-1]+delta[-1]))
    delta = np.vstack((delta, delta[-1]/lengths[-1]*1.8))
    lengths = np.r_[lengths, 1.8]
    t = np.clip(np.sum((p[:, None]-starts)*delta, axis=2)/(lengths**2), 0., 1.)
    distances = np.linalg.norm(p[:, None]-(starts+t[..., None]*delta), axis=2)
    nearest = np.argmin(distances, axis=1)
    along = np.r_[0., np.cumsum(lengths)[:-1]][nearest]+t[np.arange(len(p)), nearest]*lengths[nearest]
    return distances[np.arange(len(p)), nearest], along


class RecentOccupancy:
    """50 m square, 0.2 m resolution, 2 s TTL; -1 unknown / 0 free / 100 hit.

    Ray clearing removes old positions of moving objects. Unknown is never free.
    The grid uses SLAM poses but does not feed back into SLAM or filter detection.
    """
    resolution_m = .2
    size = 250
    ttl_s = 2.

    def __init__(self) -> None:
        self.origin: np.ndarray | None = None
        self.values = np.full((self.size, self.size), -1, dtype=np.int8)
        self.seen = np.full((self.size, self.size), -np.inf)

    def update(self, stamp_s: float, origin_xy: np.ndarray, endpoints: np.ndarray, hits: np.ndarray) -> None:
        center = np.floor(origin_xy/self.resolution_m).astype(int)-self.size//2
        if self.origin is not None:
            shift = center-self.origin
            if np.any(np.abs(shift) >= self.size):
                self.values.fill(-1); self.seen.fill(-np.inf)
            elif np.any(shift):
                self.values = np.roll(self.values, (-int(shift[1]), -int(shift[0])), axis=(0, 1))
                self.seen = np.roll(self.seen, (-int(shift[1]), -int(shift[0])), axis=(0, 1))
                for axis, delta in enumerate(shift[::-1]):
                    region = [slice(None), slice(None)]
                    region[axis] = slice(-int(delta), None) if delta > 0 else slice(None, -int(delta))
                    if delta:
                        self.values[tuple(region)] = -1; self.seen[tuple(region)] = -np.inf
        self.origin = center
        self.values[stamp_s-self.seen > self.ttl_s] = -1
        if not len(endpoints):
            return
        delta = endpoints-origin_xy
        count = np.maximum(1, np.ceil(np.linalg.norm(delta, axis=1)/self.resolution_m).astype(int))
        step = np.arange(int(count.max()))
        valid = step[None, :] < count[:, None]
        rays = origin_xy+delta[:, None, :]*(step[None, :]/count[:, None])[..., None]
        cells = np.floor(rays[valid]/self.resolution_m).astype(int)-center
        cells = cells[np.all((cells >= 0) & (cells < self.size), axis=1)]
        self.values[cells[:, 1], cells[:, 0]] = 0; self.seen[cells[:, 1], cells[:, 0]] = stamp_s
        cells = np.floor(endpoints[hits]/self.resolution_m).astype(int)-center
        cells = cells[np.all((cells >= 0) & (cells < self.size), axis=1)]
        self.values[cells[:, 1], cells[:, 0]] = 100; self.seen[cells[:, 1], cells[:, 0]] = stamp_s


class SlamObstacleDetector:
    """Observed surfaces with tentative IDs, occupancy and optional path overlap.

    Surface centroid velocity changes with visibility: it is NOT object speed.
    No stationary/slow-vehicle semantic claims or motion commands are produced.
    """
    def __init__(self) -> None:
        self.grid = RecentOccupancy()
        self.last_ns: int | None = None
        self.tracks: dict[int, dict] = {}
        self.sequence = 0

    def update(self, stamp_ns: int, ranges: np.ndarray, angle_min: float, angle_increment: float,
               range_min: float, range_max: float, lidar_pose: np.ndarray,
               path_world: np.ndarray | None = None) -> dict:
        pose = pose_array(lidar_pose)
        if type(stamp_ns) is not int or stamp_ns < 0 or (self.last_ns is not None and stamp_ns <= self.last_ns):
            raise ValueError('OBSTACLE_STAMP_RESET_REQUIRED')
        rays, hits, ids = scan_geometry(ranges, angle_min, angle_increment, range_min, range_max)
        world = transform(rays, pose)
        base = pose.copy(); base[:2] -= 1.65*np.array([math.cos(pose[2]), math.sin(pose[2])])
        body = transform(world, inverse(base))
        ego = (body[:, 0] >= -.6) & (body[:, 0] <= 1.8) & (np.abs(body[:, 1]) <= .7)
        # Do not clear free space through self returns.
        self.grid.update(stamp_ns/1e9, pose[:2], world[~ego], hits[~ego])
        selected = np.flatnonzero(hits & ~ego)
        occupied = body[selected]
        path_valid = path_world is not None
        blocking = np.zeros(len(selected), dtype=bool); arc = np.full(len(selected), np.inf)
        if path_valid:
            distance, arc = path_clearance(occupied, transform(path_world, inverse(base)))
            blocking = (distance <= .85) & (occupied[:, 0] > 1.8)
        groups = []
        if len(selected):
            breaks = np.flatnonzero((np.diff(ids[selected]) > 2) |
                                    (np.linalg.norm(np.diff(world[selected], axis=0), axis=1) > .4))+1
            groups = np.split(np.arange(len(selected)), breaks)
        available = {k: v for k, v in self.tracks.items() if stamp_ns-v['stamp_ns'] <= 500_000_000}
        updated = {}; surfaces = []
        for group in groups:
            if len(group) < 3:
                continue
            points = world[selected[group]]; center = np.mean(points, axis=0)
            candidates = [(float(np.linalg.norm(center-v['center'])), k) for k, v in available.items()]
            distance, key = min(candidates, default=(math.inf, -1))
            previous = available.pop(key) if distance <= .8 else None
            if previous is None:
                self.sequence += 1; key = self.sequence
            age = 1 if previous is None else previous['hits']+1
            updated[key] = dict(center=center, stamp_ns=stamp_ns, hits=age)
            surfaces.append(dict(id=key, confirmed=age >= 3, consecutive_hits=age,
                center_xy_m=center.tolist(), min_xy_m=points.min(axis=0).tolist(),
                max_xy_m=points.max(axis=0).tolist(), point_count=len(group),
                points_xy_m=points[::max(1, len(points)//100)].tolist(),
                extent_m=float(np.linalg.norm(np.ptp(points, axis=0))),
                classification='unclassified_occupied_surface',
                path_overlap=bool(blocking[group].any()) if path_valid else None,
                nearest_range_m=float(np.linalg.norm(occupied[group], axis=1).min())))
        self.tracks = updated; self.last_ns = stamp_ns
        return dict(stamp_ns=stamp_ns, frame='time_slam_map', input_valid=True,
            base_pose_xyyaw=base.tolist(), surfaces=surfaces, occupied_points=len(selected),
            path_valid=path_valid, path_blocked=bool(blocking.any()) if path_valid else None,
            nearest_path_obstacle_m=float(np.min(arc[blocking])) if blocking.any() else None,
            motion_authority=False, semantic_vehicle_classification=False)
