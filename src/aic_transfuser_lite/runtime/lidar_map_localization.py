"""Planar map registration with wheel prediction; no ROS or global pose input.

The map contains lane boundaries, not certified physical wall surfaces. Outputs
are localization estimates, never collision-clearance or motion authorization.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree


def pose_array(value: object) -> np.ndarray:
    out = np.asarray(value, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError('LOCALIZATION_POSE_SHAPE_OR_FINITE')
    return out.copy()


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """Float [N,2] metres -> float [N,2] metres, pose=(x m,y m,yaw rad)."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('LOCALIZATION_POINTS_SHAPE_OR_FINITE')
    x, y, yaw = pose_array(pose)
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + [x, y]


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a, b = pose_array(a), pose_array(b)
    return np.r_[transform(b[None, :2], a)[0], wrap(a[2]+b[2])]


def inverse(a: np.ndarray) -> np.ndarray:
    a = pose_array(a)
    return np.r_[transform(-a[None, :2], np.array([0., 0., -a[2]]))[0], -a[2]]


def scan_points(ranges: np.ndarray, angle_min: float, angle_increment: float,
                range_min: float, range_max: float) -> np.ndarray:
    """LaserScan metres/radians -> finite [N,2] sensor points, every third beam.

    NaN, inf, out-of-range and saturated returns are omitted. Snapshot AWSIM
    scans are supported; physical rolling-scan deskew is not implemented here.
    """
    r = np.asarray(ranges, dtype=float)
    if (r.ndim != 1 or not 3 <= len(r) <= 10000 or
            not np.isfinite([angle_min, angle_increment, range_min, range_max]).all() or
            angle_increment <= 0 or not 0 <= range_min < range_max):
        raise ValueError('LOCALIZATION_SCAN_CONTRACT')
    valid = np.flatnonzero(np.isfinite(r) & (r > max(.3, range_min)) & (r < min(15., range_max)))[::3]
    a = angle_min + valid*angle_increment
    return r[valid, None] * np.column_stack((np.cos(a), np.sin(a)))


class BoundaryMap:
    """Exact segment projection, with a <=5 cm nearest-candidate spatial index.

    Input segments: float [M,2,2], metres in map frame. Internally subtract a
    fixed origin so map coordinates near 90 km do not degrade conditioning.
    """
    def __init__(self, segments: np.ndarray, *, display_z_m: float = 0.) -> None:
        lines = np.asarray(segments, dtype=float)
        if (lines.ndim != 3 or lines.shape[1:] != (2, 2) or not 2 <= len(lines) <= 20000
                or not np.isfinite(lines).all() or not math.isfinite(display_z_m)):
            raise ValueError('LOCALIZATION_MAP_SHAPE_OR_FINITE')
        self.origin = np.min(lines.reshape(-1, 2), axis=0)
        self.segments = lines - self.origin
        self.display_z_m = display_z_m
        lengths = np.linalg.norm(self.segments[:, 1]-self.segments[:, 0], axis=1)
        if (lengths < 1e-6).any() or lengths.sum() > 20000:
            raise ValueError('LOCALIZATION_MAP_SEGMENT_BUDGET')
        samples, indices = [], []
        for i, (a, b) in enumerate(self.segments):
            n = math.ceil(lengths[i]/.05)
            samples.append(a+np.linspace(0, 1, n+1)[:, None]*(b-a))
            indices.extend([i]*(n+1))
        self.tree = cKDTree(np.concatenate(samples))
        self.indices = np.asarray(indices)

    @classmethod
    def from_lanelet(cls, path: Path) -> 'BoundaryMap':
        if path.stat().st_size > 8*1024*1024:
            raise ValueError('LOCALIZATION_MAP_FILE_BUDGET')
        root = ET.fromstring(path.read_bytes())
        nodes, elevations = {}, []
        for node in root.findall('node'):
            tags = {t.attrib['k']: t.attrib['v'] for t in node.findall('tag')}
            nodes[node.attrib['id']] = [float(tags['local_x']), float(tags['local_y'])]
            elevations.append(float(tags.get('ele', '0')))
        ids = {m.attrib['ref'] for r in root.findall('relation') for m in r.findall('member')
               if m.attrib.get('role') in ('left', 'right') and m.attrib['type'] == 'way'}
        segments = []
        for way in root.findall('way'):
            if way.attrib['id'] not in ids:
                continue
            points = [nodes[n.attrib['ref']] for n in way.findall('nd')]
            segments.extend([[a, b] for a, b in zip(points[:-1], points[1:]) if a != b])
        return cls(np.asarray(segments), display_z_m=float(np.median(elevations)))

    def nearest(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Local-map [N,2] -> distance, projection, normal and endpoint mask."""
        _, samples = self.tree.query(xy, k=8)
        segments = self.segments[self.indices[samples]]
        a, v = segments[:, :, 0], segments[:, :, 1]-segments[:, :, 0]
        fraction = np.clip(np.sum((xy[:, None]-a)*v, axis=2)/np.sum(v*v, axis=2), 0, 1)
        projection = a+fraction[:, :, None]*v
        distance = np.linalg.norm(xy[:, None]-projection, axis=2)
        best = np.argmin(distance, axis=1); rows = np.arange(len(xy))
        direction = v[rows, best]
        normal = np.column_stack((-direction[:, 1], direction[:, 0]))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        at_end = (fraction[rows,best] <= 1e-9) | (fraction[rows,best] >= 1-1e-9)
        return distance[rows, best], projection[rows, best], normal, at_end


@dataclass(frozen=True)
class Match:
    accepted: bool
    reason: str
    pose: np.ndarray
    rank: int
    inliers: int
    fraction: float
    mean_distance_m: float
    p90_distance_m: float
    correction_m: float
    correction_rad: float
    all_mean_distance_m: float
    all_p90_distance_m: float


def match_scan(course: BoundaryMap, points_lidar: np.ndarray, predicted: np.ndarray,
               *, initializing: bool = False) -> Match:
    """Point-to-line registration; wheel prior regularizes weak directions.

    Sensor points [N,2] m. Fixed verified mount: forward 1.65 m, zero yaw.
    Thresholds are bounded diagnostic-localization gates, not a safety rating.
    """
    predicted = pose_array(predicted)
    points = transform(points_lidar, np.array([1.65, 0., 0.]))
    if len(points) > 4000:
        raise ValueError('LOCALIZATION_POINT_BUDGET')
    prior = predicted.copy(); prior[:2] -= course.origin
    estimate = prior.copy()
    radius = 1.2 if initializing else .8
    rank = 0
    for _ in range(12):
        world = transform(points, estimate)
        distance, closest, normals, endpoints = course.nearest(world)
        # Search radius is not an inlier declaration. Reject the furthest 25%
        # of candidates from the solve so unseen corners/objects do not drag a
        # supported wall fit toward a different boundary.
        candidate = distance[distance < radius]
        good = distance <= min(radius, max(.05,float(np.quantile(candidate,.75)))) if len(candidate) else distance < 0
        if good.sum() < 40:
            break
        p = points[good]; n = normals[good]
        yaw = estimate[2]; c, s = math.cos(yaw), math.sin(yaw)
        derivative = p @ np.array([[-s, c], [-c, -s]])
        residual = np.sum((world[good]-closest[good])*n, axis=1)
        jac = np.column_stack((n, np.sum(derivative*n, axis=1)/5.))
        # At a segment endpoint the distance also constrains its tangent.
        # Treating a finite polyline as infinite lines discards corner evidence
        # and can make a real bend look like an unobservable straight corridor.
        end = endpoints[good]
        tangent = np.column_stack((-n[end,1],n[end,0]))
        jac = np.r_[jac,np.column_stack((tangent,np.sum(derivative[end]*tangent,axis=1)/5.))]
        residual = np.r_[residual,np.sum((world[good][end]-closest[good][end])*tangent,axis=1)]
        weight = np.minimum(1., .12/np.maximum(np.abs(residual), 1e-12))
        hessian = (jac.T*weight) @ jac
        eigenvalues, basis = np.linalg.eigh(hessian)
        strong = eigenvalues > max(1., eigenvalues[-1]*.015)
        rank = int(strong.sum())
        if rank < 2:
            break
        # Exact straight corridors have a null longitudinal direction: leave it
        # unchanged. A gentle bend carries weak but real longitudinal evidence;
        # ridge regularization limits that update instead of throwing it away
        # until a hard rank threshold is crossed. Rank still reports DEGRADED.
        observable = eigenvalues > max(.05, eigenvalues[-1]*.001)
        vectors = basis[:, observable]
        step = -vectors @ ((vectors.T @ (jac.T @ (weight*residual))) / (eigenvalues[observable]+1.))
        step[2] /= 5.
        scale = max(1., np.linalg.norm(step[:2])/.2, abs(step[2])/.04)
        estimate += step/scale
        if np.linalg.norm(step[:2]) < .001 and abs(step[2]) < .0002:
            break
    world = transform(points, estimate)
    distance, _, _, _ = course.nearest(world)
    # Independent, stricter final support test against ALL valid scan returns.
    # Dynamic/out-of-map points are allowed only as an explicit minority.
    good = distance < .30
    count = int(good.sum()); fraction = count/max(1, len(points))
    mean = float(np.mean(distance[good])) if count else float('inf')
    p90 = float(np.quantile(distance[good], .9)) if count else float('inf')
    correction = float(np.linalg.norm(estimate[:2]-prior[:2]))
    turn = abs(wrap(estimate[2]-prior[2]))
    reason = ('INSUFFICIENT_SUPPORT' if count < 40 or fraction < .55 else
              'UNOBSERVABLE' if rank < 2 else
              'CORRECTION_LIMIT' if correction > (.9 if initializing else .45) or turn > (.18 if initializing else .10) else
              'MAP_RESIDUAL' if mean > .25 or p90 > .5 else
              'DEGRADED' if rank == 2 else 'TRACKING')
    estimate[:2] += course.origin; estimate[2] = wrap(estimate[2])
    return Match(reason in ('TRACKING', 'DEGRADED'), reason, estimate, rank, count, fraction,
                 mean, p90, correction, turn,
                 float(np.mean(distance)) if len(distance) else float('inf'),
                 float(np.quantile(distance,.9)) if len(distance) else float('inf'))


class MapLocalizer:
    """Track map->wheel_odom using only scans, wheel poses and explicit seeds.

    No identity/global-zero fallback. A failed match invalidates output; a gap
    above 1 second or backwards timestamps requires a new manual initialization.
    """
    def __init__(self, course: BoundaryMap) -> None:
        self.course = course
        self.reset()

    def reset(self) -> None:
        self.seed: np.ndarray | None = None
        self.map_to_odom: np.ndarray | None = None
        self.last_stamp_ns: int | None = None
        self.last_attempt_ns: int | None = None
        self.status = 'UNINITIALIZED'
        self.matches = 0

    def initialize(self, pose: np.ndarray) -> None:
        self.reset()
        self.seed = pose_array(pose)
        self.status = 'INITIALIZING'

    def update(self, stamp_ns: int, wheel_pose: np.ndarray, points: np.ndarray) -> Match | None:
        wheel_pose = pose_array(wheel_pose)
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError('LOCALIZATION_STAMP')
        if self.last_attempt_ns is not None and stamp_ns <= self.last_attempt_ns:
            self.reset(); self.status = 'CLOCK_RESET'
            return None
        self.last_attempt_ns = stamp_ns
        if self.seed is None and self.map_to_odom is None:
            return None
        if self.last_stamp_ns is not None and stamp_ns-self.last_stamp_ns > 1_000_000_000:
            self.reset(); self.status = 'LOST_REINITIALIZE'
            return None
        prior = self.seed if self.map_to_odom is None else compose(self.map_to_odom, wheel_pose)
        result = match_scan(self.course, points, prior, initializing=self.matches < 3)
        self.status = result.reason
        if result.accepted:
            self.map_to_odom = compose(result.pose, inverse(wheel_pose))
            self.seed = None
            self.last_stamp_ns = stamp_ns
            self.matches += 1
            if self.matches < 3:
                self.status = 'INITIALIZING'
        else:
            self.status = 'REJECTED_'+result.reason
        return result

    def valid(self, now_ns: int) -> bool:
        return (self.status in ('TRACKING', 'DEGRADED') and self.last_stamp_ns is not None
                and 0 <= now_ns-self.last_stamp_ns <= 500_000_000)
