"""Planar, metric LiDAR perception. No ROS imports or simulator object inputs.

Coordinates are XY metres, angles radians and stamps seconds. Object extents
describe observed surfaces, not an assertion about the unobserved object body.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Pose2:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("Pose must be finite, in metres/radians")

    def apply(self, xy: np.ndarray) -> np.ndarray:
        """Transform an (N, 2) array from this pose's local frame to map."""
        xy = points_array(xy)
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return xy @ np.array([[c, s], [-s, c]]) + [self.x_m, self.y_m]

    def inverse_apply(self, xy: np.ndarray) -> np.ndarray:
        xy = points_array(xy) - [self.x_m, self.y_m]
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return xy @ np.array([[c, -s], [s, c]])

    def compose(self, child: Pose2) -> Pose2:
        xy = self.apply(np.array([[child.x_m, child.y_m]]))[0]
        return Pose2(float(xy[0]), float(xy[1]), self.yaw_rad + child.yaw_rad)


def points_array(xy: np.ndarray) -> np.ndarray:
    a = np.asarray(xy, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 2 or not np.isfinite(a).all():
        raise ValueError("Expected finite points with shape (N, 2), in metres")
    return a


def interpolate_pose(a: Pose2, b: Pose2, fraction: float) -> Pose2:
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("Pose interpolation fraction must be in [0, 1]")
    dyaw = math.remainder(b.yaw_rad - a.yaw_rad, 2.0 * math.pi)
    return Pose2(a.x_m + fraction * (b.x_m - a.x_m),
                 a.y_m + fraction * (b.y_m - a.y_m), a.yaw_rad + fraction * dyaw)


@dataclass(frozen=True)
class Scan:
    stamp_s: float
    ranges_m: np.ndarray
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    time_increment_s: float = 0.0

    def __post_init__(self) -> None:
        a = np.asarray(self.ranges_m, dtype=np.float64)
        if a.ndim != 1 or len(a) < 2:
            raise ValueError("LaserScan ranges must have shape (N,), N >= 2")
        scalars = [self.stamp_s, self.angle_min_rad, self.angle_increment_rad,
                   self.range_min_m, self.range_max_m, self.time_increment_s]
        if not all(math.isfinite(v) for v in scalars):
            raise ValueError("LaserScan metadata must be finite")
        if (self.stamp_s < 0 or self.range_min_m < 0 or
                self.range_max_m <= self.range_min_m or self.time_increment_s < 0 or
                self.angle_increment_rad == 0):
            raise ValueError("Invalid LaserScan range, angle increment or timestamp")
        object.__setattr__(self, "ranges_m", a)

    @property
    def duration_s(self) -> float:
        return (len(self.ranges_m) - 1) * self.time_increment_s


@dataclass(frozen=True)
class Config:
    max_range_m: float = 20.0
    wall_margin_m: float = 0.25
    cluster_gap_m: float = 0.25
    min_cluster_points: int = 4
    max_cluster_extent_m: float = 3.0
    association_gate_m: float = 0.65
    track_ttl_s: float = 0.5
    confirmation_scans: int = 3
    measurement_std_m: float = 0.15
    max_track_speed_mps: float = 8.0
    ego_front_m: float = 1.8
    ego_rear_m: float = 0.6
    ego_half_width_m: float = 0.7
    motion_model: str = "rolling"
    object_model: str = "surface"
    vehicle_length_m: float = 2.064
    vehicle_width_m: float = 1.30
    max_box_fit_rmse_m: float = 0.15

    def __post_init__(self) -> None:
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be positive and finite")
        for key in ("min_cluster_points", "confirmation_scans"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key} must be an integer")
        if self.motion_model not in {"rolling", "snapshot"}:
            raise ValueError("motion_model must be rolling or snapshot")
        if self.object_model not in {"surface", "known_vehicle"}:
            raise ValueError("object_model must be surface or known_vehicle")


class StaticMap:
    """Known-free mask with ROS row order (row zero is at origin's local y=0).

    Occupied, unknown and out-of-map returns are not new-object evidence.
    The mask is expanded by wall_margin_m, explicitly sacrificing detection
    close to mapped walls. This filter is not an obstacle-free-space proof.
    """

    def __init__(self, free: np.ndarray, resolution_m: float, origin: Pose2,
                 wall_margin_m: float) -> None:
        free = np.asarray(free, dtype=bool)
        if free.ndim != 2 or min(free.shape) == 0:
            raise ValueError("Map must be a nonempty (height, width) free-space mask")
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("Map resolution must be positive, in metres")
        if not math.isfinite(wall_margin_m) or wall_margin_m < 0:
            raise ValueError("Map wall margin must be nonnegative, in metres")
        self.resolution_m, self.origin = resolution_m, origin
        radius = math.ceil(wall_margin_m / resolution_m)
        # Padding treats the map boundary as unknown, never as free space.
        padded = np.pad(free, radius, constant_values=False)
        clean = free.copy()
        h, w = free.shape
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) <= radius:
                    clean &= padded[radius + dy:radius + dy + h, radius + dx:radius + dx + w]
        self.free_interior = clean

    def is_interior(self, xy_map: np.ndarray) -> np.ndarray:
        xy = self.origin.inverse_apply(xy_map)
        cells = np.floor(xy / self.resolution_m).astype(np.int64)
        h, w = self.free_interior.shape
        valid = (cells[:, 0] >= 0) & (cells[:, 0] < w) & (cells[:, 1] >= 0) & (cells[:, 1] < h)
        result = np.zeros(len(cells), dtype=bool)
        result[valid] = self.free_interior[cells[valid, 1], cells[valid, 0]]
        return result


def scan_points(scan: Scan, start: Pose2, end: Pose2, config: Config) -> tuple[np.ndarray, np.ndarray]:
    """Return (points_map, beam_origins_map), each (N, 2), at measured beam times."""
    r = scan.ranges_m
    keep = np.isfinite(r) & (r > max(scan.range_min_m, 0.0)) & (r < scan.range_max_m) & (r <= config.max_range_m)
    indices = np.flatnonzero(keep)
    angles = scan.angle_min_rad + indices * scan.angle_increment_rad
    local = np.column_stack([r[keep] * np.cos(angles), r[keep] * np.sin(angles)])
    if config.motion_model == "snapshot" or scan.duration_s == 0.0:
        return start.apply(local), np.tile([start.x_m, start.y_m], (len(local), 1))
    fraction = indices / (len(r) - 1)
    yaw = start.yaw_rad + fraction * math.remainder(end.yaw_rad - start.yaw_rad, 2 * math.pi)
    origins = np.column_stack([start.x_m + fraction * (end.x_m - start.x_m),
                               start.y_m + fraction * (end.y_m - start.y_m)])
    xy = np.column_stack([np.cos(yaw) * local[:, 0] - np.sin(yaw) * local[:, 1],
                           np.sin(yaw) * local[:, 0] + np.cos(yaw) * local[:, 1]]) + origins
    return xy, origins


def clusters(xy: np.ndarray, gap_m: float) -> list[np.ndarray]:
    """Euclidean connected components, returned as point-index arrays."""
    xy = points_array(xy)
    if not math.isfinite(gap_m) or gap_m <= 0:
        raise ValueError("Cluster gap must be positive, in metres")
    cells = np.floor(xy / gap_m).astype(np.int64)
    buckets: dict[tuple[int, int], list[int]] = {}
    for i, cell in enumerate(cells):
        buckets.setdefault(tuple(cell), []).append(i)
    unused = np.ones(len(xy), dtype=bool)
    result = []
    for seed in range(len(xy)):
        if not unused[seed]:
            continue
        unused[seed] = False
        component, queue = [], [seed]
        while queue:
            i = queue.pop()
            component.append(i)
            cx, cy = cells[i]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in buckets.get((cx + dx, cy + dy), ()):
                        if unused[j] and np.sum((xy[i] - xy[j]) ** 2) <= gap_m ** 2:
                            unused[j] = False
                            queue.append(j)
        result.append(np.array(component, dtype=np.int64))
    return result


class Reference:
    """Teacher-only geometry used as a heading prior for known course-aligned NPCs."""

    def __init__(self, xy: np.ndarray) -> None:
        xy = points_array(xy)
        if len(xy) < 2:
            raise ValueError("Reference must contain at least two points")
        delta = np.diff(xy, axis=0)
        good = np.linalg.norm(delta, axis=1) > 1e-6
        if not np.any(good):
            raise ValueError("Reference has no nonzero segments")
        self.starts, self.delta = xy[:-1][good], delta[good]

    def yaw_at(self, xy: np.ndarray) -> float:
        t = np.clip(np.sum((xy - self.starts) * self.delta, axis=1) / np.sum(self.delta ** 2, axis=1), 0, 1)
        index = int(np.argmin(np.sum((self.starts + t[:, None] * self.delta - xy) ** 2, axis=1)))
        return math.atan2(self.delta[index, 1], self.delta[index, 0])


def fit_vehicle_box(xy: np.ndarray, origins: np.ndarray, yaw: float,
                    length_m: float, width_m: float) -> tuple[np.ndarray, float, float]:
    """Fit a known rectangle's first ray intersections, NOT the surface centroid.

    Returns centre_map (2,), range RMSE [m], centre ambiguity radius [m].
    Size and heading are explicit teacher priors; this is not classification.
    """
    xy, origins = points_array(xy), points_array(origins)
    if len(xy) < 2 or xy.shape != origins.shape:
        raise ValueError("Box fit needs matched points/origins with shape (N, 2)")
    if not all(math.isfinite(v) and v > 0 for v in (length_m, width_m)):
        raise ValueError("Vehicle dimensions must be positive metres")
    rotation = Pose2(float(np.mean(xy[:, 0])), float(np.mean(xy[:, 1])), yaw)
    p, o = rotation.inverse_apply(xy), rotation.inverse_apply(origins)
    ranges = np.linalg.norm(p - o, axis=1)
    if np.any(ranges < 1e-6):
        raise ValueError("Ray must have positive length")
    directions = (p - o) / ranges[:, None]
    half = np.array([length_m, width_m]) / 2
    lower, upper = p.max(axis=0) - half - 0.1, p.min(axis=0) + half + 0.1
    if np.any(lower > upper):
        return np.mean(xy, axis=0), math.inf, math.inf

    def evaluate(centres: np.ndarray) -> np.ndarray:
        relative = o[None, :, :] - centres[:, None, :]
        parallel = np.abs(directions) < 1e-10
        safe = np.where(parallel, 1.0, directions)
        a = (-half - relative) / safe
        b = (half - relative) / safe
        near, far = np.minimum(a, b), np.maximum(a, b)
        near = np.where(parallel, -np.inf, near)
        far = np.where(parallel, np.inf, far)
        hit_near, hit_far = near.max(axis=2), far.min(axis=2)
        invalid = (hit_near > hit_far) | (hit_near < 0) | np.any(parallel & (np.abs(relative) > half), axis=2)
        residual = np.where(invalid, 5.0, np.abs(hit_near - ranges[None, :]))
        return np.sqrt(np.mean(np.minimum(residual, 5.0) ** 2, axis=1))

    grid = np.stack(np.meshgrid(np.linspace(lower[0], upper[0], 17),
                                np.linspace(lower[1], upper[1], 17)), axis=-1).reshape(-1, 2)
    scores = evaluate(grid)
    best = grid[int(np.argmin(scores))]
    step = np.maximum((upper - lower) / 16, 0.01)
    fine = np.stack(np.meshgrid(np.linspace(best[0] - step[0], best[0] + step[0], 9),
                                np.linspace(best[1] - step[1], best[1] + step[1], 9)), axis=-1).reshape(-1, 2)
    fine_scores = evaluate(fine)
    best_index = int(np.argmin(fine_scores))
    centre = fine[best_index]
    near_best = grid[scores <= float(fine_scores[best_index]) + 0.03]
    ambiguity = float(np.linalg.norm(near_best - centre, axis=1).max()) if len(near_best) else 0.0
    return rotation.apply(centre[None, :])[0], float(fine_scores[best_index]), ambiguity


@dataclass(frozen=True)
class Detection:
    xy_m: tuple[float, float]
    surface_xy_m: tuple[float, float]
    observed_size_xy_m: tuple[float, float]
    point_count: int
    representation: str
    std_m: float
    fit_rmse_m: float | None = None
    ambiguity_m: float | None = None


@dataclass
class Track:
    track_id: str
    stamp_s: float
    detection: Detection
    velocity_xy_mps: tuple[float, float] = (0.0, 0.0)
    consecutive_hits: int = 1


class Tracker:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.tracks: dict[str, Track] = {}
        self.sequence = 0
        self.last_stamp_s: float | None = None

    def reset(self) -> None:
        self.tracks.clear()
        self.last_stamp_s = None
        # Never reuse IDs across a reset within this process.

    def update(self, detections: Iterable[Detection], stamp_s: float) -> list[Track]:
        if not math.isfinite(stamp_s) or stamp_s < 0:
            raise ValueError("Track stamp must be finite nonnegative seconds")
        if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s:
            raise ValueError("Non-increasing scan time: reset tracker for a new episode")
        self.last_stamp_s = stamp_s
        self.tracks = {k: t for k, t in self.tracks.items() if stamp_s - t.stamp_s <= self.config.track_ttl_s}
        detections = list(detections)
        pairs = []
        for key, track in self.tracks.items():
            predicted = np.array(track.detection.xy_m) + np.array(track.velocity_xy_mps) * (stamp_s - track.stamp_s)
            for i, detection in enumerate(detections):
                distance = float(np.linalg.norm(np.array(detection.xy_m) - predicted))
                if distance <= self.config.association_gate_m and detection.representation == track.detection.representation:
                    pairs.append((distance, key, i))
        used_tracks, used_detections = set(), set()
        for _, key, i in sorted(pairs):
            if key in used_tracks or i in used_detections:
                continue
            t, d = self.tracks[key], detections[i]
            dt = stamp_s - t.stamp_s
            velocity = (np.array(d.xy_m) - t.detection.xy_m) / dt
            if np.linalg.norm(velocity) > self.config.max_track_speed_mps:
                continue
            alpha = dt / (0.3 + dt)
            t.velocity_xy_mps = tuple((1 - alpha) * np.array(t.velocity_xy_mps) + alpha * velocity)
            t.stamp_s, t.detection = stamp_s, d
            t.consecutive_hits += 1
            used_tracks.add(key)
            used_detections.add(i)
        for key, t in self.tracks.items():
            if key not in used_tracks:
                t.consecutive_hits = 0
        for i, detection in enumerate(detections):
            if i not in used_detections:
                self.sequence += 1
                key = f"lidar_{self.sequence:06d}"
                self.tracks[key] = Track(key, stamp_s, detection)
        # Do not fabricate fresh stamps or return extrapolated lost objects.
        return [t for t in self.tracks.values() if t.stamp_s == stamp_s and t.consecutive_hits >= self.config.confirmation_scans]


class Detector:
    def __init__(self, config: Config, wall_map: StaticMap, reference: Reference | None = None) -> None:
        if config.object_model == "known_vehicle" and reference is None:
            raise ValueError("known_vehicle requires an explicit teacher reference heading prior")
        self.config, self.wall_map, self.reference = config, wall_map, reference

    def detect(self, scan: Scan, lidar_start: Pose2, lidar_end: Pose2,
               base_start: Pose2) -> tuple[list[Detection], dict[str, int]]:
        malformed = (np.isnan(scan.ranges_m) | np.isneginf(scan.ranges_m) |
                     (np.isfinite(scan.ranges_m) & ((scan.ranges_m <= scan.range_min_m) |
                                                    (scan.ranges_m > scan.range_max_m))))
        if np.mean(malformed) > 0.2:
            raise ValueError("More than 20% malformed LiDAR returns; cannot report empty road")
        xy, origins = scan_points(scan, lidar_start, lidar_end, self.config)
        local = base_start.inverse_apply(xy)
        cfg = self.config
        ego = ((local[:, 0] >= -cfg.ego_rear_m) & (local[:, 0] <= cfg.ego_front_m) &
               (np.abs(local[:, 1]) <= cfg.ego_half_width_m))
        interior = self.wall_map.is_interior(xy)
        keep = ~ego & interior
        stats = dict(valid_points=len(xy), ego_points=int(ego.sum()),
                     malformed_points=int(malformed.sum()),
                     background_points=int((~interior).sum()), residual_points=int(keep.sum()),
                     small_clusters=0, oversized_clusters=0, rejected_box_fits=0)
        xy, origins = xy[keep], origins[keep]
        detections = []
        for indices in clusters(xy, cfg.cluster_gap_m):
            points = xy[indices]
            if len(points) < cfg.min_cluster_points:
                stats["small_clusters"] += 1
                continue
            size = np.ptp(points, axis=0)
            if float(np.linalg.norm(size)) > cfg.max_cluster_extent_m:
                stats["oversized_clusters"] += 1
                continue
            surface = (points.min(axis=0) + points.max(axis=0)) / 2
            centre, error, ambiguity = surface, None, None
            if cfg.object_model == "known_vehicle":
                assert self.reference is not None
                centre, error, ambiguity = fit_vehicle_box(points, origins[indices],
                    self.reference.yaw_at(surface), cfg.vehicle_length_m, cfg.vehicle_width_m)
                if error > cfg.max_box_fit_rmse_m:
                    stats["rejected_box_fits"] += 1
                    continue
            detections.append(Detection(tuple(centre), tuple(surface), tuple(size), len(points),
                                        cfg.object_model, cfg.measurement_std_m, error, ambiguity))
        return detections, stats


def v2x_payload(tracks: Iterable[Track], stamp_s: float, frame_id: str = "map") -> dict:
    """V44 schema: covariance.{x,y,z} are STANDARD DEVIATIONS [m], not m^2.

    No native V2X input or merging is performed. This is a dedicated shadow
    stream. Vehicle body dimensions cannot be encoded in this message.
    """
    if not frame_id or not math.isfinite(stamp_s) or stamp_s < 0:
        raise ValueError("Valid map frame and nonnegative stamp required")
    vehicles = []
    for t in tracks:
        d = t.detection
        values = (*d.xy_m, d.std_m, t.stamp_s)
        if not all(math.isfinite(v) for v in values) or d.std_m <= 0 or t.stamp_s > stamp_s or not t.track_id:
            raise ValueError("Invalid track at V2X serialization boundary")
        vehicles.append(dict(vehicle_id=t.track_id, stamp_s=t.stamp_s, frame_id=frame_id,
                             position=dict(x=float(d.xy_m[0]), y=float(d.xy_m[1]), z=0.0),
                             covariance=dict(x=d.std_m, y=d.std_m, z=d.std_m)))
    return dict(stamp_s=stamp_s, frame_id=frame_id, vehicles=vehicles)
