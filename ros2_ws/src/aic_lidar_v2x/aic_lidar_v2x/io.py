"""File/geometry adapters shared by offline replay and the ROS node."""
from __future__ import annotations

import math
from bisect import bisect_left
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from .core import Pose2, Reference, StaticMap, interpolate_pose


def load_map(path: str | Path, wall_margin_m: float) -> StaticMap:
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data.get("mode", "trinary") != "trinary":
        raise ValueError("Only ROS trinary maps are supported")
    free_threshold = float(data["free_thresh"])
    occupied_threshold = float(data["occupied_thresh"])
    if not 0 <= free_threshold < occupied_threshold <= 1:
        raise ValueError("Invalid map thresholds")
    if int(data["negate"]) not in (0, 1):
        raise ValueError("Map negate must be 0 or 1")
    with Image.open(path.parent / data["image"]) as image:
        gray = np.asarray(image.convert("L"), dtype=np.float64) / 255.0
        alpha = np.asarray(image.convert("RGBA"))[:, :, 3] == 255
    occupancy = gray if data["negate"] else 1.0 - gray
    # Image top-down order differs from nav_msgs/OccupancyGrid bottom-up order.
    free = np.flipud((occupancy < free_threshold) & alpha)
    return StaticMap(free, float(data["resolution"]), Pose2(*map(float, data["origin"])), wall_margin_m)


def load_reference(path: str | Path) -> Reference:
    rows = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    if not {"x_m", "y_m"}.issubset(rows.dtype.names or ()):
        raise ValueError("Reference CSV requires x_m,y_m columns")
    # The heading is derived from XY geometry; unrelated CSV heading conventions
    # must not silently rotate a physical box.
    return Reference(np.column_stack([rows["x_m"], rows["y_m"]]))


def planar_pose(x: float, y: float, quaternion: tuple[float, float, float, float]) -> Pose2:
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
        raise ValueError("TF quaternion must be finite and normalized")
    roll = math.atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw*qy - qz*qx))))
    if max(abs(roll), abs(pitch)) > math.radians(5):
        raise ValueError("Planar detector does not support TF roll/pitch above 5 degrees")
    return Pose2(x, y, math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz)))


class PoseHistory:
    """Replay's timestamped TF interpolation; samples must have already arrived."""

    def __init__(self, max_gap_s: float = 0.1) -> None:
        self.stamps: list[float] = []
        self.poses: list[Pose2] = []
        self.max_gap_s = max_gap_s

    def add(self, stamp_s: float, pose: Pose2) -> None:
        if not math.isfinite(stamp_s) or stamp_s < 0:
            raise ValueError("Invalid pose timestamp")
        i = bisect_left(self.stamps, stamp_s)
        if i < len(self.stamps) and self.stamps[i] == stamp_s:
            self.poses[i] = pose
        else:
            self.stamps.insert(i, stamp_s)
            self.poses.insert(i, pose)
        if len(self.stamps) > 1024:
            self.stamps.pop(0)
            self.poses.pop(0)

    def at(self, stamp_s: float) -> Pose2:
        i = bisect_left(self.stamps, stamp_s)
        if i < len(self.stamps) and abs(self.stamps[i] - stamp_s) < 1e-9:
            return self.poses[i]
        if i == 0 or i == len(self.stamps):
            raise LookupError("Measurement is outside the received TF history")
        left, right = self.stamps[i-1], self.stamps[i]
        if right - left > self.max_gap_s:
            raise LookupError("TF gap exceeds interpolation limit")
        return interpolate_pose(self.poses[i-1], self.poses[i], (stamp_s - left) / (right - left))
