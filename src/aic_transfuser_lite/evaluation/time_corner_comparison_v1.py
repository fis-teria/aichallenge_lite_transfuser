"""Spatial comparisons of recorded map poses; never modifies runtime targets."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from ..control.time_reference_v1 import TimedBodyPose
from ..control.time_trial_v1 import interpolate_body_pose
from .time_clearance_v1 import PoseIndex, project_to_polyline


def angle_delta(a: float, b: float) -> float:
    """Signed a-b in radians, wrapped at pi."""
    return math.atan2(math.sin(a-b), math.cos(a-b))


def world_points(xy_m: np.ndarray, pose: TimedBodyPose) -> np.ndarray:
    """Observation-base-link points [N,2] metres to the unchanged map frame."""
    xy = np.asarray(xy_m, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
        raise ValueError("POINTS_SHAPE_OR_NONFINITE")
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return xy @ np.array([[c, s], [-s, c]]) + [pose.x_m, pose.y_m]


def replay_observation_pose(sources: Sequence[tuple[TimedBodyPose, int]], *,
                            observation_ns: int, freeze_receipt_ns: int) -> TimedBodyPose:
    """Use only the teacher anchor's recorded source IDs, with receipt eligibility.

    Sources are (capture-stamped map/base_link pose, bag receipt nanoseconds).
    Unrelated messages with the same stamp must not replace the recorded IDs.
    """
    if len(sources) not in (1, 2) or type(observation_ns) is not int or type(freeze_receipt_ns) is not int:
        raise ValueError("ANCHOR_POSE_SOURCE_CONTRACT")
    if any(type(receipt) is not int or receipt > freeze_receipt_ns for _, receipt in sources):
        raise ValueError("ANCHOR_POSE_NOT_AVAILABLE_AT_FREEZE")
    poses = [p for p, _ in sources]
    if len({p.stamp_ns for p in poses}) != len(poses):
        raise ValueError("ANCHOR_POSE_DUPLICATE_SOURCE_STAMP")
    if any((p.world_frame, p.body_frame) != ("map", "base_link") for p in poses):
        raise ValueError("ANCHOR_POSE_FRAME")
    return interpolate_body_pose(poses, observation_ns)


@dataclass(frozen=True)
class Projection:
    distance_m: float
    left_m: float
    progress_m: float
    stamp_ns: int
    body_yaw_rad: float
    tangent_yaw_rad: float
    segment_index: int
    fraction: float
    xy_m: tuple[float, float]


class RecordedLine:
    """Ordered map/base_link poses, SI units; do not bridge gaps or ambiguity.

    Progress is recorded chord length, not an aligned clock or a road centerline.
    Conflicting duplicate capture stamps invalidate their adjacent segments.
    Exact duplicate messages are collapsed. Zero length segments are not targets.
    """

    def __init__(self, poses: Sequence[TimedBodyPose]) -> None:
        if len(poses) < 2:
            raise ValueError("TOO_FEW_POSES")
        identities = {(p.clock, p.epoch, p.world_frame, p.body_frame) for p in poses}
        if len(identities) != 1:
            raise ValueError("POSE_IDENTITY_MISMATCH")
        if poses[0].world_frame != "map" or poses[0].body_frame != "base_link":
            raise ValueError("MAP_BASE_LINK_REQUIRED")
        unique: dict[int, TimedBodyPose] = {}
        conflicts: set[int] = set()
        exact_duplicates = 0
        for p in poses:
            if p.stamp_ns in unique:
                if unique[p.stamp_ns] != p:
                    conflicts.add(p.stamp_ns)
                else:
                    exact_duplicates += 1
            unique[p.stamp_ns] = p
        self.rows = sorted(unique.values(), key=lambda p: p.stamp_ns)
        self.index = PoseIndex(list(poses))
        self.xy = np.array([[p.x_m, p.y_m] for p in self.rows])
        self.delta = np.diff(self.xy, axis=0)
        self.length = np.linalg.norm(self.delta, axis=1)
        self.arc = np.r_[0., np.cumsum(self.length)]
        self.yaw = np.array([p.yaw_rad for p in self.rows])
        self.times = np.array([p.stamp_ns for p in self.rows], dtype=np.int64)
        dt = np.diff(self.times)
        conflict = np.array([a.stamp_ns in conflicts or b.stamp_ns in conflicts
                             for a, b in zip(self.rows, self.rows[1:])])
        self.usable = (self.length > 1e-6) & (dt > 0) & (dt <= 50_000_000) & ~conflict
        self.usable &= self.length <= 10.*dt/1e9
        self.audit = {"input_poses": len(poses), "unique_stamps": len(self.rows),
                      "exact_duplicates": exact_duplicates, "conflicting_stamps": len(conflicts),
                      "excluded_segments": int((~self.usable).sum())}
        if not self.usable.any():
            raise ValueError("NO_VALID_MOVING_SEGMENTS")

    def project(self, point_xy_m: np.ndarray, *, yaw_hint_rad: float | None = None,
                progress_bounds_m: tuple[float, float] | None = None,
                max_distance_m: float = 3.) -> Projection:
        """Nearest segment with optional same-direction and progress constraints.

        Distance is Euclidean; left_m is perpendicular signed displacement, so
        longitudinal endpoint residuals cannot masquerade as a lateral error.
        Endpoint-clipped points outside the recorded progress interval reject.
        """
        point = np.asarray(point_xy_m, float)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError("POINT_SHAPE_OR_NONFINITE")
        if not math.isfinite(max_distance_m) or max_distance_m <= 0:
            raise ValueError("INVALID_DISTANCE_BOUND")
        mask = self.usable.copy()
        fraction_raw = np.sum((point-self.xy[:-1])*self.delta, axis=1)/np.maximum(self.length**2, 1e-20)
        fraction = np.clip(fraction_raw, 0., 1.)
        progress = self.arc[:-1]+fraction*self.length
        lower, upper = 0., float(self.arc[-1])
        if progress_bounds_m is not None:
            lower, upper = progress_bounds_m
            if not np.isfinite([lower, upper]).all() or lower >= upper:
                raise ValueError("INVALID_PROGRESS_BOUNDS")
            mask &= (progress >= lower) & (progress <= upper)
        if yaw_hint_rad is not None:
            if not math.isfinite(yaw_hint_rad):
                raise ValueError("INVALID_HEADING_HINT")
            yaw = self.yaw[:-1] + fraction*np.arctan2(np.sin(np.diff(self.yaw)), np.cos(np.diff(self.yaw)))
            mask &= np.abs(np.arctan2(np.sin(yaw-yaw_hint_rad), np.cos(yaw-yaw_hint_rad))) <= math.pi/3
        nearest = self.xy[:-1]+fraction[:, None]*self.delta
        distances = np.linalg.norm(point-nearest, axis=1)
        if not mask.any():
            raise ValueError("NO_MATCHING_SEGMENT")
        i = int(np.argmin(np.where(mask, distances, np.inf)))
        if distances[i] > max_distance_m:
            raise ValueError("REFERENCE_TOO_FAR")
        if ((progress[i] <= lower+1e-8 and fraction_raw[i] < 0.)
                or (progress[i] >= upper-1e-8 and fraction_raw[i] > 1.)):
            raise ValueError("REFERENCE_ENDPOINT")
        residual = point-nearest[i]
        left = (self.delta[i, 0]*residual[1]-self.delta[i, 1]*residual[0])/self.length[i]
        yaw = self.yaw[i]+float(fraction[i])*angle_delta(self.yaw[i+1], self.yaw[i])
        return Projection(float(distances[i]), float(left), float(progress[i]),
                          int(self.times[i])+round(float(fraction[i])*int(self.times[i+1]-self.times[i])),
                          angle_delta(yaw, 0.), math.atan2(*self.delta[i, ::-1]), i, float(fraction[i]),
                          (float(nearest[i, 0]), float(nearest[i, 1])))


def future_tracking_error(raw_xy_m: np.ndarray, observed: TimedBodyPose, poses: PoseIndex,
                          horizon_s: float, *, before_ns: int) -> dict[str, float | bool]:
    """Observed future pose against the original prediction at 1/2/3 s.

    Temporal point error and geometric polyline distance are separate. Later
    controller replanning means this is a following diagnostic, not causality.
    """
    raw = np.asarray(raw_xy_m, float)
    if raw.shape != (30, 2) or not np.isfinite(raw).all() or horizon_s not in (1., 2., 3.):
        raise ValueError("FUTURE_SHAPE_OR_HORIZON")
    future_ns = observed.stamp_ns+int(horizon_s*1e9)
    if future_ns > before_ns:
        raise ValueError("FUTURE_AFTER_FIRST_FAULT")
    future = poses.at(future_ns)
    if (future.clock, future.epoch, future.world_frame, future.body_frame) != (
            observed.clock, observed.epoch, observed.world_frame, observed.body_frame):
        raise ValueError("POSE_IDENTITY_MISMATCH")
    world = world_points(np.vstack([np.zeros((1, 2)), raw]), observed)
    j = int(horizon_s*10)
    delta = world[j]-world[j-1]
    if np.linalg.norm(delta) <= 1e-6:
        raise ValueError("PREDICTION_TANGENT_UNDEFINED")
    tangent = delta/np.linalg.norm(delta)
    error = np.array([future.x_m, future.y_m])-world[j]
    projection = project_to_polyline(np.array([future.x_m, future.y_m]), world)
    return {"point_distance_m": float(np.linalg.norm(error)),
            "point_left_m": float(tangent[0]*error[1]-tangent[1]*error[0]),
            "point_along_m": float(error@tangent),
            "polyline_distance_m": projection["distance_m"],
            "polyline_endpoint": bool((projection["segment_index"] == 0 and projection["fraction"] == 0.)
                                      or (projection["segment_index"] == 29 and projection["fraction"] == 1.))}
