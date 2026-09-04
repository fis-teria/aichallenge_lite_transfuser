from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


COLLECTION_REFERENCE_FORMAT = "aic_collection_reference_v1"
COLLECTION_CRITERIA_FORMAT = "aic_collection_coverage_criteria_v1"


def wrap_angle_rad(value: float) -> float:
    """Wrap one angle in radians to ``[-pi, pi)``."""

    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class RoutePointV3:
    point_id: int
    x_m: float
    y_m: float
    heading_rad: float
    frame_id: str = "map"

    def validate(self) -> None:
        values = (self.x_m, self.y_m, self.heading_rad)
        if self.point_id < 0 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"invalid route point: {self}")
        if not self.frame_id:
            raise ValueError("route point frame_id must be non-empty")


@dataclass(frozen=True)
class RouteProjectionV3:
    point_id: int
    lateral_offset_m: float
    heading_error_rad: float
    curvature_inv_m: float


@dataclass(frozen=True)
class TeacherStateSampleV3:
    run_id: str
    scenario_id: str
    timestamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    yaw_rate_rps: float

    def validate(self) -> None:
        if not self.run_id or not self.scenario_id or self.timestamp_ns < 0:
            raise ValueError(f"invalid teacher-state identity: {self}")
        values = (self.x_m, self.y_m, self.yaw_rad, self.speed_mps, self.yaw_rate_rps)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"non-finite teacher state: {self}")


@dataclass(frozen=True)
class CoverageRequirementV3:
    min_samples: int
    min_runs: int
    min_episodes: int

    def validate(self, name: str) -> None:
        if min(self.min_samples, self.min_runs, self.min_episodes) < 0:
            raise ValueError(f"coverage requirement {name!r} must be non-negative")


@dataclass(frozen=True)
class CollectionCriteriaV3:
    sample_rate_hz: float
    stopped_speed_mps: float
    moving_speed_mps: float
    curve_curvature_inv_m: float
    lateral_offset_min_m: float
    lateral_offset_far_m: float
    heading_error_min_rad: float
    launch_horizon_sec: float
    recovery_horizon_sec: float
    recovery_improvement_m: float
    episode_gap_sec: float
    requirements: Mapping[str, CoverageRequirementV3]

    def validate(self) -> None:
        positive = {
            "sample_rate_hz": self.sample_rate_hz,
            "moving_speed_mps": self.moving_speed_mps,
            "curve_curvature_inv_m": self.curve_curvature_inv_m,
            "lateral_offset_min_m": self.lateral_offset_min_m,
            "lateral_offset_far_m": self.lateral_offset_far_m,
            "heading_error_min_rad": self.heading_error_min_rad,
            "launch_horizon_sec": self.launch_horizon_sec,
            "recovery_horizon_sec": self.recovery_horizon_sec,
            "recovery_improvement_m": self.recovery_improvement_m,
            "episode_gap_sec": self.episode_gap_sec,
        }
        if any(value <= 0.0 or not math.isfinite(value) for value in positive.values()):
            raise ValueError(f"coverage thresholds must be finite and positive: {positive}")
        if self.stopped_speed_mps < 0.0:
            raise ValueError("stopped_speed_mps must be non-negative")
        if self.stopped_speed_mps >= self.moving_speed_mps:
            raise ValueError("stopped_speed_mps must be below moving_speed_mps")
        if self.lateral_offset_far_m <= self.lateral_offset_min_m:
            raise ValueError("lateral_offset_far_m must exceed lateral_offset_min_m")
        if not self.requirements:
            raise ValueError("coverage requirements must be non-empty")
        for name, requirement in self.requirements.items():
            requirement.validate(name)


def load_collection_criteria_v3(path: str | Path) -> CollectionCriteriaV3:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("format_version") != COLLECTION_CRITERIA_FORMAT:
        raise ValueError(f"unsupported collection criteria format: {raw!r}")
    thresholds = raw.get("thresholds")
    requirements_raw = raw.get("requirements")
    if not isinstance(thresholds, dict) or not isinstance(requirements_raw, dict):
        raise ValueError("criteria thresholds and requirements must be mappings")
    requirements: dict[str, CoverageRequirementV3] = {}
    for name, value in requirements_raw.items():
        if not isinstance(value, dict):
            raise ValueError(f"coverage requirement {name!r} must be a mapping")
        requirements[str(name)] = CoverageRequirementV3(
            min_samples=int(value["min_samples"]),
            min_runs=int(value["min_runs"]),
            min_episodes=int(value["min_episodes"]),
        )
    criteria = CollectionCriteriaV3(
        sample_rate_hz=float(raw["sample_rate_hz"]),
        stopped_speed_mps=float(thresholds["stopped_speed_mps"]),
        moving_speed_mps=float(thresholds["moving_speed_mps"]),
        curve_curvature_inv_m=float(thresholds["curve_curvature_inv_m"]),
        lateral_offset_min_m=float(thresholds["lateral_offset_min_m"]),
        lateral_offset_far_m=float(thresholds["lateral_offset_far_m"]),
        heading_error_min_rad=float(thresholds["heading_error_min_rad"]),
        launch_horizon_sec=float(thresholds["launch_horizon_sec"]),
        recovery_horizon_sec=float(thresholds["recovery_horizon_sec"]),
        recovery_improvement_m=float(thresholds["recovery_improvement_m"]),
        episode_gap_sec=float(thresholds["episode_gap_sec"]),
        requirements=requirements,
    )
    criteria.validate()
    return criteria


def route_points_from_arrows(
    arrows: Iterable[tuple[int, str, float, float, float, float]],
) -> tuple[RoutePointV3, ...]:
    """Convert Marker Arrow endpoints into ordered, map-frame route points.

    Each input is ``(marker_id, frame_id, tail_x_m, tail_y_m, head_x_m,
    head_y_m)``. The arrow direction is the teacher-only reference heading.
    """

    result: list[RoutePointV3] = []
    ids: set[int] = set()
    frames: set[str] = set()
    for marker_id, frame_id, x0, y0, x1, y1 in arrows:
        if marker_id in ids:
            raise ValueError(f"duplicate reference marker id: {marker_id}")
        dx = float(x1) - float(x0)
        dy = float(y1) - float(y0)
        if not all(math.isfinite(value) for value in (x0, y0, x1, y1)):
            raise ValueError(f"non-finite reference marker: {marker_id}")
        if math.hypot(dx, dy) <= 1e-6:
            raise ValueError(f"zero-length reference arrow: {marker_id}")
        point = RoutePointV3(
            point_id=int(marker_id),
            x_m=float(x0),
            y_m=float(y0),
            heading_rad=math.atan2(dy, dx),
            frame_id=str(frame_id),
        )
        point.validate()
        result.append(point)
        ids.add(marker_id)
        frames.add(point.frame_id)
    if len(result) < 3:
        raise ValueError("route reference requires at least three arrows")
    if len(frames) != 1:
        raise ValueError(f"route reference mixes frames: {sorted(frames)}")
    return tuple(sorted(result, key=lambda point: point.point_id))


def write_route_reference_v3(
    output_csv: str | Path,
    points: Sequence[RoutePointV3],
    *,
    source_topic: str,
    source_type: str,
    captured_utc: str,
    source_artifact: str | Path | None = None,
) -> Path:
    output = Path(output_csv)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite route reference: {output}")
    if not source_topic.startswith("/") or not source_type or not captured_utc:
        raise ValueError("source topic, type and captured UTC are required")
    ordered = tuple(points)
    route_points_from_arrows(
        (
            point.point_id,
            point.frame_id,
            point.x_m,
            point.y_m,
            point.x_m + math.cos(point.heading_rad),
            point.y_m + math.sin(point.heading_rad),
        )
        for point in ordered
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=("point_id", "frame_id", "x_m", "y_m", "heading_rad")
        )
        writer.writeheader()
        for point in ordered:
            writer.writerow(
                {
                    "point_id": point.point_id,
                    "frame_id": point.frame_id,
                    "x_m": f"{point.x_m:.9f}",
                    "y_m": f"{point.y_m:.9f}",
                    "heading_rad": f"{wrap_angle_rad(point.heading_rad):.9f}",
                }
            )
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    manifest = output.with_suffix(".manifest.yaml")
    manifest_payload: dict[str, Any] = {
        "format_version": COLLECTION_REFERENCE_FORMAT,
        "teacher_debug_only": True,
        "coordinate_frame": ordered[0].frame_id,
        "point_count": len(ordered),
        "source_topic": source_topic,
        "source_type": source_type,
        "captured_utc": captured_utc,
        "reference_csv": output.name,
        "reference_sha256": digest,
    }
    if source_artifact is not None:
        source_path = Path(source_artifact)
        if not source_path.is_file():
            raise FileNotFoundError(f"Reference source artifact not found: {source_path}")
        manifest_payload["source_artifact"] = str(source_path)
        manifest_payload["source_artifact_sha256"] = hashlib.sha256(
            source_path.read_bytes()
        ).hexdigest()
    manifest.write_text(
        yaml.safe_dump(manifest_payload, sort_keys=False),
        encoding="utf-8",
    )
    return manifest


def load_route_reference_v3(path: str | Path) -> tuple[RoutePointV3, ...]:
    source = Path(path)
    with source.open("r", encoding="utf-8", newline="") as handle:
        points = tuple(
            RoutePointV3(
                point_id=int(row["point_id"]),
                frame_id=row["frame_id"],
                x_m=float(row["x_m"]),
                y_m=float(row["y_m"]),
                heading_rad=float(row["heading_rad"]),
            )
            for row in csv.DictReader(handle)
        )
    for point in points:
        point.validate()
    if len(points) < 3 or len({point.point_id for point in points}) != len(points):
        raise ValueError("route reference requires at least three unique points")
    return tuple(sorted(points, key=lambda point: point.point_id))


def verify_route_reference_manifest_v3(path: str | Path) -> dict[str, Any]:
    """Verify the adjacent teacher-only manifest and CSV content hash."""

    source = Path(path)
    manifest_path = source.with_suffix(".manifest.yaml")
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Reference manifest not found: {manifest_path}")
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("format_version") != COLLECTION_REFERENCE_FORMAT:
        raise ValueError(f"unsupported Reference manifest: {manifest_path}")
    if raw.get("teacher_debug_only") is not True:
        raise ValueError("Reference manifest must set teacher_debug_only: true")
    expected_hash = str(raw.get("reference_sha256", ""))
    actual_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if expected_hash != actual_hash:
        raise ValueError(
            f"Reference SHA-256 mismatch: expected={expected_hash!r}, actual={actual_hash!r}"
        )
    points = load_route_reference_v3(source)
    if int(raw.get("point_count", -1)) != len(points):
        raise ValueError("Reference manifest point_count does not match the CSV")
    if str(raw.get("coordinate_frame", "")) != points[0].frame_id:
        raise ValueError("Reference manifest coordinate_frame does not match the CSV")
    return dict(raw)


def _route_curvatures(points: Sequence[RoutePointV3]) -> tuple[float, ...]:
    result: list[float] = []
    for index, point in enumerate(points):
        previous = points[(index - 1) % len(points)]
        following = points[(index + 1) % len(points)]
        distance = math.hypot(following.x_m - previous.x_m, following.y_m - previous.y_m)
        result.append(
            0.0
            if distance <= 1e-6
            else wrap_angle_rad(following.heading_rad - previous.heading_rad) / distance
        )
    return tuple(result)


def project_to_route_v3(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    points: Sequence[RoutePointV3],
) -> RouteProjectionV3:
    """Project pose to the nearest route segment and return signed SI errors.

    Positive lateral offset is left of the reference heading. Positive heading
    error is counter-clockwise from the reference heading. Segment projection
    avoids treating along-track distance as lateral error on a sparse curved
    Reference. The route is circular, matching the collection course contract.
    """

    if len(points) < 3:
        raise ValueError("route projection requires at least three points")
    curvatures = _route_curvatures(points)
    best: tuple[float, int, float, float, float] | None = None
    query_x = float(x_m)
    query_y = float(y_m)
    for index, point in enumerate(points):
        following = points[(index + 1) % len(points)]
        segment_x = following.x_m - point.x_m
        segment_y = following.y_m - point.y_m
        squared_length = segment_x * segment_x + segment_y * segment_y
        if squared_length <= 1e-12:
            continue
        fraction = max(
            0.0,
            min(
                1.0,
                ((query_x - point.x_m) * segment_x + (query_y - point.y_m) * segment_y)
                / squared_length,
            ),
        )
        projection_x = point.x_m + fraction * segment_x
        projection_y = point.y_m + fraction * segment_y
        squared_distance = (query_x - projection_x) ** 2 + (query_y - projection_y) ** 2
        candidate = (squared_distance, index, fraction, projection_x, projection_y)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("route projection found no non-degenerate segment")
    _, index, fraction, projection_x, projection_y = best
    point = points[index]
    following = points[(index + 1) % len(points)]
    heading_delta = wrap_angle_rad(following.heading_rad - point.heading_rad)
    heading = wrap_angle_rad(point.heading_rad + fraction * heading_delta)
    dx = query_x - projection_x
    dy = query_y - projection_y
    lateral = -math.sin(heading) * dx + math.cos(heading) * dy
    curvature = (1.0 - fraction) * curvatures[index] + fraction * curvatures[
        (index + 1) % len(points)
    ]
    return RouteProjectionV3(
        point_id=point.point_id,
        lateral_offset_m=lateral,
        heading_error_rad=wrap_angle_rad(float(yaw_rad) - heading),
        curvature_inv_m=curvature,
    )


def classify_collection_coverage_v3(
    samples: Sequence[TeacherStateSampleV3],
    route: Sequence[RoutePointV3],
    criteria: CollectionCriteriaV3,
) -> dict[str, Any]:
    """Classify teacher states and evaluate versioned coverage gates."""

    criteria.validate()
    ordered = sorted(samples, key=lambda sample: (sample.run_id, sample.timestamp_ns))
    for sample in ordered:
        sample.validate()
    projections = [project_to_route_v3(s.x_m, s.y_m, s.yaw_rad, route) for s in ordered]
    labels_by_index: list[set[str]] = [set() for _ in ordered]
    launch_ns = int(criteria.launch_horizon_sec * 1e9)
    recovery_ns = int(criteria.recovery_horizon_sec * 1e9)

    run_ranges: dict[str, tuple[int, int]] = {}
    for index, sample in enumerate(ordered):
        start, _ = run_ranges.get(sample.run_id, (index, index))
        run_ranges[sample.run_id] = (start, index + 1)

    for index, (sample, projection) in enumerate(zip(ordered, projections, strict=True)):
        labels = labels_by_index[index]
        speed = abs(sample.speed_mps)
        if speed <= criteria.stopped_speed_mps:
            labels.add("stopped")
        elif speed < criteria.moving_speed_mps:
            labels.add("low_speed")
        else:
            labels.add("moving")
        if projection.curvature_inv_m >= criteria.curve_curvature_inv_m:
            labels.add("left_curve")
        elif projection.curvature_inv_m <= -criteria.curve_curvature_inv_m:
            labels.add("right_curve")
        else:
            labels.add("straight")

        lateral = projection.lateral_offset_m
        if lateral >= criteria.lateral_offset_min_m:
            labels.add("offset_left_far" if lateral >= criteria.lateral_offset_far_m else "offset_left_near")
        elif lateral <= -criteria.lateral_offset_min_m:
            labels.add("offset_right_far" if lateral <= -criteria.lateral_offset_far_m else "offset_right_near")
        if projection.heading_error_rad >= criteria.heading_error_min_rad:
            labels.add("heading_left")
        elif projection.heading_error_rad <= -criteria.heading_error_min_rad:
            labels.add("heading_right")

        _, run_end = run_ranges[sample.run_id]
        launch_future: list[float] = []
        recovery_future: list[float] = []
        for future_index in range(index + 1, run_end):
            delta_ns = ordered[future_index].timestamp_ns - sample.timestamp_ns
            if delta_ns <= launch_ns:
                launch_future.append(abs(ordered[future_index].speed_mps))
            if delta_ns <= recovery_ns:
                recovery_future.append(abs(projections[future_index].lateral_offset_m))
            if delta_ns > max(launch_ns, recovery_ns):
                break
        if speed <= criteria.stopped_speed_mps and launch_future and max(launch_future) >= criteria.moving_speed_mps:
            labels.add("launch")
        if speed >= criteria.moving_speed_mps and launch_future and min(launch_future) <= criteria.stopped_speed_mps:
            labels.add("stop_approach")
        if (
            abs(lateral) >= criteria.lateral_offset_min_m
            and recovery_future
            and min(recovery_future) <= abs(lateral) - criteria.recovery_improvement_m
        ):
            labels.add("recovery_left" if lateral > 0.0 else "recovery_right")

    gap_ns = int(criteria.episode_gap_sec * 1e9)
    report: dict[str, Any] = {}
    gaps: list[dict[str, Any]] = []
    for label, requirement in criteria.requirements.items():
        indices = [index for index, labels in enumerate(labels_by_index) if label in labels]
        runs = sorted({ordered[index].run_id for index in indices})
        episodes = 0
        previous_index: int | None = None
        for index in indices:
            if (
                previous_index is None
                or ordered[index].run_id != ordered[previous_index].run_id
                or ordered[index].timestamp_ns - ordered[previous_index].timestamp_ns > gap_ns
            ):
                episodes += 1
            previous_index = index
        passed = (
            len(indices) >= requirement.min_samples
            and len(runs) >= requirement.min_runs
            and episodes >= requirement.min_episodes
        )
        report[label] = {
            "status": "PASS" if passed else "FAIL",
            "samples": len(indices),
            "runs": len(runs),
            "episodes": episodes,
            "required": {
                "min_samples": requirement.min_samples,
                "min_runs": requirement.min_runs,
                "min_episodes": requirement.min_episodes,
            },
        }
        if not passed:
            gaps.append(
                {
                    "bucket": label,
                    "additional_samples": max(0, requirement.min_samples - len(indices)),
                    "additional_runs": max(0, requirement.min_runs - len(runs)),
                    "additional_episodes": max(0, requirement.min_episodes - episodes),
                    "collection_instruction": _collection_instruction(label),
                }
            )
    return {
        "format_version": COLLECTION_CRITERIA_FORMAT,
        "sample_count": len(ordered),
        "run_count": len(run_ranges),
        "overall_status": "PASS" if not gaps else "FAIL",
        "buckets": report,
        "collection_gaps": gaps,
    }


def _collection_instruction(label: str) -> str:
    instructions = {
        "launch": "Record expert-controlled stop-to-launch transitions from multiple course positions.",
        "stop_approach": "Record expert-controlled deceleration to a full stop from moving speed.",
        "offset_left_near": "Start left of the reference by the near-offset band and hold expert authority.",
        "offset_right_near": "Start right of the reference by the near-offset band and hold expert authority.",
        "offset_left_far": "Start left of the reference by the far-offset band and recover safely.",
        "offset_right_far": "Start right of the reference by the far-offset band and recover safely.",
        "heading_left": "Start with positive heading error and let the expert recover.",
        "heading_right": "Start with negative heading error and let the expert recover.",
        "recovery_left": "Record successful expert recovery from a left lateral displacement.",
        "recovery_right": "Record successful expert recovery from a right lateral displacement.",
    }
    return instructions.get(label, f"Record additional expert-controlled examples for {label}.")


def write_coverage_report_v3(output: str | Path, report: Mapping[str, Any]) -> None:
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(dict(report), indent=2, sort_keys=True), encoding="utf-8")
