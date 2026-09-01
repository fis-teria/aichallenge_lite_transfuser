from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pandas as pd
from PIL import Image
import yaml


CAMERA_TOPIC = "/sensing/camera/image_raw"
LIDAR_TOPIC = "/sensing/lidar/scan"
CONTROL_TOPIC = "/control/command/control_cmd"


@dataclass(frozen=True)
class ConverterConfig:
    """Canonical converter settings.

    Units are encoded in field names. ``waypoint_times_sec`` must be strictly
    increasing. ``lidar_points`` is the saved one-dimensional LaserScan shape.
    """

    sample_rate_hz: float
    sync_tolerance_ms: float
    lidar_points: int
    waypoint_times_sec: tuple[float, ...]
    wheelbase_m: float
    label_shift_ms: float = 0.0
    target_speed_offset_sec: float = 0.5
    stop_speed_threshold_mps: float = 0.1
    min_usable_commanded_speed_mps: float = 0.5
    jpeg_quality: int = 90

    def validate(self) -> None:
        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        if self.sync_tolerance_ms <= 0.0:
            raise ValueError("sync_tolerance_ms must be positive")
        if self.lidar_points <= 1:
            raise ValueError("lidar_points must be greater than 1")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if not self.waypoint_times_sec:
            raise ValueError("waypoint_times_sec must not be empty")
        if any(value <= 0.0 for value in self.waypoint_times_sec):
            raise ValueError("waypoint_times_sec must be positive")
        if any(
            right <= left
            for left, right in zip(self.waypoint_times_sec, self.waypoint_times_sec[1:])
        ):
            raise ValueError("waypoint_times_sec must be strictly increasing")
        if self.target_speed_offset_sec <= 0.0:
            raise ValueError("target_speed_offset_sec must be positive")
        if self.stop_speed_threshold_mps < 0.0:
            raise ValueError("stop_speed_threshold_mps must be non-negative")
        if self.min_usable_commanded_speed_mps < 0.0:
            raise ValueError("min_usable_commanded_speed_mps must be non-negative")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")


@dataclass(frozen=True)
class TimedControl:
    timestamp_ns: int
    speed_mps: float
    acceleration_mps2: float
    steering_rad: float


@dataclass(frozen=True)
class TimedLidar:
    timestamp_ns: int
    ranges_m: np.ndarray
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float


@dataclass(frozen=True)
class TimedImage:
    timestamp_ns: int
    image_rgb: np.ndarray


@dataclass(frozen=True)
class RunConversionResult:
    run_id: str
    rows: list[dict[str, Any]]
    input_counts: dict[str, int]
    dropped_counts: dict[str, int]
    lidar_geometry: dict[str, Any]


def message_image_to_rgb(message: Any) -> np.ndarray:
    """Decode a ROS sensor_msgs/Image into RGB uint8 ``[H, W, 3]``.

    Supported encodings are rgb8, bgr8, rgba8, bgra8 and mono8. Row padding
    declared by ``step`` is discarded. Unsupported encodings fail explicitly.
    """

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0:
        raise ValueError(f"Invalid image shape height={height}, width={width}")

    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f"Unsupported sensor_msgs/Image encoding={encoding!r}")
    channels = channels_by_encoding[encoding]
    row_bytes = width * channels
    if step < row_bytes:
        raise ValueError(
            f"Image step={step} is smaller than width*channels={row_bytes}"
        )

    flat = np.asarray(message.data, dtype=np.uint8).reshape(-1)
    expected_bytes = height * step
    if flat.size != expected_bytes:
        raise ValueError(
            f"Image data has {flat.size} bytes, expected height*step={expected_bytes}"
        )
    pixels = flat.reshape(height, step)[:, :row_bytes].reshape(height, width, channels)
    if encoding == "rgb8":
        rgb = pixels
    elif encoding == "bgr8":
        rgb = pixels[:, :, ::-1]
    elif encoding == "rgba8":
        rgb = pixels[:, :, :3]
    elif encoding == "bgra8":
        rgb = pixels[:, :, [2, 1, 0]]
    else:
        rgb = np.repeat(pixels, 3, axis=2)
    result = np.ascontiguousarray(rgb, dtype=np.uint8)
    if result.shape != (height, width, 3):
        raise AssertionError(f"Decoded image has unexpected shape={result.shape}")
    return result


def resample_lidar_nearest(ranges_m: np.ndarray, output_points: int) -> np.ndarray:
    """Resample a 1D LaserScan on its angular index using nearest neighbours.

    NaN and infinity are intentionally preserved for downstream sanitization.
    The returned array has shape ``[output_points]`` and dtype float32.
    """

    source = np.asarray(ranges_m, dtype=np.float32)
    if source.ndim != 1:
        raise ValueError(f"Expected 1D LaserScan, got shape={source.shape}")
    if source.size <= 1:
        raise ValueError("LaserScan must contain at least two points")
    if output_points <= 1:
        raise ValueError("output_points must be greater than 1")
    if source.size == output_points:
        return source.copy()
    indices = np.rint(np.linspace(0, source.size - 1, output_points)).astype(np.int64)
    result = source[indices].astype(np.float32, copy=True)
    if result.shape != (output_points,):
        raise AssertionError(f"Resampled LiDAR has unexpected shape={result.shape}")
    return result


def nearest_index(
    timestamps_ns: Sequence[int], target_ns: int, tolerance_ns: int
) -> tuple[int, int] | None:
    """Return nearest index and signed ``sample-target`` delta in nanoseconds."""

    if not timestamps_ns:
        return None
    pos = bisect_left(timestamps_ns, target_ns)
    candidates = [idx for idx in (pos - 1, pos) if 0 <= idx < len(timestamps_ns)]
    index = min(candidates, key=lambda idx: abs(timestamps_ns[idx] - target_ns))
    delta_ns = int(timestamps_ns[index] - target_ns)
    return (index, delta_ns) if abs(delta_ns) <= tolerance_ns else None


def previous_index(
    timestamps_ns: Sequence[int], target_ns: int, tolerance_ns: int | None = None
) -> tuple[int, int] | None:
    """Return latest index at or before target and signed delta in nanoseconds."""

    if not timestamps_ns:
        return None
    index = bisect_right(timestamps_ns, target_ns) - 1
    if index < 0:
        return None
    delta_ns = int(timestamps_ns[index] - target_ns)
    if tolerance_ns is not None and abs(delta_ns) > tolerance_ns:
        return None
    return index, delta_ns


def integrate_command_waypoints(
    controls: Sequence[TimedControl],
    observation_ns: int,
    waypoint_times_sec: Sequence[float],
    wheelbase_m: float,
) -> np.ndarray:
    """Integrate commanded speed and steering with a kinematic bicycle model.

    Commands are zero-order-held using only the latest command at each time.
    Future commands are teacher-only label sources. Returned waypoints use the
    observation ego frame with x forward, y left, shape ``[N, 2]`` in metres.
    """

    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    if not controls:
        raise ValueError("controls must not be empty")
    horizons = tuple(float(value) for value in waypoint_times_sec)
    if not horizons or any(value <= 0.0 for value in horizons):
        raise ValueError("waypoint_times_sec must contain positive values")
    if any(right <= left for left, right in zip(horizons, horizons[1:])):
        raise ValueError("waypoint_times_sec must be strictly increasing")

    control_times = [item.timestamp_ns for item in controls]
    current = previous_index(control_times, observation_ns)
    if current is None:
        raise ValueError("No control command exists at or before observation")
    control_index = current[0]
    current_ns = observation_ns
    x_m = 0.0
    y_m = 0.0
    yaw_rad = 0.0
    waypoints: list[tuple[float, float]] = []

    for horizon_sec in horizons:
        target_ns = observation_ns + int(round(horizon_sec * 1e9))
        while current_ns < target_ns:
            next_change_ns = (
                controls[control_index + 1].timestamp_ns
                if control_index + 1 < len(controls)
                else target_ns
            )
            segment_end_ns = min(target_ns, max(current_ns, next_change_ns))
            if segment_end_ns == current_ns:
                control_index += 1
                continue
            dt_sec = (segment_end_ns - current_ns) / 1e9
            command = controls[control_index]
            speed_mps = float(command.speed_mps)
            yaw_rate_rps = speed_mps * math.tan(float(command.steering_rad)) / wheelbase_m
            if abs(yaw_rate_rps) < 1e-9:
                x_m += speed_mps * math.cos(yaw_rad) * dt_sec
                y_m += speed_mps * math.sin(yaw_rad) * dt_sec
            else:
                next_yaw_rad = yaw_rad + yaw_rate_rps * dt_sec
                radius_m = speed_mps / yaw_rate_rps
                x_m += radius_m * (math.sin(next_yaw_rad) - math.sin(yaw_rad))
                y_m += -radius_m * (math.cos(next_yaw_rad) - math.cos(yaw_rad))
                yaw_rad = next_yaw_rad
            current_ns = segment_end_ns
            if (
                control_index + 1 < len(controls)
                and current_ns >= controls[control_index + 1].timestamp_ns
            ):
                control_index += 1
        waypoints.append((x_m, y_m))

    result = np.asarray(waypoints, dtype=np.float32)
    if result.shape != (len(horizons), 2):
        raise AssertionError(f"Integrated waypoints have unexpected shape={result.shape}")
    if not np.isfinite(result).all():
        raise ValueError("Integrated waypoints contain NaN or infinity")
    return result


def _command_at(
    controls: Sequence[TimedControl], timestamp_ns: int, tolerance_ns: int | None = None
) -> tuple[TimedControl, int] | None:
    found = previous_index(
        [item.timestamp_ns for item in controls], timestamp_ns, tolerance_ns
    )
    return None if found is None else (controls[found[0]], found[1])


def _scenario_id(run_id: str) -> str:
    return run_id


def _safe_run_id(value: str) -> str:
    safe = "".join(char if char.isalnum() or char in "-_" else "_" for char in value)
    if not safe:
        raise ValueError(f"Cannot derive a safe run id from {value!r}")
    return safe


def _select_camera_samples(
    images: Sequence[TimedImage], sample_rate_hz: float, max_samples: int | None
) -> list[TimedImage]:
    interval_ns = int(round(1e9 / sample_rate_hz))
    selected: list[TimedImage] = []
    next_allowed_ns: int | None = None
    for image in images:
        if next_allowed_ns is None or image.timestamp_ns >= next_allowed_ns:
            selected.append(image)
            next_allowed_ns = image.timestamp_ns + interval_ns
            if max_samples is not None and len(selected) >= max_samples:
                break
    return selected


def read_run_messages(
    bag_dir: Path,
    *,
    camera_topic: str = CAMERA_TOPIC,
    lidar_topic: str = LIDAR_TOPIC,
    control_topic: str = CONTROL_TOPIC,
) -> tuple[list[TimedImage], list[TimedLidar], list[TimedControl]]:
    """Read the three required streams from one compressed rosbag2 MCAP run."""

    from rosbags.highlevel import AnyReader

    if not (bag_dir / "metadata.yaml").is_file():
        raise FileNotFoundError(f"rosbag2 metadata not found: {bag_dir / 'metadata.yaml'}")

    images: list[TimedImage] = []
    lidars: list[TimedLidar] = []
    controls: list[TimedControl] = []
    required_topics = {camera_topic, lidar_topic, control_topic}
    with AnyReader([bag_dir]) as reader:
        available = {connection.topic for connection in reader.connections}
        missing = sorted(required_topics.difference(available))
        if missing:
            raise ValueError(f"Run {bag_dir.name} is missing required topics: {missing}")
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in required_topics
        ]
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == camera_topic:
                images.append(
                    TimedImage(int(timestamp_ns), message_image_to_rgb(message))
                )
            elif connection.topic == lidar_topic:
                lidars.append(
                    TimedLidar(
                        timestamp_ns=int(timestamp_ns),
                        ranges_m=np.asarray(message.ranges, dtype=np.float32),
                        angle_min_rad=float(message.angle_min),
                        angle_increment_rad=float(message.angle_increment),
                        range_min_m=float(message.range_min),
                        range_max_m=float(message.range_max),
                    )
                )
            elif connection.topic == control_topic:
                controls.append(
                    TimedControl(
                        timestamp_ns=int(timestamp_ns),
                        speed_mps=float(message.longitudinal.speed),
                        acceleration_mps2=float(message.longitudinal.acceleration),
                        steering_rad=float(message.lateral.steering_tire_angle),
                    )
                )

    images.sort(key=lambda item: item.timestamp_ns)
    lidars.sort(key=lambda item: item.timestamp_ns)
    controls.sort(key=lambda item: item.timestamp_ns)
    if not images or not lidars or not controls:
        raise ValueError(
            f"Run {bag_dir.name} has empty required stream counts: "
            f"camera={len(images)}, lidar={len(lidars)}, control={len(controls)}"
        )
    return images, lidars, controls


def convert_run(
    bag_dir: Path,
    output_root: Path,
    config: ConverterConfig,
    *,
    max_samples: int | None = None,
) -> RunConversionResult:
    """Convert one rosbag2 directory into canonical JPEG/NPY files and rows."""

    config.validate()
    run_id = _safe_run_id(bag_dir.name)
    images, lidars, controls = read_run_messages(bag_dir)
    selected_images = _select_camera_samples(images, config.sample_rate_hz, max_samples)
    first_lidar = lidars[0]
    lidar_geometry = {
        "source_points": int(first_lidar.ranges_m.size),
        "saved_points": config.lidar_points,
        "angle_min_rad": first_lidar.angle_min_rad,
        "angle_increment_rad": first_lidar.angle_increment_rad,
        "range_min_m": first_lidar.range_min_m,
        "range_max_m": first_lidar.range_max_m,
        "resampling": "nearest_angular_index",
    }
    if max(abs(item.speed_mps) for item in controls) < config.min_usable_commanded_speed_mps:
        return RunConversionResult(
            run_id=run_id,
            rows=[],
            input_counts={
                "camera": len(images),
                "selected_camera": len(selected_images),
                "lidar": len(lidars),
                "control": len(controls),
                "output": 0,
            },
            dropped_counts={"unusable_commanded_speed_run": len(selected_images)},
            lidar_geometry=lidar_geometry,
        )

    lidar_times = [item.timestamp_ns for item in lidars]
    control_times = [item.timestamp_ns for item in controls]
    tolerance_ns = int(round(config.sync_tolerance_ms * 1e6))
    horizon_ns = int(round(max(config.waypoint_times_sec) * 1e9))
    target_speed_offset_ns = int(round(config.target_speed_offset_sec * 1e9))
    label_shift_ns = int(round(config.label_shift_ms * 1e6))
    required_future_ns = max(horizon_ns, target_speed_offset_ns, label_shift_ns)

    image_dir = output_root / "images" / run_id
    lidar_dir = output_root / "lidar" / run_id
    image_dir.mkdir(parents=True, exist_ok=True)
    lidar_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    dropped = {
        "lidar_sync": 0,
        "current_control_sync": 0,
        "future_control_missing": 0,
        "invalid_control": 0,
    }
    for image in selected_images:
        lidar_match = nearest_index(lidar_times, image.timestamp_ns, tolerance_ns)
        current_match = previous_index(control_times, image.timestamp_ns, tolerance_ns)
        if lidar_match is None:
            dropped["lidar_sync"] += 1
            continue
        if current_match is None:
            dropped["current_control_sync"] += 1
            continue
        if control_times[-1] < image.timestamp_ns + required_future_ns:
            dropped["future_control_missing"] += 1
            continue

        target_control = _command_at(
            controls, image.timestamp_ns + target_speed_offset_ns
        )
        direct_control = _command_at(controls, image.timestamp_ns + label_shift_ns)
        if target_control is None or direct_control is None:
            dropped["future_control_missing"] += 1
            continue
        current_control = controls[current_match[0]]
        numeric = (
            current_control.speed_mps,
            current_control.acceleration_mps2,
            current_control.steering_rad,
            target_control[0].speed_mps,
            direct_control[0].acceleration_mps2,
            direct_control[0].steering_rad,
        )
        if not all(math.isfinite(value) for value in numeric):
            dropped["invalid_control"] += 1
            continue

        waypoints = integrate_command_waypoints(
            controls,
            image.timestamp_ns,
            config.waypoint_times_sec,
            config.wheelbase_m,
        )
        lidar = lidars[lidar_match[0]]
        lidar_ranges = resample_lidar_nearest(lidar.ranges_m, config.lidar_points)
        sample_id = f"{run_id}_{len(rows):06d}"
        image_rel = Path("images") / run_id / f"{sample_id}.jpg"
        lidar_rel = Path("lidar") / run_id / f"{sample_id}.npy"
        Image.fromarray(image.image_rgb, mode="RGB").save(
            output_root / image_rel, quality=config.jpeg_quality
        )
        np.save(output_root / lidar_rel, lidar_ranges, allow_pickle=False)

        speed_mps = current_control.speed_mps
        steering_rad = current_control.steering_rad
        heading_rate_rps = speed_mps * math.tan(steering_rad) / config.wheelbase_m
        target_speed_mps = target_control[0].speed_mps
        stop_flag = int(abs(target_speed_mps) <= config.stop_speed_threshold_mps)
        row: dict[str, Any] = {
            "sample_id": sample_id,
            "run_id": run_id,
            "scenario_id": _scenario_id(run_id),
            "timestamp_ns": image.timestamp_ns,
            "image_path": image_rel.as_posix(),
            "lidar_path": lidar_rel.as_posix(),
            "velocity_mps": speed_mps,
            "steering_rad": steering_rad,
            "heading_rate_rps": heading_rate_rps,
            "gear": 1 if speed_mps >= 0.0 else -1,
            "target_speed_mps": target_speed_mps,
            "stop_flag": stop_flag,
            "behavior_mode": 5 if stop_flag else 0,
            "direct_steering_rad": direct_control[0].steering_rad,
            "direct_acceleration_mps2": direct_control[0].acceleration_mps2,
            "camera_dt_ms": 0.0,
            "lidar_dt_ms": lidar_match[1] / 1e6,
            "ego_dt_ms": current_match[1] / 1e6,
            "control_dt_ms": direct_control[1] / 1e6,
            "source_dataset": bag_dir.parent.name,
            "quality_score": 0.5,
        }
        for index, (x_m, y_m) in enumerate(waypoints):
            row[f"wp_{index}_x"] = float(x_m)
            row[f"wp_{index}_y"] = float(y_m)
        rows.append(row)

    return RunConversionResult(
        run_id=run_id,
        rows=rows,
        input_counts={
            "camera": len(images),
            "selected_camera": len(selected_images),
            "lidar": len(lidars),
            "control": len(controls),
            "output": len(rows),
        },
        dropped_counts=dropped,
        lidar_geometry=lidar_geometry,
    )


def discover_bag_directories(input_root: Path) -> list[Path]:
    """Discover immediate rosbag2 child directories in deterministic order."""

    if not input_root.is_dir():
        raise NotADirectoryError(f"Input root not found: {input_root}")
    result = sorted(
        path.parent
        for path in input_root.glob("*/metadata.yaml")
        if path.is_file()
    )
    if not result and (input_root / "metadata.yaml").is_file():
        result = [input_root]
    if not result:
        raise FileNotFoundError(f"No rosbag2 metadata.yaml found under {input_root}")
    return result


def assign_run_splits(
    run_ids: Sequence[str],
    *,
    val_ratio: float,
    test_ratio: float,
    seed: int,
) -> dict[str, str]:
    """Assign each complete run to one deterministic split."""

    if val_ratio < 0.0 or test_ratio < 0.0 or val_ratio + test_ratio >= 1.0:
        raise ValueError("val_ratio and test_ratio must be non-negative and sum to < 1")
    unique = sorted(set(run_ids))
    ordered = sorted(
        unique,
        key=lambda run_id: hashlib.sha256(f"{seed}:{run_id}".encode()).digest(),
    )
    count = len(ordered)
    test_count = int(round(count * test_ratio))
    val_count = int(round(count * val_ratio))
    if test_ratio > 0.0 and count >= 3:
        test_count = max(1, test_count)
    if val_ratio > 0.0 and count - test_count >= 2:
        val_count = max(1, val_count)
    if test_count + val_count >= count:
        overflow = test_count + val_count - (count - 1)
        val_count = max(0, val_count - overflow)

    mapping = {run_id: "train" for run_id in ordered}
    for run_id in ordered[:test_count]:
        mapping[run_id] = "test"
    for run_id in ordered[test_count : test_count + val_count]:
        mapping[run_id] = "val"
    return mapping


def convert_dataset(
    input_root: Path,
    output_root: Path,
    config: ConverterConfig,
    *,
    max_runs: int | None = None,
    max_samples_per_run: int | None = None,
    val_ratio: float = 0.2,
    test_ratio: float = 0.1,
    split_seed: int = 42,
) -> dict[str, Any]:
    """Convert rosbag2 MCAP runs and write canonical indices plus metadata."""

    config.validate()
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    bag_dirs = discover_bag_directories(input_root)
    if max_runs is not None:
        if max_runs <= 0:
            raise ValueError("max_runs must be positive")
        bag_dirs = bag_dirs[:max_runs]

    results = [
        convert_run(
            bag_dir,
            output_root,
            config,
            max_samples=max_samples_per_run,
        )
        for bag_dir in bag_dirs
    ]
    rows = [row for result in results for row in result.rows]
    if not rows:
        raise RuntimeError("Conversion produced no valid samples")

    converted_run_ids = [result.run_id for result in results if result.rows]
    splits = assign_run_splits(
        converted_run_ids,
        val_ratio=val_ratio,
        test_ratio=test_ratio,
        seed=split_seed,
    )
    for row in rows:
        row["split"] = splits[row["run_id"]]
    frame = pd.DataFrame(rows).sort_values(["run_id", "timestamp_ns"])
    frame.to_csv(output_root / "index.csv", index=False)
    for split in ("train", "val", "test"):
        split_frame = frame[frame["split"] == split]
        split_frame.to_csv(output_root / f"{split}_index.csv", index=False)

    metadata: dict[str, Any] = {
        "format_version": 1,
        "source_root": str(input_root.resolve()),
        "topics": {
            "camera": CAMERA_TOPIC,
            "lidar": LIDAR_TOPIC,
            "control": CONTROL_TOPIC,
        },
        "converter": asdict(config),
        "coordinate_contract": {
            "waypoint_frame": "observation_ego",
            "x_axis": "forward",
            "y_axis": "left",
            "distance_unit": "m",
            "angle_unit": "rad",
            "time_unit": "s",
        },
        "input_provenance": {
            "camera": "sensor_msgs/Image",
            "lidar": "sensor_msgs/LaserScan",
            "ego_state": "latest commanded AckermannControlCommand at or before observation",
        },
        "teacher_debug_only_provenance": {
            "future_waypoints": "future commanded speed/steering integrated with kinematic bicycle",
            "target_speed": (
                f"commanded speed at observation+{config.target_speed_offset_sec}s"
            ),
            "stop_flag": "proxy: abs(target_speed_mps) <= stop_speed_threshold_mps",
            "behavior_mode": "proxy: stop when stop_flag=1, otherwise follow",
            "direct_control": (
                f"command at observation+{config.label_shift_ms}ms"
            ),
        },
        "limitations": [
            "No measured odometry/pose topic exists in the source bags.",
            "Ego state and labels are command-derived proxies, not measured vehicle motion.",
            "Stop labels do not distinguish intentional stops from incidental zero speed.",
            "Behavior labels cannot identify avoidance direction and should use mode loss weight 0.",
            "Wheelbase must be confirmed against the official vehicle.",
            "Runs without a meaningful commanded speed are excluded because acceleration "
            "commands alone cannot recover measured velocity or pose.",
        ],
        "split_policy": {
            "unit": "run_id",
            "seed": split_seed,
            "val_ratio": val_ratio,
            "test_ratio": test_ratio,
            "assignments": splits,
        },
        "rows": int(len(frame)),
        "split_rows": {
            split: int((frame["split"] == split).sum())
            for split in ("train", "val", "test")
        },
        "runs": [
            {
                "run_id": result.run_id,
                "input_counts": result.input_counts,
                "dropped_counts": result.dropped_counts,
                "lidar_geometry": result.lidar_geometry,
            }
            for result in results
        ],
    }
    (output_root / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return metadata
