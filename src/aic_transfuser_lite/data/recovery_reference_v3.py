from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import posixpath
from typing import Any, Sequence

import numpy as np
from PIL import Image
import yaml


RECOVERY_REFERENCE_CONFIG_FORMAT = "aic_recovery_reference_generator_v1"
RECOVERY_REFERENCE_MANIFEST_FORMAT = "aic_recovery_reference_manifest_v1"


@dataclass(frozen=True)
class MpcReferencePointV3:
    """One map-frame point in the official MPC Reference CSV contract."""

    s_m: float
    x_m: float
    y_m: float
    psi_rad: float
    kappa_radpm: float
    vx_mps: float
    ax_mps2: float


@dataclass(frozen=True)
class RecoverySegmentRequestV3:
    segment_id: str
    side: str
    offset_m: float
    geometry: str

    @property
    def signed_offset_m(self) -> float:
        return self.offset_m if self.side == "left" else -self.offset_m

    def validate(self) -> None:
        if not self.segment_id.strip():
            raise ValueError("segment_id must not be empty")
        if self.side not in {"left", "right"}:
            raise ValueError(f"unsupported recovery side: {self.side!r}")
        if self.geometry not in {"left_curve", "right_curve", "mixed"}:
            raise ValueError(f"unsupported recovery geometry: {self.geometry!r}")
        if not math.isfinite(self.offset_m) or self.offset_m <= 0.0:
            raise ValueError("offset_m must be finite and positive")


@dataclass(frozen=True)
class RecoveryReferenceConfigV3:
    approach_length_m: float
    hold_length_m: float
    recovery_length_m: float
    minimum_segment_gap_m: float
    minimum_center_clearance_m: float
    geometry_curvature_threshold_inv_m: float
    preferred_abs_curvature_inv_m: float
    requests: tuple[RecoverySegmentRequestV3, ...]

    @property
    def segment_length_m(self) -> float:
        return self.approach_length_m + self.hold_length_m + self.recovery_length_m

    def validate(self) -> None:
        values = (
            self.approach_length_m,
            self.hold_length_m,
            self.recovery_length_m,
            self.minimum_segment_gap_m,
            self.minimum_center_clearance_m,
            self.geometry_curvature_threshold_inv_m,
            self.preferred_abs_curvature_inv_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("recovery Reference lengths and limits must be finite and positive")
        if not self.requests:
            raise ValueError("at least one recovery segment request is required")
        if self.preferred_abs_curvature_inv_m < self.geometry_curvature_threshold_inv_m:
            raise ValueError("preferred curvature must be at least the geometry threshold")
        ids: set[str] = set()
        for request in self.requests:
            request.validate()
            if request.segment_id in ids:
                raise ValueError(f"duplicate recovery segment_id: {request.segment_id}")
            ids.add(request.segment_id)


@dataclass(frozen=True)
class RecoveryPhaseV3:
    segment_id: str
    phase: str
    side: str
    offset_m: float
    geometry: str
    start_point_id: int
    end_point_id: int
    start_s_m: float
    end_s_m: float
    training_eligible: bool


@dataclass(frozen=True)
class GeneratedRecoveryReferenceV3:
    points: tuple[MpcReferencePointV3, ...]
    phases: tuple[RecoveryPhaseV3, ...]
    selected_segments: tuple[dict[str, Any], ...]
    minimum_center_clearance_m: float


@dataclass(frozen=True)
class OccupancyMapV3:
    """Binary occupancy map where ``True`` denotes a free 0.1 m-class cell."""

    free: np.ndarray
    resolution_m_per_px: float
    origin_x_m: float
    origin_y_m: float

    def validate(self) -> None:
        if self.free.ndim != 2 or self.free.dtype != np.bool_:
            raise ValueError("occupancy map free mask must be bool [H,W]")
        if min(self.free.shape) < 2:
            raise ValueError("occupancy map must be at least 2x2")
        values = (self.resolution_m_per_px, self.origin_x_m, self.origin_y_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("occupancy map metadata must be finite")
        if self.resolution_m_per_px <= 0.0:
            raise ValueError("occupancy map resolution must be positive")

    def footprint_is_free(self, x_m: np.ndarray, y_m: np.ndarray, radius_m: float) -> bool:
        """Return whether every circular vehicle footprint is inside free cells."""

        self.validate()
        x = np.asarray(x_m, dtype=np.float64)
        y = np.asarray(y_m, dtype=np.float64)
        if x.shape != y.shape or x.ndim != 1 or not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("footprint coordinates must be finite [N]")
        if not math.isfinite(radius_m) or radius_m <= 0.0:
            raise ValueError("footprint radius_m must be finite and positive")
        height, width = self.free.shape
        center_x = np.rint((x - self.origin_x_m) / self.resolution_m_per_px).astype(int)
        center_y = np.rint(
            (height - 1) - (y - self.origin_y_m) / self.resolution_m_per_px
        ).astype(int)
        radius_px = int(math.ceil(radius_m / self.resolution_m_per_px))
        axis = np.arange(-radius_px, radius_px + 1)
        grid_x, grid_y = np.meshgrid(axis, axis)
        disk = (grid_x * self.resolution_m_per_px) ** 2 + (
            grid_y * self.resolution_m_per_px
        ) ** 2 <= radius_m**2
        disk_x = grid_x[disk]
        disk_y = grid_y[disk]
        for cx, cy in zip(center_x, center_y):
            px = cx + disk_x
            py = cy + disk_y
            if bool((px < 0).any() or (px >= width).any() or (py < 0).any() or (py >= height).any()):
                return False
            if not bool(self.free[py, px].all()):
                return False
        return True


def load_recovery_reference_config_v3(path: str | Path) -> RecoveryReferenceConfigV3:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("format_version") != RECOVERY_REFERENCE_CONFIG_FORMAT:
        raise ValueError(f"unsupported recovery Reference config: {raw!r}")
    phases = raw.get("phase_lengths_m")
    requests = raw.get("segments")
    if not isinstance(phases, dict) or not isinstance(requests, list):
        raise ValueError("phase_lengths_m and segments are required")
    config = RecoveryReferenceConfigV3(
        approach_length_m=float(phases["approach"]),
        hold_length_m=float(phases["hold"]),
        recovery_length_m=float(phases["recovery"]),
        minimum_segment_gap_m=float(raw["minimum_segment_gap_m"]),
        minimum_center_clearance_m=float(raw["minimum_center_clearance_m"]),
        geometry_curvature_threshold_inv_m=float(raw["geometry_curvature_threshold_inv_m"]),
        preferred_abs_curvature_inv_m=float(raw["preferred_abs_curvature_inv_m"]),
        requests=tuple(
            RecoverySegmentRequestV3(
                segment_id=str(item["segment_id"]),
                side=str(item["side"]),
                offset_m=float(item["offset_m"]),
                geometry=str(item["geometry"]),
            )
            for item in requests
        ),
    )
    config.validate()
    return config


def load_mpc_reference_v3(path: str | Path) -> tuple[MpcReferencePointV3, ...]:
    source = Path(path)
    required = ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")
    with source.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(required):
            raise ValueError(f"MPC Reference columns must be {required}, got {reader.fieldnames}")
        points = tuple(MpcReferencePointV3(*(float(row[name]) for name in required)) for row in reader)
    if len(points) < 20:
        raise ValueError("MPC Reference requires at least 20 points")
    values = np.asarray([[getattr(point, name) for name in required] for point in points])
    if not np.isfinite(values).all():
        raise ValueError("MPC Reference values must be finite")
    s_m = values[:, 0]
    if abs(float(s_m[0])) > 1e-6 or bool((np.diff(s_m) <= 0.0).any()):
        raise ValueError("MPC Reference s_m must start at zero and increase strictly")
    return points


def load_occupancy_map_v3(path: str | Path) -> OccupancyMapV3:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("occupancy map YAML must be a mapping")
    image_path = source.parent / str(raw["image"])
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64) / 255.0
    if int(raw.get("negate", 0)):
        pixels = 1.0 - pixels
    origin = raw["origin"]
    result = OccupancyMapV3(
        free=np.asarray(pixels >= float(raw["occupied_thresh"]), dtype=np.bool_),
        resolution_m_per_px=float(raw["resolution"]),
        origin_x_m=float(origin[0]),
        origin_y_m=float(origin[1]),
    )
    result.validate()
    return result


def _smoothstep5(unit_interval: np.ndarray) -> np.ndarray:
    u = np.clip(np.asarray(unit_interval, dtype=np.float64), 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _offset_profile(relative_s_m: np.ndarray, request: RecoverySegmentRequestV3, config: RecoveryReferenceConfigV3) -> np.ndarray:
    relative = np.asarray(relative_s_m, dtype=np.float64)
    offset = np.zeros_like(relative)
    approach_end = config.approach_length_m
    hold_end = approach_end + config.hold_length_m
    segment_end = hold_end + config.recovery_length_m
    approach = (relative >= 0.0) & (relative < approach_end)
    hold = (relative >= approach_end) & (relative < hold_end)
    recovery = (relative >= hold_end) & (relative <= segment_end)
    signed = request.signed_offset_m
    offset[approach] = signed * _smoothstep5(relative[approach] / approach_end)
    offset[hold] = signed
    offset[recovery] = signed * (
        1.0 - _smoothstep5((relative[recovery] - hold_end) / config.recovery_length_m)
    )
    return offset


def _geometry_name(mean_curvature_inv_m: float, threshold_inv_m: float) -> str:
    if mean_curvature_inv_m >= threshold_inv_m:
        return "left_curve"
    if mean_curvature_inv_m <= -threshold_inv_m:
        return "right_curve"
    return "mixed"


def _densify_polyline(
    x_m: np.ndarray, y_m: np.ndarray, *, maximum_step_m: float
) -> tuple[np.ndarray, np.ndarray]:
    if maximum_step_m <= 0.0 or not math.isfinite(maximum_step_m):
        raise ValueError("maximum polyline step must be finite and positive")
    dense_x: list[float] = []
    dense_y: list[float] = []
    for index in range(len(x_m) - 1):
        distance = math.hypot(x_m[index + 1] - x_m[index], y_m[index + 1] - y_m[index])
        steps = max(1, int(math.ceil(distance / maximum_step_m)))
        for alpha in np.linspace(0.0, 1.0, steps, endpoint=False):
            dense_x.append(float(x_m[index] + alpha * (x_m[index + 1] - x_m[index])))
            dense_y.append(float(y_m[index] + alpha * (y_m[index + 1] - y_m[index])))
    dense_x.append(float(x_m[-1]))
    dense_y.append(float(y_m[-1]))
    return np.asarray(dense_x), np.asarray(dense_y)


def _recompute_geometry(x_m: np.ndarray, y_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    count = len(x_m)
    next_index = np.roll(np.arange(count), -1)
    previous_index = np.roll(np.arange(count), 1)
    dx = x_m[next_index] - x_m
    dy = y_m[next_index] - y_m
    psi = np.unwrap(np.arctan2(dy, dx))
    ds = np.hypot(np.diff(x_m), np.diff(y_m))
    if bool((ds <= 1e-6).any()):
        raise ValueError("generated MPC Reference contains duplicate adjacent points")
    s = np.concatenate(([0.0], np.cumsum(ds)))
    chord = np.hypot(x_m[next_index] - x_m[previous_index], y_m[next_index] - y_m[previous_index])
    heading_delta = np.angle(np.exp(1j * (psi[next_index] - psi[previous_index])))
    kappa = np.divide(heading_delta, chord, out=np.zeros(count), where=chord > 1e-6)
    return s, np.angle(np.exp(1j * psi)), kappa


def _intervals_overlap(start_a: float, end_a: float, start_b: float, end_b: float, gap_m: float) -> bool:
    return not (end_a + gap_m <= start_b or end_b + gap_m <= start_a)


def generate_recovery_reference_v3(
    base_points: Sequence[MpcReferencePointV3],
    occupancy: OccupancyMapV3,
    config: RecoveryReferenceConfigV3,
) -> GeneratedRecoveryReferenceV3:
    """Auto-select safe, disjoint course intervals and synthesize C2 excursions."""

    config.validate()
    if len(base_points) < 20:
        raise ValueError("base MPC Reference requires at least 20 points")
    s = np.asarray([point.s_m for point in base_points], dtype=np.float64)
    x = np.asarray([point.x_m for point in base_points], dtype=np.float64)
    y = np.asarray([point.y_m for point in base_points], dtype=np.float64)
    psi = np.asarray([point.psi_rad for point in base_points], dtype=np.float64)
    kappa = np.asarray([point.kappa_radpm for point in base_points], dtype=np.float64)
    total_needed = len(config.requests) * (config.segment_length_m + config.minimum_segment_gap_m)
    if total_needed >= float(s[-1]):
        raise ValueError("course is too short for requested disjoint recovery segments")

    output_x = x.copy()
    output_y = y.copy()
    selected: list[dict[str, Any]] = []
    phases: list[RecoveryPhaseV3] = []
    occupied_intervals: list[tuple[float, float]] = []
    for request in config.requests:
        candidates: list[tuple[float, int, int, float]] = []
        for start_index, start_s in enumerate(s):
            end_s = start_s + config.segment_length_m
            if end_s > s[-1]:
                break
            end_index = int(np.searchsorted(s, end_s, side="right") - 1)
            if end_index - start_index < 4:
                continue
            if any(
                _intervals_overlap(start_s, end_s, other_start, other_end, config.minimum_segment_gap_m)
                for other_start, other_end in occupied_intervals
            ):
                continue
            relative = s[start_index : end_index + 1] - start_s
            mean_curvature = float(np.mean(kappa[start_index : end_index + 1]))
            geometry = _geometry_name(mean_curvature, config.geometry_curvature_threshold_inv_m)
            if geometry != request.geometry:
                continue
            offset = _offset_profile(relative, request, config)
            candidate_x = x[start_index : end_index + 1] - np.sin(psi[start_index : end_index + 1]) * offset
            candidate_y = y[start_index : end_index + 1] + np.cos(psi[start_index : end_index + 1]) * offset
            clearance_x, clearance_y = _densify_polyline(
                candidate_x,
                candidate_y,
                maximum_step_m=occupancy.resolution_m_per_px,
            )
            if not occupancy.footprint_is_free(
                clearance_x,
                clearance_y,
                config.minimum_center_clearance_m,
            ):
                continue
            curvature_error = abs(
                abs(mean_curvature) - config.preferred_abs_curvature_inv_m
            )
            candidates.append((curvature_error, start_index, end_index, mean_curvature))
        if not candidates:
            raise ValueError(
                f"no safe, non-overlapping interval satisfies segment {request.segment_id!r}"
            )
        _, start_index, end_index, mean_curvature = min(candidates)
        start_s = float(s[start_index])
        end_s = start_s + config.segment_length_m
        relative = s[start_index : end_index + 1] - start_s
        offset = _offset_profile(relative, request, config)
        output_x[start_index : end_index + 1] = (
            x[start_index : end_index + 1] - np.sin(psi[start_index : end_index + 1]) * offset
        )
        output_y[start_index : end_index + 1] = (
            y[start_index : end_index + 1] + np.cos(psi[start_index : end_index + 1]) * offset
        )
        occupied_intervals.append((start_s, end_s))
        selected.append(
            {
                "segment_id": request.segment_id,
                "side": request.side,
                "offset_m": request.offset_m,
                "geometry": request.geometry,
                "mean_base_curvature_inv_m": mean_curvature,
                "start_point_id": start_index,
                "end_point_id": end_index,
                "base_start_s_m": start_s,
                "base_end_s_m": end_s,
            }
        )
        boundaries = (
            ("approach", 0.0, config.approach_length_m, False),
            (
                "hold",
                config.approach_length_m,
                config.approach_length_m + config.hold_length_m,
                True,
            ),
            (
                "recovery",
                config.approach_length_m + config.hold_length_m,
                config.segment_length_m,
                True,
            ),
        )
        for phase, relative_start, relative_end, eligible in boundaries:
            point_start = int(np.searchsorted(s, start_s + relative_start, side="left"))
            point_end = int(np.searchsorted(s, start_s + relative_end, side="right") - 1)
            phases.append(
                RecoveryPhaseV3(
                    segment_id=request.segment_id,
                    phase=phase,
                    side=request.side,
                    offset_m=request.offset_m,
                    geometry=request.geometry,
                    start_point_id=point_start,
                    end_point_id=point_end,
                    start_s_m=start_s + relative_start,
                    end_s_m=start_s + relative_end,
                    training_eligible=eligible,
                )
            )

    generated_s, generated_psi, generated_kappa = _recompute_geometry(output_x, output_y)
    phases = [
        RecoveryPhaseV3(
            segment_id=phase.segment_id,
            phase=phase.phase,
            side=phase.side,
            offset_m=phase.offset_m,
            geometry=phase.geometry,
            start_point_id=phase.start_point_id,
            end_point_id=phase.end_point_id,
            start_s_m=float(generated_s[phase.start_point_id]),
            end_s_m=float(generated_s[phase.end_point_id]),
            training_eligible=phase.training_eligible,
        )
        for phase in phases
    ]
    points = tuple(
        MpcReferencePointV3(
            s_m=float(generated_s[index]),
            x_m=float(output_x[index]),
            y_m=float(output_y[index]),
            psi_rad=float(generated_psi[index]),
            kappa_radpm=float(generated_kappa[index]),
            vx_mps=base_points[index].vx_mps,
            ax_mps2=base_points[index].ax_mps2,
        )
        for index in range(len(base_points))
    )
    return GeneratedRecoveryReferenceV3(
        points=points,
        phases=tuple(phases),
        selected_segments=tuple(selected),
        minimum_center_clearance_m=config.minimum_center_clearance_m,
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_generated_recovery_reference_v3(
    output_csv: str | Path,
    generated: GeneratedRecoveryReferenceV3,
    *,
    base_reference_path: str | Path,
    occupancy_map_yaml: str | Path,
    generator_config_path: str | Path,
) -> tuple[Path, Path]:
    """Write MPC CSV, training-eligibility intervals and a hash-bound manifest."""

    output = Path(output_csv)
    intervals = output.with_suffix(".intervals.csv")
    manifest = output.with_suffix(".manifest.yaml")
    collisions = [path for path in (output, intervals, manifest) if path.exists()]
    if collisions:
        raise FileExistsError(f"refusing to overwrite recovery Reference outputs: {collisions}")
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for point in generated.points:
            writer.writerow({name: f"{getattr(point, name):.9f}" for name in fieldnames})
    interval_fields = (
        "segment_id",
        "phase",
        "side",
        "offset_m",
        "geometry",
        "start_point_id",
        "end_point_id",
        "start_s_m",
        "end_s_m",
        "training_eligible",
    )
    with intervals.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=interval_fields)
        writer.writeheader()
        for phase in generated.phases:
            row = {name: getattr(phase, name) for name in interval_fields}
            row["training_eligible"] = str(phase.training_eligible).lower()
            writer.writerow(row)
    base_path = Path(base_reference_path)
    map_path = Path(occupancy_map_yaml)
    config_path = Path(generator_config_path)
    map_metadata = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    if not isinstance(map_metadata, dict) or "image" not in map_metadata:
        raise ValueError("occupancy map YAML must identify its image for provenance")
    map_image_path = map_path.parent / str(map_metadata["image"])
    payload = {
        "format_version": RECOVERY_REFERENCE_MANIFEST_FORMAT,
        "teacher_debug_only": True,
        "coordinate_frame": "map",
        "point_count": len(generated.points),
        "reference_csv": output.name,
        "reference_sha256": _sha256(output),
        "intervals_csv": intervals.name,
        "intervals_sha256": _sha256(intervals),
        "base_reference": str(base_path),
        "base_reference_sha256": _sha256(base_path),
        "occupancy_map_yaml": str(map_path),
        "occupancy_map_yaml_sha256": _sha256(map_path),
        "occupancy_map_image": str(map_image_path),
        "occupancy_map_image_sha256": _sha256(map_image_path),
        "generator_config": str(config_path),
        "generator_config_sha256": _sha256(config_path),
        "minimum_center_clearance_m": generated.minimum_center_clearance_m,
        "interval_axis": "generated_reference_s_m_and_point_id",
        "training_policy": {
            "approach": "exclude",
            "hold": "include_after_post-run_validation",
            "recovery": "include_after_post-run_validation",
        },
        "selected_segments": list(generated.selected_segments),
    }
    manifest.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return intervals, manifest


def manifest_summary_json(generated: GeneratedRecoveryReferenceV3) -> str:
    return json.dumps(
        {
            "point_count": len(generated.points),
            "phase_count": len(generated.phases),
            "selected_segments": list(generated.selected_segments),
        },
        ensure_ascii=False,
        indent=2,
    )


def render_official_mpc_recovery_config_v3(
    base_config_path: str | Path,
    output_config_path: str | Path,
    *,
    reference_container_path: str,
    package_share_container_path: str,
) -> Path:
    """Render an external MPC config without mutating the official package.

    The installed controller concatenates its package-share directory and the
    configured CSV path. A POSIX relative path therefore points safely to the
    read-only/staged Reference under ``/artifacts``.
    """

    source = Path(base_config_path)
    output = Path(output_config_path)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite MPC recovery config: {output}")
    if not reference_container_path.startswith("/") or not package_share_container_path.startswith("/"):
        raise ValueError("container Reference and package-share paths must be absolute POSIX paths")
    if not reference_container_path.endswith(".csv"):
        raise ValueError("container Reference path must name a CSV")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("official MPC base config must be a mapping")
    reference = raw.get("reference_path")
    mpc = raw.get("mpc")
    if not isinstance(reference, dict) or not isinstance(mpc, dict):
        raise ValueError("official MPC config requires reference_path and mpc mappings")
    relative = posixpath.relpath(
        posixpath.normpath(reference_container_path),
        posixpath.normpath(package_share_container_path),
    )
    reference["csv_path"] = relative
    reference["update_by_topic"] = False
    reference["circular"] = True
    reference["use_path_constraints_topic"] = False
    reference["use_border_cells_topic"] = False
    mpc["lateral_target_mode"] = "reference_path"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output
