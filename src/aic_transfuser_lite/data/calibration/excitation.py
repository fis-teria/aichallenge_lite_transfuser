from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

import yaml


EXCITATION_PLAN_FORMAT_V1 = "aic_calibration_excitation_v1"
EXCITATION_MODES = frozenset({"settle", "steering", "drive", "brake", "stop"})

# These are deliberately conservative collection guards, not promoted vehicle limits.
MAX_COLLECTION_STEERING_RAD = 0.25
MAX_COLLECTION_SPEED_MPS = 3.0
MIN_COLLECTION_ACCELERATION_MPS2 = -2.0
MAX_COLLECTION_ACCELERATION_MPS2 = 1.0
MAX_COLLECTION_DURATION_SEC = 180.0
_SAFE_ID = re.compile(r"^[A-Za-z0-9_-]+$")


@dataclass(frozen=True)
class ExcitationCommand:
    steering_rad: float
    speed_mps: float
    acceleration_mps2: float

    def validate(self) -> None:
        values = (self.steering_rad, self.speed_mps, self.acceleration_mps2)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("excitation command values must be finite")
        if abs(self.steering_rad) > MAX_COLLECTION_STEERING_RAD:
            raise ValueError("steering exceeds conservative collection guard")
        if not 0.0 <= self.speed_mps <= MAX_COLLECTION_SPEED_MPS:
            raise ValueError("speed exceeds conservative collection guard")
        if not (
            MIN_COLLECTION_ACCELERATION_MPS2
            <= self.acceleration_mps2
            <= MAX_COLLECTION_ACCELERATION_MPS2
        ):
            raise ValueError("acceleration exceeds conservative collection guard")


@dataclass(frozen=True)
class ExcitationSegment:
    segment_id: str
    mode: str
    duration_sec: float
    command: ExcitationCommand

    def validate(self) -> None:
        if not _SAFE_ID.fullmatch(self.segment_id):
            raise ValueError("segment_id must use letters, digits, '-' or '_'")
        if self.mode not in EXCITATION_MODES:
            raise ValueError(f"unsupported excitation mode: {self.mode!r}")
        if not math.isfinite(self.duration_sec) or self.duration_sec < 0.5:
            raise ValueError("segment duration_sec must be finite and at least 0.5 s")
        self.command.validate()


@dataclass(frozen=True)
class ExcitationPlan:
    plan_id: str
    target_mode: str
    publish_hz: float
    telemetry_timeout_sec: float
    preflight_hold_sec: float
    stop_speed_threshold_mps: float
    stop_hold_sec: float
    max_observed_speed_mps: float
    segments: tuple[ExcitationSegment, ...]
    format_version: str = EXCITATION_PLAN_FORMAT_V1

    @property
    def total_duration_sec(self) -> float:
        return sum(segment.duration_sec for segment in self.segments)

    def validate(self) -> None:
        if self.format_version != EXCITATION_PLAN_FORMAT_V1:
            raise ValueError(f"unsupported excitation plan format: {self.format_version!r}")
        if not _SAFE_ID.fullmatch(self.plan_id):
            raise ValueError("plan_id must use letters, digits, '-' or '_'")
        if self.target_mode not in {"steering", "drive", "brake"}:
            raise ValueError("target_mode must be steering, drive, or brake")
        scalars = (
            self.publish_hz,
            self.telemetry_timeout_sec,
            self.preflight_hold_sec,
            self.stop_speed_threshold_mps,
            self.stop_hold_sec,
            self.max_observed_speed_mps,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in scalars):
            raise ValueError("plan timing, rate, speed thresholds must be finite and positive")
        if not 10.0 <= self.publish_hz <= 100.0:
            raise ValueError("publish_hz must be in [10, 100]")
        if self.stop_speed_threshold_mps > 0.25:
            raise ValueError("stop_speed_threshold_mps must not exceed 0.25 m/s")
        if self.max_observed_speed_mps > MAX_COLLECTION_SPEED_MPS:
            raise ValueError("max_observed_speed_mps exceeds collection guard")
        if not self.segments:
            raise ValueError("excitation plan requires segments")
        ids: set[str] = set()
        for segment in self.segments:
            segment.validate()
            if segment.segment_id in ids:
                raise ValueError(f"duplicate segment_id: {segment.segment_id!r}")
            ids.add(segment.segment_id)
            if segment.command.speed_mps > self.max_observed_speed_mps:
                raise ValueError("segment speed exceeds max_observed_speed_mps")
        if self.total_duration_sec > MAX_COLLECTION_DURATION_SEC:
            raise ValueError("excitation plan exceeds maximum collection duration")
        first = self.segments[0]
        last = self.segments[-1]
        if first.mode != "settle" or not _is_stop_command(first.command):
            raise ValueError("first segment must be a stationary settle command")
        if (
            last.mode != "stop"
            or not _is_stop_command(last.command)
            or last.duration_sec < self.stop_hold_sec
        ):
            raise ValueError("last segment must hold a stop command for stop_hold_sec")
        if not _contains_target_excitation(self):
            raise ValueError(f"plan lacks meaningful {self.target_mode} excitation")

    def command_at(self, elapsed_sec: float) -> tuple[ExcitationSegment, float]:
        """Return the active segment and elapsed time in it for ``elapsed_sec``."""

        if not math.isfinite(elapsed_sec) or elapsed_sec < 0.0:
            raise ValueError("elapsed_sec must be finite and non-negative")
        cursor = 0.0
        for segment in self.segments:
            end = cursor + segment.duration_sec
            if elapsed_sec < end:
                return segment, elapsed_sec - cursor
            cursor = end
        return self.segments[-1], self.segments[-1].duration_sec


def load_excitation_plan(path: str | Path) -> ExcitationPlan:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"excitation plan not found: {source}")
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("excitation plan root must be a mapping")
    allowed = {
        "format_version",
        "plan_id",
        "target_mode",
        "publish_hz",
        "telemetry_timeout_sec",
        "preflight_hold_sec",
        "stop_speed_threshold_mps",
        "stop_hold_sec",
        "max_observed_speed_mps",
        "segments",
    }
    unknown = sorted(set(raw).difference(allowed))
    if unknown:
        raise ValueError(f"unknown excitation plan fields: {unknown}")
    segment_values = raw.get("segments")
    if not isinstance(segment_values, list):
        raise ValueError("excitation plan segments must be a list")
    segments = tuple(_parse_segment(value) for value in segment_values)
    plan = ExcitationPlan(
        format_version=str(raw.get("format_version", "")),
        plan_id=str(raw.get("plan_id", "")),
        target_mode=str(raw.get("target_mode", "")),
        publish_hz=float(raw.get("publish_hz", 0.0)),
        telemetry_timeout_sec=float(raw.get("telemetry_timeout_sec", 0.0)),
        preflight_hold_sec=float(raw.get("preflight_hold_sec", 0.0)),
        stop_speed_threshold_mps=float(raw.get("stop_speed_threshold_mps", 0.0)),
        stop_hold_sec=float(raw.get("stop_hold_sec", 0.0)),
        max_observed_speed_mps=float(raw.get("max_observed_speed_mps", 0.0)),
        segments=segments,
    )
    plan.validate()
    return plan


def excitation_plan_sha256(plan: ExcitationPlan) -> str:
    plan.validate()
    payload = {
        "format_version": plan.format_version,
        "plan_id": plan.plan_id,
        "target_mode": plan.target_mode,
        "publish_hz": plan.publish_hz,
        "telemetry_timeout_sec": plan.telemetry_timeout_sec,
        "preflight_hold_sec": plan.preflight_hold_sec,
        "stop_speed_threshold_mps": plan.stop_speed_threshold_mps,
        "stop_hold_sec": plan.stop_hold_sec,
        "max_observed_speed_mps": plan.max_observed_speed_mps,
        "segments": [
            {
                "segment_id": segment.segment_id,
                "mode": segment.mode,
                "duration_sec": segment.duration_sec,
                "steering_rad": segment.command.steering_rad,
                "speed_mps": segment.command.speed_mps,
                "acceleration_mps2": segment.command.acceleration_mps2,
            }
            for segment in plan.segments
        ],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def runtime_guard_reasons(
    *,
    speed_mps: float | None,
    telemetry_age_sec: float | None,
    safety_reason: str | None,
    nominal_publisher_count: int,
    final_publisher_count: int,
    nominal_subscriber_count: int,
    final_subscriber_count: int,
    plan: ExcitationPlan,
) -> tuple[str, ...]:
    """Return every fail-closed runtime guard violation before/during excitation."""

    reasons: list[str] = []
    if nominal_publisher_count != 1:
        reasons.append(f"nominal_publisher_count={nominal_publisher_count}")
    if final_publisher_count != 1:
        reasons.append(f"final_publisher_count={final_publisher_count}")
    if nominal_subscriber_count < 1:
        reasons.append("nominal_subscriber_missing")
    if final_subscriber_count < 1:
        reasons.append("final_subscriber_missing")
    if speed_mps is None or not math.isfinite(speed_mps):
        reasons.append("velocity_missing_or_non_finite")
    elif abs(speed_mps) > plan.max_observed_speed_mps:
        reasons.append("observed_speed_exceeds_plan_limit")
    if telemetry_age_sec is None or not math.isfinite(telemetry_age_sec):
        reasons.append("velocity_timestamp_missing")
    elif telemetry_age_sec < 0.0 or telemetry_age_sec > plan.telemetry_timeout_sec:
        reasons.append("velocity_telemetry_stale")
    if safety_reason not in {"normal", "command_clamped"}:
        reasons.append(f"safety_not_ready:{safety_reason or 'missing'}")
    return tuple(reasons)


def _parse_segment(raw: Any) -> ExcitationSegment:
    if not isinstance(raw, Mapping):
        raise ValueError("each excitation segment must be a mapping")
    allowed = {
        "segment_id",
        "mode",
        "duration_sec",
        "steering_rad",
        "speed_mps",
        "acceleration_mps2",
    }
    unknown = sorted(set(raw).difference(allowed))
    if unknown:
        raise ValueError(f"unknown excitation segment fields: {unknown}")
    return ExcitationSegment(
        segment_id=str(raw.get("segment_id", "")),
        mode=str(raw.get("mode", "")),
        duration_sec=float(raw.get("duration_sec", 0.0)),
        command=ExcitationCommand(
            steering_rad=float(raw.get("steering_rad", math.nan)),
            speed_mps=float(raw.get("speed_mps", math.nan)),
            acceleration_mps2=float(raw.get("acceleration_mps2", math.nan)),
        ),
    )


def _is_stop_command(command: ExcitationCommand) -> bool:
    return (
        command.speed_mps == 0.0
        and command.steering_rad == 0.0
        and command.acceleration_mps2 < 0.0
    )


def _contains_target_excitation(plan: ExcitationPlan) -> bool:
    commands = [
        segment.command for segment in plan.segments if segment.mode == plan.target_mode
    ]
    if plan.target_mode == "steering":
        return (
            any(command.steering_rad >= 0.02 for command in commands)
            and any(command.steering_rad <= -0.02 for command in commands)
        )
    if plan.target_mode == "drive":
        return any(command.acceleration_mps2 >= 0.1 for command in commands)
    return any(command.acceleration_mps2 <= -0.1 for command in commands)
