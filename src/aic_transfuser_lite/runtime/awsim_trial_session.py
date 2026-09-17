"""ROS-free finite trial schedule and the unchanged AWSIM LapCount log judge."""
from __future__ import annotations

import math
import re
from typing import Any, Iterable


def validate_npc_count(count: int) -> int:
    """Built-in AWSIM supports one ego vehicle plus zero to three NPCs."""
    if type(count) is not int or not 0 <= count <= 3:
        raise ValueError("NPC_COUNT_MUST_BE_INTEGER_0_TO_3")
    return count


def npc_startup_evidence(log_text: str, requested_count: int) -> dict[str, Any]:
    """Check actual NPC creation and collisions before authorizing ego motion.

    This only verifies startup, not NPC speed, subsequent contact or avoidance.
    """
    validate_npc_count(requested_count)
    spawns = re.findall(
        r"NpcRuntimeManager: spawned (\d+) NPC kart\(s\) named C1\.\.C(\d+) in ([a-z-]+) mode\.",
        log_text,
    )
    if requested_count == 0:
        if spawns:
            raise ValueError("UNEXPECTED_NPC_SPAWN")
        return {"requested_count": 0, "spawned_count": 0, "mode": None}
    if spawns != [(str(requested_count), str(requested_count), "racing-line")]:
        raise ValueError("NPC_SPAWN_EVIDENCE_MISMATCH")
    settings = re.findall(r"^Applied race settings:.*$", log_text, re.MULTILINE)
    if not settings or not re.search(r"\bcollisions=True(?:,|\s|$)", settings[-1]):
        raise ValueError("NPC_VEHICLE_COLLISIONS_NOT_ENABLED")
    return {"requested_count": requested_count, "spawned_count": requested_count,
            "mode": "racing-line", "vehicle_collisions_enabled": True,
            "settings_log": settings[-1].strip()}


def trial_duration_limits(profile: str = "bounded_10s") -> tuple[float, float, float]:
    """(drive simulation seconds, drive wall seconds, total wall seconds)."""
    if profile == "bounded_10s":
        return 10., 30., 120.
    if profile == "one_lap":
        return 600., 600., 720.
    raise ValueError("TRIAL_EXECUTION_PROFILE")


def encode_scan_values(values: Iterable[float]) -> list[float | str]:
    """Lossless JSON-safe diagnostic values; keep NaN/+inf/-inf distinct."""
    return [float(v) if math.isfinite(float(v)) else str(float(v)) for v in values]


def requested_stop(value: dict[str, Any], run_id: str) -> str:
    """Only a stop request, never an authorization to produce motion."""
    reasons = {"JUDGE_FIRST_LAP", "JUDGE_LAP_EVIDENCE_INCOMPLETE", "PROGRESS_STALLED", "OPERATOR_STOP",
               "SLAM_OBSTACLE_STOP_CONFIRMED", "SLAM_BOX_TEST_TIME_LIMIT"}
    if value.get("run_id") != run_id or value.get("reason") not in reasons:
        raise ValueError("STOP_REQUEST_IDENTITY")
    return value["reason"]


def trial_brake_reason(profile: str, elapsed_sim_s: float, elapsed_wall_s: float,
                       stop_requested: bool = False) -> str | None:
    """Check both clocks independently; an explicit stop takes precedence."""
    drive_sim_s, drive_wall_s, _ = trial_duration_limits(profile)
    if not all(math.isfinite(t) and t >= 0 for t in (elapsed_sim_s, elapsed_wall_s)):
        raise ValueError("TRIAL_ELAPSED_TIME_INVALID")
    if stop_requested:
        return "REQUESTED_BRAKE"
    if elapsed_sim_s >= drive_sim_s or elapsed_wall_s >= drive_wall_s:
        return "SCHEDULED_BRAKE"
    return None


class JudgeLog:
    """Actual unchanged LapCount logs, never distance/proximity as a lap."""
    def __init__(self, run_id: str = "SYNTHETIC"):
        self.run_id = run_id
        self.section_events = []
        self.laps = []
        self.invalid = False
        self.previous_lap_count = 0

    def feed(self, line: str, byte_offset: int | None = None) -> None:
        hit = re.search(r"Section line hit: current=(-?\d+), next=(\d+), started=(\w+)", line)
        if hit:
            previous, next_section = int(hit[1]), int(hit[2])
            if self.section_events:
                prior = self.section_events[-1]["next"]
                if previous != prior or (next_section != prior+1 and next_section != 0) or hit[3] != "True":
                    self.invalid = True
            elif next_section != 0 or hit[3] != "False":
                self.invalid = True
            self.section_events.append(dict(current=previous, next=next_section, started=hit[3], line=line,
                run_id=self.run_id, epoch=0, byte_offset=byte_offset))
        lap = re.search(r"Lap completed: ([\d.]+)s, total laps: (\d+)", line)
        if lap:
            count = int(lap[2])
            increment_valid = count == self.previous_lap_count+1
            self.laps.append(dict(lap_seconds=float(lap[1]), laps=count, line=line,
                run_id=self.run_id, epoch=0, byte_offset=byte_offset, lap_increment_valid=increment_valid,
                ordered_section_evidence=not self.invalid and increment_valid and len(self.section_events) >= 4
                    and self.section_events[0]["next"] == 0 and self.section_events[-1]["next"] == 0))
            self.previous_lap_count = count

    @property
    def completed(self) -> bool:
        return bool(self.laps and self.laps[-1]["laps"] >= 1 and self.laps[-1]["ordered_section_evidence"])


class LowSpeedStall:
    """Stop an armed lap attempt after 5 simulation seconds below 0.1 m/s."""
    def __init__(self) -> None:
        self.since_ns: int | None = None

    def update(self, sim_ns: int, speed_mps: float) -> bool:
        if type(sim_ns) is not int or not math.isfinite(speed_mps):
            raise ValueError("STALL_STATE_INVALID")
        if abs(speed_mps) >= .1:
            self.since_ns = None
            return False
        if self.since_ns is None or sim_ns < self.since_ns:
            self.since_ns = sim_ns
        return sim_ns - self.since_ns >= 5_000_000_000
