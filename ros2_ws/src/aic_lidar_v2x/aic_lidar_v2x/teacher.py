"""Collection integration policy; collision geometry remains owned by V44."""
from __future__ import annotations

from dataclasses import dataclass
import math


V2X_TOPIC = "/collection/lidar_v2x/vehicle_positions"
# Match the original subscription names as well as absolute subscriptions.
# ROS remapping is not a recursive native-topic -> replacement-topic chain.
V44_INPUT_REMAPS = (
    ("input/vehicle_positions", V2X_TOPIC),
    ("input/vehicles", V2X_TOPIC),
    ("/v2x/vehicle_positions", V2X_TOPIC),
)


@dataclass
class InputState:
    mode: str
    started_wall_s: float
    timeout_s: float = 0.75
    startup_timeout_s: float = 15.0
    last_valid_wall_s: float | None = None
    stop_requested: bool = False

    def __post_init__(self) -> None:
        if self.mode not in {"shadow", "teacher_existing_margin"}:
            raise ValueError("mode must be shadow or teacher_existing_margin")
        if not math.isfinite(self.started_wall_s):
            raise ValueError("Watchdog clock must be finite")
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout_s, self.startup_timeout_s)):
            raise ValueError("Input timeouts must be positive seconds")

    def mark_valid(self, wall_s: float) -> None:
        if not math.isfinite(wall_s):
            raise ValueError("Watchdog clock must be finite")
        self.last_valid_wall_s = wall_s

    def check_timeout(self, wall_s: float) -> bool:
        """Latch a sensor/TF failure; never define a new obstacle stop region."""
        if not math.isfinite(wall_s):
            raise ValueError("Watchdog clock must be finite")
        reference = self.started_wall_s if self.last_valid_wall_s is None else self.last_valid_wall_s
        timeout = self.startup_timeout_s if self.last_valid_wall_s is None else self.timeout_s
        if self.mode == "teacher_existing_margin" and wall_s - reference > timeout:
            self.stop_requested = True
        return self.stop_requested

    def metadata(self, input_valid: bool) -> dict:
        enabled = self.mode == "teacher_existing_margin"
        return dict(mode=self.mode, teacher_input_enabled=enabled,
                    teacher_ready=enabled and input_valid and not self.stop_requested,
                    stop_requested=self.stop_requested,
                    role="TEACHER_INPUT_EXISTING_V44_MARGIN" if enabled else "SHADOW_UNVALIDATED_FOR_CONTROL",
                    collision_geometry="V44_UNCHANGED" if enabled else "NOT_CONNECTED")
