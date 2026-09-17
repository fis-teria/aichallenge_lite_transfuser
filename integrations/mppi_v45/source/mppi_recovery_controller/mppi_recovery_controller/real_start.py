"""Arm real-vehicle recovery only after commanded autonomous forward motion."""

from dataclasses import dataclass
import math


@dataclass
class RealVehicleRecoveryGate:
    minimum_command_speed: float
    stopped_speed_threshold: float
    input_timeout_ns: int
    autonomous: bool | None = None
    drive_confirmed: bool = False
    armed: bool = False
    transition_ns: int = 0
    first_forward_command_ns: int | None = None
    latest_forward_command_ns: int | None = None

    @property
    def can_drive(self):
        return self.autonomous is True and self.drive_confirmed

    def set_autonomous(self, autonomous, now_ns):
        if self.autonomous == autonomous:
            return False
        self.autonomous = autonomous
        self.transition_ns = now_ns
        self.drive_confirmed = self.armed = False
        self.first_forward_command_ns = self.latest_forward_command_ns = None
        return True

    def observe_gear(self, is_drive, stamp_ns):
        # A queued report from before the DRIVE request is not its acknowledgement.
        if self.autonomous and stamp_ns >= self.transition_ns:
            self.drive_confirmed = is_drive

    def forwarded_command(self, speed, stamp_ns):
        if not self.can_drive or stamp_ns < self.transition_ns:
            return
        if not math.isfinite(speed) or speed < self.minimum_command_speed:
            self.first_forward_command_ns = self.latest_forward_command_ns = None
            return
        if self.first_forward_command_ns is None:
            self.first_forward_command_ns = stamp_ns
        self.latest_forward_command_ns = stamp_ns

    def observe_speed(self, speed, stamp_ns, now_ns):
        if self.armed or not self.can_drive or self.first_forward_command_ns is None:
            return False
        if (math.isfinite(speed) and speed > self.stopped_speed_threshold and
                stamp_ns > self.first_forward_command_ns and
                0 <= now_ns - stamp_ns <= self.input_timeout_ns and
                0 <= now_ns - self.latest_forward_command_ns <= self.input_timeout_ns):
            self.armed = True
            return True
        return False
