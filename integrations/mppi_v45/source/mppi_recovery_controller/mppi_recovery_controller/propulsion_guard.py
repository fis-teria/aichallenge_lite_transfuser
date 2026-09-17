"""Direction-aware propulsion cutoff; independent of recovery arming."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PropulsionGuardConfig:
    no_progress_timeout_sec: float = 0.4
    minimum_progress_m: float = 0.02

    def __post_init__(self):
        for value in (self.no_progress_timeout_sec, self.minimum_progress_m):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("propulsion cutoff limits must be finite and positive")


class PropulsionGuard:
    def __init__(self, config=PropulsionGuardConfig()):
        self.config = config
        self.reset()

    def reset(self):
        self.blocked_directions = set()
        self.exposure = {1: 0.0, -1: 0.0}
        self.anchor = None
        self.direction = 0
        self.last_time = None
        self.last_propelling = False

    def update(self, now, direction, x, y, yaw, propelling):
        if not all(math.isfinite(v) for v in (now, x, y, yaw)):
            self.last_propelling = False
            return True
        if self.last_time is not None and now < self.last_time:
            # A clock reset cannot silently re-enable a blocked direction.
            self.last_time = now
            self.last_propelling = False
            self.anchor = None
        if self.last_time is not None and self.last_propelling:
            self.exposure[self.direction] += max(0.0, now - self.last_time)
            if self.exposure[self.direction] + 1e-9 >= self.config.no_progress_timeout_sec:
                self.blocked_directions.add(self.direction)
        if direction not in (-1, 1):
            self.last_time = now
            self.last_propelling = False
            return True
        if direction != self.direction or self.anchor is None:
            self.anchor = (x, y, yaw)
        else:
            ax, ay, heading = self.anchor
            progress = direction * ((x - ax) * math.cos(heading) + (y - ay) * math.sin(heading))
            if progress >= self.config.minimum_progress_m:
                # An actual escape permits a new attempt, including a return
                # to DRIVE. Merely toggling commands or gears does not.
                self.blocked_directions.clear()
                self.exposure = {1: 0.0, -1: 0.0}
                self.anchor = (x, y, yaw)
        self.direction = direction
        self.last_time = now
        cut = direction in self.blocked_directions
        self.last_propelling = bool(propelling) and not cut
        return cut
