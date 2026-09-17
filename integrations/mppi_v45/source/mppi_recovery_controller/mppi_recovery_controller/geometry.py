"""Adapt MPPI's global reference and physical occupancy map to MPC bounds."""

import math
from array import array
from dataclasses import dataclass

from .core import ReferenceGeometry, ReferencePoint


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def reference_samples(message):
    return tuple((float(p.pose.position.x), float(p.pose.position.y),
                  yaw_from_quaternion(p.pose.orientation),
                  float(p.longitudinal_velocity_mps)) for p in message.points)


@dataclass(frozen=True)
class WallGrid:
    """Immutable, picklable occupancy snapshot shared with the build worker."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    cosine: float
    sine: float
    cells: bytes
    frame_id: str

    def is_free(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        col = math.floor((self.cosine * dx + self.sine * dy) / self.resolution)
        row = math.floor((-self.sine * dx + self.cosine * dy) / self.resolution)
        # Unknown (-1 in OccupancyGrid) is 255 in this byte representation.
        return (0 <= col < self.width and 0 <= row < self.height
                and self.cells[row * self.width + col] < 50)


def wall_snapshot(grid):
    if isinstance(grid, WallGrid):
        return grid
    info = grid.info
    angle = yaw_from_quaternion(info.origin.orientation)
    return WallGrid(float(info.resolution), int(info.width), int(info.height),
                    info.origin.position.x, info.origin.position.y,
                    math.cos(angle), math.sin(angle), array('b', grid.data).tobytes(),
                    getattr(getattr(grid, 'header', None), 'frame_id', ''))


def build_reference(samples, grid, max_width):
    """Build the same left/right scalar corridor contract used by MPC.

    Bounds come from the MPPI physical map, not MPC's older map asset. Wall
    lookup is only needed when the reference/map changes, not on control ticks.
    """
    wall = wall_snapshot(grid)
    if (len(samples) < 3 or wall.resolution <= 0 or wall.width <= 0
            or wall.height <= 0 or len(wall.cells) != wall.width * wall.height):
        raise ValueError("recovery requires a complete reference and occupancy map")
    if not all(math.isfinite(value) for point in samples for value in point):
        raise ValueError("recovery reference must be finite")
    if math.hypot(samples[0][0] - samples[-1][0], samples[0][1] - samples[-1][1]) < 1e-6:
        samples = samples[:-1]
    free = wall.is_free
    steps = max(1, math.ceil(max_width / (0.5 * wall.resolution)))

    def width(x, y, direction):
        for i in range(steps + 1):
            distance = max_width * i / steps
            if not free(x + distance * math.cos(direction), y + distance * math.sin(direction)):
                return distance
        return max_width

    closed = math.hypot(samples[0][0] - samples[-1][0], samples[0][1] - samples[-1][1]) < 5.0
    points = []
    for i, (x, y, _, speed) in enumerate(samples):
        # Match MPPI's segment tangent. Reference CSV orientations can use a
        # different convention, so they cannot define lateral wall bounds.
        previous = samples[(i - 1) % len(samples) if closed else max(0, i - 1)]
        following = samples[(i + 1) % len(samples) if closed else min(len(samples) - 1, i + 1)]
        yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        points.append(ReferencePoint(x, y, yaw, speed,
                                    -width(x, y, yaw - math.pi / 2),
                                    width(x, y, yaw + math.pi / 2)))
    return ReferenceGeometry(points, closed=closed, is_free=free)
