"""Distance and speed budgets for recovery strokes with a nearby rear car."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionBudget:
    available_m: float
    speed_limit_mps: float
    tire_steer_rad: float
    stop: bool


def body_samples(length, width, spacing=0.15):
    nx, ny = math.ceil(length / spacing), math.ceil(width / spacing)
    return tuple((-length / 2 + length * ix / nx, -width / 2 + width * iy / ny)
                 for ix in range(nx + 1) for iy in range(ny + 1))


def motion_pose(ego, distance, tire_steer, wheelbase):
    curvature = math.tan(tire_steer) / wheelbase
    if abs(curvature) < 1e-8:
        return ego.x + distance * math.cos(ego.yaw), ego.y + distance * math.sin(ego.yaw), ego.yaw
    yaw = ego.yaw + curvature * distance
    # Pose is at the body/GNSS center, while the bicycle arc is at the rear axle.
    center = wheelbase / 2
    x = ego.x + (math.sin(yaw) - math.sin(ego.yaw)) / curvature
    y = ego.y + (math.cos(ego.yaw) - math.cos(yaw)) / curvature
    return x + center * (math.cos(yaw) - math.cos(ego.yaw)), y + center * (math.sin(yaw) - math.sin(ego.yaw)), yaw


def opponent_clearance(pose, opponents, length, width):
    """V2X has position only; enclose each other physical body by its circumcircle."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    radius = math.hypot(length, width) / 2
    result = math.inf
    for ox, oy in opponents:
        dx, dy = ox - x, oy - y
        along = max(0.0, abs(c * dx + s * dy) - length / 2)
        lateral = max(0.0, abs(-s * dx + c * dy) - width / 2)
        result = min(result, math.hypot(along, lateral) - radius)
    return result


def speed_for_stopping_distance(distance, deceleration, delay, pending_acceleration=0.0):
    # Pending propulsion can continue until the braking command reaches the car.
    offset = (deceleration + pending_acceleration) * delay
    pending_travel = 0.5 * pending_acceleration * delay ** 2 + (pending_acceleration * delay) ** 2 / (2 * deceleration)
    return max(0.0, math.sqrt(offset ** 2 +
                             2 * deceleration * max(0.0, distance - pending_travel)) - offset)


def motion_budget(ego, reference, opponents, config, *, reverse, tire_steer,
                  stroke_remaining, footprint):
    direction = -1 if reverse else 1
    limit = max(0.0, min(config.short_stroke_m, stroke_remaining))
    initial_pose = (ego.x, ego.y, ego.yaw)
    initial_wall = reference.occupied_body_samples(initial_pose, footprint)
    # An existing wall contact may be escaped without first requiring a free
    # initial footprint. The sampled stroke must never deepen that overlap.
    allowed_wall = initial_wall
    available = 0.0
    step = config.short_path_step_m
    for i in range(1, math.ceil(limit / step) + 1):
        distance = min(limit, i * step)
        # Include the straight response while the tire follows the new command.
        poses = [motion_pose(ego, direction * distance, steer, config.physical_wheelbase_m)
                 for steer in (tire_steer, 0.0)]
        wall = max(reference.occupied_body_samples(p, footprint) for p in poses)
        gap = min(opponent_clearance(p, opponents, config.body_length_m, config.body_width_m)
                  for p in poses)
        if wall > allowed_wall or gap < config.short_clearance_m:
            break
        allowed_wall = wall
        available = distance
    deceleration = min(config.acceleration_max_mps2, config.short_braking_deceleration_mps2)
    speed_limit = min(config.short_speed_mps, speed_for_stopping_distance(
        available, deceleration, config.short_response_delay_sec, config.short_acceleration_mps2))
    motion_speed = max(0.0, direction * ego.speed)
    delay = config.short_response_delay_sec
    pending_acceleration = config.short_acceleration_mps2
    stopping_distance = (motion_speed * delay + 0.5 * pending_acceleration * delay ** 2 +
                         (motion_speed + pending_acceleration * delay) ** 2 / (2 * deceleration))
    return MotionBudget(available, speed_limit, tire_steer,
                        available <= max(step, stopping_distance))
