"""Offline fixed-line speed estimates; not a controller or tire calibration.

All path arrays have shape [N], one value per point on a closed lap. Distance
is metres, time seconds, angle radians, and acceleration metres/second squared.
The tire budget is an explicitly assumed friction circle. AWSIM also applies
an anti-skid force, so this model is not an exact reconstruction of its physics.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math

import numpy as np


@dataclass(frozen=True)
class KartLimits:
    speed_cap_mps: float = 35.0 / 3.6
    wheelbase_m: float = 1.087
    tire_angle_rad: float = math.pi / 10
    tire_rate_radps: float = math.pi / 3
    steering_time_constant_s: float = 0.02
    drive_cap_mps2: float = 1.37
    drive_command_cap_mps2: float = 2.0
    brake_command_cap_mps2: float = 2.0
    rolling_resistance_mps2: float = 0.37
    linear_drag_per_s: float = 0.03
    drive_fade_speed_mps: float = 10.0
    drive_fade_exponent: float = 40.0

    def validate(self) -> None:
        nonnegative = {'steering_time_constant_s', 'rolling_resistance_mps2', 'linear_drag_per_s'}
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or (value < 0 if field.name in nonnegative else value <= 0):
                raise ValueError(f'Invalid {field.name}')
        if self.tire_angle_rad >= math.pi / 2:
            raise ValueError('tire_angle_rad must be below pi/2')
        if self.drive_cap_mps2 <= self.rolling_resistance_mps2:
            raise ValueError('Drive cap must exceed rolling resistance')


def drive_cap(speed_mps: float, limits: KartLimits) -> float:
    """AWSIM DriveFadeMaxAcceleration, before rolling resistance and drag."""
    fraction = min(abs(speed_mps) / limits.drive_fade_speed_mps, 1.0)
    faded = limits.rolling_resistance_mps2 + (
        limits.drive_cap_mps2 - limits.rolling_resistance_mps2
    ) * (1 - fraction ** limits.drive_fade_exponent)
    return min(faded, limits.drive_command_cap_mps2)


def resistance(speed_mps: float, limits: KartLimits) -> float:
    """Continuous approximation to rolling resistance and Unity linear drag."""
    return limits.rolling_resistance_mps2 + limits.linear_drag_per_s * speed_mps


def _tire_longitudinal(speed_squared: float, curvature_abs: float, ay_mps2: float) -> float:
    return math.sqrt(max(0.0, ay_mps2 * ay_mps2 - (speed_squared * curvature_abs) ** 2))


def _root_upper(function, upper: float, iterations: int = 42) -> float:
    """Largest feasible nonnegative squared speed for an increasing residual."""
    if function(upper) <= 0:
        return upper
    if function(0.0) > 1e-10:
        raise ValueError('No feasible nonnegative speed under the supplied constraints')
    low, high = 0.0, upper
    for _ in range(iterations):
        middle = (low + high) * 0.5
        if function(middle) <= 0:
            low = middle
        else:
            high = middle
    return low


def estimate_closed_profile(
    curvature_1pm: np.ndarray,
    segment_lengths_m: np.ndarray,
    limits: KartLimits,
    lateral_limit_mps2: float,
    *,
    convergence_mps: float = 1e-7,
    max_passes: int = 250,
) -> dict:
    """Estimate a periodic speed envelope on a steering-feasible fixed line.

    Inputs are finite [N>=8] arrays; segment i joins point i to (i+1) modulo N.
    Forward/backward relaxation propagates acceleration and braking constraints
    across the lap seam. A segment uses its largest endpoint curvature and speed
    for the tire budget; this is conservative within the discrete model.
    The known curve permits anticipatory steering. Tracking errors, load transfer,
    yaw transients, axle slip and the intermittent acceleration hold are not modeled.
    """
    limits.validate()
    k = np.asarray(curvature_1pm, dtype=float)
    ds = np.asarray(segment_lengths_m, dtype=float)
    if k.ndim != 1 or len(k) < 8 or ds.shape != k.shape:
        raise ValueError('Curvature and segment lengths must have matching shape [N>=8]')
    if not np.isfinite(k).all() or not np.isfinite(ds).all() or np.any(ds <= 0):
        raise ValueError('Curvature must be finite and segment lengths finite and positive')
    if not math.isfinite(lateral_limit_mps2) or lateral_limit_mps2 <= limits.rolling_resistance_mps2:
        raise ValueError('lateral_limit_mps2 must exceed rolling resistance')
    if not math.isfinite(convergence_mps) or convergence_mps <= 0 or max_passes < 1:
        raise ValueError('Positive convergence tolerance and pass count are required')
    delta = np.arctan(limits.wheelbase_m * k)
    if np.any(np.abs(delta) > limits.tire_angle_rad + 1e-8):
        raise ValueError('Path exceeds the tire angle limit; slowing down cannot fix this geometry')
    delta_gradient = np.abs(np.roll(delta, -1) - delta) / ds
    node_gradient = np.maximum(delta_gradient, np.roll(delta_gradient, 1))
    steering_cap = np.full(len(k), limits.speed_cap_mps)
    changing = node_gradient > 1e-10
    steering_cap[changing] = np.minimum(steering_cap[changing], limits.tire_rate_radps / node_gradient[changing])
    if limits.steering_time_constant_s:
        # A sufficient bound for first-order inverse steering feedforward.
        margin = np.maximum(0.0, limits.tire_angle_rad - np.abs(delta))
        steering_cap[changing] = np.minimum(
            steering_cap[changing], margin[changing] / (limits.steering_time_constant_s * node_gradient[changing]))
    q = np.empty(len(k))
    for i, (curvature, upper) in enumerate(zip(np.abs(k), steering_cap ** 2)):
        # Steady motion still requires tire force to overcome losses.
        def steady_residual(value: float) -> float:
            speed = math.sqrt(value)
            losses = resistance(speed, limits)
            return max(math.hypot(value * curvature, losses) - lateral_limit_mps2,
                       losses - drive_cap(speed, limits))
        q[i] = _root_upper(steady_residual, float(upper))
    # An edge uses both endpoint curvatures, so both speeds inherit both caps.
    q = np.minimum(q, np.minimum(np.roll(q, 1), np.roll(q, -1)))
    local_cap = np.sqrt(q.copy())
    edge_k = np.maximum(np.abs(k), np.abs(np.roll(k, -1)))
    for pass_number in range(1, max_passes + 1):
        previous = np.sqrt(q.copy())
        for i in range(len(q)):
            j = (i + 1) % len(q)
            if q[j] <= q[i]:
                continue
            def forward_residual(value: float) -> float:
                peak_q = max(float(q[i]), value)
                speed = math.sqrt(peak_q)
                force = min(drive_cap(speed, limits), _tire_longitudinal(peak_q, edge_k[i], lateral_limit_mps2))
                available = force - resistance(speed, limits)
                return value - q[i] - 2 * ds[i] * available
            q[j] = min(q[j], _root_upper(forward_residual, float(q[j])))
        for i in range(len(q) - 1, -1, -1):
            j = (i + 1) % len(q)
            if q[i] <= q[j]:
                continue
            def backward_residual(value: float) -> float:
                peak_q = max(value, float(q[j]))
                speed = math.sqrt(peak_q)
                braking = min(limits.brake_command_cap_mps2,
                              _tire_longitudinal(peak_q, edge_k[i], lateral_limit_mps2))
                available = braking + resistance(math.sqrt(float(q[j])), limits)
                return value - q[j] - 2 * ds[i] * available
            q[i] = min(q[i], _root_upper(backward_residual, float(q[i])))
        if np.max(np.abs(np.sqrt(q) - previous)) <= convergence_mps:
            break
    else:
        raise RuntimeError('Periodic speed estimate did not converge')
    speed = np.sqrt(q)
    if np.any(speed <= 1e-6):
        raise ValueError('The geometry requires a stop; a rolling-lap estimate is undefined')
    acceleration = (np.roll(q, -1) - q) / (2 * ds)
    residuals = profile_residuals(k, ds, speed, limits, lateral_limit_mps2)
    if max(residuals.values()) > 2e-5:
        raise RuntimeError(f'Estimated profile failed constraint verification: {residuals}')
    return {
        'speed_mps': speed, 'local_cap_mps': local_cap, 'tire_angle_rad': delta,
        'longitudinal_acceleration_mps2': acceleration,
        'lateral_acceleration_mps2': q * k,
        'lap_time_s': float(np.sum(2 * ds / (speed + np.roll(speed, -1)))),
        'passes': pass_number, 'constraint_residuals': residuals,
    }


def profile_residuals(k: np.ndarray, ds: np.ndarray, speed: np.ndarray,
                      limits: KartLimits, ay_mps2: float) -> dict[str, float]:
    """Independently recompute positive constraint violations on every edge."""
    q = speed ** 2
    peak_q = np.maximum(q, np.roll(q, -1))
    peak_k = np.maximum(np.abs(k), np.abs(np.roll(k, -1)))
    acceleration = (np.roll(q, -1) - q) / (2 * ds)
    lateral = peak_q * peak_k
    loss_q = np.where(acceleration >= 0, peak_q, np.minimum(q, np.roll(q, -1)))
    forces = acceleration + np.array([resistance(math.sqrt(v), limits) for v in loss_q])
    drive = np.array([drive_cap(math.sqrt(v), limits) for v in peak_q])
    delta = np.arctan(limits.wheelbase_m * k)
    rate = np.abs(np.roll(delta, -1) - delta) * np.sqrt(peak_q) / ds
    return {
        'speed_mps': max(0.0, float(np.max(speed - limits.speed_cap_mps))),
        'tire_angle_rad': max(0.0, float(np.max(np.abs(delta) - limits.tire_angle_rad))),
        'tire_rate_radps': max(0.0, float(np.max(rate - limits.tire_rate_radps))),
        'drive_mps2': max(0.0, float(np.max(forces - drive))),
        'brake_mps2': max(0.0, float(np.max(-forces - limits.brake_command_cap_mps2))),
        'friction_mps2': max(0.0, float(np.max(np.hypot(forces, lateral) - ay_mps2))),
    }
