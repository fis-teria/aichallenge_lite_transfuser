"""Pure readiness checks for four cars sharing one simulated course."""
from __future__ import annotations

import math


def native_vehicle_status(summary: dict | None, number: int) -> list | None:
    """Read rank by vehicle number from AWSIM admin data, never array ordering.

    The per-domain /awsim/status stream can mirror D1. Only lap and rank are
    supplied here; unmeasured lap time, remaining time and section stay None.
    """
    if not summary:
        return None
    matches = [car for car in summary.get('vehicles', []) if car['vehicle_number'] == number]
    if not matches:
        return None
    if len(matches) != 1:
        raise ValueError('Duplicate native vehicle number')
    car = matches[0]
    lap = car['lap_count'] + (0 if car['finished'] else 1)
    return [None, min(summary['session']['required_laps'], lap), None, None, car['final_position']]


def all_vehicles_ready(vehicles: list[dict], now_s: float,
                       expected_xy_m: list[float],
                       per_vehicle_xy_m: list[list[float]] | None = None) -> bool:
    """Require four fresh, stationary, aligned poses and the requested reference."""
    if len(vehicles) != 4:
        raise ValueError('Exactly four vehicle telemetry records are required')
    references = per_vehicle_xy_m if per_vehicle_xy_m is not None else [expected_xy_m] * 4
    if len(references) != 4 or any(len(xy) != 2 or not all(math.isfinite(v) for v in xy) for xy in references):
        raise ValueError('Expected reference coordinates must have shape (4, 2), in metres')
    for car, expected in zip(vehicles, references):
        ego, gps, ref = (car.get(key) for key in ('ego', 'gnss', 'reference'))
        if not ego or not gps or not ref:
            return False
        values = [ego[k] for k in ('x_m', 'y_m', 'speed_mps', 'arrival_s')]
        values += [gps[k] for k in ('x_m', 'y_m', 'arrival_s')]
        values += [ref[k] for k in ('x_m', 'y_m')]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Non-finite readiness telemetry')
        if (not 0 <= now_s - ego['arrival_s'] < .3
                or not 0 <= now_s - gps['arrival_s'] < .3
                or abs(ego['speed_mps']) >= .15
                or math.hypot(ego['x_m'] - gps['x_m'], ego['y_m'] - gps['y_m']) >= .5):
            return False
        if math.hypot(ref['x_m'] - expected[0], ref['y_m'] - expected[1]) > .005:
            raise ValueError('A vehicle loaded a different reference')
    return True
