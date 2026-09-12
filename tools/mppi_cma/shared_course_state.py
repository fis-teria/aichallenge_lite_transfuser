"""Pure readiness checks for four cars sharing one simulated course."""
from __future__ import annotations

import math


def all_vehicles_ready(vehicles: list[dict], now_s: float,
                       expected_xy_m: list[float]) -> bool:
    """Require four fresh, stationary, aligned poses and the requested reference."""
    if len(vehicles) != 4:
        raise ValueError('Exactly four vehicle telemetry records are required')
    for car in vehicles:
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
        if math.hypot(ref['x_m'] - expected_xy_m[0], ref['y_m'] - expected_xy_m[1]) > .005:
            raise ValueError('A vehicle loaded a different reference')
    return True
