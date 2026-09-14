"""PP compatibility at recorded states, not a simulated vehicle rollout."""
from __future__ import annotations

from collections import Counter
from typing import Any, Sequence

import numpy as np

from ..control.time_reference_v1 import TimePlan, TimedBodyPose
from ..control.time_trial_v1 import time_trial_control


def pp_probe(xy_m: np.ndarray, observation: TimedBodyPose, current: TimedBodyPose,
             speed_mps: float, config: dict[str, Any]) -> dict[str, Any]:
    """Probe a [30,2] metre prediction at an explicit observed state and age.

    Speeds outside the fixed-5-km/h trial's measured operating range are
    non-applicable, not model rejections. No scan guard or actuator is executed.
    """
    if not np.isfinite(speed_mps) or not -.03 <= speed_mps <= config['overspeed_limit_mps']:
        return {'applicable': False, 'accepted': False, 'reason': 'SPEED_OUTSIDE_FIXED_5KMH_CONTRACT'}
    try:
        result = time_trial_control(TimePlan('offline_probe', observation, xy_m), current,
            speed_mps=speed_mps,
            rear_axle_offset_m=(config['geometry']['rear_axle_forward_in_base_link_m'], 0.),
            speed_policy=config['speed_policy'], lookahead_policy=config['lookahead_policy'],
            vehicle_model_policy=config['vehicle_model_policy'])
    except ValueError as exc:
        return {'applicable': True, 'accepted': False, 'reason': str(exc)}
    return {'applicable': True, 'accepted': True, 'reason': 'PP_ACCEPTED',
        'steer_rad': result['steer_rad'],
        'lookahead_distance_m': result['selected_lookahead_distance_m'],
        'age_s': result['plan_age_sec'],
        'observation_horizon_s': result.get('lookahead_selection', {}).get('observation_horizon_s')}


def summarize_pp(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Keep all attempted anchors, applicable states, and accepted paths distinct."""
    applicable = [r for r in rows if r['applicable']]
    accepted = [r for r in applicable if r['accepted']]
    steering = [abs(r['steer_rad']) for r in accepted]
    return {'attempted': len(rows), 'applicable': len(applicable), 'accepted': len(accepted),
        'accepted_fraction': len(accepted)/len(applicable) if applicable else None,
        'reasons': dict(Counter(r['reason'] for r in rows)),
        'max_absolute_steer_rad': max(steering) if steering else None,
        'scope': 'PP_CALCULATION_ONLY_NOT_SAFETY_OR_CLOSED_LOOP'}
