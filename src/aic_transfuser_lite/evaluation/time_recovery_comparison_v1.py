"""PP compatibility at recorded states, not a simulated vehicle rollout."""
from __future__ import annotations

from collections import Counter
from typing import Any, Sequence

import numpy as np

from ..control.time_reference_v1 import TimePlan, TimedBodyPose
from ..control.time_trial_v1 import time_trial_control
from .time_clearance_v1 import PoseIndex


def recorded_pose_for_pp(index: PoseIndex, observation: TimedBodyPose,
                         age_s: float) -> tuple[TimedBodyPose | None, str]:
    """Use the frozen source pose at age zero; never choose a duplicate future.

    Future ambiguity is explicit missing evaluator support, not a path rejection.
    Other malformed pose/gap errors still fail the evaluation.
    """
    if age_s not in (0., .1, .2):
        raise ValueError('expected recorded PP age 0, 0.1 or 0.2 seconds')
    if age_s == 0:
        return observation, 'FROZEN_OBSERVATION_POSE'
    try:
        return index.at(observation.stamp_ns+int(round(age_s*1e9))), 'RECORDED_FUTURE_POSE'
    except ValueError as exc:
        if str(exc) != 'AMBIGUOUS_POSE_STAMP':
            raise
        return None, 'RECORDED_STATE_AMBIGUOUS_POSE_STAMP'


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


def component_errors(prediction: np.ndarray, target: np.ndarray, mask: np.ndarray,
                     input_valid: np.ndarray, run_ids: Sequence[str]) -> dict[str, Any]:
    """Run-equal forward/left errors in observation-body metres, with support."""
    prediction, target = np.asarray(prediction), np.asarray(target)
    mask, input_valid = np.asarray(mask, bool), np.asarray(input_valid, bool)
    if (prediction.ndim != 3 or prediction.shape[1:] != (30, 2) or target.shape != prediction.shape
            or mask.shape != prediction.shape[:2] or input_valid.shape != (len(prediction),)
            or len(run_ids) != len(prediction)):
        raise ValueError('expected [N,30,2] XY, [N,30] mask and one run/input flag per anchor')
    ids = np.asarray(run_ids)
    result = {}
    for seconds in (.5, 1., 2., 3.):
        index = int(seconds*10)-1
        valid = input_valid & mask[:, index] & np.isfinite(prediction[:, index]).all(axis=1) & np.isfinite(target[:, index]).all(axis=1)
        error = prediction[:, index].astype(float)-target[:, index].astype(float)
        per_run = []
        for run in sorted(set(run_ids)):
            values = error[valid & (ids == run)]
            if len(values):
                per_run.append((float(np.abs(values[:, 0]).mean()), float(np.abs(values[:, 1]).mean()), float(values[:, 1].mean())))
        average = np.mean(per_run, axis=0).tolist() if per_run else [None, None, None]
        result[f'{seconds:g}s'] = {'forward_mae_m': average[0], 'left_mae_m': average[1],
            'left_bias_m': average[2], 'supported_anchors': int(valid.sum()), 'supported_runs': len(per_run),
            'total_anchors': len(prediction)}
    return result
