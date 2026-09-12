"""Promote only reproducibly faster, contact-free, OT-free refinements."""
from __future__ import annotations

import math
from statistics import median


def compare(candidate: list[dict], incumbent: list[dict],
            candidate_dense: list[dict], incumbent_dense: list[dict]) -> dict:
    """Compare four runs per route; lap time is in seconds, OT margin is 0.1 m."""
    groups = (candidate, incumbent, candidate_dense, incumbent_dense)
    if any(len(group) != 4 for group in groups):
        raise ValueError('Four candidate and four incumbent runs and dense checks are required')
    values = [float(m['flying_lap_s']) for group in (candidate, incumbent) for m in group]
    if not all(math.isfinite(value) and value > 0 for value in values):
        raise ValueError('Lap times must be finite, positive seconds')
    candidate_ok = all(m['preferred_feasible'] for m in candidate) and all(d['passed'] for d in candidate_dense)
    incumbent_ok = all(m['preferred_feasible'] for m in incumbent) and all(d['passed'] for d in incumbent_dense)
    candidate_median = median(values[:4]); incumbent_median = median(values[4:])
    improvement = incumbent_median - candidate_median
    wins = sum(a < b for a, b in zip(values[:4], values[4:]))
    accepted = candidate_ok and incumbent_ok and improvement >= .05 and wins >= 3
    return {'candidate_median_s': candidate_median, 'incumbent_median_s': incumbent_median,
            'improvement_s': improvement, 'minimum_improvement_s': .05, 'comparison_wins': wins,
            'candidate_validated': candidate_ok, 'incumbent_validated': incumbent_ok,
            'promoted': accepted,
            'reason': 'repeated_improvement' if accepted else 'validation_failure' if not (candidate_ok and incumbent_ok)
                      else 'improvement_not_reproducible'}
