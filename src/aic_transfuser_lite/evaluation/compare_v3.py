from __future__ import annotations

from collections import defaultdict
import math
from typing import Mapping, Sequence

import numpy as np


def paired_run_bootstrap_v3(
    baseline_rows: Sequence[Mapping[str, object]],
    candidate_rows: Sequence[Mapping[str, object]],
    *,
    metric: str,
    resamples: int = 10_000,
    seed: int = 42,
) -> dict[str, object]:
    """Paired bootstrap CI over run-level metric means (candidate-baseline)."""

    if resamples <= 0:
        raise ValueError("paired bootstrap resamples must be positive")
    baseline_by_id = {str(row["sample_id"]): row for row in baseline_rows}
    candidate_by_id = {str(row["sample_id"]): row for row in candidate_rows}
    if len(baseline_by_id) != len(baseline_rows) or len(candidate_by_id) != len(candidate_rows):
        raise ValueError("paired bootstrap sample IDs must be unique")
    if set(baseline_by_id) != set(candidate_by_id):
        raise ValueError("paired bootstrap cohorts differ")
    differences: dict[str, list[float]] = defaultdict(list)
    for sample_id in sorted(baseline_by_id):
        baseline = baseline_by_id[sample_id]
        candidate = candidate_by_id[sample_id]
        if str(baseline["run_id"]) != str(candidate["run_id"]):
            raise ValueError("paired bootstrap run identity differs")
        left = float(baseline[metric])
        right = float(candidate[metric])
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError("paired bootstrap metric must be finite")
        differences[str(baseline["run_id"])].append(right - left)
    run_delta = {
        run_id: float(np.mean(values)) for run_id, values in sorted(differences.items())
    }
    values = np.asarray(tuple(run_delta.values()), dtype=np.float64)
    rng = np.random.default_rng(seed)
    samples = values[
        rng.integers(0, len(values), size=(resamples, len(values)))
    ].mean(axis=1)
    return {
        "metric": metric,
        "delta_definition": "candidate_minus_baseline",
        "run_count": len(values),
        "paired_run_equal_delta": float(values.mean()),
        "ci95_low": float(np.percentile(samples, 2.5)),
        "ci95_high": float(np.percentile(samples, 97.5)),
        "resamples": resamples,
        "seed": seed,
        "run_delta": run_delta,
    }
