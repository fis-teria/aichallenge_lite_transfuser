from __future__ import annotations

from collections import defaultdict
import math
from typing import Mapping, Sequence

import numpy as np


def screening_gate_status_v3(
    baseline_summary: Mapping[str, object],
    candidate_summary: Mapping[str, object],
) -> dict[str, bool]:
    """Combine launch readiness with waypoint-ADE non-regression.

    Older standalone evaluation artifacts called the launch-only result
    ``screening_gate_pass``.  Accept that key when reading historical artifacts,
    but expose the two gates separately so launch readiness cannot be mistaken
    for the complete research-candidate screen.
    """

    baseline_launch_pass = bool(
        baseline_summary.get(
            "launch_gate_pass", baseline_summary.get("screening_gate_pass", False)
        )
    )
    candidate_launch_pass = bool(
        candidate_summary.get(
            "launch_gate_pass", candidate_summary.get("screening_gate_pass", False)
        )
    )
    baseline_quality = baseline_summary["teacher_quality"]
    candidate_quality = candidate_summary["teacher_quality"]
    if not isinstance(baseline_quality, Mapping) or not isinstance(
        candidate_quality, Mapping
    ):
        raise TypeError("teacher_quality summaries must be mappings")
    baseline_ade = float(baseline_quality["trajectory_waypoint_weighted_ade_m"])
    candidate_ade = float(candidate_quality["trajectory_waypoint_weighted_ade_m"])
    if not math.isfinite(baseline_ade) or not math.isfinite(candidate_ade):
        raise ValueError("trajectory waypoint ADE must be finite")
    regression_pass = candidate_ade <= baseline_ade
    return {
        "baseline_launch_gate_pass": baseline_launch_pass,
        "candidate_launch_gate_pass": candidate_launch_pass,
        "candidate_trajectory_regression_gate_pass": regression_pass,
        "baseline_screening_gate_pass": baseline_launch_pass,
        "candidate_screening_gate_pass": candidate_launch_pass and regression_pass,
    }


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
