"""Frozen validation selection; teacher PP eligibility is shared by all candidates."""
from __future__ import annotations

from collections import Counter
from typing import Any, Sequence

import numpy as np


def validation_partitions(run_ids: Sequence[str], selection_runs: Sequence[str],
                          comparison_runs: Sequence[str]) -> tuple[list[int], list[int]]:
    """Keep historical batch membership/order and comparison-only batches separate."""
    selected, compared = set(selection_runs), set(comparison_runs)
    if (not selected or not compared or selected & compared
            or set(run_ids) != selected | compared):
        raise ValueError("disjoint nonempty validation roles must cover exactly the available runs")
    return ([i for i, rid in enumerate(run_ids) if rid in selected],
            [i for i, rid in enumerate(run_ids) if rid in compared])


def pp_agreement_score(teacher: Sequence[dict[str, Any]], predicted: Sequence[dict[str, Any]],
                       run_ids: Sequence[str], *, rejection_penalty_rad: float = .6) -> dict[str, Any]:
    """Run-equal absolute physical tire error [rad], on fixed teacher support.

    A candidate rejection receives 0.6 rad: the complete [-0.3,0.3] tire range.
    Rejecting a difficult prediction never removes its denominator. This measures
    PP geometry at recorded states, not scan clearance, steering response or laps.
    """
    if (not len(teacher) == len(predicted) == len(run_ids) or not teacher
            or rejection_penalty_rad != .6):
        raise ValueError("one teacher/prediction/run per sample; fixed 0.6 rad rejection penalty required")
    groups: dict[str, list[float]] = {}
    rejected: Counter[str] = Counter()
    paired: dict[str, list[float]] = {}
    excluded: Counter[str] = Counter()
    for truth, proposal, rid in zip(teacher, predicted, run_ids, strict=True):
        if not truth["applicable"] or not truth["accepted"]:
            excluded[truth["reason"]] += 1
            continue
        angle = float(truth["steer_rad"])
        if not np.isfinite(angle) or abs(angle) > .300001:
            raise ValueError("teacher PP physical tire angle outside contract")
        if not proposal["applicable"] or not proposal["accepted"]:
            value = rejection_penalty_rad
            rejected[rid] += 1
        else:
            other = float(proposal["steer_rad"])
            if not np.isfinite(other) or abs(other) > .300001:
                raise ValueError("candidate PP physical tire angle outside contract")
            value = abs(angle - other)
            paired.setdefault(rid, []).append(value)
        groups.setdefault(rid, []).append(value)
    per_run = {rid: {"teacher_supported": len(values), "candidate_rejected": rejected[rid],
                    "penalized_mean_rad": float(np.mean(values)),
                    "paired_mean_rad": float(np.mean(paired[rid])) if paired.get(rid) else None}
               for rid, values in sorted(groups.items())}
    return {
        "scope": "TEACHER_PP_AGREEMENT_AT_OBSERVATION_NOT_CLOSED_LOOP",
        "total_anchors": len(teacher), "teacher_supported": sum(map(len, groups.values())),
        "supported_runs": len(per_run), "candidate_rejected": sum(rejected.values()),
        "teacher_exclusions": dict(excluded), "per_run": per_run,
        "run_macro_penalized_rad": float(np.mean([r["penalized_mean_rad"] for r in per_run.values()])) if per_run else None,
        "rejection_penalty_rad": rejection_penalty_rad,
    }


def select_epoch(rows: Sequence[dict[str, Any]], policy: str) -> int:
    """Select a retained epoch, without consulting comparison-only runs."""
    if not rows or policy not in {"endpoint_3s", "teacher_pp"}:
        raise ValueError("nonempty epoch scores and known policy required")
    support = [r["pp"]["teacher_supported"] for r in rows]
    run_support = [r["pp"]["per_run"] for r in rows]
    population = [{k: v["teacher_supported"] for k, v in rr.items()} for rr in run_support]
    if len(set(support)) != 1 or any(p != population[0] for p in population):
        raise ValueError("teacher PP selection denominator changed across epochs")
    if len({r["epoch"] for r in rows}) != len(rows):
        raise ValueError("duplicate epoch")
    for row in rows:
        scores = [row["xy_3s_m"]]
        if policy == "teacher_pp":
            scores += [row["pp"]["run_macro_penalized_rad"]]
        if any(v is None or not np.isfinite(v) or v < 0 for v in scores):
            raise ValueError("selection score has no finite support")
    return int(min(rows, key=lambda r: (r["xy_3s_m"], r["epoch"]) if policy == "endpoint_3s"
        else (r["pp"]["run_macro_penalized_rad"], r["xy_3s_m"], r["epoch"]))["epoch"])
