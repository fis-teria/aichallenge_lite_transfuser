#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aic_transfuser_lite.evaluation.compare_v3 import (
    paired_run_bootstrap_v3,
    screening_gate_status_v3,
)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resamples", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"comparison output already exists: {args.output}")
    baseline_summary = json.loads((args.baseline / "summary.json").read_text())
    candidate_summary = json.loads((args.candidate / "summary.json").read_text())
    if baseline_summary["cohort_identity_sha256"] != candidate_summary["cohort_identity_sha256"]:
        raise ValueError("A0 and candidate cohort identities differ")
    baseline_rows = _rows(args.baseline / "per_sample.csv")
    candidate_rows = _rows(args.candidate / "per_sample.csv")
    gate_status = screening_gate_status_v3(baseline_summary, candidate_summary)
    comparisons = {
        metric: paired_run_bootstrap_v3(
            baseline_rows,
            candidate_rows,
            metric=metric,
            resamples=args.resamples,
            seed=args.seed,
        )
        for metric in ("ade_m", "fde_m", "speed_mae_mps")
    }
    result = {
        "format": "aic_path_only_paired_comparison_v3",
        "cohort_identity_sha256": baseline_summary["cohort_identity_sha256"],
        "baseline_checkpoint_sha256": baseline_summary["checkpoint_sha256"],
        "candidate_checkpoint_sha256": candidate_summary["checkpoint_sha256"],
        "paired_run_bootstrap": comparisons,
        **gate_status,
        "baseline_teacher_quality": baseline_summary["teacher_quality"],
        "candidate_teacher_quality": candidate_summary["teacher_quality"],
        "baseline_launch": baseline_summary["stopped_commanded_motion"],
        "candidate_launch": candidate_summary["stopped_commanded_motion"],
        "closed_loop_claim": "not_evaluated",
    }
    args.output.mkdir(parents=True)
    (args.output / "comparison.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"status": "COMPLETE", "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
