"""Summarize completed WSL comparison JSON without loading models or datasets."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from statistics import fmean
import sys
from typing import Any


def read(path: Path) -> Any:
    return json.loads(path.read_text())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def group_metrics(group: dict[str, Any]) -> dict[str, Any]:
    return {
        "anchors": group["anchor_count"], "runs": group["run_count"],
        "xy_cm": {t: row["raw_error_m"] * 100 for t, row in group["xy"]["run_macro_mean"].items()},
        "components_3s_cm": {k: group["components"]["3s"][k] * 100
                             for k in ("forward_mae_m", "left_mae_m", "left_bias_m")},
        "pp_rad": group["pp"]["run_macro_penalized_rad"],
        "pp_rejected": group["pp"]["candidate_rejected"],
        "teacher_pp_supported": group["pp"]["teacher_supported"],
        "endpoint_pooled_cm": group["endpoint_pooled_cm"],
    }


def main(run: Path, output: Path) -> None:
    """Write derived metrics with source hashes; all position outputs use cm."""
    source = run / "comparison" / "summary.json"
    summary = read(source)
    names = ["uniform_l1", "balanced_l1", "uniform_geometry", "balanced_geometry"]
    assert summary["status"] == "COMPLETE" and list(summary["arms"]) == names
    assert not summary["test_evaluated"] and not summary["reserved_test_raw_or_cache_read"]
    result: dict[str, Any] = {
        "status": "COMPLETE", "scope": summary["scope"],
        "source_commit": summary["source"], "input_sha256": {str(source): sha(source)},
        "experiment_plan": summary["experiment_plan"],
        "test_evaluated": False, "reserved_test_raw_or_cache_read": False,
        "data_counts": summary["data_counts"], "arms": {}, "strict_pairs": {}, "strict_per_run": {},
    }
    for name, entry in summary["arms"].items():
        result["arms"][name] = {
            "checkpoint_sha256": entry["checkpoint_sha256"], "best_epoch": entry["best_epoch"],
            "normal_validation_3s_cm": entry["normal_validation"]["run_macro_mean"]["3s"]["raw_error_m"] * 100,
            "selection_6run_3s_cm": entry["selection_metrics"]["run_macro_mean"]["3s"]["raw_error_m"] * 100,
            "archived_recovery_max_abs_m": entry["archived_selection_recovery_prediction_max_abs_m"],
        }
        for split in ("train", "validation"):
            result["arms"][name][split] = {group: group_metrics(values)
                                           for group, values in entry[split].items()}
    for split, expected in (("train", 35), ("validation", 12)):
        rows = {}
        for name in names:
            path = run / "comparison" / f"{name}_{split}_rows.json"
            values = read(path)
            rows[name] = {row["anchor_id"]: row for row in values}
            assert len(rows[name]) == len(values) == summary["data_counts"][split]["anchors"]
            result["input_sha256"][str(path)] = sha(path)
        baseline = rows[names[0]]
        ids = [aid for aid, row in baseline.items() if row["strict_outward"]]
        assert len(ids) == expected
        pairs = []
        for aid in ids:
            teacher, rid = baseline[aid]["teacher_pp"], baseline[aid]["run_id"]
            assert teacher["applicable"] and teacher["accepted"]
            pair = {"anchor_id": aid, "run_id": rid, "teacher_pp": teacher, "arms": {}}
            for name in names:
                row = rows[name][aid]
                assert row["strict_outward"] and row["run_id"] == rid and row["teacher_pp"] == teacher
                prediction = row["prediction_pp"]
                accepted = prediction["applicable"] and prediction["accepted"]
                pair["arms"][name] = {
                    "endpoint_xy_cm": row["endpoint_error_m"] * 100,
                    "endpoint_left_cm": row["endpoint_left_error_m"] * 100,
                    "prediction_pp": prediction,
                    "signed_pp_error_rad": prediction["steer_rad"] - teacher["steer_rad"] if accepted else None,
                }
            pairs.append(pair)
        result["strict_pairs"][split] = pairs
        per_run = {}
        for rid in sorted({pair["run_id"] for pair in pairs}):
            selected = [pair for pair in pairs if pair["run_id"] == rid]
            per_run[rid] = {name: {
                "anchors": len(selected),
                "xy_cm": fmean(pair["arms"][name]["endpoint_xy_cm"] for pair in selected),
                "left_bias_cm": fmean(pair["arms"][name]["endpoint_left_cm"] for pair in selected),
                "left_positive_count": sum(pair["arms"][name]["endpoint_left_cm"] > 0 for pair in selected),
                "pp_rad": fmean(abs(pair["arms"][name]["signed_pp_error_rad"])
                    if pair["arms"][name]["signed_pp_error_rad"] is not None else .6 for pair in selected),
            } for name in names}
        for name in names:
            # Rebuild the run-equal metrics from exported individual rows.
            expected_group = result["arms"][name][split]["strict_outward"]
            assert abs(fmean(row[name]["xy_cm"] for row in per_run.values())
                       - expected_group["xy_cm"]["3s"]) < 1e-4
            assert abs(fmean(row[name]["pp_rad"] for row in per_run.values())
                       - expected_group["pp_rad"]) < 1e-10
        result["strict_per_run"][split] = per_run
    output.mkdir(parents=True, exist_ok=False)
    (output / "paired_analysis.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": "COMPLETE", "output": str(output), "strict_pairs": {
        split: len(rows) for split, rows in result["strict_pairs"].items()}}, indent=2))
    for name, entry in result["arms"].items():
        print(name, json.dumps({"normal_3s_cm": entry["normal_validation_3s_cm"], **{
            split: {g: entry[split][g] for g in ("all", "strict_outward")}
            for split in ("train", "validation")}}, allow_nan=False))


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
