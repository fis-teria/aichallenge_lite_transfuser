"""Postprocessing ONLY: join verified annotations to frozen results, never select/infer.

Preserves original selection/metrics, discloses val/validation spelling mismatch.
"""
from __future__ import annotations
import argparse
import csv
import io
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aic_transfuser_lite.evaluation.spatial_diagnostic_validation_v4 import LEDGER_SHA, read_checked, sha, write_json, grouped, comparisons


def join_annotations(selected: list[dict], annotations: list[dict]) -> list[dict]:
    by_id = {}
    ids = {c["sample_id"] for c in selected}
    for annotation in annotations:
        if annotation["sample_id"] not in ids:
            continue
        if annotation["sample_id"] in by_id:
            raise ValueError("duplicate annotation")
        by_id[annotation["sample_id"]] = annotation
    if set(by_id) != ids:
        raise ValueError("missing fixed-ID annotation")
    result = []
    for item in selected:
        annotation = by_id[item["sample_id"]]
        if annotation["split"] not in ("val", "validation") or annotation["run_id"] != item["run_id"]:
            raise ValueError("annotation split/run mismatch")
        result.append({**item, "normal_recovery": annotation["normal_recovery"], "collection_slice": annotation["collection_case"],
                       "annotation_original_split": annotation["split"]})
    return result


def annotate(run: Path, ledger: Path) -> None:
    destination = run / "artifacts/annotation_addendum.json"
    if destination.exists():
        raise FileExistsError("immutable annotation addendum")
    selection_blob = (run / "artifacts/selection.json").read_bytes()
    metric_blob = (run / "artifacts/metrics.json").read_bytes()
    selection = json.loads(selection_blob)
    rows = list(csv.DictReader(io.StringIO(read_checked(ledger, LEDGER_SHA).decode())))
    annotated = join_annotations(selection["selected"], rows)
    by_id = {r["sample_id"]: r for r in annotated}
    metrics = json.loads(metric_blob)["validation"]
    predictors = {}
    for key, value in metrics["predictions"].items():
        corrected = [{**r, **{field: by_id[r["sample_id"]][field] for field in ("normal_recovery", "collection_slice")}} for r in value["rows"]]
        predictors[key] = {"rows": corrected, "groups": grouped(corrected)}
    write_json(destination, {"kind": "ANNOTATION_ONLY_POSTPROCESSING_NOT_NEW_EVALUATION",
        "cause": "Executed evaluator filtered validation; existing coverage ledger uses val. Original metrics/selection retain UNKNOWN labels.",
        "selection_sha256": sha(selection_blob), "original_metrics_sha256": sha(metric_blob), "ledger_sha256": LEDGER_SHA,
        "new_inference_calls": 0, "new_optimizer_steps": 0, "reselection_performed": False,
        "selected_id_order_unchanged": [r["sample_id"] for r in annotated] == [r["sample_id"] for r in selection["selected"]],
        "selected_annotations": [{k: r[k] for k in ("sample_id", "run_id", "role", "normal_recovery", "collection_slice", "annotation_original_split")} for r in annotated],
        "predictions": predictors,
        "paired_comparisons": {key: comparisons(predictors["model"]["rows"], predictors[key]["rows"]) for key in ("zero", "straight", "mean_train")}})
    if (run / "artifacts/selection.json").read_bytes() != selection_blob or (run / "artifacts/metrics.json").read_bytes() != metric_blob:
        raise ValueError("original artifacts changed")
    print("ANNOTATION_ADDENDUM_COMPLETE fixed IDs/predictions unchanged; no selection/inference/training")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--ledger", type=Path, required=True)
    args = parser.parse_args()
    annotate(args.run, args.ledger)
