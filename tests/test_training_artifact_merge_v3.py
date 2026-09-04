from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_SIDE_NAMES_V1,
)
import aic_transfuser_lite.data.behavior_view_v1 as behavior_module
from aic_transfuser_lite.data.behavior_view_v1 import (
    BehaviorAnnotationV1,
    load_behavior_view_v1,
    merge_behavior_views_v1,
)
import aic_transfuser_lite.data.split_merge_v3 as split_merge_module
from aic_transfuser_lite.data.split_merge_v3 import merge_split_manifests_v3


def _behavior_view(root: Path, sample_id: str, run_id: str, dataset_sha: str) -> None:
    root.mkdir()
    row = BehaviorAnnotationV1(
        sample_id=sample_id,
        run_id=run_id,
        grid_stamp_ns=1,
        behavior_class=0,
        behavior_label="FORWARD_NORMAL",
        behavior_side=0,
        behavior_side_label="NONE",
        behavior_valid=True,
        behavior_side_valid=True,
        quality=1.0,
        source_stamp_ns=1,
        source_age_ms=0.0,
        source="recovery_reference_phase",
        authority="teacher",
        target_vehicle=None,
        invalid_reason=None,
    )
    labels = root / "behavior_labels.csv"
    with labels.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(BehaviorAnnotationV1.__dataclass_fields__),
        )
        writer.writeheader()
        writer.writerow(row.__dict__)
    manifest = {
        "format": "aic_behavior_view_v1",
        "ontology": "aic_behavior_v1",
        "class_names": list(BEHAVIOR_CLASS_NAMES_V1),
        "side_names": list(BEHAVIOR_SIDE_NAMES_V1),
        "dataset_manifest_sha256": dataset_sha,
        "labels_sha256": hashlib.sha256(labels.read_bytes()).hexdigest(),
        "sample_count": 1,
        "valid_behavior_count": 1,
        "valid_side_count": 1,
    }
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def test_merge_behavior_views_requires_exact_combined_coverage(
    tmp_path: Path,
    monkeypatch,
) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    with (dataset / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("sample_id", "run_id"))
        writer.writeheader()
        writer.writerow({"sample_id": "sample1", "run_id": "run1"})
        writer.writerow({"sample_id": "sample2", "run_id": "run2"})
    first, second = tmp_path / "first", tmp_path / "second"
    _behavior_view(first, "sample1", "run1", "a" * 64)
    _behavior_view(second, "sample2", "run2", "b" * 64)
    monkeypatch.setattr(
        behavior_module,
        "validate_complete_dataset",
        lambda _: {"manifest_sha256": "c" * 64},
    )
    output = tmp_path / "merged"
    payload = merge_behavior_views_v1(
        dataset_root=dataset,
        source_view_roots=(first, second),
        output_root=output,
    )
    assert payload["sample_count"] == 2
    assert len(load_behavior_view_v1(output, dataset_manifest_sha256="c" * 64)) == 2


def _split_manifest(path: Path, run_id: str, split: str, dataset_sha: str) -> None:
    payload = {
        "format_version": "aic_split_manifest_v1",
        "split_seed": 1,
        "dataset_manifest_sha256": dataset_sha,
        "assignments": [
            {
                "run_id": run_id,
                "group_id": (run_id + "0" * 24)[:24],
                "component_id": (run_id + "1" * 24)[:24],
                "split": split,
            }
        ],
        "fixed_benchmark_group_ids": [],
        "leakage": {"status": "PASS"},
    }
    payload["manifest_sha256"] = split_merge_module._canonical_sha(payload)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_merge_split_manifests_preserves_source_assignments(
    tmp_path: Path,
    monkeypatch,
) -> None:
    first, second = tmp_path / "first.json", tmp_path / "second.json"
    _split_manifest(first, "run1", "train", "a" * 64)
    _split_manifest(second, "run2", "validation", "b" * 64)
    monkeypatch.setattr(
        split_merge_module,
        "validate_complete_dataset",
        lambda _: {
            "manifest_sha256": "c" * 64,
            "runs": [
                {"run_id": "run1", "source_hash": "d" * 64},
                {"run_id": "run2", "source_hash": "e" * 64},
            ],
        },
    )
    payload = merge_split_manifests_v3(
        dataset_root=tmp_path / "unused",
        source_manifest_paths=(first, second),
    )
    assert [row["split"] for row in payload["assignments"]] == [
        "train",
        "validation",
    ]
    assert payload["leakage"]["status"] == "PASS"
