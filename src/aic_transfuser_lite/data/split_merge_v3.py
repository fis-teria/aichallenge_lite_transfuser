from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

from .split_v3 import SPLIT_MANIFEST_FORMAT
from .storage_v3 import validate_complete_dataset


def merge_split_manifests_v3(
    *,
    dataset_root: str | Path,
    source_manifest_paths: Sequence[str | Path],
) -> dict[str, Any]:
    """Preserve source run assignments in one combined Dataset V3 manifest."""

    if not source_manifest_paths:
        raise ValueError("at least one source split manifest is required")
    dataset = validate_complete_dataset(dataset_root)
    dataset_runs = dataset.get("runs")
    if not isinstance(dataset_runs, list):
        raise ValueError("target Dataset V3 manifest has no run inventory")
    source_hash_by_run = {str(row["run_id"]): str(row["source_hash"]) for row in dataset_runs}
    if len(source_hash_by_run) != len(dataset_runs):
        raise ValueError("target Dataset V3 contains duplicate run IDs")
    assignments: list[dict[str, str]] = []
    provenance = []
    fixed_groups: set[str] = set()
    for path_raw in source_manifest_paths:
        path = Path(path_raw)
        source = json.loads(path.read_text(encoding="utf-8"))
        expected_hash = source.get("manifest_sha256")
        payload = dict(source)
        payload.pop("manifest_sha256", None)
        if expected_hash != _canonical_sha(payload):
            raise ValueError(f"source split manifest hash mismatch: {path}")
        if source.get("format_version") != SPLIT_MANIFEST_FORMAT:
            raise ValueError(f"source split manifest format mismatch: {path}")
        if source.get("leakage", {}).get("status") != "PASS":
            raise ValueError(f"source split manifest leakage is not PASS: {path}")
        rows = source.get("assignments")
        if not isinstance(rows, list):
            raise ValueError(f"source split assignments are missing: {path}")
        assignments.extend(
            {
                "run_id": str(row["run_id"]),
                "group_id": str(row["group_id"]),
                "component_id": str(row["component_id"]),
                "split": str(row["split"]),
            }
            for row in rows
        )
        fixed_groups.update(str(value) for value in source.get("fixed_benchmark_group_ids", []))
        provenance.append(
            {
                "path": str(path.resolve()),
                "manifest_sha256": str(expected_hash),
                "dataset_manifest_sha256": str(source.get("dataset_manifest_sha256", "")),
                "split_seed": int(source["split_seed"]),
            }
        )
    by_run = {row["run_id"]: row for row in assignments}
    if len(by_run) != len(assignments):
        raise ValueError("source split manifests contain duplicate run IDs")
    missing = set(source_hash_by_run) - set(by_run)
    extra = set(by_run) - set(source_hash_by_run)
    if missing or extra:
        raise ValueError(
            f"combined split run coverage mismatch: missing={len(missing)}, extra={len(extra)}"
        )
    for identity in ("group_id", "component_id"):
        splits_by_identity: dict[str, set[str]] = {}
        for row in assignments:
            splits_by_identity.setdefault(row[identity], set()).add(row["split"])
        if any(len(splits) > 1 for splits in splits_by_identity.values()):
            raise ValueError(f"combined split crosses {identity} between splits")
    splits_by_source_hash: dict[str, set[str]] = {}
    for row in assignments:
        source_hash = source_hash_by_run[row["run_id"]]
        splits_by_source_hash.setdefault(source_hash, set()).add(row["split"])
    source_overlap = sum(
        1 for splits in splits_by_source_hash.values() if len(splits) > 1
    )
    if source_overlap:
        raise ValueError("combined split leaks identical source hashes between splits")
    assignments.sort(key=lambda row: row["run_id"])
    payload: dict[str, Any] = {
        "format_version": SPLIT_MANIFEST_FORMAT,
        "split_seed": -1,
        "split_strategy": "composed_preserve_source_assignments_v1",
        "dataset_manifest_sha256": dataset["manifest_sha256"],
        "assignments": assignments,
        "fixed_benchmark_group_ids": sorted(fixed_groups),
        "source_split_manifests": provenance,
        "leakage": {
            "run_id_overlap_count": 0,
            "source_hash_overlap_count": 0,
            "collection_session_overlap_count": 0,
            "run_family_overlap_count": 0,
            "trajectory_fingerprint_overlap_count": 0,
            "status": "PASS",
        },
    }
    payload["manifest_sha256"] = _canonical_sha(payload)
    return payload


def _canonical_sha(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
