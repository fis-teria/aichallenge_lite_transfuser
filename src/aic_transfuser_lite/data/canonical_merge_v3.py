from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Any, Sequence
import uuid

import yaml

from .storage_v3 import validate_complete_dataset


_TABLES = ("runs.csv", "samples.csv")
_ROOT_FILES = frozenset({*_TABLES, "events.jsonl"})
_ASSET_CATEGORIES = frozenset(
    {"images", "lidar", "states", "trajectories", "controls"}
)
_COMPATIBILITY_FIELDS = (
    "schema_version",
    "storage_backend",
    "coordinate_frame",
    "distance_unit",
    "angle_unit",
    "time_unit",
)


def merge_canonical_datasets_v3(
    source_roots: Sequence[str | Path],
    output_root: str | Path,
    *,
    dataset_id: str,
    topic_profile_id: str,
) -> dict[str, Any]:
    """Atomically merge disjoint Canonical Dataset V3 roots using hard links."""

    if len(source_roots) < 2:
        raise ValueError("canonical merge requires at least two source datasets")
    if not dataset_id or not topic_profile_id:
        raise ValueError("dataset_id and topic_profile_id must be non-empty")
    sources = tuple(Path(value).resolve() for value in source_roots)
    manifests = tuple(validate_complete_dataset(source) for source in sources)
    baseline = manifests[0]
    for manifest in manifests[1:]:
        for field in _COMPATIBILITY_FIELDS:
            if manifest.get(field) != baseline.get(field):
                raise ValueError(f"canonical source {field} mismatch")
    runs: list[dict[str, Any]] = []
    run_ids: set[str] = set()
    for manifest in manifests:
        source_runs = manifest.get("runs")
        if not isinstance(source_runs, list):
            raise ValueError("canonical source manifest has no run inventory")
        for run in source_runs:
            run_id = str(run["run_id"])
            if run_id in run_ids:
                raise ValueError(f"canonical sources contain duplicate run_id: {run_id!r}")
            run_ids.add(run_id)
            runs.append(dict(run))
    output = Path(output_root).resolve()
    if output.exists():
        raise FileExistsError(f"canonical merge output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(f".{output.name}.tmp-{uuid.uuid4().hex[:8]}")
    staging.mkdir()
    (staging / ".incomplete").write_text("incomplete\n", encoding="utf-8")
    try:
        for category in sorted(_ASSET_CATEGORIES):
            (staging / category).mkdir()
        for table in _TABLES:
            _merge_csv_tables(tuple(source / table for source in sources), staging / table)
        with (staging / "events.jsonl").open("wb") as output_stream:
            for source in sources:
                with (source / "events.jsonl").open("rb") as input_stream:
                    shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
        inventory: dict[str, dict[str, Any]] = {}
        hardlink_count = 0
        for source, manifest in zip(sources, manifests):
            files = manifest.get("files")
            if not isinstance(files, list):
                raise ValueError("canonical source manifest has no file inventory")
            for record in files:
                relative = str(record["path"])
                if relative in _ROOT_FILES:
                    continue
                path = Path(relative)
                if path.is_absolute() or ".." in path.parts or path.parts[0] not in _ASSET_CATEGORIES:
                    raise ValueError(f"unexpected canonical asset path: {relative!r}")
                if relative in inventory:
                    raise ValueError(f"canonical sources collide at asset: {relative!r}")
                source_file = source / path
                destination = staging / path
                destination.parent.mkdir(parents=True, exist_ok=True)
                if not source_file.is_file() or source_file.stat().st_size != int(record["size_bytes"]):
                    raise ValueError(f"canonical source asset size mismatch: {source_file}")
                os.link(source_file, destination)
                inventory[relative] = {
                    "path": relative,
                    "size_bytes": int(record["size_bytes"]),
                    "sha256": str(record["sha256"]),
                }
                hardlink_count += 1
        for relative in _ROOT_FILES:
            path = staging / relative
            inventory[relative] = {
                "path": relative,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        (staging / ".incomplete").unlink()
        payload: dict[str, Any] = {
            "dataset_id": dataset_id,
            "topic_profile_id": topic_profile_id,
            "runs": runs,
            **{field: baseline[field] for field in _COMPATIBILITY_FIELDS},
            "complete": True,
            "files": [inventory[key] for key in sorted(inventory)],
        }
        manifest_sha = _canonical_sha(payload)
        (staging / "manifest.yaml").write_text(
            yaml.safe_dump(
                {**payload, "manifest_sha256": manifest_sha},
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        os.replace(staging, output)
        validate_complete_dataset(output)
        return {
            "status": "COMPLETE",
            "dataset_id": dataset_id,
            "manifest_sha256": manifest_sha,
            "run_count": len(runs),
            "hardlink_count": hardlink_count,
        }
    except BaseException:
        if staging.exists() and staging.parent == output.parent:
            shutil.rmtree(staging)
        raise


def _merge_csv_tables(sources: Sequence[Path], output: Path) -> None:
    fieldnames: list[str] | None = None
    with output.open("w", newline="", encoding="utf-8") as output_stream:
        writer = None
        for source in sources:
            with source.open(newline="", encoding="utf-8") as input_stream:
                reader = csv.DictReader(input_stream)
                current_fields = list(reader.fieldnames or ())
                if not current_fields:
                    raise ValueError(f"canonical CSV has no header: {source}")
                if fieldnames is None:
                    fieldnames = current_fields
                    writer = csv.DictWriter(output_stream, fieldnames=fieldnames)
                    writer.writeheader()
                elif current_fields != fieldnames:
                    raise ValueError(f"canonical CSV fields mismatch: {source}")
                assert writer is not None
                for row in reader:
                    writer.writerow(row)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
