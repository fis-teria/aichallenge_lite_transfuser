from __future__ import annotations

import csv
import os
from pathlib import Path

import yaml

from aic_transfuser_lite.data.canonical_merge_v3 import (
    _canonical_sha,
    _sha256_file,
    merge_canonical_datasets_v3,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset


def _dataset(root: Path, run_id: str, sample_id: str) -> None:
    root.mkdir()
    for category in ("images", "lidar", "states", "trajectories", "controls"):
        (root / category).mkdir()
    asset = root / "states" / f"{run_id}.bin"
    asset.write_bytes(run_id.encode("utf-8"))
    with (root / "runs.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("run_id", "scenario_id"))
        writer.writeheader()
        writer.writerow({"run_id": run_id, "scenario_id": "d1"})
    with (root / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("sample_id", "run_id"))
        writer.writeheader()
        writer.writerow({"sample_id": sample_id, "run_id": run_id})
    (root / "events.jsonl").write_text("", encoding="utf-8")
    files = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        files.append(
            {
                "path": path.relative_to(root).as_posix(),
                "size_bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    payload = {
        "dataset_id": run_id,
        "topic_profile_id": "profile",
        "runs": [{"run_id": run_id, "source_hash": run_id[0] * 64}],
        "schema_version": "aic_canonical_dataset_v3",
        "storage_backend": "csv_npy_jpeg_v1",
        "coordinate_frame": "base_link@t_obs",
        "distance_unit": "m",
        "angle_unit": "rad",
        "time_unit": "s",
        "complete": True,
        "files": files,
    }
    (root / "manifest.yaml").write_text(
        yaml.safe_dump({**payload, "manifest_sha256": _canonical_sha(payload)}),
        encoding="utf-8",
    )


def test_merge_canonical_datasets_is_atomic_and_hardlinked(tmp_path: Path) -> None:
    first, second = tmp_path / "first", tmp_path / "second"
    _dataset(first, "aaa", "sample1")
    _dataset(second, "bbb", "sample2")
    output = tmp_path / "merged"
    result = merge_canonical_datasets_v3(
        (first, second),
        output,
        dataset_id="combined",
        topic_profile_id="mixed",
    )
    manifest = validate_complete_dataset(output)
    assert result["run_count"] == 2
    assert manifest["dataset_id"] == "combined"
    assert len((output / "samples.csv").read_text().splitlines()) == 3
    assert os.stat(first / "states" / "aaa.bin").st_ino == os.stat(
        output / "states" / "aaa.bin"
    ).st_ino
