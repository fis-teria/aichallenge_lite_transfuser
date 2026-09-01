from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from aic_transfuser_lite.data.canonical_schema_v3 import (
    DatasetManifestV3,
    RunRecordV3,
)
from aic_transfuser_lite.data.storage_v3 import (
    CsvNpyJpegBackend,
    validate_complete_dataset,
)


def _run() -> RunRecordV3:
    return RunRecordV3(
        run_id="run01",
        scenario_id="scenario01",
        segment_id="epoch0000",
        source_uri="file:///bag/run01",
        source_hash="a" * 64,
        topic_profile_id="default",
        start_stamp_ns=1,
        end_stamp_ns=2,
        capabilities=("trajectory_label",),
        conversion_status="complete",
    )


def _manifest() -> DatasetManifestV3:
    return DatasetManifestV3("dataset01", "default", (_run(),))


def _write(output: Path) -> str:
    with CsvNpyJpegBackend(output, _manifest()) as backend:
        backend.write_run(_run())
        image_path = backend.write_image(
            "run01/sample01.jpg", Image.new("RGB", (4, 3), color="red")
        )
        lidar_path = backend.write_array(
            "lidar", "run01/sample01.npy", np.array([1.0, 2.0], dtype=np.float32)
        )
        backend.write_sample(
            {
                "sample_id": "run01__epoch0000__1",
                "run_id": "run01",
                "image_path": image_path,
                "lidar_path": lidar_path,
            }
        )
        backend.write_event({"stamp_ns": 1, "name": "start"})
        return backend.finalize().manifest_sha256


def test_csv_npy_jpeg_backend_finalizes_atomically_and_validates(tmp_path: Path) -> None:
    output = tmp_path / "dataset"
    manifest_hash = _write(output)
    assert output.is_dir() and not (output / ".incomplete").exists()
    assert (output / "runs.csv").is_file()
    assert (output / "samples.csv").is_file()
    assert (output / "events.jsonl").is_file()
    assert np.load(output / "lidar/run01/sample01.npy", allow_pickle=False).shape == (2,)
    manifest = validate_complete_dataset(output)
    assert manifest["manifest_sha256"] == manifest_hash
    assert manifest["storage_backend"] == "csv_npy_jpeg"


def test_same_content_has_deterministic_manifest_hash(tmp_path: Path) -> None:
    assert _write(tmp_path / "one") == _write(tmp_path / "two")


def test_existing_output_and_duplicate_assets_are_rejected(tmp_path: Path) -> None:
    output = tmp_path / "dataset"
    output.mkdir()
    with pytest.raises(FileExistsError):
        CsvNpyJpegBackend(output, _manifest())

    backend = CsvNpyJpegBackend(tmp_path / "other", _manifest())
    try:
        backend.write_array("lidar", "same", np.array([1], dtype=np.float32))
        with pytest.raises(FileExistsError):
            backend.write_array("lidar", "same", np.array([2], dtype=np.float32))
    finally:
        backend.abort()


def test_exception_removes_only_partial_staging_and_never_publishes_output(
    tmp_path: Path,
) -> None:
    output = tmp_path / "dataset"
    with pytest.raises(RuntimeError, match="stop"):
        with CsvNpyJpegBackend(output, _manifest()) as backend:
            backend.write_run(_run())
            raise RuntimeError("stop")
    assert not output.exists()
    assert not list(tmp_path.glob(".dataset.tmp-*"))


def test_complete_validator_rejects_partial_and_tampered_manifest(tmp_path: Path) -> None:
    partial = tmp_path / "partial"
    partial.mkdir()
    (partial / ".incomplete").write_text("incomplete\n", encoding="utf-8")
    with pytest.raises(ValueError, match="not atomically complete"):
        validate_complete_dataset(partial)

    output = tmp_path / "dataset"
    _write(output)
    manifest_path = output / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["dataset_id"] = "tampered"
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="hash mismatch"):
        validate_complete_dataset(output)


def test_asset_paths_cannot_escape_category(tmp_path: Path) -> None:
    backend = CsvNpyJpegBackend(tmp_path / "dataset", _manifest())
    try:
        with pytest.raises(ValueError, match="stay inside"):
            backend.write_array("lidar", "../escape.npy", np.array([1]))
        with pytest.raises(ValueError, match="unsupported array category"):
            backend.write_array("unknown", "x.npy", np.array([1]))
    finally:
        backend.abort()
