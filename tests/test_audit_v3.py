from __future__ import annotations

from pathlib import Path

import pandas as pd
import pytest
import yaml

from aic_transfuser_lite.data.audit_v3 import AuditStatus, audit_dataset
from aic_transfuser_lite.data.canonical_converter_v3 import (
    write_prepared_dataset_v3,
)
from test_dataset_v3_converter import _convert


def _dataset(tmp_path: Path) -> Path:
    output = tmp_path / "dataset"
    prepared = _convert()
    write_prepared_dataset_v3(
        output,
        dataset_id="dataset01",
        topic_profile_id="default",
        runs=(prepared,),
        jpeg_quality=90,
    )
    return output


def test_v3_audit_reports_required_sections_and_gate_vocabulary(tmp_path: Path) -> None:
    root = _dataset(tmp_path)
    report = audit_dataset(root, output_directory=tmp_path / "audit")
    assert report.schema_version == "aic_canonical_dataset_v3"
    assert report.run_summary["sample_count"] > 0
    assert report.synchronization["camera_lidar_skew_ms"]["p95"] == 0.0
    assert report.missingness["actual_steering_missing_fraction"] == 1.0
    assert report.missingness["lidar_valid_fraction"] == pytest.approx(0.75)
    assert report.capabilities["run_count_by_capability"]["trajectory_label"] == 1
    assert {gate.status.value for gate in report.gates.values()} <= {
        "PASS",
        "WARN",
        "FAIL",
        "NOT_EVALUATED",
    }
    assert report.gates["split_leakage"].status is AuditStatus.NOT_EVALUATED
    assert (tmp_path / "audit/audit_report.json").is_file()
    assert (tmp_path / "audit/audit_gates.csv").is_file()


def test_storage_tamper_is_reported_as_fail(tmp_path: Path) -> None:
    root = _dataset(tmp_path)
    lidar = next((root / "lidar").rglob("*.npy"))
    lidar.write_bytes(b"tampered")
    report = audit_dataset(root)
    assert report.gates["storage_integrity"].status is AuditStatus.FAIL
    assert report.missingness["lidar_valid_fraction"] is None
    assert any(error.startswith("sha256:") or error.startswith("size:") for error in report.storage["errors"])


def test_duplicate_sample_is_a_fail(tmp_path: Path) -> None:
    root = _dataset(tmp_path)
    samples = pd.read_csv(root / "samples.csv")
    samples = pd.concat([samples, samples.iloc[[0]]], ignore_index=True)
    samples.to_csv(root / "samples.csv", index=False)
    report = audit_dataset(root)
    assert report.gates["duplicate_sample"].status is AuditStatus.FAIL


def test_schema_dispatch_recognizes_v2_without_v0_columns(tmp_path: Path) -> None:
    root = tmp_path / "v2"
    root.mkdir()
    (root / "metadata.yaml").write_text(
        yaml.safe_dump({"format_version": 2, "rows": 1}), encoding="utf-8"
    )
    pd.DataFrame([{"run_id": "run01", "new_v2_column": 3}]).to_csv(
        root / "index.csv", index=False
    )
    report = audit_dataset(root)
    assert report.schema_version == "2"
    assert report.run_summary["run_count"] == 1
    assert report.gates["v3_only"].status is AuditStatus.NOT_EVALUATED


def test_unknown_schema_is_rejected(tmp_path: Path) -> None:
    root = tmp_path / "bad"
    root.mkdir()
    (root / "manifest.yaml").write_text(
        yaml.safe_dump({"schema_version": "future"}), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="unsupported canonical schema"):
        audit_dataset(root)
