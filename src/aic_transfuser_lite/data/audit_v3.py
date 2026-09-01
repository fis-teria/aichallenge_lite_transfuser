from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pandas as pd
import yaml

from .canonical_schema_v3 import DATASET_SCHEMA_VERSION_V3
from .storage_v3 import validate_complete_dataset


class AuditStatus(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"
    NOT_EVALUATED = "NOT_EVALUATED"


@dataclass(frozen=True)
class AuditGate:
    status: AuditStatus
    reason: str
    value: Any = None
    threshold: Any = None


@dataclass(frozen=True)
class AuditReportV3:
    schema_version: str
    dataset_id: str
    run_summary: Mapping[str, Any]
    synchronization: Mapping[str, Any]
    missingness: Mapping[str, Any]
    distributions: Mapping[str, Any]
    capabilities: Mapping[str, Any]
    split_leakage: Mapping[str, Any]
    storage: Mapping[str, Any]
    gates: Mapping[str, AuditGate]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        for gate in value["gates"].values():
            gate["status"] = gate["status"].value
        return value

    @property
    def overall_status(self) -> AuditStatus:
        statuses = {gate.status for gate in self.gates.values()}
        if AuditStatus.FAIL in statuses:
            return AuditStatus.FAIL
        if AuditStatus.WARN in statuses:
            return AuditStatus.WARN
        if AuditStatus.PASS in statuses:
            return AuditStatus.PASS
        return AuditStatus.NOT_EVALUATED


def audit_dataset(
    dataset_root: str | Path, *, output_directory: str | Path | None = None
) -> AuditReportV3:
    """Dispatch a canonical audit by schema version and optionally write JSON/CSV."""

    root = Path(dataset_root).resolve()
    manifest_path = root / "manifest.yaml"
    if manifest_path.is_file():
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        schema_version = manifest.get("schema_version") if isinstance(manifest, dict) else None
        if schema_version != DATASET_SCHEMA_VERSION_V3:
            raise ValueError(f"unsupported canonical schema version: {schema_version!r}")
        report = audit_dataset_v3(root)
    else:
        metadata_path = root / "metadata.yaml"
        if not metadata_path.is_file():
            raise FileNotFoundError(f"No versioned dataset manifest found in {root}")
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict) or int(metadata.get("format_version", -1)) != 2:
            raise ValueError("unsupported legacy dataset schema")
        report = _audit_dataset_v2(root, metadata)
    if output_directory is not None:
        write_audit_report(report, output_directory)
    return report


def audit_dataset_v3(dataset_root: str | Path) -> AuditReportV3:
    root = Path(dataset_root).resolve()
    manifest = validate_complete_dataset(root)
    samples_path = root / "samples.csv"
    frame = pd.read_csv(samples_path) if samples_path.stat().st_size else pd.DataFrame()
    runs = manifest.get("runs", [])
    if not isinstance(runs, list):
        raise ValueError("manifest.runs must be a list")
    gates: dict[str, AuditGate] = {}

    duplicate_samples = int(frame["sample_id"].duplicated().sum()) if "sample_id" in frame else 0
    asset_columns = [
        name for name in ("image_path", "lidar_path", "lidar_valid_path", "trajectory_path") if name in frame
    ]
    duplicate_assets = {
        name: int(frame[name].dropna().duplicated().sum()) for name in asset_columns
    }
    gates["duplicate_sample"] = AuditGate(
        AuditStatus.PASS if duplicate_samples == 0 else AuditStatus.FAIL,
        "sample IDs must be unique",
        duplicate_samples,
        0,
    )
    duplicate_asset_count = sum(duplicate_assets.values())
    gates["duplicate_asset"] = AuditGate(
        AuditStatus.PASS if duplicate_asset_count == 0 else AuditStatus.FAIL,
        "asset references must not be shared by samples",
        duplicate_assets,
        0,
    )

    rate_by_run: dict[str, float | None] = {}
    if not frame.empty and {"run_id", "grid_stamp_ns"}.issubset(frame):
        for run_id, group in frame.groupby("run_id"):
            stamps = np.sort(group["grid_stamp_ns"].to_numpy(dtype=np.int64))
            rate_by_run[str(run_id)] = (
                float(1e9 / np.median(np.diff(stamps))) if len(stamps) > 1 else None
            )
    finite_rates = [value for value in rate_by_run.values() if value is not None]
    rates_valid = bool(finite_rates) and all(9.8 <= value <= 10.2 for value in finite_rates)
    gates["effective_rate"] = AuditGate(
        AuditStatus.PASS if rates_valid else (AuditStatus.NOT_EVALUATED if not finite_rates else AuditStatus.FAIL),
        "every evaluable run must be within 9.8-10.2 Hz",
        rate_by_run,
        [9.8, 10.2],
    )

    skew = _distribution(frame, "lidar_delta_ms", absolute=True)
    if skew is None:
        gates["camera_lidar_skew"] = AuditGate(AuditStatus.NOT_EVALUATED, "lidar_delta_ms unavailable")
    else:
        gates["camera_lidar_skew"] = AuditGate(
            AuditStatus.PASS if skew["p95"] <= 30.0 else AuditStatus.FAIL,
            "absolute Camera-LiDAR p95 skew must be <=30 ms",
            skew["p95"],
            30.0,
        )

    storage = _audit_storage(root, manifest)
    gates["storage_integrity"] = AuditGate(
        AuditStatus.PASS if not storage["errors"] else AuditStatus.FAIL,
        "all manifest files must exist with matching size and SHA-256",
        storage["errors"],
        [],
    )
    gates["split_leakage"] = AuditGate(
        AuditStatus.NOT_EVALUATED,
        "canonical dataset has no split manifest; evaluate in V3 split task",
    )
    gates["topic_presence"] = AuditGate(
        AuditStatus.NOT_EVALUATED,
        "topic inventory is external to this storage manifest",
    )
    gates["event_labels"] = AuditGate(
        AuditStatus.NOT_EVALUATED,
        "stop/collision/recovery labels are absent unless explicitly annotated",
    )

    future_invalid_fraction = _fraction_invalid_counts(frame)
    actual_steering_missing = _invalid_boolean_fraction(frame, "actual_steering_valid")
    capabilities: dict[str, int] = {}
    for run in runs:
        if isinstance(run, dict):
            for capability in run.get("capabilities", []):
                capabilities[str(capability)] = capabilities.get(str(capability), 0) + 1

    report = AuditReportV3(
        schema_version=DATASET_SCHEMA_VERSION_V3,
        dataset_id=str(manifest["dataset_id"]),
        run_summary={
            "run_count": len({str(run.get("run_id")) for run in runs if isinstance(run, dict)}),
            "scenario_count": len({str(run.get("scenario_id")) for run in runs if isinstance(run, dict)}),
            "segment_count": int(frame["segment_id"].nunique()) if "segment_id" in frame else 0,
            "sample_count": int(len(frame)),
            "split_counts": None,
            "topic_presence": None,
            "timestamp_source_distribution": None,
        },
        synchronization={
            "effective_rate_hz_by_run": rate_by_run,
            "camera_lidar_skew_ms": skew,
            "camera_delta_ms": _distribution(frame, "camera_delta_ms", absolute=True),
            "state_endpoint_delta_ms": _distribution(frame, "max_state_endpoint_delta_ms", absolute=True),
            "nominal_command_age_ms": _distribution(frame, "nominal_command_age_ms"),
            "final_command_age_ms": _distribution(frame, "final_command_age_ms"),
        },
        missingness={
            "future_invalid_fraction": future_invalid_fraction,
            "actual_steering_missing_fraction": actual_steering_missing,
            "lidar_valid_fraction": _lidar_valid_fraction(root, frame),
        },
        distributions={
            name: _distribution(frame, name)
            for name in (
                "velocity_longitudinal_mps",
                "velocity_lateral_mps",
                "yaw_rate_rps",
                "actual_steering_rad",
            )
        },
        capabilities={"run_count_by_capability": capabilities, "event_valid_counts": None},
        split_leakage={"status": AuditStatus.NOT_EVALUATED.value, "reason": gates["split_leakage"].reason},
        storage=storage,
        gates=gates,
    )
    return report


def write_audit_report(report: AuditReportV3, output_directory: str | Path) -> None:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    (output / "audit_report.json").write_text(
        json.dumps(report.to_dict(), indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    with (output / "audit_gates.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("gate", "status", "reason", "value", "threshold"))
        writer.writeheader()
        for name, gate in report.gates.items():
            writer.writerow(
                {
                    "gate": name,
                    "status": gate.status.value,
                    "reason": gate.reason,
                    "value": json.dumps(gate.value, ensure_ascii=False),
                    "threshold": json.dumps(gate.threshold, ensure_ascii=False),
                }
            )


def _audit_dataset_v2(root: Path, metadata: dict[str, Any]) -> AuditReportV3:
    frame_path = root / "index.csv"
    frame = pd.read_csv(frame_path) if frame_path.is_file() else pd.DataFrame()
    gate = AuditGate(
        AuditStatus.NOT_EVALUATED,
        "Dataset V2 is recognized; V3-only canonical gates are not applied",
    )
    return AuditReportV3(
        schema_version="2",
        dataset_id=str(root.name),
        run_summary={
            "sample_count": int(len(frame)),
            "run_count": int(frame["run_id"].nunique()) if "run_id" in frame else None,
            "metadata_rows": metadata.get("rows"),
        },
        synchronization={},
        missingness={},
        distributions={},
        capabilities={},
        split_leakage={},
        storage={},
        gates={"v3_only": gate},
    )


def _audit_storage(root: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    total = 0
    files = manifest.get("files", [])
    for record in files if isinstance(files, list) else []:
        if not isinstance(record, dict):
            errors.append("invalid_file_record")
            continue
        relative = str(record.get("path", ""))
        path = root / relative
        if not path.is_file():
            errors.append(f"missing:{relative}")
            continue
        size = path.stat().st_size
        total += size
        if size != int(record.get("size_bytes", -1)):
            errors.append(f"size:{relative}")
        if _sha256(path) != record.get("sha256"):
            errors.append(f"sha256:{relative}")
    return {"file_count": len(files) if isinstance(files, list) else 0, "size_bytes": total, "errors": errors}


def _distribution(frame: pd.DataFrame, column: str, *, absolute: bool = False) -> dict[str, float] | None:
    if column not in frame:
        return None
    values = pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=np.float64)
    values = values[np.isfinite(values)]
    if absolute:
        values = np.abs(values)
    if not len(values):
        return None
    return {
        "count": int(len(values)),
        "min": float(values.min()),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
        "mean": float(values.mean()),
    }


def _fraction_invalid_counts(frame: pd.DataFrame) -> float | None:
    required = {"future_valid_count", "future_step_count"}
    if frame.empty or not required.issubset(frame):
        return None
    valid = pd.to_numeric(frame["future_valid_count"], errors="coerce").sum()
    total = pd.to_numeric(frame["future_step_count"], errors="coerce").sum()
    return None if total <= 0 else float(1.0 - valid / total)


def _invalid_boolean_fraction(frame: pd.DataFrame, column: str) -> float | None:
    if frame.empty or column not in frame:
        return None
    values = frame[column].astype(str).str.lower().map({"true": True, "false": False, "1": True, "0": False})
    return float((~values.fillna(False)).mean())


def _lidar_valid_fraction(root: Path, frame: pd.DataFrame) -> float | None:
    if frame.empty or "lidar_valid_path" not in frame:
        return None
    valid_count = 0
    total_count = 0
    for relative in frame["lidar_valid_path"].dropna():
        values = np.load(root / str(relative), allow_pickle=False)
        valid_count += int(np.asarray(values).astype(bool).sum())
        total_count += int(np.asarray(values).size)
    return None if total_count == 0 else valid_count / total_count


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
