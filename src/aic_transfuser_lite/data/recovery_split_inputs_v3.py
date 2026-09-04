from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml

from .split_v3 import SplitGroupKey, SplitRunRecord


def build_recovery_split_inputs_v3(
    raw_root: str | Path,
    *,
    map_or_course_id: str,
    vehicle_profile_id: str,
    source_dataset_id: str,
) -> tuple[dict[str, Any], ...]:
    """Build whole-run split inputs from accepted recovery recording artifacts."""

    root = Path(raw_root)
    if not root.is_dir():
        raise FileNotFoundError(f"recovery raw root is missing: {root}")
    values: list[dict[str, Any]] = []
    for run_root in sorted(path for path in root.iterdir() if path.is_dir()):
        manifests = tuple(run_root.glob("*.recording_manifest.yaml"))
        if len(manifests) != 1:
            raise ValueError(
                f"run {run_root.name!r} requires exactly one recording manifest"
            )
        manifest = yaml.safe_load(manifests[0].read_text(encoding="utf-8"))
        if not isinstance(manifest, dict) or manifest.get("status") != "complete":
            raise ValueError(f"run {run_root.name!r} recording is not complete")
        run_id = str(manifest.get("run_id", ""))
        if run_id != run_root.name:
            raise ValueError(f"run directory/manifest mismatch: {run_root.name!r}")
        source_hash = _recorded_bag_hash(run_root / "SHA256SUMS")
        record = SplitRunRecord(
            run_id=run_id,
            source_hash=source_hash,
            group=SplitGroupKey(
                scenario_id=str(manifest.get("collection_case_id", "")),
                run_family_id=run_id,
                collection_session_id="unknown",
                map_or_course_id=map_or_course_id,
                vehicle_profile_id=vehicle_profile_id,
                controller_profile_id=str(manifest.get("teacher_controller_id", "")),
                source_dataset_id=source_dataset_id,
            ),
        )
        record.validate()
        values.append(
            {
                "run_id": record.run_id,
                "source_hash": record.source_hash,
                "group": asdict(record.group),
                "trajectory_fingerprint": record.trajectory_fingerprint,
            }
        )
    if not values:
        raise ValueError("recovery raw root contains no run directories")
    return tuple(values)


def _recorded_bag_hash(path: Path) -> str:
    if not path.is_file():
        raise FileNotFoundError(f"run checksum file is missing: {path}")
    rows = [line.split() for line in path.read_text(encoding="utf-8").splitlines() if line]
    candidates = [parts[0] for parts in rows if len(parts) >= 2 and ".mcap" in parts[-1]]
    if len(candidates) != 1:
        raise ValueError(f"checksum file requires exactly one MCAP entry: {path}")
    digest = candidates[0]
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ValueError(f"invalid recorded bag SHA-256: {digest!r}")
    return digest
