from __future__ import annotations

from pathlib import Path

import pytest

from aic_transfuser_lite.data.recovery_split_inputs_v3 import (
    build_recovery_split_inputs_v3,
)


def _write_run(root: Path, run_id: str, digest: str = "a" * 64) -> None:
    run = root / run_id
    run.mkdir(parents=True)
    (run / f"{run_id}.recording_manifest.yaml").write_text(
        "\n".join(
            (
                "format_version: 3",
                "status: complete",
                f"run_id: {run_id}",
                "collection_case_id: offset_left_near",
                "teacher_controller_id: recovery_teacher_v3",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    (run / "SHA256SUMS").write_text(
        f"{digest}  /data/{run_id}/{run_id}_0.mcap.zstd\n",
        encoding="utf-8",
    )


def test_build_recovery_split_inputs_keeps_runs_independent(tmp_path: Path) -> None:
    _write_run(tmp_path, "run01", "a" * 64)
    _write_run(tmp_path, "run02", "b" * 64)
    records = build_recovery_split_inputs_v3(
        tmp_path,
        map_or_course_id="d1",
        vehicle_profile_id="sim",
        source_dataset_id="recovery",
    )
    assert [record["run_id"] for record in records] == ["run01", "run02"]
    assert records[0]["group"]["scenario_id"] == "offset_left_near"
    assert records[0]["group"]["run_family_id"] == "run01"
    assert records[0]["group"]["collection_session_id"] == "unknown"


def test_build_recovery_split_inputs_rejects_bad_digest(tmp_path: Path) -> None:
    _write_run(tmp_path, "run01", "bad")
    with pytest.raises(ValueError, match="invalid recorded bag SHA-256"):
        build_recovery_split_inputs_v3(
            tmp_path,
            map_or_course_id="d1",
            vehicle_profile_id="sim",
            source_dataset_id="recovery",
        )
