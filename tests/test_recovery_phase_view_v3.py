from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import pytest

from aic_transfuser_lite.data.behavior_view_v1 import load_behavior_view_v1
from aic_transfuser_lite.data.recovery_phase_view_v3 import (
    RecoveryPhaseLabelV3,
    RecoveryPhaseIntervalV3,
    build_recovery_phase_labels_v3,
    classify_recovery_phase_v3,
    load_recovery_phase_intervals_v3,
    load_recovery_phase_view_v3,
    write_recovery_behavior_view_v1,
    write_recovery_phase_view_v3,
)


@dataclass(frozen=True)
class _Pose:
    timestamp_ns: int
    x_world_m: float
    y_world_m: float
    yaw_world_rad: float


def _write_reference(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"))
        for index in range(20):
            writer.writerow((index, index, 0.0, 0.0, 0.0, 1.0, 0.0))


def _write_collection_reference(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("point_id", "frame_id", "x_m", "y_m", "heading_rad"))
        for index in range(20):
            writer.writerow((index, "map", index, 0.0, 0.0))


def _write_intervals(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "segment_id", "phase", "side", "offset_m", "geometry",
            "start_point_id", "end_point_id", "start_s_m", "end_s_m",
            "training_eligible",
        ))
        writer.writerow(("left", "approach", "left", 0.35, "straight", 2, 3, 2.0, 3.0, "false"))
        writer.writerow(("left", "hold", "left", 0.35, "straight", 4, 5, 4.0, 5.0, "true"))
        writer.writerow(("left", "recovery", "left", 0.35, "straight", 6, 7, 6.0, 7.0, "true"))


def test_interval_loader_and_classifier_are_strict(tmp_path: Path) -> None:
    path = tmp_path / "intervals.csv"
    _write_intervals(path)
    intervals = load_recovery_phase_intervals_v3(path)
    assert classify_recovery_phase_v3(4, intervals) is not None
    assert classify_recovery_phase_v3(4, intervals).phase == "hold"  # type: ignore[union-attr]
    assert classify_recovery_phase_v3(10, intervals) is None
    assert intervals[1].signed_offset_m == pytest.approx(0.35)


def test_phase_labels_align_pose_in_si_units(tmp_path: Path) -> None:
    generated = tmp_path / "generated.csv"
    base = tmp_path / "base.csv"
    intervals = tmp_path / "intervals.csv"
    _write_reference(generated)
    _write_collection_reference(base)
    _write_intervals(intervals)
    rows = [
        {"sample_id": "sample", "run_id": "run", "grid_stamp_ns": "1000000000"},
    ]
    labels = build_recovery_phase_labels_v3(
        rows,
        (_Pose(1001000000, 4.25, 0.35, 0.0),),
        run_id="run",
        generated_reference_path=generated,
        base_reference_path=base,
        intervals_path=intervals,
        max_pose_delta_ms=5.0,
    )
    assert len(labels) == 1
    assert labels[0].phase == "hold"
    assert labels[0].training_eligible
    assert labels[0].pose_delta_ms == pytest.approx(1.0)
    assert labels[0].base_lateral_offset_m == pytest.approx(0.35)


def test_phase_labels_fail_on_stale_pose(tmp_path: Path) -> None:
    generated = tmp_path / "generated.csv"
    base = tmp_path / "base.csv"
    intervals = tmp_path / "intervals.csv"
    _write_reference(generated)
    _write_collection_reference(base)
    _write_intervals(intervals)
    with pytest.raises(ValueError, match="nearest pose delta"):
        build_recovery_phase_labels_v3(
            ({"sample_id": "sample", "run_id": "run", "grid_stamp_ns": "1000000000"},),
            (_Pose(1100000000, 4.0, 0.0, 0.0),),
            run_id="run",
            generated_reference_path=generated,
            base_reference_path=base,
            intervals_path=intervals,
            max_pose_delta_ms=50.0,
        )


def test_phase_view_rejects_duplicate_sample_ids(tmp_path: Path) -> None:
    label = RecoveryPhaseLabelV3(
        sample_id="duplicate",
        run_id="run",
        grid_stamp_ns=1,
        phase="hold",
        segment_id="left",
        side="left",
        geometry="straight",
        requested_signed_offset_m=0.35,
        generated_point_id=4,
        pose_source_stamp_ns=1,
        pose_delta_ms=0.0,
        generated_lateral_error_m=0.0,
        base_lateral_offset_m=0.35,
        training_eligible=True,
    )
    with pytest.raises(ValueError, match="duplicate sample_id"):
        write_recovery_phase_view_v3(
            tmp_path / "view",
            dataset_manifest_sha256="a" * 64,
            labels=(label, label),
            source_records=(),
        )


def test_phase_and_behavior_views_round_trip(tmp_path: Path) -> None:
    label = RecoveryPhaseLabelV3(
        sample_id="sample",
        run_id="run",
        grid_stamp_ns=1_000_000_000,
        phase="recovery",
        segment_id="left",
        side="left",
        geometry="straight",
        requested_signed_offset_m=0.35,
        generated_point_id=6,
        pose_source_stamp_ns=1_001_000_000,
        pose_delta_ms=1.0,
        generated_lateral_error_m=0.01,
        base_lateral_offset_m=0.2,
        training_eligible=True,
    )
    dataset_sha = "b" * 64
    phase_root = tmp_path / "phase"
    write_recovery_phase_view_v3(
        phase_root,
        dataset_manifest_sha256=dataset_sha,
        labels=(label,),
        source_records=(),
    )
    loaded = load_recovery_phase_view_v3(
        phase_root,
        dataset_manifest_sha256=dataset_sha,
    )
    assert loaded == (label,)

    behavior_root = tmp_path / "behavior"
    write_recovery_behavior_view_v1(
        behavior_root,
        dataset_manifest_sha256=dataset_sha,
        phase_view_manifest_sha256="c" * 64,
        labels=loaded,
    )
    behavior = load_behavior_view_v1(
        behavior_root,
        dataset_manifest_sha256=dataset_sha,
    )["sample"]
    assert behavior["behavior_label"] == "FORWARD_NORMAL"
    assert behavior["behavior_side_label"] == "NONE"
    assert behavior["source"] == "recovery_reference_phase"
