from __future__ import annotations

from pathlib import Path

import pytest

from aic_transfuser_lite.data.split_v3 import (
    SplitConfigV3,
    SplitGroupKey,
    SplitRunRecord,
    build_split_manifest_v3,
    group_id,
    load_split_config_v3,
)


CONFIG = Path(__file__).parents[1] / "configs" / "data" / "split_v3.yaml"


def _group(index: int, **overrides: str) -> SplitGroupKey:
    values = {
        "scenario_id": f"scenario{index}",
        "run_family_id": f"family{index}",
        "collection_session_id": f"session{index}",
        "map_or_course_id": "course",
        "vehicle_profile_id": "vehicle",
        "controller_profile_id": "controller",
        "source_dataset_id": "dataset",
    }
    values.update(overrides)
    return SplitGroupKey(**values)


def _run(index: int, **group_overrides: str) -> SplitRunRecord:
    return SplitRunRecord(
        run_id=f"run{index}",
        source_hash=f"{index:064x}",
        group=_group(index, **group_overrides),
        trajectory_fingerprint=f"trajectory{index}",
    )


def test_assignment_is_deterministic_hashed_and_not_frame_random() -> None:
    config = load_split_config_v3(CONFIG)
    runs = [_run(index) for index in range(10)]
    first = build_split_manifest_v3(runs, dataset_manifest_sha256="a" * 64, config=config)
    second = build_split_manifest_v3(reversed(runs), dataset_manifest_sha256="a" * 64, config=config)
    assert first == second
    assert len(first["assignments"]) == len(runs)
    assert first["manifest_sha256"] and first["leakage"]["status"] == "PASS"


def test_source_session_family_and_fingerprint_never_cross_splits() -> None:
    config = load_split_config_v3(CONFIG)
    runs = [
        _run(1),
        SplitRunRecord("source_copy", _run(1).source_hash, _group(2), "other"),
        _run(3, collection_session_id="session1"),
        SplitRunRecord("fingerprint_copy", "4" * 64, _group(4), "trajectory1"),
    ]
    manifest = build_split_manifest_v3(runs, dataset_manifest_sha256="b" * 64, config=config)
    assert len({item["split"] for item in manifest["assignments"]}) == 1
    assert len({item["component_id"] for item in manifest["assignments"]}) == 1


def test_fixed_benchmark_membership_is_stable_when_dataset_grows() -> None:
    base = _run(1)
    benchmark_id = group_id(base.group)
    config = SplitConfigV3(
        split_seed=42,
        ratios={"train": 0.7, "validation": 0.15, "test": 0.15},
        fixed_benchmark_group_ids=(benchmark_id,),
    )
    first = build_split_manifest_v3([base], dataset_manifest_sha256="c" * 64, config=config)
    grown = build_split_manifest_v3(
        [base, _run(2), _run(3)], dataset_manifest_sha256="d" * 64, config=config
    )
    assert first["assignments"][0]["split"] == "benchmark"
    assert next(item for item in grown["assignments"] if item["run_id"] == base.run_id)["split"] == "benchmark"


def test_missing_group_fields_become_explicit_unknown() -> None:
    value = SplitGroupKey("scenario", "", "", "course", "vehicle", "controller", "dataset")
    normalized = value.normalized()
    assert normalized.run_family_id == "unknown"
    assert normalized.collection_session_id == "unknown"


def test_duplicate_run_and_invalid_hash_are_rejected() -> None:
    config = load_split_config_v3(CONFIG)
    with pytest.raises(ValueError, match="duplicate run_id"):
        build_split_manifest_v3([_run(1), _run(1)], dataset_manifest_sha256="e" * 64, config=config)
    with pytest.raises(ValueError, match="source_hash"):
        build_split_manifest_v3(
            [SplitRunRecord("run", "bad", _group(1))],
            dataset_manifest_sha256="e" * 64,
            config=config,
        )


def test_split_config_rejects_ratio_and_field_drift() -> None:
    with pytest.raises(ValueError, match="sum to one"):
        SplitConfigV3(
            split_seed=1,
            ratios={"train": 0.9, "validation": 0.2, "test": 0.1},
            fixed_benchmark_group_ids=(),
        ).validate()
