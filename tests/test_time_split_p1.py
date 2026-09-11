from __future__ import annotations

import copy
import hashlib

import pytest

from aic_transfuser_lite.data.time_split_v1 import (
    assert_split_membership, audit_initialization_lineage, build_time_split,
    command_comparison_plan, validate_time_split, verify_time_split_sources,
)


def records() -> list[dict]:
    return [{"run_id": f"{speed}kmh_run{i:02d}", "speed_cap_kmh": speed,
             "sources": [{"path": f"runs/{speed}kmh_run{i:02d}/bag/data.db3",
                          "sha256": hashlib.sha256(f"{speed}/{i}".encode()).hexdigest()}]}
            for speed in (5, 8) for i in range(1, 11)]


def manifest() -> dict:
    return build_time_split(records(), receipt_sha256="a" * 64, seed=42)


def test_split_is_order_independent_whole_run_and_sealed() -> None:
    m = manifest()
    assert m == build_time_split(reversed(records()), receipt_sha256="a" * 64, seed=42)
    for speed in (5, 8):
        assert [sum(r["speed_cap_kmh"] == speed and r["split"] == split for r in m["runs"])
                for split in ("train", "validation", "test")] == [6, 2, 2]
    with pytest.raises(ValueError, match="not been hash verified"):
        assert_split_membership(m, [], split="train")
    test_id = next(r["run_id"] for r in m["runs"] if r["split"] == "test")
    with pytest.raises(ValueError, match="crosses"):
        assert_split_membership(m, [test_id], split="train", require_verified=False)
    m["runs"][0]["split"] = "test" if m["runs"][0]["split"] == "train" else "train"
    with pytest.raises(ValueError):
        validate_time_split(m)


@pytest.mark.parametrize("field", ["run_id", "hash", "path"])
def test_duplicate_identity_or_raw_content_is_rejected(field: str) -> None:
    r = records()
    if field == "run_id":
        r[1][field] = r[0][field]
    else:
        key = "sha256" if field == "hash" else "path"
        r[1]["sources"][0][key] = r[0]["sources"][0][key]
    with pytest.raises(ValueError, match="duplicate"):
        build_time_split(r, receipt_sha256="a" * 64)


def test_destination_verification_and_digest_failures(tmp_path) -> None:
    m = manifest()
    for speed in (5, 8):
        for i in range(1, 11):
            path = tmp_path / f"runs/{speed}kmh_run{i:02d}/bag/data.db3"
            path.parent.mkdir(parents=True)
            path.write_bytes(f"{speed}/{i}".encode())
    verified = verify_time_split_sources(m, tmp_path)
    assert not m["sources_verified"] and verified["sources_verified"]
    assert verified["manifest_sha256"] == m["manifest_sha256"]
    validate_time_split(verified, require_verified=True)
    (tmp_path / m["runs"][0]["sources"][0]["path"]).write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash mismatch"):
        verify_time_split_sources(m, tmp_path)


def test_unknown_and_overlap_lineage_never_claim_unseen_test() -> None:
    m = manifest()
    assert audit_initialization_lineage(m, mode="scratch")["status"] == "SCRATCH"
    unknown = audit_initialization_lineage(m, mode="finetune", source_checkpoint_sha256="b" * 64)
    assert unknown["status"] == "UNKNOWN" and unknown["scratch_comparison_required"]
    digest = next(r["sources"][0]["sha256"] for r in m["runs"] if r["split"] == "test")
    overlap = audit_initialization_lineage(m, mode="finetune", source_checkpoint_sha256="b" * 64,
                                           seen_source_sha256=[digest])
    assert overlap["status"] == "HELDOUT_OVERLAP" and overlap["scratch_comparison_required"]


def test_comparison_has_same_bounded_presentation_budget_and_initialization() -> None:
    plan = command_comparison_plan(manifest(), seed=7, max_anchors=120, max_optimizer_steps=3)
    assert [arm["use_command_history"] for arm in plan["arms"]] == [False, True]
    assert plan["test_usage"] == "sealed_until_final_evaluation"
    assert plan["initialization"] == "scratch"
    for kwargs in ({"seed": True, "max_anchors": 1, "max_optimizer_steps": 1},
                   {"seed": 1, "max_anchors": 0, "max_optimizer_steps": 1}):
        with pytest.raises(ValueError):
            command_comparison_plan(manifest(), **kwargs)


@pytest.mark.parametrize("path", ["../escape.db3", "/abs/db3", "C:/drive.db3", "x\\escape.db3"])
def test_source_paths_cannot_escape(path: str) -> None:
    r = records()
    r[0]["sources"][0]["path"] = path
    with pytest.raises(ValueError, match="relative POSIX"):
        build_time_split(r, receipt_sha256="a" * 64)
