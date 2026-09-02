from __future__ import annotations

import csv
import hashlib
import json

from aic_transfuser_lite.contracts.behavior_v1 import BehaviorClassV1, BehaviorSideV1
from aic_transfuser_lite.data.behavior_view_v1 import (
    BagStateEventV1,
    BehaviorAnnotationV1,
    align_behavior_annotations_v1,
    behavior_alignment_anchors_v1,
    load_behavior_view_v1,
    parse_speed_diagnostics_v1,
)
from aic_transfuser_lite.contracts.model_batch_v3 import TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3
from aic_transfuser_lite.training.losses_v3 import LossWeightsV3, compute_losses_v3
import torch
import pytest


def _diag(
    wall: float,
    loop: int,
    *,
    avoid: str = "IDLE",
    candidate: str = "none",
    brake: str = "none",
    recovery: str = "MONITORING",
) -> str:
    return (
        f"[node] [INFO] [{wall:.3f}] [mpc_controller]: [speed.diag] "
        f"loop_seq={loop} steering_mode=MPC avoid_state={avoid} avoid_vehicle=d3 "
        f"brake_vehicle={brake} avoid_candidate={candidate} recovery_state={recovery}"
    )


def test_alignment_uses_grounded_and_race_ready_start_finish() -> None:
    lines = [
        "[node] [INFO] [101.000] [orchestrator]: start condition met: /awsim/state == Grounded"
    ]
    events = (
        BagStateEventV1(1_000_000_000, "Grounded"),
        BagStateEventV1(2_000_000_000, "Ready"),
        BagStateEventV1(3_000_000_000, "Start"),
        BagStateEventV1(8_000_000_000, "Finish"),
    )
    offset, interval = behavior_alignment_anchors_v1(lines, events)
    assert offset == 100.0
    assert interval == (3_000_000_000, 8_000_000_000)


def test_priority_direction_and_transition_uncertainty_are_explicit() -> None:
    lines = [
        _diag(103.0, 1),
        _diag(103.5, 2),
        _diag(104.0, 3, avoid="AVOIDING", candidate="RIGHT_NORMAL", brake="d3"),
        _diag(104.4, 4, avoid="AVOIDING", candidate="RIGHT_NORMAL", brake="d3"),
        _diag(104.8, 5, avoid="AVOIDING", candidate="RIGHT_NORMAL", recovery="BACKING_UP"),
    ]
    diagnostics, rejected = parse_speed_diagnostics_v1(lines, wall_to_sim_offset_sec=100.0)
    assert rejected == 0
    assert diagnostics[2].behavior_class == BehaviorClassV1.FORWARD_AVOID
    assert diagnostics[2].behavior_side == BehaviorSideV1.RIGHT
    assert diagnostics[4].behavior_class == BehaviorClassV1.RECOVERY
    rows = [
        {"sample_id": "normal", "run_id": "run01", "grid_stamp_ns": "3200000000"},
        {"sample_id": "transition", "run_id": "run01", "grid_stamp_ns": "3700000000"},
        {"sample_id": "avoid", "run_id": "run01", "grid_stamp_ns": "4200000000"},
    ]
    result = align_behavior_annotations_v1(
        rows, diagnostics, run_id="run01", active_interval_ns=(3_000_000_000, 8_000_000_000)
    )
    assert result[0].behavior_label == "FORWARD_NORMAL" and result[0].behavior_valid
    assert result[1].invalid_reason == "transition_uncertain" and not result[1].behavior_valid
    assert result[2].behavior_label == "FORWARD_AVOID"
    assert result[2].behavior_side_label == "RIGHT"


def test_old_partial_speed_diag_is_rejected_instead_of_becoming_follow() -> None:
    partial = "[node] [INFO] [101.0] [mpc]: [speed.diag] loop_seq=1 avoid_state=IDLE"
    diagnostics, rejected = parse_speed_diagnostics_v1([partial], wall_to_sim_offset_sec=100.0)
    assert diagnostics == ()
    assert rejected == 1


def test_behavior_losses_use_masks_and_backpropagate() -> None:
    logits = torch.zeros(2, 5, requires_grad=True)
    side_logits = torch.zeros(2, 3, requires_grad=True)
    output = ModelOutputV3(
        trajectory_xy=torch.zeros(2, 1, 1, 2),
        trajectory_speed_mps=torch.ones(2, 1, 1),
        candidate_logits=torch.zeros(2, 1),
        behavior_logits=logits,
        behavior_side_logits=side_logits,
    )
    targets = TrainingTargetsV3(
        trajectory_xy_m=torch.zeros(2, 1, 2), trajectory_mask=torch.ones(2, 1, dtype=torch.bool),
        speed_mps=torch.ones(2, 1), speed_mask=torch.ones(2, 1, dtype=torch.bool),
        behavior_class=torch.tensor([2, -1]), behavior_mask=torch.tensor([True, False]),
        behavior_side=torch.tensor([2, -1]), behavior_side_mask=torch.tensor([True, False]),
    )
    report = compute_losses_v3(
        output, targets, LossWeightsV3(behavior=0.2, behavior_side=0.1)
    )
    assert set(report.raw) == {"trajectory", "speed_profile", "behavior", "behavior_side"}
    report.total.backward()
    assert logits.grad is not None and side_logits.grad is not None


def test_behavior_view_rejects_semantic_label_tampering(tmp_path) -> None:
    root = tmp_path / "behavior"
    root.mkdir()
    labels = root / "behavior_labels.csv"
    row = {
        "sample_id": "sample01", "run_id": "run01", "grid_stamp_ns": 4_000_000_000,
        "behavior_class": 2, "behavior_label": "FORWARD_AVOID", "behavior_side": 2,
        "behavior_side_label": "RIGHT", "behavior_valid": True,
        "behavior_side_valid": True, "quality": 0.9, "source_stamp_ns": 3_900_000_000,
        "source_age_ms": 100.0, "source": "mpc_expert_autoware_log",
        "authority": "AVOID_PURE", "target_vehicle": "d3", "invalid_reason": None,
    }

    def write_labels() -> str:
        with labels.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(BehaviorAnnotationV1.__dataclass_fields__))
            writer.writeheader()
            writer.writerow(row)
        return hashlib.sha256(labels.read_bytes()).hexdigest()

    manifest = {
        "format": "aic_behavior_view_v1", "ontology": "aic_behavior_v1",
        "class_names": [item.name for item in BehaviorClassV1],
        "side_names": [item.name for item in BehaviorSideV1], "dataset_manifest_sha256": "dataset",
        "labels_sha256": write_labels(), "sample_count": 1,
        "valid_behavior_count": 1, "valid_side_count": 1,
    }
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    assert load_behavior_view_v1(root, dataset_manifest_sha256="dataset")["sample01"]["source"]

    row["behavior_label"] = "FORWARD_NORMAL"
    manifest["labels_sha256"] = write_labels()
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="class/label mismatch"):
        load_behavior_view_v1(root, dataset_manifest_sha256="dataset")

    row.update({
        "behavior_class": -1,
        "behavior_label": "UNKNOWN",
        "behavior_valid": False,
    })
    manifest["labels_sha256"] = write_labels()
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="side cannot be valid"):
        load_behavior_view_v1(root, dataset_manifest_sha256="dataset")
