from __future__ import annotations

import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import (
    ModelBatchV3,
    TrainingTargetsV3,
)
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3


def _batch() -> ModelBatchV3:
    return ModelBatchV3(
        image=torch.zeros(2, 4, 3, 16, 32),
        image_mask=torch.ones(2, 4, dtype=torch.bool),
        lidar=torch.zeros(2, 4, 2, 20),
        lidar_mask=torch.ones(2, 4, dtype=torch.bool),
        ego=torch.zeros(2, 10, 5),
        ego_feature_mask=torch.ones(2, 10, 5, dtype=torch.bool),
        command_history=torch.zeros(2, 10, 3),
        command_mask=torch.ones(2, 10, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(2, 4, 3),
        targets=TrainingTargetsV3(
            trajectory_xy_m=torch.zeros(2, 15, 2),
            trajectory_mask=torch.ones(2, 15, dtype=torch.bool),
            speed_mps=torch.ones(2, 15),
            speed_mask=torch.ones(2, 15, dtype=torch.bool),
        ),
    )


def test_model_batch_v3_validates_all_temporal_shapes_and_masks() -> None:
    batch = _batch()
    batch.validate()
    assert batch.batch_size == 2


def test_batch_rejects_shape_mask_nonfinite_and_current_invalid() -> None:
    batch = _batch()
    with pytest.raises(ValueError, match="image_mask"):
        ModelBatchV3(**{**batch.__dict__, "image_mask": torch.ones(2, 3, dtype=torch.bool)}).validate()
    bad_image = batch.image.clone()
    bad_image[0, 0, 0, 0, 0] = float("nan")
    with pytest.raises(ValueError, match="image must be finite"):
        ModelBatchV3(**{**batch.__dict__, "image": bad_image}).validate()
    mask = batch.lidar_mask.clone()
    mask[0, -1] = False
    with pytest.raises(ValueError, match="current Camera and LiDAR"):
        ModelBatchV3(**{**batch.__dict__, "lidar_mask": mask}).validate()


def test_trajectory_is_mandatory_and_unknown_requested_output_rejected() -> None:
    batch = _batch()
    with pytest.raises(ValueError, match="mandatory"):
        ModelBatchV3(**{**batch.__dict__, "requested_outputs": frozenset({"speed_profile"})}).validate()
    with pytest.raises(ValueError, match="unknown requested"):
        ModelBatchV3(**{**batch.__dict__, "requested_outputs": frozenset({"trajectory", "magic"})}).validate()


def test_model_output_validates_required_and_skipped_heads() -> None:
    output = ModelOutputV3(
        trajectory_xy=torch.zeros(2, 1, 15, 2),
        trajectory_speed_mps=torch.ones(2, 1, 15),
        candidate_logits=torch.zeros(2, 1),
    )
    output.validate(
        batch_size=2,
        candidates=1,
        trajectory_steps=15,
        requested_outputs=frozenset({"trajectory", "speed_profile"}),
    )
    with pytest.raises(ValueError, match="requested output 'current_control' is absent"):
        output.validate(
            batch_size=2,
            candidates=1,
            trajectory_steps=15,
            requested_outputs=frozenset({"trajectory", "speed_profile", "current_control"}),
        )


def test_model_output_rejects_negative_speed_and_unrequested_head() -> None:
    negative = ModelOutputV3(
        trajectory_xy=torch.zeros(1, 1, 15, 2),
        trajectory_speed_mps=-torch.ones(1, 1, 15),
        candidate_logits=torch.zeros(1, 1),
    )
    with pytest.raises(ValueError, match="non-negative"):
        negative.validate(
            batch_size=1,
            candidates=1,
            trajectory_steps=15,
            requested_outputs=frozenset({"trajectory", "speed_profile"}),
        )


def test_behavior_targets_and_outputs_have_fixed_ontology_shapes() -> None:
    batch = _batch()
    targets = TrainingTargetsV3(
        **{
            **batch.targets.__dict__,
            "behavior_class": torch.tensor([0, 4]),
            "behavior_mask": torch.tensor([True, True]),
            "behavior_side": torch.tensor([0, 2]),
            "behavior_side_mask": torch.tensor([True, True]),
        }
    )
    targets.validate(batch_size=2)
    output = ModelOutputV3(
        trajectory_xy=torch.zeros(2, 1, 15, 2),
        trajectory_speed_mps=torch.ones(2, 1, 15),
        candidate_logits=torch.zeros(2, 1),
        behavior_logits=torch.zeros(2, 5),
        behavior_side_logits=torch.zeros(2, 3),
    )
    output.validate(
        batch_size=2, candidates=1, trajectory_steps=15,
        requested_outputs=frozenset({"trajectory", "speed_profile", "behavior", "behavior_side"}),
    )

    invalid_targets = TrainingTargetsV3(
        **{
            **targets.__dict__,
            "behavior_class": torch.tensor([0, -1]),
            "behavior_mask": torch.tensor([True, True]),
        }
    )
    with pytest.raises(ValueError, match="outside"):
        invalid_targets.validate(batch_size=2)

    invalid_output = ModelOutputV3(
        trajectory_xy=torch.zeros(2, 1, 15, 2),
        trajectory_speed_mps=torch.ones(2, 1, 15),
        candidate_logits=torch.zeros(2, 1),
        behavior_logits=torch.zeros(2, 4),
        behavior_side_logits=torch.zeros(2, 3),
    )
    with pytest.raises(ValueError, match=r"\[B,5\]"):
        invalid_output.validate(
            batch_size=2,
            candidates=1,
            trajectory_steps=15,
            requested_outputs=frozenset(
                {"trajectory", "speed_profile", "behavior", "behavior_side"}
            ),
        )
