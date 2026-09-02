from __future__ import annotations

import pytest

from aic_transfuser_lite.runtime.behavior_decode_v1 import (
    decode_behavior_logits_v1,
)


def test_non_directional_behavior_forces_none_side() -> None:
    prediction = decode_behavior_logits_v1(
        [8.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 8.0],
        confidence_threshold=0.5,
        temperature=1.0,
    )
    assert prediction.behavior_label == "FORWARD_NORMAL"
    assert prediction.behavior_side_label == "NONE"


def test_directional_behavior_requires_confident_left_or_right() -> None:
    right = decode_behavior_logits_v1(
        [0.0, 0.0, 8.0, 0.0, 0.0],
        [0.0, 0.0, 8.0],
        confidence_threshold=0.5,
        temperature=1.0,
    )
    assert right.behavior_label == "FORWARD_AVOID"
    assert right.behavior_side_label == "RIGHT"

    unknown_side = decode_behavior_logits_v1(
        [0.0, 0.0, 8.0, 0.0, 0.0],
        [8.0, 0.0, 0.0],
        confidence_threshold=0.5,
        temperature=1.0,
    )
    assert unknown_side.behavior_label == "FORWARD_AVOID"
    assert unknown_side.behavior_side == -1
    assert unknown_side.behavior_side_label == "UNKNOWN"


def test_low_confidence_and_invalid_parameters_fail_closed() -> None:
    prediction = decode_behavior_logits_v1(
        [0.0] * 5,
        [0.0] * 3,
        confidence_threshold=0.5,
        temperature=1.0,
    )
    assert prediction.behavior_class == -1
    assert prediction.behavior_side == -1

    with pytest.raises(ValueError, match="temperature"):
        decode_behavior_logits_v1(
            [0.0] * 5,
            [0.0] * 3,
            confidence_threshold=0.5,
            temperature=float("nan"),
        )
    with pytest.raises(ValueError, match="finite"):
        decode_behavior_logits_v1(
            [0.0, 0.0, float("inf"), 0.0, 0.0],
            [0.0] * 3,
            confidence_threshold=0.5,
            temperature=1.0,
        )
