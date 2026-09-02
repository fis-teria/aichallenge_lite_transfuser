from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_SIDE_NAMES_V1,
    BehaviorClassV1,
    BehaviorSideV1,
)


@dataclass(frozen=True)
class BehaviorPredictionV1:
    behavior_class: int
    behavior_label: str
    behavior_confidence: float
    behavior_side: int
    behavior_side_label: str
    behavior_side_confidence: float


def decode_behavior_logits_v1(
    behavior_logits: Sequence[float],
    side_logits: Sequence[float],
    *,
    confidence_threshold: float,
    temperature: float,
) -> BehaviorPredictionV1:
    """Decode fixed-ontology logits without granting them control authority."""

    if not math.isfinite(confidence_threshold) or not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("behavior confidence threshold must be finite within [0,1]")
    if not math.isfinite(temperature) or temperature <= 0.0:
        raise ValueError("behavior temperature must be finite and positive")
    behavior_probabilities = _softmax(
        behavior_logits,
        expected_size=len(BEHAVIOR_CLASS_NAMES_V1),
        temperature=temperature,
        name="behavior",
    )
    side_probabilities = _softmax(
        side_logits,
        expected_size=len(BEHAVIOR_SIDE_NAMES_V1),
        temperature=temperature,
        name="behavior side",
    )
    behavior_class = max(range(len(behavior_probabilities)), key=behavior_probabilities.__getitem__)
    behavior_confidence = behavior_probabilities[behavior_class]
    raw_side = max(range(len(side_probabilities)), key=side_probabilities.__getitem__)
    side_confidence = side_probabilities[raw_side]
    if behavior_confidence < confidence_threshold:
        return BehaviorPredictionV1(
            -1, "UNKNOWN", behavior_confidence, -1, "UNKNOWN", side_confidence
        )

    directional = behavior_class in {
        int(BehaviorClassV1.FORWARD_AVOID),
        int(BehaviorClassV1.FORWARD_RETURN),
    }
    if not directional:
        side = int(BehaviorSideV1.NONE)
    elif (
        raw_side == int(BehaviorSideV1.NONE)
        or side_confidence < confidence_threshold
    ):
        side = -1
    else:
        side = raw_side
    return BehaviorPredictionV1(
        behavior_class=behavior_class,
        behavior_label=BEHAVIOR_CLASS_NAMES_V1[behavior_class],
        behavior_confidence=behavior_confidence,
        behavior_side=side,
        behavior_side_label=(
            "UNKNOWN" if side < 0 else BEHAVIOR_SIDE_NAMES_V1[side]
        ),
        behavior_side_confidence=side_confidence,
    )


def _softmax(
    logits: Sequence[float], *, expected_size: int, temperature: float, name: str
) -> tuple[float, ...]:
    values = tuple(float(value) for value in logits)
    if len(values) != expected_size:
        raise ValueError(f"{name} logits must contain {expected_size} values")
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name} logits must be finite")
    scaled = tuple(value / temperature for value in values)
    maximum = max(scaled)
    exponentials = tuple(math.exp(value - maximum) for value in scaled)
    total = sum(exponentials)
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError(f"{name} softmax normalization failed")
    return tuple(value / total for value in exponentials)
