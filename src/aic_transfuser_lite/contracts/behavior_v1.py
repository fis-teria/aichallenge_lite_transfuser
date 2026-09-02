from __future__ import annotations

from enum import IntEnum


BEHAVIOR_ONTOLOGY_V1 = "aic_behavior_v1"


class BehaviorClassV1(IntEnum):
    FORWARD_NORMAL = 0
    FORWARD_FOLLOW = 1
    FORWARD_AVOID = 2
    FORWARD_RETURN = 3
    RECOVERY = 4


class BehaviorSideV1(IntEnum):
    NONE = 0
    LEFT = 1
    RIGHT = 2


BEHAVIOR_CLASS_NAMES_V1 = tuple(item.name for item in BehaviorClassV1)
BEHAVIOR_SIDE_NAMES_V1 = tuple(item.name for item in BehaviorSideV1)


def behavior_name_v1(value: int) -> str:
    try:
        return BehaviorClassV1(value).name
    except ValueError as error:
        raise ValueError(f"unknown {BEHAVIOR_ONTOLOGY_V1} class id: {value}") from error


def behavior_side_name_v1(value: int) -> str:
    try:
        return BehaviorSideV1(value).name
    except ValueError as error:
        raise ValueError(f"unknown {BEHAVIOR_ONTOLOGY_V1} side id: {value}") from error
