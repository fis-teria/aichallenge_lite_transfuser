from __future__ import annotations

from pathlib import Path

import pytest

from aic_transfuser_lite.data.mcap_reader_v3 import select_reader_topics_v3
from aic_transfuser_lite.data.topic_profile_v3 import load_topic_profile_v3


ROOT = Path(__file__).parents[1]


def _observed_topics(*, include_nominal: bool = True) -> dict[str, str]:
    profile = load_topic_profile_v3(ROOT / "configs/data/topic_profile_v3.yaml")
    observed = {
        spec.name: spec.message_type
        for spec in profile.roles.values()
        if spec.required_for_conversion or spec.role in {"actual_steering", "final_command"}
    }
    if include_nominal:
        nominal = profile.roles["nominal_command"]
        observed[nominal.name] = nominal.message_type
    return observed


def test_v3_reader_accepts_final_fallback_without_nominal_command() -> None:
    profile = load_topic_profile_v3(ROOT / "configs/data/topic_profile_v3.yaml")
    selected = select_reader_topics_v3(profile, _observed_topics(include_nominal=False))
    assert selected[profile.roles["final_command"].name] == "final_command"
    assert profile.roles["nominal_command"].name not in selected


def test_v3_reader_rejects_missing_required_sensor() -> None:
    profile = load_topic_profile_v3(ROOT / "configs/data/topic_profile_v3.yaml")
    observed = _observed_topics()
    del observed[profile.roles["lidar"].name]
    with pytest.raises(ValueError, match="missing required V3 conversion topics.*lidar/scan"):
        select_reader_topics_v3(profile, observed)


def test_v3_reader_rejects_optional_topic_type_mismatch() -> None:
    profile = load_topic_profile_v3(ROOT / "configs/data/topic_profile_v3.yaml")
    observed = _observed_topics(include_nominal=False)
    observed[profile.roles["final_command"].name] = "example_msgs/msg/WrongControl"
    with pytest.raises(ValueError, match="topic type mismatch.*control_cmd"):
        select_reader_topics_v3(profile, observed)
