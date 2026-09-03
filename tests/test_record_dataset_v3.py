from __future__ import annotations

from pathlib import Path

import pytest

from aic_transfuser_lite.data.topic_profile_v3 import load_topic_profile_v3
from tools.record_dataset_v3 import (
    build_record_command,
    parse_publisher_count,
    select_recording_topics,
)


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / "configs/data/topic_profile_recovery_collection_v3.yaml"


def _discovered() -> dict[str, str]:
    profile = load_topic_profile_v3(PROFILE)
    return {spec.name: spec.message_type for spec in profile.roles.values()}


def test_recovery_profile_records_required_and_available_optional_topics() -> None:
    profile = load_topic_profile_v3(PROFILE)
    discovered = _discovered()
    del discovered[profile.roles["collision"].name]

    topics = select_recording_topics(profile, discovered)

    assert profile.roles["reference_route"].name in topics
    assert profile.roles["actual_steering"].name in topics
    assert profile.roles["collision"].name not in topics
    command = build_record_command("/tmp/recovery_001", topics)
    assert command[-len(topics) :] == list(topics)
    assert "mcap" in command and "zstd" in command


def test_recovery_profile_fails_closed_without_reference_topic() -> None:
    profile = load_topic_profile_v3(PROFILE)
    discovered = _discovered()
    del discovered[profile.roles["reference_route"].name]

    with pytest.raises(ValueError, match="reference_route"):
        select_recording_topics(profile, discovered)


def test_official_direct_final_teacher_does_not_require_nominal_topic() -> None:
    profile = load_topic_profile_v3(PROFILE)
    discovered = _discovered()
    del discovered[profile.roles["nominal_command"].name]

    topics = select_recording_topics(profile, discovered)

    assert profile.roles["final_command"].name in topics
    assert profile.roles["nominal_command"].name not in topics


def test_teacher_publisher_count_parser_is_exact() -> None:
    assert parse_publisher_count("Type: example\nPublisher count: 1\n") == 1
    with pytest.raises(ValueError, match="lacks Publisher count"):
        parse_publisher_count("Publisher Count: unknown\n")
