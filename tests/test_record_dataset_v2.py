from __future__ import annotations

from types import SimpleNamespace

import pytest

import tools.record_dataset_v2 as recorder_module
from tools.record_dataset_v2 import (
    DATASET_V2_TOPICS,
    build_record_command,
    discover_topic_types,
    parse_topic_list_with_types,
    validate_topic_types,
)


def test_recorder_command_contains_all_versioned_topics_and_mcap_compression() -> None:
    command = build_record_command("/tmp/raw/run_001")

    assert command[:2] == ["ros2", "bag"]
    assert command[2:4] == ["record", "--storage"]
    assert "mcap" in command
    assert "--compression-mode" in command
    assert "zstd" in command
    assert "--use-sim-time" in command
    for contract in DATASET_V2_TOPICS:
        assert contract.name in command


def test_recorder_topic_type_validation_fails_on_missing_or_wrong_type() -> None:
    discovered = {contract.name: contract.message_type for contract in DATASET_V2_TOPICS}
    validate_topic_types(discovered)

    missing = dict(discovered)
    missing.pop(DATASET_V2_TOPICS[0].name)
    with pytest.raises(ValueError, match="Missing required topic"):
        validate_topic_types(missing)

    wrong = dict(discovered)
    wrong[DATASET_V2_TOPICS[0].name] = "std_msgs/msg/String"
    with pytest.raises(ValueError, match="type mismatch"):
        validate_topic_types(wrong)


def test_parse_topic_list_with_types_uses_one_coherent_snapshot() -> None:
    output = "\n".join(
        [
            f"{contract.name} [{contract.message_type}]"
            for contract in DATASET_V2_TOPICS
        ]
        + ["warning text that is not a topic"]
    )

    assert parse_topic_list_with_types(output) == {
        contract.name: contract.message_type for contract in DATASET_V2_TOPICS
    }


def test_discover_topic_types_retries_an_incomplete_first_dds_snapshot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    complete_lines = [
        f"{contract.name} [{contract.message_type}]"
        for contract in DATASET_V2_TOPICS
    ]
    incomplete_lines = complete_lines[1:]
    responses = iter(
        (
            SimpleNamespace(returncode=0, stdout="\n".join(incomplete_lines)),
            SimpleNamespace(returncode=0, stdout="\n".join(complete_lines)),
        )
    )
    calls: list[tuple[list[str], dict[str, str] | None]] = []
    sleeps: list[float] = []

    def fake_run(command: list[str], **kwargs: object) -> SimpleNamespace:
        calls.append((command, kwargs.get("env")))
        return next(responses)

    monkeypatch.setattr(recorder_module.subprocess, "run", fake_run)
    monkeypatch.setattr(recorder_module.time, "sleep", sleeps.append)
    environment = {"ROS_DOMAIN_ID": "101"}

    discovered = discover_topic_types(
        env=environment,
        attempts=2,
        retry_delay_sec=0.25,
        spin_time_sec=2.0,
    )

    assert discovered == {
        contract.name: contract.message_type for contract in DATASET_V2_TOPICS
    }
    assert calls == [
        (
            [
                "ros2",
                "topic",
                "list",
                "--no-daemon",
                "--spin-time",
                "2.0",
                "-t",
            ],
            environment,
        ),
        (
            [
                "ros2",
                "topic",
                "list",
                "--no-daemon",
                "--spin-time",
                "2.0",
                "-t",
            ],
            environment,
        ),
    ]
    assert sleeps == [0.25]


def test_discover_topic_types_keeps_missing_stream_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = "\n".join(
        f"{contract.name} [{contract.message_type}]"
        for contract in DATASET_V2_TOPICS[1:]
    )
    monkeypatch.setattr(
        recorder_module.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout=output),
    )
    monkeypatch.setattr(recorder_module.time, "sleep", lambda _: None)

    discovered = discover_topic_types(
        attempts=2,
        retry_delay_sec=0.0,
        spin_time_sec=2.0,
    )

    with pytest.raises(ValueError, match="Missing required topic"):
        validate_topic_types(discovered)
