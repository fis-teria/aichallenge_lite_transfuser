from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml

from aic_transfuser_lite.data.calibration.excitation import (
    MAX_COLLECTION_ACCELERATION_MPS2,
    MAX_COLLECTION_SPEED_MPS,
    MAX_COLLECTION_STEERING_RAD,
    excitation_plan_sha256,
    load_excitation_plan,
    runtime_guard_reasons,
)
from aic_transfuser_lite.data.topic_profile_v3 import load_topic_profile_v3


ROOT = Path(__file__).parents[1]
PLAN_PATHS = (
    ROOT / "configs/calibration/excitation_steering_low_speed_v1.yaml",
    ROOT / "configs/calibration/excitation_drive_low_speed_v1.yaml",
    ROOT / "configs/calibration/excitation_brake_low_speed_v1.yaml",
)


def _collector_module():
    path = ROOT / "tools/collect_calibration_v3.py"
    spec = importlib.util.spec_from_file_location("collect_calibration_v3", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_checked_in_excitation_plans_are_bounded_and_end_stopped() -> None:
    for path in PLAN_PATHS:
        plan = load_excitation_plan(path)
        assert len(excitation_plan_sha256(plan)) == 64
        assert plan.total_duration_sec <= 180.0
        assert plan.segments[0].mode == "settle"
        assert plan.segments[-1].mode == "stop"
        assert plan.segments[-1].duration_sec >= plan.stop_hold_sec
        for segment in plan.segments:
            assert abs(segment.command.steering_rad) <= MAX_COLLECTION_STEERING_RAD
            assert 0.0 <= segment.command.speed_mps <= MAX_COLLECTION_SPEED_MPS
            assert segment.command.acceleration_mps2 <= MAX_COLLECTION_ACCELERATION_MPS2


def test_plan_sampling_has_explicit_boundaries() -> None:
    plan = load_excitation_plan(PLAN_PATHS[0])
    first, elapsed = plan.command_at(0.0)
    assert first.segment_id == "settle"
    assert elapsed == 0.0
    second, elapsed = plan.command_at(first.duration_sec)
    assert second.segment_id == "reach_speed"
    assert elapsed == 0.0
    last, elapsed = plan.command_at(plan.total_duration_sec + 1.0)
    assert last.segment_id == "final_stop"
    assert elapsed == last.duration_sec
    with pytest.raises(ValueError, match="non-negative"):
        plan.command_at(-0.01)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda raw: raw["segments"][1].update(speed_mps=3.1), "speed"),
        (lambda raw: raw["segments"][-1].update(mode="drive"), "last segment"),
        (lambda raw: raw.update(publish_hz=5.0), "publish_hz"),
        (
            lambda raw: raw["segments"][2].update(steering_rad=float("nan")),
            "finite",
        ),
    ],
)
def test_plan_negative_cases_fail_closed(tmp_path, mutation, message: str) -> None:
    raw = yaml.safe_load(PLAN_PATHS[0].read_text(encoding="utf-8"))
    mutation(raw)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_excitation_plan(path)


def test_runtime_guard_rejects_authority_staleness_speed_and_safety() -> None:
    plan = load_excitation_plan(PLAN_PATHS[0])
    assert runtime_guard_reasons(
        speed_mps=0.0,
        telemetry_age_sec=0.01,
        safety_reason="normal",
        nominal_publisher_count=1,
        final_publisher_count=1,
        nominal_subscriber_count=1,
        final_subscriber_count=1,
        plan=plan,
    ) == ()
    reasons = runtime_guard_reasons(
        speed_mps=plan.max_observed_speed_mps + 0.1,
        telemetry_age_sec=plan.telemetry_timeout_sec + 0.1,
        safety_reason="front_obstacle_inside_stopping_distance",
        nominal_publisher_count=2,
        final_publisher_count=0,
        nominal_subscriber_count=0,
        final_subscriber_count=0,
        plan=plan,
    )
    assert "nominal_publisher_count=2" in reasons
    assert "final_publisher_count=0" in reasons
    assert "observed_speed_exceeds_plan_limit" in reasons
    assert "velocity_telemetry_stale" in reasons
    assert any(reason.startswith("safety_not_ready") for reason in reasons)


def test_collector_dry_run_builds_exact_recording_without_writes(
    tmp_path, capsys
) -> None:
    collector = _collector_module()
    output = tmp_path / "capture"
    result = collector.main(
        [
            "--plan",
            str(PLAN_PATHS[1]),
            "--topic-profile",
            str(ROOT / "configs/data/topic_profile_v3.yaml"),
            "--output-root",
            str(output),
            "--run-id",
            "drive_run_01",
            "--scenario-id",
            "awsim_calibration_pad",
        ]
    )
    preview = json.loads(capsys.readouterr().out)
    assert result == 0
    assert preview["execute"] is False
    assert preview["target_mode"] == "drive"
    assert "/vehicle/status/steering_status" in preview["record_command"]
    assert "/nominal_control_cmd" in preview["record_command"]
    assert "/control/command/control_cmd" in preview["record_command"]
    assert "calibration_capture_v3.launch.py" in preview["launch_command"]
    assert not output.exists()


def test_collector_topic_and_publisher_parsers_reject_bad_graph() -> None:
    collector = _collector_module()
    profile = load_topic_profile_v3(ROOT / "configs/data/topic_profile_v3.yaml")
    discovered = {
        spec.name: spec.message_type
        for role, spec in profile.roles.items()
        if role in collector.PREFLIGHT_INPUT_ROLES
    }
    collector.validate_preflight_topics(profile, discovered)
    discovered["/vehicle/status/velocity_status"] = "std_msgs/msg/String"
    with pytest.raises(ValueError, match="type mismatch"):
        collector.validate_preflight_topics(profile, discovered)
    assert collector.parse_publisher_count("Publisher count: 0\nSubscription count: 1\n") == 0
    with pytest.raises(ValueError, match="Publisher count"):
        collector.parse_publisher_count("Subscription count: 1\n")


def test_collector_requires_valid_explicit_archive_revision() -> None:
    collector = _collector_module()
    assert collector._source_state(
        ROOT, explicit_revision="a" * 40
    ) == ("a" * 40, False)
    with pytest.raises(ValueError, match="lowercase 40-hex"):
        collector._source_state(ROOT, explicit_revision="NOT_A_SHA")
