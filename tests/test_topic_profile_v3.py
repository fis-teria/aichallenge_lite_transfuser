from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from aic_transfuser_lite.contracts.capabilities import CapabilityRule
from aic_transfuser_lite.data.topic_profile_v3 import (
    assess_topic_profile_v3,
    load_topic_profile_v3,
)


PROFILE = Path(__file__).parents[1] / "configs" / "data" / "topic_profile_v3.yaml"


def _required_observed() -> dict[str, str]:
    profile = load_topic_profile_v3(PROFILE)
    return {
        spec.name: spec.message_type
        for spec in profile.roles.values()
        if spec.required_for_conversion
    }


def test_requirement_axes_are_separate_and_optional_missing_is_accepted() -> None:
    profile = load_topic_profile_v3(PROFILE)
    assessment = assess_topic_profile_v3(profile, _required_observed())
    assert assessment.conversion_accepted
    assert "actual_steering" not in assessment.available_roles
    assert "actual_steering" not in assessment.capabilities.available
    assert "lateral_calibration" not in assessment.capabilities.available
    assert "trajectory_label" in assessment.capabilities.available
    assert assessment.missing_for_recording


def test_any_role_capability_accepts_nominal_or_final_command() -> None:
    profile = load_topic_profile_v3(PROFILE)
    observed = _required_observed()
    nominal = profile.roles["nominal_command"]
    observed[nominal.name] = nominal.message_type
    assessment = assess_topic_profile_v3(profile, observed)
    assert "full_control_label" in assessment.capabilities.available


def test_missing_required_conversion_role_rejects_conversion_only() -> None:
    profile = load_topic_profile_v3(PROFILE)
    observed = _required_observed()
    del observed[profile.roles["lidar"].name]
    assessment = assess_topic_profile_v3(profile, observed)
    assert not assessment.conversion_accepted
    assert assessment.missing_for_conversion == ("lidar",)


def test_exact_type_mismatch_fails_explicitly() -> None:
    profile = load_topic_profile_v3(PROFILE)
    observed = _required_observed()
    observed[profile.roles["camera"].name] = "sensor_msgs/msg/CompressedImage"
    with pytest.raises(ValueError, match="topic type mismatch"):
        assess_topic_profile_v3(profile, observed)


def test_unknown_capability_is_rejected(tmp_path: Path) -> None:
    raw = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    raw["capabilities"]["invented_capability"] = {"all_roles": ["camera"]}
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown capability"):
        load_topic_profile_v3(path)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda raw: raw["roles"]["camera"].update(
            {"required_for_conversion": "yes"}
        ),
        lambda raw: raw["roles"]["camera"]["synchronization"].update(
            {"policy": "guess"}
        ),
        lambda raw: raw.update({"typo": True}),
    ],
)
def test_profile_rejects_invalid_or_unknown_contract_fields(
    tmp_path: Path, mutation
) -> None:
    raw = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    mutation(raw)
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError):
        load_topic_profile_v3(path)


def test_capability_rule_rejects_unknown_roles() -> None:
    with pytest.raises(ValueError, match="unknown roles"):
        CapabilityRule("camera_input", all_roles=("missing",)).validate(
            known_roles=frozenset({"camera"})
        )
