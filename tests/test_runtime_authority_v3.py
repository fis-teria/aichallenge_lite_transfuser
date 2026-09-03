from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.runtime.authority import (
    AuthorityRole,
    PublisherOwnership,
    ShadowAuthorityContract,
    model_control_debug_publication,
    shadow_authority_contract,
)
from aic_transfuser_lite.runtime.output_profiles import output_profile


def test_shadow_contract_separates_debug_nominal_and_final_owners() -> None:
    contract = shadow_authority_contract()
    assert contract.publishers == (
        PublisherOwnership(
            "inference_node_v3", "shadow_model_control", AuthorityRole.DEBUG
        ),
        PublisherOwnership(
            "external_controller", "nominal_control_cmd", AuthorityRole.NOMINAL
        ),
        PublisherOwnership(
            "safety_supervisor", "control/command/control_cmd", AuthorityRole.FINAL
        ),
    )


@pytest.mark.parametrize(
    "claims",
    [
        (
            PublisherOwnership("model", "same", AuthorityRole.DEBUG),
            PublisherOwnership("external_controller", "same", AuthorityRole.NOMINAL),
            PublisherOwnership("safety_supervisor", "final", AuthorityRole.FINAL),
        ),
        (
            PublisherOwnership(
                "inference_node_v3", "nominal_control_cmd", AuthorityRole.NOMINAL
            ),
            PublisherOwnership(
                "safety_supervisor", "control/command/control_cmd", AuthorityRole.FINAL
            ),
        ),
    ],
)
def test_shadow_contract_rejects_ambiguous_or_model_nominal_ownership(
    claims: tuple[PublisherOwnership, ...],
) -> None:
    with pytest.raises(ValueError):
        ShadowAuthorityContract(claims).validate()


def test_model_control_publication_selects_candidate_zero_as_debug_only() -> None:
    control = np.array([[[0.2, 3.0, -0.5], [-0.4, 5.0, 1.0]]])
    result = model_control_debug_publication(control)
    assert result.steering_rad == 0.2
    assert result.speed_mps == 3.0
    assert result.acceleration_mps2 == -0.5
    assert not result.authoritative


@pytest.mark.parametrize(
    ("control", "message"),
    [
        (np.zeros((1, 3)), r"\[1,K,3\]"),
        (np.zeros((1, 1, 2)), r"\[1,K,3\]"),
        (np.array([[[np.nan, 0.0, 0.0]]]), "finite"),
        (np.array([[[0.0, -0.1, 0.0]]]), "non-negative"),
    ],
)
def test_model_control_debug_publication_rejects_invalid_values(
    control: np.ndarray, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        model_control_debug_publication(control)


def test_shadow_output_profile_has_debug_model_control_without_nominal_authority() -> None:
    profile = output_profile("shadow_control")
    assert "current_control" in profile.requested_outputs
    assert "shadow_model_control" in profile.publisher_topics
    assert "shadow_model_control_sequence" in profile.publisher_topics
    assert "nominal_control_cmd" not in profile.publisher_topics
    assert not profile.nominal_control_authority


def test_ros_shadow_launch_and_node_remain_debug_only() -> None:
    root = Path(__file__).parents[1]
    package = root / "ros2_ws" / "src" / "aic_e2e_runtime"
    source = (package / "aic_e2e_runtime" / "inference_node_v3.py").read_text()
    launch = (package / "launch" / "transfuser_lite_v3_shadow.launch.py").read_text()
    params = (package / "config" / "runtime.v3.shadow.param.yaml").read_text()
    authority = (root / "src" / "aic_transfuser_lite" / "runtime" / "authority.py").read_text()

    assert 'RuntimeProfile.SHADOW_CONTROL' in source
    assert '"shadow_model_control"' in source
    assert 'requested.add("current_control")' in source
    assert "model_control_debug_publication(" in source
    assert 'if self.runtime_profile is RuntimeProfile.FULL_CONTROL' in source
    assert 'self.full_control_pub = None' in source
    assert "runtime.v3.shadow.param.yaml" in launch
    assert 'DeclareLaunchArgument("launch_rviz", default_value="false")' in launch
    assert 'condition=IfCondition(LaunchConfiguration("launch_rviz"))' in launch
    assert "safety_supervisor_node" not in launch
    assert "nominal_control_cmd" not in launch
    assert "runtime_profile: shadow_control" in params
    assert 'owner="external_controller"' in authority
    assert 'owner="safety_supervisor"' in authority
