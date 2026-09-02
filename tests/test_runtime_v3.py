import json
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3
from aic_transfuser_lite.runtime.model_loader_v3 import load_runtime_model_v3, sha256_file_v3
from aic_transfuser_lite.runtime.output_profiles import (
    output_profile,
    runtime_clock_has_reached_observation,
    trajectory_path_publication,
    trajectory_speed_publication,
    validate_observation_timing,
)


def _artifact(tmp_path: Path, *, behavior: bool = False) -> tuple[Path, Path, str, str, str]:
    contract = "c" * 64
    kwargs = dict(image_height=32, image_width=32, lidar_points=16, ego_dim=4,
                  hidden_dim=16, camera_tokens_hw=(1, 1), lidar_tokens=2,
                  fusion_depth=1, fusion_heads=4)
    if behavior:
        kwargs["behavior_head_enabled"] = True
    model = FullControlLiteV3(**kwargs)
    checkpoint = tmp_path / "model.pt"
    torch.save({
        "model": model.state_dict(), "identity": {"contract_hash": contract},
        "behavior_ontology": "aic_behavior_v1" if behavior else None,
    }, checkpoint)
    checkpoint_hash = sha256_file_v3(checkpoint)
    manifest = tmp_path / "artifact.json"
    manifest.write_text(json.dumps({
        "format": "aic_runtime_artifact_v3", "checkpoint_sha256": checkpoint_hash,
        "contract_hash": contract, "capabilities": (
            ["trajectory", "speed_profile", "behavior", "behavior_side"]
            if behavior else ["trajectory", "speed_profile"]
        ),
        "model_kwargs": kwargs,
    }, sort_keys=True), encoding="utf-8")
    return checkpoint, manifest, checkpoint_hash, sha256_file_v3(manifest), contract


def test_strict_v3_artifact_load(tmp_path: Path) -> None:
    args = _artifact(tmp_path)
    loaded = load_runtime_model_v3(args[0], args[1], device=torch.device("cpu"),
                                   expected_checkpoint_sha256=args[2],
                                   expected_manifest_sha256=args[3], expected_contract_hash=args[4])
    assert loaded.capabilities == frozenset({"trajectory", "speed_profile"})


def test_behavior_artifact_requires_ontology_and_enabled_head(tmp_path: Path) -> None:
    args = _artifact(tmp_path, behavior=True)
    loaded = load_runtime_model_v3(
        args[0], args[1], device=torch.device("cpu"), expected_checkpoint_sha256=args[2],
        expected_manifest_sha256=args[3], expected_contract_hash=args[4],
    )
    assert {"behavior", "behavior_side"}.issubset(loaded.capabilities)
    assert loaded.model.behavior_head is not None


def test_behavior_artifact_rejects_partial_capability_pair(tmp_path: Path) -> None:
    args = _artifact(tmp_path, behavior=True)
    manifest = json.loads(args[1].read_text(encoding="utf-8"))
    manifest["capabilities"].remove("behavior_side")
    args[1].write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    with pytest.raises(ValueError, match="declared together"):
        load_runtime_model_v3(
            args[0],
            args[1],
            device=torch.device("cpu"),
            expected_checkpoint_sha256=args[2],
            expected_manifest_sha256=sha256_file_v3(args[1]),
            expected_contract_hash=args[4],
        )


def test_current_control_capability_requires_enabled_head(tmp_path: Path) -> None:
    args = _artifact(tmp_path)
    manifest = json.loads(args[1].read_text(encoding="utf-8"))
    manifest["capabilities"].append("current_control")
    args[1].write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    with pytest.raises(ValueError, match="enabled control head"):
        load_runtime_model_v3(
            args[0],
            args[1],
            device=torch.device("cpu"),
            expected_checkpoint_sha256=args[2],
            expected_manifest_sha256=sha256_file_v3(args[1]),
            expected_contract_hash=args[4],
        )


@pytest.mark.parametrize("field", ["checkpoint", "manifest", "contract"])
def test_v3_artifact_hash_mismatch_fails(tmp_path: Path, field: str) -> None:
    args = _artifact(tmp_path)
    hashes = [args[2], args[3], args[4]]
    hashes[{"checkpoint": 0, "manifest": 1, "contract": 2}[field]] = "0" * 64
    with pytest.raises(ValueError, match="mismatch"):
        load_runtime_model_v3(args[0], args[1], device=torch.device("cpu"),
                              expected_checkpoint_sha256=hashes[0],
                              expected_manifest_sha256=hashes[1], expected_contract_hash=hashes[2])


def test_trajectory_only_has_no_nominal_control_publisher() -> None:
    profile = output_profile("trajectory_only")
    assert "predicted_trajectory" in profile.publisher_topics
    assert "predicted_speed_profile" in profile.publisher_topics
    assert "predicted_trajectory_path" in profile.publisher_topics
    assert "nominal_control_cmd" not in profile.publisher_topics
    assert not profile.nominal_control_authority


def test_external_controller_profile_is_shadow_only() -> None:
    profile = output_profile("external_controller")
    assert "shadow_external_control" in profile.publisher_topics
    assert "nominal_control_cmd" not in profile.publisher_topics
    assert not profile.nominal_control_authority


def test_trajectory_speed_publication_selects_matching_candidate_zero() -> None:
    trajectory = torch.arange(60, dtype=torch.float32).reshape(1, 2, 15, 2)
    speeds = torch.arange(30, dtype=torch.float32).reshape(1, 2, 15) / 10.0
    publication = trajectory_speed_publication(trajectory.numpy(), speeds.numpy())

    assert publication.point_count == 15
    assert publication.trajectory_xy_m == tuple(float(value) for value in range(30))
    assert publication.speed_profile_mps == pytest.approx(
        tuple(index / 10.0 for index in range(15))
    )


def test_trajectory_path_publication_preserves_xy_metres_and_frame() -> None:
    publication = trajectory_path_publication(
        (0.25, -0.5, 1.5, 2.0), frame_id="base_link"
    )

    assert publication.frame_id == "base_link"
    assert publication.points_xy_m == ((0.25, -0.5), (1.5, 2.0))


@pytest.mark.parametrize(
    ("trajectory", "frame_id", "message"),
    [
        ((0.0, 1.0), "", "non-empty"),
        ((0.0, 1.0), "   ", "non-empty"),
        ((), "base_link", r"\[N,2\]"),
        ((0.0, 1.0, 2.0), "base_link", r"\[N,2\]"),
        ((0.0, float("inf")), "base_link", "finite"),
    ],
)
def test_trajectory_path_publication_rejects_invalid_payload(
    trajectory: tuple[float, ...], frame_id: str, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        trajectory_path_publication(trajectory, frame_id=frame_id)


@pytest.mark.parametrize(
    ("trajectory", "speeds", "message"),
    [
        (torch.zeros(2, 1, 15, 2), torch.zeros(2, 1, 15), r"\[1,K,N,2\]"),
        (torch.zeros(1, 1, 15, 2), torch.zeros(1, 1, 14), "must match"),
        (
            torch.full((1, 1, 15, 2), float("nan")),
            torch.zeros(1, 1, 15),
            "trajectory_xy must be finite",
        ),
        (
            torch.zeros(1, 1, 15, 2),
            torch.full((1, 1, 15), float("inf")),
            "trajectory_speed_mps must be finite",
        ),
        (
            torch.zeros(1, 1, 15, 2),
            -torch.ones(1, 1, 15),
            "trajectory_speed_mps must be non-negative",
        ),
    ],
)
def test_trajectory_speed_publication_rejects_invalid_outputs(
    trajectory: torch.Tensor,
    speeds: torch.Tensor,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        trajectory_speed_publication(trajectory.numpy(), speeds.numpy())


@pytest.mark.parametrize(
    ("now", "camera", "roles", "message"),
    [
        (10.0, 9.0, {"lidar": 9.0}, "stale"),
        (10.0, 10.1, {"lidar": 10.0}, "future_timestamp"),
        (10.0, 9.95, {"lidar": 9.8}, "sensor_skew"),
    ],
)
def test_timing_failures_are_explicit(now: float, camera: float, roles: dict[str, float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        validate_observation_timing(now_sec=now, camera_stamp_sec=camera,
                                    role_stamps_sec=roles, timeout_sec=0.5, max_skew_sec=0.05)


def test_runtime_clock_gate_waits_for_selected_future_side_sample() -> None:
    stamps = {"camera": 10.0, "lidar": 10.02, "velocity": 10.01}
    assert not runtime_clock_has_reached_observation(
        now_sec=10.0, source_stamps_sec=stamps
    )
    assert runtime_clock_has_reached_observation(
        now_sec=10.019, source_stamps_sec=stamps
    )


@pytest.mark.parametrize(
    ("now_sec", "stamps", "tolerance", "message"),
    [
        (0.0, {"camera": 1.0}, 0.001, "invalid_runtime_clock"),
        (1.0, {}, 0.001, "source_stamps_empty"),
        (1.0, {"camera": float("nan")}, 0.001, "invalid_timestamp"),
        (1.0, {"camera": 1.0}, -0.001, "invalid_future_tolerance"),
    ],
)
def test_runtime_clock_gate_rejects_invalid_inputs(
    now_sec: float,
    stamps: dict[str, float],
    tolerance: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        runtime_clock_has_reached_observation(
            now_sec=now_sec,
            source_stamps_sec=stamps,
            future_tolerance_sec=tolerance,
        )
