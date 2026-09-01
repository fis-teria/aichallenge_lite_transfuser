from __future__ import annotations

from pathlib import Path

import pandas as pd
import pytest
import torch

from tools.evaluate_checkpoint_v1 import (
    AblatedDataset,
    FIRST_CORNER_SPEC_VERSION,
    first_corner_summary,
    make_derangement,
    subset_summary,
    validate_first_corner_spec,
)


class TinyDataset:
    def __len__(self) -> int:
        return 5

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        value = float(index)
        return {
            "image": torch.full((3, 2, 2), value),
            "lidar": torch.stack(
                (torch.full((4,), value), torch.ones(4)), dim=0
            ),
            "ego": torch.tensor([value]),
            "waypoints": torch.full((2, 2), value),
            "target_speed": torch.tensor([value]),
        }


def test_derangement_is_reproducible_complete_and_has_no_fixed_points() -> None:
    first = make_derangement(20, 42)
    second = make_derangement(20, 42)
    torch.testing.assert_close(first, second)
    assert sorted(first.tolist()) == list(range(20))
    assert bool(torch.all(first != torch.arange(20)))


@pytest.mark.parametrize(
    ("scenario", "changed_key"),
    (
        ("image_shuffle", "image"),
        ("lidar_shuffle", "lidar"),
        ("speed_shuffle", "ego"),
    ),
)
def test_shuffle_ablation_changes_one_input_and_keeps_target(
    scenario: str, changed_key: str
) -> None:
    base = TinyDataset()
    permutation = make_derangement(len(base), 42)
    ablated = AblatedDataset(base, scenario, permutation)
    original = base[0]
    sample = ablated[0]
    assert not torch.equal(sample[changed_key], original[changed_key])
    for key in {"image", "lidar", "ego"} - {changed_key}:
        torch.testing.assert_close(sample[key], original[key])
    torch.testing.assert_close(sample["waypoints"], original["waypoints"])
    torch.testing.assert_close(sample["target_speed"], original["target_speed"])


def test_invalid_mask_ablation_preserves_ranges_and_zeros_only_validity() -> None:
    base = TinyDataset()
    sample = AblatedDataset(base, "lidar_invalid_mask_all_zero")[3]
    torch.testing.assert_close(sample["lidar"][0], base[3]["lidar"][0])
    assert int(torch.count_nonzero(sample["lidar"][1])) == 0


def test_subset_and_fixed_corner_use_exact_sample_ids() -> None:
    details = {
        "distance_m": torch.tensor(
            [[0.0, 0.0], [1.0, 2.0], [3.0, 4.0]], dtype=torch.float32
        ),
        "controller_error_rad": torch.tensor([0.0, -0.2, 0.1]),
        "speed_error_mps": torch.tensor([0.0, 1.0, -2.0]),
        "curvature_bucket": torch.tensor([0, 2, 2]),
    }
    summary = subset_summary(details, torch.tensor([1, 2]))
    assert summary["sample_count"] == 2
    assert summary["ade_m"] == pytest.approx(2.5)
    assert summary["fde_m"] == pytest.approx(3.0)
    assert summary["controller_proxy_bias_rad"] == pytest.approx(-0.05)
    frame = pd.DataFrame({"sample_id": ["a", "b", "c"]})
    fixed = first_corner_summary(details, frame, ["b", "c"])
    assert fixed["sample_ids"] == ["b", "c"]
    assert fixed["ade_m"] == pytest.approx(summary["ade_m"])


def test_first_corner_spec_is_bound_to_test_index_and_metadata(tmp_path: Path) -> None:
    index = tmp_path / "test_index.csv"
    metadata = tmp_path / "metadata.yaml"
    index.write_text("sample_id\na\nb\n", encoding="utf-8")
    metadata.write_text("format_version: 2\n", encoding="utf-8")
    import hashlib
    import json

    spec = {
        "format_version": FIRST_CORNER_SPEC_VERSION,
        "dataset": {
            "test_index_sha256": hashlib.sha256(index.read_bytes()).hexdigest(),
            "metadata_sha256": hashlib.sha256(metadata.read_bytes()).hexdigest(),
        },
        "sample_ids": ["a", "b"],
    }
    path = tmp_path / "corner.json"
    path.write_text(json.dumps(spec), encoding="utf-8")
    loaded, sample_ids = validate_first_corner_spec(
        path,
        test_index=index,
        metadata_path=metadata,
        frame=pd.DataFrame({"sample_id": ["a", "b"]}),
    )
    assert loaded == spec
    assert sample_ids == ["a", "b"]

    index.write_text("sample_id\na\nb\nc\n", encoding="utf-8")
    with pytest.raises(ValueError, match="hash mismatch"):
        validate_first_corner_spec(
            path,
            test_index=index,
            metadata_path=metadata,
            frame=pd.DataFrame({"sample_id": ["a", "b", "c"]}),
        )
