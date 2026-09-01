from __future__ import annotations

import json
from pathlib import Path

from aic_transfuser_lite.cli import EXIT_SUCCESS, EXIT_VALIDATION, main
from test_bag_inventory_v3 import _write_bag
from test_dataset_v3_converter import _convert
from aic_transfuser_lite.data.canonical_converter_v3 import write_prepared_dataset_v3


ROOT = Path(__file__).parents[1]


def test_bag_scan_command_writes_structured_inventory(tmp_path: Path) -> None:
    _write_bag(tmp_path / "bags/run01")
    output = tmp_path / "inventory.json"
    assert main(["bag", "scan", "--input-root", str(tmp_path / "bags"), "--output", str(output)]) == EXIT_SUCCESS
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["format_version"] == "aic_bag_inventory_v1"
    assert len(payload["bags"]) == 1


def test_dataset_build_dry_run_does_not_create_output(tmp_path: Path) -> None:
    _write_bag(tmp_path / "bags/run01")
    output = tmp_path / "dataset"
    result = main(
        [
            "dataset",
            "build",
            "--input-root",
            str(tmp_path / "bags"),
            "--config",
            str(ROOT / "configs/data/dataset_v3.yaml"),
            "--topic-profile",
            str(ROOT / "configs/data/topic_profile_v3.yaml"),
            "--dataset-id",
            "dataset01",
            "--output",
            str(output),
            "--dry-run",
        ]
    )
    assert result == EXIT_SUCCESS and not output.exists()


def test_view_build_and_resume_are_content_stable(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    write_prepared_dataset_v3(
        dataset,
        dataset_id="dataset01",
        topic_profile_id="default",
        runs=(_convert(),),
        jpeg_quality=90,
    )
    output = tmp_path / "view.json"
    arguments = [
        "view",
        "build",
        "--dataset-root",
        str(dataset),
        "--config",
        str(ROOT / "configs/data/view_v1_compat.yaml"),
        "--output",
        str(output),
    ]
    assert main(arguments) == EXIT_SUCCESS
    assert main([*arguments, "--resume"]) == EXIT_SUCCESS
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["materialization"] == "lazy_from_canonical_assets"


def test_structured_validation_exit_code(tmp_path: Path) -> None:
    result = main(
        [
            "bag",
            "scan",
            "--input-root",
            str(tmp_path / "missing"),
            "--output",
            str(tmp_path / "out.json"),
        ]
    )
    assert result == EXIT_VALIDATION
