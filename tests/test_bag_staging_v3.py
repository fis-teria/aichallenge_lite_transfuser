from __future__ import annotations

import os
from pathlib import Path

import pytest

from aic_transfuser_lite.data.bag_staging_v3 import stage_rosbag_directories_v3


def _bag(root: Path, name: str) -> Path:
    bag = root / name
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (bag / f"{name}_0.mcap").write_bytes(name.encode("utf-8"))
    return bag


def test_stage_rosbags_uses_flat_atomic_hardlinks(tmp_path: Path) -> None:
    first = _bag(tmp_path / "first", "run01")
    second = _bag(tmp_path / "second" / "nested", "run02")
    output = tmp_path / "stage"
    manifest = stage_rosbag_directories_v3(
        (tmp_path / "first", tmp_path / "second"),
        output,
    )
    assert manifest["bag_count"] == 2
    assert (output / "run01" / "metadata.yaml").is_file()
    assert (output / "run02" / "metadata.yaml").is_file()
    assert os.stat(first / "run01_0.mcap").st_ino == os.stat(
        output / "run01" / "run01_0.mcap"
    ).st_ino
    assert os.stat(second / "run02_0.mcap").st_ino == os.stat(
        output / "run02" / "run02_0.mcap"
    ).st_ino


def test_stage_rosbags_rejects_duplicate_names_before_output(tmp_path: Path) -> None:
    _bag(tmp_path / "first", "duplicate")
    _bag(tmp_path / "second", "duplicate")
    output = tmp_path / "stage"
    with pytest.raises(ValueError, match="duplicate rosbag directory name"):
        stage_rosbag_directories_v3(
            (tmp_path / "first", tmp_path / "second"),
            output,
        )
    assert not output.exists()
