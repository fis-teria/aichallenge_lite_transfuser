from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from aic_transfuser_lite.data.bag_inventory import (
    discover_bag_inventories,
    scan_bag_metadata,
)


def _write_bag(root: Path, storage: bytes = b"not-decoded") -> Path:
    root.mkdir(parents=True)
    (root / "data_0.mcap").write_bytes(storage)
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": ["data_0.mcap"],
            "starting_time": {"nanoseconds_since_epoch": 1_000_000_000},
            "duration": {"nanoseconds": 2_000_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/camera",
                        "type": "sensor_msgs/msg/Image",
                    },
                    "message_count": 20,
                }
            ],
        }
    }
    (root / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    return root


def test_inventory_is_metadata_only_deterministic_and_complete(tmp_path: Path) -> None:
    bag = _write_bag(tmp_path / "bag")
    first = scan_bag_metadata(bag)
    second = scan_bag_metadata(bag)
    assert first == second
    assert first.scan_status == "PASS"
    assert first.duration_sec == pytest.approx(2.0)
    assert first.end_time_ns == 3_000_000_000
    assert first.storage_size_bytes == len(b"not-decoded")
    assert first.topics[0].message_count == 20
    assert len(first.bag_id) == 24


def test_bag_id_is_not_a_basename_and_changes_with_content(tmp_path: Path) -> None:
    first = _write_bag(tmp_path / "one" / "same", b"first")
    second = _write_bag(tmp_path / "two" / "same", b"first")
    first_id = scan_bag_metadata(first).bag_id
    second_id = scan_bag_metadata(second).bag_id
    assert first_id != second_id
    (first / "data_0.mcap").write_bytes(b"changed")
    assert scan_bag_metadata(first).bag_id != first_id


def test_missing_metadata_and_storage_are_explicit(tmp_path: Path) -> None:
    missing_metadata = scan_bag_metadata(tmp_path / "absent")
    assert missing_metadata.scan_status == "FAIL"
    assert missing_metadata.scan_errors == ("metadata_missing",)

    bag = _write_bag(tmp_path / "bag")
    (bag / "data_0.mcap").unlink()
    missing_storage = scan_bag_metadata(bag)
    assert missing_storage.scan_status == "FAIL"
    assert "storage_missing:data_0.mcap" in missing_storage.scan_errors


def test_broken_metadata_and_topic_entries_fail_without_decode(tmp_path: Path) -> None:
    broken = tmp_path / "broken"
    broken.mkdir()
    (broken / "metadata.yaml").write_text("[", encoding="utf-8")
    record = scan_bag_metadata(broken)
    assert record.scan_status == "FAIL"
    assert record.scan_errors[0].startswith("metadata_parse_error:")

    bag = _write_bag(tmp_path / "bad_topic")
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    info["topics_with_message_count"][0]["message_count"] = -1
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    record = scan_bag_metadata(bag)
    assert record.scan_status == "FAIL"
    assert any(error.startswith("topic_invalid:") for error in record.scan_errors)


def test_discovery_is_source_sorted(tmp_path: Path) -> None:
    _write_bag(tmp_path / "b")
    _write_bag(tmp_path / "a")
    records = discover_bag_inventories(tmp_path)
    assert [Path(record.source_path).name for record in records] == ["a", "b"]
