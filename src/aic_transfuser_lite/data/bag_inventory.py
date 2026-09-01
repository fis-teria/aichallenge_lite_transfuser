from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class StorageFileInventory:
    relative_path: str
    size_bytes: int
    sha256: str


@dataclass(frozen=True)
class TopicInventory:
    name: str
    message_type: str
    message_count: int


@dataclass(frozen=True)
class BagInventoryRecord:
    bag_id: str
    source_path: str
    metadata_path: str
    metadata_sha256: str | None
    storage_files: tuple[StorageFileInventory, ...]
    storage_sha256: str | None
    storage_size_bytes: int
    start_time_ns: int | None
    end_time_ns: int | None
    duration_sec: float | None
    topics: tuple[TopicInventory, ...]
    storage_identifier: str | None
    scan_status: str
    scan_errors: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scan_bag_metadata(bag_directory: str | Path) -> BagInventoryRecord:
    """Inventory one Rosbag2 directory without decoding message payloads.

    The result always records scan errors. A valid record uses a content and
    source-addressed 24-hex bag ID so equal basenames in different roots cannot
    collide silently.
    """

    bag_dir = Path(bag_directory).resolve()
    metadata_path = bag_dir / "metadata.yaml"
    canonical_source_uri = bag_dir.as_uri()
    errors: list[str] = []
    metadata_sha: str | None = None
    storage_sha: str | None = None
    storage_files: list[StorageFileInventory] = []
    topics: list[TopicInventory] = []
    start_ns: int | None = None
    end_ns: int | None = None
    duration_sec: float | None = None
    storage_identifier: str | None = None

    if not metadata_path.is_file():
        errors.append("metadata_missing")
        return _record(
            canonical_source_uri=canonical_source_uri,
            metadata_path=metadata_path,
            metadata_sha=None,
            storage_files=(),
            storage_sha=None,
            start_ns=None,
            end_ns=None,
            duration_sec=None,
            topics=(),
            storage_identifier=None,
            errors=errors,
        )

    metadata_sha = sha256_file(metadata_path)
    try:
        raw = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = _information_mapping(raw)
    except Exception as error:
        errors.append(f"metadata_parse_error:{type(error).__name__}:{error}")
        return _record(
            canonical_source_uri=canonical_source_uri,
            metadata_path=metadata_path,
            metadata_sha=metadata_sha,
            storage_files=(),
            storage_sha=None,
            start_ns=None,
            end_ns=None,
            duration_sec=None,
            topics=(),
            storage_identifier=None,
            errors=errors,
        )

    storage_identifier = _optional_string(info.get("storage_identifier"))
    relative_paths = info.get("relative_file_paths")
    if not isinstance(relative_paths, list) or not relative_paths:
        errors.append("storage_paths_missing")
        relative_paths = []
    seen_paths: set[str] = set()
    for raw_path in relative_paths:
        relative = Path(str(raw_path))
        normalized = relative.as_posix()
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"storage_path_unsafe:{normalized}")
            continue
        if normalized in seen_paths:
            errors.append(f"storage_path_duplicate:{normalized}")
            continue
        seen_paths.add(normalized)
        storage_path = bag_dir / relative
        if not storage_path.is_file():
            errors.append(f"storage_missing:{normalized}")
            continue
        storage_files.append(
            StorageFileInventory(
                relative_path=normalized,
                size_bytes=storage_path.stat().st_size,
                sha256=sha256_file(storage_path),
            )
        )
    storage_files.sort(key=lambda item: item.relative_path)
    if storage_files:
        storage_sha = _canonical_sha([asdict(item) for item in storage_files])

    try:
        start_ns = _nested_int(info, "starting_time", "nanoseconds_since_epoch")
        duration_ns = _nested_int(info, "duration", "nanoseconds")
        if start_ns < 0 or duration_ns < 0:
            raise ValueError("start and duration must be non-negative")
        end_ns = start_ns + duration_ns
        duration_sec = duration_ns / 1e9
    except (KeyError, TypeError, ValueError) as error:
        errors.append(f"timing_invalid:{error}")

    topic_values = info.get("topics_with_message_count")
    if not isinstance(topic_values, list):
        errors.append("topics_missing")
        topic_values = []
    topic_names: set[str] = set()
    for value in topic_values:
        try:
            if not isinstance(value, dict) or not isinstance(
                value.get("topic_metadata"), dict
            ):
                raise ValueError("entry must contain topic_metadata")
            metadata = value["topic_metadata"]
            name = str(metadata["name"])
            message_type = str(metadata["type"])
            count = int(value["message_count"])
            if not name.startswith("/") or not message_type or count < 0:
                raise ValueError("invalid name, type, or count")
            if name in topic_names:
                raise ValueError(f"duplicate topic {name}")
            topic_names.add(name)
            topics.append(TopicInventory(name, message_type, count))
        except (KeyError, TypeError, ValueError) as error:
            errors.append(f"topic_invalid:{error}")
    topics.sort(key=lambda item: item.name)

    return _record(
        canonical_source_uri=canonical_source_uri,
        metadata_path=metadata_path,
        metadata_sha=metadata_sha,
        storage_files=tuple(storage_files),
        storage_sha=storage_sha,
        start_ns=start_ns,
        end_ns=end_ns,
        duration_sec=duration_sec,
        topics=tuple(topics),
        storage_identifier=storage_identifier,
        errors=errors,
    )


def discover_bag_inventories(input_root: str | Path) -> tuple[BagInventoryRecord, ...]:
    """Find Rosbag2 metadata files recursively and return source-sorted records."""

    root = Path(input_root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Bag input root not found: {root}")
    bag_dirs = sorted(
        {path.parent for path in root.rglob("metadata.yaml")}, key=lambda path: path.as_posix()
    )
    return tuple(scan_bag_metadata(path) for path in bag_dirs)


def _record(
    *,
    canonical_source_uri: str,
    metadata_path: Path,
    metadata_sha: str | None,
    storage_files: tuple[StorageFileInventory, ...],
    storage_sha: str | None,
    start_ns: int | None,
    end_ns: int | None,
    duration_sec: float | None,
    topics: tuple[TopicInventory, ...],
    storage_identifier: str | None,
    errors: list[str],
) -> BagInventoryRecord:
    identity = canonical_source_uri + (metadata_sha or "missing") + (storage_sha or "missing")
    bag_id = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:24]
    return BagInventoryRecord(
        bag_id=bag_id,
        source_path=canonical_source_uri,
        metadata_path=metadata_path.as_uri(),
        metadata_sha256=metadata_sha,
        storage_files=storage_files,
        storage_sha256=storage_sha,
        storage_size_bytes=sum(item.size_bytes for item in storage_files),
        start_time_ns=start_ns,
        end_time_ns=end_ns,
        duration_sec=duration_sec,
        topics=topics,
        storage_identifier=storage_identifier,
        scan_status="PASS" if not errors else "FAIL",
        scan_errors=tuple(errors),
    )


def _information_mapping(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("metadata root must be a mapping")
    value = raw.get("rosbag2_bagfile_information", raw)
    if not isinstance(value, dict):
        raise ValueError("rosbag2_bagfile_information must be a mapping")
    return value


def _nested_int(mapping: dict[str, Any], outer: str, inner: str) -> int:
    value = mapping[outer]
    if not isinstance(value, dict):
        raise TypeError(f"{outer} must be a mapping")
    return int(value[inner])


def _optional_string(value: Any) -> str | None:
    if value is None:
        return None
    result = str(value)
    return result or None


def _canonical_sha(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
