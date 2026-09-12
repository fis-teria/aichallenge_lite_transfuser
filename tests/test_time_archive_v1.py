import hashlib
import io
import json
import tarfile
from pathlib import Path

import pytest

from aic_transfuser_lite.data.time_archive_v1 import verify_and_extract_time_archive


PREFIX = "time_teacher_20laps_20260911"


def _make_archive(tmp_path: Path, files: dict[str, bytes], *, duplicate: bool = False, traversal: bool = False, corrupt_hash: bool = False, missing: bool = False):
    manifest_entries = [
        {"path": name, "source": "/original/source/" + name, "size_bytes": len(data), "sha256": ("0" * 64 if corrupt_hash else hashlib.sha256(data).hexdigest())}
        for name, data in files.items()
    ]
    if missing:
        manifest_entries.append({"path": "missing.bin", "source": "/original/source/missing.bin", "size_bytes": 1, "sha256": "0" * 64})
    manifest = json.dumps({"files": manifest_entries}, separators=(",", ":")).encode()
    archive = tmp_path / "time.tar"
    with tarfile.open(archive, "w") as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(f"{PREFIX}/{name}")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
        manifest_info = tarfile.TarInfo(f"{PREFIX}/MANIFEST.json")
        manifest_info.size = len(manifest)
        tar.addfile(manifest_info, io.BytesIO(manifest))
        if duplicate:
            info = tarfile.TarInfo(f"{PREFIX}/a.bin")
            info.size = 1
            tar.addfile(info, io.BytesIO(b"x"))
        if traversal:
            info = tarfile.TarInfo(f"{PREFIX}/../escape.bin")
            info.size = 1
            tar.addfile(info, io.BytesIO(b"x"))
    return archive, manifest


def _args(archive: Path, manifest: bytes, members: int):
    return archive, hashlib.sha256(archive.read_bytes()).hexdigest(), hashlib.sha256(manifest).hexdigest(), members


def test_all_valid_extracts_atomically(tmp_path: Path):
    archive, manifest = _make_archive(tmp_path, {"a.bin": b"abc", "nested/b.bin": b"def"})
    destination = tmp_path / "out"
    archive, outer, inner, members = _args(archive, manifest, 3)
    result = verify_and_extract_time_archive(archive, destination, outer, inner, members)
    assert (destination / "a.bin").read_bytes() == b"abc"
    assert (destination / "nested/b.bin").read_bytes() == b"def"
    assert result["validated_member_count"] == 3


def test_bad_outer_hash_rejected_without_partial(tmp_path: Path):
    archive, manifest = _make_archive(tmp_path, {"a.bin": b"abc"})
    with pytest.raises(ValueError, match="archive sha256"):
        verify_and_extract_time_archive(archive, tmp_path / "out", "0" * 64, hashlib.sha256(manifest).hexdigest(), 2)


def test_bad_inner_hash_preserves_partial(tmp_path: Path):
    archive, manifest = _make_archive(tmp_path, {"a.bin": b"abc"}, corrupt_hash=True)
    with pytest.raises(ValueError, match="content mismatch"):
        verify_and_extract_time_archive(archive, tmp_path / "out", hashlib.sha256(archive.read_bytes()).hexdigest(), hashlib.sha256(manifest).hexdigest(), 2)
    assert (tmp_path / "out.partial").exists()


@pytest.mark.parametrize("kind", ["traversal", "duplicate", "missing"])
def test_bad_inventory_rejected(tmp_path: Path, kind: str):
    archive, manifest = _make_archive(tmp_path, {"a.bin": b"abc"}, duplicate=kind == "duplicate", traversal=kind == "traversal", missing=kind == "missing")
    expected_members = 3
    with pytest.raises(ValueError):
        verify_and_extract_time_archive(archive, tmp_path / "out", hashlib.sha256(archive.read_bytes()).hexdigest(), hashlib.sha256(manifest).hexdigest(), expected_members)


def test_destination_collision_rejected(tmp_path: Path):
    archive, manifest = _make_archive(tmp_path, {"a.bin": b"abc"})
    destination = tmp_path / "out"
    destination.mkdir()
    with pytest.raises(FileExistsError):
        verify_and_extract_time_archive(archive, destination, hashlib.sha256(archive.read_bytes()).hexdigest(), hashlib.sha256(manifest).hexdigest(), 2)
