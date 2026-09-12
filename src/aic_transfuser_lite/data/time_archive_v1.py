"""Strict, bounded extraction for the 20-lap time-teacher archive."""

from __future__ import annotations

import hashlib
import json
import os
import tarfile
from pathlib import Path, PurePosixPath
from typing import Any


_PREFIX = "time_teacher_20laps_20260911"
_MANIFEST_NAME = f"{_PREFIX}/MANIFEST.json"
_CHUNK = 1024 * 1024


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(_CHUNK), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _valid_relative_name(name: str) -> bool:
    if not name or "\\" in name or name.startswith("/") or name.startswith("\\"):
        return False
    if len(name) >= 2 and name[1] == ":":
        return False
    if name.endswith("/"):
        return False
    parts = name.split("/")
    return all(part not in ("", ".", "..") for part in parts) and str(PurePosixPath(name)) == name


def _valid_member_name(name: str) -> bool:
    return name.startswith(_PREFIX + "/") and _valid_relative_name(name[len(_PREFIX) + 1 :])


def _read_manifest(archive: Path, members: list[tarfile.TarInfo]) -> tuple[bytes, dict[str, dict[str, Any]]]:
    found = [member for member in members if member.name == _MANIFEST_NAME]
    if len(found) != 1:
        raise ValueError("archive must contain exactly one time-teacher MANIFEST.json")
    manifest_member = found[0]
    if not manifest_member.isfile():
        raise ValueError("MANIFEST.json must be a regular file")
    with tarfile.open(archive, "r:*") as tar:
        stream = tar.extractfile(manifest_member)
        if stream is None:
            raise ValueError("cannot read MANIFEST.json")
        manifest_bytes = stream.read()
    try:
        payload = json.loads(manifest_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("MANIFEST.json is not valid UTF-8 JSON") from exc
    entries = payload if isinstance(payload, list) else payload.get("files") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        raise ValueError("MANIFEST.json must contain a files list")
    result: dict[str, dict[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, dict) or not {"path", "size_bytes", "sha256"}.issubset(entry):
            raise ValueError("manifest entries must contain path, size_bytes, sha256")
        path, size, digest = entry["path"], entry["size_bytes"], entry["sha256"]
        if not isinstance(path, str) or not _valid_relative_name(path) or path == "MANIFEST.json":
            raise ValueError("manifest contains an invalid path")
        if path in result or not isinstance(size, int) or size < 0 or not isinstance(digest, str):
            raise ValueError("manifest contains a duplicate or invalid entry")
        if len(digest) != 64 or any(char not in "0123456789abcdefABCDEF" for char in digest):
            raise ValueError("manifest contains an invalid sha256")
        result[path] = {"size_bytes": size, "sha256": digest.lower()}
    return manifest_bytes, result


def verify_and_extract_time_archive(
    archive: Path,
    destination: Path,
    expected_archive_sha256: str,
    expected_manifest_sha256: str,
    expected_members: int = 948,
) -> dict[str, Any]:
    """Verify and atomically extract a fixed-prefix time-teacher tar archive.

    ``destination`` and its ``.partial`` staging directory must be absent. A
    failed extraction deliberately leaves the partial directory for diagnosis.
    """
    archive = Path(archive)
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(f"destination already exists: {destination}")
    staging = Path(str(destination) + ".partial")
    if staging.exists():
        raise FileExistsError(f"partial destination already exists: {staging}")
    if expected_members < 2:
        raise ValueError("expected_members must include MANIFEST.json and at least one file")
    if _sha256_file(archive).lower() != expected_archive_sha256.lower():
        raise ValueError("archive sha256 mismatch")

    try:
        with tarfile.open(archive, "r:*") as tar:
            members = tar.getmembers()
        if len(members) != expected_members:
            raise ValueError(f"member count mismatch: {len(members)} != {expected_members}")
        names: set[str] = set()
        for member in members:
            if member.name in names:
                raise ValueError(f"duplicate archive member: {member.name}")
            names.add(member.name)
            if not _valid_member_name(member.name) or not member.isfile() or member.size < 0:
                raise ValueError(f"unsafe or non-regular archive member: {member.name}")
        manifest_bytes, entries = _read_manifest(archive, members)
        if hashlib.sha256(manifest_bytes).hexdigest().lower() != expected_manifest_sha256.lower():
            raise ValueError("manifest sha256 mismatch")
        entries = {_PREFIX + "/" + path: value for path, value in entries.items()}
        expected_names = set(entries) | {_MANIFEST_NAME}
        if names != expected_names or len(entries) != expected_members - 1:
            raise ValueError("archive member inventory does not match MANIFEST.json")

        staging.mkdir(parents=False, exist_ok=False)
        validated = 0
        with tarfile.open(archive, "r:*") as tar:
            for member in tar.getmembers():
                target = staging / PurePosixPath(member.name).relative_to(_PREFIX)
                target.parent.mkdir(parents=True, exist_ok=True)
                digest = hashlib.sha256()
                size = 0
                source = tar.extractfile(member)
                if source is None:
                    raise ValueError(f"cannot read archive member: {member.name}")
                with target.open("xb") as output:
                    for chunk in iter(lambda: source.read(_CHUNK), b""):
                        output.write(chunk)
                        digest.update(chunk)
                        size += len(chunk)
                expected = entries.get(member.name)
                if member.name != _MANIFEST_NAME and (size != expected["size_bytes"] or digest.hexdigest() != expected["sha256"]):
                    raise ValueError(f"content mismatch: {member.name}")
                validated += 1
        _rename_without_replace(staging, destination)
        return {"archive_sha256": expected_archive_sha256.lower(), "manifest_sha256": expected_manifest_sha256.lower(),
                "member_count": len(members), "regular_file_count": len(members), "validated_member_count": validated,
                "extracted_member_count": validated, "destination": str(destination)}
    except Exception:
        # Keep staging and its already-written files intact for diagnosis.
        raise


def _rename_without_replace(source: Path, destination: Path) -> None:
    """Atomically publish a directory while refusing a concurrent collision."""
    if os.name == "nt":
        import ctypes
        if not ctypes.windll.kernel32.MoveFileExW(str(source), str(destination), 8):
            raise FileExistsError(f"destination appeared during publication: {destination}")
        return
    # Linux/WSL provides renameat2(RENAME_NOREPLACE), preserving no-overwrite semantics.
    import ctypes
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is not None:
        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        result = renameat2(-100, os.fsencode(source), -100, os.fsencode(destination), 1)
        if result == 0:
            return
        if ctypes.get_errno() == 17:
            raise FileExistsError(f"destination appeared during publication: {destination}")
        raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
    if destination.exists():
        raise FileExistsError(f"destination appeared during publication: {destination}")
    os.rename(source, destination)
