from __future__ import annotations

import hashlib
from pathlib import Path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_ros_vendor_is_byte_identical_to_authoritative_training_source() -> None:
    package_root = Path(__file__).resolve().parents[1]
    repository_root = package_root.parents[2]
    source_root = repository_root / "src" / "aic_transfuser_lite"
    vendor_root = package_root / "aic_transfuser_lite"
    manifest_path = package_root / "aic_transfuser_lite_vendor.sha256"
    manifest = {}
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        digest, relative = line.split("  ", 1)
        manifest[relative] = digest
    vendor_files = sorted(path for path in vendor_root.rglob("*.py") if path.is_file())
    assert set(manifest) == {
        path.relative_to(vendor_root).as_posix() for path in vendor_files
    }
    mismatches = [
        f"manifest:{relative}"
        for relative, expected in sorted(manifest.items())
        if _sha256(vendor_root / relative) != expected
    ]

    if not source_root.is_dir():
        assert mismatches == []
        return
    source_files = sorted(path for path in source_root.rglob("*.py") if path.is_file())
    assert set(manifest) == {
        path.relative_to(source_root).as_posix() for path in source_files
    }
    for source in source_files:
        relative = source.relative_to(source_root)
        vendor = vendor_root / relative
        if not vendor.is_file():
            mismatches.append(f"missing:{relative.as_posix()}")
        elif _sha256(source) != _sha256(vendor):
            mismatches.append(f"different:{relative.as_posix()}")
    assert mismatches == []
