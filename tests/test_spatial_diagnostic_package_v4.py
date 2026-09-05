"""Synthetic-only package hash/path/reference verification."""
import importlib.util
import json
from pathlib import Path
import zipfile

import pytest

spec = importlib.util.spec_from_file_location("diagnostic_package", Path(__file__).parents[1] / "tools/package_spatial_diagnostic_v4.py")
packing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packing)


@pytest.mark.parametrize("name", ["/absolute", "../up", "root/../up", "C:/bad", "root\\bad", "root//bad"])
def test_unsafe_entry(name: str) -> None:
    with pytest.raises(ValueError):
        packing.safe_name(name)


@pytest.mark.parametrize("fault", ["none", "hash", "reference", "extra"])
def test_extract_hash_and_readme_refs(tmp_path: Path, fault: str) -> None:
    blob = b"[evidence](missing.json)" if fault == "reference" else b"[evidence](data.json)"
    files = {"README_REVIEW.md": blob, "data.json": b"{}"}
    manifest = {"files": [{"path": k, "size_bytes": len(v), "sha256": packing.sha(v)} for k, v in files.items()]}
    if fault == "hash":
        manifest["files"][0]["sha256"] = "0"*64
    if fault == "extra":
        files["extra.json"] = b"{}"
    path = tmp_path / "packet.zip"
    with zipfile.ZipFile(path, "x") as archive:
        for name, content in files.items():
            archive.writestr("packet/"+name, content)
        archive.writestr("packet/PACKAGE_MANIFEST.json", json.dumps(manifest))
    if fault == "none":
        assert packing.verify_zip(path, tmp_path)["status"] == "PASS"
    else:
        with pytest.raises(ValueError):
            packing.verify_zip(path, tmp_path)
