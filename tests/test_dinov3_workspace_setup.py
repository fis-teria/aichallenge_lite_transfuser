from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from tools.setup_dinov3_workspace import import_checkpoint, workspace_path, write_matching_json


def test_weight_import_checks_hash_and_preserves_existing_bytes(tmp_path: Path) -> None:
    source = tmp_path / "received.pth"
    source.write_bytes(b"test-only-fixture")
    expected = hashlib.sha256(source.read_bytes()).hexdigest()
    target = tmp_path / "workspace/weights/model.pth"
    with pytest.raises(ValueError, match="SHA-256"):
        import_checkpoint(source, target, "0" * 64)
    assert not target.exists()
    import_checkpoint(source, target, expected)
    stamp = target.stat().st_mtime_ns
    import_checkpoint(source, target, expected)
    assert target.read_bytes() == source.read_bytes() and target.stat().st_mtime_ns == stamp
    target.write_bytes(b"independent-existing-file")
    with pytest.raises(ValueError, match="existing checkpoint differs"):
        import_checkpoint(source, target, expected)
    assert target.read_bytes() == b"independent-existing-file"


def test_asset_paths_stay_within_workspace(tmp_path: Path) -> None:
    root = tmp_path / "workspace"
    assert workspace_path(root, "weights/dinov3").is_relative_to(root)
    for relative in ("../outside", ".", str(tmp_path / "outside")):
        with pytest.raises(ValueError, match="inside workspace"):
            workspace_path(root, relative)


def test_local_model_config_is_reusable_and_manual_changes_are_preserved(tmp_path: Path) -> None:
    path = tmp_path / "setup/model_config.json"
    settings = {"camera_encoder": {"repository_path": "local/dinov3", "frozen": True}}
    write_matching_json(path, settings)
    write_matching_json(path, settings)
    changed = {"camera_encoder": {"repository_path": "another/dinov3", "frozen": True}}
    with pytest.raises(ValueError, match="configuration differs"):
        write_matching_json(path, changed)
    assert json.loads(path.read_text(encoding="utf-8")) == settings
