"""Install pinned DINOv3 source and register local weights inside this workspace.

Run with the workspace Python; no pip changes, weight download, or license
acceptance is performed. --checkpoint imports an already acquired official
ViT-S/16 .pth file only after checking its full SHA-256.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

SOURCE_URL = "https://github.com/facebookresearch/dinov3.git"
SOURCE_COMMIT = "6876159a11b4df116f30f667f8c9888617df0751"
CHECKPOINT_NAME = "dinov3_vits16_pretrain_lvd1689m-08c60483.pth"
CHECKPOINT_SHA256 = "08c60483bc63c04f533611e34bf70b120eedb7240f469bc16e9e20bf344b941d"


def workspace_path(root: Path, relative: str) -> Path:
    """Resolve an asset path, rejecting traversal/symlinks outside the workspace."""
    root = root.resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or path == root:
        raise ValueError(f"asset path must stay inside workspace: {relative}")
    return path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def import_checkpoint(source: Path, target: Path, expected: str) -> None:
    """Verify bytes before publishing; never overwrite an existing different file."""
    if sha256(source) != expected:
        raise ValueError("checkpoint SHA-256 mismatch; expected official ViT-S/16 LVD-1689M .pth")
    if target.exists():
        if sha256(target) != expected:
            raise ValueError(f"existing checkpoint differs; preserved: {target}")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, suffix=".partial", delete=False) as stream:
        temporary = Path(stream.name)
    try:
        shutil.copyfile(source, temporary)
        if sha256(temporary) != expected:
            raise ValueError("checkpoint changed during copy")
        # Atomic publication in the same directory; link fails if target exists.
        os.link(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)


def git(*args: str) -> str:
    result = subprocess.run(["git", *args], check=True, text=True, stdout=subprocess.PIPE)
    return result.stdout.strip()


def install_source(path: Path) -> None:
    if not path.exists():
        git("clone", "--no-checkout", "--depth", "1", SOURCE_URL, str(path))
        git("-C", str(path), "fetch", "--depth", "1", "origin", SOURCE_COMMIT)
        git("-C", str(path), "checkout", "--detach", SOURCE_COMMIT)
    if (not (path / ".git").is_dir()
            or git("-C", str(path), "remote", "get-url", "origin") != SOURCE_URL
            or git("-C", str(path), "rev-parse", "HEAD") != SOURCE_COMMIT
            or git("-C", str(path), "status", "--porcelain=v1", "--untracked-files=all")):
        raise ValueError(f"DINOv3 source must be clean at pinned official revision; preserved: {path}")


def write_matching_json(path: Path, value: dict[str, Any]) -> None:
    """Repeated setup must not silently replace a manually edited model config."""
    if path.exists():
        if json.loads(path.read_text(encoding="utf-8")) != value:
            raise ValueError(f"existing local configuration differs; preserved: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def prepare(root: Path, checkpoint: Path | None = None) -> dict[str, Any]:
    root = root.resolve()
    repository = workspace_path(root, "third_party/dinov3")
    target = workspace_path(root, f"weights/dinov3/{CHECKPOINT_NAME}")
    config_path = workspace_path(root, "runs/setup/dinov3/model_config.json")
    for relative in ("third_party", "weights/dinov3", ".cache/torch", ".cache/huggingface", ".cache/pip"):
        workspace_path(root, relative).mkdir(parents=True, exist_ok=True)
    install_source(repository)
    if checkpoint is not None:
        import_checkpoint(checkpoint.expanduser().resolve(), target, CHECKPOINT_SHA256)
    if target.exists() and sha256(target) != CHECKPOINT_SHA256:
        raise ValueError(f"existing checkpoint SHA-256 mismatch; preserved: {target}")
    config = json.loads((root / "configs/time_path_p1/command_off_dinov3.json").read_text(encoding="utf-8"))
    config["camera_encoder"].update(repository_path=str(repository), checkpoint_path=str(target),
                                    checkpoint_sha256=CHECKPOINT_SHA256)
    write_matching_json(config_path, config)
    manifest = {
        "source_url": SOURCE_URL, "source_commit": SOURCE_COMMIT,
        "repository_path": str(repository), "checkpoint_path": str(target),
        "checkpoint_sha256": CHECKPOINT_SHA256, "checkpoint_present": target.is_file(),
        "model_config_path": str(config_path), "workspace_root": str(root),
        "status": "READY_FOR_PRETRAINED_SMOKE" if target.is_file() else "SOURCE_READY_WEIGHTS_MISSING",
        "pretrained_smoke": "NOT_RUN",
    }
    manifest_path = config_path.with_name("setup.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, help="Previously acquired official .pth to import")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    print(json.dumps(prepare(root, args.checkpoint), indent=2))


if __name__ == "__main__":
    main()
