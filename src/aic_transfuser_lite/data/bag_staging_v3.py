from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
from typing import Any, Sequence
import uuid


def stage_rosbag_directories_v3(
    input_roots: Sequence[str | Path],
    output_root: str | Path,
) -> dict[str, Any]:
    """Create a flat, atomic hard-link staging tree for Dataset V3 conversion."""

    sources = tuple(Path(value).resolve() for value in input_roots)
    if not sources or any(not source.is_dir() for source in sources):
        raise FileNotFoundError("every bag input root must be an existing directory")
    output = Path(output_root).resolve()
    if output.exists():
        raise FileExistsError(f"bag staging output already exists: {output}")
    bags: list[Path] = []
    seen_names: set[str] = set()
    for source in sources:
        for metadata in sorted(source.rglob("metadata.yaml")):
            bag = metadata.parent
            if bag.name in seen_names:
                raise ValueError(f"duplicate rosbag directory name: {bag.name!r}")
            seen_names.add(bag.name)
            bags.append(bag)
    if not bags:
        raise ValueError("no rosbag2 metadata.yaml files found")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(f".{output.name}.tmp-{uuid.uuid4().hex[:8]}")
    staging.mkdir()
    try:
        records = []
        for bag in bags:
            destination = staging / bag.name
            destination.mkdir()
            files = sorted(path for path in bag.iterdir() if path.is_file())
            if not files or not (bag / "metadata.yaml").is_file():
                raise ValueError(f"invalid rosbag2 directory: {bag}")
            for source_file in files:
                os.link(source_file, destination / source_file.name)
            records.append(
                {
                    "bag_name": bag.name,
                    "source": str(bag),
                    "file_count": len(files),
                }
            )
        manifest = {
            "format": "aic_rosbag_hardlink_stage_v1",
            "input_roots": [str(value) for value in sources],
            "bag_count": len(records),
            "bags": records,
        }
        (staging / "stage_manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(staging, output)
        return manifest
    except BaseException:
        if staging.exists() and staging.parent == output.parent:
            shutil.rmtree(staging)
        raise
