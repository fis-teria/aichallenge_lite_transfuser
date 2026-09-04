from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from aic_transfuser_lite.data.recovery_phase_view_v3 import (
    load_recovery_phase_view_v3,
    write_recovery_behavior_view_v1,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build Behavior View V1 for recovery-reference Dataset V3."
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--phase-view", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    dataset_manifest = validate_complete_dataset(args.dataset_root)
    dataset_sha = str(dataset_manifest["manifest_sha256"])
    labels = load_recovery_phase_view_v3(
        args.phase_view,
        dataset_manifest_sha256=dataset_sha,
    )
    payload = write_recovery_behavior_view_v1(
        args.output,
        dataset_manifest_sha256=dataset_sha,
        phase_view_manifest_sha256=_sha256(args.phase_view / "manifest.json"),
        labels=labels,
    )
    print(
        json.dumps(
            {
                "status": "COMPLETE",
                "sample_count": payload["sample_count"],
                "valid_behavior_count": payload["valid_behavior_count"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
