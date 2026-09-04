from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.split_merge_v3 import merge_split_manifests_v3


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Merge run assignments for a combined Dataset V3."
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--source-manifest", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"combined split output already exists: {args.output}")
    payload = merge_split_manifests_v3(
        dataset_root=args.dataset_root,
        source_manifest_paths=args.source_manifest,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": "COMPLETE",
                "run_count": len(payload["assignments"]),
                "leakage": payload["leakage"]["status"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
