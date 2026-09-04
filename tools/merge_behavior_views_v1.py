from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.behavior_view_v1 import merge_behavior_views_v1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Merge disjoint Behavior V1 views for a combined Dataset V3."
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--source-view", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = merge_behavior_views_v1(
        dataset_root=args.dataset_root,
        source_view_roots=args.source_view,
        output_root=args.output,
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
