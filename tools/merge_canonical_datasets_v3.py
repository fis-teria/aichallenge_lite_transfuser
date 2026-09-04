from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.canonical_merge_v3 import merge_canonical_datasets_v3


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hard-link and atomically merge Canonical Dataset V3 roots."
    )
    parser.add_argument("--source", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--topic-profile-id", required=True)
    args = parser.parse_args()
    payload = merge_canonical_datasets_v3(
        args.source,
        args.output,
        dataset_id=args.dataset_id,
        topic_profile_id=args.topic_profile_id,
    )
    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
