from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.bag_staging_v3 import stage_rosbag_directories_v3


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hard-link multiple rosbag roots into one atomic conversion input."
    )
    parser.add_argument("--input-root", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = stage_rosbag_directories_v3(args.input_root, args.output)
    print(json.dumps({"status": "COMPLETE", "bag_count": manifest["bag_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
