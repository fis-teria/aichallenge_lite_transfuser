from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.data.recovery_split_inputs_v3 import (
    build_recovery_split_inputs_v3,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build run-level Dataset V3 split inputs for recovery recordings."
    )
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map-or-course-id", default="d1")
    parser.add_argument("--vehicle-profile-id", default="awsim_racing_kart")
    parser.add_argument("--source-dataset-id", required=True)
    args = parser.parse_args()

    records = build_recovery_split_inputs_v3(
        args.raw_root,
        map_or_course_id=args.map_or_course_id,
        vehicle_profile_id=args.vehicle_profile_id,
        source_dataset_id=args.source_dataset_id,
    )
    if args.output.exists():
        raise FileExistsError(f"split input output already exists: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(records, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": "COMPLETE", "run_count": len(records)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
