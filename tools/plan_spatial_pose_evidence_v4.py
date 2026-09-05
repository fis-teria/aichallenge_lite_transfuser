"""PLAN ONLY CLI: no raw execution switch or dataset input."""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from aic_transfuser_lite.data.spatial_pose_evidence_plan_v4 import run_plan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conflict-root", required=True, type=Path)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        manifest = run_plan(args.conflict_root, args.evidence_root, args.output, ROOT)
    except (OSError, ValueError) as error:
        parser.exit(3, "BLOCKED: " + str(error) + "\n")
    print(json.dumps({k: manifest[k] for k in ("status", "plan_commit", "logical_plan_identity", "exit_code")}, indent=2))
    return manifest["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
