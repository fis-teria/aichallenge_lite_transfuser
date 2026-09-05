#!/usr/bin/env python3
"""Saved JSON pose-conflict audit. Exit 0 complete scope, 2 partial, 3 blocked."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from aic_transfuser_lite.data.spatial_pose_conflict_v4 import ConflictConfig, run_audit


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    defaults = ConflictConfig()
    for name in ("max-file-bytes", "max-total-bytes", "max-records", "max-anchors", "max-steps", "max-pairs-per-group", "max-total-pairs"):
        parser.add_argument("--" + name, type=int, default=getattr(defaults, name.replace("-", "_")))
    parser.add_argument("--max-seconds", type=float, default=defaults.max_seconds)
    args = parser.parse_args()
    values = vars(args)
    root, output = values.pop("evidence_root"), values.pop("output")
    try:
        result = run_audit(root, output, ROOT, ConflictConfig(**values), command=sys.argv)
    except (ValueError, OSError) as error:
        print(json.dumps({"status": "BLOCKED", "reason": str(error)}), file=sys.stderr)
        return 3
    print(json.dumps({k: result[k] for k in ("status", "logical_identity", "exit_code")}))
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
