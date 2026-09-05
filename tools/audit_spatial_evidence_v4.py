#!/usr/bin/env python3
"""Limited h30 teacher eligibility/context evidence; defaults to dry-run (no raw)."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from aic_transfuser_lite.data.spatial_evidence_v4 import EvidenceConfig, run_evidence_audit
from aic_transfuser_lite.data.spatial_source_reader_v4 import ReadBudget


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("dataset-root", "split-manifest", "previous-audit", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--execute-raw", action="store_true")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--max-episodes", type=int, default=8)
    parser.add_argument("--max-anchors", type=int, default=32)
    parser.add_argument("--before-sec", type=float, default=1.0)
    parser.add_argument("--after-sec", type=float, default=3.1)
    parser.add_argument("--max-source-bytes", type=int, default=256 * 1024**2)
    parser.add_argument("--max-expanded-bytes", type=int, default=1024 * 1024**2)
    parser.add_argument("--max-messages", type=int, default=50000)
    parser.add_argument("--max-seconds", type=float, default=180)
    parser.add_argument("--max-temporary-bytes", type=int, default=0)
    parser.add_argument("--max-record-bytes", type=int, default=64 * 1024**2)
    args = parser.parse_args()
    config = EvidenceConfig(max_episodes=args.max_episodes, max_anchors=args.max_anchors,
                            before_sec=args.before_sec, after_sec=args.after_sec)
    budget = ReadBudget(max_source_bytes=args.max_source_bytes, max_expanded_bytes=args.max_expanded_bytes,
        max_messages=args.max_messages, max_seconds=args.max_seconds, max_temporary_bytes=args.max_temporary_bytes,
        max_record_bytes=args.max_record_bytes)
    result = run_evidence_audit(root=args.dataset_root, split=args.split_manifest, previous=args.previous_audit,
        output=args.output, repo=ROOT, config=config, budget=budget, execute_raw=args.execute_raw,
        approved_plan=args.plan, command=sys.argv)
    print(json.dumps({k: result[k] for k in ("status", "selected_anchor_count", "tiers", "source_reader_statuses")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
