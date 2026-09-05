#!/usr/bin/env python3
"""Bounded, read-only canonical spatial teacher audit; no training/inference."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from aic_transfuser_lite.data.spatial_coverage_v4 import SpatialAuditConfig, run_audit


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-identity")
    parser.add_argument("--behavior-view", type=Path)
    parser.add_argument("--phase-view", type=Path)
    parser.add_argument("--phase-parent", type=Path)
    parser.add_argument("--model-config", type=Path, default=ROOT / "configs/models/trajectory_authoritative_finetune_v3.yaml")
    parser.add_argument("--splits", nargs="+", choices=("train", "val", "test"), default=["train", "val"])
    parser.add_argument("--detailed-test", action="store_true")
    parser.add_argument("--max-anchors", type=int, default=100000)
    parser.add_argument("--max-seconds", type=float, default=600)
    args = parser.parse_args()
    config = SpatialAuditConfig(splits=tuple(args.splits), detailed_test=args.detailed_test,
                                max_anchors=args.max_anchors, max_seconds=args.max_seconds)
    report = run_audit(dataset_root=args.dataset_root, split_manifest=args.split_manifest,
        output=args.output, repo=ROOT, model_config_path=args.model_config, config=config,
        expected_identity=args.expected_identity, behavior_view=args.behavior_view,
        phase_view=args.phase_view, phase_parent=args.phase_parent, command=sys.argv)
    print(json.dumps({key: report.get(key) for key in ("status", "raw_anchor_count", "processed_geometry_count", "blocking_reasons", "output")}))
    return 2 if report["status"] == "BLOCKED" else 0


if __name__ == "__main__":
    raise SystemExit(main())
