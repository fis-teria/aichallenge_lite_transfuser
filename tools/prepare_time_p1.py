"""Create reviewable TimePath configs/splits, or verify sources after SSD migration.

This command never trains, copies raw data, or launches a simulator.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from aic_transfuser_lite.data.time_split_v1 import (
    split_from_packaging_receipt, verify_time_split_sources, command_comparison_plan,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    draft = sub.add_parser("draft")
    draft.add_argument("--receipt", type=Path, required=True)
    draft.add_argument("--output-dir", type=Path, required=True)
    draft.add_argument("--seed", type=int, default=42)
    draft.add_argument("--max-anchors", type=int, required=True)
    draft.add_argument("--max-optimizer-steps", type=int, required=True)
    verify = sub.add_parser("verify-sources")
    verify.add_argument("--split", type=Path, required=True)
    verify.add_argument("--raw-root", type=Path, required=True)
    verify.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "draft":
        manifest = split_from_packaging_receipt(args.receipt, seed=args.seed)
        plan = command_comparison_plan(manifest, seed=args.seed, max_anchors=args.max_anchors,
                                       max_optimizer_steps=args.max_optimizer_steps)
        config = TimeModelConfig()
        outputs = {"split_draft.json": manifest, "comparison_plan.json": plan,
                   "command_off.json": config.to_dict(),
                   "command_on.json": replace(config,use_command_history=True).to_dict()}
        args.output_dir.mkdir(parents=True,exist_ok=True)
        for name,value in outputs.items():
            (args.output_dir/name).write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf-8")
        print(json.dumps({"status":"PREPARATION_ONLY", "run_count":len(manifest["runs"]),
                          "sources_verified":False,"split_sha256":manifest["manifest_sha256"]}))
    else:
        manifest = json.loads(args.split.read_text(encoding="utf-8"))
        verified = verify_time_split_sources(manifest,args.raw_root)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(verified,indent=2)+"\n",encoding="utf-8")
        print("SOURCES_HASH_VERIFIED")


if __name__ == "__main__":
    main()
