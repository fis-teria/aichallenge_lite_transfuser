#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from aic_transfuser_lite.data.recovery_reference_v3 import (
    generate_recovery_reference_v3,
    load_mpc_reference_v3,
    load_occupancy_map_v3,
    load_recovery_reference_config_v3,
    manifest_summary_json,
    write_generated_recovery_reference_v3,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate map-validated MPC References for lateral recovery collection."
    )
    parser.add_argument("--base-reference", type=Path, required=True)
    parser.add_argument("--occupancy-map-yaml", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_recovery_reference_config_v3(args.config)
    generated = generate_recovery_reference_v3(
        load_mpc_reference_v3(args.base_reference),
        load_occupancy_map_v3(args.occupancy_map_yaml),
        config,
    )
    intervals, manifest = write_generated_recovery_reference_v3(
        args.output,
        generated,
        base_reference_path=args.base_reference,
        occupancy_map_yaml=args.occupancy_map_yaml,
        generator_config_path=args.config,
    )
    print(manifest_summary_json(generated))
    print(f"reference_csv={args.output.resolve()}")
    print(f"intervals_csv={intervals.resolve()}")
    print(f"manifest_yaml={manifest.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
