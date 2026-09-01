#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from aic_transfuser_lite.data.mcap_converter_v2 import (
    V2ConverterConfig,
    convert_dataset_v2,
    sha256_file,
)


def _find_unique_key(payload: Any, key: str) -> Any:
    matches: list[Any] = []

    def visit(value: Any) -> None:
        if isinstance(value, Mapping):
            for child_key, child_value in value.items():
                if child_key == key:
                    matches.append(child_value)
                visit(child_value)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(payload)
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {key!r} in vehicle config, found {len(matches)}")
    return matches[0]


def load_vehicle_contract(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Vehicle config not found: {path}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    wheelbase_m = float(_find_unique_key(payload, "wheel_base"))
    max_steering_rad = float(_find_unique_key(payload, "max_steer_angle"))
    if wheelbase_m <= 0.0 or max_steering_rad <= 0.0:
        raise ValueError("Vehicle wheelbase and max steering must be positive")
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "wheelbase_m": wheelbase_m,
        "max_steering_rad": max_steering_rad,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert measured-pose Dataset v2 rosbag2 MCAP runs."
    )
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle-config", type=Path, required=True)
    parser.add_argument("--expected-lidar-points", type=int)
    parser.add_argument("--max-runs", type=int)
    parser.add_argument("--max-samples-per-run", type=int)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--test-ratio", type=float, default=0.2)
    parser.add_argument("--split-seed", type=int, default=42)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    vehicle = load_vehicle_contract(args.vehicle_config)
    config = V2ConverterConfig(
        wheelbase_m=float(vehicle["wheelbase_m"]),
        expected_lidar_points=args.expected_lidar_points,
    )
    metadata = convert_dataset_v2(
        args.input_root,
        args.output,
        config,
        max_runs=args.max_runs,
        max_samples_per_run=args.max_samples_per_run,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        split_seed=args.split_seed,
        vehicle_config_provenance=vehicle,
    )
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "format_version": metadata["format_version"],
                "rows": metadata["rows"],
                "split_rows": metadata["split_rows"],
                "dataset_quality": metadata["dataset_quality"],
                "delay_consistency": metadata["delay_consistency"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
