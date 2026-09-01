#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.data.mcap_converter import ConverterConfig, convert_dataset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert compressed rosbag2 MCAP runs to the canonical dataset."
    )
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--wheelbase-m",
        type=float,
        required=True,
        help="Vehicle wheelbase in metres; required for command-derived waypoint labels.",
    )
    parser.add_argument("--label-shift-ms", type=float, default=0.0)
    parser.add_argument("--target-speed-offset-sec", type=float, default=0.5)
    parser.add_argument("--stop-speed-threshold-mps", type=float, default=0.1)
    parser.add_argument(
        "--min-usable-commanded-speed-mps",
        type=float,
        default=0.5,
        help="Exclude a run when max abs(commanded speed) is below this value.",
    )
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--test-ratio", type=float, default=0.1)
    parser.add_argument("--split-seed", type=int, default=42)
    parser.add_argument("--max-runs", type=int)
    parser.add_argument("--max-samples-per-run", type=int)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    project_config = load_config(args.config)
    data_config = project_config["data"]
    num_waypoints = int(data_config["num_waypoints"])
    horizon_sec = float(data_config["prediction_horizon_sec"])
    waypoint_times_sec = tuple(
        horizon_sec * (index + 1) / num_waypoints for index in range(num_waypoints)
    )
    converter_config = ConverterConfig(
        sample_rate_hz=float(data_config["sample_rate_hz"]),
        sync_tolerance_ms=float(data_config["sync_tolerance_ms"]),
        lidar_points=int(data_config["lidar_points"]),
        waypoint_times_sec=waypoint_times_sec,
        wheelbase_m=args.wheelbase_m,
        label_shift_ms=args.label_shift_ms,
        target_speed_offset_sec=args.target_speed_offset_sec,
        stop_speed_threshold_mps=args.stop_speed_threshold_mps,
        min_usable_commanded_speed_mps=args.min_usable_commanded_speed_mps,
    )
    metadata = convert_dataset(
        args.input_root,
        args.output,
        converter_config,
        max_runs=args.max_runs,
        max_samples_per_run=args.max_samples_per_run,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        split_seed=args.split_seed,
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "rows": metadata["rows"],
                "split_rows": metadata["split_rows"],
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
