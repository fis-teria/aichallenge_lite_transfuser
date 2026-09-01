#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd


REQUIRED = {
    "sample_id",
    "run_id",
    "scenario_id",
    "image_path",
    "lidar_path",
    "velocity_mps",
    "steering_rad",
    "target_speed_mps",
    "stop_flag",
}


def resolve(root: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else root / path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    index_path = Path(args.index)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    frame = pd.read_csv(index_path)
    missing_columns = sorted(REQUIRED.difference(frame.columns))
    root = index_path.parent

    missing_images = []
    missing_lidar = []
    if not missing_columns:
        for idx, row in frame.iterrows():
            if not resolve(root, row["image_path"]).is_file():
                missing_images.append(int(idx))
            if not resolve(root, row["lidar_path"]).is_file():
                missing_lidar.append(int(idx))

    numeric_columns = [
        column
        for column in [
            "velocity_mps",
            "steering_rad",
            "heading_rate_rps",
            "target_speed_mps",
            "direct_acceleration_mps2",
            "quality_score",
        ]
        if column in frame.columns
    ]
    stats = {}
    for column in numeric_columns:
        series = pd.to_numeric(frame[column], errors="coerce")
        stats[column] = {
            "count": int(series.notna().sum()),
            "nan": int(series.isna().sum()),
            "min": None if series.dropna().empty else float(series.min()),
            "max": None if series.dropna().empty else float(series.max()),
            "mean": None if series.dropna().empty else float(series.mean()),
            "std": None if series.dropna().empty else float(series.std(ddof=0)),
        }

    report = {
        "rows": int(len(frame)),
        "missing_columns": missing_columns,
        "missing_image_rows": missing_images[:100],
        "missing_lidar_rows": missing_lidar[:100],
        "missing_image_count": len(missing_images),
        "missing_lidar_count": len(missing_lidar),
        "run_count": int(frame["run_id"].nunique()) if "run_id" in frame else None,
        "scenario_count": int(frame["scenario_id"].nunique()) if "scenario_id" in frame else None,
        "stop_rate": float(pd.to_numeric(frame["stop_flag"], errors="coerce").mean())
        if "stop_flag" in frame and len(frame)
        else None,
        "statistics": stats,
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))

    invalid = bool(missing_columns or missing_images or missing_lidar)
    if invalid:
        sys.exit(2)


if __name__ == "__main__":
    main()
