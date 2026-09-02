#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from aic_transfuser_lite.data.calibration.artifact import (  # noqa: E402
    build_calibration_artifact,
    sha256_file,
    write_calibration_artifact,
)
from aic_transfuser_lite.data.calibration.lateral import (  # noqa: E402
    fit_lateral_calibration,
)
from aic_transfuser_lite.data.calibration.longitudinal import (  # noqa: E402
    derive_actual_acceleration,
    fit_longitudinal_calibration,
)


def _selected_command(row: dict[str, str]) -> dict[str, object] | None:
    for field in ("nominal_command", "final_command"):
        value = json.loads(row[field])
        if bool(value.get("valid")):
            return value
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fit a candidate V3 actuator calibration artifact"
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--vehicle-profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--acceleration-smoothing-samples", type=int, default=5)
    args = parser.parse_args()

    profile = yaml.safe_load(args.vehicle_profile.read_text(encoding="utf-8"))
    if profile.get("calibration_promotion_limit") != "candidate":
        raise ValueError("this fitter requires a candidate-only vehicle profile")
    wheelbase_m = float(profile["wheelbase_m"])
    max_abs_yaw_rate_rps = float(profile["max_abs_yaw_rate_rps"])

    with (args.dataset_root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 3:
        raise ValueError("calibration dataset must contain at least three samples")
    selected = [_selected_command(row) for row in rows]
    command_valid = np.asarray([item is not None for item in selected], dtype=np.bool_)
    if int(np.count_nonzero(command_valid)) < 3:
        raise ValueError("calibration dataset has insufficient valid commands")

    timestamps = np.asarray([int(row["grid_stamp_ns"]) * 1e-9 for row in rows])
    speed = np.asarray([float(row["velocity_longitudinal_mps"]) for row in rows])
    yaw_rate = np.asarray([float(row["yaw_rate_rps"]) for row in rows])
    actual_steering = np.asarray([float(row["actual_steering_rad"]) for row in rows])
    actual_steering_valid = np.asarray(
        [row["actual_steering_valid"].lower() == "true" for row in rows], dtype=np.bool_
    )
    command_steering = np.asarray([
        float(item["steering_rad"]) if item is not None else np.nan for item in selected
    ])
    command_acceleration = np.asarray([
        float(item["acceleration_mps2"]) if item is not None else np.nan
        for item in selected
    ])
    actual_acceleration = derive_actual_acceleration(
        timestamps,
        speed,
        smoothing_samples=args.acceleration_smoothing_samples,
    )

    lateral_mask = command_valid & actual_steering_valid
    lateral = fit_lateral_calibration(
        timestamps[lateral_mask],
        command_steering[lateral_mask],
        actual_steering[lateral_mask],
        speed[lateral_mask],
        yaw_rate[lateral_mask],
        wheelbase_m=wheelbase_m,
        max_abs_yaw_rate_rps=max_abs_yaw_rate_rps,
    )
    longitudinal_mask = command_valid & np.isfinite(speed) & (speed >= 0.0)
    longitudinal = fit_longitudinal_calibration(
        timestamps[longitudinal_mask],
        command_acceleration[longitudinal_mask],
        actual_acceleration[longitudinal_mask],
        speed[longitudinal_mask],
    )

    with (args.dataset_root / "runs.csv").open(newline="", encoding="utf-8") as stream:
        run_rows = list(csv.DictReader(stream))
    source_run_hashes: dict[str, str] = {}
    for row in run_rows:
        run_id = row["run_id"]
        source_hash = row["source_hash"]
        previous = source_run_hashes.setdefault(run_id, source_hash)
        if previous != source_hash:
            raise ValueError(f"run {run_id!r} has inconsistent source hashes")

    artifact = build_calibration_artifact(
        source_run_hashes=source_run_hashes,
        vehicle_profile_sha256=sha256_file(args.vehicle_profile),
        steering=lateral,
        drive=longitudinal.drive,
        brake=longitudinal.brake,
    )
    write_calibration_artifact(artifact, args.output)
    summary = {
        "artifact": str(args.output),
        "artifact_sha256": artifact.artifact_sha256,
        "promotion_state": artifact.promotion.state,
        "source_run_count": len(artifact.source_runs),
        "steering_valid": artifact.steering.individually_valid,
        "drive_valid": artifact.drive.individually_valid,
        "brake_valid": artifact.brake.individually_valid,
        "all_fits_valid": all((
            artifact.steering.individually_valid,
            artifact.drive.individually_valid,
            artifact.brake.individually_valid,
        )),
    }
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
