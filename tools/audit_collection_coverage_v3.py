#!/usr/bin/env python3
from __future__ import annotations

import argparse
from bisect import bisect_left
import json
from pathlib import Path
from typing import Sequence

import yaml

from aic_transfuser_lite.data.collection_reference_v3 import (
    TeacherStateSampleV3,
    classify_collection_coverage_v3,
    load_collection_criteria_v3,
    load_route_reference_v3,
    verify_route_reference_manifest_v3,
    write_coverage_report_v3,
)
from aic_transfuser_lite.data.mcap_reader_v3 import read_teacher_state_streams_v3
from aic_transfuser_lite.data.topic_profile_v3 import load_topic_profile_v3


def _find_bags(root: Path) -> tuple[Path, ...]:
    candidates = {path.parent for path in root.rglob("metadata.yaml")}
    if (root / "metadata.yaml").is_file():
        candidates.add(root)
    return tuple(sorted(candidates))


def _run_identity(bag: Path) -> tuple[str, str]:
    candidates = (
        bag / "recording_manifest.yaml",
        bag.parent / f"{bag.name}.recording_manifest.yaml",
    )
    for path in candidates:
        if not path.is_file():
            continue
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        if isinstance(raw, dict):
            return str(raw.get("run_id", bag.name)), str(raw.get("scenario_id", "unknown"))
    return bag.name, "unknown"


def _sample_run(
    bag: Path,
    *,
    profile_path: Path,
    sample_rate_hz: float,
) -> tuple[TeacherStateSampleV3, ...]:
    profile = load_topic_profile_v3(profile_path)
    streams = read_teacher_state_streams_v3(bag, profile=profile)
    if not streams.poses or not streams.velocities:
        raise ValueError(f"empty pose or velocity stream: {bag}")
    run_id, scenario_id = _run_identity(bag)
    velocity_times = [item.timestamp_ns for item in streams.velocities]
    period_ns = int(1e9 / sample_rate_hz)
    next_timestamp_ns = streams.poses[0].timestamp_ns
    result: list[TeacherStateSampleV3] = []
    for pose in streams.poses:
        if pose.timestamp_ns < next_timestamp_ns:
            continue
        velocity_index = bisect_left(velocity_times, pose.timestamp_ns)
        choices = [index for index in (velocity_index - 1, velocity_index) if 0 <= index < len(velocity_times)]
        nearest_index = min(choices, key=lambda index: abs(velocity_times[index] - pose.timestamp_ns))
        if abs(velocity_times[nearest_index] - pose.timestamp_ns) > 50_000_000:
            continue
        velocity = streams.velocities[nearest_index]
        result.append(
            TeacherStateSampleV3(
                run_id=run_id,
                scenario_id=scenario_id,
                timestamp_ns=pose.timestamp_ns,
                x_m=pose.x_world_m,
                y_m=pose.y_world_m,
                yaw_rad=pose.yaw_world_rad,
                speed_mps=velocity.longitudinal_mps,
                yaw_rate_rps=velocity.yaw_rate_rps,
            )
        )
        next_timestamp_ns = pose.timestamp_ns + period_ns
    return tuple(result)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit raw V3 bags against teacher-only route and recovery coverage gates."
    )
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--topic-profile", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--criteria", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    criteria = load_collection_criteria_v3(args.criteria)
    verify_route_reference_manifest_v3(args.reference)
    route = load_route_reference_v3(args.reference)
    bags = _find_bags(args.input_root)
    if not bags:
        raise FileNotFoundError(f"no rosbag2 metadata found below {args.input_root}")
    samples: list[TeacherStateSampleV3] = []
    bag_summary: list[dict[str, object]] = []
    for bag in bags:
        run_samples = _sample_run(
            bag,
            profile_path=args.topic_profile,
            sample_rate_hz=criteria.sample_rate_hz,
        )
        samples.extend(run_samples)
        bag_summary.append({"bag": str(bag), "samples": len(run_samples)})
    report = classify_collection_coverage_v3(samples, route, criteria)
    report["bags"] = bag_summary
    report["reference"] = str(args.reference.resolve())
    report["criteria"] = str(args.criteria.resolve())
    write_coverage_report_v3(args.output, report)
    gaps_path = args.output.with_name(f"{args.output.stem}.gaps.json")
    gaps_path.write_text(
        json.dumps(report["collection_gaps"], indent=2, sort_keys=True), encoding="utf-8"
    )
    print(f"coverage={report['overall_status']} samples={report['sample_count']} report={args.output}")
    return 0 if report["overall_status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
