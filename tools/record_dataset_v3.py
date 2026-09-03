#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
from typing import Mapping, Sequence

import yaml

from aic_transfuser_lite.data.topic_profile_v3 import (
    TopicProfileV3,
    assess_topic_profile_v3,
    load_topic_profile_v3,
)
from aic_transfuser_lite.data.collection_reference_v3 import (
    verify_route_reference_manifest_v3,
)
DEFAULT_FORBIDDEN_NODE_PATTERN = r"trajectory_authoritative|full_control.*inference|aic_transfuser.*inference"


def _safe_id(value: str, name: str) -> str:
    if not value or re.fullmatch(r"[A-Za-z0-9_-]+", value) is None:
        raise ValueError(f"{name} must use only letters, digits, '-' and '_'")
    return value


def select_recording_topics(
    profile: TopicProfileV3, discovered: Mapping[str, str]
) -> tuple[str, ...]:
    assessment = assess_topic_profile_v3(profile, discovered)
    if assessment.missing_for_recording:
        raise ValueError(f"missing required recording roles: {assessment.missing_for_recording}")
    return tuple(
        spec.name for role, spec in profile.roles.items() if role in assessment.available_roles
    )


def discover_topic_types(
    *,
    required_names: set[str],
    env: Mapping[str, str],
    attempts: int = 3,
) -> dict[str, str]:
    discovered: dict[str, str] = {}
    for attempt in range(attempts):
        result = subprocess.run(
            ["ros2", "topic", "list", "--no-daemon", "--spin-time", "2.0", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
            env=dict(env),
        )
        discovered = {}
        if result.returncode == 0:
            for raw_line in result.stdout.splitlines():
                line = raw_line.strip()
                if line.startswith("/") and " [" in line and line.endswith("]"):
                    name, message_type = line.rsplit(" [", 1)
                    discovered[name] = message_type[:-1]
        if required_names.issubset(discovered):
            return discovered
        if attempt + 1 < attempts:
            time.sleep(0.5)
    return discovered


def build_record_command(output: str | Path, topics: Sequence[str]) -> list[str]:
    if not topics:
        raise ValueError("at least one recording topic is required")
    return [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--compression-mode",
        "file",
        "--compression-format",
        "zstd",
        "--use-sim-time",
        "--output",
        str(output),
        *topics,
    ]


def _discover_nodes(env: Mapping[str, str]) -> tuple[str, ...]:
    result = subprocess.run(
        ["ros2", "node", "list", "--no-daemon", "--spin-time", "2.0"],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
        env=dict(env),
    )
    if result.returncode != 0:
        raise RuntimeError("failed to discover ROS nodes")
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip().startswith("/"))


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan or execute one profile-driven Dataset V3 expert-data recording."
    )
    parser.add_argument("--topic-profile", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--collection-case-id", required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--ros-domain-id", type=int, default=101)
    parser.add_argument("--forbidden-node-pattern", default=DEFAULT_FORBIDDEN_NODE_PATTERN)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    run_id = _safe_id(args.run_id, "run-id")
    scenario_id = _safe_id(args.scenario_id, "scenario-id")
    case_id = _safe_id(args.collection_case_id, "collection-case-id")
    if not 0 <= args.ros_domain_id <= 232:
        raise ValueError("ros-domain-id must be in [0, 232]")
    if args.duration_sec <= 0.0:
        raise ValueError("duration-sec must be positive")
    if not args.reference.is_file():
        raise FileNotFoundError(f"Reference CSV not found: {args.reference}")
    verify_route_reference_manifest_v3(args.reference)
    profile = load_topic_profile_v3(args.topic_profile)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    required_names = {
        spec.name for spec in profile.roles.values() if spec.required_for_recording
    }
    discovered = discover_topic_types(required_names=required_names, env=environment)
    topics = select_recording_topics(profile, discovered)
    nodes = _discover_nodes(environment)
    blocked = sorted(node for node in nodes if re.search(args.forbidden_node_pattern, node))
    if blocked:
        raise RuntimeError(
            "refusing expert-data recording while an E2E inference node is present: "
            f"{blocked}"
        )
    output_root = args.output_root.expanduser().resolve()
    bag_path = output_root / run_id
    sidecar = output_root / f"{run_id}.recording_manifest.yaml"
    command = build_record_command(bag_path, topics)
    plan = {
        "execute": bool(args.execute),
        "run_id": run_id,
        "scenario_id": scenario_id,
        "collection_case_id": case_id,
        "duration_sec": args.duration_sec,
        "topics": list(topics),
        "command": command,
    }
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    if bag_path.exists() or sidecar.exists():
        raise FileExistsError(f"refusing to overwrite recording or manifest for {run_id}")
    output_root.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, object] = {
        "format_version": 3,
        "status": "recording",
        "run_id": run_id,
        "scenario_id": scenario_id,
        "collection_case_id": case_id,
        "ros_domain_id": args.ros_domain_id,
        "started_utc": _utc_now(),
        "finished_utc": None,
        "duration_sec": args.duration_sec,
        "topic_profile": str(args.topic_profile.resolve()),
        "reference": str(args.reference.resolve()),
        "reference_sha256": hashlib.sha256(args.reference.read_bytes()).hexdigest(),
        "teacher_debug_only_reference": True,
        "command": command,
        "topics": list(topics),
    }
    sidecar.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    process = subprocess.Popen(command, env=environment)
    intentional_stop = False
    try:
        try:
            return_code = process.wait(timeout=args.duration_sec)
        except subprocess.TimeoutExpired:
            intentional_stop = True
            process.send_signal(signal.SIGINT)
            return_code = process.wait(timeout=30.0)
    except KeyboardInterrupt:
        intentional_stop = True
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        return_code = process.wait(timeout=30.0)
    metadata_present = (bag_path / "metadata.yaml").is_file()
    accepted = {0, -int(signal.SIGINT), 128 + int(signal.SIGINT)}
    complete = return_code in accepted and metadata_present
    manifest.update(
        {
            "status": "complete" if complete else "failed",
            "finished_utc": _utc_now(),
            "rosbag_exit_code": int(return_code),
            "intentional_stop": intentional_stop,
            "bag_metadata_present": metadata_present,
        }
    )
    sidecar.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    if bag_path.is_dir():
        shutil.copy2(sidecar, bag_path / "recording_manifest.yaml")
    if not complete:
        raise RuntimeError(
            f"ros2 bag record did not finalize: exit={return_code}, metadata={metadata_present}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
