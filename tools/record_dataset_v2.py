#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from typing import Mapping, Sequence

import yaml

from aic_transfuser_lite.data.topic_contract_v2 import RECORDING_TOPICS


# Public name retained for the recorder's complete topic contract.
DATASET_V2_TOPICS = RECORDING_TOPICS


def build_record_command(output_path: str | Path) -> list[str]:
    """Build the exact rosbag2 MCAP command used by Dataset v2 recording."""

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
        str(output_path),
        *(contract.name for contract in DATASET_V2_TOPICS),
    ]


def validate_topic_types(discovered: Mapping[str, str]) -> None:
    """Fail before recording if any stream is absent or has a different type."""

    for contract in DATASET_V2_TOPICS:
        if contract.name not in discovered:
            raise ValueError(f"Missing required topic: {contract.name}")
        actual = discovered[contract.name]
        if actual != contract.message_type:
            raise ValueError(
                f"Topic type mismatch for {contract.name}: "
                f"expected={contract.message_type}, actual={actual}"
            )


def parse_topic_list_with_types(output: str) -> dict[str, str]:
    """Parse ``ros2 topic list -t`` without weakening exact type checks."""

    discovered: dict[str, str] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line.startswith("/") or " [" not in line or not line.endswith("]"):
            continue
        name, type_block = line.rsplit(" [", 1)
        message_type = type_block[:-1].strip()
        if not name or not message_type:
            continue
        previous = discovered.get(name)
        if previous is not None and previous != message_type:
            raise ValueError(
                f"Conflicting topic types in one graph snapshot for {name}: "
                f"{previous!r} vs {message_type!r}"
            )
        discovered[name] = message_type
    return discovered


def discover_topic_types(
    *,
    env: Mapping[str, str] | None = None,
    attempts: int = 3,
    retry_delay_sec: float = 0.5,
    spin_time_sec: float = 2.0,
) -> dict[str, str]:
    """Discover one coherent graph snapshot, retrying DDS participant startup."""

    if attempts <= 0:
        raise ValueError("attempts must be positive")
    if retry_delay_sec < 0.0:
        raise ValueError("retry_delay_sec must be non-negative")
    if spin_time_sec <= 0.0:
        raise ValueError("spin_time_sec must be positive")
    required_names = {contract.name for contract in DATASET_V2_TOPICS}
    discovered: dict[str, str] = {}
    for attempt in range(attempts):
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "list",
                    "--no-daemon",
                    "--spin-time",
                    str(spin_time_sec),
                    "-t",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
                env=dict(env) if env is not None else None,
            )
        except subprocess.TimeoutExpired:
            discovered = {}
        else:
            discovered = (
                parse_topic_list_with_types(result.stdout)
                if result.returncode == 0
                else {}
            )
        if required_names.issubset(discovered):
            break
        if attempt + 1 < attempts:
            time.sleep(retry_delay_sec)
    return discovered


def _safe_id(value: str, name: str) -> str:
    if not value or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for character in value):
        raise ValueError(f"{name} must use only letters, digits, '-' and '_'")
    return value


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_manifest(path: Path, payload: Mapping[str, object]) -> None:
    path.write_text(
        yaml.safe_dump(dict(payload), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record one versioned Dataset v2 AWSIM run on vehicle ROS domain 101."
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--ros-domain-id", type=int, default=101)
    parser.add_argument("--duration-sec", type=float)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    run_id = _safe_id(args.run_id, "run-id")
    scenario_id = _safe_id(args.scenario_id, "scenario-id")
    if args.ros_domain_id < 0 or args.ros_domain_id > 232:
        raise ValueError("ros-domain-id must be in [0, 232]")
    if args.duration_sec is not None and args.duration_sec <= 0.0:
        raise ValueError("duration-sec must be positive")
    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    bag_path = output_root / run_id
    if bag_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing recording: {bag_path}")
    sidecar = output_root / f"{run_id}.recording_manifest.yaml"
    if sidecar.exists():
        raise FileExistsError(f"Refusing to overwrite existing manifest: {sidecar}")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    discovered = discover_topic_types(env=environment)
    validate_topic_types(discovered)
    command = build_record_command(bag_path)
    manifest: dict[str, object] = {
        "format_version": 2,
        "status": "recording",
        "run_id": run_id,
        "scenario_id": scenario_id,
        "ros_domain_id": args.ros_domain_id,
        "started_utc": _utc_now(),
        "finished_utc": None,
        "command": command,
        "topics": [
            {
                "role": contract.role,
                "name": contract.name,
                "expected_type": contract.message_type,
                "discovered_type": discovered[contract.name],
            }
            for contract in DATASET_V2_TOPICS
        ],
        "actual_steering_is_command_fallback": False,
    }
    _write_manifest(sidecar, manifest)
    process = subprocess.Popen(command, env=environment)
    intentional_stop = False
    try:
        if args.duration_sec is None:
            return_code = process.wait()
        else:
            try:
                return_code = process.wait(timeout=args.duration_sec)
            except subprocess.TimeoutExpired:
                intentional_stop = True
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                return_code = process.wait(timeout=30.0)
    except KeyboardInterrupt:
        intentional_stop = True
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        return_code = process.wait(timeout=30.0)
    manifest["finished_utc"] = _utc_now()
    manifest["rosbag_exit_code"] = int(return_code)
    accepted_return_codes = {0, -int(signal.SIGINT), 128 + int(signal.SIGINT)}
    bag_complete = (bag_path / "metadata.yaml").is_file()
    completed = return_code in accepted_return_codes and bag_complete
    manifest["intentional_stop"] = intentional_stop
    manifest["bag_metadata_present"] = bag_complete
    manifest["status"] = "complete" if completed else "failed"
    _write_manifest(sidecar, manifest)
    if bag_path.is_dir():
        shutil.copy2(sidecar, bag_path / "recording_manifest.yaml")
    if not completed:
        raise RuntimeError(
            f"ros2 bag record did not finalize: exit={return_code}, metadata={bag_complete}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
