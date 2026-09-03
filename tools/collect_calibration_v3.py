#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
from typing import Mapping, Sequence

from aic_transfuser_lite.data.calibration.excitation import (
    excitation_plan_sha256,
    load_excitation_plan,
)
from aic_transfuser_lite.data.topic_profile_v3 import TopicProfileV3, load_topic_profile_v3


CAPTURE_ROLES = (
    "camera",
    "lidar",
    "pose",
    "velocity",
    "actual_steering",
    "gear",
    "nominal_command",
    "final_command",
    "clock",
)
PREFLIGHT_INPUT_ROLES = (
    "camera",
    "lidar",
    "pose",
    "velocity",
    "actual_steering",
    "gear",
    "clock",
)
_SAFE_ID = re.compile(r"^[A-Za-z0-9_-]+$")


def build_record_command(
    output_path: str | Path, profile: TopicProfileV3
) -> list[str]:
    topics = _capture_topics(profile)
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
        *topics,
    ]


def build_launch_command(
    *, plan_path: str | Path, plan_sha256: str, result_path: str | Path
) -> list[str]:
    return [
        "ros2",
        "launch",
        "aic_e2e_runtime",
        "calibration_capture_v3.launch.py",
        f"plan_path:={plan_path}",
        f"arm_token:={plan_sha256}",
        f"result_path:={result_path}",
        "use_sim_time:=true",
    ]


def parse_topic_list_with_types(output: str) -> dict[str, str]:
    discovered: dict[str, str] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line.startswith("/") or " [" not in line or not line.endswith("]"):
            continue
        name, type_block = line.rsplit(" [", 1)
        message_type = type_block[:-1].strip()
        if name and message_type:
            previous = discovered.setdefault(name, message_type)
            if previous != message_type:
                raise ValueError(f"conflicting topic types for {name}")
    return discovered


def validate_preflight_topics(
    profile: TopicProfileV3, discovered: Mapping[str, str]
) -> None:
    for role in PREFLIGHT_INPUT_ROLES:
        spec = _required_role(profile, role)
        actual = discovered.get(spec.name)
        if actual is None:
            raise ValueError(f"missing calibration input topic: {spec.name}")
        if actual != spec.message_type:
            raise ValueError(
                f"topic type mismatch for {spec.name}: expected={spec.message_type}, "
                f"actual={actual}"
            )


def parse_publisher_count(output: str) -> int:
    match = re.search(r"^Publisher count:\s*(\d+)\s*$", output, re.MULTILINE)
    if match is None:
        raise ValueError("ros2 topic info output lacks Publisher count")
    return int(match.group(1))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record one fail-closed AWSIM V3 actuator-calibration excitation run. "
            "Dry-run is the default; --execute is required to publish commands."
        )
    )
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument(
        "--topic-profile",
        type=Path,
        default=Path("configs/data/topic_profile_v3.yaml"),
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--ros-domain-id", type=int, default=101)
    parser.add_argument(
        "--source-git-revision",
        help="Required 40-hex source revision when running from an archive without .git",
    )
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    run_id = _safe_id(args.run_id, "run-id")
    scenario_id = _safe_id(args.scenario_id, "scenario-id")
    if not 0 <= args.ros_domain_id <= 232:
        raise ValueError("ros-domain-id must be in [0, 232]")
    plan_path = args.plan.expanduser().resolve()
    profile_path = args.topic_profile.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    plan = load_excitation_plan(plan_path)
    profile = load_topic_profile_v3(profile_path)
    plan_sha = excitation_plan_sha256(plan)
    _capture_topics(profile)
    repository_root = Path(__file__).resolve().parents[1]
    git_revision, git_dirty = _source_state(
        repository_root, explicit_revision=args.source_git_revision
    )

    bag_path = output_root / run_id
    manifest_path = output_root / f"{run_id}.calibration_capture.json"
    result_path = output_root / f"{run_id}.excitation_result.json"
    for path in (bag_path, manifest_path, result_path):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite calibration output: {path}")
    record_command = build_record_command(bag_path, profile)
    launch_command = build_launch_command(
        plan_path=plan_path, plan_sha256=plan_sha, result_path=result_path
    )
    preview = {
        "format_version": "aic_calibration_capture_preview_v1",
        "execute": bool(args.execute),
        "run_id": run_id,
        "scenario_id": scenario_id,
        "ros_domain_id": args.ros_domain_id,
        "plan_id": plan.plan_id,
        "target_mode": plan.target_mode,
        "plan_sha256": plan_sha,
        "plan_duration_sec": plan.total_duration_sec,
        "source_git_revision": git_revision,
        "source_git_dirty": git_dirty,
        "bag_path": str(bag_path),
        "record_command": record_command,
        "launch_command": launch_command,
    }
    print(json.dumps(preview, indent=2))
    if not args.execute:
        return 0
    if git_dirty:
        raise RuntimeError("refusing calibration capture from a dirty tracked worktree")

    output_root.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    discovered = _discover_topic_types(environment)
    validate_preflight_topics(profile, discovered)
    for role in ("nominal_command", "final_command"):
        topic = _required_role(profile, role).name
        publishers = _topic_publisher_count(topic, environment)
        if publishers != 0:
            raise RuntimeError(
                f"refusing calibration capture: {topic} already has {publishers} publisher(s)"
            )

    manifest: dict[str, object] = {
        **preview,
        "format_version": "aic_calibration_capture_manifest_v1",
        "status": "recording",
        "started_utc": _utc_now(),
        "finished_utc": None,
        "input_topics": {
            role: _required_role(profile, role).name for role in PREFLIGHT_INPUT_ROLES
        },
    }
    _write_json(manifest_path, manifest)
    recorder: subprocess.Popen[str] | None = None
    launch: subprocess.Popen[str] | None = None
    error: BaseException | None = None
    try:
        recorder = subprocess.Popen(record_command, env=environment, text=True)
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise RuntimeError(f"rosbag recorder exited early: {recorder.returncode}")
        launch = subprocess.Popen(launch_command, env=environment, text=True)
        timeout = plan.total_duration_sec + plan.preflight_hold_sec + 50.0
        try:
            launch_return_code = launch.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("calibration launch exceeded bounded timeout") from exc
        if launch_return_code != 0:
            raise RuntimeError(f"calibration launch failed: exit={launch_return_code}")
    except BaseException as exc:
        error = exc
    finally:
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                launch.terminate()
                launch.wait(timeout=5.0)
        recorder_return_code = None
        if recorder is not None:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            try:
                recorder_return_code = recorder.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                recorder.terminate()
                recorder_return_code = recorder.wait(timeout=5.0)

    result: dict[str, object] | None = None
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding="utf-8"))
    metadata_present = (bag_path / "metadata.yaml").is_file()
    accepted_recorder_codes = {0, -int(signal.SIGINT), 128 + int(signal.SIGINT)}
    complete = (
        error is None
        and recorder_return_code in accepted_recorder_codes
        and metadata_present
        and result is not None
        and result.get("status") == "complete"
        and result.get("plan_sha256") == plan_sha
    )
    manifest.update(
        {
            "status": "complete" if complete else "failed",
            "finished_utc": _utc_now(),
            "rosbag_exit_code": recorder_return_code,
            "bag_metadata_present": metadata_present,
            "excitation_result": result,
            "error": None if error is None else f"{type(error).__name__}: {error}",
        }
    )
    _write_json(manifest_path, manifest)
    if bag_path.is_dir():
        shutil.copy2(manifest_path, bag_path / "calibration_capture_manifest.json")
        if result_path.is_file():
            shutil.copy2(result_path, bag_path / "excitation_result.json")
    if error is not None:
        raise error
    if not complete:
        raise RuntimeError("calibration capture did not satisfy completion gates")
    return 0


def _capture_topics(profile: TopicProfileV3) -> list[str]:
    topics = [_required_role(profile, role).name for role in CAPTURE_ROLES]
    if len(set(topics)) != len(topics):
        raise ValueError("calibration capture topics must be unique")
    return topics


def _required_role(profile: TopicProfileV3, role: str):
    spec = profile.roles.get(role)
    if spec is None:
        raise ValueError(f"topic profile lacks calibration role: {role}")
    return spec


def _discover_topic_types(environment: Mapping[str, str]) -> dict[str, str]:
    result = subprocess.run(
        ["ros2", "topic", "list", "--no-daemon", "--spin-time", "2.0", "-t"],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
        env=dict(environment),
    )
    if result.returncode != 0:
        raise RuntimeError(f"ROS graph discovery failed: {result.stderr.strip()}")
    return parse_topic_list_with_types(result.stdout)


def _topic_publisher_count(topic: str, environment: Mapping[str, str]) -> int:
    result = subprocess.run(
        ["ros2", "topic", "info", "--verbose", topic],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
        env=dict(environment),
    )
    if result.returncode != 0:
        # A topic absent from the graph has zero publishers before our launch.
        detail = result.stdout + "\n" + result.stderr
        if "Unknown topic" in detail or "does not exist" in detail:
            return 0
        raise RuntimeError(f"publisher discovery failed for {topic}: {result.stderr.strip()}")
    return parse_publisher_count(result.stdout)


def _safe_id(value: str, name: str) -> str:
    if not _SAFE_ID.fullmatch(value):
        raise ValueError(f"{name} must use letters, digits, '-' or '_'")
    return value


def _git_state(repository_root: Path) -> tuple[str, bool]:
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
        timeout=10.0,
    ).stdout.strip()
    if not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise RuntimeError("git rev-parse returned an invalid revision")
    status = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
        timeout=10.0,
    ).stdout
    return revision, bool(status.strip())


def _source_state(
    repository_root: Path, *, explicit_revision: str | None
) -> tuple[str, bool]:
    try:
        revision, dirty = _git_state(repository_root)
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        if explicit_revision is None:
            raise RuntimeError(
                "source Git revision is unavailable; pass --source-git-revision for "
                "an immutable archive"
            ) from error
        if not re.fullmatch(r"[0-9a-f]{40}", explicit_revision):
            raise ValueError("source-git-revision must be lowercase 40-hex")
        return explicit_revision, False
    if explicit_revision is not None and explicit_revision != revision:
        raise ValueError("source-git-revision does not match checkout HEAD")
    return revision, dirty


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.write_text(json.dumps(dict(payload), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
