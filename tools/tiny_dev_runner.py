"""Finite Lite make-dev Tiny trial, unchanged owned AWSIM, cumulative budgets."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from aic_transfuser_lite.runtime.tiny_lidar_sim import WEIGHT_SHA256, digest
from spatial_dev_host_v4 import HostWatch, AttemptBudget, atomic_json, cleanup_owned

IMAGE = "sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7"
ASSETS = {
    "AWSIM.x86_64": "0bfe51325720c950b4ad75ce7c3b65595fd7e939908520ec33600919919327a8",
    "AWSIM_Data/Managed/Assembly-CSharp.dll": "859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13",
    "AWSIM_Data/StreamingAssets/Vehicle/vehicle.yaml": "5b66e58691091c82d5511535ca458d4c89e85f3b59fa0d0c4a2c7eee47887244",
    "AWSIM_Data/level1": "9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b",
    "AWSIM_Data/sharedassets1.assets": "932d21250cfe685c9599acf33cca2d00d17e4141676fec236b813a069d9df34c",
    "AWSIM_Data/globalgamemanagers.assets": "9b19aa2e22e004049272dd9c9ee2a58bddef4c8f055c44540023a3de98d9cfbd",
    "AWSIM_Data/resources.assets": "1bbe49232479849779814fb4d1259d179d6466dc7a95f21f99382e168c5ad84f",
}
ORIGINAL_SCRIPT = "0a8c3442bbe5f31a267926845e18039438ad8a1b1c35654225c5b9d819f4d40e"


class TinyBudget(AttemptBudget):
    """Preserve legacy V4 count; separate Tiny counter under same total ceiling."""
    def reserve_tiny(self, attempt: str, reservation: dict, tiny_limit: int) -> None:
        previous = self.value["used"].get("tiny_forward", 0)
        if previous is None or self.value["used"]["forward"]+previous+tiny_limit > self.limits["forward"]:
            raise ValueError("COMMON_FORWARD_LIMIT_INCLUDING_TINY")
        self.reserve(attempt, reservation)
        self.value["active"]["tiny_forward_reserved"] = tiny_limit
        self.value["used"]["tiny_forward"] = previous
        atomic_json(self.path, self.value)

    def finish_tiny(self, consumption: dict, tiny_calls: int | None, *, exact: bool) -> None:
        active = self.value["active"]
        charged = active["tiny_forward_reserved"] if tiny_calls is None else tiny_calls
        active["tiny_forward_charged"] = charged
        active["tiny_forward_exact"] = tiny_calls is not None
        self.value["used"]["tiny_forward"] += charged
        self.finish(consumption, exact=exact)


class JudgeLog:
    """Actual unchanged LapCount logs, never distance/proximity as a lap."""
    def __init__(self):
        self.section_events = []
        self.laps = []
        self.invalid = False

    def feed(self, line: str) -> None:
        hit = re.search(r"Section line hit: current=(-?\d+), next=(\d+), started=(\w+)", line)
        if hit:
            previous, next_section = int(hit[1]), int(hit[2])
            if self.section_events:
                prior = self.section_events[-1]["next"]
                if previous != prior or (next_section != prior+1 and next_section != 0):
                    self.invalid = True
            self.section_events.append(dict(current=previous, next=next_section, started=hit[3], line=line))
        lap = re.search(r"Lap completed: ([\d.]+)s, total laps: (\d+)", line)
        if lap:
            self.laps.append(dict(lap_seconds=float(lap[1]), laps=int(lap[2]), line=line,
                ordered_section_evidence=not self.invalid and len(self.section_events) >= 4
                    and self.section_events[0]["next"] == 0 and self.section_events[-1]["next"] == 0))

    @property
    def completed(self) -> bool:
        return bool(self.laps and self.laps[-1]["laps"] >= 1 and self.laps[-1]["ordered_section_evidence"])


def compose_definition(source: Path, sim: Path, xvfb: Path, output: Path, official: Path,
                       project: str, commit: str) -> dict:
    common = dict(image=IMAGE, pull_policy="never", privileged=False, network_mode="none",
        cap_drop=["ALL"], security_opt=["no-new-privileges:true"], ipc="private", pids_limit=512,
        mem_limit="6g", cpus=4, shm_size="256m", user=f"{os.getuid()}:{os.getgid()}",
        entrypoint=["/bin/bash"], restart="no", gpus="all", stop_grace_period="12s", working_dir="/evidence")
    env = dict(ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        CYCLONEDDS_URI="file:///v4/integrations/awsim_dev_v4/cyclonedds.xml", TINY_SOURCE_COMMIT=commit,
        TINY_PROJECT=project, ROS_LOG_DIR="/evidence/ros_logs")
    def volume(path: Path, target: str, ro: bool = True) -> dict:
        return dict(type="bind", source=str(path.resolve()), target=target, read_only=ro)
    shared = [volume(source, "/v4"), volume(output, "/evidence", False)]
    simulator = dict(common, command=["/v4/integrations/awsim_dev_v4/simulator.sh"],
        environment=dict(env, ROS_DOMAIN_ID="0", DISPLAY=":99", __NV_PRIME_RENDER_OFFLOAD="1",
            AWSIM_START_MODE="off", AWSIM_VEHICLES="1", AWSIM_LAPS="600", AWSIM_TIMEOUT="60000000",
            AWSIM_EXTRA_ARGS="--manual-mode true --npcs 0 --boosts 0 --collisions on --wall-recovery off --start-random off --camera gpu --lidar gpu --sound off --target-fps 20 -screen-width 640 -screen-height 360 -logFile /evidence/awsim_unity.log"),
        volumes=shared+[volume(sim/"aichallenge/simulator/AWSIM", "/aichallenge/simulator/AWSIM"),
            volume(sim/"aichallenge/run_simulator.bash", "/aichallenge/run_simulator.bash"),
            volume(xvfb, "/xvfb"), volume(xvfb/"usr/bin/xkbcomp", "/usr/bin/xkbcomp"),
            volume(output/"x11", "/tmp/.X11-unix", False)])
    runtime = dict(common, network_mode="service:simulator", depends_on=["simulator"],
        command=["/v4/integrations/tiny_dev/runtime.sh"], environment=dict(env, ROS_DOMAIN_ID="1"),
        volumes=shared+[volume(official, "/official_tiny")])
    return dict(name=project, services=dict(simulator=simulator, tiny=runtime))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim-repo", type=Path, required=True)
    ap.add_argument("--official-package", type=Path, required=True)
    ap.add_argument("--xvfb-root", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--commit", required=True)
    ap.add_argument("--phase", choices=("stationary", "short", "lap"), default="stationary")
    ap.add_argument("--wall-seconds", type=int, default=120)
    ap.add_argument("--budget", type=Path, required=True)
    args = ap.parse_args()
    if os.name != "posix" or not re.fullmatch("[0-9a-f]{40}", args.commit):
        raise ValueError("LINUX_FIXED_COMMIT_REQUIRED")
    if not 10 <= args.wall_seconds <= 240:
        raise ValueError("FINITE_WALL_REQUIRED")
    source, sim = Path(__file__).resolve().parents[1], args.sim_repo.resolve()
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    if digest(args.official_package/"ckpt/tinylidarnet_weights.npy") != WEIGHT_SHA256:
        raise ValueError("OFFICIAL_WEIGHTS_MISMATCH")
    assets = {sim/"aichallenge/simulator/AWSIM"/key: sha for key, sha in ASSETS.items()}
    assets[sim/"aichallenge/run_simulator.bash"] = ORIGINAL_SCRIPT
    before = {str(p): digest(p) for p in assets}
    if any(before[str(p)] != value for p, value in assets.items()):
        raise ValueError("UNCHANGED_AWSIM_IDENTITY_FAILED")
    output.mkdir(parents=True)
    (output/"x11").mkdir(); (output/"x11").chmod(0o1777)
    project = "codex-tiny-dev-"+output.name.lower().replace("_", "-")
    if not re.fullmatch("[a-z0-9-]+", project):
        raise ValueError("PROJECT_NAME")
    cfg = yaml.safe_load((source/"configs/control/tiny_lidar_sim.yaml").read_text())
    cfg.update(phase=args.phase, wall_seconds=args.wall_seconds,
               forward_limit={"stationary": 12, "short": 160, "lap": 650}[args.phase])
    atomic_json(output/"resolved_config.json", cfg)
    spec = compose_definition(source, sim, args.xvfb_root, output, args.official_package, project, args.commit)
    atomic_json(output/"compose.json", spec)
    command = ["docker", "compose", "-p", project, "-f", str(output/"compose.json")]
    log = (output/"host.jsonl").open("x", buffering=1)
    def run(argv: list[str], *, timeout: float = 30, check: bool = True):
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        log.write(json.dumps(dict(command=argv, returncode=proc.returncode, stdout=proc.stdout, stderr=proc.stderr))+"\n")
        if check and proc.returncode:
            raise RuntimeError(proc.stderr[:2000])
        return proc
    if run(["docker", "ps", "-q"]).stdout.strip():
        raise ValueError("OTHER_RUNNING_CONTAINERS_REQUIRES_RECHECK")
    if run(["pgrep", "-x", "AWSIM.x86_64"], check=False).returncode == 0:
        raise ValueError("EXISTING_HOST_SIMULATOR_REQUIRES_RECHECK")
    if run(command+["ps", "-aq"]).stdout.strip():
        raise ValueError("PROJECT_EXISTS")
    (output/"compose_resolved.json").write_text(run(command+["config", "--format", "json"]).stdout)
    budget = TinyBudget(args.budget)
    pause_proof = json.loads((args.budget.parent/"binding.json").read_text())["host_paused_kill"]
    if (pause_proof.get("verified") is not True or pause_proof.get("probe_sha256") !=
            "f7d1f227cafe340ca702aa3e974deb485db935e6c374313f39c89a2d778a3f32"):
        raise ValueError("PRIOR_SELECTED_HOST_PAUSED_KILL_PROOF_MISSING")
    atomic_json(output/"reused_host_pause_proof.json", pause_proof)
    # No V4 forward/MPC reservation. A distinct Tiny counter shares, but does
    # not reset or rewrite, the inherited 3000-forward aggregate upper bound.
    budget.reserve_tiny(output.name, dict(wall_s=args.wall_seconds+110., forward=0, mpc=0, snapshots=0,
        powered=int(args.phase != "stationary"), powered_s=60. if args.phase != "stationary" else 0.,
        log_bytes=96*1024**2), cfg["forward_limit"])
    atomic_json(output/"budget_before.json", budget.value)
    watch = HostWatch(project)
    judge = JudgeLog()
    judge_offset = 0
    judge_pending = ""
    sim_id = runtime_id = None
    started = time.monotonic()
    error = None
    exitcode = None
    freeze_started = None
    pause_verified = False
    try:
        run(command+["up", "-d", "--no-build", "--pull", "never", "simulator", "tiny"])
        sim_id = run(command+["ps", "-q", "simulator"]).stdout.strip()
        runtime_id = run(command+["ps", "-q", "tiny"]).stdout.strip()
        if not all(re.fullmatch("[0-9a-f]{64}", value) for value in (sim_id, runtime_id)):
            raise ValueError("OWNED_CONTAINER_IDENTITY")
        atomic_json(output/"instance_inspect.json", json.loads(run(["docker", "inspect", sim_id, runtime_id]).stdout))
        while time.monotonic()-started < args.wall_seconds+60:
            if sum(p.stat().st_size for p in output.rglob("*") if p.is_file()) >= 90*1024**2:
                raise RuntimeError("LOG_RESERVATION_LIMIT")
            state = json.loads(run(["docker", "inspect", runtime_id, "--format", "{{json .State}}"]).stdout)
            if not state["Running"]:
                exitcode = state["ExitCode"]
                break
            if freeze_started is not None:
                if time.monotonic()-freeze_started > 5.:
                    raise RuntimeError("FINALIZE_TIMEOUT_WHILE_PAUSED")
                time.sleep(.1)
                continue
            heartbeat = output/"heartbeat.json"
            fault = watch.check(json.loads(heartbeat.read_text()) if heartbeat.exists() else None, time.monotonic_ns())
            if fault or watch.freeze_requested:
                run(["docker", "pause", sim_id])
                state = json.loads(run(["docker", "inspect", sim_id, "--format", "{{json .State}}"]).stdout)
                pause_verified = state["Paused"] is True
                if not pause_verified:
                    raise RuntimeError("HOST_PAUSE_NOT_VERIFIED")
                atomic_json(output/"finalization_freeze.json", dict(token=project, paused=True,
                    monotonic_ns=time.monotonic_ns(), sim_id=sim_id, reason=fault or "SUPERVISOR_FINISHED"))
                if fault:
                    error = fault
                    break
                freeze_started = time.monotonic()
                continue
            if watch.armed:
                atomic_json(output/"host_armed.json", dict(token=project, armed=True, monotonic_ns=time.monotonic_ns(),
                    sim_id=sim_id, runtime_id=runtime_id, paused_kill_verified=True,
                    paused_kill_evidence_sha256="f7d1f227cafe340ca702aa3e974deb485db935e6c374313f39c89a2d778a3f32"))
            unity = output/"awsim_unity.log"
            if unity.exists():
                with unity.open(errors="replace") as stream:
                    stream.seek(judge_offset)
                    chunk = stream.read(256*1024)
                    judge_offset = stream.tell()
                lines = (judge_pending+chunk).split("\n")
                judge_pending = lines.pop()
                for line in lines:
                    judge.feed(line)
                if judge.laps and not (output/"stop_request.json").exists():
                    atomic_json(output/"stop_request.json", dict(token=project,
                        reason="JUDGE_FIRST_LAP" if judge.completed else "JUDGE_LAP_EVIDENCE_INCOMPLETE"))
            # 09:50 JST cutoff leaves 10 minutes to stop and preserve evidence.
            if time.time() >= 1788742200:
                atomic_json(output/"stop_request.json", dict(token=project, reason="DEADLINE_SAVE_CUTOFF"))
            time.sleep(.15)
        else:
            error = "HOST_FINITE_TIMEOUT"
    except BaseException as exc:
        error = type(exc).__name__+": "+str(exc)
    finally:
        def cleanup_run(argv, timeout=15):
            proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
            try:
                log.write(json.dumps(dict(cleanup_command=argv, returncode=proc.returncode,
                    stdout=proc.stdout, stderr=proc.stderr))+"\n")
            except Exception:
                pass
            if proc.returncode:
                raise RuntimeError(proc.stderr)
            return proc
        cleanup = cleanup_owned(cleanup_run, sim_id, runtime_id, paused_kill_verified=True)
        after = {str(p): digest(p) for p in assets}
        summary = dict(source_commit=args.commit, phase=args.phase, error=error, runtime_exitcode=exitcode,
            project=project, simulator_container_id=sim_id, runtime_container_id=runtime_id,
            simulator_assets_before=before, simulator_assets_after=after, simulator_assets_unchanged=before == after,
            wall_s=time.monotonic()-started, host_armed=watch.armed, host_pause_verified=pause_verified,
            cleanup=cleanup, judge_section_events=judge.section_events, judge_laps=judge.laps,
            judge_order_invalid=judge.invalid, judge_lap_confirmed=judge.completed,
            control_claim_requires_supervisor_log=True, normal_racingkart_makefile_executed=False,
            awsimmutation=False, original_start_command="bash /aichallenge/run_simulator.bash dev")
        atomic_json(output/"host_summary.json", summary)
        consumption = dict(wall_s=summary["wall_s"], forward=0, mpc=0, snapshots=0,
            log_bytes=sum(p.stat().st_size for p in output.rglob("*") if p.is_file())+1024**2)
        worker_file, supervisor_file = output/"tiny_worker_summary.json", output/"tiny_supervisor_summary.json"
        tiny_calls = json.loads(worker_file.read_text())["tiny_forward_calls"] if worker_file.exists() else None
        if supervisor_file.exists():
            final = json.loads(supervisor_file.read_text())
            consumption.update(powered=final["powered_episode_count"], powered_s=final["powered_sim_seconds"])
        budget.finish_tiny(consumption, tiny_calls, exact=tiny_calls is not None and len(consumption) == 7)
        atomic_json(output/"budget_after.json", budget.value)
        budget.close()
        log.close()
    print(json.dumps(summary))
    return int(error is not None or exitcode != 0 or bool(cleanup["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
