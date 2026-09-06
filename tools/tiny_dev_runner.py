"""Finite Lite make-dev Tiny trial, unchanged owned AWSIM, cumulative budgets."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from aic_transfuser_lite.runtime.tiny_lidar_sim import (
    WEIGHT_SHA256, digest, TINY_AUTH_PROFILE, AUTH_REQUEST_SHA256, DRIVE_CUTOFF_UNIX_S,
    finite_number, tiny_phase_caps, validate_tiny_config, bounded_runtime_wall,
    TINY_GUI_PROFILE, GUI_AUTH_SHA256, GUI_CONTROL_METHOD,
    TINY_GUI_RETRY_PROFILE, TINY_GUI_RETRY2_PROFILE, TINY_GUI_LAP_PROFILE, GUI_AUTHORIZATIONS,
)
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
    def __init__(self, path: Path, *, profile: str):
        if profile not in (TINY_AUTH_PROFILE, *GUI_AUTHORIZATIONS):
            raise ValueError("EXPLICIT_TINY_BUDGET_PROFILE_REQUIRED")
        super().__init__(path)
        # Instance copy, never mutate AttemptBudget.limits or another profile.
        self.limits = dict(AttemptBudget.limits, forward=6000, powered_s=300.)
        if profile == TINY_GUI_PROFILE:
            self.limits["powered"] = 4
        elif profile == TINY_GUI_RETRY_PROFILE:
            self.limits["powered"] = 5
        elif profile == TINY_GUI_RETRY2_PROFILE:
            self.limits["powered"] = 6
        elif profile == TINY_GUI_LAP_PROFILE:
            self.limits.update(powered=7, forward=7100, powered_s=320.)
        self.profile = profile

    def authorize(self) -> dict:
        if self.value.get("active"):
            raise ValueError("UNRESOLVED_PRIOR_RESERVATION")
        for key in (*self.limits, "tiny_forward"):
            finite_number(self.value["used"].get(key), "used."+key,
                          integer=key not in ("wall_s", "powered_s"))
        records = self.value.setdefault("authorization_changes", [])
        gui = self.profile in GUI_AUTHORIZATIONS
        request_hash = GUI_AUTHORIZATIONS[self.profile][0] if gui else AUTH_REQUEST_SHA256
        prior = next((r for r in records if r.get("profile") == self.profile), None)
        if prior is not None:
            if prior.get("request_sha256") != request_hash or prior.get("new_limits") != self.limits:
                raise ValueError("CONFLICTING_TINY_AUTHORIZATION_RECORD")
            return prior
        if self.profile in (TINY_GUI_RETRY_PROFILE, TINY_GUI_RETRY2_PROFILE):
            previous_profile = TINY_GUI_RETRY_PROFILE if self.profile == TINY_GUI_RETRY2_PROFILE else TINY_GUI_PROFILE
            expected_previous = dict(self.limits, powered=self.limits["powered"]-1)
            if (self.value.get("tiny_authorized_profile") != previous_profile or
                    self.value.get("tiny_authorized_limits") != expected_previous):
                raise ValueError("GUI_RETRY_REQUIRES_PREVIOUS_GUI_AUTHORIZATION")
        if self.profile == TINY_GUI_LAP_PROFILE:
            expected_previous = dict(self.limits, powered=6, forward=6000, powered_s=300.)
            if (self.value.get("tiny_authorized_profile") != TINY_GUI_RETRY2_PROFILE or
                    self.value.get("tiny_authorized_limits") != expected_previous):
                raise ValueError("GUI_LAP_REQUIRES_PREVIOUS_GUI_AUTHORIZATION")
        record = dict(profile=self.profile, request_sha256=request_hash,
            request_attachment=(GUI_AUTHORIZATIONS[self.profile][1] if gui else
                                "f7325d22-7b3c-494e-bf56-ca4464746cef/pasted-text.txt"),
            applied_unix_ns=time.time_ns(), applied_utc=datetime.now(timezone.utc).isoformat(),
            authorization_source="USER_SUPPLIED_EXECUTION_REQUEST", author_name=None,
            old_limits=dict(self.value.get("tiny_authorized_limits", AttemptBudget.limits)) if gui else dict(AttemptBudget.limits),
            new_limits=dict(self.limits),
            old_episode_sim_seconds=(20. if self.profile in (TINY_GUI_RETRY_PROFILE, TINY_GUI_RETRY2_PROFILE, TINY_GUI_LAP_PROFILE) else 240. if gui else 60.),
            new_episode_sim_seconds=20. if gui and self.profile != TINY_GUI_LAP_PROFILE else 240.,
            used_at_change=dict(self.value["used"]), prior_attempt_count=len(self.value.get("attempts", [])),
            prior_active=self.value.get("active"), applied_retroactively=False)
        records.append(record)
        self.value["tiny_authorized_profile"] = self.profile
        self.value["tiny_authorized_limits"] = dict(self.limits)
        atomic_json(self.path, self.value)
        return record

    def reserve_tiny(self, attempt: str, reservation: dict, tiny_limit: int) -> None:
        if self.value.get("tiny_authorized_profile") != self.profile:
            raise ValueError("TINY_AUTHORIZATION_NOT_RECORDED")
        finite_number(tiny_limit, "tiny_limit", integer=True)
        gui = self.profile in GUI_AUTHORIZATIONS
        if gui and any(a.get("authorization_profile") == self.profile for a in self.value.get("attempts", [])):
            raise ValueError("GUI_SINGLE_ATTEMPT_ALREADY_USED")
        short_gui = gui and self.profile != TINY_GUI_LAP_PROFILE
        if not 1 <= tiny_limit <= (600 if short_gui else 5200):
            raise ValueError("TINY_FORWARD_RESERVATION_CAP")
        for key in self.limits:
            finite_number(reservation[key], "reservation."+key, integer=key not in ("wall_s", "powered_s"))
            finite_number(self.value["used"][key], "used."+key, integer=key not in ("wall_s", "powered_s"))
        if reservation["forward"] != 0 or reservation["mpc"] != 0 or not 0 <= reservation["snapshots"] <= (2 if gui else 0):
            raise ValueError("TINY_ONLY_RESERVATION")
        if (reservation["powered"] not in (0, 1) or reservation["powered_s"] > (20 if short_gui else 240)
                or reservation["wall_s"] > (230 if short_gui else 710)):
            raise ValueError("TINY_ATTEMPT_RESERVATION_CAP")
        previous = finite_number(self.value["used"].get("tiny_forward"), "used.tiny_forward", integer=True)
        if self.value["used"]["forward"]+previous+tiny_limit > self.limits["forward"]:
            raise ValueError("COMMON_FORWARD_LIMIT_INCLUDING_TINY")
        self.reserve(attempt, reservation)
        self.value["active"]["tiny_forward_reserved"] = tiny_limit
        self.value["active"]["authorization_profile"] = self.profile
        self.value["used"]["tiny_forward"] = previous
        atomic_json(self.path, self.value)

    def finish_tiny(self, consumption: dict, tiny_calls: int | None, *, exact: bool) -> None:
        active = self.value["active"]
        for key, value in consumption.items():
            finite_number(value, "consumption."+key, integer=key not in ("wall_s", "powered_s"))
        if consumption.get("forward", 0) != 0 or consumption.get("mpc", 0) != 0:
            raise ValueError("TINY_MUST_NOT_CHARGE_V4_OR_MPC")
        if tiny_calls is not None:
            finite_number(tiny_calls, "tiny_calls", integer=True)
        charged = active["tiny_forward_reserved"] if tiny_calls is None else tiny_calls
        active["tiny_forward_charged"] = charged
        active["tiny_forward_exact"] = tiny_calls is not None
        self.value["used"]["tiny_forward"] += charged
        self.finish(consumption, exact=exact)


class JudgeLog:
    """Actual unchanged LapCount logs, never distance/proximity as a lap."""
    def __init__(self, run_id: str = "SYNTHETIC"):
        self.run_id = run_id
        self.section_events = []
        self.laps = []
        self.invalid = False
        self.previous_lap_count = 0

    def feed(self, line: str, byte_offset: int | None = None) -> None:
        hit = re.search(r"Section line hit: current=(-?\d+), next=(\d+), started=(\w+)", line)
        if hit:
            previous, next_section = int(hit[1]), int(hit[2])
            if self.section_events:
                prior = self.section_events[-1]["next"]
                if previous != prior or (next_section != prior+1 and next_section != 0) or hit[3] != "True":
                    self.invalid = True
            elif next_section != 0 or hit[3] != "False":
                self.invalid = True
            self.section_events.append(dict(current=previous, next=next_section, started=hit[3], line=line,
                run_id=self.run_id, epoch=0, byte_offset=byte_offset))
        lap = re.search(r"Lap completed: ([\d.]+)s, total laps: (\d+)", line)
        if lap:
            count = int(lap[2])
            increment_valid = count == self.previous_lap_count+1
            self.laps.append(dict(lap_seconds=float(lap[1]), laps=count, line=line,
                run_id=self.run_id, epoch=0, byte_offset=byte_offset, lap_increment_valid=increment_valid,
                ordered_section_evidence=not self.invalid and increment_valid and len(self.section_events) >= 4
                    and self.section_events[0]["next"] == 0 and self.section_events[-1]["next"] == 0))
            self.previous_lap_count = count

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
    ap.add_argument("--xvfb-root", type=Path)
    ap.add_argument("--control-method", choices=(GUI_CONTROL_METHOD,))
    retry_args = ap.add_mutually_exclusive_group()
    retry_args.add_argument("--gui-retry", action="store_true", help="Explicit one-shot 4-to-5 authorization; GUI short only")
    retry_args.add_argument("--gui-retry2", action="store_true", help="Explicit one-shot 5-to-6 authorization; GUI short only")
    retry_args.add_argument("--gui-lap", action="store_true", help="Explicit one-shot 6-to-7 authorization; GUI lap only")
    ap.add_argument("--install-root", type=Path)
    ap.add_argument("--display")
    ap.add_argument("--xauthority", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--commit", required=True)
    ap.add_argument("--phase", choices=("stationary", "short", "lap"), default="stationary")
    ap.add_argument("--wall-seconds", type=int, default=120)
    ap.add_argument("--budget", type=Path, required=True)
    args = ap.parse_args()
    gui = args.control_method == GUI_CONTROL_METHOD
    if (args.gui_retry or args.gui_retry2 or args.gui_lap) and not gui:
        raise ValueError("GUI_RETRY_REQUIRES_GUARDED_METHOD")
    profile = (TINY_GUI_RETRY_PROFILE if args.gui_retry else TINY_GUI_PROFILE) if gui else TINY_AUTH_PROFILE
    if args.gui_retry2:
        profile = TINY_GUI_RETRY2_PROFILE
    if args.gui_lap:
        profile = TINY_GUI_LAP_PROFILE
    if os.name != "posix" or not re.fullmatch("[0-9a-f]{40}", args.commit):
        raise ValueError("LINUX_FIXED_COMMIT_REQUIRED")
    if not 10 <= args.wall_seconds <= tiny_phase_caps(args.phase, profile)["wall_seconds"]:
        raise ValueError("FINITE_WALL_REQUIRED")
    if time.time() >= DRIVE_CUTOFF_UNIX_S-120:
        raise ValueError("NO_STARTUP_CLEANUP_RESERVE_BEFORE_CUTOFF")
    source, sim = Path(__file__).resolve().parents[1], args.sim_repo.resolve()
    if gui:
        from tiny_gui_integration import compose_gui, make_command, window_candidates, visible_owned_windows
        if args.install_root is None or args.xauthority is None or args.display is None:
            raise ValueError("EXPLICIT_GUI_INSTALL_AND_DISPLAY_REQUIRED")
        if digest(source/GUI_AUTHORIZATIONS[profile][1]) != GUI_AUTHORIZATIONS[profile][0]:
            raise ValueError("GUI_AUTHORIZATION_BYTES_CHANGED")
    elif args.xvfb_root is None:
        raise ValueError("XVFB_ROOT_REQUIRED_FOR_LEGACY_PROFILE")
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
    if gui:
        (output/"rviz_config").mkdir()
    (output/"x11").mkdir(); (output/"x11").chmod(0o1777)
    project = "codex-tiny-dev-"+output.name.lower().replace("_", "-")
    if not re.fullmatch("[a-z0-9-]+", project):
        raise ValueError("PROJECT_NAME")
    cfg = yaml.safe_load((source/"configs/control/tiny_lidar_sim.yaml").read_text())
    cfg.update(tiny_phase_caps(args.phase, profile), phase=args.phase, wall_seconds=args.wall_seconds)
    if gui:
        cfg.update(authorization_profile=profile, authorization_request_sha256=GUI_AUTHORIZATIONS[profile][0],
                   control_method=GUI_CONTROL_METHOD, source_commit=args.commit)
    validate_tiny_config(cfg)
    spec = (compose_gui(source, sim, args.install_root, output, args.official_package, project, args.commit,
                        args.display, args.xauthority, IMAGE) if gui else
            compose_definition(source, sim, args.xvfb_root, output, args.official_package, project, args.commit))
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
    if gui:
        os.environ.update(DISPLAY=args.display, XAUTHORITY=str(args.xauthority))
        initial_tree = run(["xwininfo", "-root", "-tree"]).stdout
        atomic_json(output/"gui_before.json", dict(tree=initial_tree, candidates=window_candidates(initial_tree)))
        if any(window_candidates(initial_tree).values()):
            raise ValueError("OTHER_AWSIM_OR_RVIZ_WINDOWS_PRESENT")
    budget = TinyBudget(args.budget, profile=cfg["authorization_profile"])
    atomic_json(output/"budget_before_authorization.json", budget.value)
    authorization = budget.authorize()
    atomic_json(output/"budget_authorization_change.json", authorization)
    cfg["wall_seconds"] = bounded_runtime_wall(args.phase, args.wall_seconds,
        budget.limits["wall_s"]-budget.value["used"]["wall_s"], time.time(), profile)
    validate_tiny_config(cfg)
    atomic_json(output/"resolved_config.json", cfg)
    pause_proof = json.loads((args.budget.parent/"binding.json").read_text())["host_paused_kill"]
    if (pause_proof.get("verified") is not True or pause_proof.get("probe_sha256") !=
            "f7d1f227cafe340ca702aa3e974deb485db935e6c374313f39c89a2d778a3f32"):
        raise ValueError("PRIOR_SELECTED_HOST_PAUSED_KILL_PROOF_MISSING")
    atomic_json(output/"reused_host_pause_proof.json", pause_proof)
    # No V4 forward/MPC reservation. A distinct Tiny counter shares, but does
    # not reset or rewrite, the explicitly authorized shared forward bound.
    atomic_json(output/"budget_before_reservation.json", budget.value)
    budget.reserve_tiny(output.name, dict(wall_s=cfg["wall_seconds"]+110., forward=0, mpc=0, snapshots=2 if gui else 0,
        powered=int(args.phase != "stationary"), powered_s=cfg["single_episode_sim_limit_s"],
        log_bytes=96*1024**2), cfg["forward_limit"])
    atomic_json(output/"budget_before.json", budget.value)
    watch = HostWatch(project)
    judge = JudgeLog(project)
    judge_offset = 0
    judge_pending = b""
    sim_id = runtime_id = None
    started = time.monotonic()
    started_unix_ns = time.time_ns()
    error = None
    exitcode = None
    freeze_started = None
    pause_verified = False
    gui_windows = {}
    gui_last_check = 0.
    snapshot_calls = 0
    def capture_windows() -> None:
        nonlocal snapshot_calls
        if not gui:
            return
        for kind, window in gui_windows.items():
            snapshot_calls += 1
            run(["xwd", "-silent", "-id", window["window_id"], "-out", str(output/("gui_"+kind+".xwd"))],
                timeout=2, check=False)
    try:
        if gui:
            argv = make_command(source, sim, output, project)
            atomic_json(output/"make_invocation.json", dict(command=argv, existing_makefile=str(sim/"Makefile"),
                additional_makefile=str(source/"integrations/tiny_gui/Makefile"), source_root=str(source)))
            run(argv, timeout=60)
        else:
            run(command+["up", "-d", "--no-build", "--pull", "never", "simulator", "tiny"])
        sim_id = run(command+["ps", "-q", "simulator"]).stdout.strip()
        runtime_id = run(command+["ps", "-q", "autoware" if gui else "tiny"]).stdout.strip()
        if not all(re.fullmatch("[0-9a-f]{64}", value) for value in (sim_id, runtime_id)):
            raise ValueError("OWNED_CONTAINER_IDENTITY")
        atomic_json(output/"instance_inspect.json", json.loads(run(["docker", "inspect", sim_id, runtime_id]).stdout))
        while time.monotonic()-started < cfg["wall_seconds"]+60:
            if time.time() >= DRIVE_CUTOFF_UNIX_S:
                raise RuntimeError("ABSOLUTE_DRIVING_CUTOFF_FREEZE")
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
                capture_windows()
                continue
            if watch.armed:
                atomic_json(output/"host_armed.json", dict(token=project, armed=True, monotonic_ns=time.monotonic_ns(),
                    sim_id=sim_id, runtime_id=runtime_id, paused_kill_verified=True,
                    paused_kill_evidence_sha256="f7d1f227cafe340ca702aa3e974deb485db935e6c374313f39c89a2d778a3f32"))
            if gui and not gui_windows and time.monotonic()-gui_last_check >= .5:
                gui_last_check = time.monotonic()
                tree = run(["xwininfo", "-root", "-tree"], timeout=2).stdout
                gui_windows = visible_owned_windows(run, tree, sim_id, runtime_id)
                if gui_windows:
                    atomic_json(output/"gui_ready.json", dict(token=project, windows=gui_windows,
                        monotonic_ns=time.monotonic_ns(), display=args.display, tree=tree))
            unity = output/"awsim_unity.log"
            if unity.exists():
                if unity.stat().st_size < judge_offset:
                    raise ValueError("JUDGE_LOG_TRUNCATED_OR_RESET")
                line_offset = judge_offset-len(judge_pending)
                with unity.open("rb") as stream:
                    stream.seek(judge_offset)
                    chunk = stream.read(256*1024)
                    judge_offset = stream.tell()
                lines = (judge_pending+chunk).split(b"\n")
                judge_pending = lines.pop()
                for line in lines:
                    judge.feed(line.decode("utf-8", errors="replace"), byte_offset=line_offset)
                    line_offset += len(line)+1
                if judge.laps and not (output/"stop_request.json").exists():
                    atomic_json(output/"stop_request.json", dict(token=project,
                        reason="JUDGE_FIRST_LAP" if judge.completed else "JUDGE_LAP_EVIDENCE_INCOMPLETE"))
            # 09:50 JST cutoff leaves 10 minutes to stop and preserve evidence.
            if time.time() >= DRIVE_CUTOFF_UNIX_S-10:
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
        # A make failure may occur after simulator-up but before autoware-up.
        # Resolve only our compose scope so that such a partial startup is stopped.
        for service, present in (("simulator", sim_id), ("autoware" if gui else "tiny", runtime_id)):
            if present is None:
                try:
                    candidate = cleanup_run(command+["ps", "-aq", service], timeout=3).stdout.strip()
                    if candidate and not re.fullmatch(r"[0-9a-f]{64}", candidate):
                        raise ValueError("CLEANUP_OWNED_CONTAINER_IDENTITY")
                    if service == "simulator":
                        sim_id = candidate or None
                    else:
                        runtime_id = candidate or None
                except Exception as exc:
                    error = error or "PARTIAL_STARTUP_IDENTITY:"+str(exc)
        cleanup = cleanup_owned(cleanup_run, sim_id, runtime_id, paused_kill_verified=True)
        after = {str(p): digest(p) for p in assets}
        summary = dict(source_commit=args.commit, phase=args.phase, error=error, runtime_exitcode=exitcode,
            project=project, simulator_container_id=sim_id, runtime_container_id=runtime_id,
            simulator_assets_before=before, simulator_assets_after=after, simulator_assets_unchanged=before == after,
            wall_s=time.monotonic()-started, started_unix_ns=started_unix_ns, finished_unix_ns=time.time_ns(),
            host_armed=watch.armed, host_pause_verified=pause_verified,
            cleanup=cleanup, judge_section_events=judge.section_events, judge_laps=judge.laps,
            judge_order_invalid=judge.invalid, judge_lap_confirmed=judge.completed,
            control_claim_requires_supervisor_log=True, normal_racingkart_makefile_executed=gui,
            guarded_method_include_used=gui, gui_windows=gui_windows, snapshot_calls=snapshot_calls,
            judge_log_consumed_bytes=judge_offset, judge_unterminated_tail_bytes=len(judge_pending),
            authorization_profile=cfg["authorization_profile"], authorization_request_sha256=cfg["authorization_request_sha256"],
            awsimmutation=False, original_start_command="bash /aichallenge/run_simulator.bash dev")
        atomic_json(output/"host_summary.json", summary)
        consumption = dict(wall_s=summary["wall_s"], forward=0, mpc=0, snapshots=snapshot_calls,
            log_bytes=sum(p.stat().st_size for p in output.rglob("*") if p.is_file())+1024**2)
        worker_file, supervisor_file = output/"tiny_worker_summary.json", output/"tiny_supervisor_summary.json"
        tiny_calls = json.loads(worker_file.read_text())["tiny_forward_calls"] if worker_file.exists() else None
        if supervisor_file.exists():
            final = json.loads(supervisor_file.read_text())
            consumption.update(powered=final["powered_episode_count"], powered_s=final["powered_sim_seconds"])
            if gui:
                summary["supervisor_exception"] = final["exception"]
                if final["exception"] or not final["observed_motion"] or not final["observed_braking_stop"]:
                    error = error or "GUI_MOTION_STOP_NOT_COMPLETE:"+str(final["stop_reason"])
                if args.phase == "lap" and not judge.completed:
                    error = error or "GUI_LAP_NOT_CONFIRMED"
        elif gui:
            error = error or "GUI_SUPERVISOR_SUMMARY_MISSING"
        summary["error"] = error
        atomic_json(output/"host_summary.json", summary)
        budget.finish_tiny(consumption, tiny_calls, exact=tiny_calls is not None and len(consumption) == 7)
        atomic_json(output/"budget_after.json", budget.value)
        budget.close()
        log.close()
    print(json.dumps(summary))
    return int(error is not None or exitcode != 0 or bool(cleanup["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
