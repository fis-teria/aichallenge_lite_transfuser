"""Owned single-vehicle make-dev trial, finite model-only drive then measured stop.

Run under an outer timeout of (configured outer wall limit - 10 s) + 10 s kill grace.
Normal Autoware RViz is reused. No remote Git operation, global cleanup or
simulator/driver modification. The prepared install and checkpoint are explicit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time

from integrate_normal_rviz_v4 import ensure_time_path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aic_transfuser_lite.runtime.awsim_trial_session import JudgeLog, LowSpeedStall, trial_duration_limits


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--deployment", type=Path, required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--display", required=True, help="Verified active desktop, e.g. :0 or :1")
    ap.add_argument("--config", type=Path, default=Path("configs/control/time_path_awsim_trial_20260913.json"),
                    help="Trial configuration relative to this source tree")
    args = ap.parse_args()
    if not re.fullmatch(r"codex-time-[a-z0-9-]+", args.run_id):
        raise ValueError("INVALID_OWNED_RUN_ID")
    if not re.fullmatch(r":[0-9]+", args.display):
        raise ValueError("EXPLICIT_LOCAL_DISPLAY_REQUIRED")
    deployment = args.deployment.resolve()
    if deployment.parent != Path("/home/graneple/e2e_autonomous"):
        raise ValueError("UNEXPECTED_DEPLOYMENT_ROOT")
    repo = Path("/home/graneple/git/autononous_ai/aichallenge-racingkart")
    source = Path(__file__).resolve().parents[1]
    config_path = (source / args.config).resolve()
    if config_path.parent != source / "configs/control":
        raise ValueError("CONFIG_MUST_BE_IN_SOURCE_CONTROL_DIRECTORY")
    config = json.loads(config_path.read_text())
    execution_profile = config.get("execution_profile", "bounded_10s")
    drive_sim_s, drive_wall_s, outer_wall_s = trial_duration_limits(execution_profile)
    output = deployment / args.run_id; output.mkdir(exist_ok=False)
    (output / "trial_config.json").write_bytes(config_path.read_bytes())
    env = dict(os.environ)
    result = {"status": "FAILED", "run_id": args.run_id, "start_time_utc": time.time(),
              "scope": "ONE_LAP_MODEL_TRIAL" if execution_profile == "one_lap" else "10_SIM_SECOND_MODEL_TRIAL",
              "official_start_requested": False, "execution_profile": execution_profile,
              "speed_policy": config.get("speed_policy", "source_capped_0p25"),
              "trial_config_sha256": sha(config_path)}
    streams = []; processes = []; compose = None; owned = False
    started = time.monotonic()
    judge = JudgeLog(args.run_id); judge_offset = 0; judge_pending = b""
    stall = LowSpeedStall(); rviz_window = None; next_capture_sim_s = 2.

    def run(command, timeout=8, check=True):
        return subprocess.run(command, cwd=repo, env=env, text=True, capture_output=True, timeout=timeout, check=check)

    def launch(command, name):
        stream = (output / (name + ".log")).open("x"); streams.append(stream)
        process = subprocess.Popen(command, cwd=repo, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); return process

    def interrupted(signum, frame):
        raise RuntimeError("OUTER_SUPERVISOR_TERMINATED")
    signal.signal(signal.SIGTERM, interrupted)

    def stop_request(reason: str) -> None:
        path = output / "stop_request.json"
        if not path.exists():
            pending = output / "stop_request.pending"
            pending.write_text(json.dumps({"run_id": args.run_id, "reason": reason}))
            pending.replace(path)

    try:
        inventory = run(["docker", "ps", "-a", "--format", "{{json .}}"])
        (output / "containers_before.jsonl").write_text(inventory.stdout)
        (output / "compose_before.json").write_text(run(["docker", "compose", "ls", "--all", "--format", "json"]).stdout)
        if run(["docker", "ps", "-q"]).stdout.strip():
            raise RuntimeError("ACTIVE_CONTAINER_PRESENT")
        if any(Path(p).exists() for p in ("/dev/vcu", "/dev/gnss", "/dev/ttyUSB0")):
            raise RuntimeError("PHYSICAL_DEVICE_PRESENT")
        gpu = run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"], check=False)
        result["gpu"] = {"returncode": gpu.returncode, "stdout": gpu.stdout, "stderr": gpu.stderr}
        if gpu.returncode:
            raise RuntimeError("GPU_PREFLIGHT_FAILED")
        scene = repo / "aichallenge/simulator/AWSIM/AWSIM_Data/level1"
        if sha(scene) != config["geometry"]["scene_sha256"]:
            raise RuntimeError("SCENE_GEOMETRY_IDENTITY_CHANGED")
        checkpoint = deployment / "command_off_best.pt"
        if sha(checkpoint) != config["checkpoint_sha256"]:
            raise RuntimeError("CHECKPOINT_IDENTITY_CHANGED")
        if not (deployment / "install/setup.bash").is_file():
            raise RuntimeError("ROS_INSTALL_MISSING")
        auth = list(Path("/run/user/1000").glob(".mutter-Xwaylandauth.*"))
        # A reboot can select an Xorg desktop instead of Xwayland. Both are
        # normal user-session authentication paths; never copy/read the secret.
        xorg_auth = Path("/run/user/1000/gdm/Xauthority")
        if xorg_auth.is_file():
            auth.append(xorg_auth)
        if len(auth) != 1:
            raise RuntimeError("DISPLAY_AUTH_UNKNOWN")
        # Task-local transport keeps the established loopback transport; the
        # isolated image cannot require a host-global 10 MB sysctl minimum.
        dds = (repo / "vehicle/cyclonedds.xml").read_text()
        if 'name="lo"' not in dds or '<SocketReceiveBufferSize min="10MB"/>' not in dds:
            raise RuntimeError("UNRECOGNIZED_DDS_PROFILE")
        runtime_dds = output / "cyclonedds.xml"
        dds = dds.replace('<SocketReceiveBufferSize min="10MB"/>', '<SocketReceiveBufferSize min="128kB"/>')
        dds = dds.replace('<AllowMulticast>default</AllowMulticast>', '<AllowMulticast>false</AllowMulticast>')
        discovery = ('<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
                     '<Peers><Peer Address="127.0.0.1"/></Peers></Discovery>')
        runtime_dds.write_text(dds.replace('</Domain>', discovery+'</Domain>'))
        override = output / "compose.json"
        override.write_text(json.dumps({"services": {service: {"volumes": [
            str(runtime_dds) + ":/opt/autoware/cyclonedds.xml:ro"]} for service in ("simulator", "autoware", "autoware-command")}}, indent=2))
        env.update(DISPLAY=args.display, XAUTHORITY=str(auth[0]), COMPOSE_PROJECT_NAME=args.run_id,
            COMPOSE_FILE=":".join(map(str, (repo/"docker-compose.yml", repo/"docker-compose.gpu.yml", override))),
            CONTROL_METHOD="v4_20_external", V4_SHADOW_ENABLED="false")
        compose = ["docker", "compose", "-p", args.run_id]
        rviz = repo / "aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz"
        rviz_bytes = rviz.read_bytes()
        (output / "autoware.rviz.before").write_bytes(rviz_bytes)
        text = rviz_bytes.decode("utf-8")
        newline = "\r\n" if "\r\n" in text else "\n"
        enabled = ensure_time_path(text.replace("\r\n", "\n")).replace("\n", newline).encode("utf-8")
        if enabled != rviz_bytes:
            rviz.write_bytes(enabled)
        result["rviz_config"] = str(rviz); result["rviz_sha256"] = sha(rviz)
        sidecar = args.run_id + "-nodes"
        inside = Path("/time") / output.name
        source_in_container = Path("/time") / source.relative_to(deployment)
        node_command = ["python3", str(source_in_container / "tools/run_time_path_trial_nodes.py"),
                        "--output", str(inside), "--run-id", args.run_id, "--checkpoint", "/time/command_off_best.pt",
                        "--config", str(source_in_container / config_path.relative_to(source))]
        shell = "source /aichallenge/workspace/install/setup.bash && source /time/install/setup.bash && exec " + shlex.join(node_command)
        probe_command = ["docker", "run", "--rm", "--name", sidecar, "--gpus", "all", "--network", "host",
            "-e", "ROS_DOMAIN_ID=1", "-e", "CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml",
            "-v", str(runtime_dds)+":/opt/autoware/cyclonedds.xml:ro", "-v", str(deployment)+":/time",
            "-v", str(repo/"aichallenge")+":/aichallenge:ro", "--entrypoint", "bash",
            "codex-cartographer-v4-build:20260910", "-lc", shell]
        make_args = ["CONTROL_METHOD=v4_20_external", "CAPTURE=false", "ROSBAG=false", "AWSIM_LAPS=1",
                     "RUN_ID="+args.run_id, "OUTPUT_HOST_ROOT="+str(output)]
        if execution_profile == "one_lap":
            # Compose mounts this host output directory at /output in AWSIM.
            make_args += ["AWSIM_TIMEOUT=660", "AWSIM_EXTRA_ARGS=-logFile " + str(Path("/output")/args.run_id/"awsim_unity.log")]
        result["commands"] = [probe_command, ["make", "dev", "DEV_AUTO_START=false", *make_args]]
        owned = True
        probe = launch(probe_command, "nodes")
        make = launch(result["commands"][1], "make")
        while time.monotonic() - started < outer_wall_s - 25:
            if probe.poll() is not None:
                raise RuntimeError("TRIAL_NODES_EXIT")
            if make.poll() is not None and make.returncode:
                raise RuntimeError("MAKE_DEV_FAILED")
            ip = output/"inference_heartbeat.json"; cp = output/"control_heartbeat.json"
            unity = output/args.run_id/"awsim_unity.log"
            if execution_profile == "one_lap" and unity.exists():
                if unity.stat().st_size < judge_offset:
                    raise RuntimeError("JUDGE_LOG_TRUNCATED_OR_RESET")
                line_offset = judge_offset - len(judge_pending)
                with unity.open("rb") as stream:
                    stream.seek(judge_offset); chunk = stream.read(256*1024); judge_offset = stream.tell()
                lines = (judge_pending + chunk).split(b"\n"); judge_pending = lines.pop()
                for line in lines:
                    judge.feed(line.decode("utf-8", errors="replace"), byte_offset=line_offset)
                    line_offset += len(line)+1
                if judge.laps:
                    stop_request("JUDGE_FIRST_LAP" if judge.completed else "JUDGE_LAP_EVIDENCE_INCOMPLETE")
            if cp.exists():
                control = json.loads(cp.read_text()); result["last_control"] = control
                if control["fault"]:
                    raise RuntimeError("CONTROL_"+control["fault"])
                if time.monotonic_ns() - control["monotonic_ns"] > 1_000_000_000:
                    raise RuntimeError("CONTROLLER_WATCHDOG")
                if control["stop_confirmed"]:
                    result["status"] = ("COMPLETE_LAP" if judge.completed else "STOPPED_NO_LAP") if execution_profile == "one_lap" else "COMPLETE_BOUNDED_TRIAL"
                    break
                if execution_profile == "one_lap" and control["armed_ns"] is not None:
                    elapsed_sim_s = (control["sim_ns"] - control["armed_ns"]) / 1e9
                    if control["reason"] == "CLOCK_STALE":
                        raise RuntimeError("ARMED_CLOCK_STALE")
                    if control.get("speed_mps") is not None and stall.update(control["sim_ns"], control["speed_mps"]):
                        stop_request("PROGRESS_STALLED")
                    if rviz_window is not None and elapsed_sim_s >= next_capture_sim_s:
                        run(["xwd", "-silent", "-id", rviz_window, "-out", str(output/f"rviz_drive_{int(next_capture_sim_s):03d}.xwd")], timeout=3)
                        next_capture_sim_s += 60.
                progress = output/"lap_progress.pending"
                progress.write_text(json.dumps({"sections":judge.section_events,"laps":judge.laps,
                    "lap_confirmed":judge.completed,"last_control":control},allow_nan=False))
                progress.replace(output/"lap_progress.json")
            if ip.exists() and cp.exists() and make.poll() == 0 and not result["official_start_requested"]:
                inference = json.loads(ip.read_text()); result["last_inference"] = inference
                if inference["plans"] >= 5 and control["reason"] == "WAIT_AUTHORIZATION" and control["commands"] >= 5:
                    if not any(name.startswith("rviz") for name in inference.get("path_subscribers", [])):
                        if time.monotonic()-started > 45:
                            raise RuntimeError("RVIZ_PATH_NOT_SUBSCRIBED")
                        time.sleep(.1); continue
                    if execution_profile == "one_lap" and not unity.exists():
                        raise RuntimeError("JUDGE_LOG_MISSING")
                    result["rviz_path_subscribers"] = inference["path_subscribers"]
                    inspections = []
                    for service in ("simulator", "autoware"):
                        cid = run(compose+["ps", "-q", service]).stdout.strip()
                        info = json.loads(run(["docker", "inspect", cid]).stdout)[0]
                        if info["Config"]["Labels"].get("com.docker.compose.project") != args.run_id:
                            raise RuntimeError("COMPOSE_OWNER_MISMATCH")
                        if any(m["Destination"] in ("/dev/vcu", "/dev/gnss") for m in info["Mounts"]):
                            raise RuntimeError("PHYSICAL_DEVICE_MOUNT")
                        inspections.append(info)
                    (output/"inspect.json").write_text(json.dumps(inspections, indent=2))
                    try:
                        tree = run(["xwininfo", "-root", "-tree"]).stdout
                        windows = [line for line in tree.splitlines() if 'autoware.rviz' in line]
                        if len(windows) == 1:
                            rviz_window = windows[0].strip().split()[0]
                            run(["xwd", "-silent", "-id", windows[0].strip().split()[0],
                                 "-out", str(output/"normal_rviz.xwd")])
                            result["rviz_window"] = windows[0].strip()
                    except Exception as exc:
                        result["rviz_capture_error"] = str(exc)
                    if rviz_window is None:
                        raise RuntimeError("NORMAL_RVIZ_WINDOW_MISSING")
                    start_cmd = ["make", "awsim-request-start", *make_args]
                    result["commands"].append(start_cmd)
                    official = launch(start_cmd, "official_start")
                    if official.wait(timeout=20):
                        raise RuntimeError("OFFICIAL_START_FAILED")
                    result["official_start_requested"] = True
                    (output/"drive_authorized.json").write_text(json.dumps({"scope": "TIME_PATH_AWSIM_TRIAL",
                        "run_id": args.run_id, "expires_monotonic_s": time.monotonic()+20,
                        "source": "USER_REQUEST_20260913"}))
            if time.monotonic()-started > 45 and not cp.exists():
                raise RuntimeError("CONTROLLER_STARTUP_TIMEOUT")
            time.sleep(.1)
        else:
            raise RuntimeError("TRIAL_WALL_LIMIT")
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        cleanup = []
        if owned and compose is not None:
            # Freeze owned simulation before stopping its only controller.
            for service in ("simulator", "autoware"):
                try:
                    cid = run(compose+["ps", "-q", service], timeout=3).stdout.strip()
                    if cid:
                        if service == "simulator":
                            run(["docker", "pause", cid], timeout=3, check=False)
                            run(["docker", "kill", cid], timeout=3)
                        else:
                            run(["docker", "stop", "-t", "2", cid], timeout=5)
                except Exception as exc:
                    cleanup.append(str(exc))
            run(["docker", "stop", "-t", "2", args.run_id+"-nodes"], timeout=5, check=False)
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2)
            try:
                run(compose+["down", "--timeout", "2"], timeout=8)
            except Exception as exc:
                cleanup.append(str(exc))
        for stream in streams:
            stream.close()
        result.update(cleanup_errors=cleanup, end_time_utc=time.time(), wall_s=time.monotonic()-started)
        result.update(judge_lap_confirmed=judge.completed, judge_section_events=judge.section_events,
                      judge_laps=judge.laps, judge_order_invalid=judge.invalid)
        (output/"host_result.json").write_text(json.dumps(result, indent=2, allow_nan=False))
        (output/"containers_after.jsonl").write_text(run(["docker", "ps", "-a", "--format", "{{json .}}"], check=False).stdout)
        (output/"compose_after.json").write_text(run(["docker", "compose", "ls", "--all", "--format", "json"], check=False).stdout)
        print(json.dumps(result, allow_nan=False))
    if result["status"] not in ("COMPLETE_BOUNDED_TRIAL", "COMPLETE_LAP"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
