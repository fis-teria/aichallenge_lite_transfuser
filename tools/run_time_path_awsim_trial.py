"""Owned single-ego make-dev trial, finite model-only drive then measured stop.

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

from integrate_normal_rviz_v4 import ensure_time_path, follow_ego_view

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aic_transfuser_lite.runtime.awsim_trial_session import (
    JudgeLog, LowSpeedStall, npc_startup_evidence, trial_duration_limits,
)
from aic_transfuser_lite.control.awsim_steering import steering_asset_contract
from aic_transfuser_lite.control.time_dev_v1 import configure_dev_speeds
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config


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
    ap.add_argument('--recovery-side', choices=('left', 'right'),
                    help='Teacher bootstrap, finite pulse, then one-way E2E takeover')
    ap.add_argument('--record-video', action='store_true', help='Record only the owned AWSIM and normal RViz windows')
    ap.add_argument('--ros-launch', action='store_true', help='Use the installed time_path_awsim ROS launch')
    ap.add_argument('--max-speed-kmh', type=float)
    ap.add_argument('--corner-max-speed-kmh', type=float)
    ap.add_argument('--npcs', type=int, choices=range(4), default=0)
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
    if args.ros_launch:
        if args.recovery_side is not None or args.max_speed_kmh is None or args.corner_max_speed_kmh is None:
            raise ValueError('TIME_DEV_REQUIRES_BOTH_SPEED_PARAMETERS_AND_NO_TEACHER')
        config = configure_dev_speeds(config, max_speed_kmh=args.max_speed_kmh,
                                     corner_max_speed_kmh=args.corner_max_speed_kmh)
    elif args.max_speed_kmh is not None or args.corner_max_speed_kmh is not None:
        raise ValueError('SPEED_PARAMETERS_REQUIRE_ROS_LAUNCH')
    validate_trial_config(config)
    execution_profile = config.get("execution_profile", "bounded_10s")
    if args.npcs and (execution_profile != 'one_lap' or args.recovery_side is not None):
        raise ValueError('NPC_TRIAL_REQUIRES_ONE_LAP_WITHOUT_TEACHER')
    drive_sim_s, drive_wall_s, outer_wall_s = trial_duration_limits(execution_profile)
    output = deployment / args.run_id; output.mkdir(exist_ok=False)
    effective_config = output / 'trial_config.json'
    effective_config.write_bytes((json.dumps(config, indent=2, allow_nan=False)+'\n').encode()
                                if args.ros_launch else config_path.read_bytes())
    env = dict(os.environ)
    result = {"status": "FAILED", "run_id": args.run_id, "start_time_utc": time.time(),
              "scope": "ONE_LAP_MODEL_TRIAL" if execution_profile == "one_lap" else "10_SIM_SECOND_MODEL_TRIAL",
              "official_start_requested": False, "execution_profile": execution_profile,
              "requested_npcs": args.npcs,
              "speed_policy": config.get("speed_policy", "source_capped_0p25"),
              "trial_config_sha256": sha(effective_config)}
    if args.recovery_side is not None:
        result.update(scope='ONE_LAP_AFTER_TEACHER_BOOTSTRAP', recovery_side=args.recovery_side)
    streams = []; processes = []; compose = None; owned = False
    started = time.monotonic()
    judge = JudgeLog(args.run_id); judge_offset = 0; judge_pending = b""
    stall = LowSpeedStall(); rviz_window = None; next_capture_sim_s = 2.; video = None
    rviz = None; rviz_bytes = None; enabled = None

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
        asset_hashes = {name: sha(scene.parent.parent / name) for name in steering_asset_contract(config)}
        if asset_hashes != steering_asset_contract(config):
            raise RuntimeError("STEERING_ACTUATOR_ASSET_CHANGED")
        result["steering_asset_sha256"] = asset_hashes
        checkpoint = deployment / "command_off_best.pt"
        if sha(checkpoint) != config["checkpoint_sha256"]:
            raise RuntimeError("CHECKPOINT_IDENTITY_CHANGED")
        if not (deployment / "install/setup.bash").is_file():
            raise RuntimeError("ROS_INSTALL_MISSING")
        if args.recovery_side is not None:
            from aic_transfuser_lite.runtime.time_recovery_takeover_v1 import validate_recovery_reference
            if execution_profile != 'one_lap':
                raise ValueError('RECOVERY_ONE_LAP_PROFILE_REQUIRED')
            assets = json.loads((deployment/'recovery_assets.json').read_text())
            for entry in assets['files']:
                asset = (deployment/entry['path']).resolve()
                if deployment not in asset.parents or sha(asset) != entry['sha256']:
                    raise ValueError('RECOVERY_ASSET_IDENTITY')
            reference_path = deployment/'references'/(args.recovery_side+'.json')
            reference = json.loads(reference_path.read_text())
            validate_recovery_reference(reference)
            if sha(reference_path.with_suffix('.csv')) != reference['reference_sha256']:
                raise ValueError('RECOVERY_REFERENCE_CSV_IDENTITY')
            result['recovery_assets_sha256'] = sha(deployment/'recovery_assets.json')
            result['recovery_reference_sha256'] = sha(reference_path)
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
        enabled_text = ensure_time_path(text.replace("\r\n", "\n"))
        if execution_profile == "one_lap":
            enabled_text = follow_ego_view(enabled_text)
        enabled = enabled_text.replace("\n", newline).encode("utf-8")
        if enabled != rviz_bytes:
            rviz.write_bytes(enabled)
        result["rviz_config"] = str(rviz); result["rviz_sha256"] = sha(rviz)
        sidecar = args.run_id + "-nodes"
        inside = Path("/time") / output.name
        source_in_container = Path("/time") / source.relative_to(deployment)
        node_command = ["python3", str(source_in_container / "tools/run_time_path_trial_nodes.py"),
                        "--output", str(inside), "--run-id", args.run_id, "--checkpoint", "/time/command_off_best.pt",
                        "--config", str(source_in_container / config_path.relative_to(source))]
        if args.recovery_side is not None:
            node_command += ['--recovery-reference', '/time/references/'+args.recovery_side+'.json']
        if args.ros_launch:
            node_command = ['ros2', 'launch', 'aic_e2e_runtime', 'time_path_awsim.launch.py',
                'output:='+str(inside), 'run_id:='+args.run_id, 'checkpoint:=/time/command_off_best.pt',
                'config:='+str(inside/'trial_config.json'), 'authorize_awsim_only:=true',
                'max_speed_kmh:='+str(args.max_speed_kmh),
                'corner_max_speed_kmh:='+str(args.corner_max_speed_kmh)]
        shell = ("source /aichallenge/workspace/install/setup.bash && "
                 + ('source /time/cpp_install/setup.bash && ' if args.recovery_side is not None else '')
                 + "source /time/install/setup.bash && exec " + shlex.join(node_command))
        probe_command = ["docker", "run", "--rm", "--name", sidecar, "--gpus", "all", "--network", "host",
            "-e", "ROS_DOMAIN_ID=1", "-e", "CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml",
            "-v", str(runtime_dds)+":/opt/autoware/cyclonedds.xml:ro", "-v", str(deployment)+":/time",
            "-v", str(repo/"aichallenge")+":/aichallenge:ro", "--entrypoint", "bash",
            "codex-cartographer-v4-build:20260910", "-lc", shell]
        make_args = ["CONTROL_METHOD=v4_20_external", "CAPTURE=false", "ROSBAG=false", "AWSIM_LAPS=1",
                     "RUN_ID="+args.run_id, "OUTPUT_HOST_ROOT="+str(output)]
        simulator_args = ['--npcs', str(args.npcs)]
        if args.npcs:
            simulator_args += ['--collisions', 'on']
        if execution_profile == "one_lap":
            # Compose mounts this host output directory at /output in AWSIM.
            make_args += ["AWSIM_TIMEOUT=660"]
            simulator_args += ['-logFile', str(Path('/output')/args.run_id/'awsim_unity.log')]
        make_args += ['AWSIM_EXTRA_ARGS=' + ' '.join(simulator_args)]
        result["commands"] = [probe_command, ["make", "dev", "DEV_AUTO_START=false", *make_args]]
        owned = True
        probe = launch(probe_command, "nodes")
        make = launch(result["commands"][1], "make")
        while time.monotonic() - started < outer_wall_s - 25:
            if video is not None:
                video.monitor()
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
                    if result['status'] == 'COMPLETE_LAP' and args.recovery_side is not None:
                        if control.get('recovery_state', {}).get('takeover_ns') is None:
                            raise RuntimeError('RECOVERY_TAKEOVER_MISSING')
                        result['status'] = 'COMPLETE_LAP_AFTER_TEACHER_BOOTSTRAP'
                    break
                if execution_profile == "one_lap" and control["armed_ns"] is not None:
                    elapsed_sim_s = (control["sim_ns"] - control["armed_ns"]) / 1e9
                    if control["reason"] == "CLOCK_STALE":
                        raise RuntimeError("ARMED_CLOCK_STALE")
                    if control.get("speed_mps") is not None and stall.update(control["sim_ns"], control["speed_mps"]):
                        stop_request("PROGRESS_STALLED")
                    if rviz_window is not None and elapsed_sim_s >= next_capture_sim_s:
                        run(["xwd", "-silent", "-id", rviz_window, "-out", str(output/f"rviz_drive_{int(next_capture_sim_s):03d}.xwd")], timeout=3)
                        next_capture_sim_s = 20. if next_capture_sim_s == 2. else next_capture_sim_s + 60.
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
                    if execution_profile == 'one_lap':
                        result['npc_startup'] = npc_startup_evidence(unity.read_text(errors='replace'), args.npcs)
                        (output/'npc_startup.json').write_text(json.dumps(result['npc_startup'], indent=2))
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
                    if args.record_video:
                        from time_trial_video import TrialVideo, select_video_windows
                        video = TrialVideo(output, args.run_id, args.display, select_video_windows(tree, rviz_window))
                        video.start()
                    if args.recovery_side is not None:
                        from aic_transfuser_lite.data.time_recovery_collection_v1 import validate_collection_speed_parameters
                        import yaml
                        params = run(['docker', 'exec', sidecar, 'bash', '-lc',
                            'source /aichallenge/workspace/install/setup.bash && source /time/cpp_install/setup.bash && ros2 param dump /recovery_teacher_pure_pursuit'], timeout=15)
                        (output/'pure_pursuit_loaded.yaml').write_text(params.stdout)
                        loaded = yaml.safe_load(params.stdout)
                        if not isinstance(loaded, dict) or len(loaded) != 1:
                            raise RuntimeError('PP_PARAMETER_DUMP_SHAPE')
                        validate_collection_speed_parameters('aligned_gain4_v1', next(iter(loaded.values()))['ros__parameters'])
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
                            frozen = run(["docker", "pause", cid], timeout=3, check=False)
                            if frozen.returncode == 0 and rviz_window is not None:
                                try:
                                    run(["xwd", "-silent", "-id", rviz_window, "-out", str(output/"rviz_after_freeze.xwd")], timeout=3)
                                except Exception as exc:
                                    result["final_rviz_capture_error"] = str(exc)
                            if video is not None:
                                cleanup.extend(video.stop()); video = None
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
        if video is not None:
            cleanup.extend(video.stop())
        if rviz is not None and rviz_bytes is not None and enabled is not None and enabled != rviz_bytes:
            try:
                if rviz.read_bytes() == enabled:
                    rviz.write_bytes(rviz_bytes)
                else:
                    cleanup.append('RVIZ_CHANGED_DURING_RUN_PRESERVED')
            except Exception as exc:
                cleanup.append('RVIZ_RESTORE:'+str(exc))
        for stream in streams:
            stream.close()
        result.update(cleanup_errors=cleanup, end_time_utc=time.time(), wall_s=time.monotonic()-started)
        result.update(judge_lap_confirmed=judge.completed, judge_section_events=judge.section_events,
                      judge_laps=judge.laps, judge_order_invalid=judge.invalid)
        (output/"host_result.json").write_text(json.dumps(result, indent=2, allow_nan=False))
        (output/"containers_after.jsonl").write_text(run(["docker", "ps", "-a", "--format", "{{json .}}"], check=False).stdout)
        (output/"compose_after.json").write_text(run(["docker", "compose", "ls", "--all", "--format", "json"], check=False).stdout)
        print(json.dumps(result, allow_nan=False))
    if result["status"] not in ("COMPLETE_BOUNDED_TRIAL", "COMPLETE_LAP", "COMPLETE_LAP_AFTER_TEACHER_BOOTSTRAP"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
