"""New guarded CONTROL_METHOD glue; no recursive source/build/data fingerprint."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from aic_transfuser_lite.runtime.tiny_lidar_sim import (
    digest, GUI_CONTROL_METHOD, GUI_AUTH_SHA256, TINY_GUI_PROFILE,
)
from spatial_dev_host_v4 import atomic_json


def compose_gui(source: Path, sim: Path, install: Path, output: Path, official: Path,
                project: str, commit: str, display: str, xauthority: Path, image: str) -> dict:
    if not re.fullmatch(r":\d+(\.\d+)?", display) or not xauthority.is_file():
        raise ValueError("EXPLICIT_LOCAL_DESKTOP_REQUIRED")
    if not (install/"setup.bash").is_file():
        raise ValueError("BUILT_ROS_PACKAGE_REQUIRED")
    common = dict(image=image, pull_policy="never", privileged=False, network_mode="none",
        cap_drop=["ALL"], security_opt=["no-new-privileges:true"], ipc="private", pids_limit=512,
        mem_limit="6g", cpus=4, shm_size="256m", user=f"{os.getuid()}:{os.getgid()}",
        hostname=os.uname().nodename, entrypoint=["/bin/bash"], restart="no", gpus="all",
        stop_grace_period="12s", working_dir="/evidence")
    def mount(path: Path, target: str, ro: bool = True) -> dict:
        return dict(type="bind", source=str(path.resolve()), target=target, read_only=ro)
    shared = [mount(source, "/v4"), mount(output, "/evidence", False),
              mount(Path("/tmp/.X11-unix"), "/tmp/.X11-unix"), mount(xauthority, "/desktop_xauth")]
    env = dict(ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        CYCLONEDDS_URI="file:///v4/integrations/awsim_dev_v4/cyclonedds.xml",
        TINY_SOURCE_COMMIT=commit, TINY_PROJECT=project, CONTROL_METHOD=GUI_CONTROL_METHOD,
        DISPLAY=display, XAUTHORITY="/desktop_xauth", QT_X11_NO_MITSHM="1", ROS_LOG_DIR="/evidence/ros_logs",
        XAUTHLOCALHOSTNAME=os.uname().nodename,
        __NV_PRIME_RENDER_OFFLOAD="1", PYTHONDONTWRITEBYTECODE="1")
    simulator = dict(common, command=["/v4/integrations/tiny_gui/simulator.sh"],
        environment=dict(env, ROS_DOMAIN_ID="0", AWSIM_START_MODE="off", AWSIM_VEHICLES="1",
            AWSIM_LAPS="600", AWSIM_TIMEOUT="60000000",
            AWSIM_EXTRA_ARGS="--manual-mode true --npcs 0 --boosts 0 --collisions on --wall-recovery off --start-random off --camera gpu --lidar gpu --sound off --target-fps 20 -screen-width 960 -screen-height 540 -logFile /evidence/awsim_unity.log"),
        volumes=shared+[mount(sim/"aichallenge/simulator/AWSIM", "/aichallenge/simulator/AWSIM"),
                        mount(sim/"aichallenge/run_simulator.bash", "/aichallenge/run_simulator.bash")])
    autoware = dict(common, network_mode="service:simulator", depends_on=["simulator"],
        command=["/v4/integrations/tiny_gui/runtime.sh"], environment=dict(env, ROS_DOMAIN_ID="1"),
        volumes=shared+[mount(install, "/tiny_install"), mount(official, "/official_tiny")])
    # Docker forbids explicit Hostname with container:/service: network sharing.
    # Keep cookie lookup local to the desktop hostname via XAUTHLOCALHOSTNAME;
    # do not disable X authentication or expose a host network to work around it.
    autoware.pop("hostname")
    return dict(name=project, services=dict(simulator=simulator, autoware=autoware))


def make_command(source: Path, sim: Path, output: Path, project: str) -> list[str]:
    """Existing racing-kart dev recipe, new method-specific include only."""
    return ["make", "-f", str(source/"integrations/tiny_gui/Makefile"), "dev",
        "CONTROL_METHOD="+GUI_CONTROL_METHOD, "RACINGKART_REPO="+str(sim),
        "TINY_SOURCE_ROOT="+str(source), "TINY_TRIAL_OUTPUT="+str(output),
        "COMPOSE_FILE="+str(output/"compose.json"), "COMPOSE_PROJECT_NAME="+project,
        "RUN_ID="+output.name, "OUTPUT_HOST_ROOT="+str(output.parent), "OUTPUT_ROOT=/evidence",
        "DEV_AUTO_START=false", "ROSBAG=false"]


def narrow_fingerprint(source: Path, sim: Path, output: Path) -> dict:
    config = json.loads((output/"resolved_config.json").read_text())
    if (config.get("control_method") != GUI_CONTROL_METHOD or config.get("authorization_profile") != TINY_GUI_PROFILE
            or config.get("authorization_request_sha256") != GUI_AUTH_SHA256):
        raise ValueError("NARROW_FINGERPRINT_ONLY_FOR_EXPLICIT_GUI_PROFILE")
    relative = ["tools/run_tiny_lidar_dev.py", "tools/tiny_dev_runner.py", "tools/tiny_gui_integration.py",
        "tools/spatial_dev_host_v4.py", "src/aic_transfuser_lite/runtime/tiny_lidar_sim.py",
        "integrations/tiny_gui/Makefile", "integrations/tiny_gui/runtime.sh", "integrations/tiny_gui/simulator.sh",
        "configs/control/tiny_gui_authorization_20260907.json",
        "ros2_ws/src/aic_tiny_sim_test/package.xml", "ros2_ws/src/aic_tiny_sim_test/setup.py",
        "ros2_ws/src/aic_tiny_sim_test/launch/guarded_tiny.launch.py",
        "ros2_ws/src/aic_tiny_sim_test/config/tiny_scan.rviz",
        "ros2_ws/src/aic_tiny_sim_test/aic_tiny_sim_test/supervisor.py"]
    paths = [source/p for p in relative]+[sim/"Makefile", sim/"aichallenge/run_simulator.bash",
                                       output/"compose.json", output/"resolved_config.json"]
    value = dict(schema="TINY_GUI_NARROW_FINGERPRINT_V1", created_unix_ns=time.time_ns(),
        source_commit=config["source_commit"], control_method=GUI_CONTROL_METHOD,
        legacy_full_tree_fingerprint_executed=False, full_tree_integrity_claim=False,
        source_build_install_recursive_reads=False, other_model_weight_reads=False,
        files=[dict(path=str(p), bytes=p.stat().st_size, sha256=digest(p)) for p in paths])
    atomic_json(output/"tiny_gui_fingerprint.json", value)
    return value


def window_candidates(tree: str) -> dict[str, list[str]]:
    windows: dict[str, list[str]] = {"awsim": [], "rviz": []}
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-fA-F]+)\s+"([^"]+)"', line)
        if match:
            title = match[2].lower()
            if "awsim" in title:
                windows["awsim"].append(match[1])
            if "rviz" in title:
                windows["rviz"].append(match[1])
    return windows


def visible_owned_windows(run, tree: str, sim_id: str, runtime_id: str) -> dict:
    """Match X window PID to the scoped container's process namespace PID."""
    candidates = window_candidates(tree)
    if not all(candidates.values()):
        return {}
    result = {}
    for kind, container, command_name in (("awsim", sim_id, "AWSIM.x86_64"), ("rviz", runtime_id, "rviz2")):
        top = run(["docker", "top", container, "-eo", "pid,comm"], timeout=2).stdout
        internal = set()
        for line in top.splitlines()[1:]:
            fields = line.split()
            if len(fields) != 2 or not fields[0].isdigit() or fields[1] != command_name:
                continue
            try:
                status = Path("/proc", fields[0], "status").read_text()
            except FileNotFoundError:
                continue
            ns = re.search(r"^NSpid:\s+([\d\s]+)$", status, re.MULTILINE)
            if ns:
                internal.add(int(ns[1].split()[-1]))
        for window in candidates[kind]:
            prop = run(["xprop", "-id", window, "_NET_WM_PID"], timeout=2, check=False).stdout
            pid = re.search(r"=\s*(\d+)", prop)
            info = run(["xwininfo", "-id", window], timeout=2, check=False).stdout
            if pid and int(pid[1]) in internal and "Map State: IsViewable" in info:
                result[kind] = dict(window_id=window, namespace_pid=int(pid[1]), container_id=container,
                                    viewable=True, details=info, property=prop)
                break
    return result if set(result) == {"awsim", "rviz"} else {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fingerprint", type=Path, required=True)
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--racing-repo", type=Path, required=True)
    args = ap.parse_args()
    narrow_fingerprint(args.source.resolve(), args.racing_repo.resolve(), args.fingerprint.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
