"""Launch separate inference/controller processes inside the owned trial container."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--recovery-reference", type=Path)
    args = ap.parse_args()
    if args.recovery_reference is not None:
        raise ValueError('GLOBAL_TEACHER_BOOTSTRAP_UNSUPPORTED_WITH_LOCAL_ODOMETRY')
    config = json.loads(args.config.read_text())
    # Use the installed, source-matched standard-library schedule helper.
    from aic_e2e_runtime import spatial_path_shadow_node_v4 as _source_layout
    from aic_transfuser_lite.runtime.awsim_trial_session import trial_duration_limits
    _, _, outer_wall_s = trial_duration_limits(config.get("execution_profile", "bounded_10s"))
    args.output.mkdir(parents=True, exist_ok=True)
    common = ["--output", str(args.output), "--run-id", args.run_id,
              "--checkpoint-sha256", config["checkpoint_sha256"]]
    commands = [
        [sys.executable, "-m", "aic_e2e_runtime.time_path_node", *common,
         "--checkpoint", str(args.checkpoint), "--device", "cuda"],
        [sys.executable, "-m", "aic_e2e_runtime.time_trial_controller_node", *common,
         "--trial-config", str(args.config),
         "--rear-axle-forward-m", str(config["geometry"]["rear_axle_forward_in_base_link_m"]),
         "--authorize-awsim-only"],
    ]
    names = ['inference', 'controller']
    if args.recovery_reference is not None:
        commands[1] += ['--recovery-reference', str(args.recovery_reference)]
        commands += [
            ['ros2', 'run', 'simple_trajectory_generator', 'simple_trajectory_generator_node', '--ros-args',
             '-r', '__node:=recovery_teacher_trajectory', '-r', 'trajectory:=/recovery_teacher/trajectory',
             '-p', 'use_sim_time:=true', '-p', 'z:=0.0',
             '-p', 'csv_path:='+str(args.recovery_reference.with_suffix('.csv'))],
            ['ros2', 'launch', '/time/inputs/pure_pursuit.launch.xml',
             'node_name:=recovery_teacher_pure_pursuit', 'use_sim_time:=true',
             'input_kinematics:=/localization/kinematic_state', 'input_trajectory:=/recovery_teacher/trajectory',
             'output_control_cmd:=/recovery_teacher/nominal_control_cmd',
             'output_raw_control_cmd:=/recovery_teacher/raw_control_cmd',
             'use_external_target_vel:=true', 'external_target_vel:=1.3888888888888888',
             'speed_proportional_gain:=4.0', 'use_overtake_reference_override:=false'],
        ]
        names += ['teacher_generator', 'teacher_pure_pursuit']
    (args.output/'node_commands.json').write_text(json.dumps(dict(zip(names, commands)), indent=2))
    children = []; streams = []
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        for name, command in zip(names, commands):
            stream = (args.output / (name + ".log")).open("x"); streams.append(stream)
            children.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        start = time.monotonic()
        while time.monotonic() - start < outer_wall_s - 20:
            if any(p.poll() is not None for p in children):
                raise RuntimeError("TRIAL_NODE_EXIT")
            # Keep the controller alive to send braking until the host freezes
            # the owned simulator; heartbeat is inspected independently by host.
            time.sleep(.05)
        raise RuntimeError("TRIAL_NODES_WALL_LIMIT")
    except KeyboardInterrupt:
        pass
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=2)
        for stream in streams:
            stream.close()


if __name__ == "__main__":
    main()
