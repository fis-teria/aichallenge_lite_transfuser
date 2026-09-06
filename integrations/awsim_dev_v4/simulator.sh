#!/bin/bash
# Task-owned virtual display. The original AWSIM binary/config are read-only.
set -eo pipefail
export LD_LIBRARY_PATH="/xvfb/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
/xvfb/usr/bin/Xvfb :99 -screen 0 640x360x24 -nolisten tcp -ac > /evidence/xvfb.log 2>&1 &
xvfb_pid=$!
trap 'kill -TERM "$xvfb_pid" 2>/dev/null || true' EXIT
for attempt in $(seq 1 50); do
    if test -S /tmp/.X11-unix/X99; then break; fi
    sleep .1
done
test -S /tmp/.X11-unix/X99
source /autoware/install/setup.bash
export DISPLAY=:99
export XDG_CONFIG_HOME=/evidence/playerconfig
mkdir -p "$XDG_CONFIG_HOME"
bash /aichallenge/run_simulator.bash dev > /evidence/awsim.log 2>&1 &
sim_pid=$!
trap 'kill -INT "$sim_pid" 2>/dev/null || true; kill -TERM "$xvfb_pid" 2>/dev/null || true' INT TERM
wait "$sim_pid"
