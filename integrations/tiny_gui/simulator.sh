#!/bin/bash
# Original AWSIM executable and original dev script, real desktop, read-only assets.
set -eo pipefail
source /autoware/install/setup.bash
export XDG_CONFIG_HOME=/evidence/playerconfig
mkdir -p "$XDG_CONFIG_HOME"
exec bash /aichallenge/run_simulator.bash dev > /evidence/awsim.log 2>&1
