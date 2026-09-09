#!/bin/bash
set -eo pipefail
source /aichallenge/workspace/install/setup.bash
cd /v4
colcon build --base-paths ros2_ws/src/aic_e2e_runtime --packages-select aic_e2e_runtime --build-base build --install-base install
source /v4/install/setup.bash
ros2 pkg executables aic_e2e_runtime
python3 -c 'from aic_e2e_runtime.v4_shadow_node import validate_config; import json,time; c=json.load(open("/v4/live.template.json")); c["envelope"]["authorized_until_unix_s"]=time.time()+300; validate_config(c); print("CONFIG_OK_NO_INFERENCE")'
