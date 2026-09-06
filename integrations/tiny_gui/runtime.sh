#!/bin/bash
set -eo pipefail
test "$CONTROL_METHOD" = tiny_lidar_net_guarded
source /autoware/install/setup.bash
source /tiny_install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec ros2 launch aic_tiny_sim_test guarded_tiny.launch.py
