#!/bin/bash
set -eo pipefail
source /autoware/install/setup.bash
export PYTHONPATH="/v4/src:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec python3 /v4/tools/run_tiny_lidar_dev.py --config /evidence/resolved_config.json \
    --output /evidence --project "$TINY_PROJECT" --authorize-sim-session
