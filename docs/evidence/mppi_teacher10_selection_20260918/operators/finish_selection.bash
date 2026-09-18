#!/usr/bin/env bash
set -euo pipefail
root=/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918
.venv/bin/python -u tools/curate_native_teacher_data.py \
  --root "$root" --output "$root/collect10_curated_v1" \
  --run-pattern 'lidar-v45-pc10-front-*' \
  --prefix-directory collect10_pose_prefix_v1 \
  --clearance-directory collect10_prefix_clearance_v1
.venv/bin/python -u "$root/collect10_operators/replay_selected.py"
