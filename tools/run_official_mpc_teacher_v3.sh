#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "usage: $0 CONFIG_YAML REF_VEL_YAML" >&2
    exit 2
fi

config_path="$1"
ref_vel_path="$2"
if [ ! -f "${config_path}" ] || [ ! -f "${ref_vel_path}" ]; then
    echo "teacher MPC config or reference-velocity file is missing" >&2
    exit 2
fi

mpc_prefix="$(ros2 pkg prefix multi_purpose_mpc_ros)"
exec "${mpc_prefix}/lib/multi_purpose_mpc_ros/run_mpc_controller.bash" \
    -c "${config_path}" \
    -r "${ref_vel_path}" \
    --ros-args \
    -r __node:=aic_teacher_mpc \
    -p use_sim_time:=true \
    -p use_obstacle_avoidance:=false \
    -p use_stats:=false \
    -r control/control_mode_request_topic:=/awsim/control_mode_request_topic \
    -r /control/command/control_cmd:=/nominal_control_cmd \
    -r /control/command/control_cmd_raw:=/teacher_control_cmd_raw
