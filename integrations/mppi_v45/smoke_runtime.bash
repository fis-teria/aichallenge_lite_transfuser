#!/usr/bin/env bash
# Run with Docker --network none and no AWSIM; validates wiring before driving.
set -eo pipefail
source /aichallenge/workspace/install/setup.bash
source /runtime/install/local_setup.bash
export PYTHONPATH="/source/integrations/mppi_v45/support:${PYTHONPATH:-}"
export QT_QPA_PLATFORM=offscreen
export CYCLONEDDS_URI=file:///source/integrations/mppi_v45/cyclonedds.xml
export ROS_DOMAIN_ID=97
export TEACHER_SPEED_CAP_MPS=1.3888888888888888
export ROS_HOME=/tmp/ros
ros2 launch aic_lidar_v2x teacher_v45.launch.py \
  map_yaml:=/source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml \
  domain_id:=97 speed_cap_mps:="$TEACHER_SPEED_CAP_MPS" run_rviz:=false > /tmp/teacher.log 2>&1 &
teacher_pid=$!
trap 'kill -INT "$teacher_pid" 2>/dev/null || true' EXIT
set +e
python3 /source/integrations/mppi_v45/check_runtime.py
result=$?
cat /tmp/teacher.log
exit "$result"
