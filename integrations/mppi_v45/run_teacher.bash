#!/usr/bin/env bash
set -eo pipefail
source /aichallenge/workspace/install/setup.bash
source /runtime/install/local_setup.bash
test "$(ros2 pkg prefix reference_space_mppi_planner)" = /runtime/install/reference_space_mppi_planner
test "$(ros2 pkg prefix aic_lidar_v2x)" = /runtime/install/aic_lidar_v2x
export ROS_HOME="${LOG_DIR:?}/d1/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "$ROS_LOG_DIR"
cd "${LOG_DIR}/d1"
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
root=Path('/runtime')
identity=json.loads((root/'runtime-identity.json').read_text())
assert identity['teacher']=='MPPI_SIM_V45'
for entry in identity['files'].values():
    assert hashlib.sha256((root/'install'/entry['path']).read_bytes()).hexdigest()==entry['sha256']
identity.update(speed_cap_mps=float(os.environ['TEACHER_SPEED_CAP_MPS']),perception='LIDAR_V2X_SURFACE',student_control=False)
Path('teacher-runtime-identity.json').write_text(json.dumps(identity,indent=2)+'\n')
PY
exec ros2 launch aic_lidar_v2x teacher_v45.launch.py \
  map_yaml:=/source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml \
  domain_id:=1 speed_cap_mps:="${TEACHER_SPEED_CAP_MPS:?}" run_rviz:=false
