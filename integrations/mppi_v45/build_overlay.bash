#!/usr/bin/env bash
# Dedicated container paths: /source (read-only), /runtime, /teacher-build.
set -eo pipefail
source /aichallenge/workspace/install/setup.bash
export ROS_LOG_DIR=/runtime/build_ros_log
export CYCLONEDDS_URI=file:///source/integrations/mppi_v45/cyclonedds.xml
export ROS_DOMAIN_ID=224
export MAKEFLAGS=-j2
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MPPI_REFERENCE_BENCHMARK_CSV=/source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/mppi_cma_normal35_20260913.csv
mkdir -p "$ROS_LOG_DIR"
cd /teacher-build
colcon --log-base /runtime/build_log build \
  --base-paths /source/integrations/mppi_v45/source /source/ros2_ws/src/aic_lidar_v2x \
  --build-base /teacher-build/build --install-base /runtime/install \
  --packages-select multi_purpose_mpc_ros_msgs cma_pure_pursuit reference_space_mppi_planner \
    mppi_recovery_controller aichallenge_submit_launch simple_trajectory_generator aic_lidar_v2x \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
set +e
colcon --log-base /runtime/test_log test \
  --base-paths /source/integrations/mppi_v45/source \
  --build-base /teacher-build/build --install-base /runtime/install \
  --packages-select reference_space_mppi_planner --executor sequential --return-code-on-test-failure
test_status=$?
colcon test-result --test-result-base /teacher-build/build --verbose > /runtime/test-results.txt
mkdir -p /runtime/test_evidence
cp -a /teacher-build/build/reference_space_mppi_planner/Testing \
  /teacher-build/build/reference_space_mppi_planner/test_results /runtime/test_evidence/
exit "$test_status"
