cmake_minimum_required(VERSION 3.8)
project(reference_space_mppi_planner)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.submission_build")
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(driving_fsm STATIC src/driving_fsm.cpp)
target_compile_features(driving_fsm PUBLIC cxx_std_17)
target_include_directories(driving_fsm PUBLIC include)

ament_auto_add_library(reference_space_mppi_core SHARED
  src/batch_optimizer.cpp
  src/longitudinal_planner.cpp
  src/ot_lane_entry_planner.cpp
  src/passing_preparation_planner.cpp
  src/reference_space_mppi.cpp
  src/static_wall_map.cpp)

ament_auto_add_executable(reference_space_mppi_node
  src/reference_space_mppi_node.cpp)
target_link_libraries(reference_space_mppi_node reference_space_mppi_core driving_fsm)

if(BUILD_TESTING)
  ament_auto_add_executable(test_wall_geometry test/test_wall_geometry.cpp)
  target_compile_definitions(test_wall_geometry PRIVATE MPPI_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_wall_geometry reference_space_mppi_core gtest_main gtest)
  add_test(NAME wall_geometry COMMAND test_wall_geometry)
  add_executable(test_awsim_response test/test_awsim_response.cpp)
  target_compile_features(test_awsim_response PRIVATE cxx_std_17)
  target_compile_options(test_awsim_response PRIVATE -UNDEBUG)
  target_include_directories(test_awsim_response PRIVATE include)
  target_link_libraries(test_awsim_response reference_space_mppi_core)
  add_test(NAME awsim_response COMMAND test_awsim_response)
  add_executable(test_execution_revision test/test_execution_revision.cpp)
  target_compile_features(test_execution_revision PRIVATE cxx_std_17)
  target_include_directories(test_execution_revision PRIVATE include)
  add_test(NAME execution_revision COMMAND test_execution_revision)
  add_executable(test_curvature_horizon test/test_curvature_horizon.cpp)
  target_compile_features(test_curvature_horizon PRIVATE cxx_std_17)
  target_include_directories(test_curvature_horizon PRIVATE include)
  target_link_libraries(test_curvature_horizon reference_space_mppi_core)
  add_test(NAME curvature_horizon COMMAND test_curvature_horizon)
  add_executable(test_reference_geometry test/test_reference_geometry.cpp)
  target_compile_features(test_reference_geometry PRIVATE cxx_std_17)
  target_include_directories(test_reference_geometry PRIVATE include)
  ament_target_dependencies(test_reference_geometry cma_pure_pursuit)
  add_test(NAME reference_geometry COMMAND test_reference_geometry
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/h2h_203913_reference_corner.csv")
  add_executable(test_field_cost_split test/test_field_cost_split.cpp)
  target_compile_features(test_field_cost_split PRIVATE cxx_std_17)
  target_include_directories(test_field_cost_split PRIVATE include)
  ament_target_dependencies(test_field_cost_split cma_pure_pursuit)
  add_test(NAME field_cost_split COMMAND test_field_cost_split)
  find_package(ament_cmake_gtest REQUIRED)
  # Exercise the real node's generation/fallback wiring with no ROS spinning.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/reference_space_mppi_node.cpp")
  file(READ "src/reference_space_mppi_node.cpp" node_test_source)
  string(REPLACE "\nprivate:" "\npublic:" node_test_source "${node_test_source}")
  string(REPLACE "int main(" "int unused_node_main(" node_test_source "${node_test_source}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/node_test_access.hpp" "${node_test_source}")
  ament_auto_add_executable(test_node_fallback test/test_node_fallback.cpp)
  target_include_directories(test_node_fallback PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(test_node_fallback reference_space_mppi_core driving_fsm gtest_main gtest)
  add_test(NAME node_fallback COMMAND test_node_fallback)
  ament_add_gtest(test_driving_fsm test/test_driving_fsm.cpp)
  target_link_libraries(test_driving_fsm driving_fsm)
  ament_add_gtest(test_entry_connector test/test_entry_connector.cpp)
  target_include_directories(test_entry_connector PRIVATE include)
  ament_target_dependencies(test_entry_connector cma_pure_pursuit)
  target_link_libraries(test_entry_connector reference_space_mppi_core)
  ament_add_gtest(test_reference_space_mppi
    test/test_preparation_boost.cpp
    test/test_longitudinal_planner.cpp
    test/test_ot_lane_entry.cpp
    test/test_passing_preparation.cpp
    test/test_prior_lap_candidate.cpp
    test/test_base_projection.cpp
    test/test_prior_lap_prediction.cpp
    test/test_leader_lap_prediction.cpp
    test/test_recent_motion_prediction.cpp
    test/test_online_motion_prediction.cpp
    test/test_ordered_history.cpp
    test/test_execution_speed_search.cpp
    test/test_approach_speed.cpp
    test/test_passing_progress.cpp
    test/test_overtake_priority.cpp
    test/test_execution_continuation.cpp
    test/test_cross_shape_sampling.cpp
    test/test_reference_pose_index.cpp
    test/test_reference_space_mppi.cpp)
  target_include_directories(test_reference_space_mppi PRIVATE include)
  ament_target_dependencies(test_reference_space_mppi cma_pure_pursuit)
  target_link_libraries(test_reference_space_mppi reference_space_mppi_core)

  add_executable(test_execution_contract test/test_execution_contract.cpp)
  target_compile_features(test_execution_contract PRIVATE cxx_std_17)
  target_compile_definitions(test_execution_contract PRIVATE MPPI_EXECUTION_CONTRACT_V2)
  target_include_directories(test_execution_contract PRIVATE include)
  ament_target_dependencies(test_execution_contract cma_pure_pursuit)
  target_link_libraries(test_execution_contract reference_space_mppi_core)
  add_test(NAME execution_contract COMMAND test_execution_contract)
  ament_auto_add_executable(test_execution_speed_geometry test/test_execution_speed_geometry.cpp)
  target_link_libraries(test_execution_speed_geometry reference_space_mppi_core)
  add_test(NAME execution_speed_geometry COMMAND test_execution_speed_geometry)
  add_executable(test_prediction_comparison test/test_prediction_comparison.cpp)
  target_compile_features(test_prediction_comparison PRIVATE cxx_std_17)
  target_include_directories(test_prediction_comparison PRIVATE include)
  ament_target_dependencies(test_prediction_comparison cma_pure_pursuit)
  target_link_libraries(test_prediction_comparison reference_space_mppi_core)
  add_test(NAME prediction_comparison COMMAND test_prediction_comparison)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME node_execution_source_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_execution_source_contract.py)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
