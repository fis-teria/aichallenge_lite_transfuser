cmake_minimum_required(VERSION 3.8)
project(cma_pure_pursuit)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.submission_build")
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
  cma_pure_pursuit
  src/simple_pure_pursuit.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_reference_delay_ros test/test_reference_delay_ros.py
    TIMEOUT 60 APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_gtest(test_reference_delay_fallback test/test_reference_delay_fallback.cpp)
  ament_add_gtest(test_reference_return test/test_reference_return.cpp)
  target_include_directories(test_reference_return PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_reference_return autoware_auto_planning_msgs geometry_msgs)
  target_include_directories(test_reference_delay_fallback PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_direct_driving_mode test/test_direct_driving_mode.cpp)
  target_include_directories(test_direct_driving_mode PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_curvature_feedforward_mode
    test/test_curvature_feedforward_mode.cpp)
  target_include_directories(test_curvature_feedforward_mode PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_curvature_feedforward
    test/test_curvature_feedforward.cpp)
  target_include_directories(test_curvature_feedforward PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_lookahead_speed_basis
    test/test_lookahead_speed_basis.cpp)
  target_include_directories(test_lookahead_speed_basis PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_execution_bounds
    test/test_execution_bounds.cpp)
  target_include_directories(test_execution_bounds PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_execution_bounds
    autoware_auto_planning_msgs
    geometry_msgs)
  ament_add_gtest(test_delay_compensation
    test/test_delay_compensation.cpp)
  target_include_directories(test_delay_compensation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_travel_direction
    test/test_travel_direction.cpp)
  target_include_directories(test_travel_direction PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_gear_relative_command
    test/test_gear_relative_command.cpp)
  target_include_directories(test_gear_relative_command PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_input_freshness
    test/test_input_freshness.cpp)
  target_include_directories(test_input_freshness PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_path_source_mapping
    test/test_path_source_mapping.cpp)
  target_include_directories(test_path_source_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_steering_tracking_speed_gate
    test/test_steering_tracking_speed_gate.cpp)
  target_include_directories(test_steering_tracking_speed_gate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_wall_edge_tracking_speed
    test/test_wall_edge_tracking_speed.cpp)
  target_include_directories(test_wall_edge_tracking_speed PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_pure_pursuit_core
    test/test_pure_pursuit_core.cpp)
  target_include_directories(test_pure_pursuit_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_rotation_recovery
    test/test_rotation_recovery.cpp)
  target_include_directories(test_rotation_recovery PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_steering_actuator_model
    test/test_steering_actuator_model.cpp)
  target_include_directories(test_steering_actuator_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
