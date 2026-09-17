cmake_minimum_required(VERSION 3.8)
project(simple_trajectory_generator)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.submission_build")
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(execution_profile SHARED
  src/execution_profile.cpp
)

ament_auto_add_library(dual_reference SHARED
  src/dual_reference.cpp
)

ament_auto_add_executable(simple_trajectory_generator_node
  src/simple_trajectory_generator.cpp
)
target_link_libraries(simple_trajectory_generator_node execution_profile)
target_link_libraries(simple_trajectory_generator_node dual_reference)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_execution_profile test/test_execution_profile.cpp)
  target_link_libraries(test_execution_profile execution_profile)
  ament_target_dependencies(test_execution_profile autoware_auto_planning_msgs)

  ament_add_gtest(test_dual_reference test/test_dual_reference.cpp)
  target_link_libraries(test_dual_reference dual_reference)
  ament_target_dependencies(test_dual_reference autoware_auto_planning_msgs)

  ament_add_gtest(test_race_reference test/test_race_reference.cpp)
  target_include_directories(test_race_reference PRIVATE include)
  target_compile_features(test_race_reference PRIVATE cxx_std_17)
endif()

ament_auto_package(INSTALL_TO_SHARE
  data
)
