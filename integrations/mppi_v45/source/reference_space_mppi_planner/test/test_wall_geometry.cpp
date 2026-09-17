#include "reference_space_mppi_planner/static_wall_map.hpp"
#include <rclcpp/parameter_map.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <random>

using namespace reference_space_mppi_planner;

namespace {
nav_msgs::msg::OccupancyGrid mapWithWall() {
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = .1F;
  map.info.width = map.info.height = 100;
  map.info.origin.orientation.w = 1.;
  map.data.assign(10000, 0);
  for (int y = 0; y < 100; ++y) map.data[y * 100 + 50] = 100;
  return map;
}
ConvexWallFootprint installedBody() {
  const auto params = rclcpp::parameter_map_from_yaml_file(
      MPPI_SOURCE_DIR "/config/awsim_wall_map/footprint.param.yaml");
  for (const auto &node : params)
    for (const auto &p : node.second)
      if (p.get_name() == "brain.wall_footprint_xy_m")
        return ConvexWallFootprint(p.as_double_array());
  throw std::runtime_error("missing physical footprint parameter");
}
}

TEST(ConvexWallGeometry, FrontIsMeasuredFromBaseLinkAndCornersAreNotAnAabb) {
  auto map = mapWithWall();
  const ConvexWallFootprint body({-.38, 0., .2, -.77, 1.615, 0., .2, .77});
  EXPECT_TRUE(occupancyGridFootprintFree(map, {3.5, 5., 0.}, 1.06, 1.10, .65));
  EXPECT_FALSE(occupancyGridFootprintFree(map, {3.5, 5., 0.}, body));
  std::fill(map.data.begin(), map.data.end(), 0);
  map.data[57 * 100 + 49] = 100;
  EXPECT_FALSE(occupancyGridFootprintFree(map, {3.5, 5., 0.}, 1.615, .38, .77));
  EXPECT_TRUE(occupancyGridFootprintFree(map, {3.5, 5., 0.}, body));
}

TEST(ConvexWallGeometry, RectangleCompatibilityAndIndexAcrossRotatedMaps) {
  const ConvexWallFootprint body({1.06, .65, -1.10, .65, -1.10, -.65, 1.06, -.65});
  std::mt19937 rng(20260908);
  std::uniform_real_distribution<double> xy(-1., 11.), yaw(-M_PI, M_PI);
  for (double rotation : {0., .37, M_PI_2}) {
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(mapWithWall());
    map->info.origin.position.x = 89600.; map->info.origin.position.y = 43100.;
    map->info.origin.orientation.z = std::sin(rotation / 2.);
    map->info.origin.orientation.w = std::cos(rotation / 2.);
    map->data[2020] = -1;
    const OccupancyGridWallIndex index(map);
    for (int i = 0; i < 2000; ++i) {
      const double x = xy(rng), y = xy(rng);
      const WallFootprintPose pose{89600. + std::cos(rotation)*x - std::sin(rotation)*y,
          43100. + std::sin(rotation)*x + std::cos(rotation)*y, yaw(rng)};
      const bool expected = occupancyGridFootprintFree(*map, pose, 1.06, 1.10, .65);
      EXPECT_EQ(expected, occupancyGridFootprintFree(*map, pose, body));
      EXPECT_EQ(expected, occupancyGridFootprintFree(*map, pose, body, &index));
    }
  }
}

TEST(ConvexWallGeometry, SweepUnknownAndChangedSnapshot) {
  const ConvexWallFootprint body({.2, .1, -.2, .1, -.2, -.1, .2, -.1});
  auto old_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(mapWithWall());
  const OccupancyGridWallIndex old_index(old_map);
  EXPECT_FALSE(occupancyGridFootprintPathFree(*old_map,
      {{4., 5., 0.}, {6., 5., 0.}}, body, .05, &old_index));
  EXPECT_TRUE(occupancyGridFootprintPathFree(*old_map,
      {{3., 3., 3.1}, {3., 4., -3.1}}, body, .05, &old_index));
  auto changed = *old_map;
  changed.data[3030] = -1;
  EXPECT_FALSE(occupancyGridFootprintFree(changed, {3.05, 3.05, 0.}, body, &old_index));
  EXPECT_FALSE(occupancyGridFootprintFree(changed, {.1, 3., 0.}, body));
}

TEST(ConvexWallGeometry, TouchingCellBoundaryIsBlocked) {
  auto map = mapWithWall();
  map.info.resolution = .125F;
  std::fill(map.data.begin(), map.data.end(), 0);
  const ConvexWallFootprint body({.5, .5, -.5, .5, -.5, -.5, .5, -.5});
  map.data[28 * 100 + 27] = 100;  // Cell ends exactly at footprint x=3.5.
  EXPECT_FALSE(occupancyGridFootprintFree(map, {4., 4., 0.}, body));
}

TEST(ConvexWallGeometry, InvalidShapesDoNotSilentlyChangeTheFootprint) {
  EXPECT_THROW(ConvexWallFootprint({0., 0., 1., 0., .5, 0.}), std::invalid_argument);
  EXPECT_THROW(ConvexWallFootprint({0., 0., 1., 0., .2, .2, 1., 1., 0., 1.}), std::invalid_argument);
  EXPECT_THROW(ConvexWallFootprint({0., 0., 1., 0., 1., 0., 0., 1.}), std::invalid_argument);
}

TEST(ConvexWallGeometry, GeneratedMapDetectsRecordedContactPoses) {
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  std::string error;
  ASSERT_TRUE(loadStaticWallMap(MPPI_SOURCE_DIR "/config/awsim_wall_map/occupancy_grid_map.yaml",
                               "map", map.get(), &error)) << error;
  const auto body = installedBody();
  ASSERT_EQ(body.vertices().size(), 33U);
  const OccupancyGridWallIndex index(map);
  // Measured odometry, interpolated at ROS 114.100 and 266.600 seconds.
  for (const auto &pose : std::vector<WallFootprintPose>{
       {89666.42588028480532, 43167.37269037172518, -5.33913452223986},
       {89677.36652696040983, 43159.47341008293733, -13.89098965285308}}) {
    EXPECT_FALSE(occupancyGridFootprintFree(*map, pose, body));
    EXPECT_FALSE(occupancyGridFootprintFree(*map, pose, body, &index));
  }
  for (const auto &pose : std::vector<WallFootprintPose>{
       {89664.45731306068774, 43164.53160261804442, -5.45882395044751},
       {89675.18839616906189, 43164.23704828084010, -13.58124795328675}})
    EXPECT_TRUE(occupancyGridFootprintFree(*map, pose, body, &index));
}
