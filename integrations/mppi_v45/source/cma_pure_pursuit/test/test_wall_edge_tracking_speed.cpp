#include "simple_pure_pursuit/wall_edge_tracking_speed.hpp"

#include <gtest/gtest.h>

namespace spp = simple_pure_pursuit;

TEST(WallEdgeTrackingSpeed, DoesNotAffectOrdinaryTrajectory) {
  const spp::WallEdgeTrackingSpeedConfig config;
  const auto result = spp::applyWallEdgeTrackingSpeed(
      config, false, 0.12, 0.20, 0.37, 0.0, 2.0, 6.0, 1.0, 1.0);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.speed_mps, 6.0);
  EXPECT_DOUBLE_EQ(result.acceleration_mps2, 1.0);
}

TEST(WallEdgeTrackingSpeed, KeepsFullSpeedInsideDesiredReserve) {
  const spp::WallEdgeTrackingSpeedConfig config;
  const auto result = spp::applyWallEdgeTrackingSpeed(
      config, true, 0.35, 0.20, 0.05, 0.10, 2.0, 6.0, 1.0, 1.0);
  EXPECT_FALSE(result.active);
  EXPECT_NEAR(result.tracking_reserve_consumption_m, 0.07, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.speed_mps, 6.0);
}

TEST(WallEdgeTrackingSpeed, ReducesSpeedWhenTrackingConsumesCorridor) {
  const spp::WallEdgeTrackingSpeedConfig config;
  const auto result = spp::applyWallEdgeTrackingSpeed(
      config, true, 0.23, 0.20, 0.18, 0.10, 4.0, 6.0, 1.0, 1.0);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.tracking_reserve_consumption_m, 0.20, 1.0e-12);
  EXPECT_NEAR(result.usable_corridor_reserve_m, 0.03, 1.0e-12);
  EXPECT_LT(result.speed_mps, 3.0);
  EXPECT_LT(result.acceleration_mps2, 0.0);
}

TEST(WallEdgeTrackingSpeed, AcceleratesTowardCreepCapWhenBelowIt) {
  const spp::WallEdgeTrackingSpeedConfig config;
  const auto result = spp::applyWallEdgeTrackingSpeed(
      config, true, 0.124, 0.20, 0.37, 0.0, 0.0, 1.8, 1.0, 0.3);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.reserve_ratio, 0.0);
  EXPECT_DOUBLE_EQ(result.speed_mps, config.minimum_speed_mps);
  EXPECT_DOUBLE_EQ(result.acceleration_mps2, 0.3);
}

TEST(WallEdgeTrackingSpeed, BrakesTowardCreepCapWhenAboveIt) {
  const spp::WallEdgeTrackingSpeedConfig config;
  const auto result = spp::applyWallEdgeTrackingSpeed(
      config, true, 0.124, 0.20, 0.37, 0.0, 1.8, 1.8, 1.0, 0.3);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.speed_mps, config.minimum_speed_mps);
  EXPECT_NEAR(result.acceleration_mps2, -1.3, 1.0e-12);
}

TEST(WallEdgeTrackingSpeed, DeficitRequiresFreshGeometryAndStableReserve) {
  spp::WallEdgeTrackingSpeedConfig config;
  config.release_duration_sec = 0.15;
  spp::WallEdgeTrackingSpeedState state;

  auto result =
      spp::applyWallEdgeTrackingSpeed(config, true, 0.10, 0.20, 0.20, 0.0, 4.0,
                                      6.0, 1.0, 1.0, 10U, 0.05, &state);
  EXPECT_TRUE(state.deficit_latched);
  EXPECT_DOUBLE_EQ(result.speed_mps, config.minimum_speed_mps);

  // Recovered tracking on the same geometry cannot immediately accelerate.
  result =
      spp::applyWallEdgeTrackingSpeed(config, true, 0.40, 0.20, 0.0, 0.0, 1.0,
                                      6.0, 1.0, 1.0, 10U, 0.10, &state);
  EXPECT_TRUE(state.deficit_latched);
  EXPECT_DOUBLE_EQ(result.speed_mps, config.minimum_speed_mps);

  // A hard-validated replacement must retain reserve for the release period.
  result =
      spp::applyWallEdgeTrackingSpeed(config, true, 0.40, 0.20, 0.0, 0.0, 1.0,
                                      6.0, 1.0, 1.0, 11U, 0.10, &state);
  EXPECT_TRUE(state.deficit_latched);
  result =
      spp::applyWallEdgeTrackingSpeed(config, true, 0.40, 0.20, 0.0, 0.0, 1.0,
                                      6.0, 1.0, 1.0, 11U, 0.05, &state);
  EXPECT_FALSE(state.deficit_latched);
  EXPECT_DOUBLE_EQ(result.speed_mps, 6.0);
}

TEST(DirectionalCorridorTrackingSpeed,
     InwardErrorUsesOpponentReserveAndKeepsRacingSpeed) {
  const spp::WallEdgeTrackingSpeedConfig config;
  spp::WallEdgeTrackingSpeedState state;

  // Left pass: the wall is on the positive side. The observed error is
  // negative, so the kart is inward of the wall-edge path and still has ample
  // opponent-side clearance. This reproduces the first D3 pass in H2H.
  const auto result = spp::applyDirectionalCorridorTrackingSpeed(
      config, true, 1, 0.022, 0.470, 0.20, -0.087, 0.0, 1.2, 10.0, 1.0, 1.0,
      20U, 0.05, &state);

  EXPECT_FALSE(state.deficit_latched);
  EXPECT_DOUBLE_EQ(result.wall_tracking_reserve_consumption_m, 0.0);
  EXPECT_NEAR(result.opponent_tracking_reserve_consumption_m, 0.087, 1.0e-12);
  EXPECT_NEAR(result.usable_wall_reserve_m, 0.022, 1.0e-12);
  EXPECT_NEAR(result.usable_opponent_reserve_m, 0.383, 1.0e-12);
  EXPECT_GT(result.speed_mps, 9.0);
}

TEST(DirectionalCorridorTrackingSpeed, IsSymmetricForRightPass) {
  const spp::WallEdgeTrackingSpeedConfig config;
  spp::WallEdgeTrackingSpeedState left_state;
  spp::WallEdgeTrackingSpeedState right_state;

  const auto left = spp::applyDirectionalCorridorTrackingSpeed(
      config, true, 1, 0.08, 0.45, 0.20, -0.10, 0.05, 3.0, 10.0, 1.0, 1.0, 30U,
      0.05, &left_state);
  const auto right = spp::applyDirectionalCorridorTrackingSpeed(
      config, true, -1, 0.08, 0.45, 0.20, 0.10, -0.05, 3.0, 10.0, 1.0, 1.0, 30U,
      0.05, &right_state);

  EXPECT_NEAR(left.speed_mps, right.speed_mps, 1.0e-12);
  EXPECT_NEAR(left.usable_wall_reserve_m, right.usable_wall_reserve_m, 1.0e-12);
  EXPECT_NEAR(left.usable_opponent_reserve_m, right.usable_opponent_reserve_m,
              1.0e-12);
}

TEST(DirectionalCorridorTrackingSpeed,
     PredictedHardBoundaryCrossingStillLatchesMinimumSpeed) {
  const spp::WallEdgeTrackingSpeedConfig config;
  spp::WallEdgeTrackingSpeedState state;

  const auto result = spp::applyDirectionalCorridorTrackingSpeed(
      config, true, 1, 0.05, 0.45, 0.20, 0.01, 0.25, 4.0, 10.0, 1.0, 1.0, 40U,
      0.05, &state);

  EXPECT_TRUE(state.deficit_latched);
  EXPECT_NEAR(result.projected_lateral_error_m, 0.06, 1.0e-12);
  EXPECT_LT(result.usable_wall_reserve_m, 0.0);
  EXPECT_DOUBLE_EQ(result.speed_mps, config.minimum_speed_mps);
}
