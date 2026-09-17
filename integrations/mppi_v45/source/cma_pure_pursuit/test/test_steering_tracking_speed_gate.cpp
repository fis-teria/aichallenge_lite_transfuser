#include "simple_pure_pursuit/steering_tracking_speed_gate.hpp"

#include <gtest/gtest.h>

namespace spp = simple_pure_pursuit;

TEST(SteeringTrackingSpeedGate, RequiresSustainedFreshManeuverError) {
  spp::SteeringTrackingSpeedGateConfig config;
  config.enabled = true;
  config.entry_duration_sec = 0.10;
  spp::SteeringTrackingSpeedGateState state;

  for (int cycle = 0; cycle < 9; ++cycle) {
    const auto result = spp::applySteeringTrackingSpeedGate(
        config, true, true, 0.40, 0.05, 6.0, 8.0, 1.0, 0.01, &state);
    EXPECT_FALSE(result.active);
    EXPECT_DOUBLE_EQ(result.speed_mps, 8.0);
  }
  const auto active = spp::applySteeringTrackingSpeedGate(
      config, true, true, 0.40, 0.05, 6.0, 8.0, 1.0, 0.01, &state);
  EXPECT_TRUE(active.active);
  EXPECT_DOUBLE_EQ(active.speed_mps, 6.0);
  EXPECT_DOUBLE_EQ(active.acceleration_mps2, -1.0);
}

TEST(SteeringTrackingSpeedGate, DoesNotAffectFreeRunOrStaleMeasurement) {
  spp::SteeringTrackingSpeedGateConfig config;
  config.enabled = true;
  config.entry_duration_sec = 0.0;
  spp::SteeringTrackingSpeedGateState state;

  const auto free_run = spp::applySteeringTrackingSpeedGate(
      config, false, true, 0.50, 0.0, 6.0, 8.0, 1.0, 0.01, &state);
  EXPECT_FALSE(free_run.active);
  EXPECT_DOUBLE_EQ(free_run.speed_mps, 8.0);

  const auto stale = spp::applySteeringTrackingSpeedGate(
      config, true, false, 0.50, 0.0, 6.0, 8.0, 1.0, 0.01, &state);
  EXPECT_FALSE(stale.active);
  EXPECT_DOUBLE_EQ(stale.acceleration_mps2, 1.0);
}

TEST(SteeringTrackingSpeedGate, ReleasesOnlyAfterSustainedSmallError) {
  spp::SteeringTrackingSpeedGateConfig config;
  config.enabled = true;
  config.entry_duration_sec = 0.0;
  config.release_duration_sec = 0.03;
  spp::SteeringTrackingSpeedGateState state;

  EXPECT_TRUE(spp::applySteeringTrackingSpeedGate(config, true, true, 0.40, 0.0,
                                                  6.0, 8.0, 0.5, 0.01, &state)
                  .active);
  for (int cycle = 0; cycle < 2; ++cycle) {
    EXPECT_TRUE(spp::applySteeringTrackingSpeedGate(
                    config, true, true, 0.05, 0.0, 6.0, 8.0, 0.5, 0.01, &state)
                    .active);
  }
  const auto released = spp::applySteeringTrackingSpeedGate(
      config, true, true, 0.05, 0.0, 6.0, 8.0, 0.5, 0.01, &state);
  EXPECT_FALSE(released.active);
  EXPECT_DOUBLE_EQ(released.speed_mps, 8.0);
}
