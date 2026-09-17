#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/steering_actuator_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace spp = simple_pure_pursuit;

TEST(DelayCompensation, RealTireRateIsIndependentOfCommandSlewLimit)
{
  const spp::EgoControlState state{0.0, 0.0, 0.0, 6.0};
  for (const double rate : {0.39793506945470714, 0.6}) {
    const auto prediction = spp::predictDelayedPose(
      state, -0.3141592653589793, 0.3141592653589793, 0.5, 0.01, 0.02,
      1.087, 0.3141592653589793, spp::physicalTireSteeringRate(rate, 4.0, 0.60));
    EXPECT_NEAR(prediction.applied_steering_rad,
                -0.3141592653589793 + 0.5 * rate, 1e-12);
    EXPECT_TRUE(prediction.steering_rate_limited);
  }
  EXPECT_DOUBLE_EQ(spp::physicalTireSteeringRate(0.0, 4.0, 0.60), 2.4);
}

TEST(DelayCompensation, BoundsOnlyTheInternalVehicleModelTarget)
{
  constexpr double physical_limit = 0.3665191429188092;
  const spp::EgoControlState state{0.0, 0.0, 0.0, 8.0};
  const auto prediction = spp::predictDelayedPose(
    state, 0.20, 1.90, 0.20, 0.02, 0.30, 2.14, physical_limit, 4.0);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_TRUE(prediction.target_angle_limited);
  EXPECT_DOUBLE_EQ(prediction.requested_target_steering_rad, 1.90);
  EXPECT_DOUBLE_EQ(prediction.bounded_target_steering_rad, physical_limit);
  EXPECT_LE(std::abs(prediction.applied_steering_rad), physical_limit);
  EXPECT_GT(prediction.yaw, 0.0);
}

TEST(DelayCompensation, AppliesPhysicalRateInsidePredictionSteps)
{
  constexpr double physical_limit = 0.3665191429188092;
  constexpr double physical_rate = 0.50;
  const spp::EgoControlState state{0.0, 0.0, 0.0, 8.0};
  const auto prediction = spp::predictDelayedPose(
    state, 0.0, -1.90, 0.20, 0.02, 0.001, 2.14, physical_limit,
    physical_rate);

  EXPECT_TRUE(prediction.target_angle_limited);
  EXPECT_TRUE(prediction.steering_rate_limited);
  EXPECT_NEAR(prediction.applied_steering_rad, -0.10, 1.0e-12);
  EXPECT_GE(prediction.applied_steering_rad, -physical_limit);
  EXPECT_LT(prediction.yaw, 0.0);
}

TEST(DelayCompensation, OversizedOppositeCommandsRemainMirrorSymmetric)
{
  constexpr double physical_limit = 0.3665191429188092;
  const spp::EgoControlState state{0.0, 0.0, 0.0, 7.9};
  const auto positive = spp::predictDelayedPose(
    state, 0.20, 1.97, 0.20, 0.02, 0.30, 2.14, physical_limit, 4.0);
  const auto negative = spp::predictDelayedPose(
    state, -0.20, -1.97, 0.20, 0.02, 0.30, 2.14, physical_limit, 4.0);

  EXPECT_NEAR(positive.yaw, -negative.yaw, 1.0e-12);
  EXPECT_NEAR(positive.x, negative.x, 1.0e-12);
  EXPECT_NEAR(positive.y, -negative.y, 1.0e-12);
  EXPECT_LE(std::abs(positive.yaw), 0.30);
  EXPECT_LE(std::abs(negative.yaw), 0.30);
}
