#include "simple_pure_pursuit/lookahead.hpp"

#include <gtest/gtest.h>

namespace spp = simple_pure_pursuit;

TEST(ExecutionBounds, UsesActualControllerIntervalForSteeringRate)
{
  const auto fast = spp::boundSteeringCommand(0.30, 0.0, 0.01, 0.3666, 4.0);
  ASSERT_TRUE(fast.valid);
  EXPECT_NEAR(fast.bounded_angle_rad, 0.04, 1.0e-12);
  EXPECT_TRUE(fast.rate_limited);

  const auto slower = spp::boundSteeringCommand(0.30, 0.0, 0.05, 0.3666, 4.0);
  ASSERT_TRUE(slower.valid);
  EXPECT_NEAR(slower.bounded_angle_rad, 0.20, 1.0e-12);
  EXPECT_TRUE(slower.rate_limited);
}

TEST(ExecutionBounds, ClampsAngleAfterRateWithoutPlannerCycleAssumption)
{
  const auto bounded =
    spp::boundSteeringCommand(1.10, 0.35, 0.02, 0.3665191429188092, 4.0);
  ASSERT_TRUE(bounded.valid);
  EXPECT_LE(std::abs(bounded.bounded_angle_rad), 0.3665191429188092);
  EXPECT_TRUE(bounded.angle_limited);
}

TEST(ExecutionBounds, PassthroughPreservesPlannerTrackingRequest)
{
  const auto command = spp::applySteeringExecutionContract(
    1.10, -0.20, 0.013, 0.3665191429188092, 4.0, true);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.bounded_angle_rad, 1.10);
  EXPECT_DOUBLE_EQ(command.bounded_rate_radps, command.requested_rate_radps);
  EXPECT_FALSE(command.angle_limited);
  EXPECT_FALSE(command.rate_limited);
}

TEST(ExecutionBounds, ProducesNegativeAccelerationForTrajectorySlowdown)
{
  const auto braking = spp::boundLongitudinalAcceleration(
    2.0, 6.0, 1.0, 1.0, 2.0, false, 3.0);
  ASSERT_TRUE(braking.valid);
  EXPECT_DOUBLE_EQ(braking.bounded_mps2, -2.0);
  EXPECT_TRUE(braking.deceleration_limited);
}

TEST(ExecutionBounds, SafeStopUsesDedicatedBrakingLimit)
{
  const auto braking = spp::boundLongitudinalAcceleration(
    0.0, 6.0, 1.0, 1.0, 2.0, true, 3.0);
  ASSERT_TRUE(braking.valid);
  EXPECT_DOUBLE_EQ(braking.bounded_mps2, -3.0);
  EXPECT_TRUE(braking.deceleration_limited);
}

TEST(ExecutionBounds, SafeStopHonorsNegativeTrajectoryBrakeHold)
{
  EXPECT_DOUBLE_EQ(
    spp::applySafeStopTrajectoryDeceleration(0.0, -1.0, true), -1.0);
  EXPECT_DOUBLE_EQ(
    spp::applySafeStopTrajectoryDeceleration(-2.0, -1.0, true), -2.0);
  EXPECT_DOUBLE_EQ(
    spp::applySafeStopTrajectoryDeceleration(0.0, -1.0, false), 0.0);
  EXPECT_DOUBLE_EQ(
    spp::applySafeStopTrajectoryDeceleration(0.0, 1.0, true), 0.0);
}
