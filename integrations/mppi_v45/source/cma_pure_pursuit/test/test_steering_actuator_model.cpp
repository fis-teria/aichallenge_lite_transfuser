#include "simple_pure_pursuit/steering_actuator_model.hpp"

#include <gtest/gtest.h>

namespace simple_pure_pursuit {
namespace {

TEST(SteeringActuatorModel, ConvertsCommandAndTireDomainsSymmetrically) {
  constexpr double ratio = 0.60;
  constexpr double command_limit = 0.5235987755982988;
  constexpr double tire_limit = 0.3141592653589793;

  EXPECT_NEAR(commandToTireSteeringAngle(command_limit, ratio, tire_limit),
              tire_limit, 1.0e-12);
  EXPECT_NEAR(commandToTireSteeringAngle(-command_limit, ratio, tire_limit),
              -tire_limit, 1.0e-12);
  EXPECT_NEAR(tireToCommandSteeringAngle(tire_limit, ratio, command_limit),
              command_limit, 1.0e-12);
  EXPECT_NEAR(tireToCommandSteeringAngle(-tire_limit, ratio, command_limit),
              -command_limit, 1.0e-12);
}

TEST(SteeringActuatorModel, HoldsOnlyPositiveAccelerationAtLargeDemand) {
  const auto saturated = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, true, 0.30, 0.22, 0.15, 0.08, 2.0);
  const auto lagging = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, false, true, -0.28, -0.18, 0.15, 0.08, 1.5);
  const auto braking = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, true, 0.30, 0.10, 0.15, 0.08, -1.0);
  const auto straight = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, true, 0.05, 0.0, 0.15, 0.02, 2.0);
  const auto stopped = holdPositiveAccelerationForSteeringDemand(
      true, 0.0, 3.0, true, true, 0.30, 0.30, 0.15, 0.08, 2.0);

  ASSERT_TRUE(saturated.valid);
  EXPECT_TRUE(saturated.active);
  EXPECT_DOUBLE_EQ(saturated.acceleration_mps2, 0.0);
  EXPECT_TRUE(lagging.active);
  EXPECT_DOUBLE_EQ(lagging.acceleration_mps2, 0.0);
  EXPECT_FALSE(braking.active);
  EXPECT_DOUBLE_EQ(braking.acceleration_mps2, -1.0);
  EXPECT_FALSE(straight.active);
  EXPECT_DOUBLE_EQ(straight.acceleration_mps2, 2.0);
  EXPECT_FALSE(stopped.active);
  EXPECT_DOUBLE_EQ(stopped.acceleration_mps2, 2.0);
}

TEST(SteeringActuatorModel, GentleAccelerationFollowsSteeringRecoveryAndRelapse) {
  for (const double turn_sign : {-1.0, 1.0}) {
    const double errors[] = {0.18, 0.12, 0.04, 0.18};
    const double expected[] = {0.0, 0.3, 0.6, 0.0};
    for (int index = 0; index < 4; ++index) {
      const auto result = holdPositiveAccelerationForSteeringDemand(
          true, 6.0, 3.0, true, true, turn_sign * 0.30,
          turn_sign * (0.30 - errors[index]), 0.15, 0.08, 2.0, 0.6);
      ASSERT_TRUE(result.valid);
      EXPECT_TRUE(result.active);
      EXPECT_NEAR(result.acceleration_mps2, expected[index], 1.0e-12);
    }
  }
  const auto unsaturated = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, false, true, 0.28, 0.16, 0.15, 0.08, 2.0, 0.6);
  EXPECT_TRUE(unsaturated.active);
  EXPECT_NEAR(unsaturated.acceleration_mps2, 0.3, 1.0e-12);
}

TEST(SteeringActuatorModel, GentleAllowanceNeverRaisesOriginalCommand) {
  for (const double requested : {-1.0, 0.0, 0.25}) {
    const auto result = holdPositiveAccelerationForSteeringDemand(
        true, 6.0, 3.0, true, true, 0.30, 0.26, 0.15, 0.08,
        requested, 0.6);
    ASSERT_TRUE(result.valid);
    EXPECT_DOUBLE_EQ(result.acceleration_mps2, requested);
  }
  const auto stale = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, false, 0.30, 0.26, 0.15, 0.08, 2.0, 0.6);
  EXPECT_TRUE(stale.active);
  EXPECT_DOUBLE_EQ(stale.acceleration_mps2, 0.0);
  const auto disabled = holdPositiveAccelerationForSteeringDemand(
      false, 6.0, 3.0, true, true, 0.30, 0.26, 0.15, 0.08, 2.0, 0.6);
  EXPECT_FALSE(disabled.active);
  EXPECT_DOUBLE_EQ(disabled.acceleration_mps2, 2.0);
}

TEST(SteeringActuatorModel, ZeroTrackingThresholdRequiresExactTracking) {
  const auto tracked = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, true, 0.30, 0.30, 0.15, 0.0, 2.0, 0.6);
  const auto lagging = holdPositiveAccelerationForSteeringDemand(
      true, 6.0, 3.0, true, true, 0.30, 0.29, 0.15, 0.0, 2.0, 0.6);
  EXPECT_DOUBLE_EQ(tracked.acceleration_mps2, 0.6);
  EXPECT_DOUBLE_EQ(lagging.acceleration_mps2, 0.0);
}

} // namespace
} // namespace simple_pure_pursuit
