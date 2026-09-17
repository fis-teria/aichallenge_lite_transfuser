#include "simple_pure_pursuit/gear_relative_command.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace spp = simple_pure_pursuit;

TEST(GearRelativeCommand, ConvertsReverseOnlyAtVehicleBoundary)
{
  const auto command =
    spp::applyGearRelativeReverseCommand(-1.0, -0.8, true, true);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.reverse_conversion_applied);
  EXPECT_DOUBLE_EQ(command.speed_mps, 1.0);
  EXPECT_DOUBLE_EQ(command.acceleration_mps2, 0.8);
}

TEST(GearRelativeCommand, PreservesReverseSignWhenContractIsDisabled)
{
  const auto command =
    spp::applyGearRelativeReverseCommand(-1.0, -0.8, false, false);
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.reverse_conversion_applied);
  EXPECT_DOUBLE_EQ(command.speed_mps, -1.0);
  EXPECT_DOUBLE_EQ(command.acceleration_mps2, -0.8);
}

TEST(GearRelativeCommand, DoesNotChangeForwardOrStopCommands)
{
  const auto forward =
    spp::applyGearRelativeReverseCommand(4.0, -1.5, true, false);
  ASSERT_TRUE(forward.valid);
  EXPECT_FALSE(forward.reverse_conversion_applied);
  EXPECT_DOUBLE_EQ(forward.speed_mps, 4.0);
  EXPECT_DOUBLE_EQ(forward.acceleration_mps2, -1.5);

  const auto stop =
    spp::applyGearRelativeReverseCommand(0.0, 0.0, true, false);
  ASSERT_TRUE(stop.valid);
  EXPECT_FALSE(stop.reverse_conversion_applied);
  EXPECT_DOUBLE_EQ(stop.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(stop.acceleration_mps2, 0.0);
}

TEST(GearRelativeCommand, RejectsNonFiniteInput)
{
  const auto command = spp::applyGearRelativeReverseCommand(
    -1.0, std::numeric_limits<double>::quiet_NaN(), true, true);
  EXPECT_FALSE(command.valid);
  EXPECT_FALSE(command.reverse_conversion_applied);
  EXPECT_DOUBLE_EQ(command.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.acceleration_mps2, 0.0);
}

TEST(GearRelativeCommand, ConvertsStoppingAccelerationWhileReverseRemainsSelected)
{
  const auto command =
    spp::applyGearRelativeReverseCommand(0.0, 0.7, true, true);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.reverse_conversion_applied);
  EXPECT_FALSE(command.direction_mismatch);
  EXPECT_DOUBLE_EQ(command.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.acceleration_mps2, -0.7);
}

TEST(GearRelativeCommand, FailsClosedOnTrajectoryAndGearDirectionMismatch)
{
  const auto reverse_before_shift =
    spp::applyGearRelativeReverseCommand(-1.0, -0.8, true, false);
  ASSERT_TRUE(reverse_before_shift.valid);
  EXPECT_TRUE(reverse_before_shift.direction_mismatch);
  EXPECT_DOUBLE_EQ(reverse_before_shift.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(reverse_before_shift.acceleration_mps2, 0.0);

  const auto forward_before_drive =
    spp::applyGearRelativeReverseCommand(1.0, 0.8, true, true);
  ASSERT_TRUE(forward_before_drive.valid);
  EXPECT_TRUE(forward_before_drive.direction_mismatch);
  EXPECT_DOUBLE_EQ(forward_before_drive.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(forward_before_drive.acceleration_mps2, 0.0);
}
