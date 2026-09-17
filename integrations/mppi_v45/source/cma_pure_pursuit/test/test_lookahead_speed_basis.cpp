#include "simple_pure_pursuit/lookahead_speed_basis.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>

namespace
{

TEST(LookaheadSpeedBasis, ManeuverModesUseContinuousMeasuredVehicleSpeed)
{
  constexpr std::array<const char *, 7U> modes{{
    "FOLLOW_BLOCKED", "PREPARE_OVERTAKE_LEFT", "PREPARE_OVERTAKE_RIGHT",
    "OVERTAKE_LEFT", "OVERTAKE_RIGHT", "SIDE_BY_SIDE_KEEP", "MERGE_BACK"}};
  for (const char * mode : modes) {
    EXPECT_DOUBLE_EQ(
      simple_pure_pursuit::selectLookaheadSpeedBasis(0.2, 4.9, true, true, mode),
      4.9) << mode;
    EXPECT_DOUBLE_EQ(
      simple_pure_pursuit::selectLookaheadSpeedBasis(9.72, 4.9, true, true, mode),
      4.9) << mode;
  }
}

TEST(LookaheadSpeedBasis, FreeRunPreservesCmaTargetVelocityContract)
{
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::selectLookaheadSpeedBasis(10.0, 7.5, true, true, "FREE_RUN"),
    10.0);
}

TEST(LookaheadSpeedBasis, DisabledOrStaleModePreservesLegacyBehavior)
{
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::selectLookaheadSpeedBasis(
      9.72, 4.9, false, true, "OVERTAKE_RIGHT"),
    9.72);
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::selectLookaheadSpeedBasis(
      9.72, 4.9, true, false, "OVERTAKE_RIGHT"),
    9.72);
}

TEST(LookaheadSpeedBasis, ManeuverDistanceCannotJumpOnOneControllerTick)
{
  const double applied = simple_pure_pursuit::smoothManeuverLookaheadDistance(
    1.73, 5.43, true, 0.01, 0.20, true, true, "OVERTAKE_RIGHT");
  EXPECT_GT(applied, 5.20);
  EXPECT_LT(applied, 5.43);
}

TEST(LookaheadSpeedBasis, FreeRunAndDisabledSmoothingKeepRawDistance)
{
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::smoothManeuverLookaheadDistance(
      6.0, 2.0, true, 0.01, 0.20, true, true, "FREE_RUN"),
    6.0);
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::smoothManeuverLookaheadDistance(
      1.73, 5.43, true, 0.01, 0.20, false, true, "OVERTAKE_RIGHT"),
    1.73);
}

}  // namespace
