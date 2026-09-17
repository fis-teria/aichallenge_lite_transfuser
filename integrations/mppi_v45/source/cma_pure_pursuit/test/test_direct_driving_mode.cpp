#include "simple_pure_pursuit/direct_driving_mode.hpp"
#include "simple_pure_pursuit/curvature_feedforward_mode.hpp"
#include "simple_pure_pursuit/lookahead_speed_basis.hpp"
#include <gtest/gtest.h>

TEST(DirectDrivingMode, AvoidPreservesLeftRightAndReturnTrackingPolicies) {
  using namespace simple_pure_pursuit;
  for (int side : {-1, 1}) {
    const auto mode = directTrackingMode("AVOID", side);
    EXPECT_TRUE(isMeasuredSpeedLookaheadMode(mode));
    EXPECT_TRUE(isCurvatureFeedforwardManeuverMode(mode));
  }
  const auto returning = directTrackingMode("AVOID", 0);
  EXPECT_TRUE(isMeasuredSpeedLookaheadMode(returning));
  EXPECT_FALSE(isCurvatureFeedforwardManeuverMode(returning));
  EXPECT_EQ(directTrackingMode("FREE_RUN", 1), "FREE_RUN");
  EXPECT_EQ(directTrackingMode("SAFE_STOP", -1), "SAFE_STOP");
}

TEST(DirectDrivingMode, OvertakeUsesTheSamePhysicalTrackingPolicies) {
  using namespace simple_pure_pursuit;
  EXPECT_EQ(directTrackingMode("OVERTAKE",1),"OVERTAKE_LEFT");
  EXPECT_EQ(directTrackingMode("OVERTAKE",-1),"OVERTAKE_RIGHT");
  EXPECT_EQ(directTrackingMode("OVERTAKE",0),"MERGE_BACK");
  EXPECT_TRUE(isMeasuredSpeedLookaheadMode(directTrackingMode("OVERTAKE",1)));
  EXPECT_TRUE(isCurvatureFeedforwardManeuverMode(directTrackingMode("OVERTAKE",-1)));
}
