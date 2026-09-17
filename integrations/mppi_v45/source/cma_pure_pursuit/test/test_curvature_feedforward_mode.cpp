#include "simple_pure_pursuit/curvature_feedforward_mode.hpp"

#include <gtest/gtest.h>

namespace spp = simple_pure_pursuit;

TEST(CurvatureFeedforwardMode, EnablesOnlyOwnedPassModes) {
  EXPECT_FALSE(spp::isCurvatureFeedforwardManeuverMode("PREPARE_OVERTAKE_LEFT"));
  EXPECT_FALSE(
      spp::isCurvatureFeedforwardManeuverMode("PREPARE_OVERTAKE_RIGHT"));
  EXPECT_TRUE(spp::isCurvatureFeedforwardManeuverMode("OVERTAKE_LEFT"));
  EXPECT_TRUE(spp::isCurvatureFeedforwardManeuverMode("OVERTAKE_RIGHT"));

  EXPECT_FALSE(spp::isCurvatureFeedforwardManeuverMode("FREE_RUN"));
  EXPECT_TRUE(spp::isCurvatureFeedforwardManeuverMode("SIDE_BY_SIDE_KEEP"));
  EXPECT_FALSE(spp::isCurvatureFeedforwardManeuverMode("YIELD_BEHIND"));
  EXPECT_FALSE(spp::isCurvatureFeedforwardManeuverMode("SAFE_STOP"));
}
