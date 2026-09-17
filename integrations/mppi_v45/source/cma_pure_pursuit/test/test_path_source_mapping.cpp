#include "simple_pure_pursuit/path_source_mapping.hpp"

#include <gtest/gtest.h>

namespace spp = simple_pure_pursuit;

TEST(PathSourceMapping, SelectsBaselineInputForNormalReferenceSources) {
  EXPECT_EQ(spp::splitTrajectoryInputForPathSource("CMA_REFERENCE_FREE_RUN"),
            spp::SplitTrajectoryInput::kBaseline);
  EXPECT_EQ(spp::splitTrajectoryInputForPathSource("CMA_REFERENCE_CURVE"),
            spp::SplitTrajectoryInput::kBaseline);
  EXPECT_EQ(
      spp::splitTrajectoryInputForPathSource("CMA_REFERENCE_SPEED_CAPPED"),
      spp::SplitTrajectoryInput::kBaseline);
}

TEST(PathSourceMapping, SelectsPlannerManeuverInputForForwardManeuverSources) {
  for (const char *const source : {
           "AVOID_CONNECTOR_CMA",
           "AVOID_REFERENCE",
           "RETURN_REFERENCE",
           "POST_AVOID_REFERENCE",
           "POST_RECOVERY_REFERENCE",
       }) {
    EXPECT_EQ(spp::splitTrajectoryInputForPathSource(source),
              spp::SplitTrajectoryInput::kManeuver)
        << source;
  }
}

TEST(PathSourceMapping, SelectsNoCmaInputForRecoveryAndUnknownSources) {
  for (const char *const source : {
           "RECOVERY_MPC_OWNER",
           "POST_RECOVERY_MPC_OWNER",
           "DISABLED",
           "UNKNOWN",
           "",
       }) {
    EXPECT_EQ(spp::splitTrajectoryInputForPathSource(source),
              spp::SplitTrajectoryInput::kNone)
        << source;
  }
}
