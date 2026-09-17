#include "simple_pure_pursuit/input_freshness.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace spp = simple_pure_pursuit;

TEST(InputFreshness, AcceptsCurrentSampleAtZeroSimTime) {
  EXPECT_TRUE(spp::inputSampleFresh(true, 0.0, 0.0, 0.2));
  EXPECT_TRUE(spp::inputSampleFresh(true, 10.2, 10.0, 0.2));
}

TEST(InputFreshness, RejectsMissingStaleFutureAndInvalidSamples) {
  EXPECT_FALSE(spp::inputSampleFresh(false, 10.0, 10.0, 0.2));
  EXPECT_FALSE(spp::inputSampleFresh(true, 10.21, 10.0, 0.2));
  EXPECT_FALSE(spp::inputSampleFresh(true, 9.9, 10.0, 0.2));
  EXPECT_FALSE(spp::inputSampleFresh(
      true, std::numeric_limits<double>::quiet_NaN(), 10.0, 0.2));
  EXPECT_FALSE(spp::inputSampleFresh(true, 10.0, 10.0, 0.0));
}

TEST(InputFreshness, RejectsOlderDirectTrajectoryGeneration) {
  EXPECT_TRUE(spp::directTrajectoryGenerationAccepted(false, 0U, 0U));
  EXPECT_TRUE(spp::directTrajectoryGenerationAccepted(true, 10U, 10U));
  EXPECT_TRUE(spp::directTrajectoryGenerationAccepted(true, 10U, 11U));
  EXPECT_FALSE(spp::directTrajectoryGenerationAccepted(true, 10U, 9U));
}
