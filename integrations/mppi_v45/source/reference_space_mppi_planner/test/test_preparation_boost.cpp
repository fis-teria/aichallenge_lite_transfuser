#include "reference_space_mppi_planner/preparation_boost.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace reference_space_mppi_planner {
namespace {
PreparationBoostPolicy racing(int lap = 1) {
  PreparationBoostPolicy policy;
  policy.observeState("ready");
  policy.observeState("start");
  policy.observeStatus(lap, 2, false);
  return policy;
}
PreparationBoostConfig enabled() {
  PreparationBoostConfig config;
  config.enabled = true;
  return config;
}
}  // namespace

TEST(PreparationBoost, FirstAllocationOnlyOnLapsOneThroughFour) {
  for (int lap = 0; lap <= 8; ++lap) {
    const auto selected = racing(lap).select(enabled(), true, 20.);
    if (lap >= 1 && lap <= 4) {
      ASSERT_TRUE(selected); EXPECT_EQ(*selected, 0U);
    } else if (lap >= 5 && lap <= 6) {
      ASSERT_TRUE(selected); EXPECT_EQ(*selected, 1U);
    } else EXPECT_FALSE(selected);
  }
}

TEST(PreparationBoost, OneUsePerAllocationDespiteRepeatedPlanAndStatus) {
  auto policy = racing();
  const auto config = enabled();
  const auto first = policy.select(config, true, 0.);
  ASSERT_TRUE(first); policy.fired[*first] = true;
  for (int lap = 1; lap <= 4; ++lap) {
    policy.observeStatus(lap, 2, false);  // includes a delayed consumption ACK
    EXPECT_FALSE(policy.select(config, true, 20.));
  }
  policy.observeStatus(5, 1, false);
  const auto second = policy.select(config, true, 60.);
  ASSERT_TRUE(second); EXPECT_EQ(*second, 1U); policy.fired[*second] = true;
  policy.observeStatus(6, 1, false);
  EXPECT_FALSE(policy.select(config, true, 20.));
}

TEST(PreparationBoost, MissingEarlyOpportunityDoesNotSpendTwiceLate) {
  auto policy = racing(4);
  EXPECT_FALSE(policy.select(enabled(), false, 20.));
  policy.observeStatus(5, 2, false);
  const auto late = policy.select(enabled(), true, 20.);
  ASSERT_TRUE(late); EXPECT_EQ(*late, 1U); policy.fired[*late] = true;
  policy.observeStatus(6, 1, false);
  EXPECT_FALSE(policy.select(enabled(), true, 20.));
}

TEST(PreparationBoost, ZoneBoundsAndPreparationAreRequired) {
  const auto policy = racing();
  EXPECT_TRUE(policy.select(enabled(), true, 0.));
  EXPECT_TRUE(policy.select(enabled(), true, 60.));
  EXPECT_FALSE(policy.select(enabled(), true, -.01));
  EXPECT_FALSE(policy.select(enabled(), true, 60.01));
  EXPECT_FALSE(policy.select(enabled(), true, 349.));
  EXPECT_FALSE(policy.select(enabled(), true, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(policy.select(enabled(), false, 20.));
  EXPECT_FALSE(policy.select(PreparationBoostConfig{}, true, 20.));
}

TEST(PreparationBoost, RequiresRaceStatusBudgetAndInactiveBoost) {
  auto policy = racing();
  policy.status_received = false;
  EXPECT_FALSE(policy.select(enabled(), true, 20.));
  policy.observeStatus(1, 0, false);
  EXPECT_FALSE(policy.select(enabled(), true, 20.));
  policy.observeStatus(1, 2, true);
  EXPECT_FALSE(policy.select(enabled(), true, 20.));
  policy.observeStatus(1, 2, false);
  policy.observeState("finish");
  EXPECT_FALSE(policy.select(enabled(), true, 20.));
}

TEST(PreparationBoost, RaceRestartResetsAllocationsAndSupportsStateOrdering) {
  for (const bool start_first : {false, true}) {
    auto policy = racing();
    policy.fired.fill(true);
    policy.observeState("finish");
    policy.observeState("spawned");
    policy.observeState("grounded");
    policy.observeStatus(0, 2, false);
    policy.observeState(start_first ? "start" : "ready");
    EXPECT_FALSE(policy.select(enabled(), true, 20.));
    policy.observeState(start_first ? "ready" : "start");
    EXPECT_FALSE(policy.select(enabled(), true, 20.));  // official lap is still zero
    policy.observeStatus(1, 2, false);
    ASSERT_TRUE(policy.select(enabled(), true, 20.));
    EXPECT_EQ(*policy.select(enabled(), true, 20.), 0U);
  }
}
}  // namespace reference_space_mppi_planner
