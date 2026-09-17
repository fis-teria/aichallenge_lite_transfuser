#include "simple_trajectory_generator/race_reference.hpp"
#include <gtest/gtest.h>

using simple_trajectory_generator::RaceProgressRanking;
using simple_trajectory_generator::RaceReference;
using simple_trajectory_generator::selectRaceReference;

TEST(RaceReference, FirstLapAlwaysUsesOriginalRoute) {
  for (int lap : {0, 1}) {
    EXPECT_EQ(selectRaceReference(lap, 1), RaceReference::kLap1);
    EXPECT_EQ(selectRaceReference(lap, 4), RaceReference::kLap1);
    EXPECT_EQ(selectRaceReference(lap, std::nullopt), RaceReference::kLap1);
  }
}

TEST(RaceReference, LaterLapsFollowRankInBothDirections) {
  for (int lap : {2, 3, 4}) {
    EXPECT_EQ(selectRaceReference(lap, 2), RaceReference::kNormal);
    EXPECT_EQ(selectRaceReference(lap, 1), RaceReference::kLeader);
    EXPECT_EQ(selectRaceReference(lap, 2), RaceReference::kNormal);
    EXPECT_EQ(selectRaceReference(lap, std::nullopt), RaceReference::kNormal);
  }
}

TEST(RaceReference, GridAcrossSeamAndOvertake) {
  RaceProgressRanking ranking(100.0);
  ranking.update("ego", 98.0, 1.0, 98.0);
  ranking.update("other", 2.0, 1.0, 98.0);
  EXPECT_EQ(ranking.rank("ego", 1.0, 0.5), 2);
  EXPECT_EQ(ranking.leader(1.0,0.5),"other");
  ranking.update("ego", 4.0, 2.0, 98.0);
  ranking.update("other", 3.0, 2.0, 98.0);
  EXPECT_EQ(ranking.rank("ego", 2.0, 0.5), 1);
  EXPECT_EQ(ranking.leader(2.0,0.5),"ego");
  EXPECT_FALSE(ranking.leader(2.6,0.5));
  ranking.update("other", 6.0, 2.1, 98.0);
  EXPECT_EQ(ranking.rank("ego", 2.1, 0.5), 2);
}

TEST(RaceReference, LappedCarAheadOnRoadDoesNotBecomeLeader) {
  RaceProgressRanking ranking(100.0);
  ranking.update("ego", 0.0, 1.0, 0.0);
  ranking.update("other", 10.0, 1.0, 0.0);
  for (int step = 1; step <= 11; ++step) {
    const double stamp = 1.0 + step;
    ranking.update("ego", std::fmod(step * 10.0, 100.0), stamp, 0.0);
    ranking.update("other", 20.0, stamp, 0.0);
  }
  EXPECT_EQ(ranking.rank("ego", 12.0, 0.5), 1);
}

TEST(RaceReference, StaleOrMissingObservationCannotAssertFirstPlace) {
  RaceProgressRanking ranking(100.0);
  EXPECT_FALSE(ranking.rank("ego", 1.0, 0.5));
  ranking.update("ego", 20.0, 1.0, 20.0);
  ranking.update("other", 10.0, 1.0, 20.0);
  ranking.update("ego", 25.0, 2.0, 20.0);
  EXPECT_FALSE(ranking.rank("ego", 2.0, 0.5));
  ranking.update("other", 15.0, 2.0, 20.0);
  EXPECT_EQ(ranking.rank("ego", 2.0, 0.5), 1);
}

TEST(RaceReference, DelayedPacketDoesNotUndoOvertake) {
  RaceProgressRanking ranking(100.0);
  ranking.update("ego", 20.0, 2.0, 0.0);
  ranking.update("other", 10.0, 2.0, 0.0);
  ranking.update("other", 30.0, 1.0, 0.0);
  EXPECT_EQ(ranking.rank("ego", 2.0, 0.5), 1);
}

TEST(RaceReference, ResetDropsPreviousRaceProgress) {
  RaceProgressRanking ranking(100.0);
  ranking.update("ego", 20.0, 20.0, 0.0);
  ranking.update("other", 10.0, 20.0, 0.0);
  ranking.reset();
  EXPECT_FALSE(ranking.rank("ego", 1.0, 0.5));
  ranking.update("ego", 0.0, 1.0, 0.0);
  ranking.update("other", 10.0, 1.0, 0.0);
  EXPECT_EQ(ranking.rank("ego", 1.0, 0.5), 2);
}
