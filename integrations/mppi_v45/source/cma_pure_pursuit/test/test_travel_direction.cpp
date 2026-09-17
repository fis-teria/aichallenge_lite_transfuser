#include "simple_pure_pursuit/travel_direction.hpp"

#include <gtest/gtest.h>

#include <limits>

TEST(TravelDirection, KeepsForwardSteering)
{
  EXPECT_DOUBLE_EQ(simple_pure_pursuit::steeringForTravelDirection(0.25, 1.0), 0.25);
}

TEST(TravelDirection, InvertsSteeringForReverseTrajectory)
{
  EXPECT_DOUBLE_EQ(simple_pure_pursuit::steeringForTravelDirection(0.25, -1.0), -0.25);
}

TEST(TravelDirection, FailsClosedForNonFiniteInput)
{
  EXPECT_DOUBLE_EQ(
    simple_pure_pursuit::steeringForTravelDirection(
      std::numeric_limits<double>::quiet_NaN(), -1.0),
    0.0);
}
