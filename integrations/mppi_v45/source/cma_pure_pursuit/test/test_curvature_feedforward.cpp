#include "simple_pure_pursuit/curvature_feedforward.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

namespace
{

using Point = std::pair<double, double>;

double previewCurvature(const std::vector<Point> & points, double distance)
{
  return simple_pure_pursuit::estimateDistanceWindowSignedCurvature(
    points, 0U, distance, [](const Point & point) {return point;});
}

std::vector<Point> leftArc(double radius_m, double step_rad, int count)
{
  std::vector<Point> points;
  for (int i = 0; i < count; ++i) {
    const double angle = step_rad * static_cast<double>(i);
    points.emplace_back(radius_m * std::sin(angle), radius_m * (1.0 - std::cos(angle)));
  }
  return points;
}

TEST(CurvatureFeedforward, DistanceWindowIsIndependentOfPointSamplingDensity)
{
  const auto coarse = leftArc(10.0, 0.04, 40);
  const auto fine = leftArc(10.0, 0.01, 160);
  EXPECT_NEAR(previewCurvature(coarse, 3.0), 0.1, 2.0e-3);
  EXPECT_NEAR(previewCurvature(fine, 3.0), 0.1, 2.0e-3);
  EXPECT_NEAR(previewCurvature(coarse, 3.0), previewCurvature(fine, 3.0), 2.0e-3);
}

TEST(CurvatureFeedforward, TimeSmoothingUsesControllerElapsedTime)
{
  double at_100_hz = 0.0;
  for (int i = 0; i < 100; ++i) {
    at_100_hz = simple_pure_pursuit::timeDomainLowPass(1.0, at_100_hz, 0.01, 0.20);
  }
  double at_50_hz = 0.0;
  for (int i = 0; i < 50; ++i) {
    at_50_hz = simple_pure_pursuit::timeDomainLowPass(1.0, at_50_hz, 0.02, 0.20);
  }
  EXPECT_NEAR(at_100_hz, at_50_hz, 1.0e-12);
  EXPECT_GT(at_100_hz, 0.99);
}

TEST(CurvatureFeedforward, OneOppositeSampleDoesNotFlipTheFilteredSign)
{
  const double filtered =
    simple_pure_pursuit::timeDomainLowPass(-0.10, 0.10, 0.01, 0.20);
  EXPECT_GT(filtered, 0.0);
}

}  // namespace
