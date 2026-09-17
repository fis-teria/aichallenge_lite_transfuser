#include "simple_pure_pursuit/pure_pursuit_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace simple_pure_pursuit {
namespace {

PurePursuitCoreInput nominalInput() {
  PurePursuitCoreInput input;
  input.rear_x = 0.0;
  input.rear_y = 0.0;
  input.yaw = 0.0;
  input.far_target_x = 5.0;
  input.far_target_y = 1.0;
  input.near_target_x = 2.5;
  input.near_target_y = 0.25;
  input.far_nominal_distance_m = 5.0;
  input.near_nominal_distance_m = 2.5;
  input.actual_distance_blend = 0.0;
  input.near_steering_blend = 0.25;
  input.wheel_base_m = 1.087;
  input.steering_gain = 1.5;
  input.target_velocity_mps = 8.0;
  input.steering_reference_rad = 0.0;
  input.dt_sec = 0.01;
  input.hard_steering_angle_rad = 0.3665191429188092;
  input.hard_steering_rate_radps = 100.0;
  return input;
}

TEST(PurePursuitCore, MatchesControllerGeometryEquation) {
  const auto input = nominalInput();
  const auto output = computePurePursuitCore(input);
  ASSERT_TRUE(output.valid);
  const double far_heading = std::atan2(1.0, 5.0);
  const double near_heading = std::atan2(0.25, 2.5);
  const double far =
      std::atan2(2.0 * input.wheel_base_m * std::sin(far_heading), 5.0);
  const double near =
      std::atan2(2.0 * input.wheel_base_m * std::sin(near_heading), 2.5);
  const double expected = input.steering_gain * (0.75 * far + 0.25 * near);
  EXPECT_NEAR(output.pure_pursuit_steering_rad, expected, 1.0e-12);
  EXPECT_NEAR(output.requested_steering_rad, expected, 1.0e-12);
  EXPECT_NEAR(output.bounded_steering_rad, expected, 1.0e-12);
}

TEST(PurePursuitCore, AppliesAngleThenRateExecutionBounds) {
  auto input = nominalInput();
  input.far_target_x = 1.0;
  input.far_target_y = 10.0;
  input.near_target_x = 0.5;
  input.near_target_y = 5.0;
  input.hard_steering_angle_rad = 0.30;
  input.hard_steering_rate_radps = 1.0;
  input.dt_sec = 0.05;
  input.steering_reference_rad = 0.10;
  const auto output = computePurePursuitCore(input);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.angle_limited);
  EXPECT_TRUE(output.rate_limited);
  EXPECT_NEAR(output.bounded_steering_rad, 0.15, 1.0e-12);
}

TEST(PurePursuitCore, ReverseTravelFlipsGeometryBeforeFeedforward) {
  auto input = nominalInput();
  const auto forward = computePurePursuitCore(input);
  input.target_velocity_mps = -2.0;
  input.curvature_feedforward_rad = 0.01;
  const auto reverse = computePurePursuitCore(input);
  ASSERT_TRUE(forward.valid);
  ASSERT_TRUE(reverse.valid);
  EXPECT_NEAR(reverse.pure_pursuit_steering_rad,
              -forward.pure_pursuit_steering_rad, 1.0e-12);
  EXPECT_NEAR(reverse.requested_steering_rad,
              -forward.pure_pursuit_steering_rad + 0.01, 1.0e-12);
}

TEST(PurePursuitCore, RejectsNonFiniteOrDegenerateInput) {
  auto input = nominalInput();
  input.wheel_base_m = 0.0;
  EXPECT_FALSE(computePurePursuitCore(input).valid);
  input = nominalInput();
  input.far_target_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(computePurePursuitCore(input).valid);
}

TEST(PurePursuitLookahead, MatchesCmaSpeedAndCurvatureSchedule) {
  const PurePursuitLookaheadPolicy policy{0.9, 2.0, 1.2, 2.0};
  EXPECT_NEAR(computePurePursuitRawLookahead(6.0, 0.0, policy), 7.4, 1.0e-12);
  EXPECT_NEAR(computePurePursuitRawLookahead(6.0, 0.5, policy), 3.7, 1.0e-12);
  EXPECT_NEAR(computePurePursuitRawLookahead(-2.0, 100.0, policy), 1.2,
              1.0e-12);
}

struct PreviewPoint {
  double x{0.0};
  double y{0.0};
};

TEST(PurePursuitPreview, PreservesDiscreteEuclideanControllerSelection) {
  const std::vector<PreviewPoint> points{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const auto selected = selectPurePursuitPreview(
      points.data(), points.size(), 1U, 1.5, 0.0, 0.0, false,
      [](const PreviewPoint &point) { return point.x; },
      [](const PreviewPoint &point) { return point.y; });
  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.lower_index, 2U);
  EXPECT_EQ(selected.upper_index, 2U);
  EXPECT_DOUBLE_EQ(selected.x, 2.0);
}

TEST(PurePursuitPreview, InterpolatesForwardArcAndMarksEndpointFallback) {
  const std::vector<PreviewPoint> points{{0.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}};
  const auto interpolated = selectPurePursuitPreview(
      points.data(), points.size(), 0U, 2.0, 0.0, 0.0, true,
      [](const PreviewPoint &point) { return point.x; },
      [](const PreviewPoint &point) { return point.y; });
  ASSERT_TRUE(interpolated.valid);
  EXPECT_EQ(interpolated.lower_index, 1U);
  EXPECT_EQ(interpolated.upper_index, 2U);
  EXPECT_NEAR(interpolated.interpolation_ratio, 0.5, 1.0e-12);
  EXPECT_NEAR(interpolated.x, 1.0, 1.0e-12);
  EXPECT_NEAR(interpolated.y, 1.0, 1.0e-12);

  const auto endpoint = selectPurePursuitPreview(
      points.data(), points.size(), 0U, 10.0, 0.0, 0.0, true,
      [](const PreviewPoint &point) { return point.x; },
      [](const PreviewPoint &point) { return point.y; });
  ASSERT_TRUE(endpoint.valid);
  EXPECT_TRUE(endpoint.endpoint_fallback);
  EXPECT_EQ(endpoint.lower_index, 2U);
}

} // namespace
} // namespace simple_pure_pursuit
