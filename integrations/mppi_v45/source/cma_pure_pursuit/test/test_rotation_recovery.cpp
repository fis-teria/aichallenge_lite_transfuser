#include "simple_pure_pursuit/rotation_recovery.hpp"
#include "simple_pure_pursuit/rotation_controller.hpp"

#include <gtest/gtest.h>

namespace simple_pure_pursuit {
namespace {

TEST(RotationController, HoldsReleasesAndRespectsCooldown) {
  RotationControllerParameters p;
  p.rotation_gate_enabled = true;
  RotationControllerState state;
  RotationControllerInput in{1., .02, .1, .6, 1.1, .6,
      0., .2, .2, 0., false, true};
  auto result = stepRotationController(p,in,state);
  ASSERT_TRUE(state.active);
  ASSERT_TRUE(result.recovery.valid);
  EXPECT_LT(result.recovery.requested_steering_rad,in.nominal_tire_angle_rad);
  in.stamp_sec = 1.02;
  in.measured_yaw_rate_radps = .6;
  stepRotationController(p,in,state);
  EXPECT_TRUE(state.active);
  in.stamp_sec = 1.08;
  stepRotationController(p,in,state);
  EXPECT_FALSE(state.active);
  in.measured_yaw_rate_radps = 1.1;
  in.stamp_sec = 1.3;
  stepRotationController(p,in,state);
  EXPECT_FALSE(state.active);
  in.stamp_sec = 1.6;
  stepRotationController(p,in,state);
  EXPECT_TRUE(state.active);
}

TEST(RotationController, RolloutsCopyHistoryIndependently) {
  RotationControllerParameters p;
  p.rotation_gate_enabled = true;
  RotationControllerState live;
  RotationControllerInput in{1., .02, .1, .6, .6, .6,
      0., .2, .2, 0., false, true};
  stepRotationController(p,in,live);
  const auto initial = live;
  auto first = live, second = live;
  in.stamp_sec += .02;
  in.measured_yaw_rate_radps = 1.1;
  stepRotationController(p,in,first);
  in.measured_yaw_rate_radps = .6;
  stepRotationController(p,in,second);
  EXPECT_TRUE(first.active);
  EXPECT_FALSE(second.active);
  EXPECT_FALSE(live.active);
  EXPECT_DOUBLE_EQ(live.previous_yaw_rate_radps,initial.previous_yaw_rate_radps);
}

PredictiveRotationRecoveryConfig predictiveConfig() {
  PredictiveRotationRecoveryConfig config;
  config.prediction_horizon_sec = 0.35;
  config.steering_response_fraction = 0.50;
  config.maximum_yaw_acceleration_radps2 = 6.0;
  config.reactive_yaw_rate_threshold_radps = 0.30;
  config.predictive_yaw_rate_threshold_radps = 0.18;
  config.minimum_predictive_yaw_acceleration_radps2 = 0.30;
  config.oversteer_slip_angle_threshold_rad = 0.04;
  config.oversteer_slip_yaw_rate_gain = 2.0;
  config.countersteer_gain = 0.35;
  config.maximum_countersteer_rad = 0.12;
  return config;
}

TEST(RotationRecovery, DetectsExcessRotationSymmetrically) {
  const auto left =
      computeRotationRecovery(true, 0.10, 0.60, 1.10, 0.20, 0.30, 0.40, 0.12);
  const auto right = computeRotationRecovery(true, -0.10, -0.60, -1.10, -0.20,
                                             0.30, 0.40, 0.12);

  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_NEAR(left.signed_yaw_rate_excess_radps, 0.50, 1.0e-12);
  EXPECT_NEAR(right.signed_yaw_rate_excess_radps, 0.50, 1.0e-12);
  EXPECT_NEAR(left.countersteer_rad, 0.08, 1.0e-12);
  EXPECT_NEAR(right.countersteer_rad, 0.08, 1.0e-12);
  EXPECT_NEAR(left.requested_steering_rad, 0.12, 1.0e-12);
  EXPECT_NEAR(right.requested_steering_rad, -0.12, 1.0e-12);
}

TEST(RotationRecovery, DoesNotChangeSteeringBelowThresholdOrWhileInactive) {
  const auto below =
      computeRotationRecovery(true, 0.10, 0.60, 0.80, 0.20, 0.30, 0.40, 0.12);
  const auto inactive =
      computeRotationRecovery(false, 0.10, 0.60, 1.30, 0.20, 0.30, 0.40, 0.12);

  ASSERT_TRUE(below.valid);
  ASSERT_TRUE(inactive.valid);
  EXPECT_DOUBLE_EQ(below.countersteer_rad, 0.0);
  EXPECT_DOUBLE_EQ(below.requested_steering_rad, 0.20);
  EXPECT_DOUBLE_EQ(inactive.countersteer_rad, 0.0);
  EXPECT_DOUBLE_EQ(inactive.requested_steering_rad, 0.20);
}

TEST(RotationRecovery, BoundsCountersteerAndRejectsInvalidInputs) {
  const auto bounded =
      computeRotationRecovery(true, 0.10, 0.20, 2.00, 0.20, 0.30, 1.00, 0.12);
  const auto invalid =
      computeRotationRecovery(true, 0.10, 0.20, 2.00, 0.20, 0.30, -1.00, 0.12);

  ASSERT_TRUE(bounded.valid);
  EXPECT_DOUBLE_EQ(bounded.countersteer_rad, 0.12);
  EXPECT_DOUBLE_EQ(bounded.requested_steering_rad, 0.08);
  EXPECT_FALSE(invalid.valid);
}

TEST(RotationRecovery, SuppressesPropulsionButPreservesBraking) {
  EXPECT_DOUBLE_EQ(suppressPositiveAccelerationForRotation(1.5), 0.0);
  EXPECT_DOUBLE_EQ(suppressPositiveAccelerationForRotation(0.0), 0.0);
  EXPECT_DOUBLE_EQ(suppressPositiveAccelerationForRotation(-2.0), -2.0);
}

TEST(PredictiveRotationRecovery,
     TriggersBeforeReactiveThresholdFromYawAcceleration) {
  const auto result = computePredictiveRotationRecovery(
      true, 0.10, 0.60, 0.72, 0.50, 0.72, 0.0, false, 0.20, predictiveConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.signed_yaw_rate_excess_radps, 0.12, 1.0e-12);
  EXPECT_NEAR(result.predicted_inertial_yaw_rate_radps, 0.895, 1.0e-12);
  EXPECT_NEAR(result.predicted_signed_yaw_rate_excess_radps, 0.295, 1.0e-12);
  EXPECT_TRUE(result.predictive_trigger);
  EXPECT_GT(result.countersteer_rad, 0.0);
  EXPECT_LT(result.requested_steering_rad, 0.20);
}

TEST(PredictiveRotationRecovery, IncludesPendingSteeringResponseInPrediction) {
  const auto result =
      computePredictiveRotationRecovery(false, 0.10, 0.60, 0.70, 0.35, 1.10,
                                        0.0, false, 0.20, predictiveConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.predicted_steering_yaw_rate_radps, 0.90, 1.0e-12);
  EXPECT_NEAR(result.predicted_signed_yaw_rate_excess_radps, 0.30, 1.0e-12);
  EXPECT_TRUE(result.predictive_trigger);
  EXPECT_DOUBLE_EQ(result.countersteer_rad, 0.0);
}

TEST(PredictiveRotationRecovery, DetectsOversteerSlipSymmetrically) {
  const auto left =
      computePredictiveRotationRecovery(false, 0.10, 0.60, 0.72, 0.0, 0.72,
                                        -0.10, true, 0.20, predictiveConfig());
  const auto right =
      computePredictiveRotationRecovery(false, -0.10, -0.60, -0.72, 0.0, -0.72,
                                        0.10, true, -0.20, predictiveConfig());

  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_NEAR(left.oversteer_slip_angle_rad, 0.10, 1.0e-12);
  EXPECT_NEAR(right.oversteer_slip_angle_rad, 0.10, 1.0e-12);
  EXPECT_NEAR(left.risk_yaw_rate_excess_radps, 0.24, 1.0e-12);
  EXPECT_NEAR(right.risk_yaw_rate_excess_radps, 0.24, 1.0e-12);
  EXPECT_TRUE(left.predictive_trigger);
  EXPECT_TRUE(right.predictive_trigger);
}

TEST(PredictiveRotationRecovery, DoesNotTriggerWhileRotationIsUnwinding) {
  const auto result =
      computePredictiveRotationRecovery(false, 0.10, 0.60, 0.72, -0.50, 1.10,
                                        0.08, true, 0.20, predictiveConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.oversteer_slip_angle_rad, 0.0);
  EXPECT_FALSE(result.predictive_trigger);
}

TEST(PredictiveRotationRecovery, BoundsCorrectionAndRejectsInvalidConfig) {
  auto config = predictiveConfig();
  config.countersteer_gain = 2.0;
  const auto bounded = computePredictiveRotationRecovery(
      true, 0.10, 0.60, 1.20, 3.0, 1.20, -0.20, true, 0.20, config);
  config.steering_response_fraction = 1.1;
  const auto invalid = computePredictiveRotationRecovery(
      true, 0.10, 0.60, 1.20, 3.0, 1.20, -0.20, true, 0.20, config);

  ASSERT_TRUE(bounded.valid);
  EXPECT_DOUBLE_EQ(bounded.countersteer_rad, 0.12);
  EXPECT_DOUBLE_EQ(bounded.requested_steering_rad, 0.08);
  EXPECT_FALSE(invalid.valid);
}

TEST(PredictiveRotationRecovery, NeverCrossesZeroOrAccumulatesAgainstNominal) {
  auto config = predictiveConfig();
  config.countersteer_gain = 10.0;
  config.maximum_countersteer_rad = 0.12;
  const auto small_left = computePredictiveRotationRecovery(
      true, 0.10, 0.20, 1.20, 3.0, 1.20, -0.20, true, 0.05, config);
  const auto already_countersteering = computePredictiveRotationRecovery(
      true, 0.10, 0.20, 1.20, 3.0, 1.20, -0.20, true, -0.05, config);

  ASSERT_TRUE(small_left.valid);
  EXPECT_DOUBLE_EQ(small_left.countersteer_rad, 0.05);
  EXPECT_DOUBLE_EQ(small_left.requested_steering_rad, 0.0);
  ASSERT_TRUE(already_countersteering.valid);
  EXPECT_DOUBLE_EQ(already_countersteering.countersteer_rad, 0.0);
  EXPECT_DOUBLE_EQ(already_countersteering.requested_steering_rad, -0.05);
}

} // namespace
} // namespace simple_pure_pursuit
