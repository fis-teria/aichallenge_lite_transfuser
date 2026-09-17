#pragma once

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit {

struct RotationRecoveryResult {
  bool valid{false};
  double signed_yaw_rate_excess_radps{0.0};
  double countersteer_rad{0.0};
  double requested_steering_rad{0.0};
};

struct PredictiveRotationRecoveryConfig {
  double prediction_horizon_sec{0.0};
  double steering_response_fraction{0.0};
  double maximum_yaw_acceleration_radps2{0.0};
  double reactive_yaw_rate_threshold_radps{0.0};
  double predictive_yaw_rate_threshold_radps{0.0};
  double minimum_predictive_yaw_acceleration_radps2{0.0};
  double oversteer_slip_angle_threshold_rad{0.0};
  double oversteer_slip_yaw_rate_gain{0.0};
  double countersteer_gain{0.0};
  double maximum_countersteer_rad{0.0};
};

struct PredictiveRotationRecoveryResult {
  bool valid{false};
  bool predictive_trigger{false};
  double signed_yaw_rate_excess_radps{0.0};
  double predicted_inertial_yaw_rate_radps{0.0};
  double predicted_steering_yaw_rate_radps{0.0};
  double predicted_signed_yaw_rate_excess_radps{0.0};
  double oversteer_slip_angle_rad{0.0};
  double risk_yaw_rate_excess_radps{0.0};
  double countersteer_rad{0.0};
  double requested_steering_rad{0.0};
};

// Rotation recovery must suppress propulsion without cancelling a braking
// request that was already issued by trajectory tracking.
inline double suppressPositiveAccelerationForRotation(
    double requested_acceleration_mps2) noexcept {
  return std::isfinite(requested_acceleration_mps2)
             ? std::min(0.0, requested_acceleration_mps2)
             : 0.0;
}

// Positive excess means that the kart is rotating farther into the corner
// than the path curvature requests. Recovery subtracts a bounded steering
// component in the opposite direction; the caller applies the actuator's
// normal angle/rate contract afterward.
inline RotationRecoveryResult computeRotationRecovery(
    bool active, double signed_path_curvature_1pm,
    double reference_yaw_rate_radps, double measured_yaw_rate_radps,
    double nominal_steering_rad, double yaw_rate_threshold_radps,
    double countersteer_gain, double maximum_countersteer_rad) noexcept {
  RotationRecoveryResult result;
  const double values[] = {
      signed_path_curvature_1pm, reference_yaw_rate_radps,
      measured_yaw_rate_radps,   nominal_steering_rad,
      yaw_rate_threshold_radps,  countersteer_gain,
      maximum_countersteer_rad,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return result;
    }
  }
  if (yaw_rate_threshold_radps < 0.0 || countersteer_gain < 0.0 ||
      maximum_countersteer_rad < 0.0) {
    return result;
  }
  const double turn_sign = signed_path_curvature_1pm < 0.0 ? -1.0 : 1.0;
  result.signed_yaw_rate_excess_radps =
      turn_sign * (measured_yaw_rate_radps - reference_yaw_rate_radps);
  result.requested_steering_rad = nominal_steering_rad;
  const double nominal_turn_component = turn_sign * nominal_steering_rad;
  if (active && nominal_turn_component > 0.0) {
    const double excess_over_threshold = std::max(
        0.0, result.signed_yaw_rate_excess_radps - yaw_rate_threshold_radps);
    result.countersteer_rad =
        std::min({maximum_countersteer_rad, nominal_turn_component,
                  countersteer_gain * excess_over_threshold});
    result.requested_steering_rad =
        nominal_steering_rad - turn_sign * result.countersteer_rad;
  }
  result.valid = true;
  return result;
}

// Predict rotation at the end of the actuation delay instead of waiting for
// the measured yaw-rate error to cross the reactive threshold. A prediction
// is allowed to trigger recovery only when either yaw acceleration or lateral
// slip confirms that rotation is developing; a large nominal steering command
// by itself is therefore not classified as oversteer.
inline PredictiveRotationRecoveryResult computePredictiveRotationRecovery(
    bool active, double signed_path_curvature_1pm,
    double reference_yaw_rate_radps, double measured_yaw_rate_radps,
    double filtered_yaw_acceleration_radps2, double commanded_yaw_rate_radps,
    double slip_angle_rad, bool slip_angle_valid, double nominal_steering_rad,
    const PredictiveRotationRecoveryConfig &config) noexcept {
  PredictiveRotationRecoveryResult result;
  const double values[] = {
      signed_path_curvature_1pm,
      reference_yaw_rate_radps,
      measured_yaw_rate_radps,
      filtered_yaw_acceleration_radps2,
      commanded_yaw_rate_radps,
      slip_angle_rad,
      nominal_steering_rad,
      config.prediction_horizon_sec,
      config.steering_response_fraction,
      config.maximum_yaw_acceleration_radps2,
      config.reactive_yaw_rate_threshold_radps,
      config.predictive_yaw_rate_threshold_radps,
      config.minimum_predictive_yaw_acceleration_radps2,
      config.oversteer_slip_angle_threshold_rad,
      config.oversteer_slip_yaw_rate_gain,
      config.countersteer_gain,
      config.maximum_countersteer_rad,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return result;
    }
  }
  if (config.prediction_horizon_sec < 0.0 ||
      config.steering_response_fraction < 0.0 ||
      config.steering_response_fraction > 1.0 ||
      config.maximum_yaw_acceleration_radps2 < 0.0 ||
      config.reactive_yaw_rate_threshold_radps < 0.0 ||
      config.predictive_yaw_rate_threshold_radps < 0.0 ||
      config.minimum_predictive_yaw_acceleration_radps2 < 0.0 ||
      config.oversteer_slip_angle_threshold_rad < 0.0 ||
      config.oversteer_slip_yaw_rate_gain < 0.0 ||
      config.countersteer_gain < 0.0 || config.maximum_countersteer_rad < 0.0) {
    return result;
  }

  const double turn_sign = signed_path_curvature_1pm < 0.0 ? -1.0 : 1.0;
  const double bounded_yaw_acceleration = std::clamp(
      filtered_yaw_acceleration_radps2, -config.maximum_yaw_acceleration_radps2,
      config.maximum_yaw_acceleration_radps2);
  result.predicted_inertial_yaw_rate_radps =
      measured_yaw_rate_radps +
      bounded_yaw_acceleration * config.prediction_horizon_sec;
  result.predicted_steering_yaw_rate_radps =
      measured_yaw_rate_radps +
      config.steering_response_fraction *
          (commanded_yaw_rate_radps - measured_yaw_rate_radps);
  result.signed_yaw_rate_excess_radps =
      turn_sign * (measured_yaw_rate_radps - reference_yaw_rate_radps);
  const double predicted_signed_yaw_rate =
      std::max({turn_sign * measured_yaw_rate_radps,
                turn_sign * result.predicted_inertial_yaw_rate_radps,
                turn_sign * result.predicted_steering_yaw_rate_radps});
  result.predicted_signed_yaw_rate_excess_radps =
      predicted_signed_yaw_rate - turn_sign * reference_yaw_rate_radps;

  // beta = velocity heading - body heading. In a left turn, negative beta is
  // the tail moving outward; the sign is mirrored for a right turn.
  result.oversteer_slip_angle_rad =
      slip_angle_valid ? std::max(0.0, -turn_sign * slip_angle_rad) : 0.0;
  const double slip_risk_radps =
      config.oversteer_slip_yaw_rate_gain *
      std::max(0.0, result.oversteer_slip_angle_rad -
                        config.oversteer_slip_angle_threshold_rad);
  result.risk_yaw_rate_excess_radps =
      std::max(result.signed_yaw_rate_excess_radps + slip_risk_radps,
               result.predicted_signed_yaw_rate_excess_radps);

  const double signed_yaw_acceleration = turn_sign * bounded_yaw_acceleration;
  const bool developing_oversteer =
      signed_yaw_acceleration >=
          config.minimum_predictive_yaw_acceleration_radps2 ||
      (slip_angle_valid && result.oversteer_slip_angle_rad >=
                               config.oversteer_slip_angle_threshold_rad) ||
      result.signed_yaw_rate_excess_radps >=
          config.reactive_yaw_rate_threshold_radps;
  result.predictive_trigger =
      developing_oversteer && result.risk_yaw_rate_excess_radps >=
                                  config.predictive_yaw_rate_threshold_radps;

  result.requested_steering_rad = nominal_steering_rad;
  const double nominal_turn_component = turn_sign * nominal_steering_rad;
  if (active && nominal_turn_component > 0.0) {
    const double reactive_error = result.signed_yaw_rate_excess_radps -
                                  config.reactive_yaw_rate_threshold_radps;
    const double predictive_error = result.risk_yaw_rate_excess_radps -
                                    config.predictive_yaw_rate_threshold_radps;
    result.countersteer_rad = std::min(
        {config.maximum_countersteer_rad, nominal_turn_component,
         config.countersteer_gain *
             std::max(0.0, std::max(reactive_error, predictive_error))});
    result.requested_steering_rad =
        nominal_steering_rad - turn_sign * result.countersteer_rad;
  }
  result.valid = true;
  return result;
}

} // namespace simple_pure_pursuit
