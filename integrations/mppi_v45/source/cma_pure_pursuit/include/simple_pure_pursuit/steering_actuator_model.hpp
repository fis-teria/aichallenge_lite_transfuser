#pragma once

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit {

// Zero preserves the legacy command-derived prediction bound. A measured
// physical tire rate is independent of how quickly commands may change.
inline double physicalTireSteeringRate(double physical_rate_radps,
                                      double command_rate_radps,
                                      double command_to_tire_ratio) noexcept {
  return physical_rate_radps > 0.0
             ? physical_rate_radps
             : command_rate_radps * command_to_tire_ratio;
}

// AWSIM's Ackermann command is an actuator-side target, while SteeringReport
// is the physical tire angle. Keeping the conversion explicit prevents the
// controller and the MPPI rollout from silently using different curvatures.
inline double commandToTireSteeringAngle(
    double command_angle_rad, double command_to_tire_ratio,
    double maximum_tire_angle_rad) noexcept {
  if (!std::isfinite(command_angle_rad) ||
      !std::isfinite(command_to_tire_ratio) || command_to_tire_ratio <= 0.0 ||
      !std::isfinite(maximum_tire_angle_rad) || maximum_tire_angle_rad <= 0.0) {
    return 0.0;
  }
  return std::clamp(command_angle_rad * command_to_tire_ratio,
                    -maximum_tire_angle_rad, maximum_tire_angle_rad);
}

inline double tireToCommandSteeringAngle(
    double tire_angle_rad, double command_to_tire_ratio,
    double maximum_command_angle_rad) noexcept {
  if (!std::isfinite(tire_angle_rad) ||
      !std::isfinite(command_to_tire_ratio) || command_to_tire_ratio <= 0.0 ||
      !std::isfinite(maximum_command_angle_rad) ||
      maximum_command_angle_rad <= 0.0) {
    return 0.0;
  }
  return std::clamp(tire_angle_rad / command_to_tire_ratio,
                    -maximum_command_angle_rad, maximum_command_angle_rad);
}

struct SteeringDemandAccelerationHoldResult {
  bool valid{false};
  bool active{false};
  double tracking_error_rad{0.0};
  double allowed_acceleration_mps2{0.0};
  double acceleration_mps2{0.0};
};

// Limit positive acceleration at large steering demand. A nonzero allowance
// restores gentle acceleration as the tire catches up; zero retains coasting.
inline SteeringDemandAccelerationHoldResult
holdPositiveAccelerationForSteeringDemand(
    bool enabled, double current_speed_mps, double minimum_active_speed_mps,
    bool steering_demand_limited, bool steering_status_fresh,
    double target_tire_angle_rad, double measured_tire_angle_rad,
    double minimum_tire_angle_rad, double tracking_error_threshold_rad,
    double requested_acceleration_mps2,
    double maximum_hold_acceleration_mps2 = 0.0) noexcept {
  SteeringDemandAccelerationHoldResult result;
  const double values[] = {current_speed_mps,
                           minimum_active_speed_mps,
                           target_tire_angle_rad,
                           measured_tire_angle_rad,
                           minimum_tire_angle_rad,
                           tracking_error_threshold_rad,
                           requested_acceleration_mps2,
                           maximum_hold_acceleration_mps2};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return result;
    }
  }
  if (minimum_active_speed_mps < 0.0 || minimum_tire_angle_rad < 0.0 ||
      tracking_error_threshold_rad < 0.0 || maximum_hold_acceleration_mps2 < 0.0) {
    return result;
  }

  result.tracking_error_rad =
      std::abs(target_tire_angle_rad - measured_tire_angle_rad);
  const bool tracking_lag =
      steering_status_fresh &&
      result.tracking_error_rad >= tracking_error_threshold_rad;
  result.active = enabled && current_speed_mps >= minimum_active_speed_mps &&
                  requested_acceleration_mps2 > 0.0 &&
                  std::abs(target_tire_angle_rad) >= minimum_tire_angle_rad &&
                  (steering_demand_limited || tracking_lag);
  result.acceleration_mps2 = requested_acceleration_mps2;
  if (result.active) {
    // Full allowance at the hold threshold, tapering to coast at twice that
    // error. Saturation alone may permit gentle acceleration once tracked.
    const double tracking_allowance =
        tracking_error_threshold_rad > 0.0
            ? std::clamp(2.0 - result.tracking_error_rad /
                                   tracking_error_threshold_rad,
                         0.0, 1.0)
            : (result.tracking_error_rad == 0.0 ? 1.0 : 0.0);
    result.allowed_acceleration_mps2 =
        steering_status_fresh
            ? maximum_hold_acceleration_mps2 * tracking_allowance
            : 0.0;
    result.acceleration_mps2 = std::min(
        requested_acceleration_mps2, result.allowed_acceleration_mps2);
  }
  result.valid = true;
  return result;
}

} // namespace simple_pure_pursuit
