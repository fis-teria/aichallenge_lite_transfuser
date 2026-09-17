#pragma once

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit {

constexpr double kSteeringTrackingDurationEpsilonSec = 1.0e-12;

struct SteeringTrackingSpeedGateConfig {
  bool enabled{false};
  double entry_error_rad{0.18};
  double release_error_rad{0.08};
  double entry_duration_sec{0.10};
  double release_duration_sec{0.10};
  double deceleration_mps2{1.0};
};

struct SteeringTrackingSpeedGateState {
  bool active{false};
  double entry_elapsed_sec{0.0};
  double release_elapsed_sec{0.0};
};

struct SteeringTrackingSpeedGateResult {
  bool active{false};
  double steering_error_rad{0.0};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

inline SteeringTrackingSpeedGateResult applySteeringTrackingSpeedGate(
    const SteeringTrackingSpeedGateConfig &config, const bool maneuver_active,
    const bool measured_steering_fresh, const double requested_steering_rad,
    const double measured_steering_rad, const double current_speed_mps,
    const double requested_speed_mps, const double requested_acceleration_mps2,
    const double dt_sec, SteeringTrackingSpeedGateState *state) noexcept {
  SteeringTrackingSpeedGateResult result;
  result.speed_mps = requested_speed_mps;
  result.acceleration_mps2 = requested_acceleration_mps2;
  if (state == nullptr) {
    return result;
  }

  const bool inputs_valid =
      config.enabled && maneuver_active && measured_steering_fresh &&
      std::isfinite(requested_steering_rad) &&
      std::isfinite(measured_steering_rad) &&
      std::isfinite(current_speed_mps) && std::isfinite(requested_speed_mps) &&
      std::isfinite(requested_acceleration_mps2) && std::isfinite(dt_sec) &&
      dt_sec > 0.0 && config.entry_error_rad > config.release_error_rad &&
      config.release_error_rad >= 0.0 && config.entry_duration_sec >= 0.0 &&
      config.release_duration_sec >= 0.0 && config.deceleration_mps2 >= 0.0;
  if (!inputs_valid) {
    *state = SteeringTrackingSpeedGateState{};
    return result;
  }

  result.steering_error_rad =
      std::abs(requested_steering_rad - measured_steering_rad);
  if (!state->active) {
    state->release_elapsed_sec = 0.0;
    state->entry_elapsed_sec =
        result.steering_error_rad >= config.entry_error_rad
            ? state->entry_elapsed_sec + dt_sec
            : 0.0;
    state->active =
        state->entry_elapsed_sec + kSteeringTrackingDurationEpsilonSec >=
        config.entry_duration_sec;
  } else {
    state->entry_elapsed_sec = 0.0;
    state->release_elapsed_sec =
        result.steering_error_rad <= config.release_error_rad
            ? state->release_elapsed_sec + dt_sec
            : 0.0;
    if (state->release_elapsed_sec + kSteeringTrackingDurationEpsilonSec >=
        config.release_duration_sec) {
      *state = SteeringTrackingSpeedGateState{};
    }
  }

  result.active = state->active;
  if (result.active) {
    result.speed_mps =
        std::min(requested_speed_mps, std::abs(current_speed_mps));
    result.acceleration_mps2 =
        std::min(requested_acceleration_mps2, -config.deceleration_mps2);
  }
  return result;
}

} // namespace simple_pure_pursuit
