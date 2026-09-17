#pragma once
#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit {
struct BoundedSteeringCommand {
  bool valid{false};
  double requested_angle_rad{0.0};
  double bounded_angle_rad{0.0};
  double requested_rate_radps{0.0};
  double bounded_rate_radps{0.0};
  bool angle_limited{false};
  bool rate_limited{false};
};

struct BoundedLongitudinalAcceleration {
  bool valid{false};
  double requested_mps2{0.0};
  double bounded_mps2{0.0};
  bool acceleration_limited{false};
  bool deceleration_limited{false};
};

inline BoundedLongitudinalAcceleration boundLongitudinalAcceleration(
    double target_speed_mps, double current_speed_mps,
    double proportional_gain, double acceleration_limit_mps2,
    double deceleration_limit_mps2, bool safe_stop,
    double safe_stop_deceleration_mps2) {
  BoundedLongitudinalAcceleration result;
  if (!std::isfinite(target_speed_mps) ||
      !std::isfinite(current_speed_mps) ||
      !std::isfinite(proportional_gain) || proportional_gain < 0.0 ||
      !std::isfinite(acceleration_limit_mps2) ||
      acceleration_limit_mps2 < 0.0 ||
      !std::isfinite(deceleration_limit_mps2) ||
      deceleration_limit_mps2 <= 0.0 ||
      !std::isfinite(safe_stop_deceleration_mps2) ||
      safe_stop_deceleration_mps2 <= 0.0) {
    return result;
  }
  result.requested_mps2 =
      proportional_gain * (target_speed_mps - current_speed_mps);
  const double braking_limit_mps2 =
      safe_stop ? safe_stop_deceleration_mps2 : deceleration_limit_mps2;
  result.bounded_mps2 =
      std::clamp(result.requested_mps2, -braking_limit_mps2,
                 acceleration_limit_mps2);
  result.acceleration_limited =
      result.requested_mps2 > acceleration_limit_mps2 + 1.0e-12;
  result.deceleration_limited =
      result.requested_mps2 < -braking_limit_mps2 - 1.0e-12;
  result.valid = std::isfinite(result.bounded_mps2);
  return result;
}

// A zero-speed SAFE_STOP normally produces zero acceleration once the vehicle
// has stopped. A trajectory may carry an explicit negative acceleration when
// the upstream owner needs the brake actuator held before a gear shift. Only
// accept that feed-forward request in SAFE_STOP, and never let it weaken
// braking already required by the feedback controller.
inline double applySafeStopTrajectoryDeceleration(
    double controller_acceleration_mps2,
    double trajectory_acceleration_mps2, bool safe_stop) {
  if (!std::isfinite(controller_acceleration_mps2)) {
    return 0.0;
  }
  if (!safe_stop || !std::isfinite(trajectory_acceleration_mps2) ||
      trajectory_acceleration_mps2 >= 0.0) {
    return controller_acceleration_mps2;
  }
  return std::min(controller_acceleration_mps2,
                  trajectory_acceleration_mps2);
}

// PPの要求を実車両とMuxが共有するhard angle/rate内へ写像する純粋モデル。
// 順序はMuxと同じく angle clamp -> rate clamp -> angle clamp とする。
inline BoundedSteeringCommand boundSteeringCommand(
    double requested_angle_rad, double reference_angle_rad, double dt_sec,
    double hard_angle_limit_rad, double hard_rate_limit_radps) {
  BoundedSteeringCommand result;
  result.requested_angle_rad = requested_angle_rad;
  if (!std::isfinite(requested_angle_rad) ||
      !std::isfinite(reference_angle_rad) || !std::isfinite(dt_sec) ||
      dt_sec <= 0.0 || !std::isfinite(hard_angle_limit_rad) ||
      hard_angle_limit_rad <= 0.0 ||
      !std::isfinite(hard_rate_limit_radps) ||
      hard_rate_limit_radps <= 0.0) {
    return result;
  }

  const double bounded_reference =
      std::clamp(reference_angle_rad, -hard_angle_limit_rad,
                 hard_angle_limit_rad);
  const double angle_clamped_request =
      std::clamp(requested_angle_rad, -hard_angle_limit_rad,
                 hard_angle_limit_rad);
  const double max_delta_rad = hard_rate_limit_radps * dt_sec;
  if (!std::isfinite(max_delta_rad) || max_delta_rad <= 0.0) {
    return result;
  }
  const double rate_clamped =
      std::clamp(angle_clamped_request, bounded_reference - max_delta_rad,
                 bounded_reference + max_delta_rad);
  result.bounded_angle_rad =
      std::clamp(rate_clamped, -hard_angle_limit_rad, hard_angle_limit_rad);
  result.requested_rate_radps =
      (requested_angle_rad - bounded_reference) / dt_sec;
  result.bounded_rate_radps =
      (result.bounded_angle_rad - bounded_reference) / dt_sec;
  if (!std::isfinite(result.requested_rate_radps) ||
      !std::isfinite(result.bounded_rate_radps)) {
    return result;
  }
  result.angle_limited =
      std::abs(requested_angle_rad - angle_clamped_request) > 1.0e-12;
  result.rate_limited =
      std::abs(angle_clamped_request - result.bounded_angle_rad) > 1.0e-12;
  result.valid = true;
  return result;
}

inline BoundedSteeringCommand applySteeringExecutionContract(
    double requested_angle_rad, double reference_angle_rad, double dt_sec,
    double hard_angle_limit_rad, double hard_rate_limit_radps,
    bool passthrough_enabled) {
  if (!passthrough_enabled) {
    return boundSteeringCommand(requested_angle_rad, reference_angle_rad,
                                dt_sec, hard_angle_limit_rad,
                                hard_rate_limit_radps);
  }
  BoundedSteeringCommand result;
  result.requested_angle_rad = requested_angle_rad;
  if (!std::isfinite(requested_angle_rad) ||
      !std::isfinite(reference_angle_rad) || !std::isfinite(dt_sec) ||
      dt_sec <= 0.0) {
    return result;
  }
  result.bounded_angle_rad = requested_angle_rad;
  result.requested_rate_radps =
      (requested_angle_rad - reference_angle_rad) / dt_sec;
  result.bounded_rate_radps = result.requested_rate_radps;
  result.valid = std::isfinite(result.requested_rate_radps);
  return result;
}
} // namespace simple_pure_pursuit
