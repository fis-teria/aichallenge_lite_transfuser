#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace simple_pure_pursuit {

struct PurePursuitLookaheadPolicy {
  double gain{1.0};
  double minimum_distance_m{1.0};
  double curvature_minimum_distance_m{1.0};
  double curvature_sensitivity{0.0};
};

inline double
computePurePursuitRawLookahead(double speed_basis_mps,
                               double absolute_curvature_1pm,
                               const PurePursuitLookaheadPolicy &policy) {
  const double values[] = {speed_basis_mps,
                           absolute_curvature_1pm,
                           policy.gain,
                           policy.minimum_distance_m,
                           policy.curvature_minimum_distance_m,
                           policy.curvature_sensitivity};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return 0.0;
    }
  }
  const double speed_lookahead =
      policy.gain * std::max(0.0, speed_basis_mps) + policy.minimum_distance_m;
  return std::max(policy.curvature_minimum_distance_m,
                  speed_lookahead /
                      (1.0 + policy.curvature_sensitivity *
                                 std::max(0.0, absolute_curvature_1pm)));
}

struct PurePursuitPreviewSelection {
  bool valid{false};
  std::size_t lower_index{0U};
  std::size_t upper_index{0U};
  double interpolation_ratio{0.0};
  double x{0.0};
  double y{0.0};
  bool endpoint_fallback{false};
};

// Shared preview selection for both ROS trajectories and fixed-capacity MPPI
// references. Accessors keep the core independent of either representation.
template <typename Point, typename XAccessor, typename YAccessor>
PurePursuitPreviewSelection selectPurePursuitPreview(
    const Point *points, std::size_t count, std::size_t start_index,
    double requested_distance_m, double rear_x, double rear_y,
    bool continuous_arc_interpolation, XAccessor x_of, YAccessor y_of) {
  PurePursuitPreviewSelection result;
  if (points == nullptr || count == 0U ||
      !std::isfinite(requested_distance_m) || !std::isfinite(rear_x) ||
      !std::isfinite(rear_y)) {
    return result;
  }
  const std::size_t first = std::min(start_index, count - 1U);
  const double requested = std::max(0.0, requested_distance_m);
  result.lower_index = first;
  result.upper_index = first;
  result.x = x_of(points[first]);
  result.y = y_of(points[first]);
  if (!std::isfinite(result.x) || !std::isfinite(result.y)) {
    return PurePursuitPreviewSelection{};
  }
  if (!continuous_arc_interpolation) {
    for (std::size_t index = first; index < count; ++index) {
      const double x = x_of(points[index]);
      const double y = y_of(points[index]);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        return PurePursuitPreviewSelection{};
      }
      if (std::hypot(x - rear_x, y - rear_y) >= requested) {
        result.valid = true;
        result.lower_index = index;
        result.upper_index = index;
        result.x = x;
        result.y = y;
        return result;
      }
    }
    result.valid = true;
    result.lower_index = count - 1U;
    result.upper_index = count - 1U;
    result.x = x_of(points[count - 1U]);
    result.y = y_of(points[count - 1U]);
    result.endpoint_fallback = true;
    return result;
  }
  if (requested <= 1.0e-9) {
    result.valid = true;
    return result;
  }
  double accumulated = 0.0;
  for (std::size_t index = first + 1U; index < count; ++index) {
    const double lower_x = x_of(points[index - 1U]);
    const double lower_y = y_of(points[index - 1U]);
    const double upper_x = x_of(points[index]);
    const double upper_y = y_of(points[index]);
    const double segment = std::hypot(upper_x - lower_x, upper_y - lower_y);
    if (!std::isfinite(segment)) {
      return PurePursuitPreviewSelection{};
    }
    if (segment <= 1.0e-9) {
      continue;
    }
    if (accumulated + segment >= requested) {
      result.valid = true;
      result.lower_index = index - 1U;
      result.upper_index = index;
      result.interpolation_ratio =
          std::clamp((requested - accumulated) / segment, 0.0, 1.0);
      result.x = lower_x + result.interpolation_ratio * (upper_x - lower_x);
      result.y = lower_y + result.interpolation_ratio * (upper_y - lower_y);
      return result;
    }
    accumulated += segment;
  }
  result.valid = true;
  result.lower_index = count - 1U;
  result.upper_index = count - 1U;
  result.x = x_of(points[count - 1U]);
  result.y = y_of(points[count - 1U]);
  result.endpoint_fallback = true;
  return result;
}

// ROS-independent Pure Pursuit geometry shared by the 100 Hz controller and
// planner-side closed-loop rollouts.  Callers own progress/preview selection;
// this function owns the command geometry and the actuator execution bounds.
struct PurePursuitCoreInput {
  double rear_x{0.0};
  double rear_y{0.0};
  double yaw{0.0};
  double far_target_x{0.0};
  double far_target_y{0.0};
  double near_target_x{0.0};
  double near_target_y{0.0};
  double far_nominal_distance_m{0.0};
  double near_nominal_distance_m{0.0};
  double actual_distance_blend{0.0};
  double near_steering_blend{0.0};
  double wheel_base_m{0.0};
  double steering_gain{1.0};
  double target_velocity_mps{0.0};
  double curvature_feedforward_rad{0.0};
  double steering_reference_rad{0.0};
  double dt_sec{0.0};
  double hard_steering_angle_rad{0.0};
  double hard_steering_rate_radps{0.0};
  bool steering_passthrough_enabled{false};
};

struct PurePursuitCoreOutput {
  bool valid{false};
  double far_target_local_x{0.0};
  double far_target_local_y{0.0};
  double near_target_local_x{0.0};
  double near_target_local_y{0.0};
  double far_actual_distance_m{0.0};
  double near_actual_distance_m{0.0};
  double far_steering_distance_m{0.0};
  double near_steering_distance_m{0.0};
  double far_steering_rad{0.0};
  double near_steering_rad{0.0};
  double pure_pursuit_steering_rad{0.0};
  double requested_steering_rad{0.0};
  double bounded_steering_rad{0.0};
  double requested_rate_radps{0.0};
  double bounded_rate_radps{0.0};
  bool angle_limited{false};
  bool rate_limited{false};
};

inline PurePursuitCoreOutput
computePurePursuitCore(const PurePursuitCoreInput &input) {
  PurePursuitCoreOutput result;
  const double values[] = {
      input.rear_x,
      input.rear_y,
      input.yaw,
      input.far_target_x,
      input.far_target_y,
      input.near_target_x,
      input.near_target_y,
      input.far_nominal_distance_m,
      input.near_nominal_distance_m,
      input.actual_distance_blend,
      input.near_steering_blend,
      input.wheel_base_m,
      input.steering_gain,
      input.target_velocity_mps,
      input.curvature_feedforward_rad,
      input.steering_reference_rad,
      input.dt_sec,
      input.hard_steering_angle_rad,
      input.hard_steering_rate_radps,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return result;
    }
  }
  if (input.far_nominal_distance_m <= 1.0e-6 ||
      input.near_nominal_distance_m <= 1.0e-6 || input.wheel_base_m <= 1.0e-6 ||
      input.dt_sec <= 0.0 || input.hard_steering_angle_rad <= 0.0 ||
      input.hard_steering_rate_radps <= 0.0) {
    return result;
  }

  const double cos_yaw = std::cos(input.yaw);
  const double sin_yaw = std::sin(input.yaw);
  const double far_dx = input.far_target_x - input.rear_x;
  const double far_dy = input.far_target_y - input.rear_y;
  const double near_dx = input.near_target_x - input.rear_x;
  const double near_dy = input.near_target_y - input.rear_y;
  result.far_target_local_x = cos_yaw * far_dx + sin_yaw * far_dy;
  result.far_target_local_y = -sin_yaw * far_dx + cos_yaw * far_dy;
  result.near_target_local_x = cos_yaw * near_dx + sin_yaw * near_dy;
  result.near_target_local_y = -sin_yaw * near_dx + cos_yaw * near_dy;
  result.far_actual_distance_m = std::hypot(far_dx, far_dy);
  result.near_actual_distance_m = std::hypot(near_dx, near_dy);

  const double distance_blend =
      std::clamp(input.actual_distance_blend, 0.0, 1.0);
  result.far_steering_distance_m =
      (1.0 - distance_blend) * input.far_nominal_distance_m +
      distance_blend * result.far_actual_distance_m;
  result.near_steering_distance_m =
      (1.0 - distance_blend) * input.near_nominal_distance_m +
      distance_blend * result.near_actual_distance_m;
  if (result.far_steering_distance_m <= 1.0e-6 ||
      result.near_steering_distance_m <= 1.0e-6) {
    return result;
  }

  const double far_heading_error = std::atan2(far_dy, far_dx) - input.yaw;
  const double near_heading_error = std::atan2(near_dy, near_dx) - input.yaw;
  result.far_steering_rad =
      std::atan2(2.0 * input.wheel_base_m * std::sin(far_heading_error),
                 result.far_steering_distance_m);
  result.near_steering_rad =
      std::atan2(2.0 * input.wheel_base_m * std::sin(near_heading_error),
                 result.near_steering_distance_m);
  const double near_blend = std::clamp(input.near_steering_blend, 0.0, 1.0);
  const double forward_steering =
      input.steering_gain * ((1.0 - near_blend) * result.far_steering_rad +
                             near_blend * result.near_steering_rad);
  result.pure_pursuit_steering_rad = input.target_velocity_mps < -1.0e-3
                                         ? -forward_steering
                                         : forward_steering;
  result.requested_steering_rad =
      result.pure_pursuit_steering_rad + input.curvature_feedforward_rad;

  if (input.steering_passthrough_enabled) {
    result.bounded_steering_rad = result.requested_steering_rad;
  } else {
    const double bounded_reference =
        std::clamp(input.steering_reference_rad, -input.hard_steering_angle_rad,
                   input.hard_steering_angle_rad);
    const double angle_clamped = std::clamp(result.requested_steering_rad,
                                            -input.hard_steering_angle_rad,
                                            input.hard_steering_angle_rad);
    const double maximum_delta = input.hard_steering_rate_radps * input.dt_sec;
    const double rate_clamped =
        std::clamp(angle_clamped, bounded_reference - maximum_delta,
                   bounded_reference + maximum_delta);
    result.bounded_steering_rad =
        std::clamp(rate_clamped, -input.hard_steering_angle_rad,
                   input.hard_steering_angle_rad);
    result.angle_limited =
        std::abs(result.requested_steering_rad - angle_clamped) > 1.0e-12;
    result.rate_limited =
        std::abs(angle_clamped - result.bounded_steering_rad) > 1.0e-12;
  }
  const double rate_reference = input.steering_passthrough_enabled
                                    ? input.steering_reference_rad
                                    : std::clamp(input.steering_reference_rad,
                                                 -input.hard_steering_angle_rad,
                                                 input.hard_steering_angle_rad);
  result.requested_rate_radps =
      (result.requested_steering_rad - rate_reference) / input.dt_sec;
  result.bounded_rate_radps =
      (result.bounded_steering_rad - rate_reference) / input.dt_sec;
  result.valid = std::isfinite(result.bounded_steering_rad) &&
                 std::isfinite(result.requested_rate_radps) &&
                 std::isfinite(result.bounded_rate_radps);
  return result;
}

} // namespace simple_pure_pursuit
