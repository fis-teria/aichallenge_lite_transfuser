#include "simple_trajectory_generator/dual_reference.hpp"

#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace simple_trajectory_generator {
namespace {

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

constexpr double kMinimumSegmentLengthM = 1.0e-6;
constexpr double kTerminalDuplicateDistanceM = 1.0e-3;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

double planarDistance(const TrajectoryPoint &lhs, const TrajectoryPoint &rhs) {
  return std::hypot(rhs.pose.position.x - lhs.pose.position.x,
                    rhs.pose.position.y - lhs.pose.position.y);
}

bool finitePosition(const TrajectoryPoint &point) {
  return std::isfinite(point.pose.position.x) &&
         std::isfinite(point.pose.position.y) &&
         std::isfinite(point.pose.position.z);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw_rad) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw_rad);
  quaternion.w = std::cos(0.5 * yaw_rad);
  return quaternion;
}

builtin_interfaces::msg::Duration durationFromSeconds(const double seconds) {
  const auto nanoseconds = static_cast<std::int64_t>(
      std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  duration.nanosec = static_cast<std::uint32_t>(
      nanoseconds % kNanosecondsPerSecond);
  return duration;
}

TrajectoryPoint interpolatedPoint(const TrajectoryPoint &start,
                                  const TrajectoryPoint &end,
                                  const double ratio,
                                  const double execution_speed_mps) {
  TrajectoryPoint point;
  const double dx = end.pose.position.x - start.pose.position.x;
  const double dy = end.pose.position.y - start.pose.position.y;
  point.pose.position.x = start.pose.position.x + ratio * dx;
  point.pose.position.y = start.pose.position.y + ratio * dy;
  point.pose.position.z = start.pose.position.z +
                          ratio * (end.pose.position.z - start.pose.position.z);
  point.pose.orientation = quaternionFromYaw(std::atan2(dy, dx));
  point.longitudinal_velocity_mps =
      static_cast<float>(execution_speed_mps);
  point.lateral_velocity_mps = 0.0F;
  point.acceleration_mps2 = 0.0F;
  point.heading_rate_rps = 0.0F;
  return point;
}

} // namespace

CircularReferenceResult buildCircularReference(
    const Trajectory &source, const CircularReferenceConfig &config,
    const double anchor_x_m, const double anchor_y_m,
    const bool rotate_to_anchor) {
  CircularReferenceResult result;
  if (!std::isfinite(config.execution_speed_mps) ||
      config.execution_speed_mps <= 0.0 ||
      !std::isfinite(config.maximum_arc_spacing_m) ||
      config.maximum_arc_spacing_m <= 0.0 ||
      config.maximum_output_points < 2U) {
    result.reason = "invalid_config";
    return result;
  }
  if (source.points.size() < 2U) {
    result.reason = "too_few_source_points";
    return result;
  }

  std::vector<TrajectoryPoint> source_points(source.points.begin(),
                                              source.points.end());
  for (const auto &point : source_points) {
    if (!finitePosition(point)) {
      result.reason = "invalid_source_point";
      return result;
    }
  }
  if (planarDistance(source_points.front(), source_points.back()) <=
      kTerminalDuplicateDistanceM) {
    source_points.pop_back();
  }
  if (source_points.size() < 2U) {
    result.reason = "too_few_unique_source_points";
    return result;
  }

  if (rotate_to_anchor) {
    if (!std::isfinite(anchor_x_m) || !std::isfinite(anchor_y_m)) {
      result.reason = "invalid_anchor";
      return result;
    }
    const auto nearest = std::min_element(
        source_points.begin(), source_points.end(),
        [anchor_x_m, anchor_y_m](const auto &lhs, const auto &rhs) {
          const double lhs_distance =
              std::hypot(lhs.pose.position.x - anchor_x_m,
                         lhs.pose.position.y - anchor_y_m);
          const double rhs_distance =
              std::hypot(rhs.pose.position.x - anchor_x_m,
                         rhs.pose.position.y - anchor_y_m);
          return lhs_distance < rhs_distance;
        });
    std::rotate(source_points.begin(), nearest, source_points.end());
  }

  auto &reference = result.reference;
  reference.trajectory.header = source.header;
  reference.trajectory.points.reserve(std::min(
      config.maximum_output_points, source_points.size() * 8U));
  for (std::size_t source_index = 0U; source_index < source_points.size();
       ++source_index) {
    const auto &start = source_points[source_index];
    const auto &end = source_points[(source_index + 1U) % source_points.size()];
    const double distance_m = planarDistance(start, end);
    if (!std::isfinite(distance_m) || distance_m <= kMinimumSegmentLengthM) {
      result.reason = "degenerate_source_segment";
      result.reference = CircularReference{};
      return result;
    }
    const auto steps = static_cast<std::size_t>(
        std::ceil(distance_m / config.maximum_arc_spacing_m));
    if (steps == 0U || steps > config.maximum_output_points ||
        reference.trajectory.points.size() >
            config.maximum_output_points - steps) {
      result.reason = "output_point_limit_exceeded";
      result.reference = CircularReference{};
      return result;
    }
    for (std::size_t step = 0U; step < steps; ++step) {
      const double ratio =
          static_cast<double>(step) / static_cast<double>(steps);
      reference.trajectory.points.push_back(interpolatedPoint(
          start, end, ratio, config.execution_speed_mps));
    }
  }

  reference.cumulative_s_m.reserve(reference.trajectory.points.size());
  reference.cumulative_s_m.push_back(0.0);
  for (std::size_t index = 1U; index < reference.trajectory.points.size();
       ++index) {
    reference.cumulative_s_m.push_back(
        reference.cumulative_s_m.back() +
        planarDistance(reference.trajectory.points[index - 1U],
                       reference.trajectory.points[index]));
  }
  reference.length_m =
      reference.cumulative_s_m.back() +
      planarDistance(reference.trajectory.points.back(),
                     reference.trajectory.points.front());
  if (!std::isfinite(reference.length_m) ||
      reference.length_m <= kMinimumSegmentLengthM) {
    result.reason = "invalid_reference_length";
    result.reference = CircularReference{};
    return result;
  }
  result.valid = true;
  return result;
}

PathProjection projectToCircularReference(const CircularReference &reference,
                                          const double x_m,
                                          const double y_m) {
  PathProjection projection;
  if (reference.trajectory.points.empty() ||
      reference.cumulative_s_m.size() != reference.trajectory.points.size() ||
      !std::isfinite(x_m) || !std::isfinite(y_m)) {
    return projection;
  }
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < reference.trajectory.points.size();
       ++index) {
    const auto &position = reference.trajectory.points[index].pose.position;
    const double distance_m = std::hypot(position.x - x_m, position.y - y_m);
    if (distance_m < nearest_distance_m) {
      nearest_distance_m = distance_m;
      projection.nearest_index = index;
    }
  }
  projection.valid = std::isfinite(nearest_distance_m);
  projection.distance_m = nearest_distance_m;
  projection.s_m = reference.cumulative_s_m[projection.nearest_index];
  return projection;
}

Trajectory buildLocalCircularTrajectory(const CircularReference &reference,
                                        const std::size_t nearest_index,
                                        const double distance_behind_m,
                                        const double forward_distance_m,
                                        const std::size_t maximum_output_points) {
  Trajectory local;
  local.header = reference.trajectory.header;
  const auto &points = reference.trajectory.points;
  if (points.size() < 2U || nearest_index >= points.size() ||
      !std::isfinite(distance_behind_m) || distance_behind_m < 0.0 ||
      !std::isfinite(forward_distance_m) || forward_distance_m <= 0.0 ||
      maximum_output_points < 2U) {
    return local;
  }

  std::size_t start_index = nearest_index;
  double traversed_behind_m = 0.0;
  while (traversed_behind_m < distance_behind_m) {
    const std::size_t previous_index =
        (start_index + points.size() - 1U) % points.size();
    traversed_behind_m += planarDistance(points[previous_index],
                                         points[start_index]);
    start_index = previous_index;
    if (start_index == nearest_index) {
      break;
    }
  }

  const double target_length_m = traversed_behind_m + forward_distance_m;
  double traversed_m = 0.0;
  std::size_t current_index = start_index;
  while (local.points.size() < maximum_output_points) {
    TrajectoryPoint point = points[current_index];
    point.time_from_start = durationFromSeconds(
        traversed_m /
        std::max(0.1, static_cast<double>(point.longitudinal_velocity_mps)));
    local.points.push_back(point);
    if (traversed_m >= target_length_m && local.points.size() >= 2U) {
      break;
    }
    const std::size_t next_index = (current_index + 1U) % points.size();
    traversed_m += planarDistance(points[current_index], points[next_index]);
    current_index = next_index;
    if (current_index == start_index) {
      break;
    }
  }
  return local;
}

LapProgressTracker::LapProgressTracker(LapProgressConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.path_length_m) ||
      config_.path_length_m <= 0.0 ||
      !std::isfinite(config_.start_window_m) ||
      config_.start_window_m <= 0.0 ||
      !std::isfinite(config_.end_window_m) ||
      config_.end_window_m <= 0.0 ||
      !std::isfinite(config_.minimum_coverage_ratio) ||
      config_.minimum_coverage_ratio <= 0.0 ||
      config_.minimum_coverage_ratio > 1.0 ||
      !std::isfinite(config_.maximum_forward_step_m) ||
      config_.maximum_forward_step_m <= 0.0 ||
      !std::isfinite(config_.reverse_jitter_tolerance_m) ||
      config_.reverse_jitter_tolerance_m < 0.0) {
    throw std::invalid_argument("invalid lap progress configuration");
  }
}

void LapProgressTracker::reset() {
  initialized_ = false;
  start_armed_ = false;
  seam_armed_ = false;
  lap_complete_ = false;
  last_s_m_ = 0.0;
  last_measurement_time_sec_ = 0.0;
  accumulated_forward_m_ = 0.0;
}

LapProgressUpdate LapProgressTracker::update(
    const double s_m, const double measurement_time_sec) {
  LapProgressUpdate update;
  update.start_armed = start_armed_;
  update.seam_armed = seam_armed_;
  update.lap_complete = lap_complete_;
  update.accumulated_forward_m = accumulated_forward_m_;
  if (!std::isfinite(s_m) || s_m < 0.0 || s_m >= config_.path_length_m ||
      !std::isfinite(measurement_time_sec)) {
    update.reason = "invalid_measurement";
    return update;
  }
  if (!initialized_) {
    initialized_ = true;
    last_s_m_ = s_m;
    last_measurement_time_sec_ = measurement_time_sec;
    // The vehicle may spawn after reference point zero.  Credit the initial
    // projection as the already-traversed prefix of the first race lap so the
    // first valid end-to-zero crossing can complete that lap.  Forward-step,
    // reverse-motion, end-window, and minimum-coverage checks still protect
    // the crossing itself.
    start_armed_ = true;
    accumulated_forward_m_ = s_m;
    update.accepted = true;
    update.start_armed = start_armed_;
    update.accumulated_forward_m = accumulated_forward_m_;
    update.reason = s_m <= config_.start_window_m
                        ? "start_armed"
                        : "initial_progress_credited";
    return update;
  }
  if (measurement_time_sec <= last_measurement_time_sec_) {
    update.reason = "non_monotonic_time";
    return update;
  }

  const double raw_delta_m = s_m - last_s_m_;
  const bool forward_wrap = raw_delta_m < -0.5 * config_.path_length_m;
  const bool backward_wrap = raw_delta_m > 0.5 * config_.path_length_m;
  double forward_delta_m = raw_delta_m;
  if (forward_wrap) {
    forward_delta_m += config_.path_length_m;
  } else if (backward_wrap) {
    forward_delta_m -= config_.path_length_m;
  }

  if (!start_armed_) {
    if (forward_wrap || s_m <= config_.start_window_m) {
      start_armed_ = true;
      seam_armed_ = false;
      accumulated_forward_m_ = 0.0;
    }
    last_s_m_ = s_m;
    last_measurement_time_sec_ = measurement_time_sec;
    update.accepted = true;
    update.start_armed = start_armed_;
    update.reason = start_armed_ ? "start_armed" : "waiting_for_start";
    return update;
  }

  if (forward_delta_m < -config_.reverse_jitter_tolerance_m) {
    update.reason = "reverse_motion_rejected";
    return update;
  }
  if (forward_delta_m > config_.maximum_forward_step_m) {
    update.reason = "forward_jump_rejected";
    return update;
  }

  if (lap_complete_) {
    // The first lap completion remains latched, but a deferred reference
    // switch still needs one event at each later forward end-to-zero seam.
    // Rearm only in the configured end window so a low-s projection jitter
    // cannot repeatedly request a switch in the middle of a lap.
    if (s_m >= config_.path_length_m - config_.end_window_m) {
      seam_armed_ = true;
    }
    if (forward_wrap && seam_armed_) {
      update.seam_crossed = true;
      seam_armed_ = false;
    }
    last_s_m_ = s_m;
    last_measurement_time_sec_ = measurement_time_sec;
    update.accepted = true;
    update.start_armed = start_armed_;
    update.seam_armed = seam_armed_;
    update.lap_complete = true;
    update.accumulated_forward_m = accumulated_forward_m_;
    update.reason = update.seam_crossed ? "subsequent_seam_crossed"
                                        : "lap_already_complete";
    return update;
  }

  accumulated_forward_m_ += std::max(0.0, forward_delta_m);
  if (s_m >= config_.path_length_m - config_.end_window_m) {
    seam_armed_ = true;
  }
  const double minimum_coverage_m =
      config_.path_length_m * config_.minimum_coverage_ratio;
  if (forward_wrap && seam_armed_ &&
      accumulated_forward_m_ >= minimum_coverage_m) {
    lap_complete_ = true;
    update.seam_crossed = true;
    seam_armed_ = false;
  }
  last_s_m_ = s_m;
  last_measurement_time_sec_ = measurement_time_sec;
  update.accepted = true;
  update.start_armed = start_armed_;
  update.seam_armed = seam_armed_;
  update.lap_complete = lap_complete_;
  update.accumulated_forward_m = accumulated_forward_m_;
  update.reason = lap_complete_ ? "lap_complete" : "progress_accepted";
  return update;
}

} // namespace simple_trajectory_generator
