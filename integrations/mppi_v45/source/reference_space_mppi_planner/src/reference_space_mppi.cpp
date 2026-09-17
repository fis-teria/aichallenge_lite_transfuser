#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/execution_motion.hpp"
#include "reference_space_mppi_planner/wall_line_cost.hpp"
#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include "reference_space_mppi_planner/ot_lane_entry_planner.hpp"
#include "reference_space_mppi_planner/maneuver_policy.hpp"

#include "reference_space_mppi_planner/steering_delay.hpp"
#include "reference_space_mppi_planner/cma_preview_curvature.hpp"
#include "reference_space_mppi_planner/local_horizon.hpp"
#include "reference_space_mppi_planner/spatial_noise.hpp"
#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include "reference_space_mppi_planner/entry_connector.hpp"
#include "reference_space_mppi_planner/overtake_objective.hpp"
#include "reference_space_mppi_planner/passing_progress.hpp"
#include "reference_space_mppi_planner/collision_uncertainty.hpp"

#include "simple_pure_pursuit/pure_pursuit_core.hpp"
#include "simple_pure_pursuit/execution_bounds.hpp"
#include "simple_pure_pursuit/steering_actuator_model.hpp"
#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/curvature_feedforward.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace reference_space_mppi_planner::mppi {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double value) { return std::isfinite(value); }

double stagedSpeedBasis(const PlanRequest &request) {
  double speed = 0.0;
  for (std::size_t i = 0; i < request.base_reference_count; ++i)
    speed = std::max(speed, request.base_reference[i].speed_mps);
  return speed;
}

double normalizeAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

std::array<double, kParameterCount> asArray(const Parameters &parameters) {
  return {{parameters.d_pass_m, parameters.l_out_m, parameters.l_hold_m,
           parameters.l_merge_m, parameters.speed_scale,
           parameters.lateral_control_near_scale,
           parameters.lateral_control_far_scale}};
}

Parameters fromArray(const std::array<double, kParameterCount> &values) {
  return Parameters{values[0], values[1], values[2], values[3],
                    values[4], values[5], values[6]};
}

bool finiteParameters(const Parameters &parameters) {
  for (const double value : asArray(parameters)) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

class DeterministicGaussian {
public:
  explicit DeterministicGaussian(std::uint64_t seed) : state_(seed) {}

  double next() {
    if (spare_valid_) {
      spare_valid_ = false;
      return spare_;
    }
    const double u1 = std::max(nextUniform(), 1.0e-15);
    const double u2 = nextUniform();
    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = 2.0 * kPi * u2;
    spare_ = radius * std::sin(angle);
    spare_valid_ = true;
    return radius * std::cos(angle);
  }

private:
  std::uint64_t nextBits() {
    state_ += 0x9e3779b97f4a7c15ULL;
    std::uint64_t value = state_;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  }

  double nextUniform() {
    return static_cast<double>(nextBits() >> 11U) * (1.0 / 9007199254740992.0);
  }

  std::uint64_t state_{0U};
  double spare_{0.0};
  bool spare_valid_{false};
};

double distance(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

double pointToSegmentDistance(double x, double y,
                              const TemporaryReferencePoint &a,
                              const TemporaryReferencePoint &b) {
  const double segment_x = b.x_m - a.x_m;
  const double segment_y = b.y_m - a.y_m;
  const double length_squared = segment_x * segment_x + segment_y * segment_y;
  if (length_squared <= 1.0e-12) {
    return distance(x, y, a.x_m, a.y_m);
  }
  const double ratio = std::clamp(
      ((x - a.x_m) * segment_x + (y - a.y_m) * segment_y) / length_squared, 0.0,
      1.0);
  return distance(x, y, a.x_m + ratio * segment_x, a.y_m + ratio * segment_y);
}

double crossTrackDistance(const TemporaryReference &reference,
                          std::size_t nearest, double x, double y) {
  nearest = std::min(nearest, reference.count - 1U);
  double best = distance(x, y, reference.points[nearest].x_m,
                         reference.points[nearest].y_m);
  if (nearest > 0U) {
    best = std::min(best,
                    pointToSegmentDistance(x, y, reference.points[nearest - 1U],
                                           reference.points[nearest]));
  }
  if (nearest + 1U < reference.count) {
    best =
        std::min(best, pointToSegmentDistance(x, y, reference.points[nearest],
                                              reference.points[nearest + 1U]));
  }
  return best;
}

// Geometric tracking only: do not feed this projection back into controller
// preview or route progress (a self-approaching path can have multiple branches).
double geometricTrackingDistance(const TemporaryReference &reference,
                                 double x, double y) {
  double best = distance(x, y, reference.points[0].x_m, reference.points[0].y_m);
  for (std::size_t i = 1; i < reference.count; ++i) {
    best = std::min(best, pointToSegmentDistance(
        x, y, reference.points[i - 1], reference.points[i]));
  }
  return best;
}

double normalizedTrackingExcess(double error, double tolerance) {
  return std::max(0.0, error - tolerance) / tolerance;
}

bool parameterActiveForWarmCost(const PlanRequest &request, std::size_t index) {
  switch (index) {
  case 0U: // d_pass
  case 1U: // l_out
  case 5U: // near lateral shape
  case 6U: // far lateral shape
    return request.phase != Phase::MERGE;
  case 2U: // l_hold
    return request.complete_maneuver && request.phase != Phase::MERGE;
  case 3U: // l_merge
    return request.complete_maneuver || request.phase == Phase::MERGE;
  case 4U: // speed_scale
    return true;
  default:
    return false;
  }
}

double signedCurvature(const TemporaryReferencePoint &a,
                       const TemporaryReferencePoint &b,
                       const TemporaryReferencePoint &c) {
  const double ab = distance(a.x_m, a.y_m, b.x_m, b.y_m);
  const double bc = distance(b.x_m, b.y_m, c.x_m, c.y_m);
  const double ca = distance(c.x_m, c.y_m, a.x_m, a.y_m);
  const double denominator = ab * bc * ca;
  if (!finite(denominator) || denominator <= 1.0e-9) {
    return 0.0;
  }
  const double twice_area =
      (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
  return 2.0 * twice_area / denominator;
}

std::size_t advanceNearestReference(const TemporaryReference &reference,
                                    std::size_t current, double x, double y) {
  current = std::min(current, reference.count - 1U);
  double best = distance(x, y, reference.points[current].x_m,
                         reference.points[current].y_m);
  while (current + 1U < reference.count) {
    const double next = distance(x, y, reference.points[current + 1U].x_m,
                                 reference.points[current + 1U].y_m);
    if (next > best + 1.0e-9) {
      break;
    }
    best = next;
    ++current;
  }
  return current;
}

struct ProjectedState {
  double s_m{0.0};
  double d_m{0.0};
};

double intervalSeparation(double minimum_a, double maximum_a, double minimum_b,
                          double maximum_b) {
  if (maximum_a < minimum_b) {
    return minimum_b - maximum_a;
  }
  if (maximum_b < minimum_a) {
    return minimum_a - maximum_b;
  }
  return 0.0;
}

void recordRejection(Evaluation *evaluation, RejectReason reason,
                     std::size_t step_index, std::size_t reference_index,
                     double s_m, double d_m) {
  evaluation->valid = false;
  evaluation->reject_reason = reason;
  evaluation->reject_step_index = step_index;
  evaluation->reject_reference_index = reference_index;
  evaluation->reject_s_m = s_m;
  evaluation->reject_d_m = d_m;
}

struct DynamicEnvelope {
  double minimum_s_m{0.0};
  double maximum_s_m{0.0};
  double minimum_d_m{0.0};
  double maximum_d_m{0.0};
};

struct ProjectedHalfExtents {
  double longitudinal_m{0.0};
  double lateral_m{0.0};
};

ProjectedHalfExtents projectedHalfExtents(double relative_yaw_rad,
                                          const Config &config) {
  const double cosine = std::abs(std::cos(relative_yaw_rad));
  const double sine = std::abs(std::sin(relative_yaw_rad));
  return ProjectedHalfExtents{cosine * config.vehicle_half_length_m +
                                  sine * config.vehicle_half_width_m,
                              sine * config.vehicle_half_length_m +
                                  cosine * config.vehicle_half_width_m};
}

DynamicEnvelope predictEnvelope(const DynamicObstacle &obstacle,
                                double time_sec, double ego_relative_yaw_rad,
                                const Config &config) {
  const double t = std::max(0.0, time_sec);
  const auto predicted = opponent_prediction::positionAt(obstacle, t);
  const double center_s = predicted.s;
  const double center_d = predicted.d;
  // Project both oriented vehicle rectangles into the local Frenet axes. The
  // former point-vs-axis-aligned-envelope check only represented two parallel
  // vehicles and missed the nose sweep while the passing kart was turning.
  const auto ego_extent = projectedHalfExtents(ego_relative_yaw_rad, config);
  const auto obstacle_extent =
      projectedHalfExtents(predicted.relative_yaw, config);
  // A bounded-acceleration tube remains conservative for every constant
  // acceleration in [-bound, +bound] over the MPPI horizon.
  const double longitudinal_half_extent =
      std::max(config.obstacle_longitudinal_inflation_m,
               ego_extent.longitudinal_m + obstacle_extent.longitudinal_m) +
      std::max(0.0, obstacle.longitudinal_uncertainty_m) +
      0.5 * std::max(0.0, obstacle.longitudinal_acceleration_bound_mps2) * t *
          t;
  const double lateral_half_extent =
      std::max(config.obstacle_lateral_inflation_m,
               ego_extent.lateral_m + obstacle_extent.lateral_m) +
      std::max(0.0, obstacle.lateral_uncertainty_m);
  return DynamicEnvelope{
      center_s - longitudinal_half_extent, center_s + longitudinal_half_extent,
      center_d - lateral_half_extent, center_d + lateral_half_extent};
}

DynamicObstacle collisionObstacle(DynamicObstacle obstacle, double time_sec) {
  const double scale = collisionUncertaintyScale(time_sec);
  obstacle.longitudinal_acceleration_bound_mps2 *= scale;
  obstacle.longitudinal_uncertainty_m *= scale;
  obstacle.lateral_uncertainty_m *= scale;
  return obstacle;
}

bool finiteExecutionInput(const PlanRequest &request) {
  if (!request.valid || request.base_reference_count < 3U ||
      request.base_reference_count > kMaximumReferencePoints ||
      request.static_obstacle_count > kMaximumStaticObstacles ||
      request.dynamic_obstacle_count > kMaximumDynamicObstacles ||
      request.ego.pending_steering_target_count > kMaximumHorizonSteps) return false;
  const auto &e = request.ego;
  for (double value : {request.stamp_sec, e.x_m, e.y_m, e.yaw_rad,
       e.speed_mps, e.steering_rad, e.acceleration_mps2, e.s_m, e.d_m}) {
    if (!finite(value)) return false;
  }
  if (e.speed_mps < 0.0) return false;
  for (std::size_t i=0; i<e.pending_steering_target_count; ++i)
    if (!finite(e.pending_steering_targets_rad[i])) return false;
  for (std::size_t i=0; i<request.base_reference_count; ++i) {
    const auto &b=request.base_reference[i];
    for(double v : {b.s_m,b.x_m,b.y_m,b.yaw_rad,b.curvature_1pm,
                   b.speed_mps,b.minimum_d_m,b.maximum_d_m})
      if(!finite(v)) return false;
    if(b.minimum_d_m>b.maximum_d_m || b.speed_mps<0.0 ||
       (i>0 && b.s_m<=request.base_reference[i-1].s_m) ||
       (b.active_d_valid && !finite(b.active_d_m)) ||
       (b.active_speed_valid && (!finite(b.active_speed_mps) || b.active_speed_mps<0.0)))
      return false;
  }
  for(std::size_t i=0;i<request.static_obstacle_count;++i) {
    const auto &o=request.static_obstacles[i];
    for(double v:{o.minimum_s_m,o.maximum_s_m,o.minimum_d_m,o.maximum_d_m})
      if(!finite(v)) return false;
    if(o.minimum_s_m>o.maximum_s_m || o.minimum_d_m>o.maximum_d_m) return false;
  }
  for(std::size_t i=0;i<request.dynamic_obstacle_count;++i) {
    const auto &o=request.dynamic_obstacles[i];
    for(double v:{o.s_m,o.d_m,o.longitudinal_speed_mps,o.lateral_speed_mps,
        o.longitudinal_acceleration_bound_mps2,o.longitudinal_uncertainty_m,
        o.lateral_uncertainty_m,o.heading_relative_to_reference_rad})
      if(!finite(v)) return false;
    if(o.longitudinal_acceleration_bound_mps2<0 || o.longitudinal_uncertainty_m<0 ||
       o.lateral_uncertainty_m<0) return false;
  }
  return true;
}

// Projection onto segments, not a waypoint's rotating tangent. s remains the
// immutable Reference station; it is never replaced by candidate arc length.
std::size_t baseIndexAtProjection(const PlanRequest &request,
                                  const SegmentProjection &projection) {
  const auto lower = projection.index;
  const auto upper = lower + 1U;
  // Boundary and corridor fields are waypoint samples, not continuous wall
  // geometry. Use the nearest station within the very segment that supplied
  // d; a separate greedy point search can remain pinned behind that segment.
  // Ties select the upper station, as the old advancing search did.
  return projection.s - request.base_reference[lower].s_m <
                 request.base_reference[upper].s_m - projection.s
             ? lower : upper;
}
double phaseNoiseScale(Phase phase) {
  switch (phase) {
  case Phase::PREPARE:
    return 1.0;
  case Phase::OVERTAKE:
    return 0.55;
  case Phase::PASS_CLEAR:
    return 0.35;
  case Phase::MERGE:
    return 0.50;
  }
  return 1.0;
}

double radicalInverse(std::size_t value, std::size_t base) {
  double result = 0.0;
  double inverse_base = 1.0 / static_cast<double>(base);
  while (value > 0U) {
    result += static_cast<double>(value % base) * inverse_base;
    value /= base;
    inverse_base /= static_cast<double>(base);
  }
  return result;
}

double interpolateControlKnot(
    const std::array<double, kMaximumControlKnotCount> &values,
    std::size_t count, double normalized_position) {
  if (count == 0U) {
    return 0.0;
  }
  if (count == 1U) {
    return values[0U];
  }
  const double position = std::clamp(normalized_position, 0.0, 1.0) *
                          static_cast<double>(count - 1U);
  const std::size_t lower =
      std::min(static_cast<std::size_t>(std::floor(position)), count - 1U);
  const std::size_t upper = std::min(lower + 1U, count - 1U);
  const double ratio = position - static_cast<double>(lower);
  const double smooth_ratio = ReferenceSpaceMppiPlanner::quinticBlend(ratio);
  return values[lower] + smooth_ratio * (values[upper] - values[lower]);
}

} // namespace

std::optional<double> approachSpeedTarget(const DynamicObstacle &obstacle,
    double ego_s_m, double ego_d_m, double ego_relative_yaw_rad,
    double time_sec, const Config &config) {
  const auto position = opponent_prediction::positionAt(obstacle, time_sec);
  if (position.s <= ego_s_m) return std::nullopt;
  // Use the forecast body for a speed preference. Prediction uncertainty is
  // already handled by the unchanged collision and clearance evaluations.
  auto nominal = obstacle;
  nominal.longitudinal_uncertainty_m = nominal.lateral_uncertainty_m = 0.0;
  nominal.longitudinal_acceleration_bound_mps2 = 0.0;
  const auto body = predictEnvelope(nominal, time_sec, ego_relative_yaw_rad, config);
  if (ego_d_m <= body.minimum_d_m || ego_d_m >= body.maximum_d_m)
    return std::nullopt;
  double lead_speed = obstacle.longitudinal_speed_mps;
  if (obstacle.prediction && obstacle.prediction->points.size() >= 2U) {
    const auto &points = obstacle.prediction->points;
    auto upper = std::upper_bound(points.begin(), points.end(), time_sec,
        [](double t, const auto &p) { return t < p.time; });
    if (upper == points.begin()) ++upper;
    if (upper == points.end()) --upper;
    const auto &lower = *std::prev(upper);
    lead_speed = (upper->global_s - lower.global_s) / (upper->time - lower.time);
  }
  const double gap = body.minimum_s_m - ego_s_m;
  const double follow_gap=followingGapForSpeed(
      std::hypot(obstacle.longitudinal_speed_mps,obstacle.lateral_speed_mps),
      config.follow_gap_m,config.overtake_follow_gap_m);
  if (config.approach_at_follow_gap_only && gap > follow_gap)
    return std::nullopt;
  return std::max(0.0, lead_speed + (gap - follow_gap) / config.approach_time_sec);
}

ReferenceSpaceMppiPlanner::ReferenceSpaceMppiPlanner(Config config)
    : config_(config) {}

const char *ReferenceSpaceMppiPlanner::validateConfig() const {
  if (config_.sample_count < 4U || config_.sample_count > kMaximumSampleCount ||
      config_.sample_count % 2U != 0U) {
    return "sample_count_must_be_even_and_within_capacity";
  }
  if (config_.horizon_steps < 2U ||
      config_.horizon_steps > kMaximumHorizonSteps) {
    return "horizon_steps_out_of_range";
  }
  if (config_.control_knot_count < 2U ||
      config_.control_knot_count > kMaximumControlKnotCount) {
    return "control_knot_count_out_of_range";
  }
  if (!finite(config_.dt_sec) || config_.dt_sec <= 0.0 ||
      !finite(config_.temperature) || config_.temperature <= 0.0 ||
      !finite(config_.update_gain) || config_.update_gain < 0.0 ||
      config_.update_gain > 1.0 || !finite(config_.minimum_valid_ratio) ||
      config_.minimum_valid_ratio < 0.0 || config_.minimum_valid_ratio > 1.0 ||
      config_.minimum_valid_count > config_.sample_count) {
    return "invalid_sampling_parameters";
  }
  for (const double sigma : config_.sigma) {
    if (!finite(sigma) || sigma < 0.0) {
      return "invalid_parameter_sigma";
    }
  }
  const double positive_values[] = {
      config_.approach_time_sec,
      config_.reference_spacing_m,
      config_.maximum_reference_segment_m,
      config_.final_swept_check_step_m,
      config_.wheel_base_m,
      config_.lookahead_gain,
      config_.lookahead_min_distance_m,
      config_.curvature_lookahead_min_distance_m,
      config_.maneuver_lookahead_time_constant_sec,
      config_.steering_gain,
      config_.steering_time_constant_sec,
      config_.cma_prediction_steering_time_constant_sec,
      config_.yaw_response_time_constant_sec,
      config_.steering_command_to_tire_angle_ratio,
      config_.maximum_steering_angle_rad,
      config_.maximum_tire_steering_angle_rad,
      config_.maximum_steering_rate_radps,
      config_.maximum_lateral_acceleration_mps2,
      config_.maximum_cross_track_error_m,
      config_.speed_proportional_gain,
      config_.acceleration_time_constant_sec,
      config_.maximum_acceleration_mps2,
      config_.maximum_deceleration_mps2,
      config_.vehicle_half_length_m,
      config_.vehicle_half_width_m,
      config_.clearance_target_m,
      config_.control_entry_ramp_m,
  };
  for (const double value : positive_values) {
    if (!finite(value) || value <= 0.0) {
      return "invalid_vehicle_or_reference_parameter";
    }
  }
  if (!finite(config_.curvature_lookahead_smoothing_alpha) ||
      config_.curvature_lookahead_smoothing_alpha < 0.0 ||
      config_.curvature_lookahead_smoothing_alpha > 1.0) {
    return "invalid_lookahead_smoothing";
  }
  const double unit_interval_values[] = {
      config_.actual_lookahead_distance_blend,
      config_.dual_preview_near_ratio,
      config_.dual_preview_blend,
      config_.steering_command_to_tire_angle_ratio,
  };
  for (const double value : unit_interval_values) {
    if (!finite(value) || value < 0.0 || value > 1.0) {
      return "invalid_preview_parameter";
    }
  }
  const double nonnegative_values[] = {
      config_.physical_tire_steering_rate_radps,
      config_.cma_curvature_preview_distance_m,
      config_.curvature_lookahead_sensitivity,
      config_.curvature_feedforward_gain,
      config_.steering_control_delay_sec,
      config_.cma_prediction_delay_sec,
      config_.steering_acceleration_hold_minimum_speed_mps,
      config_.steering_acceleration_hold_minimum_tire_angle_rad,
      config_.steering_acceleration_hold_tracking_error_rad,
      config_.steering_acceleration_hold_maximum_acceleration_mps2,
      config_.freeze_guard_time_sec,
      config_.freeze_guard_min_distance_m,
      config_.obstacle_longitudinal_inflation_m,
      config_.obstacle_lateral_inflation_m,
      config_.cost_progress_weight,
      config_.cost_terminal_lateral_weight,
      config_.cost_clearance_weight,
      config_.cost_approach_speed_weight,
      config_.cost_passing_progress_weight,
      config_.cost_passing_opportunity_weight,
      config_.follow_gap_m,
      config_.overtake_follow_gap_m,
      config_.cost_reference_weight,
      config_.cost_curvature_weight,
      config_.cost_pp_error_weight,
      config_.cost_steering_rate_weight,
      config_.cost_reference_change_weight,
      config_.cost_pass_separation_weight,
      config_.cost_wall_line_weight,
      config_.control_lateral_std_m,
      config_.control_lateral_reference_spacing_m,
      config_.control_speed_scale_std,
      config_.control_maximum_lateral_adjustment_m,
      config_.control_maximum_speed_scale_adjustment,
      config_.cost_control_smoothness_weight,
      config_.cost_control_change_weight,
      config_.history_lateral_cost_scale,
  };
  for (const double value : nonnegative_values) {
    if (!finite(value) || value < 0.0) {
      return "invalid_nonnegative_parameter";
    }
  }
  if (!finite(config_.control_noise_correlation) ||
      config_.control_noise_correlation < 0.0 ||
      config_.control_noise_correlation >= 1.0) {
    return "control_noise_correlation_out_of_range";
  }
  if (config_.steering_acceleration_hold_minimum_tire_angle_rad >
      config_.maximum_tire_steering_angle_rad) {
    return "steering_acceleration_hold_threshold_out_of_range";
  }
  if (config_.allow_reference_switch && config_.shadow_only) {
    return "reference_switch_forbidden_in_shadow_mode";
  }
  return nullptr;
}

void ReferenceSpaceMppiPlanner::reset() {
  warm_mean_ = Parameters{};
  warm_mean_valid_ = false;
  warm_side_ = 0;
  warm_phase_ = Phase::PREPARE;
  warm_semantic_key_ = 0U;
  warm_control_sequence_ = ReferenceControlSequence{};
  warm_control_sequence_valid_ = false;
  warm_stamp_sec_ = 0.0;
  warm_base_reference_count_ = 0U;
}

double ReferenceSpaceMppiPlanner::quinticBlend(double normalized_s) {
  const double u = std::clamp(normalized_s, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  return u3 * (10.0 + u * (-15.0 + 6.0 * u));
}

double ReferenceSpaceMppiPlanner::multiPointLateralBlend(
    double normalized_s, double near_control_scale, double far_control_scale) {
  const double u = std::clamp(normalized_s, 0.0, 1.0);
  // Degree seven gives three equal endpoint control points. Therefore lateral
  // position, slope and curvature join the measured/terminal profiles without
  // a discontinuity, while P3 and P4 remain independent MPPI search variables.
  std::array<double, 8U> controls{
      {0.0, 0.0, 0.0, std::clamp(near_control_scale, 0.0, 1.0),
       std::clamp(far_control_scale, 0.0, 1.0), 1.0, 1.0, 1.0}};
  for (std::size_t remaining = controls.size() - 1U; remaining > 0U;
       --remaining) {
    for (std::size_t index = 0U; index < remaining; ++index) {
      controls[index] = (1.0 - u) * controls[index] + u * controls[index + 1U];
    }
  }
  return controls[0U];
}

double ReferenceSpaceMppiPlanner::activeManeuverLength(Phase phase,
                                                       double shift_length_m,
                                                       double merge_length_m) {
  return phase == Phase::MERGE ? merge_length_m : shift_length_m;
}

const char *ReferenceSpaceMppiPlanner::toString(RejectReason reason) {
  switch (reason) {
  case RejectReason::NONE:
    return "none";
  case RejectReason::DISABLED:
    return "disabled";
  case RejectReason::INVALID_CONFIG:
    return "invalid_config";
  case RejectReason::INVALID_INPUT:
    return "invalid_input";
  case RejectReason::UNKNOWN_SIDE:
    return "unknown_side";
  case RejectReason::PARAMETER:
    return "parameter";
  case RejectReason::GEOMETRY:
    return "geometry";
  case RejectReason::TRACK:
    return "track";
  case RejectReason::COLLISION:
    return "collision";
  case RejectReason::WALL:
    return "wall";
  case RejectReason::VEHICLE_LIMIT:
    return "vehicle_limit";
  case RejectReason::PP_TRACKABILITY:
    return "pp_trackability";
  case RejectReason::NON_FINITE:
    return "non_finite";
  case RejectReason::NO_VALID_SAMPLE:
    return "no_valid_sample";
  case RejectReason::COUNT:
    return "count";
  }
  return "unknown";
}

Parameters ReferenceSpaceMppiPlanner::project(const Parameters &parameters,
                                              const ParameterBounds &bounds,
                                              int side) const {
  const auto values = asArray(parameters);
  const auto minimum = asArray(bounds.minimum);
  const auto maximum = asArray(bounds.maximum);
  std::array<double, kParameterCount> projected{};
  for (std::size_t index = 0U; index < kParameterCount; ++index) {
    projected[index] =
        std::clamp(values[index], minimum[index], maximum[index]);
  }
  if (side > 0) {
    projected[0] = std::max(0.0, projected[0]);
  } else if (side < 0) {
    projected[0] = std::min(0.0, projected[0]);
  }
  return fromArray(projected);
}

void ReferenceSpaceMppiPlanner::generateNoise(const PlanRequest &request,
                                              std::size_t sample_count,
                                              Scratch *scratch) const {
  for (auto &sample : scratch->noise) {
    sample.fill(0.0);
  }
  DeterministicGaussian gaussian(config_.deterministic_seed ^
                                 (request.generation * 0x9e3779b97f4a7c15ULL));
  const double sampling_scale = phaseNoiseScale(request.phase);
  const std::size_t pair_count = (sample_count - 2U) / 2U;
  for (std::size_t pair = 0U; pair < pair_count; ++pair) {
    const std::size_t positive = 1U + 2U * pair;
    const std::size_t negative = positive + 1U;
    for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
      // Optimize terminal separation, transition length/speed and two
      // interior lateral control points. L_hold and L_merge stay fixed.
      const bool active_mvp_dimension = parameter == 0U || parameter == 1U ||
                                        parameter == 4U || parameter == 5U ||
                                        parameter == 6U;
      const double value =
          active_mvp_dimension
              ? gaussian.next() * config_.sigma[parameter] * sampling_scale
              : 0.0;
      scratch->noise[positive][parameter] = value;
      scratch->noise[negative][parameter] = -value;
    }
  }
  const std::size_t unpaired = sample_count - 1U;
  for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
    const bool active_mvp_dimension = parameter == 0U || parameter == 1U ||
                                      parameter == 4U || parameter == 5U ||
                                      parameter == 6U;
    scratch->noise[unpaired][parameter] =
        active_mvp_dimension
            ? gaussian.next() * config_.sigma[parameter] * sampling_scale
            : 0.0;
  }
}

ReferenceControlSequence ReferenceSpaceMppiPlanner::projectControlSequence(
    const ReferenceControlSequence &sequence) const {
  ReferenceControlSequence projected;
  projected.count = std::min(sequence.count, config_.control_knot_count);
  for (std::size_t knot = 0U; knot < projected.count; ++knot) {
    projected.lateral_adjustment_m[knot] =
        std::clamp(sequence.lateral_adjustment_m[knot],
                   -config_.control_maximum_lateral_adjustment_m,
                   config_.control_maximum_lateral_adjustment_m);
    projected.speed_scale_adjustment[knot] =
        std::clamp(sequence.speed_scale_adjustment[knot],
                   -config_.control_maximum_speed_scale_adjustment,
                   config_.control_maximum_speed_scale_adjustment);
  }
  return projected;
}

ReferenceControlSequence ReferenceSpaceMppiPlanner::controlSequenceMean(
    const PlanRequest &request) const {
  ReferenceControlSequence mean;
  mean.count = config_.control_knot_count;
  if (!request.sample_control_sequence) return mean;
  const double lateral_span = std::max(1.0,
      request.base_reference[request.base_reference_count-1].s_m-request.anchor_s_m);
  const double origin = request.base_reference[0].s_m;
  const double speed_span = std::max(1.0,
      request.base_reference[request.base_reference_count-1].s_m-origin);
  const bool same = warm_control_sequence_valid_ && warm_side_==request.side &&
      warm_phase_==request.phase && warm_semantic_key_==request.semantic_key &&
      warm_control_sequence_.count==mean.count && request.stamp_sec>=warm_stamp_sec_ &&
      warm_base_reference_count_>=2U;
  if (same) {
    const auto &start=request.base_reference[0];
    const auto projection=projectOnBase(warm_base_reference_.data(),
        warm_base_reference_count_,start.x_m,start.y_m);
    // A changed/unrelated map is not transportable warm state. This only
    // discards a proposal; every resulting candidate still requires validation.
    if (finite(projection.error) && projection.error<=1e-6) {
      const double travelled=projection.s-warm_base_reference_[0].s_m;
      for(std::size_t k=0;k<mean.count;++k) {
        const double u=static_cast<double>(k)/(mean.count-1U);
        const double lateral_u=previousKnotCoordinate(u,lateral_span,
            travelled+request.anchor_s_m-warm_anchor_s_m_,warm_control_horizon_m_);
        const double speed_u=request.sample_staged_speed ? std::sqrt(
            previousKnotCoordinate(u*u,speed_span,travelled,warm_speed_horizon_m_)) :
            previousKnotCoordinate(u,lateral_span,
                travelled+request.anchor_s_m-warm_anchor_s_m_,warm_control_horizon_m_);
        mean.lateral_adjustment_m[k]=interpolateControlKnot(
            warm_control_sequence_.lateral_adjustment_m,mean.count,lateral_u);
        mean.speed_scale_adjustment[k]=interpolateControlKnot(
            warm_control_sequence_.speed_scale_adjustment,mean.count,speed_u);
      }
    }
  }
  // All modes see the same actually published speed field, rather than their
  // own hypothetical winner. It is a seed, not a frozen speed command.
  if(request.sample_staged_speed && request.base_reference[0].active_speed_valid) {
    const double baseline=warm_mean_valid_ && warm_side_==request.side &&
        warm_phase_==request.phase && warm_semantic_key_==request.semantic_key ?
        project(warm_mean_,request.bounds,request.side).speed_scale :
        project(request.nominal,request.bounds,request.side).speed_scale;
    const double nominal=stagedSpeedBasis(request);
    std::size_t i=1;
    for(std::size_t k=0;k<mean.count;++k) {
      const double u=static_cast<double>(k)/(mean.count-1U);
      const double station=origin+u*u*speed_span;
      while(i+1<request.base_reference_count && request.base_reference[i].s_m<station) ++i;
      const auto &a=request.base_reference[i-1]; const auto &b=request.base_reference[i];
      if(!a.active_speed_valid || !b.active_speed_valid) continue;
      const double t=std::clamp((station-a.s_m)/(b.s_m-a.s_m),0.0,1.0);
      const double active=a.active_speed_mps+t*(b.active_speed_mps-a.active_speed_mps);
      if(nominal>1e-9) mean.speed_scale_adjustment[k]=active/nominal-baseline;
    }
  }
  return projectControlSequence(mean);
}

void ReferenceSpaceMppiPlanner::generateControlNoise(const PlanRequest &request,
                                                     std::size_t sample_count,
                                                     Scratch *scratch) const {
  for (auto &sample : scratch->lateral_control_noise) {
    sample.fill(0.0);
  }
  for (auto &sample : scratch->speed_control_noise) {
    sample.fill(0.0);
  }
  if (!request.sample_control_sequence) {
    return;
  }

  DeterministicGaussian gaussian(config_.deterministic_seed ^
                                 0x6a09e667f3bcc909ULL ^
                                 (request.generation * 0x9e3779b97f4a7c15ULL));
  const double correlation = config_.control_noise_correlation;
  const double span = std::max(1.0,
      request.base_reference[request.base_reference_count - 1U].s_m - request.anchor_s_m);
  const double spatial_ratio = spatialKnotRatio(span, config_.control_knot_count,
      config_.control_lateral_reference_spacing_m);
  const double lateral_correlation = spatialCorrelation(correlation, spatial_ratio);
  const double lateral_innovation = std::sqrt(std::max(0.0,
      1.0 - lateral_correlation * lateral_correlation));
  const double innovation_scale =
      std::sqrt(std::max(0.0, 1.0 - correlation * correlation));
  const double phase_scale = phaseNoiseScale(request.phase);
  const auto fill_sample = [&](std::size_t positive, std::size_t negative,
                               bool antithetic) {
    double previous_lateral = 0.0;
    double previous_speed = 0.0;
    for (std::size_t knot = 0U; knot < config_.control_knot_count; ++knot) {
      previous_lateral =
          lateral_correlation * previous_lateral + lateral_innovation * gaussian.next();
      previous_speed =
          correlation * previous_speed + innovation_scale * gaussian.next();
      const double lateral =
          config_.control_lateral_std_m * spatialLateralAmplitude(spatial_ratio) *
          phase_scale * previous_lateral;
      const double speed =
          config_.control_speed_scale_std * phase_scale * previous_speed;
      scratch->lateral_control_noise[positive][knot] = lateral;
      scratch->speed_control_noise[positive][knot] = speed;
      if (antithetic) {
        scratch->lateral_control_noise[negative][knot] = -lateral;
        scratch->speed_control_noise[negative][knot] = -speed;
      }
    }
  };
  const std::size_t pair_count = (sample_count - 2U) / 2U;
  for (std::size_t pair = 0U; pair < pair_count; ++pair) {
    const std::size_t positive = 1U + 2U * pair;
    fill_sample(positive, positive + 1U, true);
  }
  fill_sample(sample_count - 1U, sample_count - 1U, false);
}

bool ReferenceSpaceMppiPlanner::generateReference(
    const Parameters &parameters, const PlanRequest &request,
    const ReferenceControlSequence *control_sequence,
    TemporaryReference *reference, Evaluation *evaluation) const {
  reference->count = 0U;
  reference->overtake_start_source_s_m=std::numeric_limits<double>::infinity();
  reference->wall_connection_end_index=0U;
  reference->front_merge_end_index=0U;
  if(request.connect_to_wall_line && (!request.precomputed_wall_lines ||
      request.precomputed_wall_lines->world!=request.world_reference)) {
    recordRejection(evaluation,RejectReason::INVALID_INPUT,0U,0U,request.anchor_s_m,request.anchor_d_m);
    return false;
  }
  if (!finiteParameters(parameters) || request.base_reference_count < 3U ||
      request.base_reference_count > kMaximumReferencePoints) {
    recordRejection(evaluation, RejectReason::INVALID_INPUT, 0U, 0U,
                    request.anchor_s_m, request.anchor_d_m);
    return false;
  }
  const double staged_speed_basis = request.sample_staged_speed ? stagedSpeedBasis(request) : 0.0;
  const auto preparation = request.front_merge_attack ? request.passing_preparation : nullptr;
  std::optional<double> preparation_origin;
  if(preparation && preparation->front_merge) {
    preparation_origin=preparationStation(*preparation,request.ego.x_m,request.ego.y_m);
    if(!preparation_origin) {
      recordRejection(evaluation,RejectReason::INVALID_INPUT,0U,0U,request.ego.s_m,request.ego.d_m);
      return false;
    }
  }
  if (request.side == 0) {
    recordRejection(evaluation, RejectReason::UNKNOWN_SIDE, 0U, 0U,
                    request.anchor_s_m, request.anchor_d_m);
    return false;
  }
  if ((request.side > 0 && parameters.d_pass_m < -1.0e-9) ||
      (request.side < 0 && parameters.d_pass_m > 1.0e-9) ||
      parameters.l_out_m <= config_.reference_spacing_m ||
      parameters.l_merge_m <= config_.reference_spacing_m ||
      parameters.l_hold_m < 0.0 || parameters.speed_scale < 0.0 ||
      parameters.lateral_control_near_scale < 0.0 ||
      parameters.lateral_control_near_scale > 1.0 ||
      parameters.lateral_control_far_scale < 0.0 ||
      parameters.lateral_control_far_scale > 1.0 ||
      parameters.lateral_control_near_scale >
          parameters.lateral_control_far_scale) {
    recordRejection(evaluation, RejectReason::PARAMETER, 0U, 0U,
                    request.anchor_s_m, request.anchor_d_m);
    return false;
  }
  if (control_sequence != nullptr) {
    if (control_sequence->count != config_.control_knot_count) {
      recordRejection(evaluation, RejectReason::PARAMETER, 0U, 0U,
                      request.anchor_s_m, request.anchor_d_m);
      return false;
    }
    for (std::size_t knot = 0U; knot < control_sequence->count; ++knot) {
      if (!finite(control_sequence->lateral_adjustment_m[knot]) ||
          !finite(control_sequence->speed_scale_adjustment[knot])) {
        recordRejection(evaluation, RejectReason::NON_FINITE, 0U, 0U,
                        request.anchor_s_m, request.anchor_d_m);
        return false;
      }
    }
  }

  const double control_horizon_m = std::max(
      1.0, request.base_reference[request.base_reference_count - 1U].s_m -
               request.anchor_s_m);

  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    const auto &base = request.base_reference[index];
    const double relative_s = base.s_m - request.anchor_s_m;
    const double active_d =
        base.active_d_valid ? base.active_d_m : request.anchor_d_m;
    const double profile_origin_d_m = base.pass_origin_d_valid
                                          ? base.pass_origin_d_m
                                          : request.pass_profile_origin_d_m;
    const double profile_scale_m =
        std::abs(request.pass_profile_scale_m) > 1.0e-6
            ? request.pass_profile_scale_m
            : request.nominal.d_pass_m;
    const double unclamped_target_d =
        base.pass_d_valid && std::abs(profile_scale_m) > 1.0e-6
            ? profile_origin_d_m + (base.pass_d_m - profile_origin_d_m) *
                                       (parameters.d_pass_m / profile_scale_m)
            : base.corridor_d_m + parameters.d_pass_m -
                  request.corridor_nominal_d_m;
    const double target_d =
        base.pass_d_valid
            ? std::clamp(unclamped_target_d, base.minimum_d_m, base.maximum_d_m)
            : unclamped_target_d;
    double offset = request.connect_to_wall_line ? 0. : active_d;
    if (relative_s > 0.0 && !request.connect_to_wall_line) {
      if (request.phase == Phase::MERGE) {
        offset = active_d + (base.corridor_d_m - active_d) *
                                quinticBlend(relative_s / parameters.l_merge_m);
      } else if (relative_s < parameters.l_out_m) {
        offset = active_d + (target_d - active_d) *
                                multiPointLateralBlend(
                                    relative_s / parameters.l_out_m,
                                    parameters.lateral_control_near_scale,
                                    parameters.lateral_control_far_scale);
      } else if (request.complete_maneuver &&
                 relative_s < parameters.l_out_m + parameters.l_hold_m) {
        offset = target_d;
      } else if (request.complete_maneuver &&
                 relative_s < parameters.l_out_m + parameters.l_hold_m +
                                  parameters.l_merge_m) {
        const double merge_s =
            relative_s - parameters.l_out_m - parameters.l_hold_m;
        offset = target_d + (base.corridor_d_m - target_d) *
                                quinticBlend(merge_s / parameters.l_merge_m);
      } else if (request.complete_maneuver) {
        offset = base.corridor_d_m;
      } else {
        offset = target_d;
      }
    }
    double sampled_speed_adjustment = 0.0;
    if (control_sequence != nullptr &&
        (relative_s >= 0.0 || request.sample_staged_speed)) {
      const double normalized_position = relative_s / control_horizon_m;
      const double lateral_adjustment =
          interpolateControlKnot(control_sequence->lateral_adjustment_m,
                                 control_sequence->count, normalized_position);
      sampled_speed_adjustment =
          interpolateControlKnot(control_sequence->speed_scale_adjustment,
                                 control_sequence->count,
                                 request.sample_staged_speed ?
                                     std::sqrt(std::clamp(
                                         (base.s_m-request.base_reference[0].s_m) /
                                         std::max(1.0,request.base_reference[request.base_reference_count-1].s_m-
                                                      request.base_reference[0].s_m),0.0,1.0)) :
                                     normalized_position);
      const double entry_envelope = quinticBlend(
          std::clamp(relative_s / config_.control_entry_ramp_m, 0.0, 1.0));
      double exit_envelope = 1.0;
      if (request.phase == Phase::MERGE) {
        exit_envelope = quinticBlend(std::clamp(
            (parameters.l_merge_m - relative_s) / config_.control_entry_ramp_m,
            0.0, 1.0));
      } else if (request.complete_maneuver) {
        const double maneuver_end_m =
            parameters.l_out_m + parameters.l_hold_m + parameters.l_merge_m;
        exit_envelope = quinticBlend(std::clamp(
            (maneuver_end_m - relative_s) / config_.control_entry_ramp_m, 0.0,
            1.0));
      }
      // Apply the bounded target through the entry envelope as well. Clamping
      // the measured prefix itself can move the first point away from ego.
      const double bounded_target = std::clamp(
          offset + lateral_adjustment, base.minimum_d_m, base.maximum_d_m);
      if(!request.connect_to_wall_line)
        offset += entry_envelope * exit_envelope * (bounded_target - offset);
    }
    if (!finite(base.s_m) || !finite(base.x_m) || !finite(base.y_m) ||
        !finite(base.yaw_rad) || !finite(base.curvature_1pm) ||
        !finite(base.speed_mps) || !finite(base.minimum_d_m) ||
        !finite(base.maximum_d_m) || base.minimum_d_m > base.maximum_d_m) {
      recordRejection(evaluation, RejectReason::TRACK, 0U, index, base.s_m,
                      offset);
      return false;
    }
    const double frenet_jacobian = 1.0 - base.curvature_1pm * offset;
    if (offset < base.minimum_d_m - 1.0e-9 ||
        offset > base.maximum_d_m + 1.0e-9) {
      evaluation->reject_minimum_d_m = base.minimum_d_m;
      evaluation->reject_maximum_d_m = base.maximum_d_m;
      evaluation->reject_curvature_1pm = base.curvature_1pm;
      evaluation->reject_frenet_jacobian = frenet_jacobian;
      if (!config_.collision_only_rejection) {
        recordRejection(evaluation, RejectReason::TRACK, 0U, index, base.s_m,
                        offset);
        return false;
      }
    }
    auto &point = reference->points[index];
    point.s_m = base.s_m;
    point.d_m = offset;
    point.x_m = base.x_m - std::sin(base.yaw_rad) * offset;
    point.y_m = base.y_m + std::cos(base.yaw_rad) * offset;
    if(request.connect_to_wall_line) {
      const auto xy=request.precomputed_wall_lines->at(
          request.wall_line_origin_station_m+base.s_m,request.side);
      if(!xy) {
        recordRejection(evaluation,RejectReason::INVALID_INPUT,0U,index,base.s_m,offset);
        return false;
      }
      point.x_m=(*xy)[0];point.y_m=(*xy)[1];
      if(request.front_merge_attack) {
        // The preparation owns these world stations. Independent entry/hold
        // samples must not move the passing point or the return interval.
        const double merge_start=preparation_origin ?
            preparation->merge_start_station_m-*preparation_origin : parameters.l_out_m+parameters.l_hold_m;
        const double merge_length=preparation_origin ?
            preparation->merge_end_station_m-preparation->merge_start_station_m : parameters.l_merge_m;
        const double ratio=preparation_origin ? 1. :
            std::clamp(parameters.d_pass_m/request.pass_profile_scale_m,0.,1.);
        const double blend=1.-quinticBlend(std::clamp((base.s_m-merge_start)/merge_length,0.,1.));
        point.x_m=base.x_m+ratio*blend*(point.x_m-base.x_m);
        point.y_m=base.y_m+ratio*blend*(point.y_m-base.y_m);
        if(!reference->front_merge_end_index && base.s_m>=merge_start+merge_length)
          reference->front_merge_end_index=index;
      }
      point.d_m=-(point.x_m-base.x_m)*std::sin(base.yaw_rad)+
                 (point.y_m-base.y_m)*std::cos(base.yaw_rad);
    }
    point.yaw_rad = base.yaw_rad;
    point.curvature_1pm = base.curvature_1pm;
    // Do not use base-reference curvature for the final speed contract. The
    // laterally shifted generated path can have materially different
    // curvature, while the base can also contain one-point discretization
    // spikes. Apply the physical cap after generated curvature is available.
    const double sampled_speed_scale = std::clamp(
        parameters.speed_scale + sampled_speed_adjustment,
        request.bounds.minimum.speed_scale, request.bounds.maximum.speed_scale);
    point.speed_mps =
        std::max(request.minimum_speed_mps,
                 std::max(0.0, sampled_speed_scale *
                     (request.sample_staged_speed ? staged_speed_basis : base.speed_mps)));
    point.uncapped_speed_mps = point.speed_mps;
    reference->count = index + 1U;
  }

  if(request.cartesian_measured_prefix) {
    // Geometry is independent of the speed proposal. The delayed PP model
    // below still evaluates the generated path without any post-check repair.
    double entry=request.connect_to_wall_line ? parameters.l_out_m :
        std::max(request.anchor_s_m-request.base_reference[0].s_m,config_.control_entry_ramp_m);
    const auto full_count=reference->count;
    if(preparation_origin) {
      const bool entering=preparation->entry_station_m-*preparation_origin>config_.reference_spacing_m;
      const double boundary=entering ? preparation->entry_station_m :
          preparation->merge_start_station_m>*preparation_origin ? preparation->merge_start_station_m :
          preparation->merge_end_station_m;
      std::size_t last_join=1;
      while(last_join+2<full_count && reference->points[last_join].s_m<boundary-*preparation_origin)
        ++last_join;
      // The connector normally searches farther ahead if a join fails. Keep
      // it within this preparation phase; entry must end at the planned point.
      reference->count=last_join+2;
      if(entering) entry=std::numeric_limits<double>::infinity();
    }
    std::size_t end = 0;
    const bool connected=entry_connector::connect(request.ego, config_, entry, reference, &end);
    reference->count=full_count;
    if (!connected) {
      recordRejection(evaluation, RejectReason::GEOMETRY, 0, 0,
                      request.ego.s_m, request.ego.d_m);
      evaluation->reject_stage="entry_connection";
      return false;
    }
    if(request.connect_to_wall_line) reference->wall_connection_end_index=end;
    for(std::size_t i=0;i<end;++i) {
      auto &point=reference->points[i];
      point.d_m=projectOnBase(request.base_reference.data(),request.base_reference_count,point.x_m,point.y_m).d;
    }
  }

  for (std::size_t index = 0U; index < reference->count; ++index) {
    const std::size_t lower = index == 0U ? 0U : index - 1U;
    const std::size_t upper =
        index + 1U < reference->count ? index + 1U : reference->count - 1U;
    if (lower != upper) {
      const auto &a = reference->points[lower];
      const auto &b = reference->points[upper];
      reference->points[index].yaw_rad =
          std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
    }
  }
  if(request.cartesian_measured_prefix) reference->points[0].yaw_rad=request.ego.yaw_rad;
  for (std::size_t index = 1U; index < reference->count; ++index) {
    const auto &previous = reference->points[index - 1U];
    const auto &current = reference->points[index];
    const auto &base = request.base_reference[index];
    const double dx = current.x_m - previous.x_m;
    const double dy = current.y_m - previous.y_m;
    const double segment_m = std::hypot(dx, dy);
    const double forward_m =
        std::cos(base.yaw_rad) * dx + std::sin(base.yaw_rad) * dy;
    if (!finite(segment_m)) {
      recordRejection(evaluation, RejectReason::NON_FINITE, 0U, index,
                      current.s_m, current.d_m);
      return false;
    }
    if (!config_.collision_only_rejection &&
        (segment_m > config_.maximum_reference_segment_m ||
         forward_m < -0.02)) {
      recordRejection(evaluation, RejectReason::GEOMETRY, 0U, index,
                      current.s_m, current.d_m);
      return false;
    }
  }
  for (std::size_t index = 1U; index + 1U < reference->count; ++index) {
    reference->points[index].curvature_1pm =
        signedCurvature(reference->points[index - 1U], reference->points[index],
                        reference->points[index + 1U]);
    if (!finite(reference->points[index].curvature_1pm)) {
      recordRejection(evaluation, RejectReason::NON_FINITE, 0U, index,
                      reference->points[index].s_m,
                      reference->points[index].d_m);
      return false;
    }
  }
  if (reference->count >= 3U) {
    reference->points.front().curvature_1pm =
        reference->points[1U].curvature_1pm;
    reference->points[reference->count - 1U].curvature_1pm =
        reference->points[reference->count - 2U].curvature_1pm;
  }
  for (std::size_t index = 0U; index < reference->count; ++index) {
    // Staged proposals are command speeds, as in execution-speed search.
    // The controller/vehicle rollout validates their achieved response.
    if (request.sample_staged_speed || config_.longitudinal_planning_enabled) break;
    const double curvature_speed = std::sqrt(
        config_.maximum_lateral_acceleration_mps2 /
        std::max(std::abs(reference->points[index].curvature_1pm), 1.0e-4));
    // minimum_speed_mps is a tactical preference, not permission to violate
    // the vehicle's lateral-acceleration limit.
    reference->points[index].speed_mps =
        std::min(reference->points[index].speed_mps, curvature_speed);
  }
  if (config_.longitudinal_planning_enabled) {
    if(request.phase==Phase::OVERTAKE || request.phase==Phase::PASS_CLEAR)
      markOvertakeStart(*reference);
    auto speed = planLongitudinalSpeed(*reference, request, config_);
    if (!speed.valid) {
      evaluation->reject_reason = RejectReason::INVALID_INPUT;
      evaluation->reject_stage = "longitudinal_planning";
      return false;
    }
    *reference = std::move(speed.reference);
  }
  evaluation->reject_reason = RejectReason::NONE;
  return true;
}

Evaluation ReferenceSpaceMppiPlanner::evaluate(
    const Parameters &parameters, const PlanRequest &request,
    const ReferenceControlSequence *control_sequence,
    TemporaryReference *reference) const {
  Evaluation result;
  if (!finiteExecutionInput(request)) return result;
  result.reject_stage="generation";
  if (!generateReference(parameters,request,control_sequence,reference,&result)) {
    return result;
  }
  result=evaluateReference(parameters,request,control_sequence,reference);
  if(config_.longitudinal_planning_enabled && request.passing_preparation &&
      !request.passing_preparation->front_merge) {
    auto prepared=planLongitudinalSpeed(*reference,request,config_,true);
    bool different=prepared.valid && prepared.reference.count==reference->count;
    if(different) {
      different=false;
      for(std::size_t i=0;i<reference->count;++i)
        different|=prepared.reference.points[i].speed_mps!=reference->points[i].speed_mps;
    }
    if(different) {
      const auto alternative=evaluateReference(parameters,request,control_sequence,&prepared.reference);
      if(alternative.valid && (!result.valid || alternative.cost<result.cost)) {
        *reference=std::move(prepared.reference);
        result=alternative;
      }
    }
  }
  return result;
}

Evaluation ReferenceSpaceMppiPlanner::validateReference(
    const TemporaryReference &reference, const PlanRequest &request) const {
  Evaluation result;
  if(validateConfig()!=nullptr || !finiteExecutionInput(request) ||
     horizonSteps(request)<2U || horizonSteps(request)>config_.horizon_steps)
    return result;
  return evaluateReference(request.nominal,request,nullptr,&reference);
}

Evaluation ReferenceSpaceMppiPlanner::evaluateExecutionReference(
    const TemporaryReference &reference, const PlanRequest &request) const {
  if (!finiteExecutionInput(request) || reference.count > kMaximumReferencePoints)
    return Evaluation{};
  auto expressed = reference;
  for (std::size_t i = 0; i < expressed.count; ++i) {
    auto &p = expressed.points[i];
    // Order is a property of this Cartesian path, not of its projection onto
    // a finite polyline (whose corners and endpoints can have repeated s).
    p.s_m = i == 0 ? 0. : expressed.points[i-1].s_m +
        std::hypot(p.x_m-expressed.points[i-1].x_m, p.y_m-expressed.points[i-1].y_m);
  }
  return evaluateOrderedExecutionReference(expressed, request, nullptr);
}

std::function<Evaluation(const TemporaryReference &)>
ReferenceSpaceMppiPlanner::makeExecutionSpeedEvaluator(
    const TemporaryReference &reference, const PlanRequest &request) const {
  if (!finiteExecutionInput(request) || reference.count < 3U ||
      reference.count > kMaximumReferencePoints)
    return [](const TemporaryReference &) { return Evaluation{}; };
  auto ordered = reference;
  for (std::size_t i=0; i<ordered.count; ++i) {
    auto &p=ordered.points[i];
    p.s_m=i==0 ? 0. : ordered.points[i-1].s_m +
      std::hypot(p.x_m-ordered.points[i-1].x_m,p.y_m-ordered.points[i-1].y_m);
  }
  PreparedExecutionGeometry geometry;
  geometry.base_projection=std::make_shared<const BaseProjectionIndex>(
      request.base_reference.data(),request.base_reference_count);
  const auto field=projectReferenceField(ordered,request,geometry.base_projection.get());
  if (config_.cma_lookahead_curvature_enabled)
    geometry.preview_curvatures=cmaPreviewCurvatures(ordered,config_.cma_curvature_preview_distance_m);
  if (request.sample_staged_speed)
    geometry.curvature_cost=spatialCurvatureCost(ordered,request);
  geometry.wall_line_cost=wallLineCost(field,request);
  auto speed_request = request;
  if (request.path_constraint_geometry_only && request.path_constraint_validator &&
      !request.path_constraint_execution_prefix) {
    speed_request.path_constraint_validator =
        [validate = request.path_constraint_validator,
         cached = std::optional<RejectReason>{}](const TemporaryReference &path) mutable {
          if (!cached) cached = validate(path);
          return *cached;
        };
  }
  struct Context {
    ReferenceSpaceMppiPlanner planner;
    PlanRequest request;
    TemporaryReference ordered, field;
    PreparedExecutionGeometry geometry;
  };
  ExecutionValidators validators{speed_request.path_constraint_validator,
                                 speed_request.rollout_constraint_validator};
  auto context=std::make_shared<const Context>(
      Context{*this,std::move(speed_request),ordered,field,std::move(geometry)});
  // Copies for parallel proposals share immutable geometry/input only.
  // Mutable callback state remains local to each evaluator copy.
  return [context=std::move(context),validators=std::move(validators)](const TemporaryReference &speeds) {
    if (speeds.count != context->ordered.count) return Evaluation{};
    auto candidate=context->ordered;
    auto candidate_field=context->field;
    for (std::size_t i=0; i<candidate.count; ++i) {
      candidate.points[i].speed_mps=speeds.points[i].speed_mps;
      candidate.points[i].uncapped_speed_mps=speeds.points[i].uncapped_speed_mps;
      candidate_field.points[i].speed_mps=speeds.points[i].speed_mps;
      candidate_field.points[i].uncapped_speed_mps=speeds.points[i].uncapped_speed_mps;
    }
    return context->planner.evaluateOrderedExecutionReference(candidate,context->request,
        &candidate_field,&context->geometry,&validators);
  };
}

Evaluation ReferenceSpaceMppiPlanner::evaluateOrderedExecutionReference(
    const TemporaryReference &expressed, const PlanRequest &request,
    const TemporaryReference *field, const PreparedExecutionGeometry *geometry,
    const ExecutionValidators *validators) const {
  if(validateConfig()!=nullptr || !finiteExecutionInput(request) ||
     horizonSteps(request)<2U || horizonSteps(request)>config_.horizon_steps)
    return Evaluation{};
  auto result = evaluateReference(request.nominal,request,nullptr,&expressed,field,geometry,validators);
  if (!result.valid) return result;
  for (std::size_t i = 0; i < result.predicted_rollout_count; ++i) {
    const auto &state = result.predicted_rollout[i];
    const auto projected = projectExecutionTrajectory(expressed, state.x_m, state.y_m);
    if (projected.lower+2 != expressed.count || projected.ratio < 1.) continue;
    const auto &a = expressed.points[expressed.count-2];
    const auto &b = expressed.points[expressed.count-1];
    const double beyond = ((state.x_m-b.x_m)*(b.x_m-a.x_m)+
        (state.y_m-b.y_m)*(b.y_m-a.y_m))/std::hypot(b.x_m-a.x_m,b.y_m-a.y_m);
    if (beyond > 1e-3) {
      result.valid = false;
      result.reject_reason = RejectReason::INVALID_INPUT;
      result.reject_stage = "execution_reference_exhausted";
      result.cost = std::numeric_limits<double>::infinity();
      result.overtake_time_sec = std::numeric_limits<double>::infinity();
      break;
    }
  }
  return result;
}

double ReferenceSpaceMppiPlanner::spatialCurvatureCost(
    const TemporaryReference &reference, const PlanRequest &request) const {
  // A shared spatial prefix avoids rewarding delayed arrival at a curve.
  double integral=0., distance=0.;
  for (std::size_t i=1; i<reference.count; ++i) {
    const auto &a=reference.points[i-1];const auto &b=reference.points[i];
    const double ds=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m);
    if(ds<=1e-12)continue;
    const double used=std::min(ds,std::max(0.,request.curvature_evaluation_distance_m-distance));
    if(used<=0.)break;
    const double ka=5*a.curvature_1pm,kb=5*b.curvature_1pm;
    const double end_squared=ka*ka+(kb*kb-ka*ka)*(used/ds);
    integral+=.5*(ka*ka+end_squared)*used;
    distance+=used;
    if(used<ds)break;
  }
  return config_.cost_curvature_weight*integral/std::max(distance,1e-6);
}

bool visitExecutionMotion(
    const TemporaryReference &path,const PlanRequest &request,const Config &config,
    std::size_t steps,const std::function<bool(const ExecutionMotionSample &)> &visitor,
    const std::array<double,kMaximumReferencePoints> *prepared_curvatures) {
  if(path.count<3U || path.count>path.points.size() || !visitor) return false;
  const auto *reference=&path;
  const auto &config_=config;
  double x = request.ego.x_m;
  double y = request.ego.y_m;
  double yaw = request.ego.yaw_rad;
  double speed = std::max(0.0, request.ego.speed_mps);
  // SteeringReport is a physical tire angle. The command history and PP
  // output are actuator-side targets and stay in that domain until the AWSIM
  // ratio is applied after the pure command delay.
  double steering = std::clamp(request.ego.steering_rad,
                               -config_.maximum_tire_steering_angle_rad,
                               config_.maximum_tire_steering_angle_rad);
  double commanded_steering = simple_pure_pursuit::tireToCommandSteeringAngle(
      steering, config_.steering_command_to_tire_angle_ratio,
      config_.maximum_steering_angle_rad);
  SteeringCommandDelayLine<kMaximumHorizonSteps> steering_delay_line;
  steering_delay_line.reset(commanded_steering,
                            request.ego.pending_steering_targets_rad.data(),
                            request.ego.pending_steering_target_count);
  double acceleration = request.ego.acceleration_mps2;
  double yaw_rate = request.ego.yaw_rate_radps;
  auto rotation_state = request.rotation_prediction.state;
  AwsimLongitudinalResponse longitudinal_response;
  std::size_t reference_index = 0U;
  double smoothed_curvature = 0.0;
  double smoothed_lookahead = 0.0;
  const auto new_preview_curvatures = prepared_curvatures ? *prepared_curvatures : config_.cma_lookahead_curvature_enabled
      ? cmaPreviewCurvatures(*reference, config_.cma_curvature_preview_distance_m)
      : std::array<double, kMaximumReferencePoints>{};
  const auto prior=request.reference_activation_delay_sec>0. && request.prior_execution_reference &&
      request.prior_execution_reference->count>=3U ? request.prior_execution_reference.get() : nullptr;
  const auto prior_preview_curvatures=prior && config_.cma_lookahead_curvature_enabled ?
      cmaPreviewCurvatures(*prior,config_.cma_curvature_preview_distance_m) :
      std::array<double,kMaximumReferencePoints>{};
  double comparison_feedforward_curvature = 0.0;
  bool preview_state_initialized = false;
  const simple_pure_pursuit::PurePursuitLookaheadPolicy lookahead_policy{
      config_.lookahead_gain, config_.lookahead_min_distance_m,
      config_.curvature_lookahead_min_distance_m,
      config_.curvature_lookahead_sensitivity};

  for (std::size_t step = 0U; step < steps; ++step) {
    const auto selected=prior && step*config_.dt_sec<request.reference_activation_delay_sec ? prior : &path;
    if(selected!=reference) reference_index=0U;
    reference=selected;
    const auto &preview_curvatures=reference==prior ? prior_preview_curvatures : new_preview_curvatures;
    ExecutionMotionSample sample;
    sample.before=RolloutState{
        x, y, yaw, speed, static_cast<double>(step) * config_.dt_sec};
    const std::size_t physical_reference_index = reference_index;
    double rear_x = x - config_.wheel_base_m * 0.5 * std::cos(yaw);
    double rear_y = y - config_.wheel_base_m * 0.5 * std::sin(yaw);
    double control_yaw = yaw;
    if (config_.compare_cma_delay_compensation && speed >= 0.20) {
      const auto predicted = simple_pure_pursuit::predictDelayedPose(
          {rear_x, rear_y, yaw, speed}, steering,
          simple_pure_pursuit::commandToTireSteeringAngle(commanded_steering,
              config_.steering_command_to_tire_angle_ratio,
              config_.maximum_tire_steering_angle_rad),
          config_.cma_prediction_delay_sec, 0.02,
          config_.cma_prediction_steering_time_constant_sec, config_.wheel_base_m,
          config_.maximum_tire_steering_angle_rad,
          simple_pure_pursuit::physicalTireSteeringRate(
              config_.physical_tire_steering_rate_radps,
              config_.maximum_steering_rate_radps, config_.steering_command_to_tire_angle_ratio));
      rear_x = predicted.x;
      rear_y = predicted.y;
      control_yaw = predicted.yaw;
    }
    if (config_.compare_cma_delay_compensation) {
      const double control_x = rear_x + config_.wheel_base_m * 0.5 * std::cos(control_yaw);
      const double control_y = rear_y + config_.wheel_base_m * 0.5 * std::sin(control_yaw);
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < reference->count; ++i) {
        const auto &p = reference->points[i];
        const double squared = (p.x_m-control_x)*(p.x_m-control_x) +
                               (p.y_m-control_y)*(p.y_m-control_y);
        if (squared < best) { best = squared; reference_index = i; }
      }
    } else {
      reference_index = advanceNearestReference(*reference, reference_index, x, y);
    }
    const auto &nearest_reference = reference->points[reference_index];
    const double target_speed = nearest_reference.speed_mps;
    const double current_curvature = config_.cma_lookahead_curvature_enabled
        ? preview_curvatures[reference_index]
        : std::abs(nearest_reference.curvature_1pm);
    if (!preview_state_initialized) {
      smoothed_curvature = current_curvature;
    } else {
      smoothed_curvature =
          config_.curvature_lookahead_smoothing_alpha * current_curvature +
          (1.0 - config_.curvature_lookahead_smoothing_alpha) *
              smoothed_curvature;
    }
    const double raw_lookahead =
        simple_pure_pursuit::computePurePursuitRawLookahead(
            speed, smoothed_curvature, lookahead_policy);
    if (!preview_state_initialized) {
      smoothed_lookahead = raw_lookahead;
      preview_state_initialized = true;
    } else {
      const double lookahead_alpha =
          1.0 - std::exp(-config_.dt_sec /
                         config_.maneuver_lookahead_time_constant_sec);
      smoothed_lookahead +=
          lookahead_alpha * (raw_lookahead - smoothed_lookahead);
    }
    const double lookahead = smoothed_lookahead;
    const double near_lookahead =
        std::max(0.5, lookahead * config_.dual_preview_near_ratio);
    const auto far_target = simple_pure_pursuit::selectPurePursuitPreview(
        reference->points.data(), reference->count, reference_index, lookahead,
        rear_x, rear_y, config_.continuous_preview_interpolation_enabled,
        [](const TemporaryReferencePoint &point) { return point.x_m; },
        [](const TemporaryReferencePoint &point) { return point.y_m; });
    const auto near_target = simple_pure_pursuit::selectPurePursuitPreview(
        reference->points.data(), reference->count, reference_index,
        near_lookahead, rear_x, rear_y,
        config_.continuous_preview_interpolation_enabled,
        [](const TemporaryReferencePoint &point) { return point.x_m; },
        [](const TemporaryReferencePoint &point) { return point.y_m; });
    if (!far_target.valid || !near_target.valid) {
      sample.rejection=RejectReason::PP_TRACKABILITY;
      visitor(sample);
      return false;
    }
    double feedforward_curvature = nearest_reference.curvature_1pm;
    if (config_.compare_cma_preview_feedforward) {
      struct ReferenceView {
        const TemporaryReference &r;
        std::size_t size() const { return r.count; }
        const TemporaryReferencePoint &operator[](std::size_t i) const { return r.points[i]; }
      };
      const double preview = simple_pure_pursuit::estimateDistanceWindowSignedCurvature(
          ReferenceView{*reference}, reference_index, 3.0,
          [](const auto &p) { return std::pair<double,double>{p.x_m,p.y_m}; });
      comparison_feedforward_curvature = config_.curvature_feedforward_enabled ?
          simple_pure_pursuit::timeDomainLowPass(preview,
              comparison_feedforward_curvature, config_.dt_sec, 0.20) : 0.0;
      feedforward_curvature = comparison_feedforward_curvature;
    }
    const double tire_curvature_feedforward =
        config_.curvature_feedforward_enabled
            ? config_.curvature_feedforward_gain *
                  std::atan(config_.wheel_base_m *
                            feedforward_curvature)
            : 0.0;
    const double command_curvature_feedforward =
        tire_curvature_feedforward /
        config_.steering_command_to_tire_angle_ratio;
    const simple_pure_pursuit::PurePursuitCoreInput pp_input{
        rear_x,
        rear_y,
        control_yaw,
        far_target.x,
        far_target.y,
        near_target.x,
        near_target.y,
        lookahead,
        near_lookahead,
        config_.actual_lookahead_distance_blend,
        config_.dual_preview_blend,
        config_.wheel_base_m,
        config_.steering_gain / config_.steering_command_to_tire_angle_ratio,
        target_speed,
        command_curvature_feedforward,
        commanded_steering,
        config_.dt_sec,
        config_.maximum_steering_angle_rad,
        config_.maximum_steering_rate_radps,
        config_.steering_passthrough_enabled};
    const auto pp = simple_pure_pursuit::computePurePursuitCore(pp_input);
    if (!pp.valid) {
      sample.rejection=RejectReason::PP_TRACKABILITY;
      visitor(sample);
      return false;
    }

    const double previous_command = commanded_steering;
    commanded_steering = pp.bounded_steering_rad;
    bool rotation_acceleration_hold = false;
    if (request.rotation_prediction.valid) {
      const auto &snapshot = request.rotation_prediction;
      const double curvature = simple_pure_pursuit::rotationSignedCurvature(
          reference->count, reference_index, [&](std::size_t i) {
            const auto &p = reference->points[i];
            return std::pair<double,double>{p.x_m,p.y_m};
          });
      const double nominal_tire = simple_pure_pursuit::commandToTireSteeringAngle(
          commanded_steering, config_.steering_command_to_tire_angle_ratio,
          config_.maximum_tire_steering_angle_rad);
      // The kinematic plant has no future lateral velocity measurement.
      // Use measured slip only at the initial state; later triggers use yaw.
      const auto rotation = simple_pure_pursuit::stepRotationController(
          snapshot.parameters, simple_pure_pursuit::RotationControllerInput{
              request.stamp_sec + step*config_.dt_sec, config_.dt_sec,
              curvature, speed*curvature, yaw_rate,
              speed*std::tan(nominal_tire)/config_.wheel_base_m,
              step == 0 ? snapshot.slip_angle_rad : 0., nominal_tire, steering,
              geometricTrackingDistance(*reference,x,y),
              step == 0 && snapshot.slip_angle_valid, true}, rotation_state);
      rotation_acceleration_hold = rotation_state.active && rotation.recovery.valid;
      if (rotation_acceleration_hold) {
        const double target = simple_pure_pursuit::tireToCommandSteeringAngle(
            rotation.recovery.requested_steering_rad,
            config_.steering_command_to_tire_angle_ratio,
            config_.maximum_steering_angle_rad);
        const auto recovery = simple_pure_pursuit::applySteeringExecutionContract(
            target, previous_command, config_.dt_sec,
            config_.maximum_steering_angle_rad, config_.maximum_steering_rate_radps,
            config_.steering_passthrough_enabled);
        if (recovery.valid) commanded_steering = recovery.bounded_angle_rad;
      }
    }
    // AWSIM holds the previously received actuator target during its pure
    // command delay. Apply the first-order steering response only after that
    // dead time so MPPI certifies the path the kart will actually trace.
    const auto delayed_steering = steering_delay_line.pushAndSelect(
        commanded_steering, config_.steering_control_delay_sec, config_.dt_sec);
    if (!delayed_steering.valid) {
      sample.rejection=RejectReason::NON_FINITE;
      visitor(sample);
      return false;
    }
    const double steering_alpha =
        1.0 - std::exp(-config_.dt_sec / config_.steering_time_constant_sec);
    const double delayed_target_tire_steering =
        simple_pure_pursuit::commandToTireSteeringAngle(
            delayed_steering.delayed_target_steering_rad,
            config_.steering_command_to_tire_angle_ratio,
            config_.maximum_tire_steering_angle_rad);
    const double lagged_steering =
        config_.awsim_vehicle_response_enabled
            ? awsimSteeringResponse(steering, delayed_target_tire_steering,
                                    config_.dt_sec, config_.steering_time_constant_sec)
            : steering + steering_alpha * (delayed_target_tire_steering - steering);
    const double maximum_steering_step =
        simple_pure_pursuit::physicalTireSteeringRate(
            config_.physical_tire_steering_rate_radps,
            config_.maximum_steering_rate_radps,
            config_.steering_command_to_tire_angle_ratio) * config_.dt_sec;
    steering =
        std::clamp(std::clamp(lagged_steering, steering - maximum_steering_step,
                              steering + maximum_steering_step),
                   -config_.maximum_tire_steering_angle_rad,
                   config_.maximum_tire_steering_angle_rad);
    double requested_acceleration = std::clamp(
        config_.speed_proportional_gain * (target_speed - speed),
        -config_.maximum_deceleration_mps2, config_.maximum_acceleration_mps2);
    if (rotation_acceleration_hold)
      requested_acceleration = simple_pure_pursuit::suppressPositiveAccelerationForRotation(
          requested_acceleration);
    const double target_tire_steering =
        simple_pure_pursuit::commandToTireSteeringAngle(
            commanded_steering, config_.steering_command_to_tire_angle_ratio,
            config_.maximum_tire_steering_angle_rad);
    const auto acceleration_hold =
        simple_pure_pursuit::holdPositiveAccelerationForSteeringDemand(
            config_.steering_demand_acceleration_hold_enabled, speed,
            config_.steering_acceleration_hold_minimum_speed_mps,
            pp.angle_limited, true, target_tire_steering, steering,
            config_.steering_acceleration_hold_minimum_tire_angle_rad,
            config_.steering_acceleration_hold_tracking_error_rad,
            requested_acceleration,
            config_.steering_acceleration_hold_maximum_acceleration_mps2);
    if (acceleration_hold.valid) {
      requested_acceleration = acceleration_hold.acceleration_mps2;
      if (acceleration_hold.active) {
        acceleration = std::min(requested_acceleration, acceleration);
      }
    }
    const double acceleration_alpha =
        1.0 -
        std::exp(-config_.dt_sec / config_.acceleration_time_constant_sec);
    acceleration +=
        acceleration_alpha * (requested_acceleration - acceleration);
    acceleration = std::clamp(acceleration, -config_.maximum_deceleration_mps2,
                              config_.maximum_acceleration_mps2);
    if (config_.awsim_vehicle_response_enabled) {
      acceleration = longitudinal_response.acceleration(
          requested_acceleration, speed, static_cast<double>(step) * config_.dt_sec);
    }

    x += speed * std::cos(yaw) * config_.dt_sec;
    y += speed * std::sin(yaw) * config_.dt_sec;
    const double target_yaw_rate = speed / config_.wheel_base_m * std::tan(steering);
    if (config_.awsim_vehicle_response_enabled) {
      // Tire angle changes before body yaw rate settles. Integrate the
      // first-order response from measured yaw rate, rather than jumping to
      // the bicycle model's steady-state yaw rate at every steering update.
      const double tau = config_.yaw_response_time_constant_sec;
      const double alpha = 1. - std::exp(-config_.dt_sec / tau);
      yaw = normalizeAngle(yaw + target_yaw_rate * config_.dt_sec +
                           (yaw_rate - target_yaw_rate) * tau * alpha);
      yaw_rate += alpha * (target_yaw_rate - yaw_rate);
    } else {
      yaw = normalizeAngle(yaw + target_yaw_rate * config_.dt_sec);
      yaw_rate = target_yaw_rate;
    }
    speed = std::max(0.0, speed + acceleration * config_.dt_sec);
    if (!finite(x) || !finite(y) || !finite(yaw) || !finite(speed) ||
        !finite(steering) || !finite(acceleration)) {
      sample.rejection=RejectReason::NON_FINITE;
      visitor(sample);
      return false;
    }

    sample.after={x,y,yaw,speed,(step+1)*config_.dt_sec};
    sample.control_reference_index=reference_index;
    reference_index=advanceNearestReference(*reference,
        config_.compare_cma_delay_compensation ? physical_reference_index : reference_index,x,y);
    sample.physical_reference_index=reference_index;
    if(reference!=&path) {
      sample.control_reference_index=advanceNearestReference(path,0U,sample.before.x_m,sample.before.y_m);
      sample.physical_reference_index=advanceNearestReference(path,0U,x,y);
    }
    sample.tire_steering_rad=steering;sample.commanded_steering_rad=commanded_steering;
    sample.bounded_steering_rad=pp.bounded_steering_rad;sample.acceleration_mps2=acceleration;
    sample.far_x_m=far_target.x;sample.far_y_m=far_target.y;
    sample.near_x_m=near_target.x;sample.near_y_m=near_target.y;
    if(!visitor(sample)) return false;
  }
  return true;
}

Evaluation ReferenceSpaceMppiPlanner::evaluateReference(
    const Parameters &parameters, const PlanRequest &request,
    const ReferenceControlSequence *control_sequence,
    const TemporaryReference *reference, const TemporaryReference *prepared_field,
    const PreparedExecutionGeometry *geometry, const ExecutionValidators *validators) const {
  Evaluation result;
  const auto &path_validator=validators ? validators->path : request.path_constraint_validator;
  const auto &rollout_validator=validators ? validators->rollout : request.rollout_constraint_validator;
  if(reference==nullptr || reference->count<3U || reference->count>kMaximumReferencePoints)
    return result;
  for(std::size_t i=0;i<reference->count;++i) {
    const auto &p=reference->points[i];
    for(double v:{p.s_m,p.d_m,p.x_m,p.y_m,p.yaw_rad,p.curvature_1pm,p.speed_mps})
      if(!finite(v)) {result.reject_reason=RejectReason::NON_FINITE;return result;}
    if(p.speed_mps<0.0 || (i>0 && p.s_m<=reference->points[i-1].s_m)) return result;
  }

  // This separate field is used only for Reference-relative costs. Never
  // pass its possibly repeated stations to the ordered path validator.
  std::optional<BaseProjectionIndex> local_projection;
  const BaseProjectionIndex *base_projection=geometry ? geometry->base_projection.get() : nullptr;
  if (!base_projection) {
    local_projection.emplace(request.base_reference.data(),request.base_reference_count);
    base_projection=&*local_projection;
  }
  const auto field_reference = prepared_field ? *prepared_field :
      projectReferenceField(*reference,request,base_projection);
  for (std::size_t point_index = 0U; point_index < reference->count;
       ++point_index) {
    const auto &projected = field_reference.points[point_index];
    for (std::size_t obstacle_index = 0U;
         obstacle_index < request.static_obstacle_count; ++obstacle_index) {
      const auto &obstacle = request.static_obstacles[obstacle_index];
      if (projected.s_m >= obstacle.minimum_s_m -
                           config_.obstacle_longitudinal_inflation_m &&
          projected.s_m <= obstacle.maximum_s_m +
                           config_.obstacle_longitudinal_inflation_m &&
          projected.d_m >=
              obstacle.minimum_d_m - config_.obstacle_lateral_inflation_m &&
          projected.d_m <=
              obstacle.maximum_d_m + config_.obstacle_lateral_inflation_m) {
        recordRejection(&result, RejectReason::COLLISION, 0U, point_index,
                        projected.s_m, projected.d_m);
        return result;
      }
    }
  }

  double x = request.ego.x_m;
  double y = request.ego.y_m;
  double yaw = request.ego.yaw_rad;
  double speed = std::max(0.0, request.ego.speed_mps);
  std::array<bool, kMaximumDynamicObstacles> approach_active{};
  const auto initial_projection = base_projection->project(x,y);
  for (std::size_t i = 0; i < request.dynamic_obstacle_count; ++i) {
    // Activate from the measured gap, so a future close approach does not
    // introduce the speed preference before the requested following distance.
    approach_active[i] = !config_.approach_at_follow_gap_only ||
        approachSpeedTarget(request.dynamic_obstacles[i], request.ego.s_m,
            request.ego.d_m, normalizeAngle(yaw - initial_projection.yaw),
            0.0, config_).has_value();
  }
  std::size_t reference_index=0U;
  std::size_t base_index = 0U;
  const double expected_progress = std::max(
      1.0, speed * config_.dt_sec * static_cast<double>(horizonSteps(request)));
  double accumulated_cost = 0.0;
  if (request.sample_staged_speed) {
    result.cost_terms[3]=geometry ? geometry->curvature_cost : spatialCurvatureCost(*reference,request);
    accumulated_cost+=result.cost_terms[3];
  }
  double previous_bounded_steering=simple_pure_pursuit::tireToCommandSteeringAngle(
      std::clamp(request.ego.steering_rad,-config_.maximum_tire_steering_angle_rad,
          config_.maximum_tire_steering_angle_rad),config_.steering_command_to_tire_angle_ratio,
      config_.maximum_steering_angle_rad);
  double required_reference_s = reference->points.front().s_m;
  const auto include_reference_position = [&](double px, double py, double extension) {
    const auto projection = projectExecutionTrajectory(*reference, px, py);
    const auto &a = reference->points[projection.lower];
    const auto &b = reference->points[projection.lower + 1U];
    required_reference_s = std::max(required_reference_s,
        a.s_m + projection.ratio * (b.s_m - a.s_m) + extension);
  };
  const auto braking_reserve = [&](double velocity) {
    // Cover saturated braking, the proportional low-speed tail and actuator
    // lag/hold. This is a reference coverage bound; physical wall collision
    // remains checked by the execution sweep, with its measured initial state.
    const double deceleration = config_.awsim_vehicle_response_enabled
        ? std::min(3.0, config_.maximum_deceleration_mps2)
        : config_.maximum_deceleration_mps2;
    const double lag = config_.awsim_vehicle_response_enabled
        ? .1 : config_.acceleration_time_constant_sec;
    return velocity * velocity / (2.0 * deceleration) +
        velocity / config_.speed_proportional_gain + velocity * lag;
  };
  const bool motion_complete=visitExecutionMotion(*reference,request,config_,horizonSteps(request),
      [&](const ExecutionMotionSample &sample) {
    const auto step=result.predicted_rollout_count;
    reference_index=sample.control_reference_index;
    const auto &nearest_reference=reference->points[reference_index];
    if(sample.rejection!=RejectReason::NONE) {
      recordRejection(&result,sample.rejection,step,reference_index,
          nearest_reference.s_m,nearest_reference.d_m);
      return false;
    }
    const auto &rollout_start=sample.before;
    x=sample.after.x_m;y=sample.after.y_m;yaw=sample.after.yaw_rad;speed=sample.after.speed_mps;
    const double steering=sample.tire_steering_rad;
    if(request.path_constraint_execution_prefix) {
      include_reference_position(rollout_start.x_m,rollout_start.y_m,braking_reserve(rollout_start.speed_mps));
      include_reference_position(sample.far_x_m,sample.far_y_m,0.);
      include_reference_position(sample.near_x_m,sample.near_y_m,0.);
    }
    const double prediction_time_sec =
        static_cast<double>(step + 1U) * config_.dt_sec;
    result.predicted_rollout[step] =
        RolloutState{x, y, yaw, speed, prediction_time_sec};
    result.predicted_rollout_count = step + 1U;
    if (!validateExecutionSegment(rollout_start, result.predicted_rollout[step],
                                  request, step, &result, base_projection)) return false;
    if (rollout_validator) {
      const RejectReason reason = rollout_validator(
          rollout_start, result.predicted_rollout[step]);
      const bool hard_environment_rejection =
          reason == RejectReason::COLLISION || reason == RejectReason::WALL ||
          reason == RejectReason::INVALID_INPUT ||
          reason == RejectReason::NON_FINITE;
      if (reason != RejectReason::NONE &&
          (!config_.collision_only_rejection || hard_environment_rejection)) {
        recordRejection(&result, reason, step, reference_index,
                        nearest_reference.s_m, nearest_reference.d_m);
        result.reject_stage="environment_execution_segment";
        result.reject_time_sec=prediction_time_sec;
        result.reject_x_m=x; result.reject_y_m=y;
        return false;
      }
    }

    const auto precise_projection=base_projection->project(x,y);
    base_index = baseIndexAtProjection(request, precise_projection);
    const auto &base = request.base_reference[base_index];
    const ProjectedState projected{precise_projection.s,precise_projection.d};
    if (!config_.collision_only_rejection &&
        (projected.d_m < base.minimum_d_m ||
         projected.d_m > base.maximum_d_m)) {
      recordRejection(&result, RejectReason::TRACK, step, base_index,
                      projected.s_m, projected.d_m);
      return false;
    }
    reference_index = sample.physical_reference_index;
    const double cross_track_error = request.sample_staged_speed ?
        geometricTrackingDistance(*reference, x, y) :
        crossTrackDistance(*reference, reference_index, x, y);
    result.maximum_cross_track_error_m =
        std::max(result.maximum_cross_track_error_m, cross_track_error);
    if (!config_.collision_only_rejection &&
        cross_track_error > config_.maximum_cross_track_error_m) {
      recordRejection(&result, RejectReason::PP_TRACKABILITY, step,
                      reference_index, projected.s_m, projected.d_m);
      return false;
    }
    const double lateral_acceleration =
        speed * speed * std::abs(std::tan(steering)) / config_.wheel_base_m;
    result.maximum_lateral_acceleration_mps2 = std::max(
        result.maximum_lateral_acceleration_mps2, lateral_acceleration);
    if (!config_.collision_only_rejection &&
        lateral_acceleration >
            config_.maximum_lateral_acceleration_mps2 + 1.0e-9) {
      recordRejection(&result, RejectReason::VEHICLE_LIMIT, step, base_index,
                      projected.s_m, projected.d_m);
      return false;
    }

    double clearance = std::min(projected.d_m - base.minimum_d_m,
                                base.maximum_d_m - projected.d_m);
    const double boundary_gap=clearance;
    double static_gap=std::numeric_limits<double>::infinity();
    double dynamic_gap=std::numeric_limits<double>::infinity();
    std::size_t clearance_source=0;
    int dynamic_index=-1;
    double approach_speed_excess = 0.0;
    for (std::size_t obstacle_index = 0U;
         obstacle_index < request.static_obstacle_count; ++obstacle_index) {
      const auto &obstacle = request.static_obstacles[obstacle_index];
      const double s_separation = intervalSeparation(
          projected.s_m, projected.s_m,
          obstacle.minimum_s_m - config_.obstacle_longitudinal_inflation_m,
          obstacle.maximum_s_m + config_.obstacle_longitudinal_inflation_m);
      const double d_separation = intervalSeparation(
          projected.d_m, projected.d_m,
          obstacle.minimum_d_m - config_.obstacle_lateral_inflation_m,
          obstacle.maximum_d_m + config_.obstacle_lateral_inflation_m);
      if (s_separation <= 0.0 && d_separation <= 0.0) {
        recordRejection(&result, RejectReason::COLLISION, step, base_index,
                        projected.s_m, projected.d_m);
        return false;
      }
      const double gap=std::hypot(s_separation,d_separation);
      static_gap=std::min(static_gap,gap);
      if(gap<clearance)clearance_source=1;
      clearance = std::min(clearance, gap);
    }
    for (std::size_t obstacle_index = 0U;
         obstacle_index < request.dynamic_obstacle_count; ++obstacle_index) {
      const auto &obstacle = request.dynamic_obstacles[obstacle_index];
      if (config_.cost_approach_speed_weight > 0.0 && approach_active[obstacle_index]) {
        const auto target = approachSpeedTarget(obstacle, projected.s_m,
            projected.d_m, normalizeAngle(yaw - precise_projection.yaw),
            prediction_time_sec, config_);
        if (target) approach_speed_excess =
            std::max(approach_speed_excess, speed - *target);
      }
      const auto envelope =
          predictEnvelope(obstacle, prediction_time_sec,
                          normalizeAngle(yaw - precise_projection.yaw), config_);
      const double s_separation =
          intervalSeparation(projected.s_m, projected.s_m, envelope.minimum_s_m,
                             envelope.maximum_s_m);
      const double d_separation =
          intervalSeparation(projected.d_m, projected.d_m, envelope.minimum_d_m,
                             envelope.maximum_d_m);
      // Already checked throughout the execution segment. Keep this envelope
      // for soft cost/TTC, never as a second hard gate with full uncertainty.
      const double gap=std::hypot(s_separation,d_separation);
      if(gap<dynamic_gap){dynamic_gap=gap;dynamic_index=static_cast<int>(obstacle_index);}
      if(gap<clearance)clearance_source=2;
      clearance = std::min(clearance, gap);
      if (d_separation <= 0.0 && envelope.minimum_s_m > projected.s_m) {
        const double closing_speed = speed - obstacle.longitudinal_speed_mps;
        if (closing_speed > 1.0e-3) {
          result.minimum_ttc_sec =
              std::min(result.minimum_ttc_sec,
                       (envelope.minimum_s_m - projected.s_m) / closing_speed);
        }
      }
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, clearance);
    const double clearance_error =
        std::max(0.0, (config_.clearance_target_m - clearance) /
                          std::max(config_.clearance_target_m, 1.0e-3));
    // Fixed-library requests supply a common scale. A negative sentinel keeps
    // stochastic requests on their documented nominal-based scale.
    const double reference_cost_scale = request.sample_staged_speed
        ? (request.cost_preferred_pass_separation_m >= 0.0
               ? request.cost_preferred_pass_separation_m
               : std::abs(request.nominal.d_pass_m - request.corridor_nominal_d_m))
        : std::abs(parameters.d_pass_m - request.corridor_nominal_d_m);
    const double normalized_reference =
        (projected.d_m - base.corridor_d_m) / std::max(0.5, reference_cost_scale);
    const double normalized_curvature =
        request.sample_staged_speed ? 0.0 : std::abs(nearest_reference.curvature_1pm) * 5.0;
    const double normalized_pp = request.sample_staged_speed ?
        normalizedTrackingExcess(cross_track_error, config_.maximum_cross_track_error_m) :
        cross_track_error / config_.maximum_cross_track_error_m;
    const double normalized_steering_rate =
        std::abs(sample.bounded_steering_rad - previous_bounded_steering) /
        (config_.maximum_steering_rate_radps * config_.dt_sec);
    const double diagnostic_step_scale =
        (request.sample_staged_speed ? 1.0 : static_cast<double>(config_.horizon_steps)) /
        horizonSteps(request);
    result.cost_terms[0] -= config_.cost_progress_weight *
        std::max(0.0, projected.s_m - request.ego.s_m) / expected_progress /
        horizonSteps(request);
    result.cost_terms[1] += diagnostic_step_scale * config_.cost_clearance_weight * clearance_error * clearance_error;
    const double approach_cost = (config_.longitudinal_planning_enabled ? 0. :
        config_.cost_approach_speed_weight) *
        approach_speed_excess * approach_speed_excess / horizonSteps(request);
    result.cost_terms[11] += approach_cost;
    accumulated_cost += approach_cost;
    const double clearance_step_cost=diagnostic_step_scale * config_.cost_clearance_weight * clearance_error * clearance_error;
    result.clearance_cost_sources[clearance_source]+=clearance_step_cost;
    if(clearance_step_cost>result.clearance_peak[7])
      result.clearance_peak={static_cast<double>(step),x,y,boundary_gap,static_gap,
                            dynamic_gap,static_cast<double>(dynamic_index),clearance_step_cost};
    result.cost_terms[2] += diagnostic_step_scale * config_.cost_reference_weight * normalized_reference * normalized_reference;
    result.cost_terms[3] += diagnostic_step_scale * config_.cost_curvature_weight * normalized_curvature * normalized_curvature;
    result.cost_terms[4] += diagnostic_step_scale * config_.cost_pp_error_weight * normalized_pp * normalized_pp;
    result.cost_terms[5] += diagnostic_step_scale * config_.cost_steering_rate_weight * normalized_steering_rate * normalized_steering_rate;
    accumulated_cost +=
        -config_.cost_progress_weight *
            std::max(0.0, projected.s_m - request.ego.s_m) / expected_progress /
            static_cast<double>(horizonSteps(request)) +
        ((request.sample_staged_speed ? 1.0 : static_cast<double>(config_.horizon_steps)) /
         horizonSteps(request)) * (
        config_.cost_clearance_weight * clearance_error * clearance_error +
        config_.cost_reference_weight * normalized_reference *
            normalized_reference +
        config_.cost_curvature_weight * normalized_curvature *
            normalized_curvature +
        config_.cost_pp_error_weight * normalized_pp * normalized_pp +
        config_.cost_steering_rate_weight * normalized_steering_rate *
            normalized_steering_rate);
    previous_bounded_steering = sample.commanded_steering_rad;
    result.progress_m =
        std::max(result.progress_m, projected.s_m - request.ego.s_m);
    return true;
  },geometry ? &geometry->preview_curvatures : nullptr);
  if(!motion_complete) return result;

  // d_pass remains free to cover the entire physical corridor. This soft
  // objective keeps the selected path near the requested passing separation
  // unless wall, vehicle-limit or opponent costs justify a wider line.
  const double preferred_pass_separation_m =
      request.cost_preferred_pass_separation_m >= 0.0
          ? request.cost_preferred_pass_separation_m
          : std::abs(request.nominal.d_pass_m);
  double physical_pass_separation_m = std::abs(parameters.d_pass_m);
  if (request.sample_staged_speed) {
    // A supplied trajectory has no generating d_pass parameter. Measure both
    // it and new proposals in the same spatial opponent-relative field.
    physical_pass_separation_m = 0.;
    for (std::size_t j = 0; j < reference->count; ++j) {
      const auto &p = field_reference.points[j];
      const auto upper=std::lower_bound(request.base_reference.begin()+1,
          request.base_reference.begin()+request.base_reference_count,p.s_m,
          [](const BaseReferencePoint &b,double s) {return b.s_m<s;});
      const auto i=std::min(request.base_reference_count-1,
          static_cast<std::size_t>(upper-request.base_reference.begin()));
      const auto &a = request.base_reference[i-1]; const auto &b = request.base_reference[i];
      const double u = std::clamp((p.s_m-a.s_m)/std::max(1e-9,b.s_m-a.s_m),0.,1.);
      const double origin = a.pass_origin_d_valid && b.pass_origin_d_valid ?
          a.pass_origin_d_m+u*(b.pass_origin_d_m-a.pass_origin_d_m) : request.pass_profile_origin_d_m;
      physical_pass_separation_m = std::max(physical_pass_separation_m, std::abs(p.d_m-origin));
    }
  }
  const double normalized_pass_separation_error =
      (physical_pass_separation_m - preferred_pass_separation_m) /
      std::max(0.5, preferred_pass_separation_m);
  accumulated_cost += config_.cost_pass_separation_weight *
                      normalized_pass_separation_error *
                      normalized_pass_separation_error;
  result.cost_terms[6] = config_.cost_pass_separation_weight *
      normalized_pass_separation_error * normalized_pass_separation_error;
  if (request.sample_staged_speed && config_.cost_wall_line_weight > 0.0) {
    const double wall_cost =
        config_.cost_wall_line_weight * (geometry ? geometry->wall_line_cost : wallLineCost(field_reference, request));
    result.cost_terms[6] += wall_cost;
    accumulated_cost += wall_cost;
  }

  if (request.sample_staged_speed) {
    // Preserve Frenet diagnostics, but score the same Cartesian tracking error
    // and tolerance as the stage objective, without saturating large errors.
    const auto terminal = base_projection->project(x,y);
    const auto on_path = projectExecutionTrajectory(*reference, x, y);
    const auto &a = field_reference.points[on_path.lower];
    const auto &b = field_reference.points[on_path.lower+1];
    const double goal_d = a.d_m + on_path.ratio * (b.d_m - a.d_m);
    result.terminal_s_m = terminal.s;
    result.terminal_d_m = terminal.d;
    result.terminal_reference_d_m = goal_d;
    const double residual = normalizedTrackingExcess(
        geometricTrackingDistance(*reference, x, y), config_.maximum_cross_track_error_m);
    accumulated_cost += config_.cost_terminal_lateral_weight * residual * residual;
    result.cost_terms[7] = config_.cost_terminal_lateral_weight * residual * residual;
  }

  if (request.sample_staged_speed) {
    const auto terminal_projection=projectExecutionTrajectory(field_reference,x,y);
    const auto arc_field=cartesianArcField(field_reference);
    const auto &a=arc_field.points[terminal_projection.lower];
    const auto &b=arc_field.points[terminal_projection.lower+1];
    result.field_terminal_arc_m=a.s_m+terminal_projection.ratio*(b.s_m-a.s_m);
    const auto costs = executionFieldCosts(field_reference, request, config_, &result);
    result.cost_terms[8] = costs[0]; result.cost_terms[9] = costs[1];
    accumulated_cost += costs[0]+costs[1];
  } else if (control_sequence != nullptr && control_sequence->count > 1U) {
    // Legacy refinement retains proposal-space regularization. Brain staged
    // execution uses physical fields above, including supplied continuations.
    const double speed_normalizer = std::max(config_.control_speed_scale_std, 1.0e-3);
    const double spatial_ratio = spatialKnotRatio(
        std::max(1.0, request.base_reference[request.base_reference_count - 1U].s_m -
                          request.anchor_s_m), control_sequence->count,
        config_.control_lateral_reference_spacing_m);
    double normalized_variation = 0.0;
    for (std::size_t knot = 1U; knot < control_sequence->count; ++knot) {
      const double lateral_delta =
          (control_sequence->lateral_adjustment_m[knot] -
           control_sequence->lateral_adjustment_m[knot - 1U]) /
          (std::max(config_.control_lateral_std_m, 1.0e-3) * spatial_ratio);
      const double speed_delta =
          (control_sequence->speed_scale_adjustment[knot] -
           control_sequence->speed_scale_adjustment[knot - 1U]) /
          speed_normalizer;
      normalized_variation +=
          lateral_delta * lateral_delta + speed_delta * speed_delta;
    }
    accumulated_cost += config_.cost_control_smoothness_weight *
                        normalized_variation /
                        static_cast<double>(control_sequence->count - 1U);
    result.cost_terms[8] = config_.cost_control_smoothness_weight * normalized_variation /
                          static_cast<double>(control_sequence->count - 1U);
    if (warm_control_sequence_valid_ && warm_side_ == request.side &&
        warm_semantic_key_ == request.semantic_key && warm_phase_ == request.phase &&
        warm_control_sequence_.count == control_sequence->count) {
      const ReferenceControlSequence shifted_warm =
          controlSequenceMean(request);
      double normalized_change = 0.0;
      for (std::size_t knot = 0U; knot < control_sequence->count; ++knot) {
        const double lateral_delta =
            (control_sequence->lateral_adjustment_m[knot] -
             shifted_warm.lateral_adjustment_m[knot]) /
            std::max(config_.control_lateral_std_m, 1.0e-3);
        const double speed_delta =
            (control_sequence->speed_scale_adjustment[knot] -
             shifted_warm.speed_scale_adjustment[knot]) /
            speed_normalizer;
        normalized_change +=
            lateral_delta * lateral_delta + speed_delta * speed_delta;
      }
      accumulated_cost += config_.cost_control_change_weight *
                          normalized_change /
                          static_cast<double>(control_sequence->count);
      result.cost_terms[9] = config_.cost_control_change_weight * normalized_change /
                            static_cast<double>(control_sequence->count);
    }
  }

  if (!request.sample_staged_speed && warm_mean_valid_ && warm_side_ == request.side &&
      warm_phase_ == request.phase &&
      warm_semantic_key_ == request.semantic_key) {
    const auto candidate = asArray(parameters);
    const auto previous = asArray(warm_mean_);
    const auto bounds_minimum = asArray(request.bounds.minimum);
    const auto bounds_maximum = asArray(request.bounds.maximum);
    double normalized_change = 0.0;
    for (std::size_t index = 0U; index < kParameterCount; ++index) {
      if (!parameterActiveForWarmCost(request, index) ||
          bounds_maximum[index] - bounds_minimum[index] <= 1.0e-9 ||
          config_.sigma[index] <= 1.0e-9) {
        continue;
      }
      const double scale = config_.sigma[index];
      const double delta = (candidate[index] - previous[index]) / scale;
      normalized_change += delta * delta;
    }
    accumulated_cost +=
        config_.cost_reference_change_weight * normalized_change;
    result.cost_terms[10] = config_.cost_reference_change_weight * normalized_change;
  }
  if (!finite(accumulated_cost)) {
    recordRejection(&result, RejectReason::NON_FINITE,
                    horizonSteps(request) - 1U, base_index, result.progress_m,
                    request.ego.d_m);
    return result;
  }
  // These swept and Cartesian checks are part of sample feasibility. Running
  // them only after MPPI has selected/averaged a candidate can discard the
  // optimum without allowing the next feasible sample to win. Keep them last
  // so obviously infeasible rollouts do not pay the occupancy-grid cost.
  if (!finalSweptValidate(*reference, request, &result, base_projection)) {
    result.cost = std::numeric_limits<double>::infinity();
    return result;
  }
  if (path_validator) {
    auto validation_reference = *reference;
    if (request.path_constraint_execution_prefix) {
      include_reference_position(x, y, braking_reserve(speed));
      // Include the complete boundary segment; do not skip a wall between
      // points or change the path sent to Pure Pursuit by truncating it.
      validation_reference.count = 2U;
      while (validation_reference.count < reference->count &&
             reference->points[validation_reference.count - 1U].s_m < required_reference_s)
        ++validation_reference.count;
      if(request.front_merge_attack)
        validation_reference.count=std::min(reference->count,
            std::max(validation_reference.count,reference->front_merge_end_index+1U));
    }
    result.validated_reference_distance_m =
        validation_reference.points[validation_reference.count - 1U].s_m -
        validation_reference.points.front().s_m;
    const RejectReason reason = path_validator(validation_reference);
    const bool hard_environment_rejection =
        reason == RejectReason::COLLISION || reason == RejectReason::WALL ||
        reason == RejectReason::INVALID_INPUT ||
        reason == RejectReason::NON_FINITE;
    if (reason != RejectReason::NONE &&
        (!config_.collision_only_rejection || hard_environment_rejection)) {
      recordRejection(&result, reason, horizonSteps(request) - 1U, 0U,
                      reference->points.front().s_m,
                      reference->points.front().d_m);
      result.reject_stage="environment_reference_path";
      result.cost = std::numeric_limits<double>::infinity();
      return result;
    }
  }
  result.valid = true;
  FrontMergeMetrics merge_metrics;
  const auto merge_reason=validateFrontMerge(request,config_,*reference,
      request.compare_passing_points ? &merge_metrics : nullptr);
  if(merge_reason!=RejectReason::NONE) {
    result.valid=false;result.reject_reason=merge_reason;
    result.reject_stage="front_merge_no_yield";return result;
  }
  result.reject_reason = RejectReason::NONE;
  result.merge_completion_time_sec=merge_metrics.completion_time_sec;
  result.merge_braking_mps=merge_metrics.braking_mps;
  result.merge_alongside_clearance_m=std::isfinite(merge_metrics.alongside_clearance_m) ?
      merge_metrics.alongside_clearance_m : 0.;
  const auto passing = passingProgress(request, config_, result);
  result.cost_terms[12] = -config_.cost_passing_progress_weight * passing.progress;
  result.cost_terms[13] = -config_.cost_passing_opportunity_weight * passing.opportunity;
  result.cost_terms[14] = otLaneEntryCost(request, result);
  accumulated_cost += result.cost_terms[12] + result.cost_terms[13] + result.cost_terms[14];
  result.cost = accumulated_cost;
  result.overtake_time_sec = predictedOvertakeTime(request, config_, result);
  return result;
}

bool ReferenceSpaceMppiPlanner::validateExecutionSegment(
    const RolloutState &previous,const RolloutState &state,
    const PlanRequest &request,std::size_t step,Evaluation *evaluation,
    const BaseProjectionIndex *projection) const {
  if(request.dynamic_obstacle_count==0U && request.static_obstacle_count==0U) return true;
  std::optional<BaseProjectionIndex> local_projection;
  if (!projection) {
    local_projection.emplace(request.base_reference.data(),request.base_reference_count);
    projection=&*local_projection;
  }
  const double dt=state.time_sec-previous.time_sec;
  const double yaw_delta=normalizeAngle(state.yaw_rad-previous.yaw_rad);
  const double radius=std::hypot(config_.vehicle_half_length_m,config_.vehicle_half_width_m);
  double swept_motion=std::hypot(state.x_m-previous.x_m,state.y_m-previous.y_m)+
      radius*std::abs(yaw_delta);
  double opponent_motion=0.0;
  for(std::size_t i=0;i<request.dynamic_obstacle_count;++i) {
    const auto &o=request.dynamic_obstacles[i];
    opponent_motion=std::max(opponent_motion,
        std::hypot(o.longitudinal_speed_mps,o.lateral_speed_mps)*dt+
        o.longitudinal_acceleration_bound_mps2*state.time_sec*dt);
    if(o.prediction && request.world_reference) {
      const auto a=opponent_prediction::positionAt(o,previous.time_sec);
      const auto b=opponent_prediction::positionAt(o,state.time_sec);
      const auto pa=request.world_reference->pose(a.global_s,a.d);
      const auto pb=request.world_reference->pose(b.global_s,b.d);
      if(pa && pb) opponent_motion=std::max(opponent_motion,
          std::hypot((*pb)[0]-(*pa)[0],(*pb)[1]-(*pa)[1])+
          radius*std::abs(normalizeAngle((*pb)[2]+b.relative_yaw-(*pa)[2]-a.relative_yaw))+
          o.longitudinal_acceleration_bound_mps2*state.time_sec*dt);
    }
  }
  // Include relative obstacle motion even when ego is stopped. The configured
  // sweep resolution is unchanged; Reference target speed never supplies t.
  const double intervals=std::max((swept_motion+opponent_motion)/
                                  config_.final_swept_check_step_m,dt/0.025);
  if(!finite(intervals) || intervals>100000.0 || dt<=0.0) {
    recordRejection(evaluation,RejectReason::INVALID_INPUT,step,0,0,0);return false;
  }
  const std::size_t subdivisions=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(intervals)));
  for(std::size_t j=0;j<=subdivisions;++j) {
    const double u=static_cast<double>(j)/subdivisions;
    const double x=previous.x_m+u*(state.x_m-previous.x_m);
    const double y=previous.y_m+u*(state.y_m-previous.y_m);
    const double yaw=previous.yaw_rad+u*yaw_delta;
    const double t=previous.time_sec+u*dt;
    const auto p=projection->project(x,y);
    if(!finite(p.error)) {recordRejection(evaluation,RejectReason::INVALID_INPUT,step,0,0,0);return false;}
    const auto reject=[&](std::size_t obstacle,const DynamicEnvelope &e) {
      evaluation->reject_dynamic_obstacle=false;
      evaluation->reject_reference_yaw_rad=std::numeric_limits<double>::quiet_NaN();
      recordRejection(evaluation,RejectReason::COLLISION,step,p.index,p.s,p.d);
      evaluation->reject_stage="execution_sweep"; evaluation->reject_time_sec=t;
      evaluation->reject_x_m=x; evaluation->reject_y_m=y;
      evaluation->reject_obstacle_index=obstacle;
      evaluation->reject_obstacle_envelope={e.minimum_s_m,e.maximum_s_m,e.minimum_d_m,e.maximum_d_m};
    };
    for(std::size_t i=0;i<request.dynamic_obstacle_count;++i) {
      const auto o=collisionObstacle(request.dynamic_obstacles[i],t);
      const auto e=predictEnvelope(o,t,normalizeAngle(yaw-p.yaw),config_);
      bool collision=p.s>=e.minimum_s_m && p.s<=e.maximum_s_m && p.d>=e.minimum_d_m && p.d<=e.maximum_d_m;
      if(request.world_reference) {
        const auto predicted=opponent_prediction::positionAt(o,t);
        const double s=predicted.global_s;
        const double d=predicted.d;
        const double hs=std::max(0.0,o.longitudinal_uncertainty_m)+
            .5*std::max(0.0,o.longitudinal_acceleration_bound_mps2)*t*t;
        const double hd=std::max(0.0,o.lateral_uncertainty_m);
        const auto hit=request.world_reference->occupancyOverlap({x,y,yaw},
            s-hs,s+hs,d-hd,d+hd,predicted.relative_yaw,
            config_.vehicle_half_length_m,config_.vehicle_half_width_m,
            config_.obstacle_longitudinal_inflation_m,config_.obstacle_lateral_inflation_m);
        if(!hit) {
          recordRejection(evaluation,RejectReason::INVALID_INPUT,step,p.index,p.s,p.d);
          return false;
        }
        collision=*hit; // The old Frenet box is NOT a sound world prefilter.
      }
      if(collision) {
        reject(i,e);
        evaluation->reject_yaw_rad=yaw;
        evaluation->reject_reference_yaw_rad=p.yaw;
        evaluation->reject_dynamic_obstacle=true;
        auto nominal=o;
        nominal.longitudinal_acceleration_bound_mps2=0;
        const auto position=predictEnvelope(nominal,t,normalizeAngle(yaw-p.yaw),config_);
        nominal.longitudinal_uncertainty_m=0;
        nominal.lateral_uncertainty_m=0;
        const auto body_box=predictEnvelope(nominal,t,normalizeAngle(yaw-p.yaw),config_);
        evaluation->reject_position_envelope={position.minimum_s_m,position.maximum_s_m,position.minimum_d_m,position.maximum_d_m};
        evaluation->reject_nominal_envelope={body_box.minimum_s_m,body_box.maximum_s_m,body_box.minimum_d_m,body_box.maximum_d_m};
        return false;
      }
    }
    for(std::size_t i=0;i<request.static_obstacle_count;++i) {
      const auto &o=request.static_obstacles[i];
      DynamicEnvelope e{o.minimum_s_m-config_.obstacle_longitudinal_inflation_m,
          o.maximum_s_m+config_.obstacle_longitudinal_inflation_m,
          o.minimum_d_m-config_.obstacle_lateral_inflation_m,
          o.maximum_d_m+config_.obstacle_lateral_inflation_m};
      if(p.s>=e.minimum_s_m && p.s<=e.maximum_s_m && p.d>=e.minimum_d_m && p.d<=e.maximum_d_m) {
        reject(i,e);return false;
      }
    }
  }
  return true;
}

bool ReferenceSpaceMppiPlanner::finalSweptValidate(
    const TemporaryReference &reference,const PlanRequest &request,
    Evaluation *evaluation,const BaseProjectionIndex *projection) const {
  if(evaluation==nullptr || reference.count<2 || reference.count>kMaximumReferencePoints) return false;
  if(config_.collision_only_rejection && request.static_obstacle_count==0U) return true;
  std::optional<BaseProjectionIndex> local_projection;
  if (!projection) {
    local_projection.emplace(request.base_reference.data(),request.base_reference_count);
    projection=&*local_projection;
  }
  // Only time-independent obligations belong to a geometric Reference. All
  // moving-obstacle checks already swept the actual closed-loop state trace.
  for(std::size_t i=1;i<reference.count;++i) {
    const auto &a=reference.points[i-1]; const auto &b=reference.points[i];
    const double length=distance(a.x_m,a.y_m,b.x_m,b.y_m);
    if(!finite(length)) {recordRejection(evaluation,RejectReason::NON_FINITE,0,i,b.s_m,b.d_m);return false;}
    const auto subdivisions=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(length/config_.final_swept_check_step_m)));
    for(std::size_t j=0;j<=subdivisions;++j) {
      const double u=static_cast<double>(j)/subdivisions;
      const auto projected=projection->project(
          a.x_m+u*(b.x_m-a.x_m),a.y_m+u*(b.y_m-a.y_m));
      const double station=projected.s,d=projected.d;
      const auto &ba=request.base_reference[projected.index];
      const auto &bb=request.base_reference[projected.index+1];
      const double ratio=std::clamp((station-ba.s_m)/(bb.s_m-ba.s_m),0.0,1.0);
      const double low=ba.minimum_d_m+ratio*(bb.minimum_d_m-ba.minimum_d_m);
      const double high=ba.maximum_d_m+ratio*(bb.maximum_d_m-ba.maximum_d_m);
      if(!config_.collision_only_rejection && (d<low-1e-9 || d>high+1e-9)) {
        recordRejection(evaluation,RejectReason::TRACK,0,i,station,d);return false;
      }
      for(std::size_t k=0;k<request.static_obstacle_count;++k) {
        const auto &o=request.static_obstacles[k];
        if(station>=o.minimum_s_m-config_.obstacle_longitudinal_inflation_m &&
           station<=o.maximum_s_m+config_.obstacle_longitudinal_inflation_m &&
           d>=o.minimum_d_m-config_.obstacle_lateral_inflation_m &&
           d<=o.maximum_d_m+config_.obstacle_lateral_inflation_m) {
          recordRejection(evaluation,RejectReason::COLLISION,0,i,station,d);
          evaluation->reject_stage="reference_static_geometry";return false;
        }
      }
    }
  }
  evaluation->reject_reason=RejectReason::NONE;
  return true;
}

ReferenceControlSequence ReferenceSpaceMppiPlanner::stagedSpeedProfile(
    const PlanRequest &request, const Parameters &parameters,
    ReferenceControlSequence control, std::size_t sample) const {
  if (control.count < 2U) return control;
  const double span = std::max(1.0,
      request.base_reference[request.base_reference_count - 1U].s_m -
          request.base_reference[0].s_m);
  const double matching_speed=std::max(request.preferred_matching_speed_mps,
                                     request.minimum_speed_mps);
  const double speed_basis=std::max(stagedSpeedBasis(request),1e-9);
  const double matching_scale=std::clamp(matching_speed /
      speed_basis,
      request.bounds.minimum.speed_scale,request.bounds.maximum.speed_scale);
  const double matching_distance = std::max(1.0,
      matching_speed * config_.dt_sec * horizonSteps(request));
  const double start = sample == 3U ?
      std::min(0.5 * parameters.l_out_m, 0.25 * matching_distance) :
      std::min(parameters.l_out_m, 0.5 * matching_distance);
  const double ramp = std::clamp(0.5 * matching_distance, 1.0, 5.0);
  for (std::size_t knot = 0U; knot < control.count; ++knot) {
    // A separate quadratic speed grid resolves entry acceleration without
    // changing the lateral grid or adding knots/candidate curvature noise.
    const double ratio = static_cast<double>(knot) / (control.count - 1U);
    const double s = span * ratio * ratio;
    const double blend = sample == 1U ? 1.0 : sample == 2U ? 0.0 :
        quinticBlend(std::clamp((s - start) / ramp, 0.0, 1.0));
    const double scale = sample == 10U ?
        std::clamp(request.ego.speed_mps / speed_basis,
            request.bounds.minimum.speed_scale, request.bounds.maximum.speed_scale) : sample >= 5U ?
        std::clamp(.2 * static_cast<double>(sample - 5U) *
            request.bounds.maximum.speed_scale, request.bounds.minimum.speed_scale,
            request.bounds.maximum.speed_scale) :
        matching_scale + blend * (request.bounds.maximum.speed_scale - matching_scale);
    control.speed_scale_adjustment[knot] = scale - parameters.speed_scale;
  }
  return projectControlSequence(control);
}

PlanResult ReferenceSpaceMppiPlanner::plan(const PlanRequest &request,
                                           Scratch *scratch) {
  const auto started = std::chrono::steady_clock::now();
  PlanResult result;
  result.generation = request.generation;
  const auto finish = [&started, &result]() {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - started)
                            .count();
    return result;
  };
  if (!config_.enabled) {
    result.reject_reason = RejectReason::DISABLED;
    return finish();
  }
  if (validateConfig() != nullptr || scratch == nullptr) {
    result.reject_reason = RejectReason::INVALID_CONFIG;
    return finish();
  }
  const std::size_t sample_count = request.sample_count_override == 0U
                                       ? config_.sample_count
                                       : request.sample_count_override;
  if (sample_count < 4U || sample_count > kMaximumSampleCount ||
      sample_count % 2U != 0U || config_.minimum_valid_count > sample_count) {
    result.reject_reason = RejectReason::INVALID_INPUT;
    return finish();
  }
  if (!finiteExecutionInput(request) ||
      (request.horizon_steps_override != 0U &&
       (request.horizon_steps_override < 2U ||
        request.horizon_steps_override > config_.horizon_steps)) ||
      !finite(request.stamp_sec) ||
      !finite(request.anchor_s_m) || !finite(request.anchor_d_m) ||
      !finite(request.corridor_nominal_d_m) ||
      !finite(request.cost_preferred_pass_separation_m) ||
      !finite(request.wall_line_start_s_m) || !finite(request.wall_line_end_s_m) ||
      !finite(request.pass_profile_origin_d_m) ||
      !finite(request.pass_profile_scale_m) ||
      !finite(request.minimum_speed_mps) || request.minimum_speed_mps < 0.0 ||
      !finite(request.preferred_matching_speed_mps) || request.preferred_matching_speed_mps < 0.0 ||
      !finiteParameters(request.nominal) ||
      !finiteParameters(request.bounds.minimum) ||
      !finiteParameters(request.bounds.maximum) ||
      request.base_reference_count < 3U ||
      request.base_reference_count > kMaximumReferencePoints ||
      request.static_obstacle_count > kMaximumStaticObstacles ||
      request.dynamic_obstacle_count > kMaximumDynamicObstacles) {
    result.reject_reason = RejectReason::INVALID_INPUT;
    return finish();
  }
  if (request.side != -1 && request.side != 1) {
    result.reject_reason = RejectReason::UNKNOWN_SIDE;
    return finish();
  }
  const auto minimum = asArray(request.bounds.minimum);
  const auto maximum = asArray(request.bounds.maximum);
  for (std::size_t index = 0U; index < kParameterCount; ++index) {
    if (minimum[index] > maximum[index]) {
      result.reject_reason = RejectReason::INVALID_INPUT;
      return finish();
    }
  }
  for (std::size_t index = 1U; index < request.base_reference_count; ++index) {
    if (!(request.base_reference[index].s_m >
          request.base_reference[index - 1U].s_m)) {
      result.reject_reason = RejectReason::INVALID_INPUT;
      return finish();
    }
  }

  if (request.nominal_only) {
    result.selected = request.nominal;
    result.updated_mean = request.nominal;
    ReferenceControlSequence nominal_control;
    const ReferenceControlSequence *nominal_control_ptr = nullptr;
    if (request.sample_control_sequence) {
      nominal_control.count = config_.control_knot_count;
      nominal_control_ptr = &nominal_control;
      result.selected_control_sequence = nominal_control;
      result.updated_control_sequence = nominal_control;
    }
    result.selected_evaluation =
        evaluate(request.nominal, request, nominal_control_ptr,
                 &scratch->candidate_reference);
    result.selected_reference = scratch->candidate_reference;
    result.best_raw_cost = result.selected_evaluation.cost;
    result.updated_mean_cost = result.selected_evaluation.cost;
    result.effective_sample_size = result.selected_evaluation.valid ? 1.0 : 0.0;
    if (!result.selected_evaluation.valid) {
      result.reject_reason = result.selected_evaluation.reject_reason;
      result.dominant_rejection = result.selected_evaluation;
      const auto reason =
          static_cast<std::size_t>(result.selected_evaluation.reject_reason);
      if (reason < result.reject_counts.size()) {
        result.reject_counts[reason] = 1U;
      }
      return finish();
    }
    result.valid = true;
    result.reject_reason = RejectReason::NONE;
    result.valid_sample_count = 1U;
    return finish();
  }

  const Parameters mean =
      warm_mean_valid_ && warm_side_ == request.side &&
              warm_phase_ == request.phase &&
              warm_semantic_key_ == request.semantic_key
          ? project(warm_mean_, request.bounds, request.side)
          : project(request.nominal, request.bounds, request.side);
  const auto mean_values = asArray(mean);
  const ReferenceControlSequence control_mean = controlSequenceMean(request);
  generateNoise(request, sample_count, scratch);
  generateControlNoise(request, sample_count, scratch);
  if (config_.longitudinal_planning_enabled) {
    for (std::size_t sample=0; sample<kMaximumSampleCount; ++sample) {
      scratch->noise[sample][4] = 0.;
      scratch->speed_control_noise[sample].fill(0.);
    }
  }
  if (request.sample_lateral_bounds && sample_count > 1U) {
    const double lateral_minimum = request.bounds.minimum.d_pass_m;
    const double lateral_span =
        request.bounds.maximum.d_pass_m - lateral_minimum;
    const auto minimum = asArray(request.bounds.minimum);
    const auto maximum = asArray(request.bounds.maximum);
    // sample 0 remains the projected warm/nominal trajectory. Deterministic
    // coverage starts at sample 1 so every cycle retains a true baseline.
    for (std::size_t sample = 1U; sample < sample_count; ++sample) {
      const double ratio = static_cast<double>(sample - 1U) /
                           static_cast<double>(std::max<std::size_t>(1U, sample_count - 2U));
      const double lateral_sample = lateral_minimum + ratio * lateral_span;
      scratch->noise[sample][0U] = lateral_sample - mean.d_pass_m;
      // A deterministic low-discrepancy layout covers transition timing and
      // both lateral shape controls on every brain cycle. This avoids waiting
      // for a Gaussian draw to discover a locally feasible path.
      const auto sample_dimension = [&](std::size_t parameter,
                                        std::size_t base) {
        const double unit = radicalInverse(sample, base);
        const double sampled = minimum[parameter] +
                               unit * (maximum[parameter] - minimum[parameter]);
        scratch->noise[sample][parameter] = sampled - mean_values[parameter];
      };
      const double sampled_transition =
          minimum[1U] + ratio * (maximum[1U] - minimum[1U]);
      scratch->noise[sample][1U] = sampled_transition - mean_values[1U];
      // Cover speed independently of transition length. Coupling long shifts
      // only to low speed excluded fast, gentle passes from a ten-sample mode.
      const double sampled_speed =
          sample == 1U || sample + 1U == sample_count
              ? maximum[4U]
              : sample == 2U ? minimum[4U]
                             : minimum[4U] + radicalInverse(sample, 7U) *
                                                  (maximum[4U] - minimum[4U]);
      if (!config_.longitudinal_planning_enabled)
        scratch->noise[sample][4U] = sampled_speed - mean_values[4U];
      if (sample == 1U) {
        // The first proposal is the immediate avoidance extreme: shortest
        // transition, highest speed and earliest admissible lateral growth.
        scratch->noise[sample][5U] = maximum[5U] - mean_values[5U];
        scratch->noise[sample][6U] = maximum[6U] - mean_values[6U];
      } else if (sample + 1U == sample_count) {
        // Retain the opposite smooth extreme in every bounded sample set.
        scratch->noise[sample][5U] = minimum[5U] - mean_values[5U];
        scratch->noise[sample][6U] = minimum[6U] - mean_values[6U];
      } else {
        sample_dimension(5U, 3U);
        sample_dimension(6U, 5U);
      }
    }
  }
  scratch->costs.fill(std::numeric_limits<double>::infinity());
  scratch->overtake_times.fill(std::numeric_limits<double>::infinity());
  scratch->valid.fill(false);
  double best_cost = std::numeric_limits<double>::infinity();
  Evaluation best_evaluation;
  double best_pass_time = std::numeric_limits<double>::infinity();
  Parameters best_parameters = mean;
  ReferenceControlSequence best_control_sequence = control_mean;
  double best_rejected_progress_m = -1.0;
  Parameters best_rejected_parameters = mean;
  ReferenceControlSequence best_rejected_control_sequence = control_mean;
  Evaluation best_rejected_evaluation;
  bool best_rejected_reference_available = false;
  std::size_t dominant_reject_count = 0U;
  const std::size_t visualized_sample_target =
      std::min({request.visualized_sample_count, kMaximumVisualizedSampleCount,
                sample_count});
  std::size_t next_visualized_sample = 0U;
  const auto requestedVisualizedIndex = [&]() {
    return request.sample_staged_speed && sample_count > 4U
        ? next_visualized_sample + 1U
        : visualized_sample_target <= 1U ? 0U
        : next_visualized_sample * (sample_count - 1U) /
              (visualized_sample_target - 1U);
  };

  const std::size_t evaluated_sample_count =
      request.sample_staged_speed && request.sample_control_sequence ?
          std::min(kMaximumSampleCount, sample_count +
              (config_.longitudinal_planning_enabled ? 4U : 10U)) : sample_count;

  for (std::size_t sample = 0U; sample < evaluated_sample_count; ++sample) {
    std::array<double, kParameterCount> values{};
    for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
      values[parameter] =
          mean_values[parameter] + scratch->noise[sample][parameter];
    }
    Parameters candidate =
        project(fromArray(values), request.bounds, request.side);
    if (!config_.longitudinal_planning_enabled &&
        request.sample_staged_speed && request.sample_control_sequence &&
        sample > 0U && sample <= 9U) {
      candidate.speed_scale = request.bounds.maximum.speed_scale;
      scratch->noise[sample][4U] = candidate.speed_scale - mean_values[4U];
    }
    ReferenceControlSequence candidate_control = control_mean;
    const ReferenceControlSequence *candidate_control_ptr = nullptr;
    if (request.sample_control_sequence) {
      for (std::size_t knot = 0U; knot < candidate_control.count; ++knot) {
        candidate_control.lateral_adjustment_m[knot] +=
            scratch->lateral_control_noise[sample][knot];
        candidate_control.speed_scale_adjustment[knot] +=
            scratch->speed_control_noise[sample][knot];
      }
      candidate_control = projectControlSequence(candidate_control);
      if (!config_.longitudinal_planning_enabled &&
          request.sample_staged_speed && sample > 0U && sample <= 9U) {
        // Keep sample zero warm. These absolute profiles do not inherit a
        // stale speed perturbation: fast, matching, early/late acceleration,
        // then the constant 0/20/40/60/80 percent execution-search proposals.
        candidate_control = stagedSpeedProfile(request, candidate, candidate_control, sample);
        // The weighted update must use the proposal actually evaluated, not
        // the Gaussian perturbation replaced by this deterministic seed.
        for (std::size_t knot = 0U; knot < candidate_control.count; ++knot) {
          scratch->speed_control_noise[sample][knot] =
              candidate_control.speed_scale_adjustment[knot] -
              control_mean.speed_scale_adjustment[knot];
        }
      }
      candidate_control_ptr = &candidate_control;
    }
    if (sample >= sample_count) {
      const std::size_t extra = sample - sample_count;
      candidate = mean;
      candidate_control = control_mean;
      candidate.speed_scale = request.bounds.maximum.speed_scale;
      // Cross three transition lengths with fast/measured/late profiles instead
      // of only speeding up a geometry already selected at low speed. Sample
      // zero already transports the accepted absolute speed; use the extra
      // warm geometry for the measured speed from execution search.
      std::size_t profile = 10U;
      if (extra > 0U) {
        const std::size_t geometry = config_.longitudinal_planning_enabled ?
            extra-1U : (extra - 1U) / 3U;
        const double ratio = static_cast<double>(geometry) / 2.0;
        candidate.l_out_m = request.bounds.minimum.l_out_m + ratio *
            (request.bounds.maximum.l_out_m - request.bounds.minimum.l_out_m);
        // The ordinary lateral sweep ties the wall-side extreme to the
        // longest transition. Cross the early-entry speed proposals with
        // that extreme too; retain the other two lengths at the warm offset.
        if (request.sample_lateral_bounds && geometry == 0U) {
          candidate.d_pass_m = request.side > 0
              ? request.bounds.maximum.d_pass_m : request.bounds.minimum.d_pass_m;
        }
        candidate.lateral_control_near_scale = request.bounds.maximum.lateral_control_near_scale;
        candidate.lateral_control_far_scale = request.bounds.maximum.lateral_control_far_scale;
        candidate_control.lateral_adjustment_m.fill(0.0);
        const std::size_t speed_variant = (extra - 1U) % 3U;
        profile = speed_variant == 0U ? 1U : speed_variant == 1U ? 10U : 4U;
      }
      if (!config_.longitudinal_planning_enabled)
        candidate_control = stagedSpeedProfile(request, candidate, candidate_control, profile);
      candidate_control_ptr = &candidate_control;
      const auto actual_values = asArray(candidate);
      for (std::size_t p = 0; p < kParameterCount; ++p)
        scratch->noise[sample][p] = actual_values[p] - mean_values[p];
      for (std::size_t knot = 0; knot < candidate_control.count; ++knot) {
        scratch->lateral_control_noise[sample][knot] =
            candidate_control.lateral_adjustment_m[knot] - control_mean.lateral_adjustment_m[knot];
        scratch->speed_control_noise[sample][knot] =
            candidate_control.speed_scale_adjustment[knot] - control_mean.speed_scale_adjustment[knot];
      }
    }
    if (config_.longitudinal_planning_enabled) {
      candidate.speed_scale = mean.speed_scale;
      candidate_control.speed_scale_adjustment.fill(0.);
    }
    // Clamping/absolute seeds change the proposal. Softmin updates must use
    // the parameters actually evaluated, not the discarded Gaussian draw.
    const auto actual_values=asArray(candidate);
    for(std::size_t p=0;p<kParameterCount;++p)
      scratch->noise[sample][p]=actual_values[p]-mean_values[p];
    if(request.sample_control_sequence) for(std::size_t k=0;k<candidate_control.count;++k) {
      scratch->lateral_control_noise[sample][k]=candidate_control.lateral_adjustment_m[k]-control_mean.lateral_adjustment_m[k];
      scratch->speed_control_noise[sample][k]=candidate_control.speed_scale_adjustment[k]-control_mean.speed_scale_adjustment[k];
    }
    bool duplicate=false;
    if(request.sample_staged_speed && request.sample_control_sequence) {
      for(std::size_t prior=0;prior<sample && !duplicate;++prior) {
        bool equal=true;
        for(std::size_t p=0;p<kParameterCount;++p)
          equal=equal && std::abs(scratch->noise[sample][p]-scratch->noise[prior][p])<=1e-12;
        for(std::size_t k=0;k<candidate_control.count;++k)
          equal=equal && std::abs(scratch->lateral_control_noise[sample][k]-scratch->lateral_control_noise[prior][k])<=1e-12 &&
              std::abs(scratch->speed_control_noise[sample][k]-scratch->speed_control_noise[prior][k])<=1e-12;
        duplicate=equal;
      }
    }
    if(duplicate) {
      ++result.duplicate_sample_count;
      // Do not strand the diagnostic cursor on a duplicate proposal.
      if (next_visualized_sample < visualized_sample_target) {
        const std::size_t requested = requestedVisualizedIndex();
        if (sample == requested) ++next_visualized_sample;
      }
      continue;
    }
    ++result.evaluated_sample_count;
    const Evaluation evaluation =
        evaluate(candidate, request, candidate_control_ptr,
                 &scratch->candidate_reference);
    if (next_visualized_sample < visualized_sample_target) {
      const std::size_t visualized_index = requestedVisualizedIndex();
      if (sample == visualized_index) {
        const std::size_t display_slot=result.visualized_sample_count;
        result.visualized_sample_references[display_slot] =
            scratch->candidate_reference;
        result.visualized_sample_evaluations[display_slot] =
            evaluation;
        result.visualized_sample_indices[display_slot] = sample;
        ++result.visualized_sample_count;
        ++next_visualized_sample;
      }
    }
    if (!evaluation.valid) {
      if (scratch->candidate_reference.count > 1U &&
          evaluation.progress_m > best_rejected_progress_m) {
        best_rejected_progress_m = evaluation.progress_m;
        best_rejected_parameters = candidate;
        best_rejected_control_sequence = candidate_control;
        best_rejected_evaluation = evaluation;
        // This scratch buffer is not used by the weighted update until the
        // valid-sample gate below has passed. Keep rejected diagnostics apart
        // from the best valid reference so its evaluation can be reused.
        scratch->updated_reference = scratch->candidate_reference;
        best_rejected_reference_available = true;
      }
      const std::size_t reason =
          static_cast<std::size_t>(evaluation.reject_reason);
      if (reason < result.reject_counts.size()) {
        const std::size_t count = ++result.reject_counts[reason];
        if (count > dominant_reject_count) {
          dominant_reject_count = count;
          result.dominant_rejection = evaluation;
        }
      }
      continue;
    }
    scratch->valid[sample] = true;
    scratch->costs[sample] = evaluation.cost;
    scratch->overtake_times[sample] = evaluation.overtake_time_sec;
    ++result.valid_sample_count;
    if (betterCandidate(evaluation.cost, evaluation.overtake_time_sec, best_cost, best_pass_time)) {
      best_cost = evaluation.cost;
      best_evaluation = evaluation;
      best_pass_time = evaluation.overtake_time_sec;
      best_parameters = candidate;
      best_control_sequence = candidate_control;
      scratch->best_raw_reference = scratch->candidate_reference;
    }
  }
  result.best_raw_cost = best_cost;
  const std::size_t required_valid =
      std::max(config_.minimum_valid_count,
               static_cast<std::size_t>(
                   std::ceil(config_.minimum_valid_ratio * result.evaluated_sample_count)));
  if (result.valid_sample_count < required_valid || !finite(best_cost)) {
    result.reject_reason = RejectReason::NO_VALID_SAMPLE;
    if (best_rejected_reference_available) {
      // Keep the farthest-progressing rejected proposal for diagnostics and
      // RViz. result.valid remains false, so this is never an executable path.
      result.selected = best_rejected_parameters;
      result.selected_control_sequence = best_rejected_control_sequence;
      result.selected_reference = scratch->updated_reference;
      result.selected_evaluation = best_rejected_evaluation;
    } else {
      result.selected_evaluation = result.dominant_rejection;
    }
    return finish();
  }

  double weight_sum = 0.0;
  double weight_square_sum = 0.0;
  std::array<double, kParameterCount> weighted_noise{};
  std::array<double, kMaximumControlKnotCount> weighted_lateral_noise{};
  std::array<double, kMaximumControlKnotCount> weighted_speed_noise{};
  for (std::size_t sample = 0U; sample < evaluated_sample_count; ++sample) {
    if (!scratch->valid[sample]) {
      continue;
    }
    const double weight = candidateWeight(scratch->costs[sample], scratch->overtake_times[sample],
                                         best_cost, best_pass_time, config_.temperature);
    if (!finite(weight)) {
      result.reject_reason = RejectReason::NON_FINITE;
      return finish();
    }
    weight_sum += weight;
    weight_square_sum += weight * weight;
    for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
      weighted_noise[parameter] += weight * scratch->noise[sample][parameter];
    }
    if (request.sample_control_sequence) {
      for (std::size_t knot = 0U; knot < control_mean.count; ++knot) {
        weighted_lateral_noise[knot] +=
            weight * scratch->lateral_control_noise[sample][knot];
        if (!config_.longitudinal_planning_enabled)
          weighted_speed_noise[knot] +=
              weight * scratch->speed_control_noise[sample][knot];
      }
    }
  }
  if (!finite(weight_sum) || weight_sum <= 0.0 || !finite(weight_square_sum) ||
      weight_square_sum <= 0.0) {
    result.reject_reason = RejectReason::NON_FINITE;
    return finish();
  }
  result.effective_sample_size = weight_sum * weight_sum / weight_square_sum;
  std::array<double, kParameterCount> updated_values = mean_values;
  for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
    updated_values[parameter] +=
        config_.update_gain * weighted_noise[parameter] / weight_sum;
  }
  result.updated_mean =
      project(fromArray(updated_values), request.bounds, request.side);
  result.updated_control_sequence = control_mean;
  if (request.sample_control_sequence) {
    for (std::size_t knot = 0U; knot < control_mean.count; ++knot) {
      result.updated_control_sequence.lateral_adjustment_m[knot] +=
          config_.update_gain * weighted_lateral_noise[knot] / weight_sum;
      if (config_.longitudinal_planning_enabled)
        result.updated_control_sequence.speed_scale_adjustment[knot] = 0.;
      else
        result.updated_control_sequence.speed_scale_adjustment[knot] +=
            config_.update_gain * weighted_speed_noise[knot] / weight_sum;
    }
    result.updated_control_sequence =
        projectControlSequence(result.updated_control_sequence);
  }
  const ReferenceControlSequence *updated_control_ptr =
      request.sample_control_sequence ? &result.updated_control_sequence
                                      : nullptr;
  Evaluation updated_evaluation =
      evaluate(result.updated_mean, request, updated_control_ptr,
               &scratch->updated_reference);
  result.updated_mean_cost = updated_evaluation.cost;

  if (updated_evaluation.valid &&
      (!best_evaluation.valid ||
       !betterEvaluation(best_evaluation, updated_evaluation))) {
    result.selected = result.updated_mean;
    result.selected_control_sequence = result.updated_control_sequence;
    result.selected_reference = scratch->updated_reference;
    result.selected_evaluation = updated_evaluation;
  } else if (best_evaluation.valid) {
    result.selected = best_parameters;
    result.selected_control_sequence = best_control_sequence;
    result.selected_reference = scratch->best_raw_reference;
    result.selected_evaluation = best_evaluation;
  } else {
    result.reject_reason = updated_evaluation.reject_reason;
    result.selected_evaluation = updated_evaluation;
    return finish();
  }
  result.valid = true;
  result.reject_reason = RejectReason::NONE;
  if (!config_.longitudinal_planning_enabled &&
      request.speed_pair_diagnostics && request.sample_staged_speed &&
      request.sample_control_sequence && result.selected_control_sequence.count > 1U) {
    // These profiles are already part of joint sampling. Keep the optional
    // frozen-geometry counterfactuals for visualization, without another
    // optimization pass or any effect on the selected candidate/warm state.
    const auto selected_control = result.selected_control_sequence;
    auto parameters = result.selected;
    parameters.speed_scale = request.bounds.maximum.speed_scale;
    for (std::size_t sample = 1U; sample <= 4U; ++sample) {
      const auto control = stagedSpeedProfile(
          request, parameters, selected_control, sample);
      TemporaryReference reference;
      Evaluation evaluation;
      evaluation.reject_stage = "generation";
      if (generateReference(parameters, request, &control, &reference, &evaluation)) {
        bool same = reference.count == result.selected_reference.count;
        for (std::size_t i = 0; same && i < reference.count; ++i) {
          const auto &a = reference.points[i];
          const auto &b = result.selected_reference.points[i];
          same = a.x_m == b.x_m && a.y_m == b.y_m && a.s_m == b.s_m &&
              a.d_m == b.d_m && a.yaw_rad == b.yaw_rad &&
              a.curvature_1pm == b.curvature_1pm && a.speed_mps == b.speed_mps &&
              a.uncapped_speed_mps == b.uncapped_speed_mps;
        }
        evaluation = same ? result.selected_evaluation :
            evaluateReference(parameters, request, &control, &reference);
      }
      auto &diag = result.speed_pairs[sample - 1U];
      diag.reference = std::make_shared<const TemporaryReference>(reference);
      ++result.speed_pair_count;
      diag.evaluation = evaluation;
      diag.valid = evaluation.valid;
      diag.reason = evaluation.reject_reason;
      diag.cost = evaluation.cost;
      diag.terms = evaluation.cost_terms;
      if (reference.count > 0U) {
        diag.first_speed = reference.points[0].speed_mps;
        diag.first_uncapped_speed = reference.points[0].uncapped_speed_mps;
      }
      for (std::size_t i = 0; i < std::min(reference.count, result.selected_reference.count); ++i) {
        diag.geometry_delta_m = std::max(diag.geometry_delta_m,
            std::hypot(reference.points[i].x_m - result.selected_reference.points[i].x_m,
                       reference.points[i].y_m - result.selected_reference.points[i].y_m));
      }
    }
  }
  if(!request.defer_warm_update) accept(request,result);
  return finish();
}

void ReferenceSpaceMppiPlanner::accept(const PlanRequest &request,const PlanResult &result) {
  if(!result.valid || result.generation!=request.generation || !finiteExecutionInput(request) ||
      (warm_mean_valid_ && request.stamp_sec<warm_stamp_sec_)) return;
  warm_mean_=result.selected; warm_mean_valid_=true;
  warm_side_=request.side; warm_phase_=request.phase; warm_semantic_key_=request.semantic_key;
  warm_control_sequence_=result.selected_control_sequence;
  warm_control_sequence_valid_=request.sample_control_sequence &&
      warm_control_sequence_.count==config_.control_knot_count;
  warm_stamp_sec_=request.stamp_sec; warm_anchor_s_m_=request.anchor_s_m;
  warm_control_horizon_m_=std::max(1.0,
      request.base_reference[request.base_reference_count-1].s_m-request.anchor_s_m);
  warm_speed_horizon_m_=std::max(1.0,
      request.base_reference[request.base_reference_count-1].s_m-request.base_reference[0].s_m);
  warm_base_reference_=request.base_reference;
  warm_base_reference_count_=request.base_reference_count;
}

} // namespace reference_space_mppi_planner::mppi
