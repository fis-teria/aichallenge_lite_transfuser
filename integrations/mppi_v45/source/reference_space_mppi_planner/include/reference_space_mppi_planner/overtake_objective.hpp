#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"

namespace reference_space_mppi_planner::mppi {

// Ranking evidence, not an alternative safety check. The caller must retain
// the uncertainty-aware swept collision and wall validation unchanged.
inline double predictedOvertakeTime(const PlanRequest &request, const Config &config,
                                   const Evaluation &evaluation) {
  const double unknown = std::numeric_limits<double>::infinity();
  if (!evaluation.valid || !std::isfinite(evaluation.cost) ||
      request.phase != Phase::OVERTAKE || !request.world_reference ||
      request.overtake_target_index >= request.dynamic_obstacle_count ||
      request.overtake_target_index >= request.dynamic_obstacles.size() ||
      evaluation.predicted_rollout_count == 0U ||
      evaluation.predicted_rollout_count > evaluation.predicted_rollout.size()) return unknown;
  const auto &world = *request.world_reference;
  const auto &target = request.dynamic_obstacles[request.overtake_target_index];
  const auto margin = [&](const RolloutState &state) {
    const auto predicted = opponent_prediction::positionAt(target, state.time_sec);
    const double origin = predicted.global_s;
    const auto opponent = world.pose(origin, predicted.d);
    if (!opponent || !std::isfinite(state.x_m) || !std::isfinite(state.y_m) ||
        !std::isfinite(state.yaw_rad) || !std::isfinite(state.time_sec)) return -unknown;
    const double opponent_yaw = (*opponent)[2] + predicted.relative_yaw;
    double rear = unknown, front = -unknown;
    for (double longitudinal : {-config.vehicle_half_length_m, config.vehicle_half_length_m}) {
      for (double lateral : {-config.vehicle_half_width_m, config.vehicle_half_width_m}) {
        const auto ego_s = world.projectStation(
            state.x_m + longitudinal*std::cos(state.yaw_rad) - lateral*std::sin(state.yaw_rad),
            state.y_m + longitudinal*std::sin(state.yaw_rad) + lateral*std::cos(state.yaw_rad));
        const auto opponent_s = world.projectStation(
            (*opponent)[0] + longitudinal*std::cos(opponent_yaw) - lateral*std::sin(opponent_yaw),
            (*opponent)[1] + longitudinal*std::sin(opponent_yaw) + lateral*std::cos(opponent_yaw));
        if (!ego_s || !opponent_s) return -unknown;
        rear = std::min(rear, world.stationDifference(*ego_s, origin));
        front = std::max(front, world.stationDifference(*opponent_s, origin));
      }
    }
    return rear - front;
  };
  const double initial = margin({request.ego.x_m, request.ego.y_m, request.ego.yaw_rad,
                                 request.ego.speed_mps, 0.0});
  // Already passed, unknown projection, or no terminal passage: old cost rank.
  if (!std::isfinite(initial) || initial > 0.0) return unknown;
  const auto count = evaluation.predicted_rollout_count;
  if (!(margin(evaluation.predicted_rollout[count-1U]) > 0.0)) return unknown;
  double passage = evaluation.predicted_rollout[count-1U].time_sec;
  for (std::size_t i = count-1U; i > 0U; --i) {
    const auto &previous = evaluation.predicted_rollout[i-1U];
    if (!(previous.time_sec < passage)) return unknown;
    const double separation = margin(previous);
    if (!std::isfinite(separation)) return unknown;
    if (separation <= 0.0) break;
    passage = previous.time_sec;
  }
  return passage > 0.0 ? passage : unknown;
}

} // namespace reference_space_mppi_planner::mppi
