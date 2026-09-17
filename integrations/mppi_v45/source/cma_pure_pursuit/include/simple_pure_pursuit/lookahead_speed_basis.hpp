#ifndef SIMPLE_PURE_PURSUIT__LOOKAHEAD_SPEED_BASIS_HPP_
#define SIMPLE_PURE_PURSUIT__LOOKAHEAD_SPEED_BASIS_HPP_

#include <cmath>
#include <string>

namespace simple_pure_pursuit
{

inline bool isMeasuredSpeedLookaheadMode(const std::string & mode)
{
  return mode == "FOLLOW_BLOCKED" ||
         mode == "PREPARE_OVERTAKE_LEFT" ||
         mode == "PREPARE_OVERTAKE_RIGHT" ||
         mode == "OVERTAKE_LEFT" ||
         mode == "OVERTAKE_RIGHT" ||
         mode == "SIDE_BY_SIDE_KEEP" ||
         mode == "MERGE_BACK";
}

inline double selectLookaheadSpeedBasis(
  const double target_velocity_mps, const double measured_velocity_mps,
  const bool maneuver_measured_speed_enabled, const bool mode_fresh,
  const std::string & mode)
{
  const bool use_measured = maneuver_measured_speed_enabled && mode_fresh &&
    isMeasuredSpeedLookaheadMode(mode) && std::isfinite(measured_velocity_mps);
  const double selected = use_measured ? measured_velocity_mps : target_velocity_mps;
  return std::isfinite(selected) ? std::abs(selected) : 0.0;
}

inline double smoothManeuverLookaheadDistance(
  const double raw_distance_m, const double previous_distance_m,
  const bool previous_initialized, const double dt_sec,
  const double time_constant_sec, const bool smoothing_enabled,
  const bool mode_fresh, const std::string & mode)
{
  const bool eligible = smoothing_enabled && mode_fresh &&
    isMeasuredSpeedLookaheadMode(mode) && previous_initialized &&
    std::isfinite(previous_distance_m) && std::isfinite(dt_sec) && dt_sec > 0.0 &&
    std::isfinite(time_constant_sec) && time_constant_sec > 0.0;
  if (!eligible || !std::isfinite(raw_distance_m)) {
    return std::isfinite(raw_distance_m) ? raw_distance_m : 0.0;
  }
  const double alpha = 1.0 - std::exp(-dt_sec / time_constant_sec);
  return previous_distance_m + alpha * (raw_distance_m - previous_distance_m);
}

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT__LOOKAHEAD_SPEED_BASIS_HPP_
