#pragma once

#include <cmath>

namespace reference_space_mppi_planner {

constexpr double kAvoidanceMaximumLeaderSpeedMps = 20.0 / 3.6;
constexpr double kEarlyAvoidanceMaximumLeaderSpeedMps = 5.0 / 3.6;

inline bool isAvoidanceSpeed(double speed_mps) noexcept {
  return std::isfinite(speed_mps) && speed_mps <= kAvoidanceMaximumLeaderSpeedMps;
}

// Callers select a fresh, forward, path-conflicting vehicle. Early lateral
// planning does not change the longitudinal following gap.
inline bool isAvoidanceEligible(double speed_mps, double body_gap_m,
                                double follow_gap_m) noexcept {
  return isAvoidanceSpeed(speed_mps) && std::isfinite(body_gap_m) &&
         (speed_mps <= kEarlyAvoidanceMaximumLeaderSpeedMps || body_gap_m <= follow_gap_m);
}

inline double followingGapForSpeed(double speed_mps, double avoidance_gap_m,
                                  double overtake_gap_m) noexcept {
  return std::isfinite(speed_mps) && !isAvoidanceSpeed(speed_mps)
      ? overtake_gap_m : avoidance_gap_m;
}

}  // namespace reference_space_mppi_planner
