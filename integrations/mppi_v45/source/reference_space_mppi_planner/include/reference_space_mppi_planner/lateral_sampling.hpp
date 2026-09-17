#pragma once

#include <algorithm>
#include <cmath>

namespace reference_space_mppi_planner::lateral_sampling {

struct Interval {
  bool feasible{false};
  double minimum_separation_m{0.0};
  double maximum_separation_m{0.0};
  double nominal_separation_m{0.0};
};

inline double minimumTransitionLength(double ego_speed_mps) noexcept {
  // Preserve the gentle high-speed entry, but allow a slow kart to move
  // sideways before reaching a nearby opponent instead of waiting 10+ metres.
  return std::clamp(2.5 * std::abs(ego_speed_mps), 4.0, 15.0);
}

inline double gentleTransitionLength(double speed_mps, double lateral_change_m,
                                     double lateral_acceleration_mps2) noexcept {
  // Peak |d''(u)| of the quintic smoothstep is 10/sqrt(3). This sizes
  // candidate geometry on a straight; the CMA rollout still checks the road,
  // steering response and obstacles independently.
  return std::clamp(std::abs(speed_mps) *
      std::sqrt((10.0 / std::sqrt(3.0)) * std::abs(lateral_change_m) /
                lateral_acceleration_mps2), 4.0, 35.0);
}

inline Interval feasibleSeparationIntervalFromAvailableWidth(
    double maximum_available_separation_m, double ego_half_width_m,
    double opponent_half_width_m, double obstacle_lateral_inflation_m,
    double lateral_uncertainty_m, double configured_minimum_separation_m,
    double preferred_separation_m) noexcept {
  const double values[] = {
      maximum_available_separation_m, ego_half_width_m,
      opponent_half_width_m,          obstacle_lateral_inflation_m,
      lateral_uncertainty_m,          configured_minimum_separation_m,
      preferred_separation_m,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return {};
    }
  }
  if (maximum_available_separation_m < 0.0 || ego_half_width_m < 0.0 ||
      opponent_half_width_m < 0.0 || obstacle_lateral_inflation_m < 0.0 ||
      lateral_uncertainty_m < 0.0 || configured_minimum_separation_m < 0.0 ||
      preferred_separation_m < 0.0) {
    return {};
  }

  const double collision_separation_m =
      std::max(obstacle_lateral_inflation_m,
               ego_half_width_m + opponent_half_width_m) +
      lateral_uncertainty_m;
  const double minimum_separation_m =
      std::max(collision_separation_m, configured_minimum_separation_m);
  if (maximum_available_separation_m + 1.0e-9 < minimum_separation_m) {
    return Interval{false, minimum_separation_m, maximum_available_separation_m,
                    0.0};
  }
  return Interval{
      true,
      minimum_separation_m,
      maximum_available_separation_m,
      std::clamp(preferred_separation_m, minimum_separation_m,
                 maximum_available_separation_m),
  };
}

// wall_center_limit_m is already eroded by the ego footprint.  The returned
// interval therefore combines the remaining wall space with the two-vehicle
// collision envelope without subtracting the ego width a second time.
inline Interval feasibleSeparationInterval(
    int side, double target_d_m, double wall_center_limit_m,
    double ego_half_width_m, double opponent_half_width_m,
    double obstacle_lateral_inflation_m, double lateral_uncertainty_m,
    double configured_minimum_separation_m,
    double preferred_separation_m) noexcept {
  const double values[] = {
      target_d_m,
      wall_center_limit_m,
      ego_half_width_m,
      opponent_half_width_m,
      obstacle_lateral_inflation_m,
      lateral_uncertainty_m,
      configured_minimum_separation_m,
      preferred_separation_m,
  };
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return {};
    }
  }
  if ((side != 1 && side != -1) || ego_half_width_m < 0.0 ||
      opponent_half_width_m < 0.0 || obstacle_lateral_inflation_m < 0.0 ||
      lateral_uncertainty_m < 0.0 || configured_minimum_separation_m < 0.0 ||
      preferred_separation_m < 0.0) {
    return {};
  }

  const double maximum_separation_m = side > 0
                                          ? wall_center_limit_m - target_d_m
                                          : target_d_m - wall_center_limit_m;
  return feasibleSeparationIntervalFromAvailableWidth(
      maximum_separation_m, ego_half_width_m, opponent_half_width_m,
      obstacle_lateral_inflation_m, lateral_uncertainty_m,
      configured_minimum_separation_m, preferred_separation_m);
}

} // namespace reference_space_mppi_planner::lateral_sampling
