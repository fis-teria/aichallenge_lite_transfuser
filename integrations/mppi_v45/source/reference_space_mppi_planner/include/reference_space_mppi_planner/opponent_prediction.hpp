#pragma once

#include <algorithm>
#include <cmath>

namespace reference_space_mppi_planner::opponent_prediction {

inline double boundedArrivalTime(double distance_m, double initial_speed_mps,
                                 double cruise_speed_mps,
                                 double acceleration_mps2,
                                 double horizon_sec) noexcept {
  if (!std::isfinite(distance_m) || !std::isfinite(initial_speed_mps) ||
      !std::isfinite(cruise_speed_mps) || !std::isfinite(acceleration_mps2) ||
      !std::isfinite(horizon_sec) || distance_m <= 0.0 || horizon_sec <= 0.0) {
    return 0.0;
  }

  const double initial_speed = std::max(0.0, initial_speed_mps);
  const double cruise_speed = std::max({0.5, initial_speed, cruise_speed_mps});
  const double acceleration = std::max(0.0, acceleration_mps2);
  if (acceleration <= 1.0e-6 || initial_speed >= cruise_speed - 1.0e-6) {
    return std::min(horizon_sec, distance_m / cruise_speed);
  }

  const double acceleration_time =
      (cruise_speed - initial_speed) / acceleration;
  const double acceleration_distance =
      initial_speed * acceleration_time +
      0.5 * acceleration * acceleration_time * acceleration_time;
  double arrival_time = 0.0;
  if (distance_m <= acceleration_distance) {
    arrival_time =
        (-initial_speed + std::sqrt(initial_speed * initial_speed +
                                    2.0 * acceleration * distance_m)) /
        acceleration;
  } else {
    arrival_time =
        acceleration_time + (distance_m - acceleration_distance) / cruise_speed;
  }
  return std::clamp(arrival_time, 0.0, horizon_sec);
}

inline double lateralPositionAtArrival(double current_d_m,
                                       double lateral_speed_mps,
                                       double distance_m, double ego_speed_mps,
                                       double cruise_speed_mps,
                                       double acceleration_mps2,
                                       double horizon_sec) noexcept {
  const double values[] = {current_d_m,   lateral_speed_mps, distance_m,
                           ego_speed_mps, cruise_speed_mps,  acceleration_mps2,
                           horizon_sec};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return current_d_m;
    }
  }
  return current_d_m +
         lateral_speed_mps * boundedArrivalTime(distance_m, ego_speed_mps,
                                                cruise_speed_mps,
                                                acceleration_mps2, horizon_sec);
}

} // namespace reference_space_mppi_planner::opponent_prediction
