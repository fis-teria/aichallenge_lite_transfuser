#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace reference_space_mppi_planner {

inline double localHorizonDistance(double speed, double minimum, double maximum,
                                  double lookahead_sec) {
  return std::clamp(std::abs(speed) * lookahead_sec, minimum, maximum);
}

// Geometry covers both consumption before replenishment and the local
// validation window at the greatest reachable speed. This does not increase
// the physical prediction time used to validate a command now.
inline double suppliedReferenceDistance(double speed, double minimum, double maximum,
                                       double lookahead_sec, double supply_sec,
                                       double speed_cap, double acceleration) {
  const double initial = std::abs(speed);
  const double cap = std::max(initial, speed_cap);
  const double accelerating = acceleration > 0. ?
      std::min(supply_sec, (cap-initial)/acceleration) : 0.;
  const double future_speed = initial + acceleration*accelerating;
  const double consumed = initial*accelerating + .5*acceleration*accelerating*accelerating +
      future_speed*(supply_sec-accelerating);
  return consumed + localHorizonDistance(future_speed, minimum, maximum, lookahead_sec);
}

// All candidates in a batch use the same reachable-distance upper bound.
// Slower samples do not receive a longer obstacle-prediction window.
inline std::size_t localHorizonSteps(double distance, double preview_reserve,
                                    double speed, double speed_cap,
                                    double acceleration, double dt,
                                    std::size_t maximum_steps) {
  double travelled = 0.0;
  double upper_speed = std::abs(speed);
  const double cap = std::max(upper_speed, speed_cap);
  std::size_t steps = 0U;
  for (; steps < maximum_steps; ++steps) {
    upper_speed = std::min(cap, upper_speed + acceleration * dt);
    const double next = travelled + upper_speed * dt;
    if (next + preview_reserve > distance) break;
    travelled = next;
  }
  return steps;
}

inline double previousKnotCoordinate(double new_coordinate, double new_length,
                                     double travelled, double old_length) {
  return std::clamp((new_coordinate * new_length + travelled) /
                        std::max(1.0, old_length),
                    0.0, 1.0);
}

} // namespace reference_space_mppi_planner
