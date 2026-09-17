#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace reference_space_mppi_planner::mppi {
struct SegmentArrival {
  double time_sec;
  double speed_mps;
};

// A particular idealized constant-acceleration policy, then cruising. This is
// not the arrival time of the PP/actuator closed loop and must never certify a
// moving-obstacle interaction. Used only for proposal/analytical utilities.
// A stopped vehicle cannot traverse the remaining distance at fictitious creep.
inline SegmentArrival segmentArrival(double distance_m, double initial_speed,
    double target_speed, double acceleration, double deceleration) {
  const double infinity=std::numeric_limits<double>::infinity();
  for(double value:{distance_m,initial_speed,target_speed,acceleration,deceleration})
    if(!std::isfinite(value)) return {infinity,0.0};
  if(distance_m<0.0 || initial_speed<0.0 || target_speed<0.0 ||
     acceleration<0.0 || deceleration<0.0) return {infinity,0.0};
  const double v = std::max(0.0, initial_speed);
  const double target = std::max(0.0, target_speed);
  if (distance_m <= 0.0) return {0.0, v};
  if (v == target) return {
      v > 0.0 ? distance_m / v : std::numeric_limits<double>::infinity(), v};
  const double a = target > v ? acceleration : -deceleration;
  if (a == 0.0) return {v>0.0 ? distance_m/v : infinity,v};
  const double transition_distance = (target * target - v * v) / (2.0 * a);
  if (distance_m <= transition_distance) {
    const double end = std::sqrt(std::max(0.0, v * v + 2.0 * a * distance_m));
    return {2.0 * distance_m / (v + end), end};
  }
  const double transition_time = (target - v) / a;
  return {target > 0.0 ? transition_time + (distance_m - transition_distance) / target :
      std::numeric_limits<double>::infinity(), target};
}
}  // namespace reference_space_mppi_planner::mppi
