#pragma once

#include <cmath>

namespace simple_pure_pursuit
{

inline double steeringForTravelDirection(
  const double forward_geometry_steering_rad,
  const double target_velocity_mps)
{
  if (!std::isfinite(forward_geometry_steering_rad) ||
    !std::isfinite(target_velocity_mps))
  {
    return 0.0;
  }
  // Pure Pursuit geometry is expressed in the vehicle-forward frame.  When
  // following a trajectory in reverse the bicycle yaw response changes sign,
  // so the same geometric path requires the opposite front-wheel command.
  return target_velocity_mps < -1.0e-3 ?
         -forward_geometry_steering_rad : forward_geometry_steering_rad;
}

}  // namespace simple_pure_pursuit
