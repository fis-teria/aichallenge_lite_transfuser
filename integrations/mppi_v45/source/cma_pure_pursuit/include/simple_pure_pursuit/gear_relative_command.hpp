#ifndef SIMPLE_PURE_PURSUIT_GEAR_RELATIVE_COMMAND_HPP_
#define SIMPLE_PURE_PURSUIT_GEAR_RELATIVE_COMMAND_HPP_

#include <cmath>

namespace simple_pure_pursuit
{

struct GearRelativeLongitudinalCommand
{
  bool valid{false};
  bool reverse_conversion_applied{false};
  bool direction_mismatch{false};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

// Planner trajectories keep reverse motion negative so Pure Pursuit can use
// the travel direction for geometry and feedback. AWSIM interprets speed and
// acceleration relative to the selected gear, so only the vehicle-facing
// command boundary converts values while reverse remains selected, including
// the zero-speed stopping phase before DRIVE is confirmed.
inline GearRelativeLongitudinalCommand applyGearRelativeReverseCommand(
  const double signed_target_speed_mps, const double signed_acceleration_mps2,
  const bool enabled, const bool reverse_gear_active)
{
  GearRelativeLongitudinalCommand result;
  if (!std::isfinite(signed_target_speed_mps) ||
    !std::isfinite(signed_acceleration_mps2))
  {
    return result;
  }

  result.speed_mps = signed_target_speed_mps;
  result.acceleration_mps2 = signed_acceleration_mps2;
  if (!enabled) {
    result.valid = true;
    return result;
  }

  const bool reverse_trajectory = signed_target_speed_mps < 0.0;
  const bool forward_trajectory = signed_target_speed_mps > 0.0;
  if ((reverse_gear_active && forward_trajectory) ||
    (!reverse_gear_active && reverse_trajectory))
  {
    result.speed_mps = 0.0;
    result.acceleration_mps2 = 0.0;
    result.direction_mismatch = true;
    result.valid = true;
    return result;
  }

  if (reverse_gear_active) {
    result.speed_mps = -signed_target_speed_mps;
    result.acceleration_mps2 = -signed_acceleration_mps2;
    result.reverse_conversion_applied = true;
  }
  result.valid = true;
  return result;
}

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_GEAR_RELATIVE_COMMAND_HPP_
