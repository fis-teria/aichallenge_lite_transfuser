#pragma once
#include <algorithm>
#include <cmath>
namespace reference_space_mppi_planner::mppi {
// Installed AWSIM Vehicle/vehicle.yaml and VehicleRosInput plant response.
inline double awsimSteeringResponse(double current, double target, double dt,
                                    double time_constant) {
  constexpr double rate = 1.0471975511965976; // Physical tire angle, 60 deg/s.
  for (double remaining = dt; remaining > 1e-12;) {
    const double step = std::min(.005, remaining);
    current += std::clamp(step / (time_constant + step) * (target - current),
                          -rate * step, rate * step);
    remaining -= step;
  }
  return current;
}
class AwsimLongitudinalResponse {
public:
  double acceleration(double requested, double speed, double time_sec) {
    // The unknown initial Unity 10 Hz update phase is approximated by the
    // first rollout step. Hold subsequent commands between input updates.
    if (time_sec + 1e-9 >= next_update_sec_) {
      input_ = std::clamp(requested, -3.0, 1.37);
      next_update_sec_ = time_sec + .1;
    }
    const double cap = .37 + (1.37 - .37) *
        (1. - std::pow(std::clamp(speed / 10., 0., 1.), 40.));
    return (input_ > 0. ? std::min(input_, cap) : input_) -
           (speed > 0. ? .37 : 0.) - .03 * speed;
  }
private:
  double input_{0.}, next_update_sec_{0.};
};
} // namespace reference_space_mppi_planner::mppi
