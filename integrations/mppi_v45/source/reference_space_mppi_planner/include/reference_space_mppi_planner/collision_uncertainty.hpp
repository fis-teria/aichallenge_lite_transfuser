#pragma once

#include <algorithm>

namespace reference_space_mppi_planner::mppi {

// Only hard rejection tapers the uncertainty. Keep the full prediction for
// soft clearance costs; nominal vehicle geometry is checked at every time.
inline double collisionUncertaintyScale(double time_sec) {
  return std::clamp(3.0 - time_sec, 0.0, 1.0);
}

}  // namespace reference_space_mppi_planner::mppi
