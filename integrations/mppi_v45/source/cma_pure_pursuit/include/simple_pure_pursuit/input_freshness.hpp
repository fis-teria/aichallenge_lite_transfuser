#pragma once

#include <cmath>
#include <cstdint>

namespace simple_pure_pursuit {

inline bool inputSampleFresh(bool received, double now_sec, double received_sec,
                             double timeout_sec) noexcept {
  if (!received || !std::isfinite(now_sec) || !std::isfinite(received_sec) ||
      !std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    return false;
  }
  const double age_sec = now_sec - received_sec;
  return age_sec >= -1.0e-6 && age_sec <= timeout_sec + 1.0e-6;
}

inline bool
directTrajectoryGenerationAccepted(bool has_current,
                                   std::uint64_t current_generation,
                                   std::uint64_t incoming_generation) noexcept {
  return !has_current || incoming_generation >= current_generation;
}

} // namespace simple_pure_pursuit
