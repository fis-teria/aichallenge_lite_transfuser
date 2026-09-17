#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace reference_space_mppi_planner {
inline double spatialKnotRatio(double span, std::size_t count, double reference_spacing) {
  return reference_spacing > 0.0 && count > 1U
      ? std::max(1.0e-3, span / (count - 1U) / reference_spacing) : 1.0;
}
inline double spatialLateralAmplitude(double ratio) {
  // A displacement's curvature scales as amplitude / spacing squared.
  return std::min(1.0, ratio * ratio);
}
inline double spatialCorrelation(double correlation, double ratio) {
  return std::pow(correlation, ratio);
}
} // namespace reference_space_mppi_planner
