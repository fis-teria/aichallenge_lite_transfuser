#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace reference_space_mppi_planner::mppi {

// Diagnostic only: rigid rectangles in ONE planar frame, with a rectangular
// set of relative translations. Frenet on a curve is not such a rigid frame.
// A clear result must never override the production curved-road validator.
struct LocalEnvelopeDiagnostic {
  bool valid{false};
  bool overlap{false};
  double separating_gap_m{std::numeric_limits<double>::quiet_NaN()};
};

inline LocalEnvelopeDiagnostic localEnvelopeDiagnostic(
    double relative_s, double relative_d, double ego_yaw, double opponent_yaw,
    double half_length, double half_width, double uncertainty_s,
    double uncertainty_d) {
  for (double x : {relative_s, relative_d, ego_yaw, opponent_yaw,
                   half_length, half_width, uncertainty_s, uncertainty_d})
    if (!std::isfinite(x)) return {};
  if (half_length <= 0 || half_width <= 0 || uncertainty_s < 0 || uncertainty_d < 0)
    return {};
  const double ec=std::cos(ego_yaw), es=std::sin(ego_yaw);
  const double oc=std::cos(opponent_yaw), os=std::sin(opponent_yaw);
  // These are all edge normals of the Minkowski sum of both rectangles and
  // the translation box. Testing only the two road axes gives the old AABB.
  const std::array<std::array<double,2>,6> axes{{
      {1,0},{0,1},{ec,es},{-es,ec},{oc,os},{-os,oc}}};
  double gap=-std::numeric_limits<double>::infinity();
  for (const auto &axis: axes) {
    const double x=axis[0],y=axis[1];
    const double radius=half_length*(std::abs(x*ec+y*es)+std::abs(x*oc+y*os))+
        half_width*(std::abs(-x*es+y*ec)+std::abs(-x*os+y*oc))+
        uncertainty_s*std::abs(x)+uncertainty_d*std::abs(y);
    const double distance=std::abs(x*relative_s+y*relative_d);
    if (!std::isfinite(radius) || !std::isfinite(distance)) return {};
    gap=std::max(gap,distance-radius);
  }
  // Boundary contact is overlap. This is an axis gap, not Euclidean distance.
  return {true,gap<=1e-9,gap};
}

} // namespace reference_space_mppi_planner::mppi
