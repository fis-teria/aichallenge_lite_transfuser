#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace reference_space_mppi_planner::mppi {
struct PathProjection {
  bool valid{false};
  std::size_t lower_index{0};
  double ratio{0.0};
  double x_m{0.0}, y_m{0.0};
  double s_m{0.0}, d_m{0.0};
  double tangent_x{1.0}, tangent_y{0.0};
  double distance_m{std::numeric_limits<double>::infinity()};
};

// Keep the clamped Cartesian point. At an endpoint, d alone cannot encode
// the residual along the tangent and cannot reconstruct this projection.
template <class Points, class Position>
PathProjection projectTrajectory(const Points &points, Position position,
                                 double x, double y) {
  PathProjection best;
  double arc = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto a = position(points[i-1]), b = position(points[i]);
    const double dx = b[0]-a[0], dy = b[1]-a[1], length = std::hypot(dx,dy);
    if (!std::isfinite(length) || length < 1e-4) continue;
    const double u = std::clamp(((x-a[0])*dx+(y-a[1])*dy)/(length*length),0.,1.);
    const double px = a[0]+u*dx, py = a[1]+u*dy;
    const double ex = x-px, ey = y-py, error = std::hypot(ex,ey);
    if (error < best.distance_m) {
      best = {true,i-1,u,px,py,arc+u*length,(dx*ey-dy*ex)/length,
              dx/length,dy/length,error};
    }
    arc += length;
  }
  return best;
}
}  // namespace reference_space_mppi_planner::mppi
