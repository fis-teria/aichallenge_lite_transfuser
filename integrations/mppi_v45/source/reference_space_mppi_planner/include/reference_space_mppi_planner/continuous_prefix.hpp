#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include "reference_space_mppi_planner/reference_space_mppi.hpp"

namespace reference_space_mppi_planner::mppi {

// Intersect an accepted segment with the base point's normal, not the reverse
// nearest-point projection (whose distance is in a different Frenet frame).
inline std::optional<double> offsetAtNormal(
    double x, double y, double yaw, double ax, double ay, double bx, double by) {
  const double tx = std::cos(yaw), ty = std::sin(yaw);
  const double a = (ax - x) * tx + (ay - y) * ty;
  const double b = (bx - x) * tx + (by - y) * ty;
  if (b - a <= 1e-9 || a > 0.0 || b < 0.0) return std::nullopt;
  const double t = -a / (b - a);
  return -(ax + t * (bx - ax) - x) * ty +
      (ay + t * (by - ay) - y) * tx;
}

// Replace the entire entry interval rather than pinning only its first point.
// Quintic Hermite boundaries match measured position/heading and the accepted
// profile's position, slope and second derivative at the join.
inline void connectMeasuredPrefix(PlanRequest &request, double length_m) {
  const auto n = request.base_reference_count;
  if (n < 3U) return;
  auto &points = request.base_reference;
  std::size_t end = 1U;
  while (end + 1U < n && points[end].s_m < length_m) ++end;
  const double length = points[end].s_m - points[0].s_m;
  if (length <= 1e-6) return;
  const double left_ds = points[end].s_m - points[end - 1U].s_m;
  const double left = (points[end].active_d_m - points[end - 1U].active_d_m) /
      std::max(left_ds, 1e-6);
  double slope = left, second = 0.0;
  if (end + 1U < n) {
    const double right_ds = points[end + 1U].s_m - points[end].s_m;
    const double right = (points[end + 1U].active_d_m - points[end].active_d_m) /
        std::max(right_ds, 1e-6);
    slope = (left * right_ds + right * left_ds) / (left_ds + right_ds);
    second = 2.0 * (right - left) / (left_ds + right_ds);
  }
  const double c0 = request.ego.d_m;
  const double c1 = length * (1.0 - points[0].curvature_1pm * c0) *
      std::tan(request.ego.yaw_rad - points[0].yaw_rad);
  const double p = points[end].active_d_m - c0 - c1;
  const double v = length * slope - c1;
  const double a = length * length * second;
  const double c3 = 10.0 * p - 4.0 * v + 0.5 * a;
  const double c4 = -15.0 * p + 7.0 * v - a;
  const double c5 = 6.0 * p - 3.0 * v + 0.5 * a;
  for (std::size_t i = 0; i < end; ++i) {
    const double t = (points[i].s_m - points[0].s_m) / length;
    // Do not independently clamp these points: collision/wall evaluation must
    // assess the continuous candidate, not a clipped, kinked replacement.
    points[i].active_d_m = c0 + c1 * t + t * t * t * (c3 + t * (c4 + t * c5));
    points[i].active_d_valid = true;
  }
}
}  // namespace reference_space_mppi_planner::mppi
