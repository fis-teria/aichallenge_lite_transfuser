#ifndef SIMPLE_PURE_PURSUIT__CURVATURE_FEEDFORWARD_HPP_
#define SIMPLE_PURE_PURSUIT__CURVATURE_FEEDFORWARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace simple_pure_pursuit
{

template<typename Points, typename Position>
double estimateDistanceWindowSignedCurvature(
  const Points & points, const std::size_t nearest_index,
  const double preview_distance_m, Position position)
{
  if (points.size() < 3U || nearest_index >= points.size() ||
    !std::isfinite(preview_distance_m) || preview_distance_m <= 0.0)
  {
    return 0.0;
  }
  const std::size_t first = nearest_index > 0U ? nearest_index - 1U : 0U;
  std::size_t last = first;
  double total_arc_m = 0.0;
  for (std::size_t i = first + 1U; i < points.size(); ++i) {
    const auto previous = position(points[i - 1U]);
    const auto current = position(points[i]);
    const double ds = std::hypot(current.first - previous.first, current.second - previous.second);
    if (!std::isfinite(ds)) {
      return 0.0;
    }
    total_arc_m += ds;
    last = i;
    if (total_arc_m >= preview_distance_m && last >= first + 2U) {
      break;
    }
  }
  if (last < first + 2U || total_arc_m <= 1.0e-6) {
    return 0.0;
  }

  const double half_arc_m = 0.5 * total_arc_m;
  std::size_t middle = first + 1U;
  double arc_m = 0.0;
  double best_error_m = half_arc_m;
  for (std::size_t i = first + 1U; i < last; ++i) {
    const auto previous = position(points[i - 1U]);
    const auto current = position(points[i]);
    arc_m += std::hypot(current.first - previous.first, current.second - previous.second);
    const double error_m = std::abs(arc_m - half_arc_m);
    if (error_m < best_error_m) {
      best_error_m = error_m;
      middle = i;
    }
  }

  const auto a = position(points[first]);
  const auto b = position(points[middle]);
  const auto c = position(points[last]);
  const double ab = std::hypot(b.first - a.first, b.second - a.second);
  const double bc = std::hypot(c.first - b.first, c.second - b.second);
  const double ca = std::hypot(a.first - c.first, a.second - c.second);
  const double denominator = ab * bc * ca;
  if (!std::isfinite(denominator) || denominator <= 1.0e-6) {
    return 0.0;
  }
  const double twice_signed_area =
    (b.first - a.first) * (c.second - a.second) -
    (b.second - a.second) * (c.first - a.first);
  return 2.0 * twice_signed_area / denominator;
}

inline double timeDomainLowPass(
  const double sample, const double previous, const double dt_sec,
  const double time_constant_sec)
{
  if (!std::isfinite(sample)) {
    return std::isfinite(previous) ? previous : 0.0;
  }
  if (!std::isfinite(previous) || !std::isfinite(dt_sec) || dt_sec <= 0.0 ||
    !std::isfinite(time_constant_sec) || time_constant_sec <= 0.0)
  {
    return sample;
  }
  const double alpha = std::clamp(1.0 - std::exp(-dt_sec / time_constant_sec), 0.0, 1.0);
  return previous + alpha * (sample - previous);
}

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT__CURVATURE_FEEDFORWARD_HPP_
