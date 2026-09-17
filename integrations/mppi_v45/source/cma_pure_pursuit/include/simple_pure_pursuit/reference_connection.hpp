#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
#include <tuple>

namespace simple_pure_pursuit::reference_connection {

struct Vector {
  double x, y;
  Vector operator+(Vector b) const { return {x + b.x, y + b.y}; }
  Vector operator-(Vector b) const { return {x - b.x, y - b.y}; }
  Vector operator*(double a) const { return {x * a, y * a}; }
  double norm() const { return std::hypot(x, y); }
};
template<class Point>
inline Vector xy(const Point &p) { return {p.x_m, p.y_m}; }
inline double cross(Vector a, Vector b) { return a.x * b.y - a.y * b.x; }
inline double curvature(Vector a, Vector b, Vector c) {
  const auto ab = b - a, bc = c - b, ac = c - a;
  const double denominator = ab.norm() * bc.norm() * ac.norm();
  return denominator > 1e-12 ? 2 * cross(ab, bc) / denominator : 0;
}

struct Quintic {
  // Relative coordinates avoid subtracting large world-frame polynomial terms.
  std::array<Vector, 6> c{};
  Vector position(double u) const {
    return c[0] + (c[1] + (c[2] + (c[3] + (c[4] + c[5] * u) * u) * u) * u) * u;
  }
  Vector tangent(double u) const {
    return c[1] + (c[2] * 2 + (c[3] * 3 + (c[4] * 4 + c[5] * (5 * u)) * u) * u) * u;
  }
  Vector second(double u) const {
    return c[2] * 2 + (c[3] * 6 + (c[4] * 12 + c[5] * (20 * u)) * u) * u;
  }
};
inline Quintic makeQuintic(Vector end, Vector start_tangent, Vector end_tangent,
    double start_curvature, double end_curvature, double scale) {
  Quintic q;
  q.c[1] = start_tangent * scale;
  q.c[2] = Vector{-start_tangent.y, start_tangent.x} * (.5 * scale * scale * start_curvature);
  const Vector position = end - q.c[1] - q.c[2];
  const Vector velocity = end_tangent * scale - q.c[1] - q.c[2] * 2;
  const Vector second = Vector{-end_tangent.y, end_tangent.x} *
      (scale * scale * end_curvature) - q.c[2] * 2;
  q.c[3] = position * 10 - velocity * 4 + second * .5;
  q.c[4] = position * -15 + velocity * 7 - second;
  q.c[5] = position * 6 - velocity * 3 + second * .5;
  return q;
}

// A geometric proposal screen, not a replacement for delayed PP execution or
// swept footprint validation. Check between reference nodes too: C2 endpoint
// constraints alone do not bound a quintic's interior curvature.
inline bool withinCurvature(const Quintic &q, double limit) {
  for (std::size_t i = 0; i <= 128; ++i) {
    const double u = static_cast<double>(i) / 128;
    const auto tangent = q.tangent(u);
    const double norm = tangent.norm();
    const double k = cross(tangent, q.second(u)) / (norm * norm * norm);
    if (norm < 1e-6 || !std::isfinite(k) || std::abs(k) > limit + 1e-9) return false;
  }
  return true;
}

// At most 16 joining stations x 5 tangent scales. Keep station/source/speed
// labels and the suffix intact; only the entry XY is generated here. Length
// and derivatives are Cartesian, never abs(1 - k_reference * d_ego).
template<class Ego, class Config, class Reference>
inline bool connect(const Ego &ego, const Config &config, double minimum_entry_m,
    Reference *reference, std::size_t *join_index = nullptr) {
  constexpr std::size_t capacity = std::tuple_size<std::decay_t<decltype(reference->points)>>::value;
  if (reference->count < 3 || reference->count > capacity) return false;
  const Vector origin{ego.x_m, ego.y_m};
  const Vector start_tangent{std::cos(ego.yaw_rad), std::sin(ego.yaw_rad)};
  // Same physical-angle initialization as evaluateReference, including a
  // report that is a rounding epsilon outside the configured tire limit.
  const double start_curvature = std::tan(std::clamp(ego.steering_rad,
      -config.maximum_tire_steering_angle_rad, config.maximum_tire_steering_angle_rad)) / config.wheel_base_m;
  const double limit = std::tan(config.maximum_tire_steering_angle_rad) / config.wheel_base_m;
  std::array<double, capacity> arc{};
  for (std::size_t i = 1; i < reference->count; ++i) {
    arc[i] = arc[i - 1] + (xy(reference->points[i]) - xy(reference->points[i - 1])).norm();
    if (!std::isfinite(arc[i])) return false;
  }
  const double available = arc[reference->count - 2];
  const double first = std::min(minimum_entry_m, available);
  const double spacing = std::max(1.0, (available - first) / 15.0);
  std::size_t previous_end = 0;
  for (std::size_t attempt = 0; attempt < 16; ++attempt) {
    const double desired = attempt == 15 ? available : std::min(first + attempt * spacing, available);
    std::size_t end = 1;
    while (end + 2 < reference->count && arc[end] < desired) ++end;
    if (end == previous_end) continue;
    previous_end = end;
    const Vector join = xy(reference->points[end]);
    const Vector before = xy(reference->points[end - 1]), after = xy(reference->points[end + 1]);
    const double left = (join - before).norm(), right = (after - join).norm();
    const double chord = (join - origin).norm();
    const double station_span = reference->points[end].s_m - reference->points[0].s_m;
    if (left < 1e-6 || right < 1e-6 || chord < 1e-6 || station_span < 1e-6) continue;
    auto end_tangent = (join - before) * (right / left) + (after - join) * (left / right);
    if (end_tangent.norm() < 1e-6) continue;
    end_tangent = end_tangent * (1 / end_tangent.norm());
    const double end_curvature = curvature(before, join, after);
    if (std::abs(end_curvature) > limit + 1e-9) continue;
    for (double factor : {1.0, .75, 1.25, 1.5, 2.0}) {
      const auto q = makeQuintic(join - origin, start_tangent, end_tangent,
          start_curvature, end_curvature, chord * factor);
      if (!withinCurvature(q, limit)) continue;
      std::array<Vector, capacity> points{};
      bool feasible = true;
      for (std::size_t i = 0; i <= end + 1; ++i) {
        const double u = (reference->points[i].s_m - reference->points[0].s_m) / station_span;
        points[i] = i < end ? q.position(u) + origin : xy(reference->points[i]);
        if (i >= 2 && std::abs(curvature(points[i - 2], points[i - 1], points[i])) > limit + 1e-6) {
          feasible = false; break;
        }
      }
      if (!feasible) continue;
      for (std::size_t i = 0; i < end; ++i) {
        reference->points[i].x_m = points[i].x;
        reference->points[i].y_m = points[i].y;
      }
      if (join_index != nullptr) *join_index = end;
      return true;
    }
  }
  return false;
}
} // namespace simple_pure_pursuit::reference_connection
