#pragma once

#include "reference_space_mppi_planner/prior_lap_prediction.hpp"

namespace reference_space_mppi_planner::opponent_prediction {

struct CollectionMotion {
  double x_m, y_m, vx_mps, vy_mps, sigma_x_m, sigma_y_m;
};

inline double collectionMedian(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return (values[(n - 1) / 2] + values[n / 2]) * .5;
}

// Causal vector fit for noisy LiDAR surface tracks, at the last source epoch.
// Adjacent displacement magnitudes must NOT be summed: stationary jitter has
// positive travelled distance. Long-baseline median slopes reject isolated
// centroid outliers; projecting the intercept to the latest epoch avoids the
// positional lag of a moving average. No acceleration/yaw extrapolation.
inline std::optional<CollectionMotion> fitCollectionMotion(
    const std::vector<PositionObservation> &history) {
  if (history.empty()) return {};
  const double stamp = history.back().stamp;
  if (!std::isfinite(stamp)) return {};
  std::vector<PositionObservation> points;
  for (const auto &p : history) {
    if (!std::isfinite(p.stamp) || !std::isfinite(p.x) || !std::isfinite(p.y)) return {};
    if (p.stamp < stamp - .6 - 1e-8) continue;
    if (p.stamp > stamp || (!points.empty() && p.stamp <= points.back().stamp)) return {};
    points.push_back(p);
  }
  if (points.empty()) return {};
  // Startup has insufficient evidence of motion. Keep the latest observed
  // position and existing collision covariance; never invent a velocity.
  if (points.size() < 3 || stamp - points.front().stamp < .2 - 1e-8)
    return CollectionMotion{points.back().x, points.back().y, 0., 0., 0., 0.};
  std::vector<double> slopes_x, slopes_y;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      const double dt = points[j].stamp - points[i].stamp;
      if (dt < .15 - 1e-8) continue;
      slopes_x.push_back((points[j].x - points[i].x) / dt);
      slopes_y.push_back((points[j].y - points[i].y) / dt);
    }
  }
  if (slopes_x.empty()) return {};
  double vx = collectionMedian(slopes_x), vy = collectionMedian(slopes_y);
  if (std::hypot(vx, vy) < .2) vx = vy = 0.;
  std::vector<double> xs, ys;
  for (const auto &p : points) {
    xs.push_back(p.x + vx * (stamp - p.stamp));
    ys.push_back(p.y + vy * (stamp - p.stamp));
  }
  const double x = collectionMedian(xs), y = collectionMedian(ys);
  for (auto &value : xs) value = std::abs(value - x);
  for (auto &value : ys) value = std::abs(value - y);
  return CollectionMotion{x, y, vx, vy,
      1.4826 * collectionMedian(xs), 1.4826 * collectionMedian(ys)};
}

} // namespace reference_space_mppi_planner::opponent_prediction
