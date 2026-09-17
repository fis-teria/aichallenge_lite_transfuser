#pragma once

#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include "reference_space_mppi_planner/prior_lap_route.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace reference_space_mppi_planner::opponent_prediction {

struct PositionObservation {
  double stamp, x, y, station;
};

struct PredictionPoint {
  double time, global_s, d, relative_yaw;
};

struct Prediction {
  std::vector<PredictionPoint> points;
  double source_stamp{0.0};
  double vx{0.0}, vy{0.0};

  PredictionPoint at(double time) const {
    const double t = std::clamp(time, points.front().time, points.back().time);
    auto it = std::lower_bound(points.begin(), points.end(), t,
        [](const auto &p, double value) { return p.time < value; });
    if (it == points.begin()) return *it;
    if (it == points.end()) return points.back();
    const auto &a = *(it - 1), &b = *it;
    const double u = (t - a.time) / (b.time - a.time);
    return {t, a.global_s + u * (b.global_s - a.global_s),
            a.d + u * (b.d - a.d),
            a.relative_yaw + u * std::remainder(b.relative_yaw - a.relative_yaw,
                                               2.0 * std::acos(-1.0))};
  }
};

struct ObstaclePosition {
  double s, global_s, d, relative_yaw;
};

template <class Obstacle>
ObstaclePosition positionAt(const Obstacle &obstacle, double time) {
  const double t = std::max(0.0, time);
  if (obstacle.prediction) {
    const auto p = obstacle.prediction->at(t);
    return {obstacle.s_m + p.global_s - obstacle.prediction->points.front().global_s,
            p.global_s, p.d, p.relative_yaw};
  }
  return {obstacle.s_m + obstacle.longitudinal_speed_mps * t,
          obstacle.global_reference_s_m + obstacle.longitudinal_speed_mps * t,
          obstacle.d_m + obstacle.lateral_speed_mps * t,
          obstacle.heading_relative_to_reference_rad};
}

// Fit position and velocity at the last received source epoch. No opponent
// odometry or samples received after the input snapshot enter this model.
inline bool fitRecent(const std::vector<PositionObservation> &history,
                      std::array<double, 4> &state,
                      std::array<double, 2> *acceleration = nullptr) {
  if (history.size() < 3U) return false;
  const auto &last = history.back();
  double a[3][5]{};
  std::size_t count = 0;
  double previous = -std::numeric_limits<double>::infinity();
  for (const auto &p : history) {
    if (p.stamp < last.stamp - .6 - 1e-8 || p.stamp <= previous) continue;
    previous = p.stamp;
    const double t = p.stamp - last.stamp;
    const double v[3]{1., t, .5*t*t};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) a[i][j] += v[i]*v[j];
      a[i][3] += v[i]*(p.x-last.x);
      a[i][4] += v[i]*(p.y-last.y);
    }
    ++count;
  }
  if (count < 3U) return false;
  for (int k = 0; k < 3; ++k) {
    int pivot = k;
    for (int i = k+1; i < 3; ++i)
      if (std::abs(a[i][k]) > std::abs(a[pivot][k])) pivot = i;
    if (std::abs(a[pivot][k]) < 1e-12) return false;
    for (int j = k; j < 5; ++j) std::swap(a[k][j], a[pivot][j]);
    const double divisor = a[k][k];
    for (int j = k; j < 5; ++j) a[k][j] /= divisor;
    for (int i = 0; i < 3; ++i) if (i != k) {
      const double factor = a[i][k];
      for (int j = k; j < 5; ++j) a[i][j] -= factor*a[k][j];
    }
  }
  state = {last.x+a[0][3], last.y+a[0][4], a[1][3], a[1][4]};
  if (acceleration) *acceleration = {a[2][3], a[2][4]};
  return std::all_of(state.begin(), state.end(), [](double v) { return std::isfinite(v); });
}

// One immutable forecast per input snapshot, shared by all candidates. The
// previously observed line belongs to this opponent, with unwrapped Reference
// stations; motion advances by its physical XY arc length. Missing coverage
// returns null so every consumer retains the existing constant-speed model.
inline std::shared_ptr<const Prediction> buildPriorLapPrediction(
    const std::vector<PositionObservation> &history, double length,
    double start_stamp, double horizon, const ReferencePoseIndex &world) {
  std::array<double, 4> fitted{};
  if (!(length > 0.0) || !(horizon > 0.0) || !fitRecent(history, fitted)) return {};
  const auto &last = history.back();
  const double speed = std::hypot(fitted[2], fitted[3]);
  const double anchor = last.station - length;
  const double extent = speed * std::max(std::abs(start_stamp-last.stamp),
                                       std::abs(start_stamp+horizon-last.stamp)) + 1.;
  auto line = selectContinuousPriorPass(history, anchor-extent, anchor+extent);
  // Clip only after selecting a continuous pass. Filtering the history first
  // could hide an excursion and reconnect observations on either side of it.
  line.erase(std::remove_if(line.begin(), line.end(), [&](const auto &p) {
    return p.station >= last.station - length*.5;
  }), line.end());
  if (line.size() < 3U || line.front().station > anchor-extent ||
      line.back().station < anchor+extent) return {};
  const auto xyAt = [&](double station) {
    auto it = std::lower_bound(line.begin(), line.end(), station,
        [](const auto &p, double s) { return p.station < s; });
    if (it == line.begin()) return std::array<double,2>{it->x, it->y};
    if (it == line.end()) return std::array<double,2>{line.back().x, line.back().y};
    const auto &a = *(it-1), &b = *it;
    const double u = (station-a.station)/(b.station-a.station);
    return std::array<double,2>{a.x+u*(b.x-a.x), a.y+u*(b.y-a.y)};
  };
  const auto yawAt = [&](double station) {
    const auto a = xyAt(station-.5), b = xyAt(station+.5);
    return std::atan2(b[1]-a[1], b[0]-a[0]);
  };
  std::vector<double> arc(line.size(), 0.0);
  for (std::size_t i = 1; i < line.size(); ++i)
    arc[i] = arc[i-1] + std::hypot(line[i].x-line[i-1].x, line[i].y-line[i-1].y);
  const auto anchor_it = std::lower_bound(line.begin(), line.end(), anchor,
      [](const auto &p, double s) { return p.station < s; });
  const auto k = static_cast<std::size_t>(anchor_it-line.begin());
  const double origin = arc[k-1] + (anchor-line[k-1].station) /
      (line[k].station-line[k-1].station)*(arc[k]-arc[k-1]);
  const auto center = xyAt(anchor);
  const double theta = yawAt(anchor), beta = std::atan2(fitted[3],fitted[2])-theta;
  const double along = (fitted[0]-center[0])*std::cos(theta) + (fitted[1]-center[1])*std::sin(theta);
  const double lateral = -(fitted[0]-center[0])*std::sin(theta) + (fitted[1]-center[1])*std::cos(theta);
  auto result = std::make_shared<Prediction>();
  result->source_stamp = last.stamp;
  result->vx = fitted[2]; result->vy = fitted[3];
  const auto steps = static_cast<std::size_t>(std::ceil(horizon/.0125));
  result->points.reserve(steps+1);
  for (std::size_t i = 0; i <= steps; ++i) {
    const double t = horizon*static_cast<double>(i)/steps;
    const double dt = start_stamp+t-last.stamp;
    const double distance = origin+speed*std::cos(beta)*dt;
    if (distance < arc.front() || distance > arc.back()) return {};
    const auto found = std::lower_bound(arc.begin(), arc.end(), distance);
    const auto j = std::max<std::size_t>(1, found-arc.begin());
    if (!(arc[j] > arc[j-1])) return {};
    const double u = (distance-arc[j-1])/(arc[j]-arc[j-1]);
    const double station = line[j-1].station+u*(line[j].station-line[j-1].station);
    const auto xy = xyAt(station);
    const double yaw = yawAt(station), decay = std::exp(-std::max(0.,dt)/.6);
    const double duration = dt > 0 ? -.6*std::expm1(-dt/.6) : dt;
    const double offset = lateral+speed*std::sin(beta)*duration;
    const double x = xy[0]+along*std::cos(yaw)-offset*std::sin(yaw);
    const double y = xy[1]+along*std::sin(yaw)+offset*std::cos(yaw);
    const auto s = world.projectStation(x,y);
    if (!s) return {};
    const auto pose = world.pose(*s,0.);
    if (!pose) return {};
    const double d = -(x-(*pose)[0])*std::sin((*pose)[2])+(y-(*pose)[1])*std::cos((*pose)[2]);
    const double global_s = result->points.empty() ? *s : result->points.back().global_s +
        world.stationDifference(*s,result->points.back().global_s);
    result->points.push_back({t,global_s,d,std::remainder(
        yaw+std::atan2(std::sin(beta)*decay,std::cos(beta))-(*pose)[2],2.*std::acos(-1.))});
  }
  return result;
}

} // namespace reference_space_mppi_planner::opponent_prediction
