#pragma once

#include "reference_space_mppi_planner/prior_lap_prediction.hpp"

namespace reference_space_mppi_planner::opponent_prediction {

// A complete rolling lap, ending at the last received observation. Interpolate
// in source time, not distance at the current speed: slow sections recur at
// their observed places. No observations beyond the source epoch are needed.
inline std::shared_ptr<const Prediction> buildLeaderLapPrediction(
    const std::vector<PositionObservation> &history, double length,
    double start_stamp, double horizon, const ReferencePoseIndex &world) {
  if (history.size() < 3U || !(length > 0.) || !(horizon > 0.)) return {};
  const auto &last = history.back();
  const double anchor = last.station - length;
  auto line = selectContinuousPriorPass(history, anchor, last.station);
  if (line.size() < 3U) return {};
  auto first = std::lower_bound(line.begin(), line.end(), anchor,
      [](const auto &p, double s) { return p.station < s; });
  if (first == line.begin() && first->station > anchor + 1e-6) return {};
  if (first == line.end()) return {};
  const auto sampleStation = [&](double station) {
    auto it = std::lower_bound(line.begin(), line.end(), station,
        [](const auto &p, double s) { return p.station < s; });
    if (it == line.begin()) return *it;
    if (it == line.end()) return line.back();
    const auto &a = *(it-1), &b = *it;
    const double u = (station-a.station)/(b.station-a.station);
    return PositionObservation{a.stamp+u*(b.stamp-a.stamp),
        a.x+u*(b.x-a.x), a.y+u*(b.y-a.y), station};
  };
  const auto origin = sampleStation(anchor);
  const double period = last.stamp-origin.stamp;
  if (!(period > 1.) || start_stamp < last.stamp-0.5) return {};
  // A dropout cannot silently become a well-observed traversal interval.
  for (auto it = first; it != line.end(); ++it) {
    if (it != line.begin() &&
        (!(it->stamp > (it-1)->stamp) || it->stamp-(it-1)->stamp > 1.)) return {};
  }
  const auto atTime = [&](double elapsed) {
    const double cycles = std::floor(elapsed/period);
    const double stamp = origin.stamp + elapsed-cycles*period;
    auto it = std::lower_bound(line.begin(), line.end(), stamp,
        [](const auto &p, double t) { return p.stamp < t; });
    if (it == line.begin()) ++it;
    if (it == line.end()) --it;
    const auto &a = *(it-1), &b = *it;
    const double u = (stamp-a.stamp)/(b.stamp-a.stamp);
    return PositionObservation{stamp, a.x+u*(b.x-a.x), a.y+u*(b.y-a.y),
        a.station+u*(b.station-a.station)+(cycles+1.)*length};
  };
  auto result = std::make_shared<Prediction>();
  result->source_stamp = last.stamp;
  const double offset_x = last.x-origin.x, offset_y = last.y-origin.y;
  std::array<double,4> recent{};
  if (!fitRecent(history,recent)) return {};
  const auto next = atTime(.025);
  const double correction_x = recent[2]-(next.x-origin.x)/.025+offset_x/.6;
  const double correction_y = recent[3]-(next.y-origin.y)/.025+offset_y/.6;
  const auto point = [&](double elapsed) {
    if (elapsed<0.) {
      auto it=std::lower_bound(line.begin(),line.end(),last.stamp+elapsed,
          [](const auto &p,double t){return p.stamp<t;});
      if (it==line.begin()) ++it;
      if (it==line.end()) --it;
      const auto &a=*(it-1),&b=*it;
      const double u=(last.stamp+elapsed-a.stamp)/(b.stamp-a.stamp);
      return std::array<double,2>{a.x+u*(b.x-a.x),a.y+u*(b.y-a.y)};
    }
    const auto p = atTime(elapsed);
    const double blend = std::exp(-std::max(0.,elapsed)/.6);
    return std::array<double,2>{p.x+blend*(offset_x+correction_x*elapsed),
        p.y+blend*(offset_y+correction_y*elapsed)};
  };
  const auto steps = static_cast<std::size_t>(std::ceil(horizon/.05));
  for (std::size_t i=0; i<=steps; ++i) {
    const double t = horizon*static_cast<double>(i)/steps;
    const double elapsed = start_stamp-last.stamp+t;
    const auto xy = point(elapsed), before = point(elapsed), after = point(elapsed+.025);
    const auto s = world.projectStation(xy[0],xy[1]);
    if (!s) return {};
    const auto pose = world.pose(*s,0.);
    if (!pose) return {};
    const double yaw = std::atan2(after[1]-before[1],after[0]-before[0]);
    const double d = -(xy[0]-(*pose)[0])*std::sin((*pose)[2])+
        (xy[1]-(*pose)[1])*std::cos((*pose)[2]);
    const double global_s = result->points.empty() ? *s :
        result->points.back().global_s + world.stationDifference(*s,result->points.back().global_s);
    result->points.push_back({t,global_s,d,std::remainder(yaw-(*pose)[2],2.*std::acos(-1.))});
    if (i == 0U) {
      result->vx = (after[0]-before[0])/.025;
      result->vy = (after[1]-before[1])/.025;
    }
  }
  return result;
}
} // namespace reference_space_mppi_planner::opponent_prediction
