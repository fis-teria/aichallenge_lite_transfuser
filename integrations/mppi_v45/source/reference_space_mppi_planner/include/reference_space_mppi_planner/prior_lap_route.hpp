#pragma once

#include <cmath>
#include <vector>

namespace reference_space_mppi_planner::opponent_prediction {

// Keep observation order: sorting or averaging by station can join different
// visits to the same part of the course. A usable pass must cover the requested
// station interval without a reversal or sideways motion at fixed station.
// If more than one pass covers it, use the most recently observed complete pass.
template <class Observation>
std::vector<Observation> selectContinuousPriorPass(
    const std::vector<Observation> &history, double minimum_station,
    double maximum_station) {
  std::vector<Observation> pass, selected;
  const auto finish = [&]() {
    if (pass.size() >= 3U && pass.front().station <= minimum_station &&
        pass.back().station >= maximum_station)
      selected = pass;
  };
  for (const auto &point : history) {
    if (!pass.empty()) {
      const auto &previous = pass.back();
      if (std::hypot(point.x - previous.x, point.y - previous.y) <= 1e-6)
        continue;
      if (point.station <= previous.station + 1e-6) {
        finish();
        pass.clear();
      }
    }
    pass.push_back(point);
  }
  finish();
  return selected;
}

} // namespace reference_space_mppi_planner::opponent_prediction
