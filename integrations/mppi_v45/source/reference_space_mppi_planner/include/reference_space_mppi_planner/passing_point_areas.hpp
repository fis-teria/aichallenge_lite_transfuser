#pragma once

#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace reference_space_mppi_planner {

struct PassingPointAreaConfig {
  double corner_margin_m{5.};
  std::vector<double> straight_start_xy;
  std::vector<double> corner_entry_xy;
};

struct PassingPointAreas {
  using Interval = std::pair<double, double>;
  std::vector<Interval> allowed, forbidden;
  std::vector<double> deadlines;
  double length_m{0.};
  bool closed{false};

  bool contains(double station) const {
    if (!std::isfinite(station)) return false;
    if (closed && length_m>0.) {station=std::fmod(station,length_m); if(station<0.) station+=length_m;}
    return std::any_of(allowed.begin(),allowed.end(),[&](const auto &interval) {
      return station>=interval.first && station<=interval.second;
    });
  }
};

// Fixed map anchors survive reference resampling and a shifted lap seam.
// The margin is measured along the current reference after projection.
inline std::optional<PassingPointAreas> passingPointAreas(
    const ReferencePoseIndex &world, double length, bool closed,
    const PassingPointAreaConfig &config) {
  if (!(length>0.) || !std::isfinite(length) ||
      !std::isfinite(config.corner_margin_m) || config.corner_margin_m<0. ||
      config.straight_start_xy.size()%2 ||
      config.straight_start_xy.size()!=config.corner_entry_xy.size())
    return {};
  PassingPointAreas result;
  result.length_m=length;result.closed=closed;
  for (std::size_t i=0; i<config.straight_start_xy.size(); i+=2) {
    const auto start_s=world.projectStation(config.straight_start_xy[i],config.straight_start_xy[i+1]);
    const auto end_s=world.projectStation(config.corner_entry_xy[i],config.corner_entry_xy[i+1]);
    if (!start_s || !end_s || *start_s==*end_s) return {};
    const double start=*start_s;
    double end=*end_s;
    if (end<=start) {
      if (!closed) return {};
      end+=length;
    }
    const double deadline=end-config.corner_margin_m;
    result.deadlines.push_back(deadline);
    if (deadline<=start) continue;
    if (deadline>length) {
      result.allowed.emplace_back(start,length);
      result.allowed.emplace_back(0.,deadline-length);
    } else result.allowed.emplace_back(start,deadline);
  }
  std::sort(result.allowed.begin(),result.allowed.end());
  std::vector<PassingPointAreas::Interval> merged;
  for (const auto &interval:result.allowed) {
    if (!merged.empty() && interval.first<=merged.back().second)
      merged.back().second=std::max(merged.back().second,interval.second);
    else merged.push_back(interval);
  }
  result.allowed=std::move(merged);
  double cursor=0.;
  for (const auto &interval:result.allowed) {
    if (interval.first>cursor) result.forbidden.emplace_back(cursor,interval.first);
    cursor=interval.second;
  }
  if (cursor<length) result.forbidden.emplace_back(cursor,length);
  return result;
}

}  // namespace reference_space_mppi_planner
