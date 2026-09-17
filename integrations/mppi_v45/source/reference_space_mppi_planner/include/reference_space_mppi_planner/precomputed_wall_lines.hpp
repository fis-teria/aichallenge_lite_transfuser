#pragma once

#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include <cstdint>

namespace reference_space_mppi_planner {

// Course-station labels are shared with preparation. XY belongs to this
// immutable library, so cropping at a different ego pose cannot reshape it.
struct PrecomputedWallLines {
  using XY = std::array<double, 2>;
  struct Sample {
    double station_m{0.};
    std::array<XY, 2> xy{};
  };
  std::shared_ptr<const ReferencePoseIndex> world;
  std::uint64_t generation{0};
  double length_m{0.};
  bool closed{false};
  std::vector<Sample> samples;

  std::optional<XY> at(double station, int side) const {
    if (samples.size()<2 || !std::isfinite(station) || (side!=1 && side!=-1)) return {};
    if (closed && length_m>0.) {
      station=std::fmod(station,length_m);
      if(station<0.) station+=length_m;
    } else if(station<0. || station>length_m+1e-6) return {};
    const auto upper=std::upper_bound(samples.begin(),samples.end(),station,
        [](double s,const Sample &p){return s<p.station_m;});
    const std::size_t index=side>0 ? 0 : 1;
    if(upper==samples.end()) return samples.back().xy[index];
    if(upper==samples.begin()) return upper->xy[index];
    const auto &a=*std::prev(upper), &b=*upper;
    const double u=(station-a.station_m)/(b.station_m-a.station_m);
    return XY{a.xy[index][0]+u*(b.xy[index][0]-a.xy[index][0]),
                       a.xy[index][1]+u*(b.xy[index][1]-a.xy[index][1])};
  }
};

} // namespace reference_space_mppi_planner
