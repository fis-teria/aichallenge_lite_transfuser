#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace reference_space_mppi_planner {
using ReferenceXY = std::array<double, 2>;
inline std::vector<ReferenceXY> smoothReferenceGeometry(
    const std::vector<ReferenceXY> &points, bool closed, double knot_spacing_m) {
  if (!(knot_spacing_m > 0.) || !std::isfinite(knot_spacing_m))
    throw std::invalid_argument("reference geometry knot spacing must be positive");
  if (points.size()<3) return points;
  std::vector<double> arc(points.size());
  for (std::size_t i=1;i<points.size();++i) {
    arc[i]=arc[i-1]+std::hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]);
    if (!std::isfinite(arc[i])) return points;
  }
  const double length=arc.back()+(closed?std::hypot(points.back()[0]-points.front()[0],
                                                  points.back()[1]-points.front()[1]):0.);
  if(length<1e-6) return points;
  const auto sample=[&](double station) {
    if(closed)station-=std::floor(station/length)*length;
    if(closed && station>arc.back()) {
      const double u=(station-arc.back())/(length-arc.back());
      return ReferenceXY{points.back()[0]+u*(points.front()[0]-points.back()[0]),
                         points.back()[1]+u*(points.front()[1]-points.back()[1])};
    }
    auto upper=std::upper_bound(arc.begin(),arc.end(),station);
    std::size_t hi=std::min<std::size_t>(std::max<std::size_t>(1,upper-arc.begin()),points.size()-1);
    std::size_t lo=hi-1;
    // Linear endpoint extension preserves open straight paths, including
    // duplicate endpoint samples, without introducing a clamped tangent.
    while(lo>0 && arc[hi]-arc[lo]<1e-9)--lo;
    while(hi+1<points.size() && arc[hi]-arc[lo]<1e-9)++hi;
    const double u=(station-arc[lo])/std::max(arc[hi]-arc[lo],1e-9);
    return ReferenceXY{points[lo][0]+u*(points[hi][0]-points[lo][0]),
                       points[lo][1]+u*(points[hi][1]-points[lo][1])};
  };
  // Uniform arc stations, rather than input point indices, prevent dense
  // polyline corners from becoming abrupt normals after lateral offsetting.
  // A cubic B-spline gives one C2 geometry for projection and generation.
  const int intervals=std::max(3,static_cast<int>(std::ceil(length/knot_spacing_m)));
  const double step=length/intervals;
  std::vector<ReferenceXY> controls;
  for(int i=-1;i<=intervals+2;++i)controls.push_back(sample(i*step));
  std::vector<ReferenceXY> result;
  result.reserve(points.size());
  for(double station:arc) {
    const int i=std::min(intervals-1,static_cast<int>(station/step));
    const double u=(station-i*step)/step,v=1-u;
    const std::array<double,4> weights{{v*v*v/6.,(3*u*u*u-6*u*u+4)/6.,
                                      (-3*u*u*u+3*u*u+3*u+1)/6.,u*u*u/6.}};
    ReferenceXY p{};
    // Sum relative to the local control point to retain precision in map coordinates.
    const auto origin=controls[i+1];
    for(int j=0;j<4;++j)for(int d=0;d<2;++d)p[d]+=weights[j]*(controls[i+j][d]-origin[d]);
    for(int d=0;d<2;++d)p[d]+=origin[d];
    result.push_back(p);
  }
  return result;
}
} // namespace reference_space_mppi_planner
