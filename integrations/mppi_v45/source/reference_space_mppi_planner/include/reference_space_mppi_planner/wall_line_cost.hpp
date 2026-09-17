#pragma once
#include <algorithm>
#include <cmath>
#include "reference_space_mppi_planner/reference_space_mppi.hpp"

namespace reference_space_mppi_planner::mppi {
// Evaluate a fixed Reference-s interval, independent of candidate speed and
// endpoint. Missing coverage pays the full inward penalty, never a free tail.
inline double wallLineCost(const TemporaryReference &path,const PlanRequest &r) {
  if(r.phase!=Phase::OVERTAKE || r.wall_line_end_s_m<=r.wall_line_start_s_m ||
     r.base_reference_count<2 || path.count<2) return 0.;
  double sum=0.,weight_sum=0.;
  constexpr int count=32;
  for(int k=0;k<count;++k) {
    const double u=(k+.5)/count;
    const double s=r.wall_line_start_s_m+u*(r.wall_line_end_s_m-r.wall_line_start_s_m);
    const double weight=std::pow(std::sin(3.141592653589793*u),2);
    weight_sum+=weight;
    if(s<path.points[0].s_m || s>path.points[path.count-1].s_m) {sum+=weight;continue;}
    std::size_t j=1,b=1;
    while(j+1<path.count && path.points[j].s_m<s)++j;
    while(b+1<r.base_reference_count && r.base_reference[b].s_m<s)++b;
    const auto &p=path.points[j-1],&q=path.points[j];
    const auto &a=r.base_reference[b-1],&z=r.base_reference[b];
    const double t=std::clamp((s-p.s_m)/std::max(1e-9,q.s_m-p.s_m),0.,1.);
    const double v=std::clamp((s-a.s_m)/std::max(1e-9,z.s_m-a.s_m),0.,1.);
    const double d=p.d_m+t*(q.d_m-p.d_m);
    const double origin=a.pass_origin_d_valid && z.pass_origin_d_valid ?
        a.pass_origin_d_m+v*(z.pass_origin_d_m-a.pass_origin_d_m):r.pass_profile_origin_d_m;
    const bool left=d>=origin;
    const double wall=left ? a.maximum_d_m+v*(z.maximum_d_m-a.maximum_d_m):
        a.minimum_d_m+v*(z.minimum_d_m-a.minimum_d_m);
    const double room=left?wall-origin:origin-wall;
    const double inward=left?wall-d:d-wall;
    const double error=room>1e-9?std::clamp(inward/room,0.,1.):1.;
    sum+=weight*error*error;
  }
  return sum/weight_sum;
}
} // namespace reference_space_mppi_planner::mppi
