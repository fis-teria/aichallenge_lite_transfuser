#pragma once
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace reference_space_mppi_planner::mppi {
// CMA estimateCurvature's three-point stencil, followed by its forward
// distance-window maximum. Cached once per candidate, not per rollout step.
inline std::array<double, kMaximumReferencePoints> cmaPreviewCurvatures(
    const TemporaryReference &path, double window_m) {
  std::array<double, kMaximumReferencePoints> curvature{}, arc{}, result{};
  if(path.count<3 || path.count>kMaximumReferencePoints) return result;
  for(std::size_t i=0;i<path.count;++i) {
    const auto &a=path.points[i>1?i-1:0];
    const auto &c=path.points[std::min(path.count-1,i+4)];
    const auto &b=path.points[((i>1?i-1:0)+std::min(path.count-1,i+4))/2];
    const double denominator=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m)*
        std::hypot(c.x_m-b.x_m,c.y_m-b.y_m)*std::hypot(a.x_m-c.x_m,a.y_m-c.y_m);
    if(denominator>1e-6)
      curvature[i]=2*std::abs((b.x_m-a.x_m)*(c.y_m-a.y_m)-
                            (b.y_m-a.y_m)*(c.x_m-a.x_m))/denominator;
    if(i) arc[i]=arc[i-1]+std::hypot(path.points[i].x_m-path.points[i-1].x_m,
                                    path.points[i].y_m-path.points[i-1].y_m);
  }
  std::array<std::size_t,kMaximumReferencePoints> queue{};
  std::size_t head=0,tail=0,end=0;
  for(std::size_t i=0;i<path.count;++i) {
    while(head<tail && queue[head]<i) ++head;
    while(end<path.count && (end==i || arc[end]-arc[i]<=window_m)) {
      while(head<tail && curvature[queue[tail-1]]<=curvature[end]) --tail;
      queue[tail++]=end++;
    }
    result[i]=curvature[queue[head]];
  }
  return result;
}
} // namespace reference_space_mppi_planner::mppi
