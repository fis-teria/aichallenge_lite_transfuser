#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace reference_space_mppi_planner::mppi {

struct SegmentProjection { double s, d, yaw, error; std::size_t index; };

inline SegmentProjection projectOnBase(const BaseReferencePoint *points,
    std::size_t count, double x, double y) {
  SegmentProjection best{0,0,0,std::numeric_limits<double>::infinity(),0};
  double best_t=0.;
  for (std::size_t i=1;i<count;++i) {
    const auto &a=points[i-1]; const auto &b=points[i];
    const double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,l2=dx*dx+dy*dy;
    if(l2<=1e-12) continue;
    const double t=std::clamp(((x-a.x_m)*dx+(y-a.y_m)*dy)/l2,0.,1.);
    const double ex=x-a.x_m-t*dx,ey=y-a.y_m-t*dy,err=ex*ex+ey*ey;
    if(err<best.error) {best.error=err;best.index=i-1;best_t=t;}
  }
  // Only the winning segment needs station, lateral distance and heading.
  // Keep the full scan and strict comparison so ties select the same segment.
  if(std::isfinite(best.error)) {
    const auto &a=points[best.index];const auto &b=points[best.index+1];
    const double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,l2=dx*dx+dy*dy;
    const double ex=x-a.x_m-best_t*dx,ey=y-a.y_m-best_t*dy;
    best.s=a.s_m+best_t*(b.s_m-a.s_m);
    best.d=(dx*ey-dy*ex)/std::sqrt(l2);
    best.yaw=std::atan2(dy,dx);
  }
  return best;
}

// Reuse segment arithmetic and reject distant blocks before evaluating their
// segments. Blocks and segments retain source order, including exact ties.
class BaseProjectionIndex {
 public:
  BaseProjectionIndex(const BaseReferencePoint *points, std::size_t count) {
    if (count < 2) return;
    segments_.reserve(count-1);
    blocks_.reserve((count-1+kBlockSize-1)/kBlockSize);
    for (std::size_t i=1; i<count; ++i) {
      const auto &a=points[i-1]; const auto &b=points[i];
      const double dx=b.x_m-a.x_m, dy=b.y_m-a.y_m;
      segments_.push_back({a.x_m,a.y_m,a.s_m,b.s_m,dx,dy,dx*dx+dy*dy});
      if ((i-1)%kBlockSize==0) blocks_.push_back({});
      auto &block=blocks_.back();
      for (const auto *p : {&a,&b}) {
        if (!std::isfinite(p->x_m) || !std::isfinite(p->y_m) ||
            std::abs(p->x_m)>1e140 || std::abs(p->y_m)>1e140) {
          block.bounded=false;
          continue;
        }
        // Widen bounds for rounding in both projection and residual arithmetic.
        const double pad=32*std::numeric_limits<double>::epsilon()*
            std::max({1.,std::abs(p->x_m),std::abs(p->y_m)});
        block.min_x=std::min(block.min_x,p->x_m-pad);
        block.max_x=std::max(block.max_x,p->x_m+pad);
        block.min_y=std::min(block.min_y,p->y_m-pad);
        block.max_y=std::max(block.max_y,p->y_m+pad);
      }
    }
  }

  SegmentProjection project(double x, double y) const {
    SegmentProjection best{0,0,0,std::numeric_limits<double>::infinity(),0};
    double best_t=0.;
    const bool finite_query=std::isfinite(x) && std::isfinite(y) &&
        std::abs(x)<1e140 && std::abs(y)<1e140;
    for (std::size_t block_index=0; block_index<blocks_.size(); ++block_index) {
      const auto &block=blocks_[block_index];
      if (finite_query && block.bounded) {
        const double ex=std::max({block.min_x-x,0.,x-block.max_x});
        const double ey=std::max({block.min_y-y,0.,y-block.max_y});
        if (ex*ex+ey*ey>best.error) continue;
      }
      const auto end=std::min(segments_.size(),(block_index+1)*kBlockSize);
      for (std::size_t i=block_index*kBlockSize; i<end; ++i) {
        const auto &s=segments_[i];
        if (s.l2<=1e-12) continue;
        const double t=std::clamp(((x-s.x)*s.dx+(y-s.y)*s.dy)/s.l2,0.,1.);
        const double ex=x-s.x-t*s.dx,ey=y-s.y-t*s.dy,err=ex*ex+ey*ey;
        if (err<best.error) {best.error=err;best.index=i;best_t=t;}
      }
    }
    if (std::isfinite(best.error)) {
      const auto &s=segments_[best.index];
      const double ex=x-s.x-best_t*s.dx,ey=y-s.y-best_t*s.dy;
      best.s=s.s+best_t*(s.end_s-s.s);
      best.d=(s.dx*ey-s.dy*ex)/std::sqrt(s.l2);
      best.yaw=std::atan2(s.dy,s.dx);
    }
    return best;
  }

 private:
  static constexpr std::size_t kBlockSize=8;
  struct Segment {double x,y,s,end_s,dx,dy,l2;};
  struct Block {
    double min_x{INFINITY},max_x{-INFINITY},min_y{INFINITY},max_y{-INFINITY};
    bool bounded{true};
  };
  std::vector<Segment> segments_;
  std::vector<Block> blocks_;
};

inline TemporaryReference projectReferenceField(const TemporaryReference &path,
    const PlanRequest &request, const BaseProjectionIndex *prepared=nullptr) {
  std::optional<BaseProjectionIndex> local;
  if (!prepared) {
    local.emplace(request.base_reference.data(),request.base_reference_count);
    prepared=&*local;
  }
  auto field=path;
  for(std::size_t i=0;i<field.count;++i) {
    const auto p=prepared->project(path.points[i].x_m,path.points[i].y_m);
    field.points[i].s_m=p.s; field.points[i].d_m=p.d;
  }
  return field;
}

// A separate ordered coordinate, never Reference station or transport s.
// Both paths start at the measured pose after remaining-prefix consumption.
inline TemporaryReference cartesianArcField(const TemporaryReference &path) {
  auto field=path;
  if(field.count==0 || field.count>field.points.size()) {field.count=0;return field;}
  field.points[0].s_m=0.;
  for(std::size_t i=1;i<field.count;++i)
    field.points[i].s_m=field.points[i-1].s_m+
      std::hypot(path.points[i].x_m-path.points[i-1].x_m,
                 path.points[i].y_m-path.points[i-1].y_m);
  return field;
}

// No endpoint clamp/extrapolation: unmatched stations have no active history.
inline std::optional<TemporaryReferencePoint> sampleReferenceField(
    const TemporaryReference &field,double s) {
  for(std::size_t i=1;i<field.count;++i) {
    const auto &a=field.points[i-1];const auto &b=field.points[i];
    if(b.s_m<=a.s_m+1e-9 || s<a.s_m || s>b.s_m)continue;
    const double u=(s-a.s_m)/(b.s_m-a.s_m);
    auto p=a;p.s_m=s;
    p.x_m=a.x_m+u*(b.x_m-a.x_m);
    p.y_m=a.y_m+u*(b.y_m-a.y_m);
    p.d_m=a.d_m+u*(b.d_m-a.d_m);
    p.speed_mps=a.speed_mps+u*(b.speed_mps-a.speed_mps);
    return p;
  }
  return std::nullopt;
}

inline void setExecutionHistory(PlanRequest &request,const TemporaryReference &remaining) {
  request.execution_history_field=std::make_shared<const TemporaryReference>(
      cartesianArcField(remaining));
  for(std::size_t i=0;i<request.base_reference_count;++i) {
    auto &b=request.base_reference[i];
    // Use the same remaining-distance correspondence as the history cost.
    // Convert that physical point to this base waypoint's normal only for
    // the lateral proposal seed; do not search an arbitrary folded s branch.
    const auto p=sampleReferenceField(*request.execution_history_field,b.s_m-request.ego.s_m);
    b.active_d_valid=b.active_speed_valid=p.has_value();
    b.active_d_m=p?-(p->x_m-b.x_m)*std::sin(b.yaw_rad)+
                    (p->y_m-b.y_m)*std::cos(b.yaw_rad):request.ego.d_m;
    b.active_speed_mps=p?p->speed_mps:0.;
  }
}
} // namespace reference_space_mppi_planner::mppi
