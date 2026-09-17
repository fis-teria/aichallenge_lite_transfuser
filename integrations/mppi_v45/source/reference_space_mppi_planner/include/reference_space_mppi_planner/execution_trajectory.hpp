#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/reference_field.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

namespace reference_space_mppi_planner::mppi {

// Fade one translation, not a different normal at every polyline vertex.
// Rotating a large lateral offset can reverse an otherwise forward segment.
inline std::array<double, 2> executionConnectionTranslation(
    double dx, double dy, double arc_m, double connector_m) {
  const double u = std::clamp(arc_m/connector_m, 0., 1.);
  const double fade = 1-u*u*u*(10+u*(-15+6*u));
  return {dx*fade, dy*fade};
}

struct TrajectoryProjection {
  std::size_t lower{0};
  double ratio{0.0};
  double distance_squared{std::numeric_limits<double>::infinity()};
};

inline TrajectoryProjection projectExecutionTrajectory(
    const TemporaryReference &r, double x, double y) {
  TrajectoryProjection out;
  if (r.count > r.points.size()) return out;
  for (std::size_t i = 1; i < r.count; ++i) {
    const auto &a = r.points[i-1]; const auto &b = r.points[i];
    const double dx = b.x_m-a.x_m, dy = b.y_m-a.y_m, l2 = dx*dx+dy*dy;
    if (l2 < 1e-12) continue;
    const double u = std::clamp(((x-a.x_m)*dx+(y-a.y_m)*dy)/l2, 0., 1.);
    const double ex = x-a.x_m-u*dx, ey = y-a.y_m-u*dy;
    if (ex*ex+ey*ey < out.distance_squared) out = {i-1, u, ex*ex+ey*ey};
  }
  return out;
}

inline void updateExecutionGeometry(TemporaryReference &r) {
  for (std::size_t i = 1; i < r.count; ++i) {
    const auto &a = r.points[i-1];
    const auto &c = r.points[std::min(i+1, r.count-1)];
    r.points[i].yaw_rad = std::atan2(c.y_m-a.y_m, c.x_m-a.x_m);
  }
  for (std::size_t i = 1; i+1 < r.count; ++i) {
    const auto &a = r.points[i-1]; auto &b = r.points[i]; const auto &c = r.points[i+1];
    const double denom = std::hypot(b.x_m-a.x_m,b.y_m-a.y_m) *
        std::hypot(c.x_m-b.x_m,c.y_m-b.y_m) * std::hypot(c.x_m-a.x_m,c.y_m-a.y_m);
    b.curvature_1pm = denom > 1e-9 ?
        2*((b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m))/denom : 0.;
  }
  if (r.count >= 3) {
    r.points[0].curvature_1pm = r.points[1].curvature_1pm;
    r.points[r.count-1].curvature_1pm = r.points[r.count-2].curvature_1pm;
  }
}

// Always refresh the accepted source, never the previously refreshed output.
// Retained points keep their absolute speeds: consuming a prefix must consume
// the matching-speed section, not move its acceleration point forward again.
inline std::optional<TemporaryReference> remainingExecutionTrajectory(
    const TemporaryReference &active, const EgoState &ego, double maximum_arc_m,
    const TemporaryReference *speed_profile=nullptr) {
  if (active.count < 3 || active.count > active.points.size() ||
      !std::isfinite(maximum_arc_m) || maximum_arc_m <= 0 ||
      !std::isfinite(ego.x_m) || !std::isfinite(ego.y_m) || !std::isfinite(ego.yaw_rad))
    return std::nullopt;
  const auto projection = projectExecutionTrajectory(active, ego.x_m, ego.y_m);
  if (!std::isfinite(projection.distance_squared)) return std::nullopt;
  const auto &a = active.points[projection.lower];
  const auto &b = active.points[projection.lower+1];
  const double px = a.x_m+projection.ratio*(b.x_m-a.x_m);
  const double py = a.y_m+projection.ratio*(b.y_m-a.y_m);
  TemporaryReference result;
  result.overtake_start_source_s_m=speed_profile ?
      speed_profile->overtake_start_source_s_m : active.overtake_start_source_s_m;
  auto anchor = a;
  anchor.x_m = ego.x_m; anchor.y_m = ego.y_m; anchor.yaw_rad = ego.yaw_rad;
  anchor.s_m = 0.;
  anchor.source_s_m=a.s_m+projection.ratio*(b.s_m-a.s_m);
  anchor.speed_mps = a.speed_mps+projection.ratio*(b.speed_mps-a.speed_mps);
  if(speed_profile) {
    const auto speed=sampleReferenceField(*speed_profile,anchor.source_s_m);
    if(speed)anchor.speed_mps=speed->speed_mps;
  }
  anchor.uncapped_speed_mps = anchor.speed_mps;
  result.points[result.count++] = anchor;
  const double connector = std::clamp(5*std::sqrt(projection.distance_squared), 4., 12.);
  double previous_x = px, previous_y = py, original_arc = 0.;
  for (std::size_t i = projection.lower+1; i < active.count; ++i) {
    auto point = active.points[i];
    point.source_s_m=point.s_m;
    if(speed_profile) {
      const auto speed=sampleReferenceField(*speed_profile,point.source_s_m);
      if(speed)point.speed_mps=point.uncapped_speed_mps=speed->speed_mps;
    }
    const double ds = std::hypot(point.x_m-previous_x, point.y_m-previous_y);
    if (!std::isfinite(ds)) return std::nullopt;
    if (ds < 1e-3) continue;
    original_arc += ds; previous_x = point.x_m; previous_y = point.y_m;
    const auto translation = executionConnectionTranslation(
        ego.x_m-px, ego.y_m-py, original_arc, connector);
    point.x_m += translation[0]; point.y_m += translation[1];
    const auto &last = result.points[result.count-1];
    const double arc = std::hypot(point.x_m-last.x_m, point.y_m-last.y_m);
    if (arc < 1e-3) continue;
    point.s_m = last.s_m+arc;
    const bool trim = point.s_m >= maximum_arc_m;
    if (trim) {
      const double ratio = (maximum_arc_m-last.s_m)/arc;
      point.x_m = last.x_m+ratio*(point.x_m-last.x_m);
      point.y_m = last.y_m+ratio*(point.y_m-last.y_m);
      point.speed_mps = last.speed_mps+ratio*(point.speed_mps-last.speed_mps);
      point.source_s_m=last.source_s_m+ratio*(point.source_s_m-last.source_s_m);
      point.uncapped_speed_mps = point.speed_mps;
      point.s_m = maximum_arc_m;
    }
    if (point.s_m <= last.s_m+1e-9) break;
    result.points[result.count++] = point;
    if (trim || result.count == result.points.size()) break;
  }
  if (result.count < 3) return std::nullopt; // No invented tail/constant-speed extension.
  updateExecutionGeometry(result);
  return result;
}

// Ordered physical-distance regularization. Reference projection can fold;
// it must not supply an integration measure or a branch correspondence.
// Lateral change is measured on the matched history segment's normal, not
// by subtracting d values expressed on unrelated Reference segments.
inline std::array<double, 2> executionFieldCosts(
    const TemporaryReference &r, const PlanRequest &request, const Config &config,
    Evaluation *diagnostic = nullptr) {
  if(r.count<2 || r.count>r.points.size() || request.base_reference_count<2 ||
      request.base_reference_count>request.base_reference.size()) return {};
  const auto path=cartesianArcField(r);
  if (diagnostic) diagnostic->field_cost_split.fill(0.);
  double smooth = 0., arc = 0., change = 0., covered = 0.;
  const double lateral_scale = std::max(1e-3, config.control_lateral_std_m);
  const double speed_range = std::max(1e-3,
      request.bounds.maximum.speed_scale-request.bounds.minimum.speed_scale);
  const double spacing = config.control_lateral_reference_spacing_m > 0. ?
      config.control_lateral_reference_spacing_m :
      (request.base_reference[request.base_reference_count-1].s_m-request.base_reference[0].s_m)/
          std::max<std::size_t>(1,config.control_knot_count-1);
  std::size_t first_base = 1, first_history = 1;
  for (std::size_t j = 1; j < path.count; ++j) {
    const auto &p = path.points[j-1]; const auto &q = path.points[j];
    const double ds = q.s_m-p.s_m;
    if (ds <= 1e-9) continue;
    // Integrate over the intersection of each piecewise-linear field segment.
    while (first_base+1 < request.base_reference_count &&
           request.base_reference[first_base].s_m-request.ego.s_m <= p.s_m) ++first_base;
    for (std::size_t i = first_base; i < request.base_reference_count &&
         request.base_reference[i-1].s_m-request.ego.s_m < q.s_m; ++i) {
      const auto &a = request.base_reference[i-1]; const auto &b = request.base_reference[i];
      const double base_start=a.s_m-request.ego.s_m, base_end=b.s_m-request.ego.s_m;
      const double lo = std::max(p.s_m,base_start), hi = std::min(q.s_m,base_end);
      if (hi <= lo || b.s_m <= a.s_m) continue;
      const auto penalty = [&](double s, double lateral, double active_speed) {
        const double u = (s-p.s_m)/ds, v = (s-base_start)/(base_end-base_start);
        const double speed_scale = std::max(1e-3,(a.speed_mps+v*(b.speed_mps-a.speed_mps))*speed_range);
        const double speed = (p.speed_mps+u*(q.speed_mps-p.speed_mps)-active_speed)/speed_scale;
        return std::array<double,2>{config.history_lateral_cost_scale *
            (lateral*lateral/(lateral_scale*lateral_scale)),
            config.longitudinal_planning_enabled ? 0. : speed*speed};
      };
      const auto integrate = [&](double low,double high,const auto &at) {
        const auto left=at(low),middle=at(.5*(low+high)),right=at(high);
        // Preserve the original Simpson sum and normalizer. Classify its
        // quadrature contributions without changing the objective's sampling.
        change+=(high-low)*((left[0]+left[1])+4*(middle[0]+middle[1])+
                            (right[0]+right[1]))/6;
        if (diagnostic) {
          const auto attribute = [&](double s, const auto &value, double weight) {
            const std::size_t outside=s>diagnostic->field_terminal_arc_m ? 1U : 0U;
            for(std::size_t term=0;term<2;++term)
              diagnostic->field_cost_split[2*term+outside]+=(high-low)*weight*value[term]/6;
          };
          attribute(low,left,1.);attribute(.5*(low+high),middle,4.);attribute(high,right,1.);
        }
        covered+=high-low;
      };
      if(request.execution_history_field) {
        const auto &history=*request.execution_history_field;
        while(first_history+1<history.count && history.points[first_history].s_m<=lo)
          ++first_history;
        // Split once on both ordered paths and the speed normalizer's knots.
        // No all-pairs comparison and no minimum-penalty branch selection.
        for(std::size_t h=first_history;h<history.count && history.points[h-1].s_m<hi;++h) {
          const auto &ha=history.points[h-1];const auto &hb=history.points[h];
          if(hb.s_m<=ha.s_m+1e-9)continue;
          const double low=std::max(lo,ha.s_m),high=std::min(hi,hb.s_m);
          const double length=std::hypot(hb.x_m-ha.x_m,hb.y_m-ha.y_m);
          if(high<=low || length<=1e-9)continue;
          integrate(low,high,[&](double s) {
            const double u=(s-p.s_m)/ds,v=(s-ha.s_m)/(hb.s_m-ha.s_m);
            const double ex=p.x_m+u*(q.x_m-p.x_m)-ha.x_m-v*(hb.x_m-ha.x_m);
            const double ey=p.y_m+u*(q.y_m-p.y_m)-ha.y_m-v*(hb.y_m-ha.y_m);
            const double lateral=(-(hb.y_m-ha.y_m)*ex+(hb.x_m-ha.x_m)*ey)/length;
            return penalty(s,lateral,ha.speed_mps+v*(hb.speed_mps-ha.speed_mps));
          });
        }
      } else if (a.active_d_valid && b.active_d_valid && a.active_speed_valid && b.active_speed_valid) {
        // Legacy request without an accepted Cartesian path: its active seed
        // is attached to the ordered base waypoints, not a projected history.
        integrate(lo,hi,[&](double s) {
          const double u=(s-p.s_m)/ds,v=(s-base_start)/(base_end-base_start);
          const double lateral=p.d_m+u*(q.d_m-p.d_m)-a.active_d_m-v*(b.active_d_m-a.active_d_m);
          return penalty(s,lateral,a.active_speed_mps+v*(b.active_speed_mps-a.active_speed_mps));
        });
      }
      const double v = (.5*(hi+lo)-base_start)/(base_end-base_start);
      const double speed_scale = std::max(1e-3,(a.speed_mps+v*(b.speed_mps-a.speed_mps))*speed_range);
      const double speed_gradient = config.longitudinal_planning_enabled ? 0. :
          (q.speed_mps-p.speed_mps)/ds*spacing/speed_scale;
      // Geometric smoothness is already scored from physical curvature in
      // term 3. Penalizing d' here as well charges an intentional lane change
      // twice and makes a supplied trajectory depend on its proposal shape.
      smooth += (hi-lo)*speed_gradient*speed_gradient;
      if (diagnostic) {
        const double inside=std::clamp(diagnostic->field_terminal_arc_m-lo,0.,hi-lo);
        diagnostic->field_cost_split[4]+=inside*speed_gradient*speed_gradient;
        diagnostic->field_cost_split[5]+=(hi-lo-inside)*speed_gradient*speed_gradient;
      }
      arc += hi-lo;
    }
  }
  if (diagnostic) {
    diagnostic->field_history_coverage_m=covered;
    diagnostic->field_gradient_coverage_m=arc;
    for(std::size_t i=0;i<6;++i) {
      const double denominator=i<4 ? covered : arc;
      const double weight=i<4 ? config.cost_control_change_weight : config.cost_control_smoothness_weight;
      diagnostic->field_cost_split[i]*=denominator>1e-9 ? weight/denominator : 0.;
    }
  }
  return {arc > 1e-9 ? config.cost_control_smoothness_weight*smooth/arc : 0.,
          covered > 1e-9 ? config.cost_control_change_weight*change/covered : 0.};
}
} // namespace reference_space_mppi_planner::mppi
