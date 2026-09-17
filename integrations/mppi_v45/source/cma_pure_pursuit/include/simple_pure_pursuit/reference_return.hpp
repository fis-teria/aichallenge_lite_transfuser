#pragma once

#include "simple_pure_pursuit/reference_connection.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <vector>

namespace simple_pure_pursuit {

struct ReferenceReturnState {
  double x_m, y_m, yaw_rad, steering_rad;
};
struct ReferenceReturnConfig {
  double wheel_base_m, maximum_tire_steering_angle_rad;
};

// Geometry only. The planner supplies its execution/footprint validator; the
// controller uses this same connection when its direct command is delayed.
inline std::optional<autoware_auto_planning_msgs::msg::Trajectory>
makeReferenceReturn(const autoware_auto_planning_msgs::msg::Trajectory &route,
    const ReferenceReturnState &ego, const ReferenceReturnConfig &config,
    double minimum_length_m = 8.0) {
  if (route.points.size() < 3) return std::nullopt;
  struct Point { double x_m{}, y_m{}, s_m{}; };
  struct Reference { std::array<Point,384> points{}; std::size_t count{}; } reference;
  std::vector<double> arc(route.points.size());
  for (std::size_t i=1;i<arc.size();++i) {
    const auto &a=route.points[i-1].pose.position,&b=route.points[i].pose.position;
    arc[i]=arc[i-1]+std::hypot(b.x-a.x,b.y-a.y);
  }
  if (!(arc.back()>1e-3) || !std::isfinite(arc.back())) return std::nullopt;
  reference.count=std::min<std::size_t>(384,std::max<std::size_t>(3,
      static_cast<std::size_t>(std::ceil(arc.back()/.2))+1));
  auto out=route;out.points.clear();
  std::size_t segment=1;
  for (std::size_t i=0;i<reference.count;++i) {
    // Resolve the measured-curvature endpoint in the discrete path too. The
    // first two short intervals avoid turning a C2 endpoint into a coarse
    // three-point curvature jump at the controller's nearest sample.
    const double step=arc.back()/std::max<std::size_t>(1,reference.count-3);
    const double s=reference.count>4 ?
        (i<3?std::min(.02,step*.25)*i:step*(i-2)):
        arc.back()*i/(reference.count-1);
    while(segment+1<arc.size() && arc[segment]<s) ++segment;
    const auto &a=route.points[segment-1],&b=route.points[segment];
    const double ratio=std::clamp((s-arc[segment-1])/std::max(1e-9,arc[segment]-arc[segment-1]),0.,1.);
    auto point=a;
    point.pose.position.x=a.pose.position.x+ratio*(b.pose.position.x-a.pose.position.x);
    point.pose.position.y=a.pose.position.y+ratio*(b.pose.position.y-a.pose.position.y);
    point.longitudinal_velocity_mps=a.longitudinal_velocity_mps+ratio*(b.longitudinal_velocity_mps-a.longitudinal_velocity_mps);
    reference.points[i]={point.pose.position.x,point.pose.position.y,s};
    out.points.push_back(point);
  }
  if (!reference_connection::connect(ego,config,minimum_length_m,&reference)) return std::nullopt;
  for (std::size_t i=0;i<reference.count;++i) {
    const auto &p=reference.points[i];
    auto &pose=out.points[i].pose;pose.position.x=p.x_m;pose.position.y=p.y_m;
    const auto &a=reference.points[i?i-1:0],&b=reference.points[std::min(i+1,reference.count-1)];
    const double yaw=i?std::atan2(b.y_m-a.y_m,b.x_m-a.x_m):ego.yaw_rad;
    pose.orientation.x=pose.orientation.y=0.;
    pose.orientation.z=std::sin(yaw*.5);pose.orientation.w=std::cos(yaw*.5);
  }
  return out;
}
}  // namespace simple_pure_pursuit
