#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace reference_space_mppi_planner {

inline visualization_msgs::msg::MarkerArray makePassingOpportunityMarkers(
    const mppi::PlanRequest *request, std_msgs::msg::Header header,
    const std::string &leader_id) {
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray out;
  if (header.frame_id.empty()) header.frame_id = "map";
  Marker clear;
  clear.header = header;
  clear.action = Marker::DELETEALL;
  out.markers.push_back(clear);
  if (!request || !request->world_reference || request->leader_passing_entry_m < 0. ||
      !(request->leader_passing_exit_m > request->leader_passing_entry_m) ||
      request->leader_preparation_m < 0. || request->leader_pass_distance_m < 0. ||
      !(request->leader_pass_time_sec > 0.))
    return out;
  const auto &r = *request;
  const auto &world = *r.world_reference;
  const auto origin = world.projectStation(r.ego.x_m, r.ego.y_m);
  if (!origin) return out;
  const auto position = [&](double distance) -> std::optional<geometry_msgs::msg::Point> {
    const auto pose = world.pose(*origin + distance, 0.);
    if (!pose) return {};
    geometry_msgs::msg::Point point;
    point.x = (*pose)[0]; point.y = (*pose)[1]; point.z = .5;
    return point;
  };
  const auto preparation = position(r.leader_preparation_m);
  const auto completion = position(r.leader_pass_distance_m);
  if (!preparation || !completion) return out;
  const auto marker = [&](int id, int type, float red, float green, float blue) {
    Marker m;
    m.header = header; m.ns = "mppi_passing_opportunity"; m.id = id;
    m.type = type; m.action = Marker::ADD; m.pose.orientation.w = 1.;
    m.color.r = red; m.color.g = green; m.color.b = blue; m.color.a = .95F;
    m.lifetime = rclcpp::Duration::from_seconds(.5);
    return m;
  };
  auto interval = marker(0, Marker::LINE_STRIP, 1.F, .75F, .05F);
  interval.scale.x = .35;
  const auto steps = std::max(1U, static_cast<unsigned>(
      std::ceil(r.leader_passing_exit_m - r.leader_passing_entry_m)));
  for (unsigned i = 0; i <= steps; ++i) {
    const auto point = position(r.leader_passing_entry_m +
        (r.leader_passing_exit_m - r.leader_passing_entry_m) * i / steps);
    if (!point) return out;
    interval.points.push_back(*point);
  }
  auto prepare = marker(1, Marker::SPHERE, .1F, .85F, 1.F);
  prepare.pose.position = *preparation;
  prepare.scale.x = prepare.scale.y = prepare.scale.z = .9;
  auto pass = marker(2, Marker::SPHERE, 1.F, .15F, .8F);
  pass.pose.position = *completion;
  pass.scale.x = pass.scale.y = pass.scale.z = 1.4;
  auto label = marker(3, Marker::TEXT_VIEW_FACING, 1.F, .65F, .95F);
  label.pose.position = *completion; label.pose.position.z = 2.5;
  const auto completion_pose = world.pose(*origin + r.leader_pass_distance_m, 0.);
  label.pose.position.x -= 3. * std::sin((*completion_pose)[2]);
  label.pose.position.y += 3. * std::cos((*completion_pose)[2]);
  label.scale.z = 1.4;
  std::ostringstream text;
  text << "PASS CANDIDATE " << leader_id << "\n+" << std::fixed
       << std::setprecision(1) << r.leader_pass_time_sec << " s / "
       << r.leader_pass_distance_m << " m ahead";
  label.text = text.str();
  out.markers.push_back(std::move(interval));
  out.markers.push_back(std::move(prepare));
  out.markers.push_back(std::move(pass));
  out.markers.push_back(std::move(label));
  return out;
}

inline visualization_msgs::msg::MarkerArray makePreparationPlanMarkers(
    const mppi::PassingPreparationPlan *plan,std_msgs::msg::Header header,bool active) {
  if(!plan || !plan->world || rclcpp::Time(header.stamp).seconds()>plan->epoch_sec+plan->targetTimeSec())
    return makePassingOpportunityMarkers(nullptr,header,"");
  const auto origin=plan->world->pose(plan->origin_station_m,0.);
  if(!origin) return makePassingOpportunityMarkers(nullptr,header,"");
  mppi::PlanRequest request;
  request.world_reference=plan->world;
  request.ego.x_m=(*origin)[0];request.ego.y_m=(*origin)[1];
  request.leader_passing_entry_m=std::max(0.,plan->entry_station_m-plan->origin_station_m);
  request.leader_passing_exit_m=plan->exit_station_m-plan->origin_station_m;
  request.leader_preparation_m=plan->acceleration_start_station_m-plan->origin_station_m;
  request.leader_pass_distance_m=plan->targetStationM()-plan->origin_station_m;
  request.leader_pass_time_sec=plan->targetTimeSec();
  auto result=makePassingOpportunityMarkers(&request,header,plan->target_id);
  if(result.markers.size()!=5U) return result;
  auto &label=result.markers[4];
  std::ostringstream text;
  text << (active?"ACTIVE ":"CANDIDATE ") << plan->target_id << " plan=" << plan->id << "/" << plan->revision
       << (plan->complete_pass ? "\npass +" : "\nprepare +") << std::fixed << std::setprecision(1)
       << std::max(0.,plan->epoch_sec+plan->targetTimeSec()-rclcpp::Time(header.stamp).seconds()) << " s";
  label.text=text.str();
  const auto pose=plan->world->pose(plan->lateral_start_station_m,0.);
  if(pose) {
    auto lateral=result.markers[2];
    lateral.id=4;lateral.type=visualization_msgs::msg::Marker::CUBE;
    lateral.pose.position.x=(*pose)[0];lateral.pose.position.y=(*pose)[1];
    lateral.scale.x=lateral.scale.y=.65;lateral.scale.z=1.4;
    lateral.color.r=.5F;lateral.color.g=1.F;lateral.color.b=.2F;
    result.markers.push_back(std::move(lateral));
  }
  return result;
}

}  // namespace reference_space_mppi_planner
