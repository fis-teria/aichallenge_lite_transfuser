#pragma once

#include "reference_space_mppi_planner/precomputed_wall_lines.hpp"
#include "reference_space_mppi_planner/passing_point_areas.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

namespace reference_space_mppi_planner {

// Compatibility names for existing marker callers; planning uses the same geometry.
using PassingPointDisplayConfig=PassingPointAreaConfig;
using PassingPointDisplayAreas=PassingPointAreas;
inline auto passingPointDisplayAreas(const ReferencePoseIndex &world,double length,bool closed,
    const PassingPointDisplayConfig &config) {
  return passingPointAreas(world,length,closed,config);
}

inline visualization_msgs::msg::MarkerArray makePassingPointForbiddenMarkers(
    const PrecomputedWallLines *lines,
    const PassingPointDisplayConfig &config, std_msgs::msg::Header header) {
  using Marker=visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray out;
  if (header.frame_id.empty()) header.frame_id="map";
  Marker clear;
  clear.header=header; clear.action=Marker::DELETEALL;
  out.markers.push_back(clear);
  if (!lines || !lines->world || lines->samples.size()<2) return out;
  const auto areas=passingPointDisplayAreas(*lines->world,lines->length_m,lines->closed,config);
  if (!areas) return out;
  const auto marker=[&](int id,int type) {
    Marker m;
    m.header=header; m.ns="mppi_passing_point_forbidden"; m.id=id;
    m.type=type; m.action=Marker::ADD; m.pose.orientation.w=1.;
    m.scale.x=m.scale.y=m.scale.z=1.;
    m.color.r=1.F; m.color.g=.15F; m.color.b=.1F; m.color.a=.26F;
    return m;
  };
  const auto cross_section=[&](double s,double z)
      ->std::optional<std::array<geometry_msgs::msg::Point,2>> {
    const auto left=lines->at(s,1), right=lines->at(s,-1);
    if (!left || !right) return {};
    std::array<geometry_msgs::msg::Point,2> points;
    for (std::size_t i=0;i<2;++i) {
      const auto &xy=i ? *right : *left;
      points[i].x=xy[0]; points[i].y=xy[1]; points[i].z=z;
    }
    return points;
  };
  auto fill=marker(0,Marker::TRIANGLE_LIST);
  for (const auto &[start,end]:areas->forbidden) {
    const auto steps=std::max(1U,static_cast<unsigned>(std::ceil((end-start)/.5)));
    for (unsigned i=0;i<steps;++i) {
      const auto a=cross_section(start+(end-start)*i/steps,.08);
      const auto b=cross_section(start+(end-start)*(i+1)/steps,.08);
      if (!a || !b) return out;
      for (const auto &p:std::array<geometry_msgs::msg::Point,6>{
          (*a)[0],(*a)[1],(*b)[1],(*a)[0],(*b)[1],(*b)[0]})
        fill.points.push_back(p);
    }
  }
  out.markers.push_back(std::move(fill));
  auto boundaries=marker(1,Marker::LINE_LIST);
  boundaries.scale.x=.18; boundaries.color.g=.65F; boundaries.color.b=.05F;
  boundaries.color.a=1.F;
  for (std::size_t i=0;i<areas->deadlines.size();++i) {
    double station=areas->deadlines[i];
    if (lines->closed) {station=std::fmod(station,lines->length_m); if(station<0.)station+=lines->length_m;}
    const auto section=cross_section(station,.15);
    const auto pose=lines->world->pose(station,0.);
    if (!section || !pose) continue;
    boundaries.points.insert(boundaries.points.end(),section->begin(),section->end());
    auto label=marker(static_cast<int>(i)+2,Marker::TEXT_VIEW_FACING);
    label.color=boundaries.color; label.scale.z=1.4;
    label.pose.position=(*section)[0]; label.pose.position.z=1.5;
    label.pose.position.x-=2.*std::sin((*pose)[2]);
    label.pose.position.y+=2.*std::cos((*pose)[2]);
    std::ostringstream text;
    text<<"NO PASS POINT\nC"<<i+1<<" corner -"<<std::fixed
        <<std::setprecision(1)<<config.corner_margin_m<<" m";
    label.text=text.str();
    out.markers.push_back(std::move(label));
  }
  out.markers.push_back(std::move(boundaries));
  return out;
}

}  // namespace reference_space_mppi_planner
