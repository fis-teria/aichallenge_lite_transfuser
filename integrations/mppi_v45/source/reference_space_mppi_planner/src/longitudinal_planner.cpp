#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include "reference_space_mppi_planner/reference_field.hpp"
#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include "reference_space_mppi_planner/ot_lane_entry_planner.hpp"
#include "reference_space_mppi_planner/maneuver_policy.hpp"

#include <algorithm>
#include <cmath>

namespace reference_space_mppi_planner::mppi {

void markOvertakeStart(TemporaryReference &geometry) {
  geometry.overtake_start_source_s_m=std::numeric_limits<double>::infinity();
  if(geometry.count<2 || geometry.count>geometry.points.size()) return;
  double arc=0.;
  geometry.points[0].source_s_m=0.;
  for(std::size_t i=1;i<geometry.count;++i) {
    const auto &a=geometry.points[i-1];
    auto &b=geometry.points[i];
    if(!std::isfinite(geometry.overtake_start_source_s_m) &&
        std::abs(b.d_m-geometry.points[0].d_m)>1e-6)
      geometry.overtake_start_source_s_m=arc;
    arc+=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m);
    b.source_s_m=arc;
  }
}

std::optional<LongitudinalLead> longitudinalLead(
    const DynamicObstacle &obstacle, double s, double d, double relative_yaw,
    double time, const Config &config) {
  const auto position = opponent_prediction::positionAt(obstacle, time);
  if (position.s <= s) return std::nullopt;
  const auto extent = [&](double yaw) {
    const double c=std::abs(std::cos(yaw)), sn=std::abs(std::sin(yaw));
    return std::array<double,2>{
        c*config.vehicle_half_length_m+sn*config.vehicle_half_width_m,
        sn*config.vehicle_half_length_m+c*config.vehicle_half_width_m};
  };
  const auto ego=extent(relative_yaw), other=extent(position.relative_yaw);
  const double width=std::max(config.obstacle_lateral_inflation_m,ego[1]+other[1]);
  if (d<=position.d-width || d>=position.d+width) return std::nullopt;
  double speed=obstacle.longitudinal_speed_mps;
  if (obstacle.prediction && obstacle.prediction->points.size()>=2) {
    const auto &points=obstacle.prediction->points;
    auto upper=std::upper_bound(points.begin(),points.end(),time,
        [](double t,const auto &p){return t<p.time;});
    if(upper==points.begin()) ++upper;
    if(upper==points.end()) --upper;
    const auto &lower=*std::prev(upper);
    if(upper->time<=lower.time) return std::nullopt;
    speed=(upper->global_s-lower.global_s)/(upper->time-lower.time);
  }
  return LongitudinalLead{position.s-
      std::max(config.obstacle_longitudinal_inflation_m,ego[0]+other[0])-s,speed,
      std::hypot(obstacle.longitudinal_speed_mps,obstacle.lateral_speed_mps)};
}

double followingAcceleration(const LongitudinalLead &lead, double speed,
                             const Config &config) {
  const double tau=config.approach_time_sec;
  const double gap=followingGapForSpeed(lead.observed_speed_mps.value_or(lead.speed_mps),
      config.follow_gap_m,config.overtake_follow_gap_m);
  return (lead.body_gap_m-gap)/(tau*tau)+
      2.0*(lead.speed_mps-speed)/tau;
}

LongitudinalSpeedPlan planLongitudinalSpeed(
    const TemporaryReference &geometry, const PlanRequest &request,
    const Config &config, bool use_preparation) {
  LongitudinalSpeedPlan result;
  result.reference=geometry;
  if(geometry.count<3 || geometry.count>geometry.points.size() ||
      request.base_reference_count<2 ||
      request.base_reference_count>request.base_reference.size() ||
      request.dynamic_obstacle_count>request.dynamic_obstacles.size() ||
      !std::isfinite(request.ego.speed_mps) ||
      !(config.dt_sec>0.) || !(config.approach_time_sec>0.) ||
      !(config.speed_proportional_gain>0.)) return result;
  const auto path=cartesianArcField(geometry);
  for(std::size_t i=0;i<path.count;++i) {
    const auto &p=path.points[i];
    if(!std::isfinite(p.s_m) || !std::isfinite(p.x_m) ||
        !std::isfinite(p.y_m) || !std::isfinite(p.yaw_rad) ||
        !std::isfinite(p.curvature_1pm) ||
        (i && p.s_m<=path.points[i-1].s_m)) return result;
  }
  double cruise=0.;
  for(std::size_t i=0;i<request.base_reference_count;++i) {
    if(!std::isfinite(request.base_reference[i].speed_mps)) return result;
    cruise=std::max(cruise,request.base_reference[i].speed_mps);
  }
  if(!std::isfinite(cruise) || cruise<0.) return result;
  const BaseProjectionIndex base(request.base_reference.data(),request.base_reference_count);
  std::optional<double> preparation_origin;
  const bool preparation_required=request.passing_preparation && request.passing_preparation->front_merge;
  if((use_preparation || preparation_required) && request.passing_preparation && request.world_reference &&
      request.passing_preparation->world==request.world_reference) {
    preparation_origin=preparationStation(*request.passing_preparation,request.ego.x_m,request.ego.y_m);
  }
  double station=0., speed=std::max(0.,request.ego.speed_mps);
  double acceleration=request.ego.acceleration_mps2, previous_station=0., previous_command=0.;
  std::size_t next=0, segment=0;
  AwsimLongitudinalResponse plant;
  auto steps=std::min(kMaximumHorizonSteps,std::max<std::size_t>(
      2,request.horizon_steps_override ? request.horizon_steps_override : config.horizon_steps));
  if(preparation_required && !request.passing_preparation->schedule.empty()) {
    const auto &plan=*request.passing_preparation;
    const double remaining=std::clamp(plan.epoch_sec+plan.schedule.back().time_sec-request.stamp_sec,0.,20.);
    // Speed guidance must reach the timed target beyond the short PP rollout.
    // This loop stores no rollout states; its independent bound is 400 steps.
    steps=std::max(steps,static_cast<std::size_t>(std::min(400.,std::ceil(remaining/config.dt_sec))));
  }
  for(std::size_t step=0;step<=steps;++step) {
    while(segment+2<path.count && path.points[segment+1].s_m<station) ++segment;
    const auto &a=path.points[segment], &b=path.points[segment+1];
    const double u=std::clamp((station-a.s_m)/(b.s_m-a.s_m),0.,1.);
    const auto projected=base.project(a.x_m+u*(b.x_m-a.x_m),a.y_m+u*(b.y_m-a.y_m));
    const double yaw=a.yaw_rad+u*std::remainder(b.yaw_rad-a.yaw_rad,2.*M_PI);
    double desired=config.speed_proportional_gain*(cruise-speed);
    if(preparation_origin) {
      const auto goal=preparationTarget(*request.passing_preparation,request.stamp_sec+step*config.dt_sec);
      if(goal) {
        const double tau=config.approach_time_sec;
        desired=goal->acceleration_mps2+2.*(goal->speed_mps-speed)/tau+
            (goal->station_m-(*preparation_origin+projected.s))/(tau*tau);
      }
    }
    if(use_preparation && request.ot_lane_entry &&
        request.ot_lane_entry->preparation.world==request.world_reference) {
      const auto &entry=request.ot_lane_entry->preparation;
      const auto origin=preparationStation(entry,request.ego.x_m,request.ego.y_m);
      const auto goal=preparationTarget(entry,request.stamp_sec+step*config.dt_sec);
      if(origin && goal && *origin+projected.s<entry.entry_station_m) {
        const double tau=config.approach_time_sec;
        desired=goal->acceleration_mps2+2.*(goal->speed_mps-speed)/tau+
            (goal->station_m-(*origin+projected.s))/(tau*tau);
      }
    }
    const double source_station=a.source_s_m+u*(b.source_s_m-a.source_s_m);
    for(std::size_t i=0;source_station<geometry.overtake_start_source_s_m &&
        i<request.dynamic_obstacle_count;++i) {
      const auto lead=longitudinalLead(request.dynamic_obstacles[i],projected.s,projected.d,
          std::remainder(yaw-projected.yaw,2.*M_PI),step*config.dt_sec,config);
      if(lead) desired=std::min(desired,followingAcceleration(*lead,speed,config));
    }
    // CMA converts command speed to requested acceleration with its P gain.
    // Account for AWSIM rolling/drag at equilibrium; command != achieved speed.
    const double resistance=config.awsim_vehicle_response_enabled && speed>0. ?
        .37+.03*speed : 0.;
    const double requested=std::clamp(desired+resistance,
        -config.maximum_deceleration_mps2,config.maximum_acceleration_mps2);
    const double command=std::clamp(speed+requested/config.speed_proportional_gain,0.,cruise);
    if(!std::isfinite(command)) return result;
    while(next<path.count && path.points[next].s_m<=station+1e-9) {
      const double ratio=station>previous_station ?
          std::clamp((path.points[next].s_m-previous_station)/(station-previous_station),0.,1.) : 1.;
      auto &p=result.reference.points[next++];
      p.speed_mps=p.uncapped_speed_mps=previous_command+ratio*(command-previous_command);
    }
    if(station>=path.points[path.count-1].s_m || step==steps) {
      // The unconsumed suffix is guidance, checked again on the next snapshot.
      for(;next<path.count;++next)
        result.reference.points[next].speed_mps=result.reference.points[next].uncapped_speed_mps=command;
      break;
    }
    previous_station=station; previous_command=command;
    const double actual_request=std::clamp(config.speed_proportional_gain*(command-speed),
        -config.maximum_deceleration_mps2,config.maximum_acceleration_mps2);
    if(config.awsim_vehicle_response_enabled)
      acceleration=plant.acceleration(actual_request,speed,step*config.dt_sec);
    else
      acceleration+=(1.-std::exp(-config.dt_sec/config.acceleration_time_constant_sec))*
          (actual_request-acceleration);
    station+=speed*config.dt_sec;
    speed=std::max(0.,speed+acceleration*config.dt_sec);
    ++result.prediction_steps;
  }
  result.valid=true;
  return result;
}

ExecutionSpeedResult planAndEvaluateLongitudinalSpeed(
    const ReferenceSpaceMppiPlanner &evaluator,const TemporaryReference &geometry,
    const PlanRequest &request,const Config &config) {
  ExecutionSpeedResult result;
  const std::size_t count=request.passing_preparation || request.ot_lane_entry ? 2U : 1U;
  const bool preparation_required=request.passing_preparation && request.passing_preparation->front_merge;
  TemporaryReference normal;
  // Retiming a retained front-merge path has the same timed target as new
  // candidates; ordinary cruise cannot win merely because it costs less.
  for(std::size_t i=preparation_required ? 1U : 0U;i<count;++i) {
    auto plan=planLongitudinalSpeed(geometry,request,config,i==1U);
    if(!plan.valid) continue;
    for(std::size_t j=0;j<plan.reference.count;++j)
      plan.reference.points[j].speed_mps=static_cast<float>(plan.reference.points[j].speed_mps);
    if(i==0) normal=plan.reference;
    else {
      bool equal=normal.count==plan.reference.count;
      for(std::size_t j=0;equal && j<normal.count;++j)
        equal=normal.points[j].speed_mps==plan.reference.points[j].speed_mps;
      if(equal) continue;
    }
    const auto evaluation=evaluator.evaluateExecutionReference(plan.reference,request);
    ++result.evaluated;
    if(evaluation.valid) ++result.valid;
    if(result.evaluated==1U || (evaluation.valid &&
        (!result.evaluation.valid || evaluation.cost<result.evaluation.cost))) {
      result.reference=std::move(plan.reference);result.evaluation=evaluation;
      result.preparation_selected=i==1U;
    }
  }
  result.improved=result.evaluation.valid;
  return result;
}
}  // namespace reference_space_mppi_planner::mppi
