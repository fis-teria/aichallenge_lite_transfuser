#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include "reference_space_mppi_planner/leader_passing_opportunity.hpp"
#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include <algorithm>
#include <cmath>
#include "reference_space_mppi_planner/reference_field.hpp"
#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include "reference_space_mppi_planner/execution_motion.hpp"

namespace reference_space_mppi_planner::mppi {

namespace {
double priorLapCandidateDistance(const PlanRequest &r,const Config &c,
    const PassingPreparationPlan &plan,int side) {
  const auto blend=[](double x) {x=std::clamp(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x));};
  const auto xy=[&](double station)->std::optional<std::array<double,2>> {
    const auto center=plan.world->pose(station,0.);
    const auto wall=plan.wall_lines->at(station,side);
    if(!center || !wall) return {};
    const double entry=blend((station-plan.origin_station_m)/
        (plan.entry_station_m-plan.origin_station_m));
    const double returning=blend((station-plan.merge_start_station_m)/
        (plan.merge_end_station_m-plan.merge_start_station_m));
    return std::array<double,2>{
        (*center)[0]-(1.-entry)*r.ego.d_m*std::sin((*center)[2])+
            entry*(1.-returning)*((*wall)[0]-(*center)[0]),
        (*center)[1]+(1.-entry)*r.ego.d_m*std::cos((*center)[2])+
            entry*(1.-returning)*((*wall)[1]-(*center)[1])};
  };
  double minimum=INFINITY;
  for(const auto &point:plan.schedule) {
    const auto position=xy(point.station_m),before=xy(std::max(plan.origin_station_m,point.station_m-.05)),
        after=xy(point.station_m+.05);
    if(!position || !before || !after) return -1.;
    const auto other=r.leader_passing_prediction->at(point.time_sec);
    const double yaw=std::atan2((*after)[1]-(*before)[1],(*after)[0]-(*before)[0]);
    const std::array<double,3> ego{(*position)[0],(*position)[1],yaw};
    const auto overlap=plan.world->occupancyOverlap(ego,other.global_s,other.global_s,
        other.d,other.d,other.relative_yaw,c.vehicle_half_length_m,c.vehicle_half_width_m,
        c.obstacle_longitudinal_inflation_m,c.obstacle_lateral_inflation_m);
    if(!overlap || *overlap) return -1.;
    const auto pose=plan.world->pose(other.global_s,other.d);
    if(!pose) return -1.;
    minimum=std::min(minimum,std::hypot((*pose)[0]-ego[0],(*pose)[1]-ego[1]));
    if(point.station_m>=plan.merge_end_station_m) {
      const auto &leader=r.dynamic_obstacles[r.leader_opportunity_index];
      const double progress=leader.s_m+other.global_s-r.leader_passing_prediction->points.front().global_s;
      if(point.station_m-plan.origin_station_m<=progress+2.*c.vehicle_half_length_m) return -1.;
      return minimum;
    }
  }
  return -1.;
}

std::shared_ptr<const PassingPreparationPlan> makePriorLapFrontMergeGoal(
    const PlanRequest &r,const Config &c,const std::string &target,std::uint64_t revision,
    const std::shared_ptr<const PassingPreparationPlan> &previous,double origin,
    double minimum_pass_time_sec,bool preselect_side) {
  const auto &prediction=*r.leader_passing_prediction;
  if(prediction.points.size()<2 || prediction.points.front().time>0.) return {};
  auto p=std::make_shared<PassingPreparationPlan>();
  p->id=p->revision=revision;p->target_id=target;p->world=r.world_reference;
  p->wall_lines=r.precomputed_wall_lines;p->epoch_sec=r.stamp_sec;p->origin_station_m=origin;
  p->front_merge=p->prior_lap_guidance=true;
  p->prior_lap_source_stamp_sec=prediction.source_stamp;
  p->acceleration_start_station_m=p->lateral_start_station_m=origin;
  double s=0.,v=std::max(0.,r.ego.speed_mps),cruise=0.;
  for(std::size_t i=0;i<r.base_reference_count;++i)
    cruise=std::max(cruise,r.base_reference[i].speed_mps);
  const double entry=std::clamp(v*2.5,15.,30.);
  const double merge=std::clamp(cruise*2.5,15.,25.);
  auto leader=r.dynamic_obstacles[r.leader_opportunity_index];
  leader.prediction=r.leader_passing_prediction;
  AwsimLongitudinalResponse plant;
  bool passed=false;
  constexpr double dt=.05;
  for(double t=0.;t<=std::min(20.,prediction.points.back().time)+1e-9;t+=dt) {
    const double requested=std::clamp(c.speed_proportional_gain*(cruise-v),
        -c.maximum_deceleration_mps2,c.maximum_acceleration_mps2);
    const double a=c.awsim_vehicle_response_enabled ? plant.acceleration(requested,v,t) : requested;
    p->schedule.push_back({t,origin+s,v,a});
    const auto other=opponent_prediction::positionAt(leader,t);
    if(!passed && t>=minimum_pass_time_sec && s>=entry && s>=other.s+2.*c.vehicle_half_length_m &&
        (!r.passing_point_areas || r.passing_point_areas->contains(origin+s))) {
      passed=true;p->entry_station_m=origin+entry;p->entry_speed_mps=v;
      p->pass_station_m=p->merge_start_station_m=origin+s;p->pass_time_sec=t;
      p->merge_end_station_m=origin+s+merge;p->exit_station_m=p->merge_end_station_m+30.;
    }
    if(passed && origin+s>=p->merge_end_station_m) {
      if(!preselect_side) return p; // Both wall lines receive full candidate validation.
      p->candidate_minimum_center_distance_m={priorLapCandidateDistance(r,c,*p,1),
                                             priorLapCandidateDistance(r,c,*p,-1)};
      const auto &scores=p->candidate_minimum_center_distance_m;
      if(scores[0]<0. && scores[1]<0.) return {};
      const bool keep=previous && previous->prior_lap_guidance && previous->target_id==target &&
          previous->world==r.world_reference && previous->wall_lines==r.precomputed_wall_lines &&
          origin<previous->exit_station_m && (previous->side==1 || previous->side==-1) &&
          scores[previous->side==1 ? 0 : 1]>=0.;
      p->side=keep ? previous->side : scores[0]>=scores[1] ? 1 : -1;
      if(keep) p->id=previous->id;
      return p;
    }
    const double next=std::max(0.,v+a*dt);s+=.5*dt*(v+next);v=next;
  }
  // No end-of-history clamping or constant-speed substitution can establish
  // a prior-lap pass beyond the observed prediction horizon.
  return {};
}
} // namespace

std::shared_ptr<const PassingPreparationPlan> makeFrontMergeGoal(
    const PlanRequest &r,const Config &c,const std::string &target,std::uint64_t revision,
    const std::shared_ptr<const PassingPreparationPlan> &previous,
    double minimum_pass_time_sec,bool preselect_side) {
  if(!r.world_reference || !r.precomputed_wall_lines || target.empty() ||
      r.leader_opportunity_index>=r.dynamic_obstacle_count) return {};
  const auto origin=r.world_reference->projectStation(r.ego.x_m,r.ego.y_m);
  if(!origin) return {};
  const auto &other=r.dynamic_obstacles[r.leader_opportunity_index];
  if(!(other.s_m>0.)) return {};
  if(r.leader_passing_prediction)
    return makePriorLapFrontMergeGoal(r,c,target,revision,previous,*origin,minimum_pass_time_sec,preselect_side);
  if(!(other.longitudinal_speed_mps>0.)) return {};
  auto p=std::make_shared<PassingPreparationPlan>();
  p->id=p->revision=revision;p->target_id=target;p->world=r.world_reference;
  p->wall_lines=r.precomputed_wall_lines;p->epoch_sec=r.stamp_sec;
  p->origin_station_m=*origin;p->front_merge=true;
  p->acceleration_start_station_m=p->lateral_start_station_m=*origin;
  double s=0.,v=r.ego.speed_mps,a=0.;
  double cruise=0.;
  for(std::size_t i=0;i<r.base_reference_count;++i)
    cruise=std::max(cruise,r.base_reference[i].speed_mps);
  const double entry=std::clamp(v*2.5,15.,30.);
  const double merge=std::clamp(cruise*2.5,15.,25.);
  AwsimLongitudinalResponse plant;
  for(double t=0.;t<=15.;t+=.05) {
    const double requested=std::clamp(c.speed_proportional_gain*(cruise-v),
        -c.maximum_deceleration_mps2,c.maximum_acceleration_mps2);
    a=c.awsim_vehicle_response_enabled ? plant.acceleration(requested,v,t) : requested;
    p->schedule.push_back({t,*origin+s,v,a});
    if(t>=minimum_pass_time_sec && s>=entry && s>=other.s_m+other.longitudinal_speed_mps*t+2.*c.vehicle_half_length_m &&
        (!r.passing_point_areas || r.passing_point_areas->contains(*origin+s))) {
      p->entry_station_m=*origin+entry;
      p->pass_station_m=p->merge_start_station_m=*origin+s;
      p->pass_time_sec=t;p->merge_end_station_m=p->merge_start_station_m+merge;
      p->exit_station_m=p->merge_end_station_m+30.;
      p->entry_speed_mps=v;
      return p;
    }
    const double next=std::max(0.,v+a*.05);s+=.025*(v+next);v=next;
  }
  return {};
}

std::vector<std::shared_ptr<const PassingPreparationPlan>> makeFrontMergeGoals(
    const PlanRequest &r,const Config &c,const std::string &target,std::uint64_t revision) {
  std::vector<std::shared_ptr<const PassingPreparationPlan>> result;
  const auto first=makeFrontMergeGoal(r,c,target,revision,{},0.,false);
  if(!first) return result;
  for(int option=0;option<3;++option) {
    const auto goal=option==0 ? first : makeFrontMergeGoal(r,c,target,revision,{},
        first->pass_time_sec+option,false);
    if(!goal || (!result.empty() && std::abs(goal->pass_station_m-result.back()->pass_station_m)<1e-6)) continue;
    auto candidate=std::make_shared<PassingPreparationPlan>(*goal);
    candidate->id=revision*3+static_cast<std::uint64_t>(option);
    result.push_back(std::move(candidate));
  }
  return result;
}

RejectReason validateFrontMerge(const PlanRequest &r,const Config &c,
    const TemporaryReference &geometry,FrontMergeMetrics *metrics) {
  if(metrics) *metrics={};
  if(!r.front_merge_attack) return RejectReason::NONE;
  if(!r.world_reference || r.leader_opportunity_index>=r.dynamic_obstacle_count ||
      !geometry.front_merge_end_index || geometry.front_merge_end_index+1>=geometry.count)
    return RejectReason::GEOMETRY;
  const auto path=cartesianArcField(geometry);
  const double end=path.points[geometry.front_merge_end_index].s_m;
  const bool prior=r.passing_preparation && r.passing_preparation->prior_lap_guidance;
  if(prior && (!r.leader_passing_prediction || r.leader_passing_prediction->points.size()<2))
    return RejectReason::INVALID_INPUT;
  const auto predicted=[&](std::size_t index,double t) {
    const auto &o=r.dynamic_obstacles[index];
    if(prior && index==r.leader_opportunity_index) return r.leader_passing_prediction->at(t);
    return opponent_prediction::PredictionPoint{t,o.global_reference_s_m+o.longitudinal_speed_mps*t,o.d_m,0.};
  };
  const auto check_pose=[&](const RolloutState &state,double arc) {
    const double t=state.time_sec;
    if(prior && t>r.leader_passing_prediction->points.back().time+1e-9)
      return RejectReason::INVALID_INPUT;
    const std::array<double,3> ego{state.x_m,state.y_m,state.yaw_rad};
    for(std::size_t i=0;i<r.dynamic_obstacle_count;++i) {
      const auto other=predicted(i,t);
      const auto overlap=r.world_reference->occupancyOverlap(ego,other.global_s,other.global_s,
          other.d,other.d,other.relative_yaw,
          c.vehicle_half_length_m,c.vehicle_half_width_m,
          c.obstacle_longitudinal_inflation_m,c.obstacle_lateral_inflation_m);
      if(!overlap) return RejectReason::INVALID_INPUT;
      if(*overlap) return RejectReason::COLLISION;
      if(metrics && i==r.leader_opportunity_index) {
        const auto pose=r.world_reference->pose(other.global_s,other.d);
        if(!pose) return RejectReason::INVALID_INPUT;
        const double dx=(*pose)[0]-ego[0],dy=(*pose)[1]-ego[1];
        const double yaw=(*pose)[2]+other.relative_yaw-ego[2];
        const double longitudinal=std::abs(dx*std::cos(ego[2])+dy*std::sin(ego[2]));
        const double other_length=c.vehicle_half_length_m*std::abs(std::cos(yaw))+
            c.vehicle_half_width_m*std::abs(std::sin(yaw));
        if(longitudinal<c.vehicle_half_length_m+other_length) {
          const double other_width=c.vehicle_half_width_m*std::abs(std::cos(yaw))+
              c.vehicle_half_length_m*std::abs(std::sin(yaw));
          const double lateral=std::abs(-dx*std::sin(ego[2])+dy*std::cos(ego[2]));
          metrics->alongside_clearance_m=std::min(metrics->alongside_clearance_m,
              std::max(0.,lateral-c.vehicle_half_width_m-other_width));
        }
      }
    }
    if(arc>=end) {
      const auto station=r.world_reference->projectStation(ego[0],ego[1]);
      const auto other=predicted(r.leader_opportunity_index,t);
      if(!station) return RejectReason::INVALID_INPUT;
      const auto center=r.world_reference->pose(*station,0.);
      if(!center || std::hypot(ego[0]-(*center)[0],ego[1]-(*center)[1])>.30 ||
          std::abs(std::remainder(ego[2]-(*center)[2],2.*M_PI))>.20)
        return RejectReason::GEOMETRY;
      const double gap=r.world_reference->stationDifference(*station,
          other.global_s);
      if(metrics && gap>2.*c.vehicle_half_length_m) metrics->completion_time_sec=t;
      return gap>2.*c.vehicle_half_length_m ? RejectReason::NONE : RejectReason::COLLISION;
    }
    return RejectReason::NONE;
  };
  const RolloutState initial{r.ego.x_m,r.ego.y_m,r.ego.yaw_rad,r.ego.speed_mps,0.};
  auto reason=check_pose(initial,0.);
  if(reason!=RejectReason::NONE) return reason;
  bool completed=false;
  TemporaryReference segment;
  segment.count=2;
  visitExecutionMotion(path,r,c,static_cast<std::size_t>(std::ceil(20./c.dt_sec)),
      [&](const ExecutionMotionSample &sample) {
    if(sample.rejection!=RejectReason::NONE) {reason=sample.rejection;return false;}
    // Keep the existing geometry-only wall contract and opponent forecast
    // policy, but sweep the tracked body instead of ideal reference poses.
    if(r.path_constraint_geometry_only && r.path_constraint_validator) {
      for(std::size_t i=0;i<2;++i) {
        const auto &state=i ? sample.after : sample.before;
        auto &p=segment.points[i];p.x_m=state.x_m;p.y_m=state.y_m;p.yaw_rad=state.yaw_rad;
        p.speed_mps=state.speed_mps;
      }
      segment.points[1].s_m=std::hypot(sample.after.x_m-sample.before.x_m,sample.after.y_m-sample.before.y_m);
      reason=r.path_constraint_validator(segment);
      if(reason!=RejectReason::NONE) return false;
    }
    const auto steps=std::max<std::size_t>(1,std::ceil(c.dt_sec/.02));
    for(std::size_t i=1;i<=steps;++i) {
      const double u=static_cast<double>(i)/steps;
      const auto &a=sample.before,&b=sample.after;
      const RolloutState state{a.x_m+u*(b.x_m-a.x_m),a.y_m+u*(b.y_m-a.y_m),
          a.yaw_rad+u*std::remainder(b.yaw_rad-a.yaw_rad,2.*M_PI),
          a.speed_mps+u*(b.speed_mps-a.speed_mps),a.time_sec+u*(b.time_sec-a.time_sec)};
      const auto projected=projectExecutionTrajectory(path,state.x_m,state.y_m);
      const auto &p=path.points[projected.lower],&q=path.points[projected.lower+1];
      const double arc=p.s_m+projected.ratio*(q.s_m-p.s_m);
      reason=check_pose(state,arc);
      if(reason!=RejectReason::NONE) return false;
      if(arc>=end) {completed=true;return false;}
    }
    if(metrics) metrics->braking_mps+=std::max(0.,sample.before.speed_mps-sample.after.speed_mps);
    return true;
  });
  return reason!=RejectReason::NONE ? reason : completed ? RejectReason::NONE : RejectReason::GEOMETRY;
}

PriorLapMatch matchPriorLap(
    const std::vector<opponent_prediction::PositionObservation> &history,
    double length, const PassingPreparationConfig &config) {
  PriorLapMatch out;
  if (history.size()<6U || !(length>0.) || !(config.match_window_sec>0.)) return out;
  const auto &last=history.back();
  const auto recent=std::lower_bound(history.begin(),history.end(),last.stamp-config.match_window_sec,
      [](const auto &p,double t){return p.stamp<t;});
  if (recent==history.end() || history.end()-recent<3 ||
      last.stamp-recent->stamp<config.match_window_sec*.5) return out;
  const auto prior=opponent_prediction::selectContinuousPriorPass(
      history,recent->station-length,last.station-length);
  if (prior.size()<3U) return out;
  double squared=0., heading_squared=0., first_stamp=0., last_stamp=0.;
  std::size_t count=0;
  for(auto it=recent;it!=history.end();++it) {
    if (it!=recent && (!(it->stamp>(it-1)->stamp) || it->station<=(it-1)->station)) return out;
    const double station=it->station-length;
    auto upper=std::upper_bound(prior.begin(),prior.end(),station,
        [](double s,const auto &p){return s<p.station;});
    if(upper==prior.begin() || upper==prior.end()) return out;
    const auto &a=*std::prev(upper), &b=*upper;
    if (!(b.station>a.station) || !(b.stamp>a.stamp) || b.stamp-a.stamp>1.) return out;
    const double u=(station-a.station)/(b.station-a.station);
    const double dx=it->x-(a.x+u*(b.x-a.x)), dy=it->y-(a.y+u*(b.y-a.y));
    squared+=dx*dx+dy*dy;
    const double stamp=a.stamp+u*(b.stamp-a.stamp);
    if(count==0) first_stamp=stamp;
    last_stamp=stamp;
    if(it!=recent) {
      const double yaw=std::atan2(it->y-(it-1)->y,it->x-(it-1)->x);
      const double error=std::remainder(yaw-std::atan2(b.y-a.y,b.x-a.x),2.*M_PI);
      heading_squared+=error*error;
    }
    ++count;
  }
  out.observed_duration_sec=last.stamp-recent->stamp;
  if (!(last_stamp>first_stamp)) return out;
  out.position_error_m=std::sqrt(squared/count);
  out.heading_error_rad=std::sqrt(heading_squared/std::max<std::size_t>(1,count-1));
  out.pace_error_ratio=std::abs((last_stamp-first_stamp)/out.observed_duration_sec-1.);
  out.reason=out.position_error_m>config.maximum_position_error_m ||
      out.heading_error_rad>config.maximum_heading_error_rad ? "route_mismatch" :
      out.pace_error_ratio>config.maximum_pace_error_ratio ? "pace_mismatch" : "matched";
  out.usable=out.position_error_m<=config.maximum_position_error_m &&
      out.heading_error_rad<=config.maximum_heading_error_rad &&
      out.pace_error_ratio<=config.maximum_pace_error_ratio;
  return out;
}

bool samePreparationWindow(const PassingPreparationPlan &a,const PassingPreparationPlan &b) {
  return a.target_id==b.target_id && a.world==b.world && a.wall_lines==b.wall_lines &&
      ((a.id!=0 && a.id==b.id) || (std::abs(a.exit_station_m-b.exit_station_m)<1.5 &&
      std::max(a.entry_station_m,b.entry_station_m)<std::min(a.exit_station_m,b.exit_station_m)));
}

std::optional<PreparationPoint> preparationTarget(
    const PassingPreparationPlan &plan,double stamp) {
  const double t=stamp-plan.epoch_sec;
  if(plan.schedule.size()<2U || t<0. || t>plan.schedule.back().time_sec) return {};
  auto upper=std::upper_bound(plan.schedule.begin(),plan.schedule.end(),t,
      [](double value,const auto &p){return value<p.time_sec;});
  if(upper==plan.schedule.end()) return plan.schedule.back();
  if(upper==plan.schedule.begin()) return *upper;
  const auto &a=*std::prev(upper), &b=*upper;
  const double u=(t-a.time_sec)/(b.time_sec-a.time_sec);
  return PreparationPoint{t,a.station_m+u*(b.station_m-a.station_m),
      a.speed_mps+u*(b.speed_mps-a.speed_mps),a.acceleration_mps2+u*(b.acceleration_mps2-a.acceleration_mps2)};
}

bool preparationSearchDue(const PassingPreparationPlan &plan,double stamp,double delay) {
  // An opportunity is a request to evaluate the actual connection now.
  // The approximate longitudinal lateral-start time cannot certify geometry.
  (void)delay;
  return stamp>=plan.epoch_sec &&
      (plan.connection || stamp<=plan.epoch_sec+plan.targetTimeSec());
}

std::optional<double> preparationStation(const PassingPreparationPlan &plan,double x,double y) {
  if(!plan.world) return {};
  const auto measured=plan.world->projectStation(x,y);
  if(!measured) return {};
  if(!plan.front_merge || !plan.connection || plan.connection_stations_m.size()!=plan.connection->count)
    return plan.origin_station_m+plan.world->stationDifference(*measured,plan.origin_station_m);
  const auto projection=projectExecutionTrajectory(*plan.connection,x,y);
  if(!std::isfinite(projection.distance_squared)) return {};
  const auto i=projection.lower;const auto u=projection.ratio;
  const auto &a=plan.connection->points[i],&b=plan.connection->points[i+1];
  const auto on_path=plan.world->projectStation(a.x_m+u*(b.x_m-a.x_m),a.y_m+u*(b.y_m-a.y_m));
  if(!on_path) return {};
  return plan.connection_stations_m[i]+u*(plan.connection_stations_m[i+1]-plan.connection_stations_m[i])+
      plan.world->stationDifference(*measured,*on_path);
}

bool preparationConnectionCurrent(const PassingPreparationPlan &plan,
    const PlanRequest &r,const std::string &target) {
  if(!plan.connection || plan.side==0 || plan.target_id!=target ||
      plan.world!=r.world_reference || plan.wall_lines!=r.precomputed_wall_lines ||
      !r.world_reference) return false;
  const auto station=preparationStation(plan,r.ego.x_m,r.ego.y_m);
  return station && *station<plan.exit_station_m;
}

bool pastPreparationConnectionEnd(const PassingPreparationPlan &plan,const PlanRequest &r) {
  if(!plan.connection || !r.world_reference || plan.world!=r.world_reference) return false;
  const auto station=preparationStation(plan,r.ego.x_m,r.ego.y_m);
  return station && *station>=plan.connection_end_station_m;
}

void applyPassingPreparation(PlanRequest &r,
    const std::shared_ptr<const PassingPreparationPlan> &plan) {
  r.passing_preparation.reset();
  if(!plan || plan->world!=r.world_reference || plan->wall_lines!=r.precomputed_wall_lines) return;
  const auto station=preparationStation(*plan,r.ego.x_m,r.ego.y_m);
  if(!station) return;
  const double origin=*station;
  if(origin>=plan->exit_station_m ||
      (!plan->connection && r.stamp_sec>plan->epoch_sec+plan->targetTimeSec())) return;
  r.passing_preparation=plan;
  r.leader_passing_entry_m=std::max(0.,plan->entry_station_m-origin);
  r.leader_passing_exit_m=plan->exit_station_m-origin;
  r.leader_preparation_m=std::max(0.,plan->acceleration_start_station_m-origin);
  r.leader_pass_distance_m=plan->complete_pass ? std::max(0.,plan->pass_station_m-origin) : -1.;
  r.leader_pass_time_sec=plan->complete_pass ? plan->epoch_sec+plan->pass_time_sec-r.stamp_sec : -1.;
}

std::shared_ptr<const PassingPreparationPlan> makePassingPreparationGoal(
    const PlanRequest &r,const Config &c,double gentle,const std::string &target,
    const PriorLapMatch &match,std::uint64_t revision,
    const std::shared_ptr<const PassingPreparationPlan> &previous,
    PreparationDiagnostics *diagnostics) {
  PreparationDiagnostics local;
  auto &diag=diagnostics ? *diagnostics : local;
  diag={};
  if(!match.usable) {diag.reason=match.reason;return {};}
  if(target.empty()) {diag.reason="missing_target";return {};}
  if(!r.world_reference) {diag.reason="missing_world";return {};}
  if(!r.leader_passing_road) {diag.reason="missing_road";return {};}
  if(!r.leader_passing_prediction || r.leader_passing_prediction->points.empty()) {
    diag.reason="missing_prediction";return {};
  }
  if(r.leader_opportunity_index>=r.dynamic_obstacle_count) {diag.reason="missing_obstacle";return {};}
  if(!(gentle>0.)) {diag.reason="missing_lateral_model";return {};}
  const auto &road=*r.leader_passing_road;
  if(road.samples.size()<2U) {diag.reason="missing_road";return {};}
  const auto projected=r.world_reference->projectStation(r.ego.x_m,r.ego.y_m);
  if(!projected) {diag.reason="projection_failed";return {};}
  double origin=*projected;
  if(previous && previous->world==r.world_reference)
    origin=previous->origin_station_m+r.world_reference->stationDifference(*projected,previous->origin_station_m);
  auto leader=r.dynamic_obstacles[r.leader_opportunity_index];
  leader.prediction=r.leader_passing_prediction;
  if(leader.s_m<0.) {diag.reason="target_behind";return {};}
  diag.initial_body_gap_m=leader.s_m-2.*c.vehicle_half_length_m;
  const double shift=std::max(0.,2.*c.vehicle_half_width_m+c.clearance_target_m-std::abs(r.ego.d_m-leader.d_m));
  const double shift_time=std::sqrt(5.7735026919*shift/gentle)+c.steering_control_delay_sec+c.steering_time_constant_sec;
  const double horizon=std::min(15.,leader.prediction->points.back().time);
  diag.horizon_sec=horizon;
  std::shared_ptr<PassingPreparationPlan> best;
  std::size_t windows=0;
  const auto usable=[](const auto &p){return std::abs(p.curvature)<=.035 && p.speed>=3.;};
  for(std::size_t begin=0;begin<road.samples.size() && windows<2U;) {
    if(!usable(road.samples[begin])) {++begin;continue;}
    std::size_t end=begin;
    while(end+1<road.samples.size() && usable(road.samples[end+1])) ++end;
    const double entry=road.samples[begin].distance, exit=road.samples[end].distance;
    begin=end+1;
    if(exit-entry<8.) continue;
    ++windows;
    ++diag.windows;
    const double lateral_start=std::max(0.,entry-std::max(0.,r.ego.speed_mps)*shift_time);
    const std::array<double,3> starts{0.,lateral_start,entry};
    for(std::size_t option=0;option<starts.size();++option) {
      if(option && std::find(starts.begin(),starts.begin()+option,starts[option])!=starts.begin()+option) continue;
      ++diag.trials;
      auto plan=std::make_shared<PassingPreparationPlan>();
      plan->id=plan->revision=revision;plan->target_id=target;plan->match=match;
      plan->world=r.world_reference;plan->epoch_sec=r.stamp_sec;plan->origin_station_m=origin;
      plan->wall_lines=r.precomputed_wall_lines;
      plan->entry_station_m=origin+entry;plan->exit_station_m=origin+exit;
      plan->acceleration_start_station_m=origin+starts[option];
      plan->lateral_start_station_m=origin+lateral_start;
      double distance=0.,speed=std::max(0.,r.ego.speed_mps),acceleration=r.ego.acceleration_mps2;
      double lateral_time=-1.,acceleration_time=-1.;
      AwsimLongitudinalResponse plant;
      constexpr double dt=.05;
      bool passed=false,window_finished=false;
      for(double t=0.;t<=horizon+1e-9;t+=dt) {
        const auto index=passingRoadIndex(road,distance);
        diag.maximum_progress_m=std::max(diag.maximum_progress_m,distance);
        if(distance>exit) {++diag.window_exited;window_finished=true;break;}
        if(index>=road.samples.size()) {++diag.road_exhausted;break;}
        if(lateral_time<0. && distance>=lateral_start) lateral_time=t;
        if(acceleration_time<0. && distance>=starts[option]) acceleration_time=t;
        plan->schedule.push_back({t,origin+distance,speed,acceleration});
        const auto other=opponent_prediction::positionAt(leader,t);
        if(lateral_time>=0. && t>=lateral_time+shift_time && distance>=entry)
          diag.best_body_deficit_m=std::min(diag.best_body_deficit_m,
              other.s+2.*c.vehicle_half_length_m-distance);
        if(distance>=entry && plan->entry_speed_mps==0.) plan->entry_speed_mps=speed;
        if(lateral_time>=0. && t>=lateral_time+shift_time && distance>=entry &&
            distance>=other.s+2.*c.vehicle_half_length_m &&
            (!r.passing_point_areas || r.passing_point_areas->contains(origin+distance))) {
          plan->pass_station_m=origin+distance;plan->pass_time_sec=t;passed=true;break;
        }
        if(t+dt>horizon) {++diag.horizon_exhausted;break;}
        const auto next=opponent_prediction::positionAt(leader,t+dt);
        const double leader_speed=std::max(0.,(next.s-other.s)/dt);
        double target_speed=road.samples[index].speed;
        if(acceleration_time<0.) target_speed=std::min(target_speed,leader_speed);
        if(lateral_time<0. || t<lateral_time+shift_time) {
          const double remaining=lateral_time<0. ? shift_time : std::max(dt,lateral_time+shift_time-t);
          target_speed=std::min(target_speed,std::max(0.,leader_speed+
              (other.s-distance-2.*c.vehicle_half_length_m)/remaining));
        }
        const double resistance=c.awsim_vehicle_response_enabled && speed>0. ? .37+.03*speed : 0.;
        double requested=std::clamp(c.speed_proportional_gain*(target_speed-speed)+resistance,
            -c.maximum_deceleration_mps2,c.maximum_acceleration_mps2);
        const double steering=std::max(std::abs(std::atan(c.wheel_base_m*road.samples[index].curvature)),
            std::abs(r.ego.steering_rad)*std::exp(-t/std::max(.01,c.steering_time_constant_sec)));
        if(c.steering_demand_acceleration_hold_enabled && speed>=c.steering_acceleration_hold_minimum_speed_mps &&
            steering>=c.steering_acceleration_hold_minimum_tire_angle_rad)
          requested=std::min(requested,c.steering_acceleration_hold_maximum_acceleration_mps2);
        if(c.awsim_vehicle_response_enabled) acceleration=plant.acceleration(requested,speed,t);
        else acceleration+=(1.-std::exp(-dt/c.acceleration_time_constant_sec))*(requested-acceleration);
        // Each sample carries the feedforward for its outgoing interval,
        // including t=0 when the schedule is rebuilt from measured state.
        plan->schedule.back().acceleration_mps2=acceleration;
        const double next_speed=std::max(0.,speed+acceleration*dt);
        distance+=.5*(speed+next_speed)*dt;speed=next_speed;
      }
      if(lateral_time<0. || plan->schedule.empty() ||
          plan->schedule.back().time_sec<lateral_time+shift_time) ++diag.lateral_not_ready;
      if(acceleration_time<0. || lateral_time<0. || plan->schedule.size()<2U) continue;
      const auto &last=plan->schedule.back();
      const auto other=opponent_prediction::positionAt(leader,last.time_sec);
      const auto before=opponent_prediction::positionAt(leader,std::max(0.,last.time_sec-dt));
      plan->relative_gain_m=last.station_m-origin-(other.s-leader.s_m);
      plan->terminal_relative_speed_mps=last.speed_mps-(other.s-before.s)/dt;
      plan->complete_pass=passed;
      if(!passed) {
        if(!window_finished || last.station_m<origin+entry || last.time_sec<lateral_time+shift_time ||
            !(plan->relative_gain_m>0.) || !(plan->terminal_relative_speed_mps>0.)) continue;
        plan->pass_station_m=plan->pass_time_sec=-1.;
      }
      plan->lateral_start_time_sec=lateral_time;plan->acceleration_start_time_sec=acceleration_time;
      if(previous && samePreparationWindow(*plan,*previous)) {
        plan->id=previous->id;
        plan->entry_station_m=previous->entry_station_m;
      }
      const bool keep=previous && samePreparationWindow(*plan,*previous);
      const bool best_keep=best && previous && samePreparationWindow(*best,*previous);
      const bool better=best && (plan->complete_pass ? plan->pass_time_sec<best->pass_time_sec :
          plan->relative_gain_m>best->relative_gain_m);
      if(!best || (plan->complete_pass && !best->complete_pass) ||
          (plan->complete_pass==best->complete_pass &&
           ((keep && !best_keep) || (keep==best_keep && better)))) best=std::move(plan);
    }
  }
  diag.reason=best ? best->complete_pass ? "planned" : "prepared" : windows ? "no_improving_preparation" : "no_passing_window";
  return best;
}

namespace {
struct PreparationExtension {
  double recovery{0.0},pass_time{-1.},pass_station{-1.};
  double relative_gain{0.0},relative_speed{0.0};
  bool prepared{false};
};
PreparationExtension extendPreparation(const PlanRequest &r,const Config &c,const Evaluation &e,
    std::vector<PreparationPoint> *schedule=nullptr) {
  PreparationExtension result;
  if(!r.passing_preparation || !r.world_reference || !r.leader_passing_road ||
      !r.leader_passing_prediction || r.leader_opportunity_index>=r.dynamic_obstacle_count ||
      !e.valid || !e.predicted_rollout_count) return result;
  const auto &plan=*r.passing_preparation;
  const auto &terminal=e.predicted_rollout[e.predicted_rollout_count-1];
  const auto projected=r.world_reference->projectStation(terminal.x_m,terminal.y_m);
  const auto origin=r.world_reference->projectStation(r.ego.x_m,r.ego.y_m);
  if(!projected || !origin || plan.world!=r.world_reference) return result;
  const double request_origin=plan.origin_station_m+r.world_reference->stationDifference(*origin,plan.origin_station_m);
  double station=plan.origin_station_m+r.world_reference->stationDifference(*projected,plan.origin_station_m);
  auto leader=r.dynamic_obstacles[r.leader_opportunity_index];leader.prediction=r.leader_passing_prediction;
  const auto other_station=[&](double t) {return request_origin+opponent_prediction::positionAt(leader,t).s-r.ego.s_m;};
  const double initial_gap=other_station(terminal.time_sec)+2.*c.vehicle_half_length_m-station;
  const double request_gap=other_station(0.)+2.*c.vehicle_half_length_m-request_origin;
  double speed=std::max(0.,terminal.speed_mps),acceleration=0.;
  if(e.predicted_rollout_count>1U) {
    const auto &before=e.predicted_rollout[e.predicted_rollout_count-2];
    if(terminal.time_sec>before.time_sec) acceleration=(speed-before.speed_mps)/(terminal.time_sec-before.time_sec);
  }
  if(schedule) {
    schedule->clear();
    for(std::size_t i=0;i<e.predicted_rollout_count;++i) {
      const auto &state=e.predicted_rollout[i];
      const auto s=r.world_reference->projectStation(state.x_m,state.y_m);
      if(!s) return result;
      schedule->push_back({r.stamp_sec+state.time_sec-plan.epoch_sec,
          plan.origin_station_m+r.world_reference->stationDifference(*s,plan.origin_station_m),state.speed_mps,0.});
    }
  }
  AwsimLongitudinalResponse plant;
  constexpr double dt=.05;
  const double horizon=leader.prediction->points.back().time;
  for(double t=terminal.time_sec;t<=horizon+1e-9 && station<=plan.exit_station_m;t+=dt) {
    const double residual=other_station(t)+2.*c.vehicle_half_length_m-station;
    const double other_speed=t>0. ? (other_station(t)-other_station(std::max(0.,t-dt)))/std::min(dt,t) :
        (other_station(dt)-other_station(0.))/dt;
    result.relative_gain=request_gap-residual;
    result.relative_speed=speed-other_speed;
    result.prepared=station>=plan.entry_station_m && result.relative_gain>0. && result.relative_speed>0.;
    result.recovery=std::max(result.recovery,std::clamp((initial_gap-residual)/
        std::max(2.*c.vehicle_half_length_m,initial_gap),0.,1.));
    if(station>=plan.entry_station_m && residual<=0. &&
        (!r.passing_point_areas || r.passing_point_areas->contains(station))) {
      result.recovery=1.;result.pass_time=r.stamp_sec+t-plan.epoch_sec;result.pass_station=station;break;
    }
    const auto road_index=passingRoadIndex(*r.leader_passing_road,station-request_origin);
    if(road_index>=r.leader_passing_road->samples.size() || t+dt>horizon) break;
    const auto &road=r.leader_passing_road->samples[road_index];
    const auto target=preparationTarget(plan,r.stamp_sec+t);
    const double tau=c.approach_time_sec;
    const double road_acceleration=c.speed_proportional_gain*(road.speed-speed);
    const double desired=target ? std::min(road_acceleration,target->acceleration_mps2+
        2.*(target->speed_mps-speed)/tau+(target->station_m-station)/(tau*tau)) : road_acceleration;
    const double resistance=c.awsim_vehicle_response_enabled && speed>0. ? .37+.03*speed : 0.;
    const double requested=std::clamp(desired+resistance,-c.maximum_deceleration_mps2,c.maximum_acceleration_mps2);
    if(c.awsim_vehicle_response_enabled) acceleration=plant.acceleration(requested,speed,t-terminal.time_sec);
    else acceleration+=(1.-std::exp(-dt/c.acceleration_time_constant_sec))*(requested-acceleration);
    const double next_speed=std::max(0.,speed+acceleration*dt);
    station+=.5*(speed+next_speed)*dt;speed=next_speed;
    if(schedule && station<=plan.exit_station_m)
      schedule->push_back({r.stamp_sec+t+dt-plan.epoch_sec,station,speed,acceleration});
  }
  result.prepared=result.prepared && station>plan.exit_station_m;
  return result;
}
}  // namespace

std::shared_ptr<const PassingPreparationPlan> refinePassingPreparation(
    const PlanRequest &r,const Config &c,const Evaluation &e,std::uint64_t generation) {
  if(!r.passing_preparation || !e.valid || !e.predicted_rollout_count) return {};
  auto plan=std::make_shared<PassingPreparationPlan>(*r.passing_preparation);
  if(plan->front_merge) {
    // Retiming a held path cannot adopt a different, still-unbound goal.
    // Only bindPreparationConnection can assign its geometry and merge end.
    if(!plan->connection) return {};
    plan->geometry_generation=generation;
    plan->validated_until_sec=r.stamp_sec+e.predicted_rollout[e.predicted_rollout_count-1].time_sec;
    return plan;
  }
  const auto future=extendPreparation(r,c,e,&plan->schedule);
  if(future.pass_time<0. && !future.prepared) {
    if(!r.passing_preparation->connection) return {};
    // An expired longitudinal estimate does not invalidate a currently
    // certified connection. Do not claim its old predicted pass as current.
    plan->schedule.clear();
  }
  plan->geometry_generation=generation;
  plan->validated_until_sec=r.stamp_sec+e.predicted_rollout[e.predicted_rollout_count-1].time_sec;
  plan->pass_time_sec=future.pass_time;plan->pass_station_m=future.pass_station;
  plan->complete_pass=future.pass_time>=0.;
  plan->relative_gain_m=future.relative_gain;
  plan->terminal_relative_speed_mps=future.relative_speed;
  return plan;
}

std::shared_ptr<const PassingPreparationPlan> bindPreparationConnection(
    const PlanRequest &r,const Config &c,const Evaluation &e,
    const TemporaryReference &geometry,int side,std::uint64_t generation,bool extend_existing,
    RejectReason *rejection) {
  if(rejection) *rejection=e.valid ? RejectReason::GEOMETRY : e.reject_reason;
  if(!r.passing_preparation || !e.valid || !e.predicted_rollout_count ||
      side==0 || geometry.count<2U || !r.world_reference) return {};
  if(r.passing_preparation->prior_lap_guidance && r.passing_preparation->side!=0 &&
      side!=r.passing_preparation->side) return {};
  const auto refined=refinePassingPreparation(r,c,e,generation);
  auto plan=std::make_shared<PassingPreparationPlan>(
      refined ? *refined : *r.passing_preparation);
  const auto &old=*r.passing_preparation;
  if(extend_existing && old.connection && old.side==side) {
    // Replenishing a suffix must not move the original connection endpoint.
    plan->connection=old.connection;
    plan->connection_start_station_m=old.connection_start_station_m;
    plan->connection_end_station_m=old.connection_end_station_m;
    plan->connection_stations_m=old.connection_stations_m;
  } else {
    if(!geometry.wall_connection_end_index ||
        geometry.wall_connection_end_index>=geometry.count) return {};
    const auto &begin=geometry.points[0];
    const auto endpoint=plan->front_merge ? geometry.front_merge_end_index : geometry.wall_connection_end_index;
    if(!endpoint || endpoint>=geometry.count) return {};
    const auto &end=geometry.points[endpoint];
    const auto start_s=r.world_reference->projectStation(begin.x_m,begin.y_m);
    const auto end_s=r.world_reference->projectStation(end.x_m,end.y_m);
    if(!start_s || !end_s) return {};
    plan->connection_start_station_m=plan->origin_station_m+
        r.world_reference->stationDifference(*start_s,plan->origin_station_m);
    plan->connection_end_station_m=plan->connection_start_station_m+
        r.world_reference->stationDifference(*end_s,*start_s);
    if(plan->front_merge) {
      // Long attacks can exceed half a lap. Unwrap along the ordered path,
      // not the shortest signed difference between its two endpoints.
      double previous=*start_s;
      plan->connection_stations_m={plan->connection_start_station_m};
      for(std::size_t i=1;i<geometry.count;++i) {
        const auto station=r.world_reference->projectStation(geometry.points[i].x_m,geometry.points[i].y_m);
        if(!station) return {};
        plan->connection_stations_m.push_back(plan->connection_stations_m.back()+
            r.world_reference->stationDifference(*station,previous));
        previous=*station;
      }
      plan->connection_end_station_m=plan->connection_stations_m[endpoint];
    }
    plan->connection=std::make_shared<const TemporaryReference>(geometry);
  }
  // A connection ending after the opportunity expires would be cancelled
  // before reaching its own wall line. It is not a feasible preparation.
  if(!(plan->connection_end_station_m<plan->exit_station_m)) return {};
  if(!geometry.wall_connection_end_index || geometry.wall_connection_end_index>=geometry.count) return {};
  if(r.path_constraint_validator) {
    auto prefix=geometry;
    prefix.count=(plan->front_merge ? geometry.front_merge_end_index : geometry.wall_connection_end_index)+1U;
    // The local rollout may stop before the join. Certify the whole actual
    // connection with the same swept wall geometry check before adopting it.
    const auto reason=r.path_constraint_validator(prefix);
    if(reason!=RejectReason::NONE) {
      if(rejection) *rejection=reason;
      return {};
    }
  }
  plan->side=side;plan->geometry_generation=generation;
  plan->lateral_start_station_m=plan->connection_start_station_m;
  plan->lateral_start_time_sec=r.stamp_sec-plan->epoch_sec;
  plan->validated_until_sec=r.stamp_sec+e.predicted_rollout[e.predicted_rollout_count-1].time_sec;
  if(rejection) *rejection=RejectReason::NONE;
  return plan;
}

double preparationProgress(const PlanRequest &r,const Config &c,const Evaluation &e) {
  if(!r.passing_preparation || !r.world_reference || !e.valid || !e.predicted_rollout_count) return 0.;
  const auto &plan=*r.passing_preparation;
  if(plan.world!=r.world_reference) return 0.;
  const auto &state=e.predicted_rollout[e.predicted_rollout_count-1];
  const auto position=r.world_reference->projectStation(state.x_m,state.y_m);
  const auto target=preparationTarget(plan,r.stamp_sec+state.time_sec);
  if(!position || !target) return 0.;
  const double station=plan.origin_station_m+r.world_reference->stationDifference(*position,plan.origin_station_m);
  const double distance_error=std::abs(target->station_m-station);
  const double speed_error=std::abs(target->speed_mps-state.speed_mps);
  const double distance_scale=std::max(2.*c.vehicle_half_length_m,target->speed_mps*c.approach_time_sec);
  const double readiness=std::exp(-distance_error/distance_scale-speed_error/std::max(1.,target->speed_mps));
  const auto pose=r.world_reference->pose(*position,0.);
  if(!pose || r.leader_opportunity_index>=r.dynamic_obstacle_count) return 0.;
  const auto other=opponent_prediction::positionAt(r.dynamic_obstacles[r.leader_opportunity_index],state.time_sec);
  const double d=-(state.x_m-(*pose)[0])*std::sin((*pose)[2])+(state.y_m-(*pose)[1])*std::cos((*pose)[2]);
  const double lateral=std::clamp(std::abs(d-other.d)/(2.*c.vehicle_half_width_m+c.clearance_target_m),0.,1.);
  const double until=std::max(0.,plan.lateral_start_station_m-station);
  const double lateral_span=plan.entry_station_m-plan.lateral_start_station_m;
  const double required_lateral=lateral_span>0. ?
      std::clamp((station-plan.lateral_start_station_m)/lateral_span,0.,1.) :
      station>=plan.entry_station_m ? 1. : 0.;
  const double lateral_readiness=1.-required_lateral*(1.-lateral);
  return readiness*lateral_readiness*extendPreparation(r,c,e).recovery*
      std::exp(-until/std::max(1.,target->speed_mps*state.time_sec));
}

}  // namespace reference_space_mppi_planner::mppi
