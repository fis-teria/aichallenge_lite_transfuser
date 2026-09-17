#include "reference_space_mppi_planner/ot_lane_entry_planner.hpp"
#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include <algorithm>
#include <cmath>

namespace reference_space_mppi_planner::mppi {
namespace {
double plane(const OtLaneEntryConfig &e,double x,double y) {
  const double dx=e.exit_x_m-e.entry_x_m,dy=e.exit_y_m-e.entry_y_m;
  return ((x-e.entry_x_m)*dx+(y-e.entry_y_m)*dy)/std::hypot(dx,dy);
}
}
std::shared_ptr<const OtLaneEntryPlan> makeOtLaneEntryPlan(
    const PlanRequest &r,const Config &c,const OtLaneEntryConfig &e) {
  if(!e.enabled || !r.world_reference || !(e.target_speed_mps>0.) ||
      !(std::hypot(e.exit_x_m-e.entry_x_m,e.exit_y_m-e.entry_y_m)>1.)) return {};
  const auto origin=r.world_reference->projectStation(r.ego.x_m,r.ego.y_m);
  const auto projected=r.world_reference->projectStation(e.entry_x_m,e.entry_y_m);
  if(!origin || !projected) return {};
  // Find the physical entrance plane on this rank-dependent road Reference.
  double lo=*projected-10.,hi=*projected+10.;
  const auto a=r.world_reference->pose(lo,0.),b=r.world_reference->pose(hi,0.);
  if(!a || !b || plane(e,(*a)[0],(*a)[1])>=0. || plane(e,(*b)[0],(*b)[1])<=0.) return {};
  for(int i=0;i<24;++i) {
    const double mid=.5*(lo+hi);const auto p=r.world_reference->pose(mid,0.);
    if(!p) return {};
    if(plane(e,(*p)[0],(*p)[1])<0.) lo=mid;else hi=mid;
  }
  const double distance=r.world_reference->stationDifference(.5*(lo+hi),*origin);
  if(!(distance>0.) || distance>100. || plane(e,r.ego.x_m,r.ego.y_m)>=0.) return {};
  double cruise=0.;
  for(std::size_t i=0;i<r.base_reference_count;++i)
    cruise=std::max(cruise,r.base_reference[i].speed_mps);
  auto out=std::make_shared<OtLaneEntryPlan>();
  out->config=e;out->distance_m=distance;
  auto &p=out->preparation;
  p.world=r.world_reference;p.epoch_sec=r.stamp_sec;p.origin_station_m=*origin;
  p.entry_station_m=*origin+distance;
  p.acceleration_start_time_sec=p.acceleration_start_station_m=-1.;
  p.complete_pass=false;
  double s=0.,v=std::max(0.,r.ego.speed_mps),acc=r.ego.acceleration_mps2;
  AwsimLongitudinalResponse plant;
  constexpr double dt=.05;
  for(double t=0.;t<=20.+1e-9;t+=dt) {
    p.schedule.push_back({t,*origin+s,v,acc});
    if(s>=distance) {
      auto &last=p.schedule.back();const auto &prev=p.schedule[p.schedule.size()-2];
      const double u=(p.entry_station_m-prev.station_m)/(last.station_m-prev.station_m);
      last={prev.time_sec+u*(last.time_sec-prev.time_sec),p.entry_station_m,
          prev.speed_mps+u*(last.speed_mps-prev.speed_mps),acc};
      out->reached_entry=true;out->predicted_speed_mps=last.speed_mps;
      out->reachable_free_road=last.speed_mps>=e.target_speed_mps;
      p.entry_speed_mps=last.speed_mps;
      return out;
    }
    const auto before=r.world_reference->pose(*origin+s-2.,0.);
    const auto after=r.world_reference->pose(*origin+s+2.,0.);
    if(!before || !after) break;
    const double curve=std::abs(std::remainder((*after)[2]-(*before)[2],2.*M_PI))/4.;
    const double target=std::min(cruise,std::sqrt(c.maximum_lateral_acceleration_mps2/std::max(1e-9,curve)));
    const double resistance=c.awsim_vehicle_response_enabled && v>0. ? .37+.03*v : 0.;
    double requested=std::clamp(c.speed_proportional_gain*(target-v)+resistance,
        -c.maximum_deceleration_mps2,c.maximum_acceleration_mps2);
    const double steering=std::max(std::abs(std::atan(c.wheel_base_m*curve)),
        std::abs(r.ego.steering_rad)*std::exp(-t/std::max(.01,c.steering_time_constant_sec)));
    if(c.steering_demand_acceleration_hold_enabled && v>=c.steering_acceleration_hold_minimum_speed_mps &&
        steering>=c.steering_acceleration_hold_minimum_tire_angle_rad)
      requested=std::min(requested,c.steering_acceleration_hold_maximum_acceleration_mps2);
    if(c.awsim_vehicle_response_enabled) acc=plant.acceleration(requested,v,t);
    else acc+=(1.-std::exp(-dt/c.acceleration_time_constant_sec))*(requested-acc);
    p.schedule.back().acceleration_mps2=acc;
    if(p.acceleration_start_time_sec<0. && acc>0.) {
      p.acceleration_start_time_sec=t;p.acceleration_start_station_m=*origin+s;
    }
    const double next=std::max(0.,v+acc*dt);s+=.5*(v+next)*dt;v=next;
  }
  out->predicted_speed_mps=v;
  return out;
}

double otLaneEntryCost(const PlanRequest &r,const Evaluation &evaluation) {
  if(!r.ot_lane_entry || !evaluation.predicted_rollout_count ||
      r.ot_lane_entry->preparation.world!=r.world_reference) return 0.;
  const auto &p=*r.ot_lane_entry;
  double previous_plane=plane(p.config,r.ego.x_m,r.ego.y_m),previous_speed=r.ego.speed_mps;
  double speed=evaluation.predicted_rollout[evaluation.predicted_rollout_count-1].speed_mps;
  double target=p.config.target_speed_mps;
  bool crossed=false;
  for(std::size_t i=0;i<evaluation.predicted_rollout_count;++i) {
    const auto &state=evaluation.predicted_rollout[i];
    const double current=plane(p.config,state.x_m,state.y_m);
    if(previous_plane<0. && current>=0.) {
      const double u=-previous_plane/(current-previous_plane);
      speed=previous_speed+u*(state.speed_mps-previous_speed);crossed=true;break;
    }
    previous_plane=current;previous_speed=state.speed_mps;
  }
  if(!crossed) {
    const auto &last=evaluation.predicted_rollout[evaluation.predicted_rollout_count-1];
    const auto goal=preparationTarget(p.preparation,r.stamp_sec+last.time_sec);
    if(goal) target=std::min(target,goal->speed_mps);
  }
  const double deficit=std::max(0.,target-speed);
  return p.config.cost_weight*deficit*deficit;
}
} // namespace reference_space_mppi_planner::mppi
