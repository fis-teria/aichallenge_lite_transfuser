#pragma once
#include "reference_space_mppi_planner/reference_space_mppi.hpp"

namespace reference_space_mppi_planner::mppi {
struct LeaderPassingOpportunity {
  double entry{-1.}, exit{-1.}, preparation{-1.}, pass_time{-1.};
  double pass_distance{-1.};
};

inline PassingRoadProfile passingRoadProfile(
    const std::vector<PassingRoadSample> &road, const Config &config) {
  PassingRoadProfile profile{road};
  for (auto &p : profile.samples) {
    p.speed = std::min(p.speed, std::sqrt(config.maximum_lateral_acceleration_mps2 /
        std::max(1e-6, std::abs(p.curvature))));
  }
  for (std::size_t i=profile.samples.size(); i>1U; --i) {
    const auto &next = profile.samples[i-1];
    auto &previous = profile.samples[i-2];
    previous.speed = std::min(previous.speed, std::sqrt(next.speed*next.speed +
        2.*config.maximum_deceleration_mps2*(next.distance-previous.distance)));
  }
  return profile;
}

inline std::size_t passingRoadIndex(const PassingRoadProfile &road, double distance) {
  const auto it = std::upper_bound(road.samples.begin(),road.samples.end(),distance,
      [](double s,const auto &p){return s<p.distance;});
  if (it==road.samples.end()) return road.samples.size();
  return it==road.samples.begin() ? 0U : static_cast<std::size_t>(it-road.samples.begin()-1);
}

inline void advancePassingRoad(double target, double dt, const Config &config,
                               double &distance, double &speed) {
  const double next = std::clamp(target,
      std::max(0.,speed-config.maximum_deceleration_mps2*dt),
      speed+config.maximum_acceleration_mps2*dt);
  distance += .5*(speed+next)*dt;
  speed = next;
}

// Extend only the terminal CMA state. The same spatial speed/braking advice
// used to select the opportunity applies here, including intervening curves.
// This remains a soft value, not an execution or collision certificate.
inline double leaderPassingRecovery(const PassingRoadProfile &road,
    const DynamicObstacle &leader, double distance, double speed, double elapsed,
    double entry, double exit, const Config &config) {
  if (!leader.prediction || road.samples.size()<2U || !(exit>entry) || distance>=exit)
    return 0.;
  const double initial_gap = opponent_prediction::positionAt(leader,elapsed).s-distance;
  const double residual = initial_gap+2.*config.vehicle_half_length_m;
  if (residual<=0.) return 1.;
  double recovery = 0.;
  const double horizon = leader.prediction->points.back().time;
  while (elapsed<horizon && distance<exit) {
    const auto i=passingRoadIndex(road,distance);
    if (i>=road.samples.size()) break;
    const double dt=std::min(.05,horizon-elapsed);
    if (!(dt>1e-9)) break;
    const double previous=distance;
    advancePassingRoad(road.samples[i].speed,dt,config,distance,speed);
    double sample_time=elapsed+dt, sample_distance=distance;
    if (distance>exit && distance>previous) {
      sample_time=elapsed+dt*(exit-previous)/(distance-previous);
      sample_distance=exit;
    }
    if (sample_distance>=entry) {
      const double gap=opponent_prediction::positionAt(leader,sample_time).s-sample_distance;
      recovery=std::max(recovery,(initial_gap-gap)/std::max(2.*config.vehicle_half_length_m,residual));
    }
    elapsed+=dt;
  }
  return std::clamp(recovery,0.,1.);
}

// Coarse long-range guidance, not a collision/trackability certificate.
// Integrate speed advice with the same longitudinal acceleration bound and
// reserve a gentle quintic shift plus steering response before body overlap.
inline LeaderPassingOpportunity selectLeaderPassingOpportunity(
    const PassingRoadProfile &profile, const DynamicObstacle &leader,
    const EgoState &ego, const Config &config, double gentle_acceleration) {
  LeaderPassingOpportunity result;
  const auto &road=profile.samples;
  if (!leader.prediction || road.size()<2U || !(gentle_acceleration>0.) ||
      leader.s_m < 0. || leader.s_m > road.back().distance) return result;
  const double shift = std::max(0.,2.*config.vehicle_half_width_m+
      config.clearance_target_m-std::abs(ego.d_m-leader.d_m));
  const double preparation_time = std::sqrt(5.7735026919*shift/gentle_acceleration)+
      config.steering_control_delay_sec+config.steering_time_constant_sec;
  double distance=0., speed=std::max(0.,ego.speed_mps), preparation_distance=0.;
  constexpr double dt=.05;
  for (double elapsed=dt; elapsed<=leader.prediction->points.back().time; elapsed+=dt) {
    const auto index=passingRoadIndex(profile,distance);
    if (index>=road.size()) break;
    const auto other=opponent_prediction::positionAt(leader,elapsed);
    double target=road[index].speed;
    if (elapsed<preparation_time) {
      // During preparation use a conservative speed-matching envelope. This
      // affects only opportunity estimates, never an actuator command.
      const double before=opponent_prediction::positionAt(leader,elapsed-dt).s;
      const double leader_speed=(other.s-before)/dt;
      const double gap=other.s-distance-2.*config.vehicle_half_length_m;
      target=std::min(target,std::max(0.,leader_speed+gap/
          std::max(dt,preparation_time-elapsed)));
    }
    advancePassingRoad(target,dt,config,distance,speed);
    if (elapsed<=preparation_time) preparation_distance=distance;
    if (elapsed<preparation_time || distance<other.s+2.*config.vehicle_half_length_m ||
        std::abs(road[index].curvature)>.035 || road[index].speed<3.) continue;
    std::size_t begin=index,end=index;
    while (begin>0U && std::abs(road[begin-1].curvature)<=.035 && road[begin-1].speed>=3.) --begin;
    while (end+1U<road.size() && std::abs(road[end+1].curvature)<=.035 && road[end+1].speed>=3.) ++end;
    if (road[end].distance-road[begin].distance<8.) continue;
    return {road[begin].distance,road[end].distance,
        std::max(0.,road[begin].distance-preparation_distance),elapsed,distance};
  }
  return result;
}

inline LeaderPassingOpportunity selectLeaderPassingOpportunity(
    const std::vector<PassingRoadSample> &road, const DynamicObstacle &leader,
    const EgoState &ego, const Config &config, double gentle_acceleration) {
  return selectLeaderPassingOpportunity(passingRoadProfile(road,config),leader,
      ego,config,gentle_acceleration);
}
} // namespace reference_space_mppi_planner::mppi
