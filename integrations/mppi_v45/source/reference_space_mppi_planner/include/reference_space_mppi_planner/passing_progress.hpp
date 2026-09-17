#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/leader_passing_opportunity.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"

namespace reference_space_mppi_planner::mppi {

struct PassingProgress {
  double progress{0.0};
  double opportunity{0.0};
};

// Soft ranking evidence from the actual CMA rollout. No designated target,
// ideal-path clearance, additional collision gate, or extrapolated certificate.
inline PassingProgress passingProgress(const PlanRequest &r, const Config &c,
                                      const Evaluation &e) {
  PassingProgress out;
  if (!e.valid || !r.world_reference || !e.predicted_rollout_count ||
      (c.cost_passing_progress_weight == 0.0 &&
       c.cost_passing_opportunity_weight == 0.0)) return out;
  const auto &world = *r.world_reference;
  const auto origin = world.projectStation(r.ego.x_m, r.ego.y_m);
  if (!origin) return out;
  const double duration = e.predicted_rollout[e.predicted_rollout_count-1].time_sec;
  if (!(duration > 0.0)) return out;
  std::array<double, kMaximumDynamicObstacles> initial_gap{}, relevance{};
  double total_relevance = 0.0;
  for (std::size_t j=0; j<r.dynamic_obstacle_count; ++j) {
    const auto p = opponent_prediction::positionAt(r.dynamic_obstacles[j], 0.0);
    const double gap = world.stationDifference(p.global_s, *origin);
    initial_gap[j] = gap;
    if (gap > -2*c.vehicle_half_length_m && gap < 40.0) {
      relevance[j] = std::clamp((gap+2*c.vehicle_half_length_m) /
          (2*c.vehicle_half_length_m), 0.0, 1.0) * (1.0-std::max(0.0,gap)/40.0);
      total_relevance += relevance[j];
    }
  }
  if (total_relevance == 0.0) return out;
  const bool leader_window = r.leader_passing_entry_m>=0. &&
      r.leader_passing_exit_m>r.leader_passing_entry_m;
  double leader_completion = -1.;
  if (!r.passing_preparation && leader_window && r.leader_passing_road &&
      r.leader_opportunity_index<r.dynamic_obstacle_count) {
    const auto &terminal=e.predicted_rollout[e.predicted_rollout_count-1];
    const auto station=world.projectStation(terminal.x_m,terminal.y_m);
    if (!station) return {};
    const auto pose=world.pose(*station,0.);
    if (!pose) return {};
    auto guidance=r.dynamic_obstacles[r.leader_opportunity_index];
    if (r.leader_passing_prediction) guidance.prediction=r.leader_passing_prediction;
    leader_completion=leaderPassingRecovery(*r.leader_passing_road,guidance,
        world.stationDifference(*station,*origin),
        std::max(0.,terminal.speed_mps*std::cos(terminal.yaw_rad-(*pose)[2])),
        terminal.time_sec,r.leader_passing_entry_m,r.leader_passing_exit_m,c);
  }
  double previous_time = 0.0;
  std::array<bool,kMaximumDynamicObstacles> entry_seen{};
  for (std::size_t i=0; i<e.predicted_rollout_count; ++i) {
    const auto &state = e.predicted_rollout[i];
    const auto station = world.projectStation(state.x_m,state.y_m);
    if (!station) return {};
    const auto pose = world.pose(*station,0.0);
    if (!pose) return {};
    const double yaw = (*pose)[2];
    const double d = -(state.x_m-(*pose)[0])*std::sin(yaw) +
                      (state.y_m-(*pose)[1])*std::cos(yaw);
    const double ego_yaw = state.yaw_rad-yaw;
    const double ego_half_d = c.vehicle_half_width_m*std::abs(std::cos(ego_yaw)) +
                             c.vehicle_half_length_m*std::abs(std::sin(ego_yaw));
    const double ego_half_s = c.vehicle_half_length_m*std::abs(std::cos(ego_yaw)) +
                             c.vehicle_half_width_m*std::abs(std::sin(ego_yaw));
    const double dt = state.time_sec-previous_time;
    previous_time = state.time_sec;
    const double advance = world.stationDifference(*station,*origin);
    for (std::size_t j=0; j<r.dynamic_obstacle_count; ++j) {
      if (relevance[j] == 0.0) continue;
      // A prediction without a feasible special window must not suppress the
      // ordinary road opportunity that the same candidate had before it.
      const bool leader = leader_window && j == r.leader_opportunity_index;
      const double entry = leader ? r.leader_passing_entry_m : r.passing_entry_m;
      const double exit = leader ? r.leader_passing_exit_m : r.passing_exit_m;
      const double preparation = leader ? r.leader_preparation_m : entry;
      const bool entry_sample = !entry_seen[j] && entry>=0. && advance>=entry;
      entry_seen[j] = entry_seen[j] || entry_sample;
      const auto &obstacle = r.dynamic_obstacles[j];
      const auto p = opponent_prediction::positionAt(obstacle,state.time_sec);
      const auto before = opponent_prediction::positionAt(obstacle,
          std::max(0.0,state.time_sec-c.dt_sec));
      const double opponent_speed = world.stationDifference(p.global_s,before.global_s) /
          std::min(c.dt_sec,state.time_sec);
      const double gap = world.stationDifference(p.global_s,*station);
      const double half_d = c.vehicle_half_width_m*std::abs(std::cos(p.relative_yaw)) +
                            c.vehicle_half_length_m*std::abs(std::sin(p.relative_yaw));
      const double half_s = c.vehicle_half_length_m*std::abs(std::cos(p.relative_yaw)) +
                            c.vehicle_half_width_m*std::abs(std::sin(p.relative_yaw));
      const double width = ego_half_d+half_d+c.clearance_target_m;
      const double separation = std::clamp(std::abs(d-p.d)/width,0.0,1.0);
      const double ready = separation*separation*(3.0-2.0*separation);
      const double closing = state.speed_mps*std::cos(ego_yaw)-opponent_speed;
      const double speed_scale = std::max(1.0,r.passing_speed_mps);
      const double progress = std::clamp((initial_gap[j]-gap) /
          std::max(2*c.vehicle_half_length_m,initial_gap[j]+2*c.vehicle_half_length_m),-1.0,1.0);
      const double passed = std::clamp((-gap)/(ego_half_s+half_s),0.0,1.0);
      const bool has_opportunity = entry >= 0.0 && exit > entry;
      const double remaining = has_opportunity ?
          std::max(0.0,exit-advance) : 0.0;
      const double usable_time = std::min(6.0,remaining/speed_scale);
      const double residual = std::max(0.0,gap+ego_half_s+half_s);
      double completion = std::clamp((std::max(0.0,closing)*usable_time) /
          std::max(2*c.vehicle_half_length_m,residual),0.0,1.0);
      if (leader && has_opportunity && leader_completion>=0. && remaining>0.) {
        completion=leader_completion;
      } else if (!r.passing_preparation && leader && has_opportunity && obstacle.prediction && remaining>0.) {
        // Extend from this candidate's achieved CMA speed to the selected
        // window, using the acceleration bound and the leader's recorded
        // segment times. This is a soft terminal value, never clearance.
        const double v=std::clamp(state.speed_mps*std::cos(ego_yaw),0.,speed_scale);
        const double a=std::max(1e-6,c.maximum_acceleration_mps2);
        const double ramp=(speed_scale-v)/a;
        const double ramp_distance=.5*(v+speed_scale)*ramp;
        const double exit_time=remaining<=ramp_distance ?
            (std::sqrt(v*v+2.*a*remaining)-v)/a :
            ramp+(remaining-ramp_distance)/speed_scale;
        const double future_time=std::min(exit_time,std::max(0.,
            obstacle.prediction->points.back().time-state.time_sec));
        const double accelerating=std::min(ramp,future_time);
        const double future_advance=v*accelerating+.5*a*accelerating*accelerating+
            speed_scale*std::max(0.,future_time-ramp);
        const auto future=opponent_prediction::positionAt(obstacle,state.time_sec+future_time);
        const double recovery=future_advance-(future.global_s-p.global_s);
        completion=std::clamp(recovery/std::max(2*c.vehicle_half_length_m,residual),0.,1.);
      }
      // Readiness is useful in proportion to the relative distance the
      // achieved speed can recover before the straight ends. Absolute width
      // alone rewarded wandering around a distant, speed-matched opponent.
      const double useful_readiness = std::max(passed,ready*completion);
      // Early readiness earns more time integral; extra lateral excursion
      // beyond oriented body clearance earns nothing. Returning after a pass
      // retains the completed progress value.
      out.progress += relevance[j]*dt/duration*(progress +
          useful_readiness*(1.0+std::clamp(closing/speed_scale,-1.0,1.0)));
      if (!has_opportunity || (leader && r.passing_preparation)) continue;
      const double until_entry = std::max(0.0,preparation-advance);
      // Terminal value uses remaining distance and achieved relative speed;
      // readiness at entry is also scored when entry lies inside the rollout.
      const double opportunity = (remaining>0.0 ? 1.0 : 0.0) *
          std::exp(-until_entry/std::max(1.0,speed_scale*duration)) *
          useful_readiness;
      if (entry_sample) out.opportunity += .5*relevance[j]*opportunity;
      if (i+1==e.predicted_rollout_count)
        out.opportunity += relevance[j]*opportunity;
    }
  }
  out.progress /= std::max(1.0,total_relevance);
  out.opportunity /= std::max(1.0,total_relevance);
  if(r.passing_preparation) out.opportunity+=preparationProgress(r,c,e);
  return out;
}

} // namespace reference_space_mppi_planner::mppi
