// Controlled A/B experiment, not a replay of an AWSIM or CMA state snapshot.
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace reference_space_mppi_planner::mppi;

int main() {
  std::cout << "shape,speed,model,valid,terminal_error,progress,cost,end_x,end_y\n";
  for (int shape=0; shape<3; ++shape) for (double speed : {1.,6.,10.}) {
    Config c;
    c.enabled=true; c.shadow_only=false; c.collision_only_rejection=true;
    c.horizon_steps=60; c.cost_terminal_lateral_weight=2;
    c.lookahead_gain=.2; c.lookahead_min_distance_m=2;
    c.curvature_lookahead_min_distance_m=2; c.actual_lookahead_distance_blend=1;
    c.continuous_preview_interpolation_enabled=true;
    c.speed_proportional_gain=3; c.maximum_acceleration_mps2=2;
    PlanRequest r;
    r.valid=true; r.side=1; r.phase=Phase::OVERTAKE; r.generation=1;
    r.semantic_key=42; r.stamp_sec=1; r.ego.speed_mps=speed;
    r.sample_staged_speed=true; r.complete_maneuver=false;
    r.nominal={0,4,4,4,1,0,1};
    r.bounds.minimum={0,4,4,4,.1,0,1}; r.bounds.maximum=r.nominal;
    TemporaryReference path;
    path.count=r.base_reference_count=201;
    for(std::size_t i=0; i<path.count; ++i) {
      const double s=.2*i;
      auto &b=r.base_reference[i]; auto &p=path.points[i];
      b.s_m=p.s_m=s; b.speed_mps=p.speed_mps=speed;
      b.minimum_d_m=-10; b.maximum_d_m=10;
      b.x_m=p.x_m=s;
      if(shape==1) {
        p.y_m=p.d_m=1.5*ReferenceSpaceMppiPlanner::quinticBlend(std::clamp(s/12.,0.,1.));
      } else if(shape==2) {
        b.x_m=p.x_m=20*std::sin(s/20);
        b.y_m=p.y_m=20*(1-std::cos(s/20));
        b.yaw_rad=p.yaw_rad=s/20; b.curvature_1pm=p.curvature_1pm=.05;
      }
    }
    if(shape==1) {
      for(std::size_t i=1; i+1<path.count; ++i) {
        const double dy=(path.points[i+1].y_m-path.points[i-1].y_m)/.4;
        const double ddy=(path.points[i+1].y_m-2*path.points[i].y_m+path.points[i-1].y_m)/.04;
        path.points[i].yaw_rad=std::atan(dy);
        path.points[i].curvature_1pm=ddy/std::pow(1+dy*dy,1.5);
      }
    }
    Evaluation baseline;
    for(int mode=0; mode<4; ++mode) {
      c.compare_cma_delay_compensation=(mode&1)!=0;
      c.compare_cma_preview_feedforward=(mode&2)!=0;
      ReferenceSpaceMppiPlanner planner(c);
      if(planner.validateConfig()) throw std::runtime_error(planner.validateConfig());
      const auto e=planner.validateReference(path,r);
      if(!e.valid || e.predicted_rollout_count!=60) throw std::runtime_error("incomplete comparison rollout");
      if(mode==0) baseline=e;
      // Delay compensation changes controller geometry, not physical pose.
      if(std::abs(e.predicted_rollout[0].x_m-baseline.predicted_rollout[0].x_m)>1e-12 ||
         std::abs(e.predicted_rollout[0].y_m-baseline.predicted_rollout[0].y_m)>1e-12)
        throw std::runtime_error("controller prediction teleported physical state");
      const auto &last=e.predicted_rollout[e.predicted_rollout_count-1];
      const auto &old=baseline.predicted_rollout[baseline.predicted_rollout_count-1];
      if(shape==0 && std::hypot(last.x_m-old.x_m,last.y_m-old.y_m)>1e-10)
        throw std::runtime_error("straight rollout changed");
      if(shape==0 && std::abs(e.cost-baseline.cost)>1e-10)
        throw std::runtime_error("controller preview contaminated physical tracking cost");
      std::cout << shape << ',' << speed << ',' << mode << ',' << e.valid << ','
                << std::abs(e.terminal_d_m-e.terminal_reference_d_m) << ','
                << e.progress_m << ',' << e.cost << ',' << last.x_m << ',' << last.y_m << '\n';
    }
  }
}
