#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>

using namespace reference_space_mppi_planner::mppi;
struct Uncached {
  const ReferenceSpaceMppiPlanner &planner;
  const Config &config() const { return planner.config(); }
  Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &r) const {
    return planner.evaluateExecutionReference(p,r);
  }
};
void check(bool ok) { if(!ok) throw std::runtime_error("cached/uncached mismatch"); }
int main() {
  Config c; c.enabled=true;c.shadow_only=false;c.horizon_steps=60;
  ReferenceSpaceMppiPlanner planner(c);
  PlanRequest r; r.valid=true;r.side=1;r.phase=Phase::OVERTAKE;
  r.sample_staged_speed=true;r.ego.speed_mps=1.;r.base_reference_count=101;
  r.nominal={0,4,4,4,1,0,1};r.bounds.minimum={0,4,4,4,.1,0,1};
  r.wall_line_start_s_m=2.;r.wall_line_end_s_m=20.;
  r.curvature_evaluation_distance_m=12.3;
  r.bounds.maximum={0,4,4,4,1,0,1};r.preferred_matching_speed_mps=.833;
  TemporaryReference path;path.count=101;
  for(std::size_t i=0;i<101;++i) {
    auto &b=r.base_reference[i];b.s_m=b.x_m=.4*i;b.speed_mps=8.;
    b.minimum_d_m=-10.;b.maximum_d_m=10.;b.active_d_valid=true;
    auto &p=path.points[i];p.x_m=p.s_m=p.source_s_m=.4*i;
    p.speed_mps=p.uncapped_speed_mps=1.;
  }
  double cached_ms=0.,uncached_ms=0.;
  for(int scenario=0;scenario<4;++scenario) {
    r.static_obstacle_count=scenario==1?1:0;
    r.static_obstacles[0].minimum_s_m=4.;r.static_obstacles[0].maximum_s_m=5.;
    r.static_obstacles[0].minimum_d_m=-1.;r.static_obstacles[0].maximum_d_m=1.;
    for(std::size_t i=0;i<path.count;++i) {
      path.points[i].y_m=scenario==2?.002*i*i:0.;
      path.points[i].curvature_1pm=scenario==2?.01+1e-4*i:0.;
    }
    r.dynamic_obstacle_count=scenario==3?1:0;
    r.dynamic_obstacles[0].s_m=8.;r.dynamic_obstacles[0].d_m=-2.;
    r.dynamic_obstacles[0].lateral_speed_mps=1.;
    const auto incumbent=planner.evaluateExecutionReference(path,r);
    const auto prepared=planner.makeExecutionSpeedEvaluator(path,r);
    for(double speed:{0.,.2,.833,2.,5.,8.}) {
      auto candidate=path;
      for(std::size_t i=0;i<candidate.count;++i)
        candidate.points[i].speed_mps=candidate.points[i].uncapped_speed_mps=speed;
      const auto x=planner.evaluateExecutionReference(candidate,r), y=prepared(candidate);
      check(x.valid==y.valid && x.reject_reason==y.reject_reason && x.cost==y.cost);
      check(x.cost_terms==y.cost_terms && x.predicted_rollout_count==y.predicted_rollout_count);
      for(std::size_t i=0;i<x.predicted_rollout_count;++i)
        check(x.predicted_rollout[i].x_m==y.predicted_rollout[i].x_m &&
              x.predicted_rollout[i].y_m==y.predicted_rollout[i].y_m);
    }
    for(int repeat=0;repeat<8;++repeat) {
      const auto begin=std::chrono::steady_clock::now();
      const auto a=optimizeExecutionSpeed(Uncached{planner},path,r,incumbent);
      const auto middle=std::chrono::steady_clock::now();
      const auto b=optimizeExecutionSpeed(planner,path,r,incumbent);
      const auto end=std::chrono::steady_clock::now();
      uncached_ms+=std::chrono::duration<double,std::milli>(middle-begin).count();
      cached_ms+=std::chrono::duration<double,std::milli>(end-middle).count();
      check(a.evaluated==b.evaluated && a.valid==b.valid && a.improved==b.improved);
      check(a.evaluation.valid==b.evaluation.valid && a.evaluation.cost==b.evaluation.cost);
      check(a.evaluation.reject_reason==b.evaluation.reject_reason);
      check(a.evaluation.cost_terms==b.evaluation.cost_terms);
      check(a.evaluation.predicted_rollout_count==b.evaluation.predicted_rollout_count);
      for(std::size_t i=0;i<a.evaluation.predicted_rollout_count;++i) {
        const auto &x=a.evaluation.predicted_rollout[i], &y=b.evaluation.predicted_rollout[i];
        check(x.x_m==y.x_m && x.y_m==y.y_m);
      }
      for(std::size_t i=0;i<a.reference.count;++i)
        check(a.reference.points[i].speed_mps==b.reference.points[i].speed_mps);
    }
  }
  r.static_obstacle_count=0;r.dynamic_obstacle_count=0;
  for(std::size_t i=0;i<path.count;++i) path.points[i].y_m=0.;
  int calls=0;
  r.path_constraint_geometry_only=true;
  r.path_constraint_validator=[&](const TemporaryReference &) { ++calls; return RejectReason::NONE; };
  const auto cached_geometry=planner.makeExecutionSpeedEvaluator(path,r);
  check(cached_geometry(path).valid);
  auto faster=path;
  for(std::size_t i=0;i<faster.count;++i)faster.points[i].speed_mps=2.;
  check(cached_geometry(faster).valid);
  check(calls==1);
  // A fresh search must not inherit a previous shape/snapshot's certificate.
  r.path_constraint_validator=[&](const TemporaryReference &) { ++calls; return RejectReason::WALL; };
  const auto changed=planner.makeExecutionSpeedEvaluator(path,r);
  check(changed(path).reject_reason==RejectReason::WALL);
  check(changed(faster).reject_reason==RejectReason::WALL);
  check(calls==2);
  r.path_constraint_geometry_only=false;
  r.path_constraint_validator=[&](const TemporaryReference &p) {
    ++calls;return p.points[0].speed_mps>1.5?RejectReason::WALL:RejectReason::NONE;
  };
  const auto speed_dependent=planner.makeExecutionSpeedEvaluator(path,r);
  check(speed_dependent(path).valid);
  check(speed_dependent(faster).reject_reason==RejectReason::WALL);
  check(calls==4);
  // A 60 m route must not make a distant wall veto a locally executable
  // command. The exact same callback must still reject reachable geometry.
  r.path_constraint_geometry_only=true;
  r.path_constraint_execution_prefix=true;
  path.count=151;
  for(std::size_t i=0;i<path.count;++i) {
    path.points[i]={};
    path.points[i].x_m=path.points[i].s_m=.4*i;
    path.points[i].speed_mps=path.points[i].uncapped_speed_mps=0.;
  }
  r.ego.speed_mps=4.;
  double wall_at=59.;
  r.path_constraint_validator=[&](const TemporaryReference &p) {
    return p.points[p.count-1].x_m>=wall_at?RejectReason::WALL:RejectReason::NONE;
  };
  auto fast=path;
  for(std::size_t i=0;i<fast.count;++i)
    fast.points[i].speed_mps=fast.points[i].uncapped_speed_mps=10.;
  const auto slow_result=planner.evaluateExecutionReference(path,r);
  const auto fast_result=planner.evaluateExecutionReference(fast,r);
  check(slow_result.valid && fast_result.valid);
  check(slow_result.validated_reference_distance_m<fast_result.validated_reference_distance_m);
  check(path.count==151 && path.points[150].x_m==60.);
  wall_at=.5*(slow_result.validated_reference_distance_m+fast_result.validated_reference_distance_m);
  const auto prefix_search=planner.makeExecutionSpeedEvaluator(path,r);
  check(prefix_search(path).valid);
  check(prefix_search(fast).reject_reason==RejectReason::WALL);
  check(prefix_search(path).valid); // a longer prefix rejection cannot poison braking
  r.path_constraint_execution_prefix=false;
  check(planner.evaluateExecutionReference(path,r).reject_reason==RejectReason::WALL);
  r.path_constraint_execution_prefix=true;
  wall_at=1.;
  check(planner.evaluateExecutionReference(path,r).reject_reason==RejectReason::WALL);
  wall_at=59.;
  r.rollout_constraint_validator=[](const RolloutState &,const RolloutState &b) {
    return b.x_m>=1.?RejectReason::WALL:RejectReason::NONE;
  };
  const auto swept_wall=planner.evaluateExecutionReference(path,r);
  check(swept_wall.reject_reason==RejectReason::WALL);
  check(std::string(swept_wall.reject_stage)=="environment_execution_segment");
  r.rollout_constraint_validator={};
  r.ego.speed_mps=0.;
  auto preview_config=c;
  preview_config.cma_lookahead_curvature_enabled=true;
  preview_config.cma_curvature_preview_distance_m=25.;
  wall_at=20.;
  const auto stationary=ReferenceSpaceMppiPlanner(preview_config).evaluateExecutionReference(path,r);
  check(stationary.valid); // distant curvature data is not a body placement
  check(stationary.progress_m==0.);
  std::cout<<"uncached_ms="<<uncached_ms<<" cached_ms="<<cached_ms<<" ratio="<<cached_ms/uncached_ms<<'\n';
}
