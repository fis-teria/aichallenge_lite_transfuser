#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
namespace m = reference_space_mppi_planner::mppi;

TEST(CrossShapeSampling, FindsValidFastTransitionWithinExistingBudget) {
  m::Config c;
  c.enabled=true; c.shadow_only=false; c.collision_only_rejection=true;
  c.horizon_steps=60; c.sample_count=10; c.minimum_valid_count=1;
  c.maximum_acceleration_mps2=2; c.speed_proportional_gain=3;
  c.obstacle_lateral_inflation_m=1.15; c.cost_terminal_lateral_weight=2;
  c.compare_cma_delay_compensation=true; c.compare_cma_preview_feedforward=true;
  c.cma_lookahead_curvature_enabled=true;
  c.control_maximum_speed_scale_adjustment=1;
  c.lookahead_gain=.2; c.lookahead_min_distance_m=2;
  c.curvature_lookahead_min_distance_m=2;
  c.continuous_preview_interpolation_enabled=true; c.actual_lookahead_distance_blend=1;
  m::PlanRequest r;
  r.valid=true; r.side=1; r.phase=m::Phase::OVERTAKE;
  r.generation=1; r.semantic_key=1; r.stamp_sec=100;
  r.ego.speed_mps=3; r.anchor_s_m=.6;
  r.complete_maneuver=false; r.cartesian_measured_prefix=true;
  r.sample_control_sequence=true; r.sample_staged_speed=true;
  r.sample_lateral_bounds=true; r.preferred_matching_speed_mps=.833333333;
  r.nominal={2.2,7.5,5,8,1,0,1};
  r.bounds.minimum={2.2,7.5,5,8,0,0,.55};
  r.bounds.maximum={2.2,15.5,5,8,1,.45,1};
  r.pass_profile_scale_m=2.2; r.base_reference_count=101;
  for(std::size_t i=0;i<r.base_reference_count;++i) {
    auto &b=r.base_reference[i]; b.s_m=b.x_m=.2*i;
    b.speed_mps=10; b.minimum_d_m=-2.5; b.maximum_d_m=2.5;
    b.active_d_valid=true; b.pass_d_valid=true; b.pass_d_m=2.2;
  }
  r.dynamic_obstacle_count=1; r.dynamic_obstacles[0].s_m=11;
  r.dynamic_obstacles[0].longitudinal_speed_mps=.833333333;
  r.dynamic_obstacles[0].longitudinal_acceleration_bound_mps2=.5;
  r.rollout_constraint_validator=[](const m::RolloutState &, const m::RolloutState &s) {
    return std::abs(s.y_m)>2.5?m::RejectReason::WALL:m::RejectReason::NONE;
  };
  auto scratch=std::make_unique<m::Scratch>();
  const auto result=m::ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.selected_evaluation.cost,-.50);
  EXPECT_GT(result.selected_evaluation.progress_m,15.5);
  EXPECT_GT(result.selected_reference.points[0].speed_mps,3.);
  EXPECT_LE(result.evaluated_sample_count,20U);
  r.speed_pair_diagnostics=true;
  for(double gap:{5.,8.,11.,14.}) for(double speed:{.833333333,3.,6.}) {
    r.dynamic_obstacles[0].s_m=gap;
    r.ego.speed_mps=speed;
    const auto selected=m::ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
    auto no_diagnostic=r;
    no_diagnostic.speed_pair_diagnostics=false;
    const auto normal=m::ReferenceSpaceMppiPlanner(c).plan(no_diagnostic,scratch.get());
    ASSERT_EQ(selected.valid,normal.valid);
    if(!selected.valid) continue;
    EXPECT_DOUBLE_EQ(selected.selected_evaluation.cost,normal.selected_evaluation.cost);
    EXPECT_EQ(normal.speed_pair_count,0U);
    ASSERT_EQ(selected.selected_reference.count,normal.selected_reference.count);
    for(std::size_t i=0;i<selected.selected_reference.count;++i) {
      EXPECT_DOUBLE_EQ(selected.selected_reference.points[i].speed_mps,normal.selected_reference.points[i].speed_mps);
      EXPECT_DOUBLE_EQ(selected.selected_reference.points[i].x_m,normal.selected_reference.points[i].x_m);
      EXPECT_DOUBLE_EQ(selected.selected_reference.points[i].y_m,normal.selected_reference.points[i].y_m);
    }
    ASSERT_EQ(selected.speed_pair_count,4U);
    // Constant-speed refinement belongs to the exact execution comparison.
    // Diagnostic acceleration probes do not refine the joint-search winner.
    const m::ReferenceSpaceMppiPlanner evaluator(c);
    const auto serialize_speed=[](m::TemporaryReference p) {
      for(std::size_t i=0;i<p.count;++i)
        p.points[i].speed_mps=p.points[i].uncapped_speed_mps=static_cast<float>(p.points[i].speed_mps);
      return p;
    };
    const auto executable=serialize_speed(selected.selected_reference);
    const auto refined=m::optimizeExecutionSpeed(evaluator,executable,r,
        evaluator.evaluateExecutionReference(executable,r),m::ExecutionSpeedSearch::Coarse);
    for(std::size_t i=0;i<2;++i) {
      const auto &pair=selected.speed_pairs[i];
      ASSERT_TRUE(pair.reference);
      const auto comparison=evaluator.evaluateExecutionReference(serialize_speed(*pair.reference),r);
      EXPECT_FALSE(m::betterEvaluation(comparison,refined.evaluation))
          << "gap=" << gap << " speed=" << speed << " profile=" << i;
    }
  }
  r.rollout_constraint_validator=[](const m::RolloutState &, const m::RolloutState &) {
    return m::RejectReason::COLLISION;
  };
  EXPECT_FALSE(m::ReferenceSpaceMppiPlanner(c).plan(r,scratch.get()).valid);
}
