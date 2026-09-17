#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include "reference_space_mppi_planner/prior_lap_prediction.hpp"
#include "reference_space_mppi_planner/passing_progress.hpp"
#include <gtest/gtest.h>
#include <numeric>

namespace reference_space_mppi_planner::mppi {
namespace {
TEST(ApproachSpeed, GapRecoveryIsNotAnOpponentSpeedFloor) {
  Config c;
  DynamicObstacle o; o.s_m=7.2; o.longitudinal_speed_mps=5.;
  ASSERT_TRUE(approachSpeedTarget(o,0.,0.,0.,0.,c));
  EXPECT_NEAR(*approachSpeedTarget(o,0.,0.,0.,0.,c),5.,1e-12);
  o.s_m=5.2;
  EXPECT_NEAR(*approachSpeedTarget(o,0.,0.,0.,0.,c),4.,1e-12);
  o.s_m=11.2;
  EXPECT_NEAR(*approachSpeedTarget(o,0.,0.,0.,0.,c),7.,1e-12);
  o.s_m=5.2; o.longitudinal_speed_mps=0.;
  EXPECT_DOUBLE_EQ(*approachSpeedTarget(o,0.,0.,0.,0.,c),0.);
  o.d_m=2.;
  EXPECT_FALSE(approachSpeedTarget(o,0.,0.,0.,0.,c));
  o.d_m=0.; o.s_m=-1.;
  EXPECT_FALSE(approachSpeedTarget(o,0.,0.,0.,0.,c));
}

TEST(ApproachSpeed, ForecastVelocityAndLateralClearanceUseTheSameEpoch) {
  Config c;
  DynamicObstacle o; o.s_m=7.2; o.longitudinal_speed_mps=9.;
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  prediction->points={{0.,100.,0.,0.},{1.,105.,0.,0.},{2.,108.,2.,0.}};
  o.prediction=prediction;
  // Current speed selects the 2 m gap; forecast velocity still uses each epoch.
  EXPECT_NEAR(*approachSpeedTarget(o,0.,0.,0.,0.,c),6.5,1e-12);
  EXPECT_NEAR(*approachSpeedTarget(o,5.,0.,0.,1.,c),4.5,1e-12);
  EXPECT_FALSE(approachSpeedTarget(o,8.,0.,0.,2.,c));
}

struct ApproachFixture : testing::Test {
  Config c;
  PlanRequest r;
  TemporaryReference path;
  void SetUp() override {
    c.enabled=true; c.shadow_only=false; c.collision_only_rejection=true;
    c.horizon_steps=20; c.maximum_acceleration_mps2=2.;
    c.speed_proportional_gain=3.; c.cost_approach_speed_weight=.1;
    r.valid=true; r.side=1; r.phase=Phase::OVERTAKE; r.sample_staged_speed=true;
    r.ego.speed_mps=5.; r.base_reference_count=101;
    r.nominal={0,4,4,4,1,0,1}; r.bounds.minimum={0,4,4,4,0,0,1};
    r.bounds.maximum={0,4,4,4,1,0,1}; r.dynamic_obstacle_count=1;
    r.dynamic_obstacles[0].s_m=7.2; r.dynamic_obstacles[0].longitudinal_speed_mps=5.;
    r.preferred_matching_speed_mps=5.; path.count=101;
    for(std::size_t i=0;i<101;++i) {
      auto &b=r.base_reference[i]; b.s_m=b.x_m=.4*i; b.speed_mps=10.;
      b.minimum_d_m=-10.; b.maximum_d_m=10.; b.active_d_valid=true;
      auto &p=path.points[i]; p.s_m=p.source_s_m=p.x_m=.4*i;
      p.speed_mps=p.uncapped_speed_mps=5.;
    }
  }
  Evaluation evaluate(double speed) {
    for(std::size_t i=0;i<path.count;++i)
      path.points[i].speed_mps=path.points[i].uncapped_speed_mps=speed;
    return ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  }
};

TEST_F(ApproachFixture, MeasuredFiveMeterBoundaryDoesNotAnticipateFutureApproach) {
  r.dynamic_obstacles[0].s_m=7.3;
  ASSERT_GT(evaluate(10.).cost_terms[11],0.);
  c.approach_at_follow_gap_only=true;
  EXPECT_FALSE(approachSpeedTarget(r.dynamic_obstacles[0],0.,0.,0.,0.,c));
  const auto outside=evaluate(10.);
  ASSERT_TRUE(outside.valid);
  EXPECT_DOUBLE_EQ(outside.cost_terms[11],0.);
  r.dynamic_obstacles[0].s_m=7.2;
  const auto boundary=approachSpeedTarget(r.dynamic_obstacles[0],0.,0.,0.,0.,c);
  ASSERT_TRUE(boundary);
  EXPECT_NEAR(*boundary,5.,1e-12);
  EXPECT_GT(evaluate(10.).cost_terms[11],0.);
  r.dynamic_obstacles[0].s_m=7.1;
  EXPECT_LT(*approachSpeedTarget(r.dynamic_obstacles[0],0.,0.,0.,0.,c),5.);
  r.dynamic_obstacles[0].d_m=2.5;
  EXPECT_DOUBLE_EQ(evaluate(10.).cost_terms[11],0.);
  r.rollout_constraint_validator=[](const auto &,const auto &) { return RejectReason::WALL; };
  EXPECT_EQ(evaluate(10.).reject_reason,RejectReason::WALL);
  r.rollout_constraint_validator={};
  r.dynamic_obstacles[0].d_m=0.;r.dynamic_obstacles[0].s_m=1.;
  EXPECT_EQ(evaluate(10.).reject_reason,RejectReason::COLLISION);
}

TEST_F(ApproachFixture, MatchesBeforeContactAndCanRecoverShortGap) {
  const auto matching=evaluate(5.), fast=evaluate(10.);
  ASSERT_TRUE(matching.valid); ASSERT_TRUE(fast.valid);
  EXPECT_LT(matching.cost,fast.cost);
  EXPECT_DOUBLE_EQ(matching.cost_terms[11],0.);
  EXPECT_GT(fast.cost_terms[11],0.);
  EXPECT_NEAR(std::accumulate(fast.cost_terms.begin(),fast.cost_terms.end(),0.),fast.cost,1e-12);
  c.cost_approach_speed_weight=0.;
  EXPECT_LT(evaluate(10.).cost,evaluate(5.).cost);
  c.cost_approach_speed_weight=.1; r.dynamic_obstacles[0].s_m=5.2;
  const auto recover=evaluate(4.), hold=evaluate(5.);
  ASSERT_TRUE(recover.valid); ASSERT_TRUE(hold.valid);
  EXPECT_LT(recover.cost,hold.cost);
}

TEST_F(ApproachFixture, CommonPassingObjectiveIsAdditiveAndPreservesCollisionRejection) {
  const std::vector<std::array<double,2>> points{{0,0},{40,0}};
  r.world_reference=std::make_shared<const ReferencePoseIndex>(points,[](auto p){return p;});
  r.dynamic_obstacles[0].global_reference_s_m=7.2;
  r.passing_entry_m=0.; r.passing_exit_m=35.; r.passing_speed_mps=10.;
  const auto baseline=evaluate(6.);
  c.cost_passing_progress_weight=1.;c.cost_passing_opportunity_weight=.5;
  const auto scored=evaluate(6.);
  ASSERT_TRUE(baseline.valid); ASSERT_TRUE(scored.valid);
  const auto terms=passingProgress(r,c,scored);
  EXPECT_NEAR(scored.cost,baseline.cost-terms.progress-.5*terms.opportunity,1e-12);
  EXPECT_NEAR(std::accumulate(scored.cost_terms.begin(),scored.cost_terms.end(),0.),scored.cost,1e-12);
  EXPECT_EQ(scored.predicted_rollout_count,baseline.predicted_rollout_count);
  for(std::size_t i=0;i<scored.predicted_rollout_count;++i)
    EXPECT_DOUBLE_EQ(scored.predicted_rollout[i].x_m,baseline.predicted_rollout[i].x_m);
  r.rollout_constraint_validator=[](const auto &,const auto &){return RejectReason::WALL;};
  EXPECT_EQ(evaluate(6.).reject_reason,RejectReason::WALL);
  r.rollout_constraint_validator={};r.dynamic_obstacles[0].s_m=1.;
  r.dynamic_obstacles[0].global_reference_s_m=1.;
  EXPECT_EQ(evaluate(6.).reject_reason,RejectReason::COLLISION);
}

TEST_F(ApproachFixture, AdjacentPassAcceleratesWithinFiveMetersWithValidators) {
  r.dynamic_obstacles[0].s_m=4.2; r.dynamic_obstacles[0].d_m=2.5;
  const auto slow=evaluate(5.), fast=evaluate(10.);
  ASSERT_TRUE(slow.valid); ASSERT_TRUE(fast.valid);
  EXPECT_DOUBLE_EQ(fast.cost_terms[11],0.);
  EXPECT_LT(fast.cost,slow.cost);
  r.rollout_constraint_validator=[](const auto &,const auto &) { return RejectReason::WALL; };
  EXPECT_EQ(evaluate(10.).reject_reason,RejectReason::WALL);
  r.rollout_constraint_validator={}; r.dynamic_obstacles[0].d_m=0.;
  r.dynamic_obstacles[0].s_m=1.;
  EXPECT_EQ(evaluate(0.).reject_reason,RejectReason::COLLISION);
  EXPECT_EQ(evaluate(10.).reject_reason,RejectReason::COLLISION);
}

TEST_F(ApproachFixture, CoarseAndCachedSearchIncludeMatchingWithoutChangingGeometry) {
  r.preferred_matching_speed_mps=4.65; r.dynamic_obstacles[0].s_m=6.5;
  const ReferenceSpaceMppiPlanner planner(c);
  const auto incumbent=evaluate(10.);
  struct ObservedEvaluator {
    const ReferenceSpaceMppiPlanner &planner;
    mutable std::vector<double> speeds;
    const Config &config() const { return planner.config(); }
    Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &request) const {
      speeds.push_back(p.points[0].speed_mps);
      return planner.evaluateExecutionReference(p,request);
    }
  } observed{planner,{}};
  const auto result=optimizeExecutionSpeed(observed,path,r,incumbent,ExecutionSpeedSearch::Coarse);
  ASSERT_TRUE(result.evaluation.valid);
  EXPECT_LE(result.evaluated,5U);
  EXPECT_LT(result.reference.points[0].speed_mps,10.);
  EXPECT_NE(std::find(observed.speeds.begin(),observed.speeds.end(),
      static_cast<float>(r.preferred_matching_speed_mps)),observed.speeds.end());
  for(std::size_t i=0;i<path.count;++i) {
    EXPECT_EQ(result.reference.points[i].x_m,path.points[i].x_m);
    EXPECT_EQ(result.reference.points[i].y_m,path.points[i].y_m);
  }
  const auto cached=planner.makeExecutionSpeedEvaluator(path,r)(result.reference);
  const auto direct=planner.evaluateExecutionReference(result.reference,r);
  EXPECT_EQ(cached.valid,direct.valid); EXPECT_DOUBLE_EQ(cached.cost,direct.cost);
  EXPECT_EQ(cached.cost_terms,direct.cost_terms);
}
} // namespace
} // namespace reference_space_mppi_planner::mppi
