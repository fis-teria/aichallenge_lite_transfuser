#include "reference_space_mppi_planner/side_selection.hpp"
#include "reference_space_mppi_planner/overtake_objective.hpp"
#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include "reference_space_mppi_planner/wall_line_cost.hpp"
#include <random>
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
TEST(WallLineCost, PrefersPassingSideWallSymmetricallyWithoutSpeedReward) {
  PlanRequest r;r.phase=Phase::OVERTAKE;r.wall_line_start_s_m=5;r.wall_line_end_s_m=10;
  r.base_reference_count=2;r.base_reference[1].s_m=20;
  for(auto &b:r.base_reference){b.minimum_d_m=-3;b.maximum_d_m=3;}
  TemporaryReference path;path.count=3;
  for(int i=0;i<3;++i)path.points[i].s_m=i*10;
  double center=wallLineCost(path,r);EXPECT_NEAR(center,1.,1e-12);
  for(double sign:{-1.,1.}) {
    for(auto &p:path.points)p.d_m=sign*2.7;
    EXPECT_NEAR(wallLineCost(path,r),.01,1e-12);
    for(auto &p:path.points)p.speed_mps=10.;
    EXPECT_NEAR(wallLineCost(path,r),.01,1e-12);
    for(auto &p:path.points)p.d_m=sign*3.;
    EXPECT_NEAR(wallLineCost(path,r),0.,1e-12);
  }
}
TEST(WallLineCost, IgnoresEntryAndMergeButChargesMissingPassZone) {
  PlanRequest r;r.phase=Phase::OVERTAKE;r.wall_line_start_s_m=5;r.wall_line_end_s_m=10;
  r.base_reference_count=2;r.base_reference[1].s_m=20;
  TemporaryReference path;path.count=4;
  for(int i=0;i<4;++i){path.points[i].s_m=i*5;path.points[i].d_m=2.;}
  EXPECT_NEAR(wallLineCost(path,r),0.,1e-12);
  path.points[0].d_m=-1.;
  EXPECT_NEAR(wallLineCost(path,r),0.,1e-12);
  path.count=2;
  EXPECT_NEAR(wallLineCost(path,r),1.,1e-12);
  r.phase=Phase::MERGE;EXPECT_EQ(wallLineCost(path,r),0.);
}
TEST(OvertakePriority, RecordedValidPassBeatsPublishedStop) {
  // H2H B generation 2320: both publication references certified; actual
  // measured initial speed .638 m/s, full-body passage at 2.8 s.
  std::array<SideSelectionCandidate, 2> candidates{{
      {1, true, 10.0977562641, 2.8}, {1, true, 4.89803166727}}};
  const auto selected = selectSideCandidateRelative(candidates, 2U, 1, .1);
  ASSERT_TRUE(selected); EXPECT_EQ(*selected, 0U);
}
TEST(OvertakePriority, FasterPassBeatsCheaperPassAndSideHysteresis) {
  std::array<SideSelectionCandidate, 2> candidates{{
      {-1, true, 100.0, 2.0}, {1, true, 1.0, 3.0}}};
  EXPECT_EQ(selectSideCandidateRelative(candidates, 2U, 1, .9), 0U);
}
TEST(OvertakePriority, InvalidPassCannotWinAndNoPassKeepsLegacyCost) {
  std::array<SideSelectionCandidate, 3> candidates{{
      {-1, false, .1, 1.0}, {1, true, 4.0}, {-1, true, 3.0}}};
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.0), 2U);
  candidates[0] = {-1, true, std::numeric_limits<double>::quiet_NaN(), 1.0};
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.0), 2U);
}
TEST(OvertakePriority, StoppedContinuationLosesButExactPassTieContinues) {
  std::array<SideSelectionCandidate, 1> candidates{{{-1, true, 10.0, 2.0}}};
  auto selected = selectExecutionCandidate(candidates, 1U, {1, true, 0.0}, 1, .9);
  ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  selected = selectExecutionCandidate(candidates, 1U, {-1, true, 10.0, 2.0}, -1, .9);
  ASSERT_TRUE(selected); EXPECT_TRUE(selected->continuation);
}
PlanRequest passRequest() {
  PlanRequest request;
  const std::vector<std::array<double,2>> points{{-20,0},{60,0}};
  request.world_reference = std::make_shared<const ReferencePoseIndex>(points, [](auto p){ return p; });
  request.phase = Phase::OVERTAKE;
  request.dynamic_obstacle_count = 1;
  request.overtake_target_index = 0;
  auto &target = request.dynamic_obstacles[0];
  target.global_reference_s_m = 25; // x=5
  target.longitudinal_speed_mps = 1;
  target.d_m = 3;
  return request;
}
Evaluation passRollout() {
  Evaluation evaluation;
  evaluation.valid = true; evaluation.cost = 100;
  evaluation.predicted_rollout_count = 4;
  for (std::size_t i=0; i<4; ++i) {
    const double t=i+1;
    evaluation.predicted_rollout[i] = {4*t,0,0,4,t};
  }
  return evaluation;
}
TEST(OvertakeMetric, UsesMovingOpponentAndEntireBodyNotCenterOrTargetSpeed) {
  const auto request=passRequest(); const auto rollout=passRollout();
  // Centers pass at 2 s (8 > 7), full bodies only at 3 s (10.9 > 9.1).
  EXPECT_DOUBLE_EQ(predictedOvertakeTime(request, Config{}, rollout), 3.0);
  auto stop=rollout; for(auto &p:stop.predicted_rollout) p.x_m=0;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, stop)));
}
TEST(OvertakeMetric, NoCreditForInvalidAlreadyPassedOrMissingTarget) {
  auto request=passRequest(); auto rollout=passRollout();
  rollout.valid=false;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, rollout)));
  rollout.valid=true; request.overtake_target_index=1;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, rollout)));
  request.overtake_target_index=0; request.ego.x_m=20;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, rollout)));
  request.ego.x_m=0; request.phase=Phase::MERGE;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, rollout)));
}
TEST(OvertakeMetric, PassageMustLastThroughEnd) {
  const auto request=passRequest(); auto rollout=passRollout();
  rollout.predicted_rollout[3].x_m=8;
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request, Config{}, rollout)));
  rollout.predicted_rollout[0].x_m=10;
  rollout.predicted_rollout[3].x_m=16;
  EXPECT_DOUBLE_EQ(predictedOvertakeTime(request, Config{}, rollout), 3.0);
}
struct PassSpeedEvaluator {
  Config config_{};
  const Config &config() const { return config_; }
  Evaluation evaluateExecutionReference(const TemporaryReference &reference, const PlanRequest &) const {
    Evaluation e; const double v=reference.points[0].speed_mps;
    e.valid=v<=5; e.cost=v*v;
    if(v>=3 && e.valid) e.overtake_time_sec=10/v;
    return e;
  }
};
TEST(OvertakePriority, SpeedSearchPrefersPassingSpeedToStoppedIncumbent) {
  PlanRequest r; r.base_reference_count=2; r.bounds.maximum.speed_scale=1;
  r.base_reference[0].speed_mps=r.base_reference[1].speed_mps=10;
  TemporaryReference path; path.count=3;
  for(std::size_t i=0;i<path.count;++i) path.points[i].x_m=path.points[i].s_m=10*i;
  PassSpeedEvaluator evaluator;
  const auto result=optimizeExecutionSpeed(evaluator,path,r,evaluator.evaluateExecutionReference(path,r));
  EXPECT_TRUE(result.improved); EXPECT_TRUE(result.evaluation.valid);
  EXPECT_TRUE(std::isfinite(result.evaluation.overtake_time_sec));
  EXPECT_LE(result.reference.points[0].speed_mps, 5);
  EXPECT_LE(result.evaluated,kMaximumExecutionSpeedEvaluations);
}

TEST(OvertakePriority, WeightedUpdateCannotBeDominatedByCheaperNonPassSamples) {
  EXPECT_DOUBLE_EQ(candidateWeight(-100, INFINITY, 10, 2.8, 1), 0);
  EXPECT_DOUBLE_EQ(candidateWeight(1, 3.0, 10, 2.8, 1), 0);
  EXPECT_DOUBLE_EQ(candidateWeight(10, 2.8, 10, 2.8, 1), 1);
  EXPECT_NEAR(candidateWeight(11, 2.8, 10, 2.8, 1), std::exp(-1), 1e-15);
  EXPECT_NEAR(candidateWeight(11, INFINITY, 10, INFINITY, 1), std::exp(-1), 1e-15);
}

TEST(OvertakeProjection, ExactAgainstExhaustiveNonUniformFoldedReference) {
  std::vector<std::array<double,2>> points;
  std::mt19937 rng(701);
  std::uniform_real_distribution<double> random(-25,25);
  for(int i=0;i<101;++i) points.push_back({random(rng),random(rng)});
  const ReferencePoseIndex index(points, [](auto p){return p;});
  for(int q=0;q<1000;++q) {
    const double x=random(rng),y=random(rng);
    double station=0, best=INFINITY, expected=0;
    for(std::size_t i=0;i+1<points.size();++i) {
      const auto a=points[i],b=points[i+1];
      const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
      const double u=std::clamp(((x-a[0])*dx+(y-a[1])*dy)/(length*length),0.,1.);
      const double ex=x-(a[0]+u*dx),ey=y-(a[1]+u*dy),dist=ex*ex+ey*ey;
      if(dist<best) {best=dist;expected=station+u*length;}
      station+=length;
    }
    ASSERT_TRUE(index.projectStation(x,y));
    EXPECT_NEAR(*index.projectStation(x,y),expected,1e-10);
  }
}

TEST(OvertakeMetric, LoopSeamAndRotatedBodies) {
  std::vector<std::array<double,2>> points;
  constexpr double pi=3.14159265358979323846;
  for(int i=0;i<=1000;++i) points.push_back({30*std::cos(2*pi*i/1000),30*std::sin(2*pi*i/1000)});
  auto request=passRequest();
  request.world_reference=std::make_shared<const ReferencePoseIndex>(points,[](auto p){return p;});
  auto &target=request.dynamic_obstacles[0];
  target.global_reference_s_m=2; target.d_m=0; target.heading_relative_to_reference_rad=.6;
  const auto initial=*request.world_reference->pose(-4,-3);
  request.ego.x_m=initial[0];request.ego.y_m=initial[1];request.ego.yaw_rad=initial[2];
  auto evaluation=passRollout();
  for(std::size_t i=0;i<evaluation.predicted_rollout_count;++i) {
    const double t=i+1;
    const auto p=*request.world_reference->pose(-4+4*t,-3);
    evaluation.predicted_rollout[i]={p[0],p[1],p[2],4,t};
  }
  EXPECT_DOUBLE_EQ(predictedOvertakeTime(request,Config{},evaluation),3);
}

TEST(OvertakeMetric, CommonValidatorScoresRealAccelerationAndRejectsWallAndCollision) {
  auto request=passRequest();request.valid=true;request.side=1;
  request.sample_staged_speed=true;request.sample_control_sequence=true;
  request.nominal=Parameters{0,8,10,8,1};
  request.bounds.minimum=Parameters{0,5,0,5,0};
  request.bounds.maximum=Parameters{2,12,18,12,1};
  request.base_reference_count=101;
  TemporaryReference stop;stop.count=101;
  for(std::size_t i=0;i<stop.count;++i) {
    auto &p=stop.points[i];p.x_m=p.s_m=p.source_s_m=.3*i;
    auto &b=request.base_reference[i]; b.x_m=b.s_m=p.s_m;b.speed_mps=5;
    b.minimum_d_m=-5;b.maximum_d_m=5;
  }
  request.dynamic_obstacles[0].s_m=5;
  setExecutionHistory(request,stop);
  auto fast=stop;for(auto &p:fast.points)p.speed_mps=p.uncapped_speed_mps=5;
  Config config;config.enabled=true;config.cost_control_change_weight=100;
  const ReferenceSpaceMppiPlanner planner(config);
  const auto held=planner.evaluateExecutionReference(stop,request);
  const auto accelerated=planner.evaluateExecutionReference(fast,request);
  ASSERT_TRUE(held.valid);ASSERT_TRUE(accelerated.valid) << ReferenceSpaceMppiPlanner::toString(accelerated.reject_reason);
  EXPECT_FALSE(std::isfinite(held.overtake_time_sec));
  EXPECT_TRUE(std::isfinite(accelerated.overtake_time_sec));
  EXPECT_GT(accelerated.cost,held.cost);
  EXPECT_TRUE(betterEvaluation(accelerated,held));
  const auto retimed=optimizeExecutionSpeed(planner,stop,request,held);
  EXPECT_TRUE(retimed.improved);EXPECT_TRUE(std::isfinite(retimed.evaluation.overtake_time_sec));
  request.path_constraint_validator=[](const TemporaryReference &){return RejectReason::WALL;};
  const auto wall=planner.evaluateExecutionReference(fast,request);
  EXPECT_FALSE(wall.valid);EXPECT_FALSE(std::isfinite(wall.overtake_time_sec));
  request.path_constraint_validator={};request.dynamic_obstacles[0].d_m=0;
  const auto collision=planner.evaluateExecutionReference(fast,request);
  EXPECT_FALSE(collision.valid);EXPECT_EQ(collision.reject_reason,RejectReason::COLLISION);
  EXPECT_FALSE(std::isfinite(collision.overtake_time_sec));
}

TEST(OvertakePriority, RawWeightedAndShapeSpeedSelectionKeepFeasiblePass) {
  auto request=passRequest();request.valid=true;request.side=1;
  request.sample_staged_speed=true;request.sample_control_sequence=true;
  request.sample_lateral_bounds=true;request.complete_maneuver=false;
  request.cartesian_measured_prefix=true;request.speed_pair_diagnostics=true;
  request.nominal=Parameters{0,8,10,8,.2};
  request.bounds.minimum=Parameters{0,8,10,8,0};
  request.bounds.maximum=Parameters{0,8,10,8,1};
  request.base_reference_count=101;request.preferred_matching_speed_mps=1;
  TemporaryReference stop;stop.count=101;
  for(std::size_t i=0;i<stop.count;++i) {
    auto &p=stop.points[i];p.x_m=p.s_m=p.source_s_m=.3*i;
    auto &b=request.base_reference[i];b.x_m=b.s_m=p.s_m;b.speed_mps=5;
    b.minimum_d_m=-5;b.maximum_d_m=5;
  }
  request.dynamic_obstacles[0].s_m=5;
  setExecutionHistory(request,stop);
  Config config;config.enabled=true;config.cost_control_change_weight=100;
  config.sample_count=10;config.minimum_valid_count=1;config.minimum_valid_ratio=0;
  config.control_maximum_speed_scale_adjustment=1;
  auto scratch=std::make_unique<Scratch>();
  const auto result=ReferenceSpaceMppiPlanner(config).plan(request,scratch.get());
  ASSERT_TRUE(result.valid) << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  ASSERT_TRUE(std::isfinite(result.selected_evaluation.overtake_time_sec));
  for(std::size_t i=0;i<result.speed_pair_count;++i)
    EXPECT_FALSE(betterEvaluation(result.speed_pairs[i].evaluation,result.selected_evaluation));
  auto cost_only=request;cost_only.overtake_target_index=std::numeric_limits<std::size_t>::max();
  const auto legacy=ReferenceSpaceMppiPlanner(config).plan(cost_only,scratch.get());
  ASSERT_TRUE(legacy.valid);
  EXPECT_LT(legacy.selected_evaluation.cost,result.selected_evaluation.cost);
  EXPECT_FALSE(std::isfinite(predictedOvertakeTime(request,config,legacy.selected_evaluation)));
  EXPECT_LE(result.evaluated_sample_count,20U);
}
} // namespace
} // namespace reference_space_mppi_planner::mppi
