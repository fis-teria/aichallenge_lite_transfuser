#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
struct LongitudinalTest : testing::Test {
  Config c;
  PlanRequest r;
  TemporaryReference path;
  void SetUp() override {
    c.enabled=true; c.shadow_only=false; c.longitudinal_planning_enabled=true;
    c.collision_only_rejection=true; c.awsim_vehicle_response_enabled=true;
    c.speed_proportional_gain=3.; c.maximum_acceleration_mps2=2.;
    c.maximum_deceleration_mps2=2.; c.horizon_steps=60;
    r.valid=true; r.side=1; r.sample_staged_speed=true;
    r.ego.speed_mps=5.; r.base_reference_count=101; path.count=101;
    r.dynamic_obstacle_count=1;
    r.dynamic_obstacles[0].s_m=5.+2.*c.vehicle_half_length_m;
    r.dynamic_obstacles[0].longitudinal_speed_mps=5.;
    r.nominal={0,4,4,4,1,0,1};
    r.bounds.minimum={0,4,4,4,0,0,1}; r.bounds.maximum={0,4,4,4,1,0,1};
    for(std::size_t i=0;i<path.count;++i) {
      auto &b=r.base_reference[i]; b.s_m=b.x_m=.4*i; b.speed_mps=10.;
      b.minimum_d_m=-10.; b.maximum_d_m=10.;
      auto &p=path.points[i]; p.s_m=p.source_s_m=p.x_m=.4*i;
      p.speed_mps=p.uncapped_speed_mps=9.;
    }
  }
  double command() {
    const auto planned=planLongitudinalSpeed(path,r,c);
    EXPECT_TRUE(planned.valid);
    return planned.reference.points[0].speed_mps;
  }
};

TEST_F(LongitudinalTest, MatchesFiveMetresWithoutLosingSpeedToDrag) {
  const double cmd=command();
  AwsimLongitudinalResponse plant;
  EXPECT_NEAR(plant.acceleration(3.*(cmd-5.),5.,0.),0.,1e-12);
}
TEST_F(LongitudinalTest, FrontMergeUsesTimedPreparationForGenerationAndHeldPathRetiming) {
  r.dynamic_obstacle_count=0;
  std::vector<std::array<double,2>> points;
  for(int i=0;i<=100;++i) points.push_back({double(i),0.});
  r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
  auto plan=std::make_shared<PassingPreparationPlan>();
  plan->world=r.world_reference;plan->front_merge=true;plan->schedule={{0.,0.,3.,0.},{6.,18.,3.,0.}};
  r.passing_preparation=plan;
  const auto guided=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(guided.valid);
  EXPECT_LT(guided.reference.points[0].speed_mps,r.ego.speed_mps);
  // A retained path deliberately disables the full-merge validator but must
  // retain its preparation's speed target.
  ASSERT_FALSE(r.front_merge_attack);
  const auto selected=planAndEvaluateLongitudinalSpeed(ReferenceSpaceMppiPlanner(c),path,r,c);
  ASSERT_TRUE(selected.improved) << selected.evaluation.reject_stage;
  EXPECT_TRUE(selected.preparation_selected);EXPECT_EQ(selected.evaluated,1U);
  EXPECT_FLOAT_EQ(selected.reference.points[0].speed_mps,guided.reference.points[0].speed_mps);
  plan->front_merge=false;
  const auto ordinary=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(ordinary.valid);
  EXPECT_GT(ordinary.reference.points[0].speed_mps,r.ego.speed_mps);
}
TEST_F(LongitudinalTest, FrontMergeSpeedGuidanceReachesPreparationBeyondLocalRollout) {
  r.dynamic_obstacle_count=0;r.horizon_steps_override=20;
  std::vector<std::array<double,2>> points;
  for(int i=0;i<=100;++i) points.push_back({double(i),0.});
  r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
  auto plan=std::make_shared<PassingPreparationPlan>();
  plan->world=r.world_reference;plan->front_merge=true;
  plan->schedule={{0.,0.,5.,0.},{3.,15.,5.,0.},{5.,21.,1.,0.}};
  r.passing_preparation=plan;
  const auto guided=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(guided.valid);EXPECT_EQ(guided.prediction_steps,100U);
  EXPECT_LT(guided.reference.points[guided.reference.count-1].speed_mps,guided.reference.points[0].speed_mps);
}
TEST_F(LongitudinalTest, FreeRunKeepsCruiseTargetOnCurvedGeometry) {
  r.dynamic_obstacle_count=0;
  r.ego.speed_mps=10.;
  const auto straight=planLongitudinalSpeed(path,r,c);
  for(std::size_t i=0;i<path.count;++i) {
    auto &p=path.points[i];
    p.yaw_rad=p.s_m/5.;
    p.x_m=5.*std::sin(p.yaw_rad);
    p.y_m=5.*(1.-std::cos(p.yaw_rad));
    p.curvature_1pm=.2;
  }
  const auto curved=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(straight.valid); ASSERT_TRUE(curved.valid);
  for(std::size_t i=0;i<path.count;++i) {
    EXPECT_DOUBLE_EQ(straight.reference.points[i].speed_mps,10.);
    EXPECT_DOUBLE_EQ(curved.reference.points[i].speed_mps,10.);
  }
}
TEST_F(LongitudinalTest, FastLeaderConvergesToTwoMetresBeforeDeparture) {
  double gap=12.,speed=7.;
  AwsimLongitudinalResponse plant;
  for(int i=0;i<800;++i) {
    r.ego.speed_mps=speed;
    r.dynamic_obstacles[0].s_m=gap+2.*c.vehicle_half_length_m;
    r.dynamic_obstacles[0].longitudinal_speed_mps=7.;
    const double cmd=command();
    const double a=plant.acceleration(std::clamp(3.*(cmd-speed),-2.,2.),speed,i*.05);
    gap+=(7.-speed)*.05;
    speed=std::max(0.,speed+a*.05);
    EXPECT_GT(gap,1.5);
  }
  EXPECT_NEAR(gap,2.,.1);
  EXPECT_NEAR(speed,7.,.05);
}
TEST_F(LongitudinalTest, TwentyKmhBoundaryChangesFollowingGapWithoutChangingGeometry) {
  r.dynamic_obstacles[0].s_m=3.+2.*c.vehicle_half_length_m;
  r.ego.speed_mps=20./3.6;
  r.dynamic_obstacles[0].longitudinal_speed_mps=20./3.6;
  const auto slow=planLongitudinalSpeed(path,r,c);
  r.dynamic_obstacles[0].longitudinal_speed_mps+=1e-6;
  const auto fast=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(slow.valid);ASSERT_TRUE(fast.valid);
  AwsimLongitudinalResponse slow_plant,fast_plant;
  EXPECT_LT(slow_plant.acceleration(c.speed_proportional_gain*
      (slow.reference.points[0].speed_mps-r.ego.speed_mps),r.ego.speed_mps,0.),0.);
  EXPECT_GT(fast_plant.acceleration(c.speed_proportional_gain*
      (fast.reference.points[0].speed_mps-r.ego.speed_mps),r.ego.speed_mps,0.),0.);
  for(std::size_t i=0;i<path.count;++i) {
    EXPECT_DOUBLE_EQ(fast.reference.points[i].x_m,path.points[i].x_m);
    EXPECT_DOUBLE_EQ(fast.reference.points[i].y_m,path.points[i].y_m);
  }
}
TEST_F(LongitudinalTest, RespondsToGapAndClosingSpeedBeforeFiveMetres) {
  r.dynamic_obstacles[0].s_m+=5.;
  EXPECT_GT(command(),5.);
  r.ego.speed_mps=9.;
  EXPECT_LT(command(),9.);
}
TEST_F(LongitudinalTest, AdjacentAndPassedVehiclesDoNotImposeFollowingSpeed) {
  r.dynamic_obstacles[0].longitudinal_speed_mps=0.;
  const double blocked=command();
  r.dynamic_obstacles[0].d_m=3.;
  EXPECT_GT(command(),blocked);
  r.dynamic_obstacles[0].d_m=0.;r.dynamic_obstacles[0].s_m=-2.;
  EXPECT_GT(command(),blocked);
}
TEST_F(LongitudinalTest, SpeedProposalPreservesGeometryAndSourceCoordinates) {
  const auto a=planLongitudinalSpeed(path,r,c);
  for(auto &p:path.points) p.speed_mps=p.uncapped_speed_mps=0.;
  const auto b=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(a.valid);ASSERT_TRUE(b.valid);
  for(std::size_t i=0;i<path.count;++i) {
    EXPECT_DOUBLE_EQ(a.reference.points[i].speed_mps,b.reference.points[i].speed_mps);
    EXPECT_DOUBLE_EQ(b.reference.points[i].x_m,path.points[i].x_m);
    EXPECT_DOUBLE_EQ(b.reference.points[i].s_m,path.points[i].s_m);
    EXPECT_DOUBLE_EQ(b.reference.points[i].source_s_m,path.points[i].source_s_m);
  }
}
TEST_F(LongitudinalTest, FutureLateralClearanceAllowsAccelerationOnPassingPath) {
  const auto follow=planLongitudinalSpeed(path,r,c);
  for(std::size_t i=0;i<path.count;++i)
    path.points[i].y_m=std::min(3.,path.points[i].x_m/3.);
  const auto pass=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(pass.valid);
  EXPECT_GT(pass.reference.points[40].speed_mps,follow.reference.points[40].speed_mps+1.);
}
TEST_F(LongitudinalTest, OvertakingReleasesFollowingAtDepartureBeforeLateralClearance) {
  r.dynamic_obstacles[0].s_m-=2.;
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  for(double side:{-1.,1.}) {
    for(std::size_t i=0;i<path.count;++i) {
      auto &p=path.points[i];
      const double u=std::clamp((p.x_m-8.)/8.,0.,1.);
      p.d_m=p.y_m=side*2.*u*u*u*(10.+u*(-15.+6.*u));
    }
    const auto follow=planLongitudinalSpeed(path,r,c);
    markOvertakeStart(path);
    EXPECT_NEAR(path.overtake_start_source_s_m,8.,1e-12);
    const auto pass=planLongitudinalSpeed(path,r,c);
    ASSERT_TRUE(pass.valid);
    EXPECT_DOUBLE_EQ(pass.reference.points[0].speed_mps,follow.reference.points[0].speed_mps);
    EXPECT_LT(pass.reference.points[0].speed_mps,r.ego.speed_mps);
    EXPECT_LT(std::abs(path.points[23].d_m),c.obstacle_lateral_inflation_m);
    EXPECT_GT(pass.reference.points[23].speed_mps,follow.reference.points[23].speed_mps+.2);
    path.overtake_start_source_s_m=INFINITY;
  }
}
TEST_F(LongitudinalTest, ImmediateDepartureDoesNotRestoreFiveMetres) {
  r.dynamic_obstacles[0].s_m-=2.;
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  for(std::size_t i=0;i<path.count;++i)
    path.points[i].d_m=path.points[i].y_m=std::min(2.,path.points[i].x_m*.2);
  const double following=command();
  markOvertakeStart(path);
  EXPECT_DOUBLE_EQ(path.overtake_start_source_s_m,0.);
  EXPECT_LT(following,r.ego.speed_mps);
  EXPECT_GT(command(),r.ego.speed_mps);
}
TEST_F(LongitudinalTest, ParallelUnstartedCandidateKeepsFollowing) {
  markOvertakeStart(path);
  EXPECT_FALSE(std::isfinite(path.overtake_start_source_s_m));
  r.dynamic_obstacles[0].s_m-=2.;
  AwsimLongitudinalResponse plant;
  EXPECT_LT(plant.acceleration(c.speed_proportional_gain*(command()-r.ego.speed_mps),
      r.ego.speed_mps,0.),0.);
}
TEST_F(LongitudinalTest, DeparturePositionSurvivesConsumptionAndRetiming) {
  r.dynamic_obstacles[0].s_m-=2.;
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  for(std::size_t i=0;i<path.count;++i)
    path.points[i].d_m=path.points[i].y_m=std::min(2.,std::max(0.,path.points[i].x_m-8.)*.2);
  markOvertakeStart(path);
  auto accepted=cartesianArcField(path);
  auto profile=executionSpeedProfile(accepted);
  for(double consumed:{4.,10.,12.}) {
    r.ego.x_m=consumed; r.ego.y_m=std::max(0.,consumed-8.)*.2;
    const auto remaining=remainingExecutionTrajectory(accepted,r.ego,20.,&profile);
    ASSERT_TRUE(remaining);
    EXPECT_DOUBLE_EQ(remaining->overtake_start_source_s_m,8.);
    // Isolate following eligibility from the changed measured pose/obstacle.
    auto following=*remaining;
    following.overtake_start_source_s_m=INFINITY;
    r.dynamic_obstacles[0].s_m=consumed+3.+2.*c.vehicle_half_length_m;
    const auto planned=planLongitudinalSpeed(*remaining,r,c);
    const auto normal=planLongitudinalSpeed(following,r,c);
    ASSERT_TRUE(planned.valid); ASSERT_TRUE(normal.valid);
    if(consumed<8.)
      EXPECT_DOUBLE_EQ(planned.reference.points[0].speed_mps,normal.reference.points[0].speed_mps);
    else
      EXPECT_GT(planned.reference.points[0].speed_mps,normal.reference.points[0].speed_mps);
    profile=executionSpeedProfile(planned.reference);
    EXPECT_DOUBLE_EQ(profile.overtake_start_source_s_m,8.);
  }
}
TEST_F(LongitudinalTest, ReleasedSpeedStillRequiresCollisionAndWallValidation) {
  for(std::size_t i=0;i<path.count;++i)
    path.points[i].d_m=path.points[i].y_m=std::min(2.,path.points[i].x_m*.2);
  markOvertakeStart(path);
  r.dynamic_obstacles[0].s_m=1.;
  auto evaluated=planAndEvaluateLongitudinalSpeed(ReferenceSpaceMppiPlanner(c),path,r,c);
  EXPECT_EQ(evaluated.evaluated,1U);
  EXPECT_FALSE(evaluated.improved);
  EXPECT_EQ(evaluated.evaluation.reject_reason,RejectReason::COLLISION);
  r.dynamic_obstacle_count=0;
  r.rollout_constraint_validator=[](const auto &,const auto &){return RejectReason::WALL;};
  evaluated=planAndEvaluateLongitudinalSpeed(ReferenceSpaceMppiPlanner(c),path,r,c);
  EXPECT_FALSE(evaluated.improved);
  EXPECT_EQ(evaluated.evaluation.reject_reason,RejectReason::WALL);
}
TEST_F(LongitudinalTest, OnlyGeneratedOvertakingPathsReleaseFollowing) {
  r.dynamic_obstacle_count=0;
  r.nominal_only=true;
  r.nominal.d_pass_m=1.;
  r.bounds.minimum.d_pass_m=r.bounds.maximum.d_pass_m=1.;
  auto scratch=std::make_unique<Scratch>();
  for(const auto phase:{Phase::PREPARE,Phase::OVERTAKE,Phase::MERGE}) {
    r.phase=phase;
    const auto result=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(std::isfinite(result.selected_reference.overtake_start_source_s_m),
              phase==Phase::OVERTAKE);
  }
}
TEST_F(LongitudinalTest, StopAndRestartHaveNoSpeedFloorOrInfiniteArrivalTime) {
  r.dynamic_obstacles[0].longitudinal_speed_mps=0.; r.ego.speed_mps=0.;
  EXPECT_DOUBLE_EQ(command(),0.);
  r.dynamic_obstacles[0].longitudinal_speed_mps=2.;
  EXPECT_GT(command(),0.);
}
TEST_F(LongitudinalTest, MeasuredReplanningConvergesForMovingAndStoppedLeaders) {
  for(double lead_speed:{0.,5.}) {
    double gap=20., speed=5.;
    AwsimLongitudinalResponse plant;
    for(int i=0;i<600;++i) {
      r.ego.speed_mps=speed;
      r.dynamic_obstacles[0].s_m=gap+2.*c.vehicle_half_length_m;
      r.dynamic_obstacles[0].longitudinal_speed_mps=lead_speed;
      const double cmd=command();
      const double a=plant.acceleration(std::clamp(3.*(cmd-speed),-2.,2.),speed,i*.05);
      gap+=(lead_speed-speed)*.05;
      speed=std::max(0.,speed+a*.05);
      EXPECT_GT(gap,2.);
    }
    EXPECT_NEAR(gap,5.,.15);
    EXPECT_NEAR(speed,lead_speed,.1);
  }
}
TEST_F(LongitudinalTest, SpeedPlanningDoesNotCertifyWallOrCollisionFeasibility) {
  const auto planned=planLongitudinalSpeed(path,r,c);
  ASSERT_TRUE(planned.valid);
  r.rollout_constraint_validator=[](const auto &,const auto &){return RejectReason::WALL;};
  EXPECT_EQ(ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(planned.reference,r).reject_reason,
            RejectReason::WALL);
  r.rollout_constraint_validator={};r.dynamic_obstacles[0].s_m=1.;
  const auto blocked=planLongitudinalSpeed(path,r,c);
  EXPECT_EQ(ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(blocked.reference,r).reject_reason,
            RejectReason::COLLISION);
}
TEST_F(LongitudinalTest, MppiSpeedNoiseCannotChangeSelectedPathOrSpeed) {
  r.dynamic_obstacle_count=0;
  r.sample_control_sequence=true; r.sample_lateral_bounds=true;
  r.sample_count_override=10;
  r.nominal.d_pass_m=.5;
  r.bounds.minimum.d_pass_m=.3; r.bounds.maximum.d_pass_m=.8;
  r.bounds.minimum.l_out_m=4.; r.bounds.maximum.l_out_m=12.;
  c.minimum_valid_count=1; c.minimum_valid_ratio=0.;
  auto scratch=std::make_unique<Scratch>();
  const auto first=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  c.sigma[4]=100.; c.control_speed_scale_std=100.;
  const auto second=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  ASSERT_TRUE(first.valid); ASSERT_TRUE(second.valid);
  EXPECT_LE(first.evaluated_sample_count,14U);
  EXPECT_EQ(first.evaluated_sample_count,second.evaluated_sample_count);
  ASSERT_EQ(first.selected_reference.count,second.selected_reference.count);
  for(std::size_t i=0;i<first.selected_reference.count;++i) {
    const auto &a=first.selected_reference.points[i], &b=second.selected_reference.points[i];
    EXPECT_DOUBLE_EQ(a.x_m,b.x_m); EXPECT_DOUBLE_EQ(a.y_m,b.y_m);
    EXPECT_DOUBLE_EQ(a.speed_mps,b.speed_mps);
  }
  for(double speed:second.selected_control_sequence.speed_scale_adjustment)
    EXPECT_DOUBLE_EQ(speed,0.);
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
