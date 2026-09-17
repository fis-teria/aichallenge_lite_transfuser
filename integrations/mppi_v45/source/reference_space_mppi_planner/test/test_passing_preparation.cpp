#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include "reference_space_mppi_planner/leader_passing_opportunity.hpp"
#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include "reference_space_mppi_planner/side_selection.hpp"
#include "reference_space_mppi_planner/execution_motion.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
std::vector<opponent_prediction::PositionObservation> history(double pace=1.,double offset=0.) {
  std::vector<opponent_prediction::PositionObservation> out;
  for(int i=0;i<=2100;++i) {
    const double t=i*.02,phase=t<=40. ? t : 40.+(t-40.)*pace;
    const double angle=2.*M_PI*phase/20.;
    out.push_back({t,20.*std::cos(angle),20.*std::sin(angle)+(t>40.?offset:0.),20.*angle});
  }
  return out;
}
TEST(PreparationMatch, RequiresRouteAndPaceAgreement) {
  PassingPreparationConfig c;
  const auto matched=matchPriorLap(history(),40.*M_PI,c);
  ASSERT_TRUE(matched.usable);
  EXPECT_NEAR(matched.pace_error_ratio,0.,1e-10);
  EXPECT_LT(matched.position_error_m,1e-6);
  const auto pace=matchPriorLap(history(1.5),40.*M_PI,c);
  EXPECT_FALSE(pace.usable);EXPECT_STREQ(pace.reason,"pace_mismatch");
  const auto route=matchPriorLap(history(1.,1.),40.*M_PI,c);
  EXPECT_FALSE(route.usable);EXPECT_STREQ(route.reason,"route_mismatch");
}
TEST(PreparationMatch, MissingCoverageAndNonmonotonicRecentDataDoNotMatch) {
  auto data=history();
  data.erase(data.begin(),data.end()-20);
  EXPECT_FALSE(matchPriorLap(data,40.*M_PI,{}).usable);
  data=history();data.back().station=data[data.size()-2].station;
  EXPECT_FALSE(matchPriorLap(data,40.*M_PI,{}).usable);
}

class PreparationTest : public testing::Test {
protected:
  Config c;
  PlanRequest r;
  TemporaryReference geometry;
  PriorLapMatch match;
  void SetUp() override {
    c.enabled=true;c.shadow_only=false;c.longitudinal_planning_enabled=true;
    c.speed_proportional_gain=3.;c.maximum_acceleration_mps2=2.;
    c.maximum_deceleration_mps2=2.;c.maximum_lateral_acceleration_mps2=4.;
    c.awsim_vehicle_response_enabled=true;c.collision_only_rejection=true;
    c.cost_passing_opportunity_weight=1.;c.horizon_steps=40;
    r.valid=true;r.side=1;r.sample_staged_speed=true;r.ego.speed_mps=5.;r.stamp_sec=42.;
    std::vector<std::array<double,2>> points;
    std::vector<PassingRoadSample> road;
    for(int i=0;i<=100;++i) points.push_back({double(i),0.});
    r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
    for(int i=0;i<=80;++i) road.push_back({double(i),9.,i<20?.05:0.});
    r.leader_passing_road=std::make_shared<PassingRoadProfile>(passingRoadProfile(road,c));
    auto prediction=std::make_shared<opponent_prediction::Prediction>();
    for(int i=0;i<=300;++i) prediction->points.push_back({i*.05,8.+4.*i*.05,0.,0.});
    r.leader_passing_prediction=prediction;r.leader_opportunity_index=0;r.dynamic_obstacle_count=1;
    r.dynamic_obstacles[0].s_m=8.;r.dynamic_obstacles[0].prediction=prediction;
    r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
    match.usable=true;match.reason="matched";
    r.base_reference_count=geometry.count=101;
    for(std::size_t i=0;i<geometry.count;++i) {
      auto &b=r.base_reference[i];b.s_m=b.x_m=.4*i;b.speed_mps=9.;
      b.minimum_d_m=-10.;b.maximum_d_m=10.;
      auto &p=geometry.points[i];p.s_m=p.source_s_m=p.x_m=.4*i;
      p.speed_mps=p.uncapped_speed_mps=9.;
    }
  }
  void executionTracking() {
    // Full-maneuver tests use the same tracking profile as the SIM executor.
    c.wheel_base_m=1.087;c.lookahead_gain=.2;c.lookahead_min_distance_m=2.;
    c.curvature_lookahead_min_distance_m=2.;c.actual_lookahead_distance_blend=1.;
    c.continuous_preview_interpolation_enabled=true;
    c.steering_time_constant_sec=.02;c.steering_control_delay_sec=.07;
    c.steering_command_to_tire_angle_ratio=.6;c.maximum_tire_steering_angle_rad=M_PI/10.;
    c.steering_gain=1.45;c.curvature_feedforward_enabled=true;c.curvature_feedforward_gain=.6;
    c.compare_cma_delay_compensation=c.compare_cma_preview_feedforward=true;
    c.cma_lookahead_curvature_enabled=true;c.steering_acceleration_hold_maximum_acceleration_mps2=1.;
  }
  std::shared_ptr<const PassingPreparationPlan> goal(std::uint64_t revision=1,
      const std::shared_ptr<const PassingPreparationPlan> &previous={}) {
    return makePassingPreparationGoal(r,c,1.5,"d3",match,revision,previous);
  }
};

TEST_F(PreparationTest, PlansAchievableWindowWithPhysicalAccelerationAndSeparateStarts) {
  const auto p=goal();ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->entry_station_m,20.);EXPECT_DOUBLE_EQ(p->exit_station_m,80.);
  EXPECT_LT(p->lateral_start_station_m,p->entry_station_m);
  EXPECT_LE(p->acceleration_start_time_sec,p->pass_time_sec);
  EXPECT_GT(p->pass_time_sec,0.);EXPECT_LE(p->pass_station_m,80.);
  EXPECT_GE(p->pass_station_m,8.+4.*p->pass_time_sec+2.*c.vehicle_half_length_m);
  EXPECT_GT(p->schedule.size(),2U);
  for(std::size_t i=1;i<p->schedule.size();++i) {
    EXPECT_GT(p->schedule[i].time_sec,p->schedule[i-1].time_sec);
    EXPECT_GE(p->schedule[i].station_m,p->schedule[i-1].station_m);
    EXPECT_LE(p->schedule[i].acceleration_mps2,1.);
  }
}
TEST_F(PreparationTest, MissingMatchOrTooFastLeaderCannotInventPassingPoint) {
  match.usable=false;EXPECT_FALSE(goal());match.usable=true;
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for(int i=0;i<=300;++i) prediction->points.push_back({i*.05,8.+10.*i*.05,0.,0.});
  r.leader_passing_prediction=prediction;
  EXPECT_FALSE(goal());
}

TEST_F(PreparationTest, PreparationCarriesWallLibraryAndRejectsReplacedMapGeometry) {
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;
  r.precomputed_wall_lines=lines;
  const auto plan=goal();ASSERT_TRUE(plan);EXPECT_EQ(plan->wall_lines,lines);
  applyPassingPreparation(r,plan);ASSERT_EQ(r.passing_preparation,plan);
  auto replacement=std::make_shared<PrecomputedWallLines>(*lines);
  r.precomputed_wall_lines=replacement;
  applyPassingPreparation(r,plan);EXPECT_FALSE(r.passing_preparation);
  auto newer=*plan;newer.wall_lines=replacement;
  EXPECT_FALSE(samePreparationWindow(*plan,newer));
}
TEST_F(PreparationTest, ReplanningCarriesOutgoingAccelerationAtTheScheduleOrigin) {
  r.dynamic_obstacles[0].s_m=20.;
  for(int i=0;i<4;++i) {
    r.ego.acceleration_mps2=0.;
    r.passing_preparation=goal(i+1);
    ASSERT_TRUE(r.passing_preparation);
    EXPECT_GT(r.passing_preparation->schedule.front().acceleration_mps2,0.);
    const auto speed=planLongitudinalSpeed(geometry,r,c,true);
    ASSERT_TRUE(speed.valid);
    EXPECT_GT(c.speed_proportional_gain*(speed.reference.points[0].speed_mps-r.ego.speed_mps)
        -(.37+.03*r.ego.speed_mps),0.);
    r.stamp_sec+=.05;
  }
}
TEST_F(PreparationTest, PreparationSpeedIsConsideredBeforeRawGeometryIsRejected) {
  r.dynamic_obstacle_count=0;r.phase=Phase::OVERTAKE;r.nominal_only=true;
  r.nominal={0.,8.,8.,8.,1.};r.bounds.minimum=r.bounds.maximum=r.nominal;
  r.rollout_constraint_validator=[](const auto &,const auto &b) {
    return b.x_m>11. ? RejectReason::WALL : RejectReason::NONE;
  };
  auto scratch=std::make_unique<Scratch>();
  const auto normal=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  ASSERT_FALSE(normal.valid);
  auto plan=std::make_shared<PassingPreparationPlan>();
  plan->world=r.world_reference;plan->epoch_sec=r.stamp_sec;
  for(int i=0;i<=300;++i)plan->schedule.push_back({i*.05,3.*i*.05,3.,0.});
  r.passing_preparation=plan;
  const auto prepared=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  EXPECT_TRUE(prepared.valid);
}
TEST_F(PreparationTest, DiagnosticReasonsDistinguishMissingDataWindowsAndArrival) {
  PreparationDiagnostics diag;
  const auto check=[&] {return makePassingPreparationGoal(r,c,1.5,"d3",match,1,{},&diag);};
  ASSERT_TRUE(check());EXPECT_STREQ(diag.reason,"planned");EXPECT_GT(diag.trials,0U);
  const auto road=r.leader_passing_road;
  r.leader_passing_road.reset();EXPECT_FALSE(check());EXPECT_STREQ(diag.reason,"missing_road");
  auto curved=std::make_shared<PassingRoadProfile>(*road);
  for(auto &point:curved->samples)point.curvature=.1;
  r.leader_passing_road=curved;
  EXPECT_FALSE(check());EXPECT_STREQ(diag.reason,"no_passing_window");
  r.leader_passing_road=road;r.dynamic_obstacles[0].s_m=100.;
  auto fast=std::make_shared<opponent_prediction::Prediction>();
  for(int i=0;i<=300;++i)fast->points.push_back({i*.05,100.+12.*i*.05,0.,0.});
  r.leader_passing_prediction=fast;
  EXPECT_FALSE(check());EXPECT_STREQ(diag.reason,"no_improving_preparation");
  EXPECT_GT(diag.best_body_deficit_m,0.);EXPECT_GT(diag.window_exited+diag.horizon_exhausted+diag.road_exhausted,0U);
}
TEST_F(PreparationTest, ImprovingExitSpeedCanPrepareWithoutClaimingACompletedPass) {
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for(int i=0;i<=300;++i)prediction->points.push_back({i*.05,40.+7.*i*.05,0.,0.});
  r.leader_passing_prediction=prediction;r.dynamic_obstacles[0].prediction=prediction;
  r.dynamic_obstacles[0].s_m=40.;r.dynamic_obstacles[0].longitudinal_speed_mps=7.;
  const auto plan=goal();ASSERT_TRUE(plan);
  EXPECT_FALSE(plan->complete_pass);
  EXPECT_EQ(plan->pass_time_sec,-1.);EXPECT_EQ(plan->pass_station_m,-1.);
  EXPECT_GT(plan->relative_gain_m,0.);EXPECT_GT(plan->terminal_relative_speed_mps,0.);
  EXPECT_GT(plan->targetTimeSec(),plan->lateral_start_time_sec);
  EXPECT_LE(plan->targetStationM(),plan->exit_station_m);
  applyPassingPreparation(r,plan);ASSERT_TRUE(r.passing_preparation);
  EXPECT_EQ(r.leader_pass_distance_m,-1.);EXPECT_EQ(r.leader_pass_time_sec,-1.);
  EXPECT_TRUE(preparationSearchDue(*plan,plan->epoch_sec+plan->lateral_start_time_sec,.1));
  EXPECT_FALSE(preparationSearchDue(*plan,plan->epoch_sec+plan->targetTimeSec()+.1,.1));
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0].speed_mps=r.ego.speed_mps;
  const auto expected=preparationTarget(*plan,r.stamp_sec+2.);ASSERT_TRUE(expected);
  e.predicted_rollout[1].time_sec=2.;e.predicted_rollout[1].x_m=expected->station_m;
  e.predicted_rollout[1].speed_mps=expected->speed_mps;
  const auto refined=refinePassingPreparation(r,c,e,17);ASSERT_TRUE(refined);
  EXPECT_FALSE(refined->complete_pass);EXPECT_EQ(refined->pass_time_sec,-1.);
  EXPECT_GT(refined->targetTimeSec(),2.);EXPECT_EQ(refined->geometry_generation,17U);
  r.stamp_sec=plan->epoch_sec+plan->targetTimeSec()+.1;
  applyPassingPreparation(r,plan);EXPECT_FALSE(r.passing_preparation);
}
TEST_F(PreparationTest, ScheduleReadinessIsRewardedBeforeLateralDeparture) {
  r.dynamic_obstacles[0].s_m=20.;r.passing_preparation=goal();ASSERT_TRUE(r.passing_preparation);
  const auto target=preparationTarget(*r.passing_preparation,r.stamp_sec+.5);ASSERT_TRUE(target);
  ASSERT_LT(target->station_m,r.passing_preparation->lateral_start_station_m);
  Evaluation e;e.valid=true;e.predicted_rollout_count=1;
  auto &state=e.predicted_rollout[0];state.time_sec=.5;state.x_m=target->station_m;
  state.speed_mps=target->speed_mps;
  const double matched=preparationProgress(r,c,e);EXPECT_GT(matched,0.);
  state.x_m+=5.;state.speed_mps+=2.;
  EXPECT_LT(preparationProgress(r,c,e),matched);
}
TEST_F(PreparationTest, SameWindowRetainsIdentityAndReprojectsDistances) {
  const auto first=goal(9);ASSERT_TRUE(first);
  const auto next=goal(10,first);ASSERT_TRUE(next);
  EXPECT_EQ(next->id,9U);EXPECT_EQ(next->revision,10U);
  r.ego.x_m=5.;r.stamp_sec+=.5;
  applyPassingPreparation(r,first);ASSERT_TRUE(r.passing_preparation);
  EXPECT_DOUBLE_EQ(r.leader_passing_entry_m,15.);
  EXPECT_DOUBLE_EQ(r.leader_pass_distance_m,first->pass_station_m-5.);
  EXPECT_NEAR(r.leader_pass_time_sec,first->pass_time_sec-.5,1e-12);
  r.stamp_sec=100.;applyPassingPreparation(r,first);EXPECT_FALSE(r.passing_preparation);
}
TEST_F(PreparationTest, ConnectionSearchBeginsWithOpportunityAndExpiresWithoutAdoption) {
  auto p=goal();ASSERT_TRUE(p);
  auto copy=*p;copy.lateral_start_time_sec=3.;
  EXPECT_TRUE(preparationSearchDue(copy,copy.epoch_sec,.2));
  EXPECT_TRUE(preparationSearchDue(copy,copy.epoch_sec+2.,.2));
  EXPECT_TRUE(preparationSearchDue(copy,copy.epoch_sec+2.9,.2));
  EXPECT_FALSE(preparationSearchDue(copy,copy.epoch_sec+copy.pass_time_sec+1.,.2));
  EXPECT_FALSE(preparationTarget(copy,copy.epoch_sec-1.));
  EXPECT_FALSE(preparationTarget(copy,copy.epoch_sec+copy.schedule.back().time_sec+1.));
}
TEST_F(PreparationTest, ScheduleChangesSpeedButPreservesGeometryAndFollowing) {
  auto p=std::make_shared<PassingPreparationPlan>(*goal());
  for(auto &point:p->schedule) {
    point.speed_mps=3.;point.station_m=point.time_sec*3.;point.acceleration_mps2=0.;
  }
  r.passing_preparation=p;r.dynamic_obstacle_count=0;
  const auto baseline=planLongitudinalSpeed(geometry,r,c);
  const auto prepared=planLongitudinalSpeed(geometry,r,c,true);
  ASSERT_TRUE(baseline.valid);ASSERT_TRUE(prepared.valid);
  EXPECT_LT(prepared.reference.points[0].speed_mps,baseline.reference.points[0].speed_mps);
  EXPECT_DOUBLE_EQ(prepared.reference.points[20].x_m,geometry.points[20].x_m);
  r.dynamic_obstacle_count=1;r.dynamic_obstacles[0].prediction.reset();
  r.dynamic_obstacles[0].s_m=3.;r.dynamic_obstacles[0].longitudinal_speed_mps=0.;
  const auto blocked=planLongitudinalSpeed(geometry,r,c,true);
  ASSERT_TRUE(blocked.valid);
  EXPECT_LT(blocked.reference.points[0].speed_mps,r.ego.speed_mps);
}
TEST_F(PreparationTest, TwoSpeedVariantsAreEvaluatedUsingCommonExecutionModel) {
  r.dynamic_obstacle_count=0;
  auto p=std::make_shared<PassingPreparationPlan>();p->world=r.world_reference;p->epoch_sec=r.stamp_sec;
  for(int i=0;i<=300;++i) p->schedule.push_back({i*.05,3.*i*.05,3.,0.});
  r.passing_preparation=p;
  const auto choice=planAndEvaluateLongitudinalSpeed(ReferenceSpaceMppiPlanner(c),geometry,r,c);
  EXPECT_EQ(choice.evaluated,2U);EXPECT_GT(choice.valid,0U);EXPECT_TRUE(choice.evaluation.valid);
  r.passing_preparation.reset();
  EXPECT_EQ(planAndEvaluateLongitudinalSpeed(ReferenceSpaceMppiPlanner(c),geometry,r,c).evaluated,1U);
}
TEST_F(PreparationTest, OwnedConnectionSurvivesTimingExpiryAndRechecksTargetAndWorld) {
  r.passing_preparation=goal();ASSERT_TRUE(r.passing_preparation);
  geometry.wall_connection_end_index=10;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={0.,0.,0.,5.,0.};
  e.predicted_rollout[1]={20.,2.,0.,7.,3.};
  const auto owned=bindPreparationConnection(r,c,e,geometry,-1,52);
  ASSERT_TRUE(owned);ASSERT_TRUE(owned->connection);
  EXPECT_EQ(owned->side,-1);
  EXPECT_DOUBLE_EQ(owned->connection_end_station_m,geometry.points[10].x_m);
  EXPECT_EQ(owned->connection->count,geometry.count);
  EXPECT_FALSE(pastPreparationConnectionEnd(*owned,r));
  r.ego.x_m=owned->connection_end_station_m+.1;
  EXPECT_TRUE(pastPreparationConnectionEnd(*owned,r));
  r.ego.x_m=0.;
  r.stamp_sec=owned->epoch_sec+owned->targetTimeSec()+1.;
  EXPECT_TRUE(preparationSearchDue(*owned,r.stamp_sec,.1));
  EXPECT_TRUE(preparationConnectionCurrent(*owned,r,"d3"));
  applyPassingPreparation(r,owned);ASSERT_TRUE(r.passing_preparation);
  // Losing the future improving window cannot erase this certified shape.
  r.leader_passing_road.reset();
  auto refreshed=refinePassingPreparation(r,c,e,52);ASSERT_TRUE(refreshed);
  EXPECT_EQ(refreshed->connection,owned->connection);
  EXPECT_EQ(refreshed->side,-1);EXPECT_FALSE(refreshed->complete_pass);
  EXPECT_EQ(refreshed->pass_time_sec,-1.);
  EXPECT_FALSE(preparationConnectionCurrent(*owned,r,"another"));
  r.ego.x_m=owned->exit_station_m+.1;
  EXPECT_FALSE(preparationConnectionCurrent(*owned,r,"d3"));
  r.ego.x_m=0.;r.world_reference.reset();
  EXPECT_FALSE(preparationConnectionCurrent(*owned,r,"d3"));
}
TEST_F(PreparationTest, ConnectionAdoptionRequiresValidatedGeometryAndFreezesOnlyExtension) {
  r.passing_preparation=goal();ASSERT_TRUE(r.passing_preparation);
  geometry.wall_connection_end_index=10;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={0.,0.,0.,5.,0.};
  e.predicted_rollout[1]={20.,2.,0.,7.,3.};
  const auto first=bindPreparationConnection(r,c,e,geometry,1,52);ASSERT_TRUE(first);
  r.passing_preparation=first;geometry.wall_connection_end_index=15;
  const auto extended=bindPreparationConnection(r,c,e,geometry,1,53,true);ASSERT_TRUE(extended);
  EXPECT_EQ(extended->connection,first->connection);
  EXPECT_EQ(extended->connection_end_station_m,first->connection_end_station_m);
  const auto replaced=bindPreparationConnection(r,c,e,geometry,1,54);ASSERT_TRUE(replaced);
  EXPECT_NE(replaced->connection,first->connection);
  EXPECT_DOUBLE_EQ(replaced->connection_end_station_m,geometry.points[15].x_m);
  e.valid=false;
  EXPECT_FALSE(bindPreparationConnection(r,c,e,geometry,-1,55));
  EXPECT_FALSE(refinePassingPreparation(r,c,e,55));
}
TEST_F(PreparationTest, RejectsConnectionThatWouldBeCancelledBeforeItsOwnEndpoint) {
  auto opportunity=std::make_shared<PassingPreparationPlan>(*goal());
  opportunity->entry_station_m=27.681171;opportunity->exit_station_m=61.681171;
  r.passing_preparation=opportunity;
  geometry.wall_connection_end_index=90;
  geometry.points[90].x_m=81.113948;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={43.200367,0.,0.,8.9,0.};
  e.predicted_rollout[1]={60.,2.,0.,9.,2.};
  // Recorded first v25 H2H: re-entry started at s=43.2 but finished at
  // s=81.1, beyond the s=61.7 opportunity; it returned before arrival.
  EXPECT_FALSE(bindPreparationConnection(r,c,e,geometry,-1,2329));
  geometry.points[90].x_m=49.780989;
  EXPECT_TRUE(bindPreparationConnection(r,c,e,geometry,-1,2261));
}
TEST_F(PreparationTest, ChecksWholeConnectionEvenWhenLocalRolloutEndsBeforeTheJoin) {
  r.passing_preparation=goal();ASSERT_TRUE(r.passing_preparation);
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={0.,0.,0.,5.,0.};
  e.predicted_rollout[1]={10.,0.,0.,5.,2.};
  r.path_constraint_validator=[](const TemporaryReference &p) {
    return p.points[p.count-1].x_m>=38. ? RejectReason::WALL : RejectReason::NONE;
  };
  geometry.wall_connection_end_index=100; // Join at s=40, beyond the rollout.
  RejectReason rejection=RejectReason::NONE;
  EXPECT_FALSE(bindPreparationConnection(r,c,e,geometry,-1,1,false,&rejection));
  EXPECT_EQ(rejection,RejectReason::WALL);
  geometry.wall_connection_end_index=90; // Join at s=36; unrelated suffix is not the connection.
  EXPECT_TRUE(bindPreparationConnection(r,c,e,geometry,-1,2,false,&rejection));
  EXPECT_EQ(rejection,RejectReason::NONE);
}
TEST_F(PreparationTest, AdoptedCompletionIsUpdatedFromActualExecutionState) {
  r.passing_preparation=goal();ASSERT_TRUE(r.passing_preparation);
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={0.,2.,0.,5.,0.};
  e.predicted_rollout[1]={20.,2.,0.,7.,3.};
  const auto adopted=refinePassingPreparation(r,c,e,52);
  ASSERT_TRUE(adopted);
  EXPECT_EQ(adopted->geometry_generation,52U);
  EXPECT_DOUBLE_EQ(adopted->validated_until_sec,45.);
  EXPECT_DOUBLE_EQ(adopted->schedule[1].station_m,20.);
  EXPECT_DOUBLE_EQ(adopted->schedule[1].speed_mps,7.);
  EXPECT_GE(adopted->pass_time_sec,3.);
  EXPECT_LE(adopted->pass_station_m,adopted->exit_station_m);
  e.valid=false;EXPECT_FALSE(refinePassingPreparation(r,c,e,53));
}
TEST_F(PreparationTest, FrontMergeGoalNeedsNoPriorLapAndRequiresSpeedAdvantage) {
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;
  r.precomputed_wall_lines=lines;r.passing_speed_mps=9.;
  r.leader_passing_prediction.reset();r.leader_passing_road.reset();
  const auto p=makeFrontMergeGoal(r,c,"d3",1);ASSERT_TRUE(p);
  EXPECT_TRUE(p->front_merge);
  EXPECT_GT(p->merge_end_station_m,p->merge_start_station_m);
  EXPECT_GT(p->exit_station_m,p->merge_end_station_m+20.);
  EXPECT_GE(p->merge_start_station_m,8.+4.*p->pass_time_sec+2.*c.vehicle_half_length_m);
  r.dynamic_obstacles[0].longitudinal_speed_mps=10.;
  EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",2));
}

TEST_F(PreparationTest, FrontMergeSearchesNextAllowedPointForBothPredictionModes) {
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=100.;
  for(int i=0;i<=100;++i)
    lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  r.precomputed_wall_lines=lines;
  const auto prediction=r.leader_passing_prediction;
  for(bool prior:{false,true}) {
    r.leader_passing_prediction=prior ? prediction : nullptr;
    r.passing_point_areas.reset();
    const auto unrestricted=makeFrontMergeGoal(r,c,"d3",1);ASSERT_TRUE(unrestricted);
    EXPECT_LT(unrestricted->pass_station_m,50.);
    PassingPointAreaConfig config;config.straight_start_xy={50.,0.};config.corner_entry_xy={70.,0.};
    const auto areas=passingPointAreas(*r.world_reference,100.,false,config);ASSERT_TRUE(areas);
    r.passing_point_areas=std::make_shared<PassingPointAreas>(*areas);
    const auto restricted=makeFrontMergeGoal(r,c,"d3",2);ASSERT_TRUE(restricted);
    EXPECT_EQ(restricted->prior_lap_guidance,prior);
    EXPECT_GE(restricted->pass_station_m,50.);EXPECT_LE(restricted->pass_station_m,65.);
    EXPECT_GT(restricted->pass_time_sec,unrestricted->pass_time_sec);
    EXPECT_DOUBLE_EQ(restricted->merge_start_station_m,restricted->pass_station_m);
    // The ego can start before the configured placement area.
    EXPECT_FALSE(areas->contains(restricted->origin_station_m));
    r.passing_point_areas=std::make_shared<PassingPointAreas>();
    EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",3));
  }
}

TEST_F(PreparationTest, AllowedAreaBehindEgoDoesNotInventAForwardPointOnOpenRoad) {
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=100.;
  r.precomputed_wall_lines=lines;r.leader_passing_prediction.reset();
  PassingPointAreaConfig config;config.straight_start_xy={0.,0.};config.corner_entry_xy={10.,0.};
  const auto areas=passingPointAreas(*r.world_reference,100.,false,config);ASSERT_TRUE(areas);
  r.passing_point_areas=std::make_shared<PassingPointAreas>(*areas);
  EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",1));
}

TEST_F(PreparationTest, MultiplePassingGoalsCarrySeparateTimedSchedulesAndRespectAreas) {
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=100.;
  for(int i=0;i<=100;++i) lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  r.precomputed_wall_lines=lines;
  const auto prediction=r.leader_passing_prediction;
  for(bool prior:{false,true}) {
    r.leader_passing_prediction=prior ? prediction : nullptr;
    r.passing_point_areas.reset();
    const auto goals=makeFrontMergeGoals(r,c,"d3",17);ASSERT_EQ(goals.size(),3U);
    for(std::size_t i=0;i<goals.size();++i) {
      EXPECT_EQ(goals[i]->side,0);EXPECT_EQ(goals[i]->prior_lap_guidance,prior);
      EXPECT_GE(goals[i]->pass_time_sec,goals.front()->pass_time_sec+i);
      EXPECT_GE(goals[i]->schedule.back().station_m,goals[i]->pass_station_m);
      if(i) {EXPECT_GT(goals[i]->pass_station_m,goals[i-1]->pass_station_m);EXPECT_NE(goals[i]->id,goals[i-1]->id);}
    }
    auto areas=std::make_shared<PassingPointAreas>();areas->allowed={{50.,51.}};areas->length_m=100.;
    r.passing_point_areas=areas;
    const auto restricted=makeFrontMergeGoals(r,c,"d3",18);ASSERT_EQ(restricted.size(),1U);
    EXPECT_TRUE(areas->contains(restricted.front()->pass_station_m));
    r.passing_point_areas=std::make_shared<PassingPointAreas>();
    EXPECT_TRUE(makeFrontMergeGoals(r,c,"d3",19).empty());
  }
}

TemporaryReference frontMergePath(double origin=0.,double radius=0.) {
  TemporaryReference path;path.count=161;path.front_merge_end_index=120;
  const auto blend=[](double x) {x=std::clamp(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x));};
  for(std::size_t i=0;i<path.count;++i) {
    auto &p=path.points[i];const double s=.5*i;
    p.d_m=3.*blend(s/12.)*(1.-blend((s-35.)/25.));
    p.x_m=origin+s;p.y_m=p.d_m;
    if(radius>0.) {
      const double angle=(origin+s)/radius;
      p.x_m=(radius-p.d_m)*std::cos(angle);p.y_m=(radius-p.d_m)*std::sin(angle);
    }
    p.s_m=p.source_s_m=s;p.speed_mps=p.uncapped_speed_mps=9.;
  }
  for(std::size_t i=0;i<path.count;++i) {
    const auto &a=path.points[i?i-1:i],&b=path.points[std::min(i+1,path.count-1)];
    path.points[i].yaw_rad=std::atan2(b.y_m-a.y_m,b.x_m-a.x_m);
  }
  return path;
}

TEST_F(PreparationTest, NoYieldChecksTheWholeMergeAndEveryOpponent) {
  executionTracking();
  r.front_merge_attack=true;r.dynamic_obstacles[0].global_reference_s_m=8.;
  auto path=frontMergePath();
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::NONE);
  r.dynamic_obstacles[0].longitudinal_speed_mps=9.;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::COLLISION);
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  r.dynamic_obstacle_count=2;r.dynamic_obstacles[1].global_reference_s_m=55.;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::COLLISION);
  r.dynamic_obstacle_count=1;path.front_merge_end_index=0;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::GEOMETRY);
  r.front_merge_attack=false;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::NONE);
}

TEST_F(PreparationTest, FrontMergeCompletionIncludesTheExecutionAccelerationHold) {
  executionTracking();
  r.front_merge_attack=true;r.dynamic_obstacles[0].global_reference_s_m=8.;
  const auto path=frontMergePath();
  ASSERT_EQ(validateFrontMerge(r,c,path),RejectReason::NONE);
  c.steering_demand_acceleration_hold_enabled=true;
  c.steering_acceleration_hold_minimum_speed_mps=0.;
  c.steering_acceleration_hold_minimum_tire_angle_rad=0.;
  c.steering_acceleration_hold_tracking_error_rad=0.;
  c.steering_acceleration_hold_maximum_acceleration_mps2=0.;
  // The executor coasts instead of accelerating. The long-range comparison
  // must not keep the pass that was possible with unrestricted propulsion.
  EXPECT_NE(validateFrontMerge(r,c,path),RejectReason::NONE);
}

TEST_F(PreparationTest, FrontMergeChecksTheTrackedWallPathBeyondTheLocalHorizon) {
  executionTracking();
  r.front_merge_attack=true;r.dynamic_obstacles[0].global_reference_s_m=8.;
  r.path_constraint_geometry_only=true;
  r.path_constraint_validator=[](const TemporaryReference &path) {
    for(std::size_t i=0;i<path.count;++i)
      if(path.points[i].x_m>40.) return RejectReason::WALL;
    return RejectReason::NONE;
  };
  EXPECT_EQ(validateFrontMerge(r,c,frontMergePath()),RejectReason::WALL);
}

TEST_F(PreparationTest, PriorLapFrontMergeChecksRouteAndSpeedThroughReturn) {
  executionTracking();
  r.front_merge_attack=true;r.dynamic_obstacles[0].global_reference_s_m=8.;
  auto plan=std::make_shared<PassingPreparationPlan>();plan->prior_lap_guidance=true;
  r.passing_preparation=plan;
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  prediction->points={{0.,8.,0.,0.},{20.,88.,0.,0.}};
  r.leader_passing_prediction=prediction;r.dynamic_obstacles[0].longitudinal_speed_mps=9.;
  const auto path=frontMergePath();
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::NONE);
  prediction->points={{0.,8.,3.,0.},{20.,88.,3.,0.}};
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::COLLISION);
  prediction->points={{0.,8.,0.,0.},{20.,208.,0.,0.}};
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::COLLISION);
  prediction->points={{0.,8.,0.,0.},{.2,8.8,0.,0.}};
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::INVALID_INPUT);
  r.leader_passing_prediction.reset();
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::INVALID_INPUT);
}

TEST_F(PreparationTest, CurrentCollisionStillRejectsPriorLapGuidance) {
  r.front_merge_attack=true;
  auto plan=std::make_shared<PassingPreparationPlan>();plan->prior_lap_guidance=true;
  r.passing_preparation=plan;
  r.dynamic_obstacles[0].s_m=r.dynamic_obstacles[0].global_reference_s_m=0.;
  r.dynamic_obstacles[0].prediction.reset();r.dynamic_obstacles[0].longitudinal_speed_mps=0.;
  const auto evaluation=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(frontMergePath(),r);
  EXPECT_FALSE(evaluation.valid);EXPECT_EQ(evaluation.reject_reason,RejectReason::COLLISION);
}

TEST_F(PreparationTest, FrontMergeOwnershipCoversReturnAndKeepsMeasuredValidationLimit) {
  auto p=std::make_shared<PassingPreparationPlan>(*goal());
  p->front_merge=true;p->merge_start_station_m=35.;p->merge_end_station_m=60.;p->exit_station_m=90.;
  r.passing_preparation=p;
  auto path=frontMergePath();path.wall_connection_end_index=24;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;e.predicted_rollout[1].time_sec=2.;
  std::size_t checked=0;
  r.path_constraint_validator=[&](const auto &prefix) {checked=prefix.count;return RejectReason::NONE;};
  const auto owned=bindPreparationConnection(r,c,e,path,1,1);ASSERT_TRUE(owned);
  EXPECT_EQ(checked,121U);EXPECT_DOUBLE_EQ(owned->connection_end_station_m,60.);
  EXPECT_DOUBLE_EQ(owned->validated_until_sec,r.stamp_sec+2.);
  r.ego.x_m=50.;r.dynamic_obstacles[0].s_m=-10.;
  EXPECT_TRUE(preparationConnectionCurrent(*owned,r,"d3"));
  EXPECT_FALSE(preparationConnectionCurrent(*owned,r,"another"));
  r.path_constraint_validator=[](const auto &prefix) {
    return prefix.points[prefix.count-1].x_m>50. ? RejectReason::WALL : RejectReason::NONE;
  };
  EXPECT_FALSE(bindPreparationConnection(r,c,e,path,1,2));
}

TEST_F(PreparationTest, RetimingCannotAdoptAnUnboundFrontMergeGoal) {
  auto p=std::make_shared<PassingPreparationPlan>(*goal());
  p->front_merge=true;p->exit_station_m=90.;r.passing_preparation=p;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;e.predicted_rollout[1].time_sec=2.;
  EXPECT_FALSE(refinePassingPreparation(r,c,e,1));
  auto path=frontMergePath();path.wall_connection_end_index=24;
  const auto owned=bindPreparationConnection(r,c,e,path,1,2);ASSERT_TRUE(owned);
  r.passing_preparation=owned;
  // Restricting new placements must not cancel a connection already being executed.
  r.passing_point_areas=std::make_shared<PassingPointAreas>();
  const auto retimed=refinePassingPreparation(r,c,e,3);ASSERT_TRUE(retimed);
  EXPECT_EQ(retimed->connection,owned->connection);
  EXPECT_DOUBLE_EQ(retimed->connection_end_station_m,60.);
}

TEST_F(PreparationTest, NoYieldFrontMergeCrossesClosedReferenceSeam) {
  executionTracking();
  constexpr double radius=100.;
  std::vector<std::array<double,2>> points;
  for(int i=0;i<=1200;++i) {
    const double angle=2.*M_PI*i/1200.;points.push_back({radius*std::cos(angle),radius*std::sin(angle)});
  }
  r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
  const double origin=2.*M_PI*radius-20.;
  const auto path=frontMergePath(origin,radius);
  r.front_merge_attack=true;r.ego.x_m=path.points[0].x_m;r.ego.y_m=path.points[0].y_m;
  r.ego.yaw_rad=path.points[0].yaw_rad;
  r.dynamic_obstacles[0].global_reference_s_m=origin+8.;
  EXPECT_EQ(validateFrontMerge(r,c,path),RejectReason::NONE);
}

TEST_F(PreparationTest, CandidateIncludesInnerPassingLineReturnAndValidatedTail) {
  executionTracking();
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=100.;
  for(int i=0;i<=100;++i)
    lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  r.precomputed_wall_lines=lines;r.front_merge_attack=r.complete_maneuver=true;
  r.connect_to_wall_line=r.cartesian_measured_prefix=true;r.pass_profile_scale_m=3.;
  r.phase=Phase::OVERTAKE;r.nominal_only=true;r.nominal={1.8,12.,25.,25.,1.};
  r.bounds.minimum=r.bounds.maximum=r.nominal;
  r.dynamic_obstacles[0].global_reference_s_m=8.;
  r.base_reference_count=161;
  for(std::size_t i=0;i<r.base_reference_count;++i) {
    auto &b=r.base_reference[i];b.s_m=b.x_m=.5*i;b.speed_mps=9.;
    b.minimum_d_m=-5.;b.maximum_d_m=5.;
  }
  r.path_constraint_execution_prefix=true;
  std::size_t checked=0;
  r.path_constraint_validator=[&](const auto &p) {checked=p.count;return RejectReason::NONE;};
  auto scratch=std::make_unique<Scratch>();
  const auto result=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  ASSERT_TRUE(result.valid) << result.selected_evaluation.reject_stage;
  const auto &path=result.selected_reference;
  EXPECT_NEAR(path.points[50].y_m,1.8,1e-9);
  ASSERT_GT(path.front_merge_end_index,0U);
  EXPECT_NEAR(path.points[path.front_merge_end_index].y_m,0.,1e-9);
  EXPECT_GT(path.points[path.count-1].x_m-path.points[path.front_merge_end_index].x_m,15.);
  EXPECT_EQ(checked,path.front_merge_end_index+1U);

  // Bind the actual preparation, rather than only manually chosen connector
  // lengths: its existing wall line and return must pass the moving leader.
  r.passing_preparation=makeFrontMergeGoal(r,c,"d3",1);
  ASSERT_TRUE(r.passing_preparation);
  const auto prepared=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
  ASSERT_TRUE(prepared.valid) << prepared.selected_evaluation.reject_stage;
  const auto &prepared_path=prepared.selected_reference;
  const auto wall_index=static_cast<std::size_t>(std::ceil(r.passing_preparation->entry_station_m*2.))+1U;
  EXPECT_NEAR(prepared_path.points[wall_index].y_m,3.,1e-9);
  EXPECT_NEAR(prepared_path.points[prepared_path.front_merge_end_index].x_m,
      r.passing_preparation->merge_end_station_m,.5);
  EXPECT_EQ(validateFrontMerge(r,c,prepared_path),RejectReason::NONE);
}

TEST_F(PreparationTest, LaterPassingPointSurvivesAcceleratingDiagonalLeaderAndBindsItsOwnReturn) {
  executionTracking();
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=100.;
  for(int i=0;i<=100;++i) lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  r.precomputed_wall_lines=lines;r.front_merge_attack=r.complete_maneuver=true;r.compare_passing_points=true;
  r.connect_to_wall_line=r.cartesian_measured_prefix=true;r.pass_profile_scale_m=3.;
  r.phase=Phase::OVERTAKE;r.nominal_only=true;r.nominal={1.8,12.,25.,25.,1.};
  r.bounds.minimum=r.bounds.maximum=r.nominal;r.dynamic_obstacles[0].global_reference_s_m=8.;
  r.base_reference_count=201;
  for(std::size_t i=0;i<r.base_reference_count;++i) {
    auto &b=r.base_reference[i];b.s_m=b.x_m=.5*i;b.speed_mps=9.;b.minimum_d_m=-5.;b.maximum_d_m=5.;
  }
  // After four seconds the leader accelerates for one second while moving
  // one metre across the straight. An early return intersects its motion.
  auto prediction=std::make_shared<opponent_prediction::Prediction>();double station=8.;
  for(int j=0;j<=400;++j) {
    const double t=j*.05,phase=std::clamp(t-4.,0.,1.);
    prediction->points.push_back({t,station,std::sin(M_PI*phase),0.});
    station+=.05*(t>=4. && t<5. ? 12. : 4.);
  }
  r.leader_passing_prediction=prediction;r.dynamic_obstacles[0].prediction=prediction;
  const auto goals=makeFrontMergeGoals(r,c,"d3",1);ASSERT_EQ(goals.size(),3U);
  auto scratch=std::make_unique<Scratch>();
  std::array<SideSelectionCandidate,6> candidates{};
  std::array<std::shared_ptr<const PassingPreparationPlan>,6> owned{};
  for(std::size_t option=0;option<3;++option) for(int side:{1,-1}) {
    const auto slot=option*2+(side<0 ? 1 : 0);
    r.side=side;r.nominal.d_pass_m=side*1.8;r.pass_profile_scale_m=side*3.;
    r.bounds.minimum=r.bounds.maximum=r.nominal;applyPassingPreparation(r,goals[option]);
    const auto result=ReferenceSpaceMppiPlanner(c).plan(r,scratch.get());
    const auto &e=result.selected_evaluation;
    if(option==0) {
      EXPECT_FALSE(result.valid);EXPECT_EQ(e.reject_reason,RejectReason::COLLISION);
      EXPECT_STREQ(e.reject_stage,"front_merge_no_yield");continue;
    }
    ASSERT_TRUE(result.valid) << e.reject_stage;
    EXPECT_TRUE(std::isfinite(e.merge_completion_time_sec));
    EXPECT_GE(e.merge_braking_mps,0.);EXPECT_GT(e.merge_alongside_clearance_m,0.);
    owned[slot]=bindPreparationConnection(r,c,e,result.selected_reference,side,10+slot);
    ASSERT_TRUE(owned[slot]);
    EXPECT_EQ(owned[slot]->id,goals[option]->id);
    EXPECT_DOUBLE_EQ(owned[slot]->pass_station_m,goals[option]->pass_station_m);
    EXPECT_NEAR(owned[slot]->connection_end_station_m,goals[option]->merge_end_station_m,.5);
    candidates[slot]={side,true,e.cost,e.overtake_time_sec,e.merge_completion_time_sec,
        e.merge_braking_mps,e.merge_alongside_clearance_m};
  }
  const auto selected=selectSideCandidate(candidates,candidates.size(),0,0.);
  ASSERT_TRUE(selected);EXPECT_GE(*selected,2U);ASSERT_TRUE(owned[*selected]);
  EXPECT_GT(owned[*selected]->pass_station_m,goals[0]->pass_station_m);
}

TEST_F(PreparationTest, SharedMotionKeepsThePublishedCommandUntilActivation) {
  executionTracking();
  auto prior=geometry;
  for(std::size_t i=0;i<prior.count;++i) prior.points[i].speed_mps=prior.points[i].uncapped_speed_mps=2.;
  const auto collect=[&](const TemporaryReference &path,const PlanRequest &request,std::size_t n) {
    std::vector<RolloutState> states;
    EXPECT_TRUE(visitExecutionMotion(path,request,c,n,[&](const ExecutionMotionSample &s) {
      states.push_back(s.after);return true;
    }));
    return states;
  };
  const auto slow=collect(prior,r,40),fast=collect(geometry,r,40);
  r.prior_execution_reference=std::make_shared<const TemporaryReference>(prior);
  r.reference_activation_delay_sec=.5;
  const auto delayed=collect(geometry,r,40),longer=collect(geometry,r,160);
  ASSERT_EQ(delayed.size(),40U);ASSERT_GE(longer.size(),delayed.size());
  for(std::size_t i=0;i<delayed.size();++i) {
    EXPECT_DOUBLE_EQ(delayed[i].x_m,longer[i].x_m);
    EXPECT_DOUBLE_EQ(delayed[i].speed_mps,longer[i].speed_mps);
    if(delayed[i].time_sec<=.5+1e-9) {
      EXPECT_DOUBLE_EQ(delayed[i].x_m,slow[i].x_m);
      EXPECT_DOUBLE_EQ(delayed[i].speed_mps,slow[i].speed_mps);
    }
  }
  EXPECT_GT(delayed.back().speed_mps,slow.back().speed_mps);
  EXPECT_LT(delayed.back().x_m,fast.back().x_m);
}

TEST_F(PreparationTest, FrontMergeOwnershipUnwrapsMoreThanHalfALap) {
  std::vector<std::array<double,2>> points;
  for(int i=0;i<=600;++i) {
    const double a=2.*M_PI*i/600.;points.push_back({20.*std::cos(a),20.*std::sin(a)});
  }
  r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
  auto plan=std::make_shared<PassingPreparationPlan>();plan->world=r.world_reference;
  plan->front_merge=true;plan->origin_station_m=100.;plan->exit_station_m=220.;
  r.passing_preparation=plan;
  auto path=frontMergePath(100.,20.);path.front_merge_end_index=150;path.wall_connection_end_index=24;
  Evaluation e;e.valid=true;e.predicted_rollout_count=2;e.predicted_rollout[1].time_sec=2.;
  const auto owned=bindPreparationConnection(r,c,e,path,1,1);ASSERT_TRUE(owned);
  EXPECT_GT(owned->connection_end_station_m-owned->connection_start_station_m,74.);
  EXPECT_NEAR(owned->connection_end_station_m,175.,.1);
  const auto &end=path.points[path.front_merge_end_index];
  r.ego.x_m=end.x_m;r.ego.y_m=end.y_m;
  const auto station=preparationStation(*owned,r.ego.x_m,r.ego.y_m);ASSERT_TRUE(station);
  EXPECT_NEAR(*station,175.,.1);
  EXPECT_TRUE(pastPreparationConnectionEnd(*owned,r));
  applyPassingPreparation(r,owned);ASSERT_TRUE(r.passing_preparation);
  EXPECT_NEAR(r.leader_passing_exit_m,45.,.1);
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
