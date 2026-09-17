#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include <gtest/gtest.h>
namespace reference_space_mppi_planner::mppi {
namespace {
class PriorLapCandidateTest : public testing::Test {
protected:
  Config c;PlanRequest r;
  void SetUp() override {
    c.enabled=true;c.shadow_only=false;c.awsim_vehicle_response_enabled=true;
    c.maximum_acceleration_mps2=2.;c.maximum_deceleration_mps2=2.;c.speed_proportional_gain=3.;
    r.valid=true;r.ego.speed_mps=5.;r.stamp_sec=100.;r.leader_opportunity_index=0;
    r.dynamic_obstacle_count=1;r.dynamic_obstacles[0].s_m=8.;
    r.dynamic_obstacles[0].global_reference_s_m=8.;r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
    std::vector<std::array<double,2>> points;
    for(int i=0;i<=300;++i) points.push_back({double(i),0.});
    r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
    auto lines=std::make_shared<PrecomputedWallLines>();lines->world=r.world_reference;lines->length_m=300.;
    for(int i=0;i<=300;++i) lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
    r.precomputed_wall_lines=lines;r.base_reference_count=101;
    for(std::size_t i=0;i<r.base_reference_count;++i) {
      auto &p=r.base_reference[i];p.s_m=p.x_m=.4*i;p.speed_mps=9.;
      p.minimum_d_m=-10.;p.maximum_d_m=10.;
    }
  }
  void prior(double d,double speed=4.,double horizon=20.) {
    auto p=std::make_shared<opponent_prediction::Prediction>();p->source_stamp=99.9;
    for(double t=0.;t<=horizon+1e-9;t+=.05) p->points.push_back({t,8.+speed*t,d,0.});
    r.leader_passing_prediction=p;
  }
};
TEST_F(PriorLapCandidateTest, PriorLeftAndRightRoutesChooseOppositeCandidates) {
  prior(2.8);const auto left=makeFrontMergeGoal(r,c,"d3",1);
  ASSERT_TRUE(left);EXPECT_TRUE(left->prior_lap_guidance);EXPECT_EQ(left->side,-1);
  EXPECT_LT(left->candidate_minimum_center_distance_m[0],0.);
  EXPECT_GE(left->candidate_minimum_center_distance_m[1],0.);
  EXPECT_DOUBLE_EQ(left->prior_lap_source_stamp_sec,99.9);
  prior(-2.8);const auto right=makeFrontMergeGoal(r,c,"d3",2);
  ASSERT_TRUE(right);EXPECT_EQ(right->side,1);
}
TEST_F(PriorLapCandidateTest, PriorSpeedOverridesCurrentSpeedForPassEstimate) {
  r.dynamic_obstacles[0].longitudinal_speed_mps=10.;
  prior(2.8,4.);ASSERT_TRUE(makeFrontMergeGoal(r,c,"d3",1));
  r.dynamic_obstacles[0].longitudinal_speed_mps=4.;
  prior(2.8,10.);EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",2));
}
TEST_F(PriorLapCandidateTest, PriorDecelerationEnablesLaterPass) {
  auto p=std::make_shared<opponent_prediction::Prediction>();
  for(double t=0.;t<=20.;t+=.05)
    p->points.push_back({t,8.+10.*std::min(t,2.)+3.*std::max(0.,t-2.),2.8,0.});
  r.leader_passing_prediction=p;r.dynamic_obstacles[0].longitudinal_speed_mps=10.;
  const auto goal=makeFrontMergeGoal(r,c,"d3",1);
  ASSERT_TRUE(goal);EXPECT_GT(goal->pass_time_sec,2.);EXPECT_EQ(goal->side,-1);
}
TEST_F(PriorLapCandidateTest, ValidPreparedSideSurvivesChangedRelativePreference) {
  prior(.3);const auto first=makeFrontMergeGoal(r,c,"d3",1);ASSERT_TRUE(first);
  prior(-.3);const auto fresh=makeFrontMergeGoal(r,c,"d3",2);ASSERT_TRUE(fresh);
  EXPECT_NE(first->side,fresh->side);
  const auto kept=makeFrontMergeGoal(r,c,"d3",2,first);ASSERT_TRUE(kept);
  EXPECT_EQ(kept->side,first->side);EXPECT_EQ(kept->id,first->id);EXPECT_EQ(kept->revision,2U);
}
TEST_F(PriorLapCandidateTest, InterferingPreparedSideAndNewTargetAreReplanned) {
  prior(2.8);const auto first=makeFrontMergeGoal(r,c,"d3",1);ASSERT_TRUE(first);
  prior(-2.8);const auto moved=makeFrontMergeGoal(r,c,"d3",2,first);ASSERT_TRUE(moved);
  EXPECT_EQ(moved->side,1);EXPECT_EQ(moved->id,2U);
  prior(-.3);const auto changed=makeFrontMergeGoal(r,c,"d2",3,first);ASSERT_TRUE(changed);
  EXPECT_EQ(changed->side,1);EXPECT_EQ(changed->target_id,"d2");EXPECT_EQ(changed->id,3U);
}
TEST_F(PriorLapCandidateTest, ShortHistoryDoesNotClampOrSubstituteConstantSpeed) {
  prior(2.8,4.,.2);EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",1));
  r.leader_passing_prediction.reset();const auto fallback=makeFrontMergeGoal(r,c,"d3",2);
  ASSERT_TRUE(fallback);EXPECT_FALSE(fallback->prior_lap_guidance);EXPECT_EQ(fallback->side,0);
}
TEST_F(PriorLapCandidateTest, RouteCrossingBothCandidatesRejectsPreparation) {
  auto p=std::make_shared<opponent_prediction::Prediction>();
  for(double t=0.;t<=20.;t+=.05) p->points.push_back({t,8.+4.*t,t<3.5 ? -3. : 3.,0.});
  r.leader_passing_prediction=p;
  EXPECT_FALSE(makeFrontMergeGoal(r,c,"d3",1));
}
TEST_F(PriorLapCandidateTest, PriorPlanningLeavesCurrentCollisionPredictionUnchanged) {
  auto current=std::make_shared<opponent_prediction::Prediction>();
  current->points={{0.,8.,0.,0.},{2.,20.,0.,0.}};
  r.dynamic_obstacles[0].prediction=current;prior(2.8);
  ASSERT_TRUE(makeFrontMergeGoal(r,c,"d3",1));
  EXPECT_EQ(r.dynamic_obstacles[0].prediction,current);
  EXPECT_DOUBLE_EQ(r.dynamic_obstacles[0].prediction->points.back().global_s,20.);
}
TEST_F(PriorLapCandidateTest, OppositeSideCannotBindToPreparedPriorCandidate) {
  prior(2.8);r.passing_preparation=makeFrontMergeGoal(r,c,"d3",1);
  ASSERT_TRUE(r.passing_preparation);Evaluation e;e.valid=true;e.predicted_rollout_count=1;
  TemporaryReference geometry;geometry.count=2;
  EXPECT_FALSE(bindPreparationConnection(r,c,e,geometry,1,2));
}
}
} // namespace reference_space_mppi_planner::mppi
