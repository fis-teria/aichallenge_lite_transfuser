#include "reference_space_mppi_planner/passing_progress.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
struct PassingFixture : testing::Test {
  Config c;
  PlanRequest r;
  Evaluation rollout(double lateral, double speed=7.0, bool late=false) {
    Evaluation e; e.valid=true; e.predicted_rollout_count=40;
    for (std::size_t i=0;i<40;++i) {
      const double t=(i+1)*.05;
      e.predicted_rollout[i]={speed*t,late ? lateral*t/2.0 : lateral,0.0,speed,t};
    }
    return e;
  }
  void SetUp() override {
    c.cost_passing_progress_weight=c.cost_passing_opportunity_weight=1.0;
    const std::vector<std::array<double,2>> points{{-20,0},{100,0}};
    r.world_reference=std::make_shared<const ReferencePoseIndex>(points,[](auto p){return p;});
    r.ego.speed_mps=7.0; r.passing_entry_m=15.0; r.passing_exit_m=50.0;
    r.passing_speed_mps=8.0; r.dynamic_obstacle_count=1;
    auto &o=r.dynamic_obstacles[0]; o.global_reference_s_m=35.0;
    o.s_m=15.0; o.longitudinal_speed_mps=6.0;
  }
};
TEST_F(PassingFixture, EarlyReadinessWinsBeforeAnyCompletePass) {
  const auto early=passingProgress(r,c,rollout(1.7));
  const auto late=passingProgress(r,c,rollout(1.7,7.0,true));
  EXPECT_GT(early.progress,late.progress);
  EXPECT_GT(early.progress,passingProgress(r,c,rollout(0.0)).progress);
  EXPECT_EQ(r.overtake_target_index,std::numeric_limits<std::size_t>::max());
  EXPECT_GT(early.opportunity,0.0); // entry lies beyond the two-second horizon
}
TEST_F(PassingFixture, WidthRewardSaturatesAndRelativeSpeedMatters) {
  EXPECT_DOUBLE_EQ(passingProgress(r,c,rollout(1.7)).progress,
                   passingProgress(r,c,rollout(3.0)).progress);
  EXPECT_GT(passingProgress(r,c,rollout(1.7,8.0)).opportunity,
            passingProgress(r,c,rollout(1.7,6.0)).opportunity);
  auto turned=rollout(1.7);
  for(auto &s:turned.predicted_rollout)s.yaw_rad=.6;
  EXPECT_LT(passingProgress(r,c,turned).progress,
            passingProgress(r,c,rollout(1.7)).progress);
}
TEST_F(PassingFixture, PhaseAndTargetIdentityDoNotBiasTheSameExecution) {
  const auto e=rollout(1.7); r.phase=Phase::OVERTAKE;
  const auto a=passingProgress(r,c,e); r.phase=Phase::MERGE;
  const auto b=passingProgress(r,c,e);
  EXPECT_DOUBLE_EQ(a.progress,b.progress); EXPECT_DOUBLE_EQ(a.opportunity,b.opportunity);
  r.dynamic_obstacles[1]=r.dynamic_obstacles[0];
  r.dynamic_obstacles[1].global_reference_s_m=5.0; r.dynamic_obstacle_count=2;
  std::swap(r.dynamic_obstacles[0],r.dynamic_obstacles[1]);
  EXPECT_DOUBLE_EQ(a.progress,passingProgress(r,c,e).progress);
}
TEST_F(PassingFixture, ReturningBeforePassingLosesReadinessButNoFrontCarHasNoReward) {
  auto returning=rollout(1.7,7.0,true);
  for(auto &s:returning.predicted_rollout)s.y_m=1.7-s.y_m;
  EXPECT_LT(passingProgress(r,c,returning).opportunity,
            passingProgress(r,c,rollout(1.7)).opportunity);
  r.dynamic_obstacles[0].global_reference_s_m=5.0;
  EXPECT_DOUBLE_EQ(passingProgress(r,c,rollout(1.7)).progress,0.0);
  r.dynamic_obstacle_count=0;
  EXPECT_DOUBLE_EQ(passingProgress(r,c,rollout(1.7)).opportunity,0.0);
}
TEST_F(PassingFixture, ExpiredOpportunityAndInvalidRolloutEarnNothing) {
  auto e=rollout(1.7); e.valid=false;
  EXPECT_DOUBLE_EQ(passingProgress(r,c,e).progress,0.0);
  r.passing_entry_m=-1.0;
  EXPECT_DOUBLE_EQ(passingProgress(r,c,rollout(1.7)).opportunity,0.0);
}
TEST_F(PassingFixture, LateralWidthWithoutRecoverableRelativeDistanceEarnsNothing) {
  const auto centered=passingProgress(r,c,rollout(0.0,6.0));
  const auto wide=passingProgress(r,c,rollout(1.7,6.0));
  EXPECT_NEAR(centered.progress,wide.progress,1e-12);
  EXPECT_NEAR(wide.opportunity,0.0,1e-12);
  const double near_credit=passingProgress(r,c,rollout(1.7)).progress-
      passingProgress(r,c,rollout(0.0)).progress;
  r.dynamic_obstacles[0].global_reference_s_m=55.0;
  const double far_credit=passingProgress(r,c,rollout(1.7)).progress-
      passingProgress(r,c,rollout(0.0)).progress;
  EXPECT_GT(near_credit,far_credit);
  r.passing_entry_m=-1.0;
  EXPECT_NEAR(passingProgress(r,c,rollout(1.7)).progress,
              passingProgress(r,c,rollout(0.0)).progress,1e-12);
}
TEST_F(PassingFixture, OnlyLeaderUsesFutureLapTimingToRewardPreparation) {
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for (int i=0;i<=300;++i) {
    const double t=i*.05;
    prediction->points.push_back({t,35.+6.*std::min(3.,t)+3.*std::max(0.,t-3.),0.,0.});
  }
  r.dynamic_obstacles[0].prediction=prediction;
  const auto e=rollout(1.7,6.);
  EXPECT_NEAR(passingProgress(r,c,e).opportunity,0.,1e-10);
  r.leader_opportunity_index=0;
  r.leader_passing_entry_m=15.;r.leader_passing_exit_m=50.;
  r.leader_preparation_m=0.;
  EXPECT_GT(passingProgress(r,c,e).opportunity,0.);
  EXPECT_GT(passingProgress(r,c,e).opportunity,
      passingProgress(r,c,rollout(0.,6.)).opportunity);
  const double early=passingProgress(r,c,e).opportunity;
  r.leader_preparation_m=15.;
  EXPECT_LT(passingProgress(r,c,e).opportunity,early);
  r.leader_opportunity_index=std::numeric_limits<std::size_t>::max();
  EXPECT_NEAR(passingProgress(r,c,e).opportunity,0.,1e-10);
}

TEST_F(PassingFixture, PredictionWithoutSpecialWindowPreservesRoadOpportunity) {
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for (int i=0;i<=300;++i)
    prediction->points.push_back({i*.05,35.+6.*i*.05,0.,0.});
  r.dynamic_obstacles[0].prediction=prediction;
  const auto e=rollout(1.7,7.);
  const auto ordinary=passingProgress(r,c,e);
  ASSERT_GT(ordinary.opportunity,0.);
  r.leader_opportunity_index=0;
  const auto without_window=passingProgress(r,c,e);
  EXPECT_DOUBLE_EQ(without_window.progress,ordinary.progress);
  EXPECT_DOUBLE_EQ(without_window.opportunity,ordinary.opportunity);
}

TEST_F(PassingFixture, TerminalLeaderValueUsesItsRoadProfileAndInterveningCurve) {
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for (int i=0;i<=300;++i)
    prediction->points.push_back({i*.05,35.+4.*i*.05,0.,0.});
  r.dynamic_obstacles[0].prediction=prediction;
  r.leader_opportunity_index=0;
  r.leader_passing_entry_m=15.;r.leader_passing_exit_m=60.;r.leader_preparation_m=0.;
  std::vector<PassingRoadSample> road;
  for(int i=0;i<=80;++i)road.push_back({double(i),8.,0.});
  r.leader_passing_road=std::make_shared<const PassingRoadProfile>(passingRoadProfile(road,c));
  const auto e=rollout(1.7,7.);
  const double flat=passingProgress(r,c,e).opportunity;
  ASSERT_GT(flat,0.);
  r.passing_speed_mps=3.; // Advice for a different, generic straight.
  EXPECT_DOUBLE_EQ(passingProgress(r,c,e).opportunity,flat);
  for(auto &p:road)if(p.distance>=20.)p.curvature=1.;
  r.leader_passing_road=std::make_shared<const PassingRoadProfile>(passingRoadProfile(road,c));
  EXPECT_LT(passingProgress(r,c,e).opportunity,flat);
}
} // namespace reference_space_mppi_planner::mppi
