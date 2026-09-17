#include "reference_space_mppi_planner/leader_lap_prediction.hpp"
#include "reference_space_mppi_planner/leader_passing_opportunity.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner {
namespace {
TEST(LeaderLapPrediction, ReplaysVariableTraversalTimeAcrossWrap) {
  std::vector<std::array<double,2>> circle;
  constexpr double radius=20.;
  const double length=2.*M_PI*radius;
  for (int i=0;i<=1000;++i) {
    const double a=2.*M_PI*i/1000.;
    circle.push_back({radius*std::cos(a),radius*std::sin(a)});
  }
  ReferencePoseIndex world(circle,[](const auto &p){return p;});
  const auto angle=[](double t){return 2.*M_PI*t/20.+.35*std::sin(2.*M_PI*t/20.);};
  std::vector<opponent_prediction::PositionObservation> samples;
  for (int i=0;i<=2100;++i) {
    const double t=i*.02,a=angle(t);
    samples.push_back({t,radius*std::cos(a),radius*std::sin(a),radius*a});
  }
  const auto prediction=opponent_prediction::buildLeaderLapPrediction(samples,length,42.,15.,world);
  ASSERT_TRUE(prediction);
  for (double t=0.;t<15.;t+=.137) {
    const auto p=prediction->at(t);
    const auto xy=world.pose(p.global_s,p.d);
    ASSERT_TRUE(xy);
    EXPECT_NEAR((*xy)[0],radius*std::cos(angle(42.+t)),.015);
    EXPECT_NEAR((*xy)[1],radius*std::sin(angle(42.+t)),.015);
  }
  EXPECT_GT(prediction->at(1.).global_s-prediction->at(0.).global_s,
            prediction->at(8.).global_s-prediction->at(7.).global_s+2.);
  auto short_history=samples;
  short_history.erase(short_history.begin(),short_history.begin()+1500);
  EXPECT_FALSE(opponent_prediction::buildLeaderLapPrediction(short_history,length,42.,15.,world));
  auto dropout=samples;
  dropout.erase(dropout.begin()+1400,dropout.begin()+1500);
  EXPECT_FALSE(opponent_prediction::buildLeaderLapPrediction(dropout,length,42.,15.,world));
}

TEST(LeaderOpportunity, UsesLeaderArrivalAndReservesGentlePreparation) {
  std::vector<mppi::PassingRoadSample> road;
  for (int i=0;i<=80;++i) road.push_back({double(i),9.,i<20?.05:0.});
  mppi::DynamicObstacle leader;leader.s_m=8.;leader.d_m=0.;
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for (int i=0;i<=300;++i) prediction->points.push_back({i*.05,8.+4.*i*.05,0.,0.});
  leader.prediction=prediction;
  mppi::Config config;config.maximum_acceleration_mps2=2.;
  config.maximum_lateral_acceleration_mps2=4.;
  mppi::EgoState ego;ego.speed_mps=5.;
  const auto window=mppi::selectLeaderPassingOpportunity(road,leader,ego,config,1.5);
  EXPECT_DOUBLE_EQ(window.entry,20.);
  EXPECT_DOUBLE_EQ(window.exit,80.);
  EXPECT_GE(window.preparation,0.);
  EXPECT_LT(window.preparation,window.entry);
  EXPECT_GT(window.pass_time,2.);
  EXPECT_GE(window.pass_distance,window.entry);
  EXPECT_LE(window.pass_distance,window.exit);
  EXPECT_GE(window.pass_distance,
      opponent_prediction::positionAt(leader,window.pass_time).s+2.*config.vehicle_half_length_m);
  for (int i=0;i<=300;++i) prediction->points[i].global_s=50.+10.*i*.05;
  leader.s_m=50.;
  EXPECT_LT(mppi::selectLeaderPassingOpportunity(road,leader,ego,config,1.5).entry,0.);
  leader.prediction.reset();
  EXPECT_LT(mppi::selectLeaderPassingOpportunity(road,leader,ego,config,1.5).entry,0.);
}

TEST(LeaderOpportunity, TerminalRecoveryUsesAchievedSpeedAndUnwrappedLeaderProgress) {
  mppi::Config config;config.maximum_acceleration_mps2=2.;
  std::vector<mppi::PassingRoadSample> road;
  for(int i=0;i<=80;++i)road.push_back({double(i),9.,0.});
  const auto profile=mppi::passingRoadProfile(road,config);
  mppi::DynamicObstacle leader;leader.s_m=15.;
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  for(int i=0;i<=300;++i)prediction->points.push_back({i*.05,330.+5.*i*.05,0.,0.});
  leader.prediction=prediction;
  const double fast=mppi::leaderPassingRecovery(profile,leader,15.,8.,2.,20.,45.,config);
  const double slow=mppi::leaderPassingRecovery(profile,leader,15.,2.,2.,20.,45.,config);
  EXPECT_GT(fast,slow);
  EXPECT_GT(fast,0.);
  EXPECT_DOUBLE_EQ(mppi::leaderPassingRecovery(profile,leader,45.,8.,2.,20.,45.,config),0.);
  for(auto &p:prediction->points)p.global_s-=330.;
  EXPECT_DOUBLE_EQ(mppi::leaderPassingRecovery(profile,leader,15.,8.,2.,20.,45.,config),fast);
}
} // namespace
} // namespace reference_space_mppi_planner
