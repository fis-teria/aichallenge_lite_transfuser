#include "reference_space_mppi_planner/recent_motion_prediction.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::opponent_prediction {
TEST(RecentMotion, AccelerationExpiresWithoutLosingItsVelocityChange) {
  const RecentMotion initial{0.,0.,4.,0.,-2.,0.};
  const auto state=advanceRecentMotion(initial,0.,2.,{.5,0.});
  EXPECT_NEAR(state.speed,3.,1e-12);
  EXPECT_NEAR(state.x,6.25,1e-12);
  EXPECT_DOUBLE_EQ(state.y,0.);
}
TEST(RecentMotion, TurnPersistenceIsIndependentOfAcceleration) {
  const RecentMotion initial{0.,0.,4.,0.,-2.,1.};
  const auto state=advanceRecentMotion(initial,0.,2.,{0.,.5});
  EXPECT_NEAR(state.speed,4.,1e-12);
  EXPECT_NEAR(state.yaw,.5,1e-12);
  EXPECT_NEAR(state.x,4.*std::sin(.5)+6.*std::cos(.5),1e-12);
  EXPECT_NEAR(state.y,4.*(1.-std::cos(.5))+6.*std::sin(.5),1e-12);
}
TEST(RecentMotion, AlignmentDoesNotRestartPersistence) {
  const RecentMotion initial{0.,0.,3.,.2,-1.7,.4};
  const MotionPersistence duration{.35,.6};
  const auto aligned=advanceRecentMotion(initial,0.,.25,duration);
  const auto split=advanceRecentMotion(aligned,.25,2.,duration);
  const auto direct=advanceRecentMotion(initial,0.,2.,duration);
  EXPECT_NEAR(split.x,direct.x,1e-10);EXPECT_NEAR(split.y,direct.y,1e-10);
  EXPECT_NEAR(split.speed,direct.speed,1e-12);EXPECT_NEAR(split.yaw,direct.yaw,1e-12);
}
TEST(RecentMotion, BrakingStopsWithoutReverseMotionOrRotationInPlace) {
  const RecentMotion initial{0.,0.,.1,0.,-3.,.5};
  const auto state=advanceRecentMotion(initial,0.,4.,{2.,2.});
  const auto later=advanceRecentMotion(state,4.,8.,{2.,2.});
  EXPECT_DOUBLE_EQ(state.speed,0.);EXPECT_DOUBLE_EQ(later.x,state.x);
  EXPECT_DOUBLE_EQ(later.y,state.y);EXPECT_DOUBLE_EQ(later.yaw,state.yaw);
}
TEST(RecentMotion, FittedForecastUsesObservedEpochAndRetainsSharedRepresentation) {
  const std::vector<std::array<double,2>> line{{-20.,0.},{0.,0.},{20.,0.},{40.,0.}};
  ReferencePoseIndex world(line,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(int i=0;i<=12;++i){const double t=-.6+i*.05;history.push_back({10.+t,5.+3.*t-t*t,0.,0.});}
  const auto p=buildRecentMotionPrediction(history,10.25,2.,world,{.5,0.});
  ASSERT_TRUE(p);EXPECT_DOUBLE_EQ(p->source_stamp,10.);
  EXPECT_NEAR(p->vx,2.5,1e-9);
  const auto a=world.pose(p->at(0.).global_s,p->at(0.).d);
  const auto b=world.pose(p->at(2.).global_s,p->at(2.).d);
  ASSERT_TRUE(a);ASSERT_TRUE(b);
  EXPECT_NEAR((*a)[0],5.6875,1e-9);EXPECT_NEAR((*b)[0],9.75,1e-9);
  EXPECT_FALSE(buildRecentMotionPrediction(history,9.9,2.,world,{.5,0.}));
  history.resize(2);EXPECT_FALSE(buildRecentMotionPrediction(history,10.,2.,world,{.5,0.}));
}
TEST(RecentMotion, ConstantSpeedTurnDoesNotCreateTangentialAcceleration) {
  for(double radius:{5.,8.,20.}) {
    std::vector<PositionObservation> history;
    for(int i=0;i<=12;++i) {
      const double t=-.6+i*.05,a=7.*t/radius;
      history.push_back({t,radius*std::sin(a),radius*(1.-std::cos(a)),0.});
    }
    const auto state=fitRecentMotion(history);ASSERT_TRUE(state);
    EXPECT_NEAR(state->speed,7.,.002);
    EXPECT_NEAR(state->acceleration,0.,1e-10);
  }
}
TEST(RecentMotion, IntervalMidpointsRecoverAccelerationWithIrregularSampling) {
  std::vector<PositionObservation> history;
  for(double t:{-.6,-.51,-.39,-.35,-.22,-.05,0.})
    history.push_back({10.+t,20.+5.*t-1.5*t*t,0.,0.});
  const auto state=fitRecentMotion(history);ASSERT_TRUE(state);
  EXPECT_NEAR(state->speed,5.,1e-10);EXPECT_NEAR(state->acceleration,-3.,1e-10);
}
} // namespace reference_space_mppi_planner::opponent_prediction
