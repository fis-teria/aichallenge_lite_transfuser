#include "reference_space_mppi_planner/driving_fsm.hpp"
#include "reference_space_mppi_planner/maneuver_policy.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace reference_space_mppi_planner {

TEST(DrivingFsm, EarlyAvoidanceUsesFiveKmhBoundaryAndKeepsMovingVehicleGap) {
  DrivingFsm fsm;
  for(double speed:{0.,5./3.6,std::nextafter(5./3.6,INFINITY),
                    20./3.6,std::nextafter(20./3.6,INFINITY)}) {
    for(double gap:{-1.,4.999,5.,5.001,15.,35.}) {
      SCOPED_TRACE(::testing::Message()<<speed<<" "<<gap);
      const auto decision=fsm.plan({true,true,true,gap,speed});
      EXPECT_EQ(decision.generate_lateral,
          speed<=20./3.6 && (speed<=5./3.6 || gap<=5.));
      if(decision.generate_lateral) { EXPECT_EQ(decision.maneuver,DrivingMode::AVOID); }
    }
  }
  EXPECT_EQ(fsm.mode(),DrivingMode::FREE_RUN);
}

TEST(DrivingFsm, EarlyAvoidanceRequiresFiniteObservationAndPathConflict) {
  DrivingFsm fsm;
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  const auto infinity=std::numeric_limits<double>::infinity();
  for(double invalid:{nan,infinity,-infinity}) {
    EXPECT_FALSE(fsm.plan({true,true,true,invalid,0.}).generate_lateral);
    EXPECT_FALSE(fsm.plan({true,true,true,15.,invalid}).generate_lateral);
  }
  EXPECT_FALSE(fsm.plan({true,false,true,15.,0.}).generate_lateral);
  EXPECT_FALSE(fsm.plan({true,true,true,{},0.}).generate_lateral);
  EXPECT_FALSE(fsm.plan({true,true,true,15.,{}}).generate_lateral);
}

TEST(DrivingFsm, EarlyAvoidanceContinuesAtDistanceAndReturnsOnSpeedBoundary) {
  DrivingFsm fsm;fsm.acceptLateral();
  for(double speed:{0.,5./3.6}) {
    const auto decision=fsm.plan({true,true,false,15.,speed});
    EXPECT_TRUE(decision.generate_lateral);
    EXPECT_FALSE(decision.generate_return);
    EXPECT_EQ(decision.maneuver,DrivingMode::AVOID);
  }
  const auto faster=fsm.plan({true,true,false,15.,std::nextafter(5./3.6,INFINITY)});
  EXPECT_FALSE(faster.generate_lateral);
  EXPECT_TRUE(faster.generate_return);
  EXPECT_EQ(fsm.mode(),DrivingMode::AVOID);
  EXPECT_TRUE(fsm.plan({true,false,false}).generate_return);
}

TEST(DrivingFsm, FastAndUnknownLeadersCannotStartUnpreparedAvoidance) {
  DrivingFsm fsm;
  for(double gap:{1.,5.,15.}) {
    EXPECT_FALSE(fsm.plan({false,true,true,gap}).generate_lateral);
    EXPECT_FALSE(fsm.plan({false,true,true,gap,20./3.6+1e-6}).generate_lateral);
    EXPECT_FALSE(fsm.plan({true,true,true,gap,8.}).generate_lateral);
  }
  EXPECT_FALSE(fsm.plan({false,false,true}).generate_lateral);
}

TEST(DrivingFsm, FastFollowingClosesToTwoMetresAndSlowFollowingKeepsFive) {
  DrivingFsm fsm;
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{5.001,3.}),10.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{5.,3.}),3.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{2.001,7.}),10.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{2.,7.}),7.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{1.5,7.}),7.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{1.,0.}),0.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,std::nullopt),10.);
  EXPECT_DOUBLE_EQ(fsm.freeRunSpeed(10.,FollowObservation{4.,4.,5.7}),10.);
}

TEST(DrivingFsm, SearchDoesNotCommitModeAndAdoptionCommitsDistinctModes) {
  DrivingFsm fsm;
  EXPECT_EQ(fsm.plan({true,true,true,4.,2.}).maneuver,DrivingMode::AVOID);
  EXPECT_EQ(fsm.mode(),DrivingMode::FREE_RUN);
  fsm.acceptLateral();
  EXPECT_STREQ(fsm.modeName(),"AVOID");
  const auto avoiding=fsm.revision();
  fsm.acceptLateral();EXPECT_EQ(fsm.revision(),avoiding);
  fsm.acceptLateral(DrivingMode::OVERTAKE);
  EXPECT_STREQ(fsm.modeName(),"OVERTAKE");
  EXPECT_GT(fsm.revision(),avoiding);
}

TEST(DrivingFsm, ActiveAvoidanceReturnsWhenGapExceedsFiveEvenIfReferenceObstructed) {
  DrivingFsm fsm;fsm.acceptLateral();
  EXPECT_TRUE(fsm.plan({true,true,false,5.,2.}).generate_lateral);
  const auto returning=fsm.plan({true,true,false,5.001,2.});
  EXPECT_FALSE(returning.generate_lateral);
  EXPECT_TRUE(returning.generate_return);
  EXPECT_FALSE(returning.finish_avoid);
  EXPECT_TRUE(fsm.plan({true,true,true,12.,2.}).finish_avoid);
  EXPECT_EQ(fsm.mode(),DrivingMode::AVOID);
  fsm.finishAvoid();EXPECT_STREQ(fsm.modeName(),"FREE_RUN");
}

TEST(DrivingFsm, ActiveAvoidanceReturnsWhenTargetSpeedsUpOrDisappears) {
  DrivingFsm fsm;fsm.acceptLateral();
  EXPECT_TRUE(fsm.plan({true,true,false,3.,20./3.6+1e-6}).generate_return);
  EXPECT_TRUE(fsm.plan({true,false,false}).generate_return);
  EXPECT_FALSE(fsm.plan({false,false,true}).finish_avoid);
}

TEST(DrivingFsm, OvertakeRequiresFastTargetPlanAndDepartureTime) {
  DrivingFsm fsm;
  DrivingScene scene{true,true,true,12.,7.,false,true,true};
  EXPECT_FALSE(fsm.plan(scene).generate_lateral);
  scene.preparation_search_due=true;
  const auto prepared=fsm.plan(scene);
  EXPECT_TRUE(prepared.generate_lateral);
  EXPECT_EQ(prepared.maneuver,DrivingMode::OVERTAKE);
  scene.preparation_available=false;
  EXPECT_FALSE(fsm.plan(scene).generate_lateral);
  scene.preparation_available=true;scene.overtake_target_valid=false;
  EXPECT_FALSE(fsm.plan(scene).generate_lateral);
}

TEST(DrivingFsm, SlowVehicleTakesPriorityOverOtherPreparedFastTarget) {
  DrivingFsm fsm;
  const auto avoid=fsm.plan({true,true,true,4.,2.,true,true,true});
  EXPECT_EQ(avoid.maneuver,DrivingMode::AVOID);
  EXPECT_FALSE(fsm.plan({true,true,true,12.,2.,true,true,true}).generate_lateral);
}

TEST(DrivingFsm, AdoptedOvertakeContinuesThenReturnsWhenGoalOrTargetIsLost) {
  DrivingFsm fsm;fsm.acceptLateral(DrivingMode::OVERTAKE);
  EXPECT_TRUE(fsm.plan({true,true,false,12.,7.,false,true,true}).generate_lateral);
  EXPECT_TRUE(fsm.plan({true,true,false,12.,7.,false,true,false}).generate_return);
  EXPECT_TRUE(fsm.plan({true,false,false,{}, {},false,false,true}).generate_return);
  EXPECT_TRUE(fsm.plan({true,false,true}).finish_avoid);
  const auto slow=fsm.plan({true,true,false,3.,2.});
  EXPECT_EQ(slow.maneuver,DrivingMode::OVERTAKE);
  EXPECT_TRUE(slow.generate_return);
  EXPECT_FALSE(slow.generate_lateral);
}

TEST(DrivingFsm, AdoptedOvertakeNeverReclassifiesSlowOrStoppedLeadersAsAvoidance) {
  DrivingFsm fsm;fsm.acceptLateral(DrivingMode::OVERTAKE);
  const auto revision=fsm.revision();
  for(double speed:{0.,2.,20./3.6}) for(double gap:{-1.54,0.,3.,5.}) {
    for(bool prepared:{false,true}) {
      SCOPED_TRACE(::testing::Message()<<speed<<" "<<gap<<" "<<prepared);
      const auto decision=fsm.plan({false,true,false,gap,speed,true,prepared,prepared});
      EXPECT_EQ(decision.maneuver,DrivingMode::OVERTAKE);
      EXPECT_FALSE(decision.generate_lateral);
      EXPECT_TRUE(decision.generate_return);
      EXPECT_FALSE(decision.finish_avoid);
      EXPECT_EQ(fsm.mode(),DrivingMode::OVERTAKE);
      EXPECT_EQ(fsm.revision(),revision);
    }
  }
}

TEST(DrivingFsm, CompletedOvertakeAllowsNewSlowAvoidanceOnlyAfterModeRelease) {
  DrivingFsm fsm;fsm.acceptLateral(DrivingMode::OVERTAKE);
  EXPECT_TRUE(fsm.plan({true,false,true}).finish_avoid);
  EXPECT_EQ(fsm.plan({true,true,false,3.,2.}).maneuver,DrivingMode::OVERTAKE);
  fsm.finishAvoid();
  const auto next=fsm.plan({true,true,false,3.,2.});
  EXPECT_EQ(fsm.mode(),DrivingMode::FREE_RUN);
  EXPECT_TRUE(next.generate_lateral);
  EXPECT_EQ(next.maneuver,DrivingMode::AVOID);
  fsm.acceptLateral(next.maneuver);
  EXPECT_EQ(fsm.mode(),DrivingMode::AVOID);
}

TEST(DrivingFsm, RecoveryResetInvalidatesEarlierDecisions) {
  DrivingFsm fsm;const auto before=fsm.revision();
  fsm.reset();EXPECT_GT(fsm.revision(),before);
  fsm.acceptLateral(DrivingMode::OVERTAKE);const auto overtaking=fsm.revision();
  fsm.reset();EXPECT_GT(fsm.revision(),overtaking);
  EXPECT_EQ(fsm.mode(),DrivingMode::FREE_RUN);
  EXPECT_EQ(fsm.plan({true,true,false,3.,0.}).maneuver,DrivingMode::AVOID);
}

}  // namespace reference_space_mppi_planner
