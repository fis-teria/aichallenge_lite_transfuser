#include "reference_space_mppi_planner/online_motion_prediction.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::opponent_prediction {
namespace {
void append(std::vector<PositionObservation> &history, double t, double x, double y) {
  history.push_back({t,x,y,0.});
  history.erase(history.begin(),std::find_if(history.begin(),history.end(),
      [&](const auto &p){return p.stamp>=t-.65;}));
}
}

TEST(OnlineMotion, UnobservedFutureNeverChangesScores) {
  OnlineMotionLearner learner;
  std::vector<PositionObservation> history;
  for(int i=0;i<=9;++i) {
    const double t=i*.05;
    append(history,t,4.*t,0.);learner.observe(history);
  }
  EXPECT_GT(learner.pendingCount(),0U);
  const auto before=learner.summary();
  EXPECT_EQ(before.samples,(std::array<std::size_t,3>{0,0,0}));
  EXPECT_EQ(before.model,OnlineMotionLearner::initial_model);
  for(int i=0;i<100;++i) learner.observe(history);
  EXPECT_EQ(learner.summary().samples,before.samples);
  EXPECT_EQ(learner.summary().model,before.model);
}

TEST(OnlineMotion, RoadLearnerLearnsPersistentAccelerationAndBrakingCausally) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{500.,0.},{1000.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  for(double acceleration:{-.5,.5}) {
    OnlineMotionLearner learner;
    std::vector<PositionObservation> history;
    for(int i=0;i<=160;++i) {
      const double t=i*.05;
      append(history,t,20.+10.*t+.5*acceleration*t*t,0.);
      learner.observe(history,&world);
    }
    const auto summary=learner.summary();
    ASSERT_TRUE(summary.follows_reference);
    EXPECT_DOUBLE_EQ(summary.persistence.acceleration_sec,2.4);
    EXPECT_GT(summary.samples[2],40U);
    EXPECT_LT(summary.mean_squared_error,1e-10);
  }
}

TEST(OnlineMotion, LearnedBrakingDurationPreservesStopAndEpochAlignment) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  const ReferenceMotion state{10.,20.,1.,2.,0.,-2.};
  const auto original=buildReferenceMotionPrediction(state,10.,3.,world,.6,2.4);
  const auto aligned=buildReferenceMotionPrediction(state,10.4,2.,world,.6,2.4);
  ASSERT_TRUE(original);ASSERT_TRUE(aligned);
  EXPECT_NEAR(aligned->at(1.6).global_s,original->at(2.).global_s,1e-12);
  EXPECT_NEAR(original->at(2.).global_s,21.,1e-12);
  EXPECT_DOUBLE_EQ(original->at(2.).global_s,original->at(3.).global_s);
  EXPECT_DOUBLE_EQ(original->at(2.).d,original->at(3.).d);
  EXPECT_FALSE(buildReferenceMotionPrediction(state,10.,3.,world,.6,-1.));
}

TEST(OnlineMotion, PersistentTurnIsLearnedFromEarlierIssuedForecasts) {
  OnlineMotionLearner learner;
  std::vector<PositionObservation> history;
  for(int i=0;i<=240;++i) {
    const double t=i*.05;
    append(history,t,20.*std::sin(.3*t),20.*(1.-std::cos(.3*t)));
    learner.observe(history);
  }
  const auto result=learner.summary();
  EXPECT_DOUBLE_EQ(result.persistence.yaw_rate_sec,2.4);
  EXPECT_GT(result.samples[2],80U);
  EXPECT_LT(result.mean_squared_error,.3);
  EXPECT_LE(learner.pendingCount(),21U);
}

TEST(OnlineMotion, StoredPredictionIsNotRefittedUsingItsOutcome) {
  OnlineMotionLearner learner;
  std::vector<PositionObservation> history;
  for(int i=0;i<=40;++i) {
    const double t=i*.05;
    append(history,t,4.*t,t<.7 ? 0. : 3.);
    learner.observe(history);
  }
  EXPECT_GT(learner.summary().samples[0],0U);
  EXPECT_GT(learner.summary().mean_squared_error,.1);
}

TEST(OnlineMotion, IndependentOpponentsAndFreshRacesHaveIndependentScores) {
  OnlineMotionLearner turning,straight;
  std::vector<PositionObservation> a,b;
  for(int i=0;i<=160;++i) {
    const double t=i*.05;
    append(a,t,20.*std::sin(.3*t),20.*(1.-std::cos(.3*t)));
    append(b,t,6.*t,0.);
    turning.observe(a);straight.observe(b);
  }
  EXPECT_DOUBLE_EQ(turning.summary().persistence.yaw_rate_sec,2.4);
  EXPECT_LT(straight.summary().mean_squared_error,1e-15);
  const auto snapshot=turning;
  turning=OnlineMotionLearner{};
  EXPECT_EQ(turning.summary().samples,(std::array<std::size_t,3>{0,0,0}));
  EXPECT_GT(snapshot.summary().samples[2],0U);
}

TEST(OnlineMotion, DuplicateAndOutOfOrderSamplesCannotTrainTwice) {
  OnlineMotionLearner learner;
  std::vector<PositionObservation> history;
  for(int i=0;i<=60;++i){append(history,i*.05,i*.2,0.);learner.observe(history);}
  const auto before=learner.summary();const auto pending=learner.pendingCount();
  learner.observe(history);
  history.pop_back();learner.observe(history);
  EXPECT_EQ(learner.summary().samples,before.samples);
  EXPECT_EQ(learner.pendingCount(),pending);
}

TEST(OnlineMotion, MissingObservationIntervalIsNotUsedAsTrainingTruth) {
  OnlineMotionLearner learner;
  std::vector<PositionObservation> history;
  for(int i=0;i<=60;++i){append(history,i*.05,i*.2,0.);learner.observe(history);}
  append(history,5.,20.,0.);learner.observe(history);
  EXPECT_EQ(learner.summary().samples,(std::array<std::size_t,3>{0,0,0}));
  EXPECT_EQ(learner.pendingCount(),0U);
}

TEST(OnlineMotion, ReferenceDecayAlignsFromTheSourceEpochAndKeepsHeadingContinuous) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  const ReferenceMotion state{10.,20.,1.,5.,2.};
  const auto p=buildReferenceMotionPrediction(state,10.4,2.,world,.6);
  ASSERT_TRUE(p);
  const auto point=p->at(.6);
  double s=state.s,d=state.d;
  for(int i=0;i<10000;++i) {
    const double t=(i+.5)*.0001,yaw=std::atan2(2.*std::exp(-t/.6),5.);
    s+=std::hypot(5.,2.)*std::cos(yaw)*.0001;
    d+=std::hypot(5.,2.)*std::sin(yaw)*.0001;
  }
  EXPECT_NEAR(point.global_s,s,1e-5);
  EXPECT_NEAR(point.d,d,2e-5);
  EXPECT_NEAR(point.relative_yaw,std::atan2(2.*std::exp(-1./.6),5.),1e-12);
  EXPECT_NEAR(std::hypot(p->vx,p->vy),std::hypot(5.,2.),1e-12);
  EXPECT_FALSE(buildReferenceMotionPrediction(state,9.,2.,world,.6));
  EXPECT_NEAR(referenceMotionAt(state,.6-1e-6,world,.6)->relative_yaw,
              referenceMotionAt(state,.6+1e-6,world,.6)->relative_yaw,1e-5);
}

TEST(OnlineMotion, ReferenceCandidatesAreScoredOnlyAfterTheirOutcomesArrive) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  OnlineMotionLearner learner;std::vector<PositionObservation> history;
  for(int i=0;i<=8;++i){append(history,i*.05,10.+i*.2,1.);learner.observe(history,&world);}
  EXPECT_TRUE(learner.summary().follows_reference);
  EXPECT_EQ(learner.summary().samples[0],0U);
  for(int i=9;i<=200;++i){append(history,i*.05,10.+i*.2,1.);learner.observe(history,&world);}
  EXPECT_GT(learner.summary().samples[2],50U);
  EXPECT_LT(learner.summary().mean_squared_error,1e-15);
  EXPECT_LE(learner.pendingCount(),21U);
}

TEST(OnlineMotion, EveryRoadCandidatePreservesMeasuredLateralMotionAtSource) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  const ReferenceMotion state{10.,20.,1.,5.,2.,-1.};
  for(const double duration:OnlineMotionLearner::lateral_durations) {
    SCOPED_TRACE(duration);
    const auto prediction=buildReferenceMotionPrediction(state,10.,2.,world,duration);
    ASSERT_TRUE(prediction);
    EXPECT_NEAR(prediction->vx,5.,1e-12);
    EXPECT_NEAR(prediction->vy,2.,1e-12);
    EXPECT_NEAR(prediction->at(0.).relative_yaw,std::atan2(2.,5.),1e-12);
    EXPECT_GT(prediction->at(.01).d,state.d);
  }
}

TEST(OnlineMotion, UnscoredRoadCandidatesKeepTheNominalDecay) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  OnlineMotionLearner learner;std::vector<PositionObservation> history;
  for(int i=0;i<=8;++i){append(history,i*.05,10.+i*.2,1.);learner.observe(history,&world);}
  const auto choice=learner.summary();
  ASSERT_TRUE(choice.follows_reference);
  EXPECT_EQ(choice.samples[0],0U);
  EXPECT_DOUBLE_EQ(choice.lateral_persistence_sec,.6);
}

TEST(OnlineMotion, BrakingStartsAtObservationAndDoesNotRestartDuringAlignment) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(int i=0;i<=12;++i) {
    const double t=-.6+i*.05;
    append(history,10.+t,20.+6.*t-2.*t*t,1.);
  }
  const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
  EXPECT_NEAR(state->along,6.,1e-10);EXPECT_NEAR(state->acceleration,-4.,1e-10);
  const auto original=buildReferenceMotionPrediction(*state,10.,3.,world,.6);
  const auto aligned=buildReferenceMotionPrediction(*state,10.4,2.,world,.6);
  ASSERT_TRUE(original);ASSERT_TRUE(aligned);
  EXPECT_NEAR(aligned->vx,4.4,1e-10);
  EXPECT_NEAR(aligned->at(.6).global_s,24.32,1e-10);
  EXPECT_NEAR(aligned->at(1.6).global_s,original->at(2.).global_s,1e-10);
  EXPECT_NEAR(aligned->at(1.6).global_s,27.92,1e-10);
}

TEST(OnlineMotion, PredictedStopFreezesPositionHeadingAndVelocity) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  const ReferenceMotion state{10.,20.,1.,2.,1.,-5.};
  const double stop_time=std::hypot(2.,1.)/5.;
  const auto stop=referenceMotionAt(state,stop_time,world,.6);
  const auto later=referenceMotionAt(state,2.,world,.6);
  ASSERT_TRUE(stop);ASSERT_TRUE(later);
  EXPECT_NEAR(stop->global_s,later->global_s,1e-7);EXPECT_NEAR(stop->d,later->d,1e-7);
  EXPECT_DOUBLE_EQ(stop->relative_yaw,later->relative_yaw);
  const auto p=buildReferenceMotionPrediction(state,11.,2.,world,.6);ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->vx,0.);EXPECT_DOUBLE_EQ(p->vy,0.);
  // Check the integrated lateral displacement against independent quadrature.
  double d=state.d,s=state.s;
  for(int i=0;i<10000;++i) {
    const double dt=stop_time/10000.,t=(i+.5)*dt;
    const double yaw=std::atan2(std::exp(-t/.6),2.),distance=(std::sqrt(5.)-5.*t)*dt;
    d+=std::sin(yaw)*distance;s+=std::cos(yaw)*distance;
  }
  EXPECT_NEAR(stop->d,d,3e-5);EXPECT_NEAR(stop->global_s,s,3e-5);
}

TEST(OnlineMotion, CurvedRoadAndLapBoundaryPreserveForwardBrakingMotion) {
  std::vector<std::array<double,2>> xy;
  for(int i=0;i<=2000;++i) {
    const double a=2.*std::acos(-1.)*i/2000.;xy.push_back({20.*std::cos(a),20.*std::sin(a)});
  }
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(int i=0;i<=12;++i) {
    const double t=-.6+i*.05,a=(.2+5.*t-t*t)/20.;
    append(history,10.+t,20.*std::cos(a),20.*std::sin(a));
  }
  const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
  // The Cartesian quadratic fit approximates an arc over the 0.6 s window.
  EXPECT_NEAR(state->along,5.,.01);EXPECT_NEAR(state->acceleration,-2.,.2);
  const auto p=buildReferenceMotionPrediction(*state,10.,2.,world,0.);ASSERT_TRUE(p);
  const auto future=world.pose(p->at(2.).global_s,p->at(2.).d);ASSERT_TRUE(future);
  EXPECT_NEAR(std::hypot((*future)[0],(*future)[1]),20.,.01);
  EXPECT_NEAR(p->at(2.).global_s-p->at(0.).global_s,7.96,.2);
  EXPECT_GT(p->at(2.).global_s,p->at(0.).global_s);
}

TEST(OnlineMotion, ReferenceUnavailableFallsBackToCartesianScores) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  OnlineMotionLearner learner;std::vector<PositionObservation> history;
  for(int i=0;i<=80;++i){append(history,i*.05,10.+i*.2,1.);learner.observe(history,&world);}
  EXPECT_TRUE(learner.summary().follows_reference);
  for(int i=81;i<=84;++i){append(history,i*.05,10.+i*.2,1.);learner.observe(history);}
  EXPECT_FALSE(learner.summary().follows_reference);
  EXPECT_LT(learner.summary().mean_squared_error,1e-15);
}

TEST(OnlineMotion, AccelerationEstimateRespondsToBrakingBeforeTheLongFit) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(int i=0;i<=16;++i) {
    const double t=-.6+i*.05;
    append(history,10.+t,20.+6.*t+.5*(t<0.?2.:-3.)*t*t,1.);
  }
  const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
  const auto long_fit=fitReferenceMotionFromHistory(history,world);ASSERT_TRUE(long_fit);
  EXPECT_LT(state->acceleration,-1.);
  EXPECT_GT(long_fit->acceleration,0.);
  EXPECT_DOUBLE_EQ(state->s,long_fit->s);EXPECT_DOUBLE_EQ(state->along,long_fit->along);
  EXPECT_DOUBLE_EQ(state->d,long_fit->d);EXPECT_DOUBLE_EQ(state->lateral,long_fit->lateral);
}

TEST(OnlineMotion, AccelerationEstimateAlsoReleasesAnEndedBrake) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(int i=0;i<=16;++i) {
    const double t=-.6+i*.05;
    append(history,10.+t,20.+6.*t+.5*(t<0.?-3.:2.)*t*t,1.);
  }
  const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
  const auto long_fit=fitReferenceMotionFromHistory(history,world);ASSERT_TRUE(long_fit);
  EXPECT_GT(state->acceleration,.5);
  EXPECT_LT(long_fit->acceleration,-1.);
}

TEST(OnlineMotion, SparseShortWindowRetainsTheUsableLongWindow) {
  const std::vector<std::array<double,2>> xy{{0.,0.},{50.,0.},{100.,0.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  for(double t:{-.4,-.2,0.})history.push_back({10.+t,20.+6.*t-2.*t*t,1.,0.});
  const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
  EXPECT_NEAR(state->acceleration,-4.,1e-10);
  EXPECT_NEAR(state->along,6.,1e-10);
}

TEST(OnlineMotion, QuantizedConstantSpeedMotionDoesNotCreateAOneUnitAcceleration) {
  const std::vector<std::array<double,2>> xy{{89600.,43100.},{89700.,43100.},{89800.,43100.}};
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  std::vector<PositionObservation> history;
  double maximum=0.;
  for(int i=0;i<=160;++i) {
    const double t=i*.05;
    append(history,10.+t,static_cast<float>(89610.+6.17*t),43101.);
    if(t<.6)continue;
    const auto state=fitReferenceMotion(history,world);ASSERT_TRUE(state);
    maximum=std::max(maximum,std::abs(state->acceleration));
  }
  EXPECT_LT(maximum,.5);
}

TEST(OnlineMotion, ConstantSpeedInnerAndOuterTurnsAdvanceByPhysicalDistance) {
  std::vector<std::array<double,2>> xy;
  for(int i=0;i<=4000;++i) {
    const double a=2.*std::acos(-1.)*i/4000.;xy.push_back({8.*std::cos(a),8.*std::sin(a)});
  }
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  for(double d:{-2.,0.,2.}) {
    const ReferenceMotion state{10.,49.,d,7.,0.,0.};
    const auto p=buildReferenceMotionPrediction(state,10.,2.,world,.6);ASSERT_TRUE(p);
    EXPECT_NEAR(p->at(2.).global_s-p->at(0.).global_s,14.*8./(8.-d),.01);
    EXPECT_NEAR(std::hypot(p->vx,p->vy),7.,1e-12);
    ReferenceMotionIntegrator issued(state,world,.6,.6);
    for(double h:OnlineMotionLearner::horizons) {
      const auto sample=issued.at(h);ASSERT_TRUE(sample);
      EXPECT_NEAR(sample->point.global_s,p->at(h).global_s,1e-12);
      EXPECT_NEAR(sample->point.d,p->at(h).d,1e-12);
    }
    const auto delayed=buildReferenceMotionPrediction(state,10.4373,2.,world,.6);ASSERT_TRUE(delayed);
    const auto same=referenceMotionAt(state,1.4373,world,.6);ASSERT_TRUE(same);
    EXPECT_NEAR(delayed->at(1.).global_s,same->global_s,1e-12);
  }
}

TEST(OnlineMotion, CornerExitChangesStationRateWithoutChangingPhysicalSpeed) {
  std::vector<std::array<double,2>> xy;
  for(int i=0;i<=1000;++i) {
    const double a=.5*std::acos(-1.)*i/1000.;xy.push_back({8.*std::cos(a),8.*std::sin(a)});
  }
  xy.push_back({-30.,8.});
  ReferencePoseIndex world(xy,[](const auto &p){return p;});
  const ReferenceMotion state{10.,8.*(.5*std::acos(-1.)-.8),2.,7.,0.,0.};
  const auto p=buildReferenceMotionPrediction(state,10.,2.,world,.6);ASSERT_TRUE(p);
  const auto end=world.pose(p->at(2.).global_s,p->at(2.).d);ASSERT_TRUE(end);
  EXPECT_NEAR((*end)[0],-9.2,.04);EXPECT_NEAR((*end)[1],6.,1e-12);
}
namespace {
ReferencePoseIndex switchCircle() {
  std::vector<std::array<double,2>> xy;
  for(int i=0;i<=4000;++i) {
    const double a=2.*std::acos(-1.)*i/4000.;xy.push_back({8.*std::cos(a),8.*std::sin(a)});
  }
  return ReferencePoseIndex(xy,[](const auto &p){return p;});
}
}

TEST(OnlineMotion, InvalidRoadMetricContinuesFromSourcePositionAndHeading) {
  const auto world=switchCircle();
  const ReferenceMotion state{10.,5.,7.99,1.,4.,0.};
  const auto p=buildReferenceMotionPrediction(state,10.,6.,world,9.,.6);
  ASSERT_TRUE(p);
  const auto origin=world.pose(state.s,state.d);ASSERT_TRUE(origin);
  const double yaw=(*origin)[2]+std::atan2(4.,1.), speed=std::sqrt(17.);
  for(double t:{.0125,.1,.5,1.,2.}) {
    const auto point=p->at(t);const auto xy=world.pose(point.global_s,point.d);ASSERT_TRUE(xy);
    EXPECT_NEAR((*xy)[0],(*origin)[0]+speed*t*std::cos(yaw),.001);
    EXPECT_NEAR((*xy)[1],(*origin)[1]+speed*t*std::sin(yaw),.001);
    EXPECT_NEAR(std::remainder((*xy)[2]+point.relative_yaw-yaw,2.*std::acos(-1.)),0.,1e-10);
  }
  EXPECT_NEAR(p->vx,speed*std::cos(yaw),1e-12);
  EXPECT_NEAR(p->vy,speed*std::sin(yaw),1e-12);
}

TEST(OnlineMotion, LateRoadFailureKeepsShortForecastAndIssuedLearnerPrefix) {
  const auto world=switchCircle();
  const ReferenceMotion state{10.,5.,6.,1.,4.,0.};
  const auto short_p=buildReferenceMotionPrediction(state,10.,2.,world,9.,.3);
  const auto long_p=buildReferenceMotionPrediction(state,10.,6.,world,9.,.3);
  ASSERT_TRUE(short_p);ASSERT_TRUE(long_p);
  ReferenceMotionIntegrator issued(state,world,9.,.3);
  for(double t:OnlineMotionLearner::horizons) {
    const auto a=short_p->at(t),b=long_p->at(t);const auto c=issued.at(t);ASSERT_TRUE(c);
    EXPECT_NEAR(a.global_s,b.global_s,1e-12);EXPECT_NEAR(a.d,b.d,1e-12);
    EXPECT_NEAR(a.global_s,c->point.global_s,1e-12);
    EXPECT_NEAR(a.relative_yaw,c->point.relative_yaw,1e-12);
  }
  for(double age:{.12345,1.12345,3.12345}) {
    const auto delayed=buildReferenceMotionPrediction(state,10.+age,2.,world,9.,.3);
    ASSERT_TRUE(delayed);
    const auto direct=referenceMotionAt(state,age+1.,world,9.,.3);ASSERT_TRUE(direct);
    EXPECT_NEAR(delayed->at(1.).global_s,direct->global_s,1e-10);
    EXPECT_NEAR(delayed->at(1.).d,direct->d,1e-10);
  }
}

TEST(OnlineMotion, SwitchKeepsSourceAccelerationDeadlineAndStopsWithoutMoving) {
  const auto world=switchCircle();
  const ReferenceMotion state{10.,5.,7.99,1.,4.,-2.};
  const double speed=std::sqrt(17.);
  const auto p=buildReferenceMotionPrediction(state,10.,6.,world,9.,2.4);ASSERT_TRUE(p);
  const auto stopped=buildReferenceMotionPrediction(state,13.,2.,world,9.,2.4);ASSERT_TRUE(stopped);
  EXPECT_DOUBLE_EQ(stopped->vx,0.);EXPECT_DOUBLE_EQ(stopped->vy,0.);
  EXPECT_NEAR(p->at(3.).global_s,p->at(6.).global_s,1e-12);
  EXPECT_NEAR(p->at(3.).d,p->at(6.).d,1e-12);
  EXPECT_NEAR(p->at(3.).relative_yaw,p->at(6.).relative_yaw,1e-12);
  const auto origin=world.pose(state.s,state.d),end=world.pose(p->at(3.).global_s,p->at(3.).d);
  ASSERT_TRUE(origin);ASSERT_TRUE(end);
  EXPECT_NEAR(std::hypot((*end)[0]-(*origin)[0],(*end)[1]-(*origin)[1]),speed*speed/4.,.001);
  const auto coast=buildReferenceMotionPrediction(state,10.4,2.,world,9.,.3);ASSERT_TRUE(coast);
  EXPECT_NEAR(std::hypot(coast->vx,coast->vy),speed-.6,1e-12);
}
} // namespace reference_space_mppi_planner::opponent_prediction
