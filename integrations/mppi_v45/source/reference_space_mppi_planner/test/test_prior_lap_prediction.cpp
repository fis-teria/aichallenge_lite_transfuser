#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/prior_lap_route.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner {
namespace {
using namespace opponent_prediction;
std::vector<std::array<double,2>> circle() {
  std::vector<std::array<double,2>> points;
  for (int i=0;i<=1000;++i) {
    const double yaw=2.*std::acos(-1.)*i/1000.;
    points.push_back({20.*std::cos(yaw),20.*std::sin(yaw)});
  }
  return points;
}
std::vector<PositionObservation> history(double end) {
  std::vector<PositionObservation> samples;
  for(int i=0;i<=std::lround(end/.05);++i) {
    const double t=i*.05, angle=7.*t/20.;
    samples.push_back({t,20.*std::cos(angle),20.*std::sin(angle),7.*t});
  }
  return samples;
}
TEST(PriorLapPrediction, CircularMotionFollowsPhysicalArcAcrossReferenceWrap) {
  const auto points=circle();
  ReferencePoseIndex world(points,[](const auto &p){return p;});
  const auto samples=history(50.);
  const auto prediction=buildPriorLapPrediction(samples,2.*std::acos(-1.)*20.,50.1,6.,world);
  ASSERT_TRUE(prediction);
  EXPECT_DOUBLE_EQ(prediction->source_stamp,50.);
  for(double t=0.;t<=6.;t+=.03125) {
    const auto p=prediction->at(t);
    const auto xy=world.pose(p.global_s,p.d);
    ASSERT_TRUE(xy);
    const double angle=7.*(50.1+t)/20.;
    // A quadratic estimate over .6 s approximates circular velocity; its
    // small bias accumulates over this six-second open-loop forecast.
    EXPECT_NEAR((*xy)[0],20.*std::cos(angle),.2);
    EXPECT_NEAR((*xy)[1],20.*std::sin(angle),.2);
    EXPECT_NEAR(std::remainder((*xy)[2]+p.relative_yaw-angle-std::acos(-1.)/2.,
                               2.*std::acos(-1.)),0.,.015);
  }
  for(std::size_t i=1;i<prediction->points.size();++i)
    EXPECT_GT(prediction->points[i].global_s,prediction->points[i-1].global_s);
}
TEST(PriorLapPrediction, InsufficientPastCoveragePreservesConstantVelocity) {
  const auto points=circle();
  ReferencePoseIndex world(points,[](const auto &p){return p;});
  EXPECT_FALSE(buildPriorLapPrediction(history(10.),2.*std::acos(-1.)*20.,10.,6.,world));
  EXPECT_FALSE(buildPriorLapPrediction(history(50.),2.*std::acos(-1.)*20.,50.,30.,world));
  mppi::DynamicObstacle o;
  o.s_m=10.;o.global_reference_s_m=100.;o.d_m=-1.;
  o.longitudinal_speed_mps=7.;o.lateral_speed_mps=.4;o.heading_relative_to_reference_rad=.1;
  const auto p=positionAt(o,3.);
  EXPECT_DOUBLE_EQ(p.s,31.);EXPECT_DOUBLE_EQ(p.global_s,121.);
  EXPECT_DOUBLE_EQ(p.d,-1.+.4*3.);EXPECT_DOUBLE_EQ(p.relative_yaw,.1);
}
TEST(PriorLapPrediction, ChangingStationMetricDoesNotChangePhysicalTravel) {
  const auto points=circle();
  ReferencePoseIndex world(points,[](const auto &p){return p;});
  auto samples=history(50.);
  const double length=2.*std::acos(-1.)*20.;
  const auto uniform=buildPriorLapPrediction(samples,length,50.,6.,world);
  for(auto &p:samples) p.station+=8.*std::sin(p.station/20.);
  const auto nonuniform=buildPriorLapPrediction(samples,length,50.,6.,world);
  ASSERT_TRUE(uniform);ASSERT_TRUE(nonuniform);
  for(double t=0.;t<=6.;t+=.1) {
    const auto a=uniform->at(t),b=nonuniform->at(t);
    const auto pa=world.pose(a.global_s,a.d),pb=world.pose(b.global_s,b.d);
    ASSERT_TRUE(pa);ASSERT_TRUE(pb);
    EXPECT_NEAR((*pa)[0],(*pb)[0],.01);EXPECT_NEAR((*pa)[1],(*pb)[1],.01);
  }
}
TEST(PriorLapPrediction, QuadraticFitUsesPositionHistoryAtItsSourceEpoch) {
  std::vector<PositionObservation> samples;
  for(int i=0;i<=12;++i) {
    const double t=-.6+i*.05;
    samples.push_back({100.+t,3.+7.*t+.5*t*t,2.-t+2.*t*t,0.});
  }
  std::array<double,4> fitted;
  ASSERT_TRUE(fitRecent(samples,fitted));
  EXPECT_NEAR(fitted[0],3.,1e-10);EXPECT_NEAR(fitted[1],2.,1e-10);
  EXPECT_NEAR(fitted[2],7.,1e-10);EXPECT_NEAR(fitted[3],-1.,1e-10);
}

TEST(PriorLapRoute, RevisitedStationsNeverMixDifferentPasses) {
  std::vector<PositionObservation> samples;
  for(int i=0;i<=10;++i)samples.push_back({double(i),double(i),0.,double(i)});
  for(int i=9;i>=0;--i)samples.push_back({double(samples.size()),double(i),2.,double(i)});
  for(int i=1;i<=10;++i)samples.push_back({double(samples.size()),double(i),2.,double(i)});
  const auto selected=selectContinuousPriorPass(samples,2.,8.);
  ASSERT_EQ(selected.size(),11U);
  for(const auto &p:selected)EXPECT_DOUBLE_EQ(p.y,2.);
  for(std::size_t i=1;i<selected.size();++i) {
    EXPECT_GT(selected[i].stamp,selected[i-1].stamp);
    EXPECT_GT(selected[i].station,selected[i-1].station);
  }
}

TEST(PriorLapRoute, PartialPassesCannotCombineToManufactureCoverage) {
  std::vector<PositionObservation> samples;
  for(int i=0;i<=6;++i)samples.push_back({double(i),double(i),0.,double(i)});
  samples.push_back({7.,4.,2.,4.});
  for(int i=5;i<=10;++i)samples.push_back({double(samples.size()),double(i),2.,double(i)});
  EXPECT_TRUE(selectContinuousPriorPass(samples,2.,9.).empty());
  auto first=selectContinuousPriorPass(samples,2.,5.);
  ASSERT_FALSE(first.empty());EXPECT_DOUBLE_EQ(first.front().y,0.);
}

TEST(PriorLapRoute, RepeatedStoppedPositionsDoNotBreakForwardPass) {
  std::vector<PositionObservation> samples{{0,0,0,0},{1,1,0,1},{2,1,0,1},
      {3,1,0,1},{4,2,0,2},{5,3,0,3}};
  const auto selected=selectContinuousPriorPass(samples,0.,3.);
  ASSERT_EQ(selected.size(),4U);
  EXPECT_DOUBLE_EQ(selected.back().station,3.);
}

TEST(PriorLapRoute, MotionAtFixedStationSplitsPassInsteadOfKeepingLastPoint) {
  const std::vector<PositionObservation> samples{{0,0,0,0},{1,1,0,1},{2,2,0,2},
      {3,2,2,2},{4,3,2,3},{5,4,2,4}};
  EXPECT_TRUE(selectContinuousPriorPass(samples,0.,4.).empty());
  const auto selected=selectContinuousPriorPass(samples,2.,4.);
  ASSERT_EQ(selected.size(),3U);EXPECT_DOUBLE_EQ(selected.front().y,2.);
}

TEST(PriorLapRoute, SelectionBeforeClippingRetainsEvidenceOfExcursion) {
  const std::vector<PositionObservation> samples{{0,0,0,0},{1,1,0,1},{2,2,0,2},
      {3,20,0,20},{4,3,2,3},{5,4,2,4},{6,5,2,5}};
  auto selected=selectContinuousPriorPass(samples,0.,5.);
  selected.erase(std::remove_if(selected.begin(),selected.end(),
      [](const auto &p){return p.station>=10.;}),selected.end());
  ASSERT_EQ(selected.size(),3U);
  EXPECT_LT(selected.back().station,5.);
}

TEST(PriorLapPrediction, ReversalCannotSupplyCoverageBySortingDifferentPasses) {
  const auto points=circle();
  ReferencePoseIndex world(points,[](const auto &p){return p;});
  std::vector<PositionObservation> samples;
  const auto append=[&](double station) {
    samples.push_back({samples.size()*.05,20.*std::cos(station/20.),
                      20.*std::sin(station/20.),station});
  };
  for(double s=0.;s<=230.;s+=.35)append(s);
  for(double s=229.;s>=220.;s-=.35)append(s);
  for(double s=220.;s<=350.;s+=.35)append(s);
  EXPECT_FALSE(buildPriorLapPrediction(samples,2.*std::acos(-1.)*20.,
      samples.back().stamp,6.,world));
}
} // namespace
} // namespace reference_space_mppi_planner
