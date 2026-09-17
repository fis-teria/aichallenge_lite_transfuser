#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include "reference_space_mppi_planner/trajectory_projection.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace reference_space_mppi_planner::mppi {
namespace {
using Point = std::array<double,2>;
auto position = [](const Point &p) { return p; };

TEST(TrajectoryProjection, ClampedEndpointKeepsTangentialResidualOutOfOrigin) {
  const std::vector<Point> path{{0,0},{2,0},{2,5}};
  const auto p = projectTrajectory(path,position,2.2,-1.);
  ASSERT_TRUE(p.valid);
  EXPECT_EQ(p.lower_index,0U); EXPECT_DOUBLE_EQ(p.ratio,1.);
  EXPECT_DOUBLE_EQ(p.x_m,2.); EXPECT_DOUBLE_EQ(p.y_m,0.);
  EXPECT_DOUBLE_EQ(p.s_m,2.); EXPECT_DOUBLE_EQ(p.d_m,-1.);
  // The old ego - d*normal expression would put the origin at (2.2,0),
  // generating a reverse first segment to (2,0).
  EXPECT_NE(p.x_m,2.2+p.tangent_y*p.d_m);
}

TEST(TrajectoryProjection, InteriorNonuniformAndRepeatedPoints) {
  const std::vector<Point> path{{0,0},{0,0},{2,0},{2,5}};
  const auto p = projectTrajectory(path,position,1.,-.5);
  ASSERT_TRUE(p.valid); EXPECT_EQ(p.lower_index,1U);
  EXPECT_DOUBLE_EQ(p.ratio,.5); EXPECT_DOUBLE_EQ(p.x_m,1.);
  EXPECT_DOUBLE_EQ(p.s_m,1.); EXPECT_DOUBLE_EQ(p.d_m,-.5);
  EXPECT_FALSE(projectTrajectory(std::vector<Point>{{0,0},{0,0}},position,1.,1.).valid);
}

TEST(TrajectoryProjection, ClosedSeamKeepsExplicitClosingSegment) {
  const std::vector<Point> path{{0,0},{8,0},{8,6},{0,6},{0,0}};
  const auto p = projectTrajectory(path,position,-.3,1.);
  ASSERT_TRUE(p.valid); EXPECT_EQ(p.lower_index,3U);
  EXPECT_DOUBLE_EQ(p.x_m,0.); EXPECT_NEAR(p.y_m,1.,1e-14);
  EXPECT_NEAR(p.s_m,27.,1e-14); EXPECT_NEAR(p.d_m,-.3,1e-14);
}

PlanRequest fieldRequest() {
  PlanRequest r;
  r.bounds.maximum.speed_scale=1.;r.bounds.minimum.speed_scale=0.;
  r.base_reference_count=4;
  for(std::size_t i=0;i<4;++i) {
    auto &b=r.base_reference[i]; b.s_m=b.x_m=double(i); b.speed_mps=4.;
  }
  return r;
}

TemporaryReference foldedPath() {
  TemporaryReference r; r.count=4;
  const std::array<Point,4> xy{{{0,0},{2,0},{1,1},{3,1}}};
  for(std::size_t i=0;i<r.count;++i) {
    auto &p=r.points[i];p.x_m=xy[i][0];p.y_m=xy[i][1];
    p.s_m=i ? r.points[i-1].s_m+std::hypot(p.x_m-xy[i-1][0],p.y_m-xy[i-1][1]) : 0.;
    p.speed_mps=1.+.1*p.s_m;
  }
  return r;
}

TemporaryReference subdivide(const TemporaryReference &r) {
  TemporaryReference out;
  for(std::size_t i=1;i<r.count;++i) {
    const auto &a=r.points[i-1], &b=r.points[i];
    out.points[out.count++]=a;
    auto mid=a;
    mid.x_m=(a.x_m+b.x_m)*.5;mid.y_m=(a.y_m+b.y_m)*.5;
    mid.s_m=(a.s_m+b.s_m)*.5;mid.speed_mps=(a.speed_mps+b.speed_mps)*.5;
    out.points[out.count++]=mid;
  }
  out.points[out.count++]=r.points[r.count-1];
  return out;
}

TEST(OrderedHistory, IdenticalFoldedCartesianPathHasZeroCost) {
  auto request=fieldRequest(); const auto path=foldedPath();
  setExecutionHistory(request,path);
  const auto field=projectReferenceField(path,request);
  ASSERT_LT(field.points[2].s_m,field.points[1].s_m);
  EXPECT_NEAR(executionFieldCosts(field,request,Config{})[1],0.,1e-12);
}

TEST(OrderedHistory, FoldedIdentitySurvivesNonuniformResampling) {
  auto request=fieldRequest(); const auto path=foldedPath();
  setExecutionHistory(request,path);
  const auto field=projectReferenceField(subdivide(path),request);
  EXPECT_NEAR(executionFieldCosts(field,request,Config{})[1],0.,1e-12);
}

TEST(OrderedHistory, DistinctFoldedBranchesCannotEscapePenalty) {
  auto request=fieldRequest();auto path=foldedPath();
  setExecutionHistory(request,path);
  for(std::size_t i=0;i<path.count;++i)path.points[i].y_m+=.3;
  const auto a=executionFieldCosts(projectReferenceField(path,request),request,Config{});
  const auto b=executionFieldCosts(projectReferenceField(subdivide(path),request),request,Config{});
  EXPECT_GT(a[1],0.);EXPECT_NEAR(a[1],b[1],1e-12);
}

TEST(OrderedHistory, CommonArcSamplerDoesNotChooseTheFirstFoldedStation) {
  auto request=fieldRequest();const auto path=foldedPath();
  setExecutionHistory(request,path);
  const auto p=sampleReferenceField(*request.execution_history_field,3.);
  ASSERT_TRUE(p);
  // Arc 3 is on the return segment, not x=3 on the last branch.
  EXPECT_NEAR(p->x_m,2.-1./std::sqrt(2.),1e-12);
  EXPECT_NEAR(p->y_m,1./std::sqrt(2.),1e-12);
  EXPECT_NEAR(request.base_reference[3].active_d_m,p->y_m,1e-12);
  EXPECT_FALSE(sampleReferenceField(*request.execution_history_field,10.));
}

TEST(OrderedHistory, ConsumedPrefixAndSpeedOverlayKeepIdentity) {
  auto request=fieldRequest();auto source=foldedPath();
  EgoState ego;ego.x_m=.5;ego.y_m=.1;
  auto remaining=remainingExecutionTrajectory(source,ego,4.);
  ASSERT_TRUE(remaining);
  request.ego=ego;
  setExecutionHistory(request,*remaining);
  EXPECT_NEAR(executionFieldCosts(projectReferenceField(*remaining,request),request,Config{})[1],0.,1e-12);
}

TEST(OrderedHistory, ConstantSpeedChangeHasNoFoldDependentMultiplicity) {
  auto request=fieldRequest();auto path=foldedPath();
  setExecutionHistory(request,path);
  for(std::size_t i=0;i<path.count;++i)path.points[i].speed_mps+=.4;
  EXPECT_NEAR(executionFieldCosts(projectReferenceField(path,request),request,Config{})[1],
              Config{}.cost_control_change_weight*.01,1e-12);
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
