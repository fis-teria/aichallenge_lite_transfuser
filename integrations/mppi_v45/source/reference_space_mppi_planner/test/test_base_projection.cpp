#include "reference_space_mppi_planner/reference_field.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <random>
#include <vector>

namespace reference_space_mppi_planner::mppi {
namespace {
// Original formula is the numerical oracle for deferred projection attributes.
SegmentProjection originalProjection(const BaseReferencePoint *points,
    std::size_t count,double x,double y) {
  SegmentProjection best{0,0,0,std::numeric_limits<double>::infinity(),0};
  for(std::size_t i=1;i<count;++i) {
    const auto &a=points[i-1];const auto &b=points[i];
    const double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,l2=dx*dx+dy*dy;
    if(l2<=1e-12)continue;
    const double t=std::clamp(((x-a.x_m)*dx+(y-a.y_m)*dy)/l2,0.,1.);
    const double ex=x-a.x_m-t*dx,ey=y-a.y_m-t*dy,err=ex*ex+ey*ey;
    if(err<best.error)best={a.s_m+t*(b.s_m-a.s_m),
      (dx*ey-dy*ex)/std::sqrt(l2),std::atan2(dy,dx),err,i-1};
  }
  return best;
}

void identical(double actual,double expected) {
  if(std::isnan(expected)){EXPECT_TRUE(std::isnan(actual));return;}
  std::uint64_t a,b;
  std::memcpy(&a,&actual,sizeof(a));std::memcpy(&b,&expected,sizeof(b));
  EXPECT_EQ(a,b)<<actual<<" != "<<expected;
}

void compare(const std::vector<BaseReferencePoint> &points,double x,double y) {
  const auto old=originalProjection(points.data(),points.size(),x,y);
  const auto now=projectOnBase(points.data(),points.size(),x,y);
  EXPECT_EQ(now.index,old.index);
  identical(now.s,old.s);identical(now.d,old.d);
  identical(now.yaw,old.yaw);identical(now.error,old.error);
  const auto indexed=BaseProjectionIndex(points.data(),points.size()).project(x,y);
  EXPECT_EQ(indexed.index,old.index);
  identical(indexed.s,old.s);identical(indexed.d,old.d);
  identical(indexed.yaw,old.yaw);identical(indexed.error,old.error);
}

TEST(BaseProjection, FoldedTieKeepsFirstSegmentAndEndpointResidual) {
  std::vector<BaseReferencePoint> p(3);
  p[0].x_m=0.;p[0].s_m=10.;
  p[1].x_m=2.;p[1].s_m=14.;
  p[2].x_m=0.;p[2].s_m=22.;
  auto result=projectOnBase(p.data(),p.size(),1.,.5);
  EXPECT_EQ(result.index,0U);EXPECT_EQ(result.s,12.);
  EXPECT_EQ(result.d,.5);EXPECT_EQ(result.yaw,0.);EXPECT_EQ(result.error,.25);
  result=projectOnBase(p.data(),p.size(),3.,-.5);
  EXPECT_EQ(result.index,0U);EXPECT_EQ(result.s,14.);
  EXPECT_EQ(result.d,-.5);EXPECT_EQ(result.error,1.25);
}

TEST(BaseProjection, EmptyDegenerateThresholdAndNonfiniteInputs) {
  compare({},0.,0.);
  std::vector<BaseReferencePoint> p(3);
  compare(p,1.,1.);
  for(double dx:{0.,1e-8,1e-6,std::nextafter(1e-6,2e-6),2e-6}) {
    p[1].x_m=dx;p[2].x_m=2.*dx;
    for(double x:{-1.,0.,dx,1.})compare(p,x,-0.);
  }
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for(double value:{nan,inf,-inf,1e200}) {
    compare(p,value,0.);compare(p,0.,value);
    p[1].x_m=value;compare(p,1.,1.);
  }
}

TEST(BaseProjection, RandomizedCurvesFoldsDuplicatesAndLargeCoordinatesMatchBits) {
  std::mt19937_64 random(20260909);
  std::uniform_real_distribution<double> u(-1.,1.);
  for(int shape=0;shape<512;++shape) {
    const std::size_t count=2+random()%(kMaximumReferencePoints-1);
    std::vector<BaseReferencePoint> p(count);
    const double offset=shape%3==0?1e6:0.;
    for(std::size_t i=0;i<count;++i) {
      p[i].x_m=offset+40.*std::sin(.05*i)+u(random);
      p[i].y_m=offset+20.*std::sin(.13*i)+u(random);
      p[i].s_m=double(i)*.37;
      if(i && i%17==0) {p[i].x_m=p[i-1].x_m;p[i].y_m=p[i-1].y_m;}
    }
    for(int query=0;query<64;++query) {
      SCOPED_TRACE(::testing::Message()<<"shape="<<shape<<" query="<<query);
      compare(p,offset+60.*u(random),offset+40.*u(random));
      const auto &near=p[random()%p.size()];
      compare(p,std::nextafter(near.x_m,INFINITY),std::nextafter(near.y_m,-INFINITY));
    }
  }
}
TEST(BaseProjection, IndexOwnsGeometryAndKeepsCrossBlockTies) {
  std::vector<BaseReferencePoint> p(25);
  for(std::size_t i=0;i<p.size();++i) {
    p[i].x_m=i%2?2.:0.;p[i].s_m=i*2.;
  }
  BaseProjectionIndex index(p.data(),p.size());
  p.clear();
  const auto result=index.project(1.,.5);
  EXPECT_EQ(result.index,0U);
  identical(result.s,1.);identical(result.d,.5);
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
