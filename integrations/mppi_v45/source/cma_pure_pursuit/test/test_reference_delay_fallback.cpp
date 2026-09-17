#include <gtest/gtest.h>
#include "simple_pure_pursuit/reference_delay_fallback.hpp"

using namespace simple_pure_pursuit;

TEST(ReferenceDelayFallback, NearestPrecedingVehicleExcludesRearAndAdjacentLane) {
  ReferenceDelayFallback f;
  f.setReference({{0,0},{50,0},{100,0}});
  for (int i=0;i<=20;++i) {
    double t=1+i*.1;
    f.observe("self",10,0,t,t,0);
    f.observe("rear",5,0,t,t,0);
    f.observe("adjacent",11,3,t,t,0);
    f.observe("near",12+2*(t-1),0,t,t,0);
    f.observe("far",30+4*(t-1),0,t,t,0);
  }
  auto leader=f.leader(10,0,3,"self");
  ASSERT_TRUE(leader);EXPECT_EQ(leader->id,"near");
  EXPECT_NEAR(leader->speed_mps,2.0,.01);
  EXPECT_FALSE(f.leader(10,0,4,"self"));
}

TEST(ReferenceDelayFallback, StoppedVehicleIsZeroAndOlderMessagesDoNotOverwrite) {
  ReferenceDelayFallback f;f.setReference({{0,0},{50,0},{100,0}});
  f.observe("target",20,0,2,2,0);
  f.observe("target",20,0,2.1,2.1,0);
  f.observe("target",80,0,1,2.2,0);
  auto leader=f.leader(10,0,2.2,"self");ASSERT_TRUE(leader);
  EXPECT_DOUBLE_EQ(leader->speed_mps,0);EXPECT_DOUBLE_EQ(leader->forward_m,10);
}

TEST(ReferenceDelayFallback, WrapsForwardAcrossLapBoundary) {
  ReferenceDelayFallback f;
  f.setReference({{0,0},{20,0},{20,20},{0,20},{0,0}});
  f.observe("ahead",2,0,1,1,0);
  f.observe("rear",0,4,1,1,0);
  auto leader=f.leader(0,2,1,"self");ASSERT_TRUE(leader);
  EXPECT_EQ(leader->id,"ahead");EXPECT_NEAR(leader->forward_m,4,1e-9);
  EXPECT_NEAR(f.project(0,2).s,78,1e-9);
}

TEST(ReferenceDelayFallback, EmptyReferenceAndRecoveryResetHaveNoLeader) {
  ReferenceDelayFallback f;EXPECT_FALSE(f.valid());
  EXPECT_FALSE(f.leader(0,0,1,"self"));
  f.setReference({{0,0},{50,0},{100,0}});f.observe("target",20,0,1,1,0);
  ASSERT_TRUE(f.leader(10,0,1,"self"));f.clearVehicles();
  EXPECT_FALSE(f.leader(10,0,1,"self"));
}
