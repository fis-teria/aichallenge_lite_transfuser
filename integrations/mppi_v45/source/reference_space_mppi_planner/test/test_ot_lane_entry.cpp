#include "reference_space_mppi_planner/ot_lane_entry_planner.hpp"
#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
struct OtEntryTest : testing::Test {
  Config c; PlanRequest r; OtLaneEntryConfig entry; TemporaryReference path;
  void SetUp() override {
    c.enabled=true;c.shadow_only=false;c.awsim_vehicle_response_enabled=true;
    c.maximum_acceleration_mps2=2.;c.maximum_deceleration_mps2=2.;
    c.maximum_lateral_acceleration_mps2=4.;c.speed_proportional_gain=3.;
    c.longitudinal_planning_enabled=true;c.horizon_steps=60;
    entry.enabled=true;entry.entry_x_m=45.;entry.exit_x_m=81.;
    r.valid=true;r.ego.speed_mps=5.;r.stamp_sec=42.;
    std::vector<std::array<double,2>> points;
    for(int i=-30;i<=150;++i) points.push_back({double(i),0.});
    r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](const auto &p){return p;});
    r.base_reference_count=path.count=101;
    for(std::size_t i=0;i<path.count;++i) {
      auto &b=r.base_reference[i];b.s_m=b.x_m=.4*i;b.speed_mps=35./3.6;
      b.minimum_d_m=-10.;b.maximum_d_m=10.;
      auto &p=path.points[i];p.s_m=p.source_s_m=p.x_m=.4*i;
      p.speed_mps=p.uncapped_speed_mps=35./3.6;
    }
  }
};
TEST_F(OtEntryTest, PhysicalAccelerationReachesThirtyWithEnoughDistance) {
  const auto p=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(p);EXPECT_TRUE(p->reached_entry);EXPECT_TRUE(p->reachable_free_road);
  EXPECT_GE(p->predicted_speed_mps,30./3.6);
  EXPECT_NEAR(p->distance_m,45.,1e-5);
  EXPECT_NEAR(p->preparation.schedule.back().station_m,p->preparation.entry_station_m,1e-8);
  EXPECT_DOUBLE_EQ(p->preparation.acceleration_start_time_sec,0.);
}
TEST_F(OtEntryTest, DistanceShortageIsReportedAsUnreachable) {
  entry.entry_x_m=5.;const auto p=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(p);EXPECT_TRUE(p->reached_entry);EXPECT_FALSE(p->reachable_free_road);
  EXPECT_LT(p->predicted_speed_mps,6.);
}
TEST_F(OtEntryTest, SteeringHoldChangesAccelerationPrediction) {
  c.steering_demand_acceleration_hold_enabled=true;
  c.steering_acceleration_hold_minimum_tire_angle_rad=.01;
  c.steering_acceleration_hold_minimum_speed_mps=3.;
  c.steering_acceleration_hold_maximum_acceleration_mps2=0.;
  c.steering_time_constant_sec=2.;entry.entry_x_m=10.;
  const auto straight=makeOtLaneEntryPlan(r,c,entry);
  r.ego.steering_rad=.4;const auto turning=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(straight);ASSERT_TRUE(turning);
  EXPECT_LT(turning->predicted_speed_mps,straight->predicted_speed_mps);
}
TEST_F(OtEntryTest, DisabledAndPassedEntryDoNotCreateGoal) {
  entry.enabled=false;EXPECT_FALSE(makeOtLaneEntryPlan(r,c,entry));
  entry.enabled=true;r.ego.x_m=46.;EXPECT_FALSE(makeOtLaneEntryPlan(r,c,entry));
}
TEST_F(OtEntryTest, CurvatureLimitedEntryRemainsBelowTarget) {
  std::vector<std::array<double,2>> road;
  for(int i=-200;i<=200;++i) road.push_back({10.*std::cos(i*.01),10.*std::sin(i*.01)});
  r.world_reference=std::make_shared<ReferencePoseIndex>(road,[](const auto &p){return p;});
  r.ego.x_m=10.*std::cos(-1.);r.ego.y_m=10.*std::sin(-1.);
  entry.entry_x_m=10.;entry.entry_y_m=0.;entry.exit_x_m=10.;entry.exit_y_m=36.;
  c.maximum_lateral_acceleration_mps2=2.;
  const auto p=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(p);EXPECT_TRUE(p->reached_entry);EXPECT_FALSE(p->reachable_free_road);
  EXPECT_LT(p->predicted_speed_mps,5.);
}
TEST_F(OtEntryTest, EntryPlaneIncludesLateralOffsetInReference) {
  entry.entry_y_m=3.;entry.exit_y_m=3.;
  const auto p=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(p);EXPECT_NEAR(p->distance_m,45.,1e-5);
}
TEST_F(OtEntryTest, CandidateUsesInterpolatedActualEntrySpeed) {
  r.ot_lane_entry=makeOtLaneEntryPlan(r,c,entry);
  Evaluation e;e.predicted_rollout_count=2;
  e.predicted_rollout[0]={44.,0.,0.,7.,5.};
  e.predicted_rollout[1]={46.,0.,0.,9.,5.2};
  EXPECT_NEAR(otLaneEntryCost(r,e),std::pow(30./3.6-8.,2.),1e-9);
  e.predicted_rollout[0].speed_mps=9.;EXPECT_DOUBLE_EQ(otLaneEntryCost(r,e),0.);
}
TEST_F(OtEntryTest, ShortRolloutRewardsAccelerationPreparation) {
  r.ot_lane_entry=makeOtLaneEntryPlan(r,c,entry);
  Evaluation e;e.predicted_rollout_count=1;e.predicted_rollout[0]={10.,0.,0.,5.,2.};
  const double slow=otLaneEntryCost(r,e);
  e.predicted_rollout[0].speed_mps=7.;
  EXPECT_GT(slow,otLaneEntryCost(r,e));
}
TEST_F(OtEntryTest, ThirtyGoalDoesNotOverrideSlowerLeader) {
  r.ot_lane_entry=makeOtLaneEntryPlan(r,c,entry);
  r.dynamic_obstacle_count=1;
  r.dynamic_obstacles[0].s_m=5.+2.*c.vehicle_half_length_m;
  r.dynamic_obstacles[0].longitudinal_speed_mps=5.;
  const auto plan=planLongitudinalSpeed(path,r,c,true);
  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.reference.points[0].speed_mps,5.2);
}
TEST_F(OtEntryTest, OldReferenceGoalHasNoCostOnNewReference) {
  r.ot_lane_entry=makeOtLaneEntryPlan(r,c,entry);
  r.world_reference.reset();Evaluation e;e.predicted_rollout_count=1;
  EXPECT_DOUBLE_EQ(otLaneEntryCost(r,e),0.);
}
TEST_F(OtEntryTest, ClosedRoadEntryAcrossLapBoundary) {
  std::vector<std::array<double,2>> road{{0.,0.},{100.,0.},{100.,100.},{-30.,100.},{-30.,0.},{0.,0.}};
  r.world_reference=std::make_shared<ReferencePoseIndex>(road,[](const auto &p){return p;});
  r.ego.x_m=-10.;entry.entry_x_m=20.;entry.exit_x_m=56.;
  const auto p=makeOtLaneEntryPlan(r,c,entry);
  ASSERT_TRUE(p);EXPECT_NEAR(p->distance_m,30.,1e-5);
}
}
} // namespace reference_space_mppi_planner::mppi
