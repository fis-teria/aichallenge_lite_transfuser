#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/entry_connector.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace reference_space_mppi_planner::mppi {
// Separate executable: do not introduce a second definition of the other
// regression binary's private-access adapter.
struct PlannerEvaluationTestAccess {
  static bool generate(const ReferenceSpaceMppiPlanner &planner,
      const PlanRequest &request, TemporaryReference *reference, Evaluation *e) {
    return planner.generateReference(request.nominal, request, nullptr, reference, e);
  }
  static Evaluation evaluate(const ReferenceSpaceMppiPlanner &planner,
      const PlanRequest &request, TemporaryReference *reference) {
    return planner.evaluate(request.nominal, request, nullptr, reference);
  }
  static bool withControl(const ReferenceSpaceMppiPlanner &planner,const PlanRequest &r,
      const ReferenceControlSequence &control,TemporaryReference *reference,Evaluation *e) {
    return planner.generateReference(r.nominal,r,&control,reference,e);
  }
};
namespace {
TEST(PreparationCandidates, UseExistingWallLineAndWorldMergeStationsAcrossSamplesAndProgress) {
  Config c;c.enabled=true;c.shadow_only=false;c.collision_only_rejection=true;
  c.longitudinal_planning_enabled=true;
  std::vector<std::array<double,2>> xy;
  for(int i=0;i<=120;++i) xy.push_back({double(i),0.});
  auto world=std::make_shared<ReferencePoseIndex>(xy,[](const auto &p){return p;});
  auto lines=std::make_shared<PrecomputedWallLines>();lines->world=world;lines->length_m=120.;
  for(int i=0;i<=120;++i) lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  auto plan=std::make_shared<PassingPreparationPlan>();
  plan->world=world;plan->wall_lines=lines;plan->front_merge=true;plan->epoch_sec=100.;
  plan->entry_station_m=20.;plan->pass_station_m=plan->merge_start_station_m=40.;
  plan->merge_end_station_m=60.;plan->schedule={{0.,0.,5.,0.},{12.,60.,5.,0.}};
  for(int side:{-1,1}) for(double origin:{0.,10.}) {
    PlanRequest r;r.valid=true;r.side=side;r.sample_staged_speed=true;
    r.front_merge_attack=r.complete_maneuver=r.connect_to_wall_line=r.cartesian_measured_prefix=true;
    r.world_reference=world;r.precomputed_wall_lines=lines;r.passing_preparation=plan;
    r.ego.x_m=origin;r.ego.speed_mps=5.;r.stamp_sec=100.+origin/5.;
    r.wall_line_origin_station_m=origin;r.pass_profile_scale_m=3.*side;
    r.base_reference_count=201;r.phase=Phase::OVERTAKE;
    for(std::size_t i=0;i<r.base_reference_count;++i) {
      auto &b=r.base_reference[i];b.s_m=.5*i;b.x_m=origin+b.s_m;b.speed_mps=10.;
      b.minimum_d_m=-5.;b.maximum_d_m=5.;
    }
    TemporaryReference first;
    for(int sample=0;sample<2;++sample) {
      r.nominal={side*(sample ? 1.2 : 2.4),sample ? 29. : 12.,sample ? 40. : 5.,sample ? 12. : 30.,sample ? .2 : 1.};
      r.bounds.minimum=r.bounds.maximum=r.nominal;
      TemporaryReference path;Evaluation e;
      ASSERT_TRUE(PlannerEvaluationTestAccess::generate(ReferenceSpaceMppiPlanner(c),r,&path,&e));
      EXPECT_NEAR(path.points[static_cast<std::size_t>((30.-origin)*2.)].y_m,3.*side,1e-9);
      EXPECT_NEAR(path.points[static_cast<std::size_t>((40.-origin)*2.)].y_m,3.*side,1e-9);
      EXPECT_NEAR(path.points[path.front_merge_end_index].x_m,60.,1e-9);
      EXPECT_NEAR(path.points[path.front_merge_end_index].y_m,0.,1e-9);
      EXPECT_NEAR(path.points[path.wall_connection_end_index].x_m,20.,.51);
      if(sample==0) first=path;
      else for(std::size_t i=0;i<path.count;++i) {
        EXPECT_DOUBLE_EQ(path.points[i].x_m,first.points[i].x_m);
        EXPECT_DOUBLE_EQ(path.points[i].y_m,first.points[i].y_m);
        EXPECT_DOUBLE_EQ(path.points[i].speed_mps,first.points[i].speed_mps);
      }
    }
    if(origin==0.) {
      plan->entry_station_m=2.;
      auto limited=c;limited.maximum_tire_steering_angle_rad=.01;
      TemporaryReference path;Evaluation e;
      EXPECT_FALSE(PlannerEvaluationTestAccess::generate(ReferenceSpaceMppiPlanner(limited),r,&path,&e));
      EXPECT_STREQ(e.reject_stage,"entry_connection");
      plan->entry_station_m=20.;
    }
  }
}

PlanRequest offsetRequest(double offset, double steering = 0.0) {
  PlanRequest r;
  r.valid = true; r.side = offset < 0 ? -1 : 1;
  r.cartesian_measured_prefix = true; r.sample_staged_speed = true;
  r.nominal = {offset, 4, 4, 4, 1, 0, 1};
  r.ego.steering_rad = steering; r.ego.speed_mps = 1;
  r.base_reference_count = 151;
  for (std::size_t i = 0; i < r.base_reference_count; ++i) {
    auto &p = r.base_reference[i];
    p.s_m = p.x_m = i * .2;
    p.minimum_d_m = -10; p.maximum_d_m = 10; p.speed_mps = 1;
  }
  return r;
}
double peakCurvature(const TemporaryReference &r) {
  double peak = 0;
  for (std::size_t i = 0; i < r.count; ++i)
    peak = std::max(peak, std::abs(r.points[i].curvature_1pm));
  return peak;
}
TEST(EntryConnector, ExtendsUntrackableShortOffsetAndPreservesSuffix) {
  Config c; c.collision_only_rejection = true;
  ReferenceSpaceMppiPlanner planner(c);
  for (double offset : {-2.5, 2.5}) {
    auto r = offsetRequest(offset);
    TemporaryReference ref; Evaluation e;
    ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner, r, &ref, &e));
    EXPECT_LE(peakCurvature(ref), std::tan(c.maximum_tire_steering_angle_rad) / c.wheel_base_m + 1e-6);
    EXPECT_DOUBLE_EQ(ref.points[0].x_m, r.ego.x_m);
    EXPECT_DOUBLE_EQ(ref.points[0].y_m, r.ego.y_m);
    EXPECT_DOUBLE_EQ(ref.points[0].yaw_rad, r.ego.yaw_rad);
    EXPECT_DOUBLE_EQ(ref.points[ref.count - 1].x_m, 30);
    EXPECT_DOUBLE_EQ(ref.points[ref.count - 1].y_m, offset);
    EXPECT_DOUBLE_EQ(ref.points[ref.count - 1].speed_mps, 1);
  }
}

TEST(EntryConnector, PrecomputedSuffixSurvivesLateralNoiseAndDifferentEgoWindow) {
  Config c;c.collision_only_rejection=true;
  ReferenceSpaceMppiPlanner planner(c);
  std::vector<std::array<double,2>> center{{0.,0.},{100.,0.}};
  auto lines=std::make_shared<PrecomputedWallLines>();
  lines->world=std::make_shared<ReferencePoseIndex>(center,[](auto p){return p;});
  lines->length_m=100.;
  for(int i=0;i<=500;++i) {
    const double s=i*.2,d=2.+.15*std::sin(s*.05);
    lines->samples.push_back({s,{{{s,d},{s,-d}}}});
  }
  for(int side:{-1,1}) for(double origin:{0.,4.}) {
    auto r=offsetRequest(side*2.);r.world_reference=lines->world;r.precomputed_wall_lines=lines;
    r.connect_to_wall_line=true;r.wall_line_origin_station_m=origin;
    r.nominal.l_out_m=12.;r.ego.x_m=origin;r.ego.y_m=.3;
    for(std::size_t i=0;i<r.base_reference_count;++i) r.base_reference[i].x_m+=origin;
    ReferenceControlSequence control;control.count=c.control_knot_count;
    control.lateral_adjustment_m.fill(1.5);
    TemporaryReference ref;Evaluation e;
    ASSERT_TRUE(PlannerEvaluationTestAccess::withControl(planner,r,control,&ref,&e));
    EXPECT_DOUBLE_EQ(ref.points[0].x_m,r.ego.x_m);EXPECT_DOUBLE_EQ(ref.points[0].y_m,r.ego.y_m);
    std::size_t suffix=0;
    for(std::size_t i=ref.count;i>0;--i) {
      const auto expected=lines->at(origin+r.base_reference[i-1].s_m,side);
      ASSERT_TRUE(expected);
      if(ref.points[i-1].x_m!=(*expected)[0] || ref.points[i-1].y_m!=(*expected)[1]) break;
      ++suffix;
    }
    EXPECT_GE(suffix,60U);
    ASSERT_GT(ref.wall_connection_end_index,0U);
    ASSERT_LT(ref.wall_connection_end_index,ref.count);
    const auto join=ref.wall_connection_end_index;
    const auto expected_join=lines->at(origin+r.base_reference[join].s_m,side);
    ASSERT_TRUE(expected_join);
    EXPECT_DOUBLE_EQ(ref.points[join].x_m,(*expected_join)[0]);
    EXPECT_DOUBLE_EQ(ref.points[join].y_m,(*expected_join)[1]);
    EXPECT_LE(peakCurvature(ref),std::tan(c.maximum_tire_steering_angle_rad)/c.wheel_base_m+1e-6);
    r.precomputed_wall_lines.reset();
    EXPECT_FALSE(PlannerEvaluationTestAccess::generate(planner,r,&ref,&e));
    EXPECT_EQ(e.reject_reason,RejectReason::INVALID_INPUT);
  }
}
TEST(EntryConnector, MeasuredMaximumSteeringDoesNotOvershootInsidePolynomial) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(0, -c.maximum_tire_steering_angle_rad);
  TemporaryReference ref; Evaluation e;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(ReferenceSpaceMppiPlanner(c), r, &ref, &e));
  EXPECT_LE(peakCurvature(ref), std::tan(c.maximum_tire_steering_angle_rad) / c.wheel_base_m + 1e-6);
}
TEST(EntryConnector, InitialAngleUsesSamePhysicalClampAsExecutionModel) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(0, -c.maximum_tire_steering_angle_rad);
  ReferenceSpaceMppiPlanner planner(c); TemporaryReference a, b; Evaluation e;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner, r, &a, &e));
  r.ego.steering_rad -= 1e-5;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner, r, &b, &e));
  for(std::size_t i = 0; i < a.count; ++i) {
    EXPECT_DOUBLE_EQ(a.points[i].x_m, b.points[i].x_m);
    EXPECT_DOUBLE_EQ(a.points[i].y_m, b.points[i].y_m);
  }
}
TEST(EntryConnector, InteriorScreenDoesNotAcceptOnlyMatchingEndpoints) {
  const auto q = entry_connector::makeQuintic({4, 2.5}, {1, 0}, {1, 0}, 0, 0, 4);
  EXPECT_NEAR(entry_connector::cross(q.tangent(0), q.second(0)), 0, 1e-12);
  EXPECT_NEAR(entry_connector::cross(q.tangent(1), q.second(1)), 0, 1e-12);
  EXPECT_FALSE(entry_connector::withinCurvature(q, .3));
}
TEST(EntryConnector, GeometryDoesNotDependOnProposedSpeedOrFrenetJacobian) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(2.5); r.ego.d_m = 2;
  ReferenceSpaceMppiPlanner planner(c);
  TemporaryReference a, b; Evaluation e;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner, r, &a, &e));
  r.base_reference[0].curvature_1pm = 2; // A noisy/folded base normal is not a Cartesian derivative.
  for (auto &p : r.base_reference) p.speed_mps = 8;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner, r, &b, &e));
  ASSERT_EQ(a.count, b.count);
  for (std::size_t i = 0; i < a.count; ++i) {
    EXPECT_DOUBLE_EQ(a.points[i].x_m, b.points[i].x_m);
    EXPECT_DOUBLE_EQ(a.points[i].y_m, b.points[i].y_m);
  }
}
TEST(EntryConnector, UnavailableConnectionIsNotReportedAsWallOrCollision) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(8);
  r.base_reference_count = 11; // Two metres of road cannot contain this connection.
  TemporaryReference ref;
  const auto e = PlannerEvaluationTestAccess::evaluate(ReferenceSpaceMppiPlanner(c), r, &ref);
  EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.reject_reason, RejectReason::GEOMETRY);
  EXPECT_STREQ(e.reject_stage, "entry_connection");
}
TEST(EntryConnector, DelayedExecutionStillUsesPendingCommandsAndInitialSpeed) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(2.5, -.15);
  r.ego.pending_steering_target_count = 4;
  r.ego.pending_steering_targets_rad = {.4, .3, -.2, -.4};
  TemporaryReference ref;
  const auto e = PlannerEvaluationTestAccess::evaluate(ReferenceSpaceMppiPlanner(c), r, &ref);
  ASSERT_TRUE(e.valid);
  ASSERT_GE(e.predicted_rollout_count, 4U);
  double x = 0, y = 0, yaw = 0, steering = r.ego.steering_rad;
  for (std::size_t i = 0; i < 4; ++i) {
    const double target = r.ego.pending_steering_targets_rad[i] * c.steering_command_to_tire_angle_ratio;
    const double step = (1 - std::exp(-c.dt_sec / c.steering_time_constant_sec)) * (target - steering);
    const double limit = c.maximum_steering_rate_radps * c.steering_command_to_tire_angle_ratio * c.dt_sec;
    steering += std::clamp(step, -limit, limit);
    x += r.ego.speed_mps * std::cos(yaw) * c.dt_sec;
    y += r.ego.speed_mps * std::sin(yaw) * c.dt_sec;
    yaw += r.ego.speed_mps / c.wheel_base_m * std::tan(steering) * c.dt_sec;
    EXPECT_NEAR(e.predicted_rollout[i].x_m, x, 1e-12);
    EXPECT_NEAR(e.predicted_rollout[i].y_m, y, 1e-12);
    EXPECT_NEAR(e.predicted_rollout[i].yaw_rad, yaw, 1e-12);
    EXPECT_DOUBLE_EQ(e.predicted_rollout[i].speed_mps, r.ego.speed_mps);
  }
}
TEST(EntryConnector, GeometricFeasibilityDoesNotBypassSweptWallsOrVehicles) {
  Config c; c.collision_only_rejection = true;
  auto r = offsetRequest(2.5);
  TemporaryReference ref;
  bool checked = false;
  r.rollout_constraint_validator = [&](const RolloutState &, const RolloutState &) {
    checked = true; return RejectReason::WALL;
  };
  auto e = PlannerEvaluationTestAccess::evaluate(ReferenceSpaceMppiPlanner(c), r, &ref);
  EXPECT_TRUE(checked); EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.reject_reason, RejectReason::WALL);
  r.rollout_constraint_validator = {};
  r.path_constraint_validator = [](const TemporaryReference &) { return RejectReason::WALL; };
  e = PlannerEvaluationTestAccess::evaluate(ReferenceSpaceMppiPlanner(c), r, &ref);
  EXPECT_FALSE(e.valid); EXPECT_EQ(e.reject_reason, RejectReason::WALL);
  EXPECT_STREQ(e.reject_stage, "environment_reference_path");
  r.path_constraint_validator = {};
  r.dynamic_obstacle_count = 1; // Vehicle overlapping the starting footprint.
  e = PlannerEvaluationTestAccess::evaluate(ReferenceSpaceMppiPlanner(c), r, &ref);
  EXPECT_FALSE(e.valid); EXPECT_EQ(e.reject_reason, RejectReason::COLLISION);
}
} // namespace
} // namespace reference_space_mppi_planner::mppi
