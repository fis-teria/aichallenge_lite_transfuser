#include "reference_space_mppi_planner/batch_optimizer.hpp"
#include "reference_space_mppi_planner/cma_preview_curvature.hpp"
#include "reference_space_mppi_planner/arrival_time.hpp"
#include "reference_space_mppi_planner/continuous_prefix.hpp"
#include "reference_space_mppi_planner/generation_freshness.hpp"
#include "reference_space_mppi_planner/lateral_line_library.hpp"
#include "reference_space_mppi_planner/lateral_sampling.hpp"
#include "reference_space_mppi_planner/local_horizon.hpp"
#include "reference_space_mppi_planner/local_envelope_diagnostic.hpp"
#include "reference_space_mppi_planner/spatial_noise.hpp"
#include "reference_space_mppi_planner/opponent_prediction.hpp"
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/side_selection.hpp"
#include "reference_space_mppi_planner/sequenced_output.hpp"
#include "reference_space_mppi_planner/static_wall_map.hpp"
#include "reference_space_mppi_planner/steering_delay.hpp"
#include "reference_space_mppi_planner/target_selection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <thread>

namespace reference_space_mppi_planner::mppi {
struct PlannerEvaluationTestAccess {
  static bool execution(const ReferenceSpaceMppiPlanner &planner,
      const RolloutState &a, const RolloutState &b, const PlanRequest &request, Evaluation *e) {
    return planner.validateExecutionSegment(a,b,request,0,e);
  }
  static void noise(const ReferenceSpaceMppiPlanner &planner,
      const PlanRequest &request, std::size_t count, Scratch *scratch) {
    planner.generateNoise(request, count, scratch);
  }
  static bool swept(const ReferenceSpaceMppiPlanner &planner,
      const TemporaryReference &reference, const PlanRequest &request, Evaluation *evaluation) {
    return planner.finalSweptValidate(reference, request, evaluation);
  }
  static void setWarm(ReferenceSpaceMppiPlanner &planner, const PlanRequest &request,
      const ReferenceControlSequence &control) {
    planner.warm_mean_ = request.nominal;
    planner.warm_mean_valid_ = true;
    planner.warm_side_ = request.side;
    planner.warm_phase_ = request.phase;
    planner.warm_semantic_key_ = request.semantic_key;
    planner.warm_control_sequence_ = control;
    planner.warm_control_sequence_valid_ = true;
    planner.warm_stamp_sec_ = request.stamp_sec;
    planner.warm_anchor_s_m_ = request.anchor_s_m;
    planner.warm_control_horizon_m_ = request.base_reference[request.base_reference_count-1].s_m-request.anchor_s_m;
    planner.warm_speed_horizon_m_ = request.base_reference[request.base_reference_count-1].s_m-request.base_reference[0].s_m;
    planner.warm_base_reference_=request.base_reference;
    planner.warm_base_reference_count_=request.base_reference_count;
  }
  static ReferenceControlSequence warm(const ReferenceSpaceMppiPlanner &planner,
      const PlanRequest &request) { return planner.controlSequenceMean(request); }
  static ReferenceControlSequence staged(const ReferenceSpaceMppiPlanner &planner,
      const PlanRequest &request, const Parameters &parameters,
      ReferenceControlSequence control, std::size_t profile) {
    return planner.stagedSpeedProfile(request, parameters, control, profile);
  }
  static bool generate(const ReferenceSpaceMppiPlanner &planner,
      const Parameters &parameters, const PlanRequest &request,
      const ReferenceControlSequence *control, TemporaryReference *reference) {
    Evaluation evaluation;
    return planner.generateReference(parameters, request, control, reference, &evaluation);
  }
  static Evaluation evaluate(const ReferenceSpaceMppiPlanner &planner,
      const Parameters &parameters, const PlanRequest &request,
      const ReferenceControlSequence *control, TemporaryReference *reference) {
    return planner.evaluate(parameters, request, control, reference);
  }
};

namespace {

TEST(LocalEnvelopeDiagnostic, EmptyAabbCornerIsNotBodyOverlap) {
  const double yaw=std::acos(-1.0)/4;
  const double extent=2*(1.1+.65)/std::sqrt(2.0);
  ASSERT_LT(2.4,extent); // Old road-axis rectangle contains this point.
  const auto q=localEnvelopeDiagnostic(2.4,-2.4,yaw,yaw,1.1,.65,0,0);
  ASSERT_TRUE(q.valid);
  EXPECT_FALSE(q.overlap);
  EXPECT_GT(q.separating_gap_m,0);
}

TEST(LocalEnvelopeDiagnostic, TranslationUncertaintyAndContactAreRetained) {
  EXPECT_FALSE(localEnvelopeDiagnostic(3,0,0,0,1.1,.65,0,0).overlap);
  EXPECT_TRUE(localEnvelopeDiagnostic(3,0,0,0,1.1,.65,1,0).overlap);
  EXPECT_TRUE(localEnvelopeDiagnostic(2.2,0,0,0,1.1,.65,0,0).overlap);
  EXPECT_FALSE(localEnvelopeDiagnostic(2.20001,0,0,0,1.1,.65,0,0).overlap);
  EXPECT_TRUE(localEnvelopeDiagnostic(0,1.3,0,0,1.1,.65,0,0).overlap);
}

TEST(LocalEnvelopeDiagnostic, InvalidInputIsUnknownNotClear) {
  EXPECT_FALSE(localEnvelopeDiagnostic(NAN,0,0,0,1.1,.65,0,0).valid);
  EXPECT_FALSE(localEnvelopeDiagnostic(0,0,0,0,1.1,.65,-1,0).valid);
  EXPECT_FALSE(localEnvelopeDiagnostic(0,0,0,0,0,.65,0,0).valid);
  EXPECT_FALSE(localEnvelopeDiagnostic(0,0,0,0,1e308,.65,0,0).valid);
}

TEST(LocalEnvelopeDiagnostic, EverySampledTranslatedBodyOverlapRemainsInUncertaintySet) {
  std::size_t overlaps=0;
  for(double ey:{-.8,0.,.7}) for(double oy:{-.5,0.,1.1})
    for(int i=-8;i<=8;++i) for(int j=-8;j<=8;++j) {
      const double s=i*.5,d=j*.5;
      const auto uncertain=localEnvelopeDiagnostic(s,d,ey,oy,1.1,.65,.7,.3);
      ASSERT_TRUE(uncertain.valid);
      for(double u:{-.7,0.,.7}) for(double v:{-.3,0.,.3}) {
        if(localEnvelopeDiagnostic(s-u,d-v,ey,oy,1.1,.65,0,0).overlap) {
          ++overlaps; EXPECT_TRUE(uncertain.overlap);
        }
      }
    }
  EXPECT_GT(overlaps,0U);
}

TEST(CollisionWitness, SameTimeEnvelopesDoNotChangeRejection) {
  Config config;
  config.vehicle_half_length_m=1.10;
  config.vehicle_half_width_m=.65;
  config.obstacle_longitudinal_inflation_m=0;
  config.obstacle_lateral_inflation_m=0;
  ReferenceSpaceMppiPlanner planner(config);
  PlanRequest request;
  request.base_reference_count=3;
  for (std::size_t i=0;i<3;++i) {
    request.base_reference[i].s_m=request.base_reference[i].x_m=i*10;
  }
  request.dynamic_obstacle_count=1;
  auto &o=request.dynamic_obstacles[0];
  o.s_m=3.0; o.longitudinal_acceleration_bound_mps2=.5;
  const auto contains=[](const std::array<double,4> &box) {
    return box[0]<=0 && box[1]>=0 && box[2]<=0 && box[3]>=0;
  };
  Evaluation e;
  EXPECT_FALSE(PlannerEvaluationTestAccess::execution(planner,{0,0,0,0,1.9},{0,0,0,0,2},request,&e));
  EXPECT_EQ(e.reject_reason,RejectReason::COLLISION);
  EXPECT_FALSE(contains(e.reject_nominal_envelope));
  EXPECT_FALSE(contains(e.reject_position_envelope));
  EXPECT_TRUE(contains(e.reject_obstacle_envelope));
  EXPECT_DOUBLE_EQ(e.reject_yaw_rad,0);
  EXPECT_TRUE(e.reject_dynamic_obstacle);
  EXPECT_DOUBLE_EQ(e.reject_reference_yaw_rad,0);
  o.longitudinal_acceleration_bound_mps2=0;
  o.longitudinal_uncertainty_m=2;
  EXPECT_FALSE(PlannerEvaluationTestAccess::execution(planner,{0,0,0,0,1.9},{0,0,0,0,2},request,&e));
  EXPECT_FALSE(contains(e.reject_nominal_envelope));
  EXPECT_TRUE(contains(e.reject_position_envelope));
  o.longitudinal_uncertainty_m=0;
  EXPECT_TRUE(PlannerEvaluationTestAccess::execution(planner,{0,0,0,0,2.9},{0,0,0,0,3},request,&e));
  o.s_m=1;
  EXPECT_FALSE(PlannerEvaluationTestAccess::execution(planner,{0,0,0,0,2.9},{0,0,0,0,3},request,&e));
  EXPECT_TRUE(contains(e.reject_nominal_envelope));
  o.s_m=100;
  request.static_obstacle_count=1;
  request.static_obstacles[0]=StaticObstacle{-1,1,-1,1};
  EXPECT_FALSE(PlannerEvaluationTestAccess::execution(planner,{0,0,0,0,2.9},{0,0,0,0,3},request,&e));
  EXPECT_FALSE(e.reject_dynamic_obstacle);
  EXPECT_TRUE(std::isnan(e.reject_reference_yaw_rad));
}

TEST(WorldOccupancyExecution, RejectsBodiesNotEmptyRoadAxisCorners) {
  Config c; c.vehicle_half_length_m=1.1; c.vehicle_half_width_m=.65;
  c.collision_only_rejection=true;
  c.obstacle_longitudinal_inflation_m=0; c.obstacle_lateral_inflation_m=0;
  ReferenceSpaceMppiPlanner planner(c);
  PlanRequest r; r.base_reference_count=3; r.dynamic_obstacle_count=1;
  std::vector<std::array<double,2>> points{{0,0},{10,0},{20,0}};
  for(int i=0;i<3;++i) r.base_reference[i].s_m=r.base_reference[i].x_m=i*10;
  r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](auto p){return p;});
  auto &o=r.dynamic_obstacles[0]; o.s_m=o.global_reference_s_m=5;
  o.heading_relative_to_reference_rad=M_PI/4;
  Evaluation e;
  const RolloutState a{7.4,-2.4,M_PI/4,0,0}, b{7.4,-2.4,M_PI/4,0,.01};
  EXPECT_TRUE(PlannerEvaluationTestAccess::execution(planner,a,b,r,&e));
  r.valid=true; r.horizon_steps_override=2;
  r.ego.x_m=a.x_m; r.ego.y_m=a.y_m; r.ego.yaw_rad=a.yaw_rad;
  TemporaryReference reference; reference.count=3;
  for(int i=0;i<3;++i) {
    reference.points[i].x_m=reference.points[i].s_m=i*10;
    reference.points[i].y_m=a.y_m;
  }
  EXPECT_TRUE(planner.validateReference(reference,r).valid);
  r.rollout_constraint_validator=[](auto,auto){return RejectReason::WALL;};
  EXPECT_EQ(planner.validateReference(reference,r).reject_reason,RejectReason::WALL);
  r.rollout_constraint_validator={};
  o.longitudinal_uncertainty_m=3; o.lateral_uncertainty_m=3;
  EXPECT_FALSE(PlannerEvaluationTestAccess::execution(planner,a,b,r,&e));
  EXPECT_TRUE(e.reject_dynamic_obstacle);
}

TEST(CollisionUncertainty, NearTubeTapersButLateBodiesAndStaticObstaclesStillReject) {
  Config c;
  c.obstacle_longitudinal_inflation_m=0;
  c.obstacle_lateral_inflation_m=0;
  ReferenceSpaceMppiPlanner planner(c);
  for (bool world : {false,true}) {
    SCOPED_TRACE(world);
    PlanRequest r;
    r.base_reference_count=3;
    r.dynamic_obstacle_count=1;
    for (int i=0;i<3;++i) r.base_reference[i].s_m=r.base_reference[i].x_m=i*10;
    if(world) {
      std::vector<std::array<double,2>> points{{0,0},{10,0},{20,0}};
      r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](auto p){return p;});
    }
    auto &o=r.dynamic_obstacles[0];
    o.s_m=o.global_reference_s_m=3.1;
    o.longitudinal_acceleration_bound_mps2=.5;
    const auto passes=[&](double t) {
      Evaluation e;
      return PlannerEvaluationTestAccess::execution(
          planner,{0,0,0,0,t},{0,0,0,0,t+.001},r,&e);
    };
    EXPECT_FALSE(passes(2.0));
    EXPECT_TRUE(passes(2.5));
    EXPECT_TRUE(passes(3.0));
    EXPECT_TRUE(passes(5.0));
    o.longitudinal_uncertainty_m=2;
    EXPECT_FALSE(passes(2.0));
    EXPECT_TRUE(passes(3.0));
    o.s_m=o.global_reference_s_m=0;
    o.d_m=2;
    o.lateral_uncertainty_m=1;
    EXPECT_FALSE(passes(2.0));
    EXPECT_TRUE(passes(3.0));
    o.d_m=0;
    EXPECT_FALSE(passes(3.0));
    EXPECT_FALSE(passes(5.0));
    o.s_m=o.global_reference_s_m=100;
    r.static_obstacle_count=1;
    r.static_obstacles[0]=StaticObstacle{-1,1,-1,1};
    EXPECT_FALSE(passes(3.0));
  }
}

TEST(CollisionUncertainty, FullEvaluationAllowsFarTubeAndRetainsSoftCost) {
  Config c;
  c.collision_only_rejection=true;
  c.horizon_steps=60;
  c.cost_clearance_weight=1;
  ReferenceSpaceMppiPlanner planner(c);
  for (bool world : {false,true}) {
    SCOPED_TRACE(world);
    PlanRequest r;
    r.valid=true;
    r.base_reference_count=3;
    TemporaryReference reference;
    reference.count=3;
    for (int i=0;i<3;++i) {
      r.base_reference[i].s_m=r.base_reference[i].x_m=i*10;
      r.base_reference[i].minimum_d_m=-10;
      r.base_reference[i].maximum_d_m=10;
      reference.points[i].s_m=reference.points[i].x_m=i*10;
    }
    if(world) {
      std::vector<std::array<double,2>> points{{0,0},{10,0},{20,0}};
      r.world_reference=std::make_shared<ReferencePoseIndex>(points,[](auto p){return p;});
    }
    r.dynamic_obstacle_count=1;
    auto &o=r.dynamic_obstacles[0];
    o.s_m=o.global_reference_s_m=7;
    o.longitudinal_speed_mps=-1;
    o.longitudinal_acceleration_bound_mps2=.5;
    const auto uncertain=planner.validateReference(reference,r);
    ASSERT_TRUE(uncertain.valid);
    EXPECT_GT(uncertain.cost_terms[1],0);
    o.longitudinal_acceleration_bound_mps2=0;
    const auto nominal=planner.validateReference(reference,r);
    ASSERT_TRUE(nominal.valid);
    EXPECT_GT(uncertain.cost_terms[1],nominal.cost_terms[1]);
  }
}

TEST(TrackingContract, StationaryStageAndTerminalUseSameGeometricExcess) {
  Config config;
  config.collision_only_rejection = true;
  config.cost_pp_error_weight = 2;
  config.cost_terminal_lateral_weight = 2;
  ReferenceSpaceMppiPlanner planner(config);
  PlanRequest request;
  request.valid = true;
  request.sample_staged_speed = true;
  request.horizon_steps_override = 2;
  request.base_reference_count = 3;
  request.base_reference[1].x_m = request.base_reference[1].s_m = 10;
  request.base_reference[2].x_m = request.base_reference[2].s_m = 20;
  for (auto &p : request.base_reference) {
    p.minimum_d_m = -10; p.maximum_d_m = 10;
  }
  TemporaryReference reference;
  reference.count = 3;
  reference.points[1].x_m = reference.points[1].s_m = 10;
  reference.points[2].x_m = reference.points[2].s_m = 20;
  for (double error : {0.3, 0.8, 2.4}) {
    request.ego.y_m = error;
    const auto result = planner.validateReference(reference, request);
    ASSERT_TRUE(result.valid);
    const double excess = std::max(0.0, error - config.maximum_cross_track_error_m) /
        config.maximum_cross_track_error_m;
    EXPECT_NEAR(result.cost_terms[4], 2 * excess * excess, 1e-9);
    EXPECT_NEAR(result.cost_terms[7], result.cost_terms[4], 1e-9);
  }
  // A farther next waypoint must not pin the geometric distance at the head.
  request.ego.x_m = 2; request.ego.y_m = 0;
  reference.count = 5;
  const double xs[] = {0, 0, 2, 2, 20};
  const double ys[] = {0, 3, 3, 0, 0};
  for (std::size_t i = 0; i < reference.count; ++i) {
    reference.points[i].x_m = xs[i]; reference.points[i].y_m = ys[i];
    reference.points[i].s_m = i;
  }
  const auto result = planner.validateReference(reference, request);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.maximum_cross_track_error_m, 0);
  EXPECT_DOUBLE_EQ(result.cost_terms[4], 0);
  EXPECT_DOUBLE_EQ(result.cost_terms[7], 0);
  request.rollout_constraint_validator = [](const RolloutState &, const RolloutState &) {
    return RejectReason::WALL;
  };
  EXPECT_EQ(planner.validateReference(reference, request).reject_reason, RejectReason::WALL);
}

TEST(BoundaryProjection, NonMonotonePointDistancesDoNotPinBoundaryAtStart) {
  for (double side : {-1.0, 1.0}) {
    Config config;
    config.collision_only_rejection = true;
    ReferenceSpaceMppiPlanner planner(config);
    PlanRequest request;
    request.valid = true;
    request.sample_staged_speed = true;
    request.horizon_steps_override = 2;
    request.ego.x_m = 2;
    request.ego.y_m = side * 2;
    request.ego.speed_mps = 1;
    request.base_reference_count = 4;
    const double xs[] = {0, 1, 3, 20};
    const double ys[] = {0, -side, 0, 0};
    for (std::size_t i=0; i<4; ++i) {
      auto &p=request.base_reference[i];
      p.x_m=xs[i]; p.y_m=ys[i];
      p.s_m=i?request.base_reference[i-1].s_m+std::hypot(xs[i]-xs[i-1],ys[i]-ys[i-1]):0;
      p.minimum_d_m=i?-5:-.5; p.maximum_d_m=i?5:.5;
    }
    TemporaryReference reference;
    reference.count=19;
    for (std::size_t i=0;i<reference.count;++i) {
      auto &p=reference.points[i];
      p.s_m=i; p.x_m=2+i; p.y_m=side*2; p.speed_mps=1;
    }
    // The next waypoint is farther, but the next segment is closer. Both
    // lateral position and boundary must come from that projection region.
    const auto result=planner.validateReference(reference,request);
    ASSERT_TRUE(result.valid);
    EXPECT_DOUBLE_EQ(result.cost_terms[1],0);
    request.rollout_constraint_validator=[](const RolloutState &,const RolloutState &) {return RejectReason::WALL;};
    EXPECT_EQ(planner.validateReference(reference,request).reject_reason,RejectReason::WALL);
  }
}

TEST(ArrivalTime, BoundedAccelerationCruiseBrakingAndStop) {
  EXPECT_NEAR(segmentArrival(18, .664, 10, 2, 2).time_sec,
              (std::sqrt(.664*.664+72)-.664)/2, 1e-12);
  EXPECT_GT(segmentArrival(18, .664, 10, 2, 2).time_sec, 3);
  EXPECT_DOUBLE_EQ(segmentArrival(10, 0, 2, 2, 2).time_sec, 5.5);
  EXPECT_DOUBLE_EQ(segmentArrival(5, 4, 2, 2, 2).time_sec, 2);
  EXPECT_DOUBLE_EQ(segmentArrival(4, 4, 0, 2, 2).time_sec, 2);
  EXPECT_TRUE(std::isinf(segmentArrival(5, 4, 0, 2, 2).time_sec));
  EXPECT_TRUE(std::isinf(segmentArrival(1, 0, 0, 2, 2).time_sec));
  EXPECT_DOUBLE_EQ(segmentArrival(0, 0, 0, 2, 2).time_sec, 0);
  const auto first = segmentArrival(1, .664, 10, 2, 2);
  const auto second = segmentArrival(17, first.speed_mps, 10, 2, 2);
  EXPECT_NEAR(first.time_sec+second.time_sec, segmentArrival(18,.664,10,2,2).time_sec, 1e-12);
}

TEST(ArrivalTime, SweptUsesReachableHorizonAndStillRejectsNearCollision) {
  auto config = Config{};
  config.maximum_acceleration_mps2 = 2;
  ReferenceSpaceMppiPlanner planner(config);
  PlanRequest request;
  request.valid = true;
  request.ego.speed_mps = .664;
  request.complete_maneuver = false;
  request.horizon_steps_override = 60;
  request.base_reference_count = 21;
  request.dynamic_obstacle_count = 1;
  request.dynamic_obstacles[0].s_m = 18;
  TemporaryReference reference;
  reference.count = 21;
  for (std::size_t i = 0; i < 21; ++i) {
    reference.points[i].s_m = reference.points[i].x_m = i;
    reference.points[i].speed_mps = 10;
    request.base_reference[i].s_m = request.base_reference[i].x_m = i;
  }
  // Geometry completeness does not extend the executable 3 s clock. This
  // test keeps the original far/near/fast collision fixtures and exercises
  // the public closed-loop validator instead of a private ideal arrival rule.
  EXPECT_TRUE(planner.validateReference(reference, request).valid);
  request.dynamic_obstacles[0].s_m = 6;
  const auto near = planner.validateReference(reference, request);
  EXPECT_FALSE(near.valid);
  EXPECT_EQ(near.reject_reason, RejectReason::COLLISION);
  request.dynamic_obstacles[0].s_m = 18;
  request.complete_maneuver = true;
  EXPECT_TRUE(planner.validateReference(reference, request).valid);
  request.complete_maneuver = false;
  request.ego.speed_mps = 10;
  EXPECT_FALSE(planner.validateReference(reference, request).valid);
}

TEST(ContinuousPrefix, SamplesAcceptedPathInBaseNormalFrame) {
  const auto offset = offsetAtNormal(2, 0, 0, 0, 1, 4, 3);
  ASSERT_TRUE(offset);
  EXPECT_DOUBLE_EQ(*offset, 2);
  EXPECT_FALSE(offsetAtNormal(5, 0, 0, 0, 1, 4, 3));
  EXPECT_FALSE(offsetAtNormal(2, 0, 0, 4, 3, 0, 1));
}

TEST(ContinuousPrefix, RemovesObservedNineCentimeterSnapWithoutClippingPose) {
  PlanRequest request;
  request.ego.d_m = -.520884937239;
  request.base_reference_count = 52;
  for (std::size_t i = 0; i < request.base_reference_count; ++i) {
    auto &p = request.base_reference[i];
    p.s_m = i == 0 ? 0.0 : .0886331589331 + (i - 1) * .1;
    p.active_d_m = -.00436966039062;
    p.minimum_d_m = -.1;  // Bounds must not snap the measured start to a wall.
  }
  connectMeasuredPrefix(request, 4);
  const auto &p = request.base_reference;
  EXPECT_DOUBLE_EQ(p[0].active_d_m, request.ego.d_m);
  EXPECT_LT(std::abs(p[1].active_d_m - p[0].active_d_m), .001);
  EXPECT_DOUBLE_EQ(p[51].active_d_m, -.00436966039062);
  for (std::size_t i = 1; i + 1 < request.base_reference_count; ++i) {
    const double left = (p[i].active_d_m - p[i-1].active_d_m) / (p[i].s_m-p[i-1].s_m);
    const double right = (p[i+1].active_d_m - p[i].active_d_m) / (p[i+1].s_m-p[i].s_m);
    const double second = 2*(right-left)/(p[i+1].s_m-p[i-1].s_m);
    EXPECT_LT(std::abs(second), .25);
  }
}

TEST(ContinuousPrefix, StartsAlongMeasuredHeadingAndRetainsAcceptedSuffix) {
  PlanRequest request;
  request.ego.d_m = .3;
  request.ego.yaw_rad = .15;
  request.base_reference_count = 62;
  for (std::size_t i = 0; i < request.base_reference_count; ++i) {
    auto &p = request.base_reference[i];
    p.s_m = i == 0 ? 0 : .0001 + (i - 1)*.1;
    p.active_d_m = .7 + .05*p.s_m;
  }
  connectMeasuredPrefix(request, 4);
  const auto &p = request.base_reference;
  EXPECT_NEAR((p[1].active_d_m-p[0].active_d_m)/p[1].s_m,
              std::tan(.15), 1e-6);
  EXPECT_DOUBLE_EQ(p[61].active_d_m, .7+.05*p[61].s_m);
}

TEST(SpatialNoise, MatchesReferenceScaleAndPreservesPhysicalCorrelationLength) {
  EXPECT_DOUBLE_EQ(spatialKnotRatio(56, 8, 8), 1);
  EXPECT_DOUBLE_EQ(spatialLateralAmplitude(1), 1);
  EXPECT_DOUBLE_EQ(spatialKnotRatio(20, 8, 0), 1);
  EXPECT_NEAR(spatialLateralAmplitude(1.0 / 3), 1.0 / 9, 1e-12);
  EXPECT_NEAR(std::pow(spatialCorrelation(.75, 1.0 / 3), 3), .75, 1e-12);
  EXPECT_DOUBLE_EQ(spatialCorrelation(0, .5), 0);
  EXPECT_DOUBLE_EQ(spatialCorrelation(1, .5), 1);
}

TEST(LocalHorizon, ContinuousDistanceAndReachablePrediction) {
  EXPECT_DOUBLE_EQ(localHorizonDistance(0, 20, 30, 3), 20);
  EXPECT_DOUBLE_EQ(localHorizonDistance(8, 20, 30, 3), 24);
  EXPECT_DOUBLE_EQ(localHorizonDistance(10, 20, 30, 3), 30);
  EXPECT_EQ(localHorizonSteps(20, 3.1, 0, 10, 2, .05, 60), 60U);
  const auto steps = localHorizonSteps(30, 3.1, 10, 10, 2, .05, 60);
  EXPECT_EQ(steps, 53U);
  EXPECT_LE(steps * .05 * 10 + 3.1, 30);
}

TEST(LocalHorizon, WarmKnotsRetainPhysicalPositionsAcrossResizing) {
  EXPECT_DOUBLE_EQ(previousKnotCoordinate(.5, 20, 0, 30), 1.0 / 3.0);
  EXPECT_DOUBLE_EQ(previousKnotCoordinate(.5, 30, 0, 20), .75);
  EXPECT_DOUBLE_EQ(previousKnotCoordinate(.5, 20, 2, 30), .4);
  EXPECT_DOUBLE_EQ(previousKnotCoordinate(1, 30, 2, 20), 1);
}

TEST(LocalHorizon, SuppliedGeometryCoversConsumptionAndFutureLocalWindow) {
  EXPECT_DOUBLE_EQ(suppliedReferenceDistance(10,20,30,3,1,10,2),40.);
  EXPECT_DOUBLE_EQ(suppliedReferenceDistance(8,20,30,3,1,10,2),39.);
  EXPECT_DOUBLE_EQ(suppliedReferenceDistance(7,20,30,3,1,10,2),35.);
  for (double initial : {0., 3., 7., 8.3, 10., 12.}) {
    EXPECT_DOUBLE_EQ(suppliedReferenceDistance(initial,20,30,3,0,10,2),
                    localHorizonDistance(initial,20,30,3));
    const double length=suppliedReferenceDistance(initial,20,30,3,1,10,2);
    double position=0.,speed=initial;
    // Trapezoidal integration independently checks the supplied coverage
    // throughout a one-second accelerating hold, including speed saturation.
    for(int i=0;i<1000;++i) {
      const double next=std::min(std::max(initial,10.),speed+.002);
      position+=(speed+next)*.0005; speed=next;
      EXPECT_GE(length-position+1e-8,localHorizonDistance(speed,20,30,3));
    }
  }
}

TEST(ReferenceSpaceMppiOutput, HeartbeatsDoNotSupersedeOptimizerSnapshots) {
  struct Command {
    std::uint64_t generation;
    double valid_until_sec;
  };
  SequencedOutput output;
  std::vector<Command> received;
  const auto publish = [&](const Command &command) { received.push_back(command); };
  output.publish(Command{100U, 10.20}, publish);
  output.publish(Command{101U, 10.25}, publish);
  output.publish(Command{100U, 10.20}, publish);
  ASSERT_EQ(received.size(), 3U);
  EXPECT_EQ(received[0].generation, 1U);
  EXPECT_EQ(received[1].generation, 2U);
  EXPECT_EQ(received[2].generation, 3U);
  EXPECT_DOUBLE_EQ(received[2].valid_until_sec, 10.20);
}

TEST(ReferenceSpaceMppiOutput, ConcurrentPublishingRetainsWireOrder) {
  struct Command { std::uint64_t generation; };
  SequencedOutput output;
  std::vector<std::uint64_t> received;
  const auto publish = [&](const Command &command) {
    received.push_back(command.generation);
  };
  std::array<std::thread, 2> workers;
  for (auto &worker : workers) {
    worker = std::thread([&] {
      for (int i = 0; i < 100; ++i) output.publish(Command{0U}, publish);
    });
  }
  for (auto &worker : workers) worker.join();
  ASSERT_EQ(received.size(), 200U);
  for (std::size_t i = 0; i < received.size(); ++i) EXPECT_EQ(received[i], i + 1U);
}

TEST(ReferenceSpaceMppiLateralSampling, NarrowGapUsesFootprintsNotPreferredWidth) {
  const auto interval = lateral_sampling::feasibleSeparationIntervalFromAvailableWidth(
      1.45, 0.65, 0.65, 1.15, 0.05, 0.0, 2.2);
  ASSERT_TRUE(interval.feasible);
  EXPECT_NEAR(interval.minimum_separation_m, 1.35, 1e-9);
  EXPECT_DOUBLE_EQ(interval.maximum_separation_m, 1.45);
  EXPECT_FALSE(lateral_sampling::feasibleSeparationIntervalFromAvailableWidth(
      1.34, 0.65, 0.65, 1.15, 0.05, 0.0, 2.2).feasible);
}

TEST(ReferenceSpaceMppiLateralSampling, SlowGapEntryRetainsGentleFastEntry) {
  EXPECT_DOUBLE_EQ(lateral_sampling::minimumTransitionLength(0.0), 4.0);
  EXPECT_DOUBLE_EQ(lateral_sampling::minimumTransitionLength(1.0), 4.0);
  EXPECT_DOUBLE_EQ(lateral_sampling::minimumTransitionLength(3.0), 7.5);
  EXPECT_DOUBLE_EQ(lateral_sampling::minimumTransitionLength(6.0), 15.0);
  EXPECT_DOUBLE_EQ(lateral_sampling::minimumTransitionLength(10.0), 15.0);
}

TEST(ReferenceSpaceMppiLateralSampling, GentleEntryBoundsAccelerationOnAStraight) {
  const double speed=8.0, lateral=2.0, limit=1.5;
  const double length=lateral_sampling::gentleTransitionLength(speed,lateral,limit);
  double peak=0.0;
  constexpr double du=1e-4;
  for (double u=du;u<1.0-du;u+=.001) {
    const auto blend=[](double s) {
      return ReferenceSpaceMppiPlanner::multiPointLateralBlend(s,2.0/7.0,5.0/7.0);
    };
    EXPECT_NEAR(blend(u),ReferenceSpaceMppiPlanner::quinticBlend(u),1e-12);
    const double dd=(blend(u+du)-2*blend(u)+blend(u-du))/(du*du);
    peak=std::max(peak,std::abs(dd)*lateral*speed*speed/(length*length));
  }
  EXPECT_NEAR(peak,limit,1e-4);
  EXPECT_GT(length,12.0);
  EXPECT_GT(lateral_sampling::gentleTransitionLength(10.,lateral,limit),length);
}

TEST(ReferenceSpaceMppiGenerationFreshness,
     AcceptsBoundedBrainLagAndRejectsOlderOrFutureResults) {
  EXPECT_TRUE(generationWithinLag(100U, 100U, 3U));
  EXPECT_TRUE(generationAdoptable(100U, 110U, 3U, true));
  EXPECT_FALSE(generationAdoptable(100U, 110U, 3U, false));
  EXPECT_FALSE(generationAdoptable(110U, 100U, 3U, true));
  EXPECT_FALSE(generationAdoptable(100U, 101U, 0U, false));
  EXPECT_TRUE(generationWithinLag(100U, 103U, 3U));
  EXPECT_FALSE(generationWithinLag(100U, 104U, 3U));
  EXPECT_FALSE(generationWithinLag(101U, 100U, 3U));
  EXPECT_FALSE(generationWithinLag(100U, 101U, 0U));
}

nav_msgs::msg::OccupancyGrid wallTestMap() {
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = 0.1F;
  map.info.width = 100U;
  map.info.height = 100U;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(100U * 100U, 0);
  for (std::size_t y = 0U; y < map.info.height; ++y) {
    map.data[y * map.info.width + 50U] = 100;
  }
  return map;
}

TEST(ReferenceSpaceMppiWallMap, RejectsAThinWallInsideVehicleRectangle) {
  const auto map = wallTestMap();
  EXPECT_TRUE(occupancyGridFootprintFree(map, WallFootprintPose{3.0, 5.0, 0.0},
                                         1.0, 1.0, 0.5));
  EXPECT_FALSE(occupancyGridFootprintFree(map, WallFootprintPose{4.6, 5.0, 0.0},
                                          1.0, 1.0, 0.5));
}

TEST(ReferenceSpaceMppiWallMap, SweptPathCannotJumpAcrossWall) {
  const auto map = wallTestMap();
  const std::vector<WallFootprintPose> crossing_path{{3.5, 5.0, 0.0},
                                                     {6.5, 5.0, 0.0}};
  const std::vector<WallFootprintPose> free_path{{3.0, 3.0, M_PI_2},
                                                 {3.0, 7.0, M_PI_2}};
  EXPECT_FALSE(
      occupancyGridFootprintPathFree(map, crossing_path, 1.0, 1.0, 0.5, 0.05));
  EXPECT_TRUE(
      occupancyGridFootprintPathFree(map, free_path, 1.0, 1.0, 0.5, 0.05));
}

TEST(ReferenceSpaceMppiWallMap, UnknownAndOutsideMapAreWalls) {
  auto map = wallTestMap();
  map.data[20U * map.info.width + 20U] = -1;
  EXPECT_FALSE(occupancyGridFootprintFree(
      map, WallFootprintPose{2.05, 2.05, 0.0}, 0.1, 0.1, 0.1));
  EXPECT_FALSE(occupancyGridFootprintFree(
      map, WallFootprintPose{0.05, 5.0, 0.0}, 0.1, 0.1, 0.1));
}

TEST(ReferenceSpaceMppiWallMap, AppliesOccupancyGridOriginYaw) {
  auto map = wallTestMap();
  map.info.origin.orientation.z = std::sin(M_PI_4);
  map.info.origin.orientation.w = std::cos(M_PI_4);
  // Grid cell (50, 50) rotates from map (5.05, 5.05) to world
  // (-5.05, 5.05) for a +90 degree origin yaw.
  EXPECT_FALSE(occupancyGridFootprintFree(
      map, WallFootprintPose{-5.05, 5.05, M_PI_2}, 0.04, 0.04, 0.04));
  EXPECT_TRUE(occupancyGridFootprintFree(
      map, WallFootprintPose{-3.0, 3.0, M_PI_2}, 0.04, 0.04, 0.04));
}

TEST(ReferenceSpaceMppiWallMap, IndexedFootprintsMatchExactChecks) {
  std::mt19937 random(20260908);
  std::uniform_real_distribution<double> position(-1., 11.), yaw(-M_PI, M_PI);
  for (double origin_yaw : {0., .37, M_PI_2}) {
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(wallTestMap());
    map->info.origin.position.x = 12345.;
    map->info.origin.position.y = -9876.;
    map->info.origin.orientation.z = std::sin(origin_yaw / 2.);
    map->info.origin.orientation.w = std::cos(origin_yaw / 2.);
    map->data[20U * map->info.width + 20U] = -1;
    map->data[70U * map->info.width + 30U] = 49;
    map->data[70U * map->info.width + 31U] = 50;
    OccupancyGridWallIndex index(map);
    for (int i = 0; i < 2000; ++i) {
      const double x = position(random), y = position(random);
      const WallFootprintPose pose{
          map->info.origin.position.x + std::cos(origin_yaw) * x - std::sin(origin_yaw) * y,
          map->info.origin.position.y + std::sin(origin_yaw) * x + std::cos(origin_yaw) * y,
          yaw(random)};
      EXPECT_EQ(occupancyGridFootprintFree(*map, pose, 1.06, 1.10, .65),
                occupancyGridFootprintFree(*map, pose, 1.06, 1.10, .65, &index));
    }
  }
}

TEST(ReferenceSpaceMppiWallMap, IndexRetainsSnapshotAndCannotCertifyAnotherMap) {
  auto old_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(wallTestMap());
  auto new_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*old_map);
  new_map->data[30U * new_map->info.width + 30U] = -1;
  const WallFootprintPose pose{3.05, 3.05, 0.};
  std::weak_ptr<nav_msgs::msg::OccupancyGrid> retained = old_map;
  OccupancyGridWallIndex old_index(old_map), new_index(new_map);
  old_map.reset();
  ASSERT_FALSE(retained.expired());
  EXPECT_TRUE(occupancyGridFootprintFree(*retained.lock(), pose, .04, .04, .04, &old_index));
  EXPECT_FALSE(occupancyGridFootprintFree(*new_map, pose, .04, .04, .04, &old_index));
  EXPECT_FALSE(occupancyGridFootprintFree(*new_map, pose, .04, .04, .04, &new_index));
  const std::vector<WallFootprintPose> path{{3.5, 5., 0.}, {6.5, 5., 0.}};
  EXPECT_FALSE(occupancyGridFootprintPathFree(*new_map, path, 1., 1., .5, .05, &new_index));
}

TEST(ReferenceSpaceMppiWallMap, IndexPreservesRecordedMillimeterEdgeOverlap) {
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.resolution = .1F;
  map->info.width = 751; map->info.height = 759;
  map->info.origin.position.x = 89608.69387016428;
  map->info.origin.position.y = 43117.15165326744;
  map->info.origin.orientation.w = 1.;
  map->data.assign(map->info.width * map->info.height, 0);
  map->data[273U * map->info.width + 223U] = 100;
  const OccupancyGridWallIndex index(map);
  const WallFootprintPose pose{89631.532606523731, 43145.687933405788, -1.3784480491932065};
  EXPECT_FALSE(occupancyGridFootprintFree(*map, pose, 1.06, 1.10, .65));
  EXPECT_FALSE(occupancyGridFootprintFree(*map, pose, 1.06, 1.10, .65, &index));
}

TEST(ReferenceSpaceMppiTargetSelection,
     SelectsOnlyWhenPredictedPathFootprintsOverlap) {
  EXPECT_TRUE(
      target_selection::pathFootprintsConflict(1.50, 0.72, 0.72, 0.15, 0.05));
  EXPECT_FALSE(
      target_selection::pathFootprintsConflict(1.70, 0.72, 0.72, 0.15, 0.05));
}

TEST(ReferenceSpaceMppiTargetSelection, RejectsInvalidConflictGeometry) {
  EXPECT_FALSE(target_selection::pathFootprintsConflict(std::nan(""), 0.72,
                                                        0.72, 0.15, 0.05));
  EXPECT_FALSE(
      target_selection::pathFootprintsConflict(0.0, -0.72, 0.72, 0.15, 0.05));
}

TEST(ReferenceSpaceMppiTargetSelection,
     RetainsCommittedTargetUntilFootprintIsLongitudinallyClear) {
  EXPECT_TRUE(
      target_selection::committedTargetRetainsPriority(-7.99, 8.0, 40.0));
  EXPECT_TRUE(
      target_selection::committedTargetRetainsPriority(-8.0, 8.0, 40.0));
  EXPECT_TRUE(
      target_selection::committedTargetRetainsPriority(40.0, 8.0, 40.0));
  EXPECT_FALSE(
      target_selection::committedTargetRetainsPriority(-8.01, 8.0, 40.0));
  EXPECT_FALSE(
      target_selection::committedTargetRetainsPriority(40.01, 8.0, 40.0));
}

TEST(ReferenceSpaceMppiTargetSelection,
     RejectsInvalidCommittedTargetPriorityInput) {
  EXPECT_FALSE(target_selection::committedTargetRetainsPriority(std::nan(""),
                                                                8.0, 40.0));
  EXPECT_FALSE(
      target_selection::committedTargetRetainsPriority(0.0, -1.0, 40.0));
  EXPECT_FALSE(
      target_selection::committedTargetRetainsPriority(0.0, 8.0, -1.0));
}

TEST(ReferenceSpaceMppiTargetSelection,
     KeepsSidePreferenceOnlyForTheSameTarget) {
  EXPECT_EQ(target_selection::targetScopedPreferredSide("car_a", "car_a", -1),
            -1);
  EXPECT_EQ(target_selection::targetScopedPreferredSide("car_a", "car_b", -1),
            0);
  EXPECT_EQ(target_selection::targetScopedPreferredSide("", "car_b", 1), 0);
}

TEST(ReferenceSpaceMppiTargetSelection,
     FollowFallbackAppliesOnlyToAForwardTarget) {
  EXPECT_TRUE(target_selection::targetRequiresFollow(0.0));
  EXPECT_TRUE(target_selection::targetRequiresFollow(3.0));
  EXPECT_FALSE(target_selection::targetRequiresFollow(-0.01));
  EXPECT_FALSE(target_selection::targetRequiresFollow(std::nan("")));
}

TEST(ReferenceSpaceMppiLateralSampling,
     DerivesIntervalFromVehicleEnvelopeAndLeftWall) {
  const auto interval = lateral_sampling::feasibleSeparationInterval(
      1, 1.25, 2.95, 0.72, 0.72, 1.15, 0.15, 0.0, 2.20);
  ASSERT_TRUE(interval.feasible);
  EXPECT_NEAR(interval.minimum_separation_m, 1.59, 1.0e-12);
  EXPECT_NEAR(interval.maximum_separation_m, 1.70, 1.0e-12);
  EXPECT_NEAR(interval.nominal_separation_m, 1.70, 1.0e-12);
}

TEST(ReferenceSpaceMppiLateralSampling,
     PreservesFullOpenIntervalInsteadOfFixedPreferredOffset) {
  const auto interval = lateral_sampling::feasibleSeparationInterval(
      -1, 1.25, -2.50, 0.72, 0.72, 1.15, 0.15, 0.0, 2.20);
  ASSERT_TRUE(interval.feasible);
  EXPECT_NEAR(interval.minimum_separation_m, 1.59, 1.0e-12);
  EXPECT_NEAR(interval.maximum_separation_m, 3.75, 1.0e-12);
  EXPECT_NEAR(interval.nominal_separation_m, 2.20, 1.0e-12);
}

TEST(ReferenceSpaceMppiLateralSampling, RejectsWallGapNarrowerThanTwoKarts) {
  const auto interval = lateral_sampling::feasibleSeparationInterval(
      1, 1.25, 2.80, 0.72, 0.72, 1.15, 0.15, 0.0, 2.20);
  EXPECT_FALSE(interval.feasible);
  EXPECT_NEAR(interval.minimum_separation_m, 1.59, 1.0e-12);
  EXPECT_NEAR(interval.maximum_separation_m, 1.55, 1.0e-12);
}

TEST(ReferenceSpaceMppiLateralSampling,
     UsesPhysicalKartWidthWithoutAnImplicitComfortMargin) {
  const auto interval =
      lateral_sampling::feasibleSeparationIntervalFromAvailableWidth(
          2.20, 0.65, 0.65, 1.15, 0.0, 0.0, 2.20);
  ASSERT_TRUE(interval.feasible);
  EXPECT_NEAR(interval.minimum_separation_m, 1.30, 1.0e-12);
  EXPECT_NEAR(interval.maximum_separation_m, 2.20, 1.0e-12);
}

TEST(ReferenceSpaceMppiLateralLineLibrary,
     UsesAllFiveLinesAcrossTheOnlyFeasibleSide) {
  const auto library = lateral_line_library::make(
      lateral_line_library::SideInterval{1, false, 0.0, 0.0},
      lateral_line_library::SideInterval{-1, true, -2.4, -1.6}, 0);
  ASSERT_EQ(library.count, lateral_line_library::kLineCount);
  const std::array<double, 5U> expected{{-1.6, -1.8, -2.0, -2.2, -2.4}};
  for (std::size_t index = 0U; index < library.count; ++index) {
    EXPECT_EQ(library.lines[index].side, -1);
    EXPECT_NEAR(library.lines[index].d_m, expected[index], 1.0e-12);
  }
}

TEST(ReferenceSpaceMppiLateralLineLibrary,
     GivesPreferredSideTheAdditionalMidpoint) {
  const auto library = lateral_line_library::make(
      lateral_line_library::SideInterval{1, true, 1.6, 2.4},
      lateral_line_library::SideInterval{-1, true, -2.2, -1.6}, -1);
  ASSERT_EQ(library.count, lateral_line_library::kLineCount);
  EXPECT_EQ(library.lines[0].side, 1);
  EXPECT_NEAR(library.lines[0].d_m, 2.4, 1.0e-12);
  EXPECT_NEAR(library.lines[1].d_m, 1.6, 1.0e-12);
  EXPECT_EQ(library.lines[2].side, -1);
  EXPECT_NEAR(library.lines[2].d_m, -1.6, 1.0e-12);
  EXPECT_NEAR(library.lines[3].d_m, -1.9, 1.0e-12);
  EXPECT_NEAR(library.lines[4].d_m, -2.2, 1.0e-12);
}

TEST(ReferenceSpaceMppiLateralLineLibrary, RejectsMalformedIntervals) {
  const auto library = lateral_line_library::make(
      lateral_line_library::SideInterval{1, true, 2.0, 1.0},
      lateral_line_library::SideInterval{-1, false, 0.0, 0.0}, 0);
  EXPECT_EQ(library.count, 0U);
}

TEST(ReferenceSpaceMppiOpponentPrediction,
     PredictsLateralPositionAtAcceleratingEgoArrival) {
  const double arrival =
      opponent_prediction::boundedArrivalTime(9.0, 0.0, 6.0, 2.0, 3.0);
  EXPECT_NEAR(arrival, 3.0, 1.0e-12);
  EXPECT_NEAR(opponent_prediction::lateralPositionAtArrival(0.25, -0.40, 9.0,
                                                            0.0, 6.0, 2.0, 3.0),
              -0.95, 1.0e-12);
}

TEST(ReferenceSpaceMppiOpponentPrediction, CapsPredictionAtMppiHorizon) {
  EXPECT_NEAR(opponent_prediction::lateralPositionAtArrival(
                  1.0, -0.25, 100.0, 1.0, 10.0, 2.0, 3.0),
              0.25, 1.0e-12);
}

TEST(ReferenceSpaceMppiSideSelection, ChoosesLowestCostWithoutPreference) {
  const std::array<SideSelectionCandidate, 2U> candidates{{
      SideSelectionCandidate{1, true, 8.0},
      SideSelectionCandidate{-1, true, 4.0},
  }};
  const auto selected = selectSideCandidate(candidates, 2U, 0, 0.5);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 1U);
}

TEST(ReferenceSpaceMppiSideSelection, PassingLocationsRankCompletionThenBrakingThenAlongsideGap) {
  std::array<SideSelectionCandidate,3> options{{
      {1,true,1.,2.,9.,1.,.3}, {-1,true,2.,3.,8.,2.,.2}, {1,true,3.,4.,8.,1.,.5}}};
  EXPECT_EQ(selectSideCandidate(options,3,0,0.),2U);
  // The later first-pass time can still have the earliest completed merge.
  options[0].completion_time_sec=7.;EXPECT_EQ(selectSideCandidate(options,3,0,0.),0U);
  options[0].certified=false;
  options[1].braking_mps=1.;options[1].alongside_clearance_m=.8;
  EXPECT_EQ(selectSideCandidateRelative(options,3,1,.5),1U);
  options[1].certified=false;EXPECT_EQ(selectSideCandidate(options,3,0,0.),2U);
}

TEST(ReferenceSpaceMppiSideSelection, RelativeThresholdIsScaleInvariantAndHandlesSignedCosts) {
  std::array<SideSelectionCandidate, 3U> candidates{{
      {1, true, 19.25}, {-1, true, 11.57}, {-1, false, -1e100}}};
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.0), 1U);
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.5), 0U);
  for (auto &candidate : candidates) candidate.cost *= 1e-8;
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.0), 1U);
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.5), 0U);
  candidates[0].cost = candidates[1].cost = 0.0;
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, -1, 0.0), 1U);
  candidates[0].cost = -2.0;
  candidates[1].cost = -3.0;
  EXPECT_EQ(selectSideCandidateRelative(candidates, 3U, 1, 0.2), 1U);
  candidates[0].certified = candidates[1].certified = false;
  EXPECT_FALSE(selectSideCandidateRelative(candidates, 3U, 1, 0.0));
}

TEST(ReferenceSpaceMppiSideSelection, KeepsPreferredSideInsideHysteresis) {
  const std::array<SideSelectionCandidate, 2U> candidates{{
      SideSelectionCandidate{1, true, 8.0},
      SideSelectionCandidate{-1, true, 7.7},
  }};
  const auto selected = selectSideCandidate(candidates, 2U, 1, 0.5);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 0U);
}

TEST(ReferenceSpaceMppiSideSelection, SwitchesForMaterialCostImprovement) {
  const std::array<SideSelectionCandidate, 2U> candidates{{
      SideSelectionCandidate{1, true, 8.0},
      SideSelectionCandidate{-1, true, 7.4},
  }};
  const auto selected = selectSideCandidate(candidates, 2U, 1, 0.5);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 1U);
}

TEST(ReferenceSpaceMppiSideSelection, ImmediatelyLeavesInvalidPreferredSide) {
  const std::array<SideSelectionCandidate, 2U> candidates{{
      SideSelectionCandidate{1, false, 2.0},
      SideSelectionCandidate{-1, true, 9.0},
  }};
  const auto selected = selectSideCandidate(candidates, 2U, 1, 0.5);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 1U);
}

TEST(ReferenceSpaceMppiSideSelection, RejectsBatchWithoutCertifiedSide) {
  const std::array<SideSelectionCandidate, 2U> candidates{{
      SideSelectionCandidate{1, false, 2.0},
      SideSelectionCandidate{-1, false, 1.0},
  }};
  EXPECT_FALSE(selectSideCandidate(candidates, 2U, 1, 0.5).has_value());
}

TEST(ReferenceSpaceMppiSideSelection,
     ChoosesLowestCostTemplateWithinPreferredSide) {
  const std::array<SideSelectionCandidate, 5U> candidates{{
      SideSelectionCandidate{1, true, 8.0},
      SideSelectionCandidate{1, true, 5.0},
      SideSelectionCandidate{1, false, 1.0},
      SideSelectionCandidate{-1, true, 4.8},
      SideSelectionCandidate{-1, true, 9.0},
  }};
  const auto selected =
      selectSideCandidate(candidates, candidates.size(), 1, 0.5);
  ASSERT_TRUE(selected.has_value());
  // The 0.2 cost improvement on the other side is inside hysteresis, so the
  // cheaper of the two valid preferred-side templates must be retained.
  EXPECT_EQ(*selected, 1U);
}

Config enabledConfig() {
  Config config;
  config.enabled = true;
  config.shadow_only = true;
  config.allow_reference_switch = false;
  config.maximum_cross_track_error_m = 1.2;
  // Most legacy geometry tests intentionally isolate reference generation and
  // trackability from actuator dead time. Delay-specific behavior is covered
  // separately below with the production 0.20 s value.
  config.steering_control_delay_sec = 0.0;
  return config;
}

TEST(ReferenceSpaceMppiSteeringDelay, HoldsMeasuredSteeringForAwsimPureDelay) {
  SteeringCommandDelayLine<8U> delay_line;
  delay_line.reset(-0.05);

  const std::array<double, 5U> commands{{0.10, 0.20, 0.30, 0.40, 0.50}};
  for (std::size_t index = 0U; index < commands.size(); ++index) {
    const auto output = delay_line.pushAndSelect(commands[index], 0.20, 0.05);
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.delay_steps, 4U);
    if (index < 4U) {
      EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad, -0.05);
    } else {
      EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad, commands[0]);
    }
  }
}

TEST(ReferenceSpaceMppiSteeringDelay,
     ReplaysAlreadySentCommandsBeforeCandidateCommands) {
  const std::array<TimedSteeringCommand, 5U> history{{
      {9.80, 0.10},
      {9.85, 0.20},
      {9.90, 0.30},
      {9.95, 0.40},
      {10.00, 0.50},
  }};
  const auto pending = samplePendingSteeringTargets<5U, 8U>(
      history, history.size(), 10.00, 0.20, 0.05, -0.05);
  ASSERT_TRUE(pending.valid);
  ASSERT_EQ(pending.count, 4U);
  EXPECT_DOUBLE_EQ(pending.targets_rad[0], 0.10);
  EXPECT_DOUBLE_EQ(pending.targets_rad[1], 0.20);
  EXPECT_DOUBLE_EQ(pending.targets_rad[2], 0.30);
  EXPECT_DOUBLE_EQ(pending.targets_rad[3], 0.40);

  SteeringCommandDelayLine<8U> delay_line;
  delay_line.reset(-0.05, pending.targets_rad.data(), pending.count);
  for (std::size_t step = 0U; step < 5U; ++step) {
    const auto output =
        delay_line.pushAndSelect(0.60 + 0.10 * step, 0.20, 0.05);
    ASSERT_TRUE(output.valid);
    if (step < pending.count) {
      EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad,
                       pending.targets_rad[step]);
    } else {
      EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad, 0.60);
    }
  }
}

TEST(ReferenceSpaceMppiSteeringDelay,
     MissingOldHistoryFallsBackToMeasuredSteering) {
  const std::array<TimedSteeringCommand, 2U> history{{
      {9.90, 0.30},
      {9.95, 0.40},
  }};
  const auto pending = samplePendingSteeringTargets<2U, 8U>(
      history, history.size(), 10.00, 0.20, 0.05, -0.05);
  ASSERT_TRUE(pending.valid);
  ASSERT_EQ(pending.count, 4U);
  EXPECT_DOUBLE_EQ(pending.targets_rad[0], -0.05);
  EXPECT_DOUBLE_EQ(pending.targets_rad[1], -0.05);
  EXPECT_DOUBLE_EQ(pending.targets_rad[2], 0.30);
  EXPECT_DOUBLE_EQ(pending.targets_rad[3], 0.40);
}

TEST(ReferenceSpaceMppiSteeringDelay, ZeroDelayAppliesCurrentCommand) {
  SteeringCommandDelayLine<2U> delay_line;
  delay_line.reset(-0.05);
  const auto output = delay_line.pushAndSelect(0.10, 0.0, 0.05);
  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.delay_steps, 0U);
  EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad, 0.10);
}

TEST(ReferenceSpaceMppiSteeringDelay, RejectsInvalidTimingAndReusesDelayStorage) {
  SteeringCommandDelayLine<1U> delay_line;
  delay_line.reset(0.0);
  EXPECT_FALSE(delay_line.pushAndSelect(0.1, -0.1, 0.05).valid);
  EXPECT_FALSE(delay_line.pushAndSelect(0.1, 0.2, 0.0).valid);
  EXPECT_TRUE(delay_line.pushAndSelect(0.1, 0.2, 0.05).valid);
  const auto next=delay_line.pushAndSelect(0.2, 0.2, 0.05);
  ASSERT_TRUE(next.valid);
  EXPECT_DOUBLE_EQ(next.delayed_target_steering_rad,0.1);
}

TEST(ReferenceSpaceMppiSteeringDelay, LongRolloutKeepsDelayedCommandsAcrossWraps) {
  for(double delay : {0.,.10,.20}) {
    SteeringCommandDelayLine<4U> delay_line;
    delay_line.reset(-1.);
    const auto steps=static_cast<std::size_t>(std::round(delay/.05));
    for(std::size_t i=0;i<400;++i) {
      const auto output=delay_line.pushAndSelect(static_cast<double>(i),delay,.05);
      ASSERT_TRUE(output.valid);
      EXPECT_DOUBLE_EQ(output.delayed_target_steering_rad,i<steps ? -1. : static_cast<double>(i-steps));
    }
  }
}

PlanRequest straightRequest(bool obstacle = true) {
  PlanRequest request;
  request.valid = true;
  request.generation = 42U;
  request.stamp_sec = 10.0;
  request.side = 1;
  request.phase = Phase::PREPARE;
  request.ego.x_m = 0.0;
  request.ego.y_m = 0.0;
  request.ego.yaw_rad = 0.0;
  request.ego.speed_mps = 6.0;
  request.ego.s_m = 0.0;
  request.ego.d_m = 0.0;
  request.anchor_s_m = 1.2;
  request.anchor_d_m = 0.0;
  request.nominal = Parameters{1.55, 8.0, 10.0, 8.0, 0.90};
  request.bounds.minimum = Parameters{1.25, 5.0, 5.0, 5.0, 0.75};
  request.bounds.maximum = Parameters{1.90, 12.0, 18.0, 12.0, 1.0};
  request.base_reference_count = kMaximumReferencePoints;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    const double s = static_cast<double>(index) * 0.20;
    request.base_reference[index] =
        BaseReferencePoint{s, s, 0.0, 0.0, 0.0, 7.0, -2.5, 2.5};
  }
  if (obstacle) {
    request.static_obstacle_count = 1U;
    request.static_obstacles[0] = StaticObstacle{9.5, 10.5, -0.25, 0.25};
  }
  return request;
}

TEST(RealSteeringResponse, RolloutObeysMeasuredTireRateAcrossCommandReversal) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  config.lookahead_gain = 0.2;
  config.lookahead_min_distance_m = 2.0;
  config.steering_time_constant_sec = 0.02;
  config.horizon_steps = 10;
  auto request = straightRequest(false);
  request.ego.steering_rad = -0.28;
  TemporaryReference reference;
  reference.count = 101;
  request.base_reference_count = reference.count;
  for (std::size_t i = 0; i < reference.count; ++i) {
    const double s = 0.1 * i, yaw = s / 4.0;
    const double x = 4.0 * std::sin(yaw), y = 4.0 * (1.0 - std::cos(yaw));
    request.base_reference[i] = BaseReferencePoint{s, x, y, yaw, 0.25, 10.0, -10.0, 10.0};
    reference.points[i] = TemporaryReferencePoint{s, 0.0, x, y, yaw, 0.25, 10.0};
  }
  for (const bool awsim : {false, true}) {
    config.awsim_vehicle_response_enabled = awsim;
    for (const double rate : {0.39793506945470714, 0.6}) {
      config.physical_tire_steering_rate_radps = rate;
      const ReferenceSpaceMppiPlanner planner(config);
      const auto result = planner.evaluateExecutionReference(reference, request);
      ASSERT_TRUE(result.valid) << result.reject_stage;
      ASSERT_EQ(result.predicted_rollout_count, config.horizon_steps);
      double previous = request.ego.steering_rad;
      double previous_yaw = request.ego.yaw_rad;
      double previous_speed = request.ego.speed_mps;
      for (std::size_t i = 0; i < result.predicted_rollout_count; ++i) {
        const auto &state = result.predicted_rollout[i];
        if (!awsim) {
          // Recover the physical tire angle from the published bicycle-model
          // pose change, so this checks the full rollout rather than a helper.
          const double yaw_delta = std::atan2(std::sin(state.yaw_rad - previous_yaw),
                                              std::cos(state.yaw_rad - previous_yaw));
          const double tire = std::atan(yaw_delta * config.wheel_base_m / (config.dt_sec * previous_speed));
          EXPECT_LE(std::abs(tire - previous), rate * config.dt_sec + 1e-12);
          previous = tire;
        }
        previous_yaw = state.yaw_rad;
        previous_speed = state.speed_mps;
      }
    }
    config.physical_tire_steering_rate_radps = 0.0;
    const auto legacy = ReferenceSpaceMppiPlanner(config).evaluateExecutionReference(reference, request);
    config.physical_tire_steering_rate_radps = 2.4;
    const auto explicit_legacy = ReferenceSpaceMppiPlanner(config).evaluateExecutionReference(reference, request);
    ASSERT_EQ(legacy.predicted_rollout_count, explicit_legacy.predicted_rollout_count);
    for (std::size_t i = 0; i < legacy.predicted_rollout_count; ++i) {
      EXPECT_DOUBLE_EQ(legacy.predicted_rollout[i].yaw_rad, explicit_legacy.predicted_rollout[i].yaw_rad);
      EXPECT_DOUBLE_EQ(legacy.predicted_rollout[i].x_m, explicit_legacy.predicted_rollout[i].x_m);
    }
  }
}

TEST(SteeringDemandAcceleration, GentleAllowanceChangesCornerRolloutButNotBraking) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  config.lookahead_gain = 0.2;
  config.lookahead_min_distance_m = 2.0;
  config.horizon_steps = 10;
  auto request = straightRequest(false);
  request.ego.steering_rad = 0.28;
  request.ego.yaw_rate_radps = 1.5;
  request.base_reference_count = 101;
  TemporaryReference reference;
  reference.count = request.base_reference_count;
  constexpr double radius = 4.0;
  for (std::size_t i = 0; i < reference.count; ++i) {
    const double s = 0.1 * i;
    const double yaw = s / radius;
    const double x = radius * std::sin(yaw);
    const double y = radius * (1.0 - std::cos(yaw));
    request.base_reference[i] =
        BaseReferencePoint{s, x, y, yaw, 1.0 / radius, 10.0, -10.0, 10.0};
    reference.points[i] =
        TemporaryReferencePoint{s, 0.0, x, y, yaw, 1.0 / radius, 10.0};
  }
  for (const bool awsim_response : {false, true}) {
    config.awsim_vehicle_response_enabled = awsim_response;
    config.steering_acceleration_hold_maximum_acceleration_mps2 = 0.0;
    const ReferenceSpaceMppiPlanner coast(config);
    config.steering_acceleration_hold_maximum_acceleration_mps2 = 0.6;
    const ReferenceSpaceMppiPlanner gentle(config);
    auto accelerating = reference;
    const auto baseline = coast.evaluateExecutionReference(accelerating, request);
    const auto changed = gentle.evaluateExecutionReference(accelerating, request);
    ASSERT_TRUE(baseline.valid) << baseline.reject_stage;
    ASSERT_TRUE(changed.valid) << changed.reject_stage;
    ASSERT_EQ(baseline.predicted_rollout_count, config.horizon_steps);
    ASSERT_EQ(changed.predicted_rollout_count, config.horizon_steps);
    EXPECT_GT(changed.predicted_rollout[config.horizon_steps - 1].speed_mps,
              baseline.predicted_rollout[config.horizon_steps - 1].speed_mps + 0.05);

    auto braking = reference;
    for (std::size_t i = 0; i < braking.count; ++i) braking.points[i].speed_mps = 4.0;
    const auto original_braking = coast.evaluateExecutionReference(braking, request);
    const auto gentle_braking = gentle.evaluateExecutionReference(braking, request);
    ASSERT_TRUE(original_braking.valid);
    ASSERT_TRUE(gentle_braking.valid);
    ASSERT_EQ(original_braking.predicted_rollout_count, gentle_braking.predicted_rollout_count);
    for (std::size_t i = 0; i < original_braking.predicted_rollout_count; ++i) {
      EXPECT_DOUBLE_EQ(original_braking.predicted_rollout[i].speed_mps,
                       gentle_braking.predicted_rollout[i].speed_mps);
      EXPECT_DOUBLE_EQ(original_braking.predicted_rollout[i].x_m,
                       gentle_braking.predicted_rollout[i].x_m);
    }
  }
}

TEST(ContinuousPrefix, GeneratedStagedCandidateRetainsMeasuredStartAndSpeed) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  ReferenceSpaceMppiPlanner planner(config);
  auto request = straightRequest(false);
  request.sample_staged_speed = true;
  request.ego.d_m = -.520884937239;
  request.ego.y_m = request.ego.d_m;
  request.anchor_d_m = request.ego.d_m;
  request.anchor_s_m = .5;
  for (std::size_t i = 0; i < request.base_reference_count; ++i) {
    auto &p = request.base_reference[i];
    p.s_m = i == 0 ? 0 : .0886331589331 + (i-1)*.2;
    p.x_m = p.s_m;
    p.active_d_m = -.00436966039062;
    p.active_d_valid = true;
    p.minimum_d_m = -.1;
  }
  connectMeasuredPrefix(request, 4);
  ReferenceControlSequence control;
  control.count = 8;
  TemporaryReference reference;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(
      planner, request.nominal, request, &control, &reference));
  EXPECT_DOUBLE_EQ(reference.points[0].y_m, request.ego.y_m);
  EXPECT_LT(std::abs(reference.points[0].curvature_1pm), .25);
  EXPECT_NEAR(reference.points[0].speed_mps,
              reference.points[0].uncapped_speed_mps, 1e-9);
}

TEST(CmaPreview, CachedWindowMatchesCmaStencilAndDistanceLoop) {
  TemporaryReference path;
  path.count=80;
  for(std::size_t i=0;i<path.count;++i) {
    path.points[i].x_m=.001*i*i;
    path.points[i].y_m=std::sin(.13*i);
  }
  for(double window : {0., .7, 21.422826964417297}) {
    const auto actual=cmaPreviewCurvatures(path,window);
    const auto local=[&](std::size_t i) {
      const std::size_t first=i>1?i-1:0, last=std::min(path.count-1,i+4);
      auto a=path.points[first], b=path.points[(first+last)/2], c=path.points[last];
      double d=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m)*std::hypot(c.x_m-b.x_m,c.y_m-b.y_m)*std::hypot(a.x_m-c.x_m,a.y_m-c.y_m);
      return d<=1e-6?0:2*std::abs((b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m))/d;
    };
    for(std::size_t i=0;i<path.count;++i) {
      double expected=local(i),distance=0;
      for(std::size_t j=i+1;j<path.count;++j) {
        distance+=std::hypot(path.points[j].x_m-path.points[j-1].x_m,path.points[j].y_m-path.points[j-1].y_m);
        if(distance>window) break;
        expected=std::max(expected,local(j));
      }
      EXPECT_NEAR(actual[i],expected,1e-12);
    }
  }
}

TEST(StagedSpeed, PairedDiagnosticsPreserveGeometrySelectionAndWarmState) {
  auto config = enabledConfig();
  config.sample_count = 4;
  config.minimum_valid_count = 1;
  config.collision_only_rejection = true;
  config.control_maximum_speed_scale_adjustment = 1;
  ReferenceSpaceMppiPlanner normal(config), diagnostic(config);
  auto request = straightRequest(false);
  request.sample_control_sequence = request.sample_staged_speed = true;
  request.minimum_speed_mps = 1;
  request.nominal = Parameters{0, 4, 5, 5, 1};
  request.bounds.minimum = Parameters{0, 4, 5, 5, .1};
  request.bounds.maximum = Parameters{0, 4, 5, 5, 1};
  Scratch a, b;
  for (int cycle = 0; cycle < 3; ++cycle) {
    request.speed_pair_diagnostics = false;
    const auto expected = normal.plan(request, &a);
    request.speed_pair_diagnostics = true;
    const auto actual = diagnostic.plan(request, &b);
    ASSERT_TRUE(expected.valid);
    ASSERT_TRUE(actual.valid);
    EXPECT_GT(actual.valid_sample_count, config.sample_count);
    ASSERT_EQ(actual.speed_pair_count, 4U);
    EXPECT_EQ(expected.speed_pair_count, 0U);
    for (const auto &pair : expected.speed_pairs) EXPECT_FALSE(pair.reference);
    EXPECT_DOUBLE_EQ(actual.selected_evaluation.cost, expected.selected_evaluation.cost);
    EXPECT_DOUBLE_EQ(actual.updated_mean_cost, expected.updated_mean_cost);
    for (const auto &pair : actual.speed_pairs) {
      ASSERT_TRUE(pair.reference);
      ASSERT_GT(pair.reference->count, 0U);
      EXPECT_DOUBLE_EQ(pair.reference->points[0].speed_mps, pair.first_speed);
      EXPECT_DOUBLE_EQ(pair.geometry_delta_m, 0);
      EXPECT_EQ(pair.evaluation.valid, pair.valid);
      EXPECT_DOUBLE_EQ(pair.evaluation.cost, pair.cost);
      EXPECT_EQ(pair.evaluation.cost_terms, pair.terms);
      if (pair.valid) {
        EXPECT_TRUE(std::isfinite(pair.evaluation.terminal_reference_d_m));
        ASSERT_GT(pair.evaluation.predicted_rollout_count, 0U);
        for (std::size_t i = 1; i < pair.evaluation.predicted_rollout_count; ++i)
          EXPECT_GT(pair.evaluation.predicted_rollout[i].time_sec,
                    pair.evaluation.predicted_rollout[i-1].time_sec);
      }
    }
    ASSERT_EQ(actual.selected_reference.count, expected.selected_reference.count);
    for (std::size_t i = 0; i < actual.selected_reference.count; ++i) {
      EXPECT_DOUBLE_EQ(actual.selected_reference.points[i].x_m, expected.selected_reference.points[i].x_m);
      EXPECT_DOUBLE_EQ(actual.selected_reference.points[i].speed_mps, expected.selected_reference.points[i].speed_mps);
    }
    request.stamp_sec += .05;
  }
}
TEST(StagedSpeed, FastEntryExploresWallSideIndependentlyOfWarmLateralOffset) {
  for (int side : {-1,1}) {
    auto config=enabledConfig(); config.sample_count=4;
    config.minimum_valid_count=1; config.control_maximum_speed_scale_adjustment=1.;
    auto request=straightRequest(false);
    request.side=side;
    request.sample_control_sequence=request.sample_staged_speed=true;
    request.sample_lateral_bounds=true;
    request.nominal.d_pass_m=side*.5;
    request.bounds.minimum.d_pass_m=side>0 ? .2 : -1.9;
    request.bounds.maximum.d_pass_m=side>0 ? 1.9 : -.2;
    Scratch scratch;
    ReferenceSpaceMppiPlanner planner(config);
    planner.plan(request,&scratch);
    const auto sample=config.sample_count+1U;
    EXPECT_NEAR(request.nominal.d_pass_m+scratch.noise[sample][0],side*1.9,1e-9);
    EXPECT_NEAR(request.nominal.l_out_m+scratch.noise[sample][1],request.bounds.minimum.l_out_m,1e-9);
  }
}

TEST(StagedSpeed, EntrySpeedGridDistinguishesEarlyAndLateAcceleration) {
  auto config = enabledConfig();
  config.control_maximum_speed_scale_adjustment = 1;
  ReferenceSpaceMppiPlanner planner(config);
  auto request = straightRequest(false);
  request.sample_staged_speed = true;
  request.preferred_matching_speed_mps = .8;
  request.minimum_speed_mps = 0;
  request.horizon_steps_override = 60;
  request.base_reference_count = 101;
  request.anchor_s_m = .5;
  request.bounds.minimum.speed_scale = .08;
  request.bounds.maximum.speed_scale = 1;
  auto parameters = request.nominal;
  parameters.speed_scale = 1;
  parameters.l_out_m = 4;
  ReferenceControlSequence control;
  control.count = 8;
  const auto early = PlannerEvaluationTestAccess::staged(planner, request, parameters, control, 3);
  const auto late = PlannerEvaluationTestAccess::staged(planner, request, parameters, control, 4);
  EXPECT_GT(early.speed_scale_adjustment[2], late.speed_scale_adjustment[2]);
  EXPECT_EQ(early.lateral_adjustment_m, late.lateral_adjustment_m);
  EXPECT_DOUBLE_EQ(early.speed_scale_adjustment[0],
      .8 / request.base_reference[0].speed_mps - parameters.speed_scale);
}

TEST(StagedSpeed, AbsoluteSeedsSurviveVaryingBaseSpeedAndGeneratedCurvature) {
  auto config=enabledConfig();
  config.collision_only_rejection=true;
  config.control_maximum_speed_scale_adjustment=1.;
  ReferenceSpaceMppiPlanner planner(config);
  auto request=straightRequest(false);
  request.sample_staged_speed=true;
  request.minimum_speed_mps=0.;
  request.preferred_matching_speed_mps=3.;
  request.ego.speed_mps=4.5;
  request.bounds.minimum.speed_scale=0.;
  request.bounds.maximum.speed_scale=1.;
  auto parameters=request.nominal;
  parameters.speed_scale=1.;
  for(std::size_t i=0;i<request.base_reference_count;++i)
    request.base_reference[i].speed_mps=i%2 ? 10. : 2.;
  ReferenceControlSequence control;control.count=8;
  for(const auto sample : {1U,2U,5U,6U,7U,8U,9U,10U}) {
    const double expected=sample==1 ? 10. : sample==2 ? 3. : sample==10 ? 4.5 : 2.*(sample-5);
    const auto seeded=PlannerEvaluationTestAccess::staged(planner,request,parameters,control,sample);
    TemporaryReference reference;
    ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner,parameters,request,&seeded,&reference));
    for(std::size_t i=0;i<reference.count;++i) {
      EXPECT_NEAR(reference.points[i].speed_mps,expected,1e-12);
      EXPECT_DOUBLE_EQ(reference.points[i].speed_mps,reference.points[i].uncapped_speed_mps);
    }
  }
  // The legacy path still follows the local base speed and curvature cap.
  request.sample_staged_speed=false;
  TemporaryReference legacy;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner,parameters,request,nullptr,&legacy));
  EXPECT_LE(legacy.points[0].speed_mps,2.);
}

TEST(StagedSpeed, AcceptedAbsoluteSpeedUsesSameBasisAsInitialCandidates) {
  auto config=enabledConfig();config.control_maximum_speed_scale_adjustment=1.;
  ReferenceSpaceMppiPlanner planner(config);
  auto request=straightRequest(false);
  request.sample_control_sequence=request.sample_staged_speed=true;
  request.bounds.minimum.speed_scale=0.;request.bounds.maximum.speed_scale=1.;
  request.nominal.speed_scale=1.;request.minimum_speed_mps=0.;
  for(std::size_t i=0;i<request.base_reference_count;++i) {
    auto &p=request.base_reference[i];
    p.speed_mps=i%2 ? 10. : 2.;p.active_speed_valid=true;p.active_speed_mps=4.;
  }
  const auto warm=PlannerEvaluationTestAccess::warm(planner,request);
  TemporaryReference reference;
  ASSERT_TRUE(PlannerEvaluationTestAccess::generate(planner,request.nominal,request,&warm,&reference));
  for(std::size_t i=0;i<reference.count;++i)EXPECT_NEAR(reference.points[i].speed_mps,4.,1e-12);
}

TEST(StagedSpeed, AcceptedAccelerationMovesTowardEgoOnQuadraticSpeedGrid) {
  auto config = enabledConfig();
  config.control_maximum_speed_scale_adjustment = 1;
  ReferenceSpaceMppiPlanner planner(config);
  auto request = straightRequest(false);
  request.sample_control_sequence = request.sample_staged_speed = true;
  request.base_reference_count = 101;
  request.minimum_speed_mps = .8;
  request.horizon_steps_override = 60;
  request.anchor_s_m = .5;
  request.nominal.speed_scale = 1;
  request.bounds.minimum.speed_scale = .08;
  request.bounds.maximum.speed_scale = 1;
  ReferenceControlSequence control;
  control.count = config.control_knot_count;
  control = PlannerEvaluationTestAccess::staged(planner, request, request.nominal, control, 3);
  PlannerEvaluationTestAccess::setWarm(planner, request, control);
  const auto before = PlannerEvaluationTestAccess::warm(planner, request);
  request.stamp_sec += .5;
  request.ego.speed_mps = 2;
  // One metre of MEASURED progress, not a velocity*wall-time extrapolation.
  request.ego.x_m += 1;
  for (std::size_t i=0;i<request.base_reference_count;++i) request.base_reference[i].x_m += 1;
  const auto advanced = PlannerEvaluationTestAccess::warm(planner, request);
  EXPECT_GT(advanced.speed_scale_adjustment[0], before.speed_scale_adjustment[0]);
  EXPECT_EQ(advanced.lateral_adjustment_m, before.lateral_adjustment_m);
  request.semantic_key += 1;
  const auto reset = PlannerEvaluationTestAccess::warm(planner, request);
  EXPECT_DOUBLE_EQ(reset.speed_scale_adjustment[0], 0);
}

TEST(StagedSpeed, CurvatureCostIsIndependentOfSpeedOnSameGeometry) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  ReferenceSpaceMppiPlanner planner(config);
  auto request = straightRequest(false);
  request.sample_staged_speed = true;
  request.complete_maneuver = false;
  request.horizon_steps_override = 60;
  auto fast = request.nominal, slow = request.nominal;
  slow.speed_scale = request.bounds.minimum.speed_scale;
  fast.speed_scale = request.bounds.maximum.speed_scale;
  TemporaryReference a, b;
  const auto ea = PlannerEvaluationTestAccess::evaluate(planner, slow, request, nullptr, &a);
  const auto eb = PlannerEvaluationTestAccess::evaluate(planner, fast, request, nullptr, &b);
  ASSERT_TRUE(ea.valid);
  ASSERT_TRUE(eb.valid);
  EXPECT_GT(ea.cost_terms[3], 0);
  EXPECT_DOUBLE_EQ(ea.cost_terms[3], eb.cost_terms[3]);
  EXPECT_GT(eb.progress_m, ea.progress_m);
}

TEST(StagedSpeed, TrackingCostChargesExcessAndPreservesHardValidator) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  ReferenceSpaceMppiPlanner planner(config);
  auto request = straightRequest(false);
  request.sample_staged_speed = true;
  request.horizon_steps_override = 2;
  request.nominal.d_pass_m = 0;
  request.ego.y_m = .3;
  TemporaryReference reference;
  const auto within = PlannerEvaluationTestAccess::evaluate(planner, request.nominal, request, nullptr, &reference);
  ASSERT_TRUE(within.valid);
  EXPECT_GT(within.maximum_cross_track_error_m, 0);
  EXPECT_DOUBLE_EQ(within.cost_terms[4], 0);
  request.sample_staged_speed = false;
  const auto legacy = PlannerEvaluationTestAccess::evaluate(planner, request.nominal, request, nullptr, &reference);
  EXPECT_GT(legacy.cost_terms[4], 0);
  request.sample_staged_speed = true;
  request.ego.y_m = 2;
  const auto excess = PlannerEvaluationTestAccess::evaluate(planner, request.nominal, request, nullptr, &reference);
  ASSERT_TRUE(excess.valid);
  EXPECT_GT(excess.cost_terms[4], 0);
  request.rollout_constraint_validator = [](const RolloutState &, const RolloutState &) { return RejectReason::WALL; };
  const auto wall = PlannerEvaluationTestAccess::evaluate(planner, request.nominal, request, nullptr, &reference);
  EXPECT_FALSE(wall.valid);
  EXPECT_EQ(wall.reject_reason, RejectReason::WALL);
}

TEST(StagedSpeed, SamplesFastMatchingAndAccelerationWithoutRelaxingCollision) {
  auto config = enabledConfig();
  config.horizon_steps = 60;
  config.sample_count = 4;
  config.minimum_valid_count = 1;
  config.control_maximum_speed_scale_adjustment = 1.0;
  config.control_lateral_std_m = 0.0;
  auto request = straightRequest(false);
  request.complete_maneuver = false;
  request.anchor_s_m = 0.5;
  request.ego.speed_mps = 1.0;
  request.base_reference_count = 101;
  request.sample_control_sequence = true;
  request.sample_staged_speed = true;
  request.visualized_sample_count = 4;
  request.preferred_matching_speed_mps = 1.0;
  request.minimum_speed_mps = 0.0;
  request.nominal = Parameters{0.0, 4.0, 5.0, 5.0, 1.0};
  request.bounds.minimum = Parameters{0.0, 4.0, 5.0, 5.0, 0.1};
  request.bounds.maximum = Parameters{0.0, 4.0, 5.0, 5.0, 1.0};
  for (auto &point : request.base_reference) point.speed_mps = 10.0;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid) << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  ASSERT_EQ(result.visualized_sample_count, 3U);
  // Fast sample 1 duplicates warm sample 0. Later unique samples must remain
  // visible, packed without empty slots and retaining their raw sample IDs.
  EXPECT_GT(result.duplicate_sample_count, 0U);
  EXPECT_EQ(result.visualized_sample_indices[0], 0U);
  EXPECT_EQ(result.visualized_sample_indices[1], 2U);
  EXPECT_EQ(result.visualized_sample_indices[2], 3U);
  const auto &fast = result.visualized_sample_references[0];
  const auto &matching = result.visualized_sample_references[1];
  const auto &accelerating = result.visualized_sample_references[2];
  ASSERT_GT(fast.count, 1U);
  ASSERT_GT(matching.count, 1U);
  ASSERT_GT(accelerating.count, 1U);
  EXPECT_NEAR(fast.points.front().speed_mps, 10.0, 1e-6);
  EXPECT_NEAR(matching.points.front().speed_mps, 1.0, 1e-6);
  EXPECT_NEAR(matching.points[matching.count - 1U].speed_mps, 1.0, 1e-6);
  EXPECT_NEAR(accelerating.points.front().speed_mps, 1.0, 1e-6);
  EXPECT_NEAR(accelerating.points[accelerating.count - 1U].speed_mps, 10.0, 1e-6);
  EXPECT_GT(accelerating.points[10].speed_mps, 1.1);
  EXPECT_NEAR(scratch.speed_control_noise[2][0], -0.9, 1e-6);
  EXPECT_NEAR(scratch.speed_control_noise[3][0], -0.9, 1e-6);
  EXPECT_NEAR(scratch.speed_control_noise[3][config.control_knot_count - 1U], 0.0, 1e-6);
  for (std::size_t i = 0; i < accelerating.count; ++i)
    EXPECT_GE(accelerating.points[i].speed_mps, 1.0 - 1e-6);
  request.rollout_constraint_validator = [](const RolloutState &, const RolloutState &state) {
    return state.x_m > 4.0 ? RejectReason::COLLISION : RejectReason::NONE;
  };
  const auto matching_survives = planner.plan(request, &scratch);
  ASSERT_TRUE(matching_survives.valid);
  EXPECT_GT(matching_survives.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)], 0U);
  EXPECT_LE(matching_survives.selected_evaluation.predicted_rollout[
      matching_survives.selected_evaluation.predicted_rollout_count - 1U].x_m, 4.0);
  request.rollout_constraint_validator = {};
  ReferenceControlSequence first_control, second_control;
  first_control.count = second_control.count = config.control_knot_count;
  second_control.speed_scale_adjustment.fill(-0.5);
  auto first_parameters = request.nominal;
  auto second_parameters = request.nominal;
  first_parameters.speed_scale = 0.5;
  second_parameters.speed_scale = 1.0;
  TemporaryReference first_reference, second_reference;
  const auto first_evaluation = PlannerEvaluationTestAccess::evaluate(planner, first_parameters, request, &first_control, &first_reference);
  const auto second_evaluation = PlannerEvaluationTestAccess::evaluate(planner, second_parameters, request, &second_control, &second_reference);
  ASSERT_TRUE(first_evaluation.valid);
  ASSERT_TRUE(second_evaluation.valid);
  EXPECT_NEAR(first_evaluation.cost, second_evaluation.cost, 1e-9);
  double total_terms = 0.0;
  for (double term : first_evaluation.cost_terms) total_terms += term;
  EXPECT_NEAR(total_terms, first_evaluation.cost, 1e-9);
  request.rollout_constraint_validator = [](const RolloutState &, const RolloutState &) {
    return RejectReason::COLLISION;
  };
  const auto rejected = planner.plan(request, &scratch);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(rejected.valid_sample_count, 0U);
}

TEST(StagedSpeed, TerminalObjectiveUsesActualRolloutAndLeavesLegacyUnchanged) {
  auto config = enabledConfig();
  config.horizon_steps = 60;
  auto weighted_config = config;
  weighted_config.cost_terminal_lateral_weight = 2.0;
  ReferenceSpaceMppiPlanner plain(config), weighted(weighted_config);
  auto request = straightRequest(false);
  request.sample_staged_speed = true;
  request.complete_maneuver = false;
  request.ego.speed_mps = 1.0;
  request.base_reference_count = 101;
  request.minimum_speed_mps = 1.0;
  request.nominal.speed_scale = 0.1;
  request.bounds.minimum.speed_scale = 0.1;
  TemporaryReference first, second;
  auto a = PlannerEvaluationTestAccess::evaluate(plain, request.nominal, request, nullptr, &first);
  auto b = PlannerEvaluationTestAccess::evaluate(weighted, request.nominal, request, nullptr, &second);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  const auto &terminal = b.predicted_rollout[b.predicted_rollout_count - 1U];
  double error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < second.count; ++i) {
    const auto &p = second.points[i - 1];
    const auto &q = second.points[i];
    const double dx = q.x_m - p.x_m, dy = q.y_m - p.y_m;
    const double length2 = dx * dx + dy * dy;
    const double ratio = length2 > 1e-12 ? std::clamp(
        ((terminal.x_m - p.x_m) * dx + (terminal.y_m - p.y_m) * dy) / length2,
        0.0, 1.0) : 0.0;
    error = std::min(error, std::hypot(terminal.x_m - p.x_m - ratio * dx,
                                      terminal.y_m - p.y_m - ratio * dy));
  }
  const double residual = std::max(0.0, error - config.maximum_cross_track_error_m) /
      config.maximum_cross_track_error_m;
  EXPECT_NEAR(b.cost - a.cost, 2.0 * residual * residual, 1e-9);
  EXPECT_NEAR(b.cost_terms[7], 2.0 * residual * residual, 1e-9);
  EXPECT_GE(b.cost - a.cost, 0.0);
  request.sample_staged_speed = false;
  a = PlannerEvaluationTestAccess::evaluate(plain, request.nominal, request, nullptr, &first);
  b = PlannerEvaluationTestAccess::evaluate(weighted, request.nominal, request, nullptr, &second);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  EXPECT_DOUBLE_EQ(a.cost, b.cost);
}

TEST(LocalHorizon, FarCollisionDoesNotRejectLocalPrefixButNearCollisionDoes) {
  auto config = enabledConfig();
  auto request = straightRequest(false);
  request.nominal_only = true;
  request.static_obstacle_count = 1;
  request.static_obstacles[0] = StaticObstacle{35, 36, -5, 5};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  EXPECT_FALSE(planner.plan(request, &scratch).valid);
  request.base_reference_count = 101U;
  request.horizon_steps_override = 20U;
  const auto local = planner.plan(request, &scratch);
  ASSERT_TRUE(local.valid);
  EXPECT_DOUBLE_EQ(local.selected_reference.points[100].s_m, 20);
  request.static_obstacles[0] = StaticObstacle{10, 11, -5, 5};
  EXPECT_FALSE(planner.plan(request, &scratch).valid);
}

TEST(LocalHorizon, PartialMergeDoesNotRequireCompletionWithinWindow) {
  auto config = enabledConfig();
  auto request = straightRequest(false);
  request.nominal_only = true;
  request.phase = Phase::MERGE;
  request.anchor_d_m = 1.0;
  request.ego.y_m = request.ego.d_m = 1.0;
  request.nominal.l_merge_m = 30.0;
  request.base_reference_count = 101;
  request.horizon_steps_override = 20;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.selected_reference.points[100].d_m, 0.0);
  EXPECT_LT(result.selected_reference.points[100].d_m, 1.0);
}

TEST(LocalHorizon, TimeWindowIsIndependentOfCandidateSpeedAndBounded) {
  auto config = enabledConfig();
  auto request = straightRequest(false);
  request.nominal_only = true;
  request.horizon_steps_override = 20U;
  double end_time = 0;
  request.rollout_constraint_validator = [&](const RolloutState &, const RolloutState &end) {
    end_time = std::max(end_time, end.time_sec);
    return RejectReason::NONE;
  };
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  ASSERT_TRUE(planner.plan(request, &scratch).valid);
  EXPECT_NEAR(end_time, 1.0, 1e-9);
  end_time = 0;
  request.nominal.speed_scale = .75;
  ASSERT_TRUE(planner.plan(request, &scratch).valid);
  EXPECT_NEAR(end_time, 1.0, 1e-9);
  request.horizon_steps_override = config.horizon_steps + 1;
  EXPECT_EQ(planner.plan(request, &scratch).reject_reason, RejectReason::INVALID_INPUT);
}

TEST(SpatialNoise, ReducesLocalLateralExplorationWithoutChangingSpeedSamples) {
  auto old_config = enabledConfig();
  auto new_config = old_config;
  new_config.control_lateral_reference_spacing_m = 8;
  auto request = straightRequest(false);
  request.base_reference_count = 101;
  request.sample_control_sequence = true;
  Scratch old_scratch, new_scratch;
  ReferenceSpaceMppiPlanner old_planner(old_config), new_planner(new_config);
  old_planner.plan(request, &old_scratch);
  new_planner.plan(request, &new_scratch);
  double old_energy = 0, new_energy = 0;
  for (std::size_t sample = 0; sample < old_config.sample_count; ++sample) {
    for (std::size_t knot = 0; knot < old_config.control_knot_count; ++knot) {
      EXPECT_DOUBLE_EQ(old_scratch.speed_control_noise[sample][knot],
                       new_scratch.speed_control_noise[sample][knot]);
      old_energy += std::pow(old_scratch.lateral_control_noise[sample][knot], 2);
      new_energy += std::pow(new_scratch.lateral_control_noise[sample][knot], 2);
    }
  }
  EXPECT_GT(old_energy, 0);
  EXPECT_LT(new_energy, old_energy * .1);
}

std::size_t totalRejectedSamples(const PlanResult &result) {
  std::size_t count = 0U;
  for (const std::size_t reason_count : result.reject_counts) {
    count += reason_count;
  }
  return count;
}

void expectSameSelectedParameters(const PlanResult &first,
                                  const PlanResult &second) {
  EXPECT_EQ(first.valid, second.valid);
  EXPECT_EQ(first.reject_reason, second.reject_reason);
  EXPECT_DOUBLE_EQ(first.selected.d_pass_m, second.selected.d_pass_m);
  EXPECT_DOUBLE_EQ(first.selected.l_out_m, second.selected.l_out_m);
  EXPECT_DOUBLE_EQ(first.selected.l_hold_m, second.selected.l_hold_m);
  EXPECT_DOUBLE_EQ(first.selected.l_merge_m, second.selected.l_merge_m);
  EXPECT_DOUBLE_EQ(first.selected.speed_scale, second.selected.speed_scale);
  EXPECT_DOUBLE_EQ(first.selected.lateral_control_near_scale,
                   second.selected.lateral_control_near_scale);
  EXPECT_DOUBLE_EQ(first.selected.lateral_control_far_scale,
                   second.selected.lateral_control_far_scale);
  EXPECT_DOUBLE_EQ(first.selected_evaluation.cost,
                   second.selected_evaluation.cost);
}

TEST(ReferenceSpaceMppiBatchOptimizer,
     MatchesIndependentPlannersOnTheFirstCycle) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto first_request = straightRequest(false);
  first_request.sample_count_override = 6U;
  auto second_request = first_request;
  second_request.generation = 43U;
  second_request.nominal.d_pass_m = 1.75;

  ReferenceSpaceMppiPlanner first_scalar(config);
  ReferenceSpaceMppiPlanner second_scalar(config);
  Scratch first_scalar_scratch;
  Scratch second_scalar_scratch;
  const auto expected_first =
      first_scalar.plan(first_request, &first_scalar_scratch);
  const auto expected_second =
      second_scalar.plan(second_request, &second_scalar_scratch);

  ReferenceSpaceMppiBatchOptimizer batch(config, false);
  BatchScratch batch_scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount> requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> slots{};
  requests[0U] = &first_request;
  requests[1U] = &second_request;
  slots[0U] = 0U;
  slots[1U] = 3U;

  const auto result = batch.plan(requests, slots, 2U, &batch_scratch);

  ASSERT_EQ(result.candidate_count, 2U);
  expectSameSelectedParameters(expected_first, result.candidates[0U]);
  expectSameSelectedParameters(expected_second, result.candidates[1U]);
  EXPECT_GE(result.elapsed_ms, 0.0);
  EXPECT_GE(result.candidate_elapsed_sum_ms, 0.0);
}

TEST(ReferenceSpaceMppiBatchOptimizer,
     ExecutesEnvironmentValidatorsConcurrently) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto first_request = straightRequest(false);
  first_request.nominal_only = true;
  auto second_request = first_request;
  second_request.generation = 43U;
  std::atomic<int> active{0};
  std::atomic<int> maximum_active{0};
  const auto validator = [&active,
                          &maximum_active](const TemporaryReference &) {
    const int current = active.fetch_add(1) + 1;
    int observed = maximum_active.load();
    while (current > observed &&
           !maximum_active.compare_exchange_weak(observed, current)) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    active.fetch_sub(1);
    return RejectReason::NONE;
  };
  first_request.path_constraint_validator = validator;
  second_request.path_constraint_validator = validator;

  ReferenceSpaceMppiBatchOptimizer batch(config, true);
  BatchScratch scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount> requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> slots{};
  requests[0U] = &first_request;
  requests[1U] = &second_request;
  slots[0U] = 0U;
  slots[1U] = 1U;

  const auto result = batch.plan(requests, slots, 2U, &scratch);

  ASSERT_TRUE(result.candidates[0U].valid);
  ASSERT_TRUE(result.candidates[1U].valid);
  EXPECT_GE(maximum_active.load(), 2);
}

TEST(ReferenceSpaceMppiBatchOptimizer,
     KeepsWarmStartStateIndependentBetweenSlots) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto primary_first = straightRequest(false);
  primary_first.sample_count_override = 6U;
  auto secondary = primary_first;
  secondary.generation = 100U;
  secondary.nominal.l_out_m = 11.0;
  auto primary_second = primary_first;
  primary_second.generation = 44U;

  ReferenceSpaceMppiBatchOptimizer isolated(config, false);
  BatchScratch isolated_scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount>
      isolated_requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> isolated_slots{};
  isolated_requests[0U] = &primary_first;
  isolated_slots[0U] = 2U;
  isolated.plan(isolated_requests, isolated_slots, 1U, &isolated_scratch);
  isolated_requests[0U] = &primary_second;
  const auto expected =
      isolated.plan(isolated_requests, isolated_slots, 1U, &isolated_scratch);

  ReferenceSpaceMppiBatchOptimizer combined(config, false);
  BatchScratch combined_scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount>
      combined_requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> combined_slots{};
  combined_requests[0U] = &primary_first;
  combined_requests[1U] = &secondary;
  combined_slots[0U] = 2U;
  combined_slots[1U] = 4U;
  combined.plan(combined_requests, combined_slots, 2U, &combined_scratch);
  combined_requests[0U] = &primary_second;
  combined_slots[0U] = 2U;
  const auto actual =
      combined.plan(combined_requests, combined_slots, 1U, &combined_scratch);

  expectSameSelectedParameters(expected.candidates[0U], actual.candidates[0U]);
}

TEST(ReferenceSpaceMppiBatchOptimizer, ReusesWorkersAcrossTasksAndPlanning) {
  ReferenceSpaceMppiBatchOptimizer batch(enabledConfig(), true);
  std::array<std::thread::id, kMaximumBatchCandidateCount> ids{};
  batch.executeCandidates(ids.size(), [&](std::size_t i) { ids[i] = std::this_thread::get_id(); });
  for (std::size_t i = 0; i < ids.size(); ++i) {
    EXPECT_NE(ids[i], std::this_thread::get_id());
    for (std::size_t j = 0; j < i; ++j) { EXPECT_NE(ids[i], ids[j]); }
  }
  auto request = straightRequest();
  BatchScratch scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount> requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> slots{};
  for (std::size_t i = 0; i < ids.size(); ++i) { requests[i] = &request; slots[i] = i; }
  const auto result = batch.plan(requests, slots, ids.size(), &scratch);
  EXPECT_EQ(result.candidate_count, ids.size());
  for (std::size_t count : {0U, 1U, 3U, 5U, 2U, 5U}) {
    std::array<int, kMaximumBatchCandidateCount> values{};
    batch.executeCandidates(count, [&](std::size_t i) {
      values[i] = static_cast<int>(i + 1);
      if (count > 1) { EXPECT_EQ(ids[i], std::this_thread::get_id()); }
    });
    for (std::size_t i = 0; i < values.size(); ++i)
      EXPECT_EQ(values[i], i < count ? static_cast<int>(i + 1) : 0);
  }
}

TEST(ReferenceSpaceMppiBatchOptimizer, TaskExceptionsJoinAndLeaveWorkersReusable) {
  for (bool parallel : {false, true}) {
    ReferenceSpaceMppiBatchOptimizer batch(enabledConfig(), parallel);
    std::array<bool, kMaximumBatchCandidateCount> visited{};
    EXPECT_THROW(batch.executeCandidates(visited.size(), [&](std::size_t i) {
      visited[i] = true;
      if (i == 1) throw std::runtime_error("slot 1");
    }), std::runtime_error);
    for (bool value : visited) EXPECT_TRUE(value);
    batch.executeCandidates(visited.size(), [&](std::size_t i) { visited[i] = false; });
    for (bool value : visited) EXPECT_FALSE(value);
  }
}

TEST(ReferenceSpaceMppiBatchOptimizer, RejectsDuplicateWarmStartSlots) {
  auto request = straightRequest(false);
  ReferenceSpaceMppiBatchOptimizer batch(enabledConfig(), false);
  BatchScratch scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount> requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> slots{};
  requests[0U] = &request;
  requests[1U] = &request;
  slots[0U] = 1U;
  slots[1U] = 1U;

  const auto result = batch.plan(requests, slots, 2U, &scratch);

  EXPECT_EQ(result.candidate_count, 2U);
  EXPECT_FALSE(result.candidates[0U].valid);
  EXPECT_FALSE(result.candidates[1U].valid);
}

TEST(ReferenceSpaceMppiBatchOptimizer,
     SerialValidatorExceptionInvalidatesOnlyTheCandidate) {
  auto request = straightRequest(false);
  request.nominal_only = true;
  request.path_constraint_validator = [](const TemporaryReference &) {
    throw std::runtime_error("validator failure");
    return RejectReason::NONE;
  };
  ReferenceSpaceMppiBatchOptimizer batch(enabledConfig(), false);
  BatchScratch scratch;
  std::array<const PlanRequest *, kMaximumBatchCandidateCount> requests{};
  std::array<std::size_t, kMaximumBatchCandidateCount> slots{};
  requests[0U] = &request;

  const auto result = batch.plan(requests, slots, 1U, &scratch);

  ASSERT_EQ(result.candidate_count, 1U);
  EXPECT_FALSE(result.candidates[0U].valid);
  EXPECT_EQ(result.candidates[0U].reject_reason, RejectReason::INVALID_INPUT);
}

TEST(ReferenceSpaceMppiConfig, RejectsInvalidSamplingLayoutAndUnsafeSwitch) {
  auto config = enabledConfig();
  config.sample_count = 127U;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "sample_count_must_be_even_and_within_capacity");
  config = enabledConfig();
  config.allow_reference_switch = true;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "reference_switch_forbidden_in_shadow_mode");
  config = enabledConfig();
  config.steering_control_delay_sec = -0.01;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "invalid_nonnegative_parameter");
  config = enabledConfig();
  config.maximum_reference_segment_m = 0.0;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "invalid_vehicle_or_reference_parameter");
  config = enabledConfig();
  config.cost_pass_separation_weight = -1.0;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "invalid_nonnegative_parameter");
  config = enabledConfig();
  config.control_knot_count = kMaximumControlKnotCount + 1U;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "control_knot_count_out_of_range");
  config = enabledConfig();
  config.control_noise_correlation = 1.0;
  EXPECT_STREQ(ReferenceSpaceMppiPlanner(config).validateConfig(),
               "control_noise_correlation_out_of_range");
}

TEST(ReferenceSpaceMppiPlanning, ModelsAwsimDelayOnStraightReference) {
  auto config = enabledConfig();
  config.steering_control_delay_sec = 0.20;
  auto request = straightRequest(false);
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_GE(result.valid_sample_count, config.minimum_valid_count);
}

TEST(ReferenceSpaceMppiPlanning, NominalOnlyEvaluatesExactlyOneTemplate) {
  auto request = straightRequest(false);
  request.nominal_only = true;
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_EQ(result.valid_sample_count, 1U);
  EXPECT_DOUBLE_EQ(result.effective_sample_size, 1.0);
  EXPECT_EQ(totalRejectedSamples(result), 0U);
  EXPECT_DOUBLE_EQ(result.selected.d_pass_m, request.nominal.d_pass_m);
}

TEST(ReferenceSpaceMppiPlanning, RequestCanBoundTheMppiSamplesPerLine) {
  auto request = straightRequest(false);
  request.sample_count_override = 6U;
  request.sample_lateral_bounds = true;
  request.bounds.minimum.d_pass_m = request.nominal.d_pass_m;
  request.bounds.maximum.d_pass_m = request.nominal.d_pass_m;
  request.bounds.minimum.l_out_m = 4.0;
  request.bounds.maximum.l_out_m = 12.0;
  request.bounds.minimum.speed_scale = 0.60;
  request.bounds.maximum.speed_scale = 1.00;
  request.bounds.minimum.lateral_control_near_scale = 0.0;
  request.bounds.maximum.lateral_control_near_scale = 0.45;
  request.bounds.minimum.lateral_control_far_scale = 0.55;
  request.bounds.maximum.lateral_control_far_scale = 1.0;
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_EQ(result.valid_sample_count + totalRejectedSamples(result), 6U);
  EXPECT_NEAR(request.nominal.l_out_m + scratch.noise[0U][1U],
              request.nominal.l_out_m, 1.0e-12);
  EXPECT_NEAR(request.nominal.l_out_m + scratch.noise[1U][1U], 4.0, 1.0e-12);
  EXPECT_NEAR(request.nominal.l_out_m + scratch.noise[5U][1U], 12.0, 1.0e-12);
  EXPECT_NEAR(request.nominal.speed_scale + scratch.noise[0U][4U],
              request.nominal.speed_scale, 1.0e-12);
  EXPECT_NEAR(request.nominal.speed_scale + scratch.noise[1U][4U], 1.0,
              1.0e-12);
  EXPECT_NEAR(request.nominal.speed_scale + scratch.noise[2U][4U], 0.60,
              1.0e-12);
  EXPECT_NEAR(request.nominal.lateral_control_near_scale +
                  scratch.noise[1U][5U],
              0.45, 1.0e-12);
  EXPECT_NEAR(request.nominal.lateral_control_far_scale + scratch.noise[1U][6U],
              1.0, 1.0e-12);
  EXPECT_NEAR(request.nominal.lateral_control_near_scale +
                  scratch.noise[5U][5U],
              0.0, 1.0e-12);
  EXPECT_NEAR(request.nominal.lateral_control_far_scale + scratch.noise[5U][6U],
              0.55, 1.0e-12);
}

TEST(ReferenceSpaceMppiPlanning, RejectsInvalidPerRequestSampleCount) {
  auto request = straightRequest(false);
  request.sample_count_override = 5U;
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::INVALID_INPUT);
}

TEST(ReferenceSpaceMppiPlanning,
     NominalOnlyComparesLinesAgainstOnePreferredSeparation) {
  auto self_preferred = straightRequest(false);
  self_preferred.nominal_only = true;
  self_preferred.nominal.d_pass_m = 1.6;
  self_preferred.bounds.minimum = self_preferred.nominal;
  self_preferred.bounds.maximum = self_preferred.nominal;
  self_preferred.cost_preferred_pass_separation_m = 1.6;

  auto common_preferred = self_preferred;
  common_preferred.cost_preferred_pass_separation_m = 2.2;

  ReferenceSpaceMppiPlanner self_planner(enabledConfig());
  ReferenceSpaceMppiPlanner common_planner(enabledConfig());
  Scratch self_scratch;
  Scratch common_scratch;
  const auto self_result = self_planner.plan(self_preferred, &self_scratch);
  const auto common_result =
      common_planner.plan(common_preferred, &common_scratch);

  ASSERT_TRUE(self_result.valid);
  ASSERT_TRUE(common_result.valid);
  const double expected_penalty = enabledConfig().cost_pass_separation_weight *
                                  std::pow((1.6 - 2.2) / 2.2, 2.0);
  EXPECT_NEAR(common_result.selected_evaluation.cost -
                  self_result.selected_evaluation.cost,
              expected_penalty, 1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning,
     NominalOnlyPreservesRejectedTemplateForVisualization) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.nominal_only = true;
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum = request.nominal;
  request.bounds.maximum = request.nominal;
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{4.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::COLLISION);
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_DOUBLE_EQ(result.effective_sample_size, 0.0);
  EXPECT_EQ(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      1U);
  EXPECT_EQ(totalRejectedSamples(result), 1U);
  EXPECT_GT(result.selected_reference.count, 2U);
}

TEST(ReferenceSpaceMppiReference, QuinticBlendHasC2BoundaryConditions) {
  const double epsilon = 1.0e-4;
  EXPECT_DOUBLE_EQ(ReferenceSpaceMppiPlanner::quinticBlend(0.0), 0.0);
  EXPECT_DOUBLE_EQ(ReferenceSpaceMppiPlanner::quinticBlend(1.0), 1.0);
  const double derivative_at_start =
      (ReferenceSpaceMppiPlanner::quinticBlend(epsilon) -
       ReferenceSpaceMppiPlanner::quinticBlend(0.0)) /
      epsilon;
  const double derivative_at_end =
      (ReferenceSpaceMppiPlanner::quinticBlend(1.0) -
       ReferenceSpaceMppiPlanner::quinticBlend(1.0 - epsilon)) /
      epsilon;
  EXPECT_NEAR(derivative_at_start, 0.0, 1.0e-6);
  EXPECT_NEAR(derivative_at_end, 0.0, 1.0e-6);
  const double second_at_start =
      (ReferenceSpaceMppiPlanner::quinticBlend(2.0 * epsilon) -
       2.0 * ReferenceSpaceMppiPlanner::quinticBlend(epsilon) +
       ReferenceSpaceMppiPlanner::quinticBlend(0.0)) /
      (epsilon * epsilon);
  EXPECT_NEAR(second_at_start, 0.0, 0.01);
}

TEST(ReferenceSpaceMppiReference,
     MultiPointBlendChangesShapeAndKeepsC2Endpoints) {
  const double epsilon = 1.0e-4;
  const auto blend = [](double u, double near_control, double far_control) {
    return ReferenceSpaceMppiPlanner::multiPointLateralBlend(u, near_control,
                                                             far_control);
  };
  EXPECT_DOUBLE_EQ(blend(0.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(blend(1.0, 0.0, 1.0), 1.0);
  EXPECT_GT(blend(0.35, 0.8, 0.8), blend(0.35, 0.0, 1.0));
  EXPECT_LT(blend(0.65, 0.2, 0.2), blend(0.65, 0.0, 1.0));

  const double derivative_at_start =
      (blend(epsilon, 0.8, 0.8) - blend(0.0, 0.8, 0.8)) / epsilon;
  const double derivative_at_end =
      (blend(1.0, 0.2, 0.2) - blend(1.0 - epsilon, 0.2, 0.2)) / epsilon;
  EXPECT_NEAR(derivative_at_start, 0.0, 1.0e-5);
  EXPECT_NEAR(derivative_at_end, 0.0, 1.0e-5);
  const double second_at_start =
      (blend(2.0 * epsilon, 0.8, 0.8) - 2.0 * blend(epsilon, 0.8, 0.8) +
       blend(0.0, 0.8, 0.8)) /
      (epsilon * epsilon);
  EXPECT_NEAR(second_at_start, 0.0, 0.05);
}

TEST(ReferenceSpaceMppiReference,
     ActiveManeuverHorizonUsesOnlyTheCurrentPhase) {
  EXPECT_DOUBLE_EQ(ReferenceSpaceMppiPlanner::activeManeuverLength(
                       Phase::OVERTAKE, 30.0, 22.0),
                   30.0);
  EXPECT_DOUBLE_EQ(ReferenceSpaceMppiPlanner::activeManeuverLength(
                       Phase::PASS_CLEAR, 30.0, 22.0),
                   30.0);
  EXPECT_DOUBLE_EQ(
      ReferenceSpaceMppiPlanner::activeManeuverLength(Phase::MERGE, 30.0, 22.0),
      22.0);
}

TEST(ReferenceSpaceMppiSampling, IsDeterministicAndAntithetic) {
  const auto config = enabledConfig();
  const auto request = straightRequest(false);
  ReferenceSpaceMppiPlanner first(config);
  ReferenceSpaceMppiPlanner second(config);
  Scratch first_scratch;
  Scratch second_scratch;
  const auto first_result = first.plan(request, &first_scratch);
  const auto second_result = second.plan(request, &second_scratch);
  ASSERT_TRUE(first_result.valid)
      << ReferenceSpaceMppiPlanner::toString(first_result.reject_reason);
  ASSERT_TRUE(second_result.valid);
  EXPECT_DOUBLE_EQ(first_result.selected.d_pass_m,
                   second_result.selected.d_pass_m);
  EXPECT_DOUBLE_EQ(first_result.selected.l_out_m,
                   second_result.selected.l_out_m);
  EXPECT_DOUBLE_EQ(first_result.selected.speed_scale,
                   second_result.selected.speed_scale);
  EXPECT_DOUBLE_EQ(first_result.selected_evaluation.cost,
                   second_result.selected_evaluation.cost);
  EXPECT_EQ(first_scratch.noise, second_scratch.noise);
  // Antithetic symmetry belongs to the raw Gaussian draws. Projection may
  // clip one side; optimization records the actual projected deltas instead.
  PlannerEvaluationTestAccess::noise(first, request, config.sample_count, &first_scratch);
  for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
    EXPECT_DOUBLE_EQ(first_scratch.noise[0U][parameter], 0.0);
  }
  for (std::size_t positive = 1U; positive < 127U; positive += 2U) {
    for (std::size_t parameter = 0U; parameter < kParameterCount; ++parameter) {
      EXPECT_DOUBLE_EQ(first_scratch.noise[positive][parameter],
                       -first_scratch.noise[positive + 1U][parameter]);
    }
  }
  EXPECT_DOUBLE_EQ(first_scratch.noise[127U][2U], 0.0);
  EXPECT_DOUBLE_EQ(first_scratch.noise[127U][3U], 0.0);
}

TEST(ReferenceSpaceMppiSampling,
     ControlSequenceIsDeterministicAntitheticAndTrackBounded) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.sample_count_override = 12U;
  request.sample_control_sequence = true;
  request.visualized_sample_count = 4U;

  ReferenceSpaceMppiPlanner first(config);
  ReferenceSpaceMppiPlanner second(config);
  Scratch first_scratch;
  Scratch second_scratch;
  const auto first_result = first.plan(request, &first_scratch);
  const auto second_result = second.plan(request, &second_scratch);

  ASSERT_TRUE(first_result.valid)
      << ReferenceSpaceMppiPlanner::toString(first_result.reject_reason);
  ASSERT_TRUE(second_result.valid);
  ASSERT_EQ(first_result.visualized_sample_count, 4U);
  EXPECT_EQ(first_result.visualized_sample_indices[0U], 0U);
  EXPECT_EQ(first_result.visualized_sample_indices[1U], 3U);
  EXPECT_EQ(first_result.visualized_sample_indices[2U], 7U);
  EXPECT_EQ(first_result.visualized_sample_indices[3U], 11U);
  for (std::size_t sample = 0U; sample < 4U; ++sample) {
    EXPECT_GT(first_result.visualized_sample_references[sample].count, 2U);
  }
  ASSERT_EQ(first_result.selected_control_sequence.count,
            config.control_knot_count);
  ASSERT_EQ(second_result.selected_control_sequence.count,
            config.control_knot_count);
  for (std::size_t knot = 0U; knot < config.control_knot_count; ++knot) {
    EXPECT_DOUBLE_EQ(first_scratch.lateral_control_noise[0U][knot], 0.0);
    EXPECT_DOUBLE_EQ(first_scratch.speed_control_noise[0U][knot], 0.0);
    EXPECT_DOUBLE_EQ(
        first_result.selected_control_sequence.lateral_adjustment_m[knot],
        second_result.selected_control_sequence.lateral_adjustment_m[knot]);
    EXPECT_DOUBLE_EQ(
        first_result.selected_control_sequence.speed_scale_adjustment[knot],
        second_result.selected_control_sequence.speed_scale_adjustment[knot]);
    for (std::size_t positive = 1U; positive < 11U; positive += 2U) {
      EXPECT_DOUBLE_EQ(
          first_scratch.lateral_control_noise[positive][knot],
          -first_scratch.lateral_control_noise[positive + 1U][knot]);
      EXPECT_DOUBLE_EQ(first_scratch.speed_control_noise[positive][knot],
                       -first_scratch.speed_control_noise[positive + 1U][knot]);
    }
  }
  ASSERT_GT(first_result.selected_reference.count, 2U);
  for (std::size_t index = 0U; index < first_result.selected_reference.count;
       ++index) {
    const auto &point = first_result.selected_reference.points[index];
    EXPECT_GE(point.d_m, request.base_reference[index].minimum_d_m - 1.0e-12);
    EXPECT_LE(point.d_m, request.base_reference[index].maximum_d_m + 1.0e-12);
    if (point.s_m <= request.anchor_s_m) {
      EXPECT_NEAR(point.d_m, request.anchor_d_m, 1.0e-12);
    }
  }
}

TEST(ReferenceSpaceMppiSampling, CoversCompleteRequestedLateralInterval) {
  const auto config = enabledConfig();
  auto request = straightRequest(false);
  request.sample_lateral_bounds = true;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_NEAR(request.nominal.d_pass_m + scratch.noise.front()[0U],
              request.nominal.d_pass_m, 1.0e-12);
  EXPECT_NEAR(request.nominal.d_pass_m + scratch.noise[1U][0U],
              request.bounds.minimum.d_pass_m, 1.0e-12);
  EXPECT_NEAR(request.nominal.d_pass_m +
                  scratch.noise[config.sample_count - 1U][0U],
              request.bounds.maximum.d_pass_m, 1.0e-12);
  for (std::size_t sample = 2U; sample < config.sample_count; ++sample) {
    EXPECT_GT(scratch.noise[sample][0U], scratch.noise[sample - 1U][0U]);
  }
}

TEST(ReferenceSpaceMppiSampling,
     CoversIndependentLateralTransitionControlPoints) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  auto request = straightRequest(false);
  request.sample_lateral_bounds = true;
  request.nominal.lateral_control_near_scale = 0.0;
  request.nominal.lateral_control_far_scale = 1.0;
  request.bounds.minimum.lateral_control_near_scale = 0.0;
  request.bounds.maximum.lateral_control_near_scale = 1.0;
  request.bounds.minimum.lateral_control_far_scale = 0.0;
  request.bounds.maximum.lateral_control_far_scale = 1.0;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  double near_minimum = 1.0;
  double near_maximum = 0.0;
  double far_minimum = 1.0;
  double far_maximum = 0.0;
  for (std::size_t sample = 0U; sample < config.sample_count; ++sample) {
    near_minimum =
        std::min(near_minimum, request.nominal.lateral_control_near_scale +
                                   scratch.noise[sample][5U]);
    near_maximum =
        std::max(near_maximum, request.nominal.lateral_control_near_scale +
                                   scratch.noise[sample][5U]);
    far_minimum =
        std::min(far_minimum, request.nominal.lateral_control_far_scale +
                                  scratch.noise[sample][6U]);
    far_maximum =
        std::max(far_maximum, request.nominal.lateral_control_far_scale +
                                  scratch.noise[sample][6U]);
  }
  EXPECT_LE(near_minimum, 0.05);
  EXPECT_GE(near_maximum, 0.90);
  EXPECT_LE(far_minimum, 0.05);
  EXPECT_GE(far_maximum, 0.90);
}

TEST(ReferenceSpaceMppiSampling, IncludesFastGentleAndFastEarlyPasses) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  auto request = straightRequest(false);
  request.sample_lateral_bounds = true;
  request.sample_count_override = 10U;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  planner.plan(request, &scratch);
  EXPECT_NEAR(request.nominal.l_out_m + scratch.noise[1U][1U],
              request.bounds.minimum.l_out_m, 1e-9);
  EXPECT_NEAR(request.nominal.l_out_m + scratch.noise[9U][1U],
              request.bounds.maximum.l_out_m, 1e-9);
  for (const std::size_t sample : {1U, 9U}) {
    EXPECT_NEAR(request.nominal.speed_scale + scratch.noise[sample][4U],
                request.bounds.maximum.speed_scale, 1e-9);
  }
  EXPECT_NEAR(request.nominal.speed_scale + scratch.noise[2U][4U],
              request.bounds.minimum.speed_scale, 1e-9);
}

TEST(ReferenceSpaceMppiSampling, TwoSampleCoverageIsFinite) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  auto request = straightRequest(false);
  request.sample_lateral_bounds = true;
  request.sample_count_override = 2U;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  planner.plan(request, &scratch);
  for (const auto value : scratch.noise[1U]) EXPECT_TRUE(std::isfinite(value));
}

TEST(ReferenceSpaceMppiPlanning,
     RejectsNonMonotonicLateralTransitionControlPoints) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  auto request = straightRequest(false);
  request.nominal.lateral_control_near_scale = 0.80;
  request.nominal.lateral_control_far_scale = 0.20;
  request.bounds.minimum.lateral_control_near_scale = 0.80;
  request.bounds.maximum.lateral_control_near_scale = 0.80;
  request.bounds.minimum.lateral_control_far_scale = 0.20;
  request.bounds.maximum.lateral_control_far_scale = 0.20;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(
      result.reject_counts[static_cast<std::size_t>(RejectReason::PARAMETER)],
      config.sample_count);
}

TEST(ReferenceSpaceMppiPlanning, FindsTrackableStaticObstacleBypass) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  const auto result = planner.plan(straightRequest(true), &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_GE(result.valid_sample_count, 8U);
  EXPECT_GT(result.effective_sample_size, 0.0);
  EXPECT_GT(result.selected.d_pass_m, 1.2);
  EXPECT_GT(result.selected_reference.count, 100U);
  EXPECT_GT(result.selected_evaluation.progress_m, 5.0);
  EXPECT_GT(result.selected_evaluation.minimum_clearance_m, 0.0);
  EXPECT_LE(result.selected_evaluation.maximum_cross_track_error_m, 1.2);
}

TEST(ReferenceSpaceMppiPlanning, PreservesFrozenPrefix) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  const auto request = straightRequest(false);
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid);
  for (std::size_t index = 0U; index < result.selected_reference.count;
       ++index) {
    const auto &point = result.selected_reference.points[index];
    if (point.s_m <= request.anchor_s_m) {
      EXPECT_NEAR(point.d_m, request.anchor_d_m, 1.0e-12);
    }
  }
}

TEST(ReferenceSpaceMppiPlanning, CertifiesRequestedMinimumSpeedFloor) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.minimum_speed_mps = 6.0;
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  // Model a noisy base-reference curvature estimate. The generated Cartesian
  // reference is straight, so the rollout must certify the requested floor
  // instead of inheriting sqrt(6 / 1) == 2.45 m/s from this artifact.
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    request.base_reference[index].curvature_1pm = 1.0;
  }

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  ASSERT_GT(result.selected_reference.count, 0U);
  for (std::size_t index = 0U; index < result.selected_reference.count;
       ++index) {
    EXPECT_GE(result.selected_reference.points[index].speed_mps, 6.0);
  }
}

TEST(ReferenceSpaceMppiPlanning,
     AllowsPassageInsideWallBoundsDespiteNoisyBaseCurvature) {
  auto config = enabledConfig();
  config.maximum_cross_track_error_m = 1.5;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  auto request = straightRequest(false);
  request.side = -1;
  request.phase = Phase::OVERTAKE;
  request.complete_maneuver = false;
  request.ego.y_m = -1.6;
  request.ego.d_m = -1.6;
  request.anchor_d_m = -1.6;
  request.nominal.d_pass_m = -1.6;
  request.bounds.minimum.d_pass_m = -1.6;
  request.bounds.maximum.d_pass_m = -1.6;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    auto &base = request.base_reference[index];
    base.curvature_1pm = -1.0;
    base.minimum_d_m = -3.2;
    base.maximum_d_m = 3.8;
    base.active_d_m = -1.6;
    base.active_d_valid = true;
  }

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  ASSERT_GT(result.selected_reference.count, 0U);
  EXPECT_NEAR(
      result.selected_reference.points[result.selected_reference.count - 1U]
          .d_m,
      -1.6, 1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning, BrainManeuverReturnsToBaseAfterPassing) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.complete_maneuver = true;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  const double maneuver_end = request.anchor_s_m + result.selected.l_out_m +
                              result.selected.l_hold_m +
                              result.selected.l_merge_m;
  bool checked_return = false;
  for (std::size_t index = 0U; index < result.selected_reference.count;
       ++index) {
    const auto &point = result.selected_reference.points[index];
    if (point.s_m > maneuver_end + 0.5) {
      EXPECT_NEAR(point.d_m, 0.0, 1.0e-9);
      checked_return = true;
      break;
    }
  }
  EXPECT_TRUE(checked_return);
}

TEST(ReferenceSpaceMppiPlanning, RecedingOvertakeHoldsOffsetAtHorizon) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.complete_maneuver = false;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  ASSERT_GT(result.selected_reference.count, 3U);
  const auto &last =
      result.selected_reference.points[result.selected_reference.count - 1U];
  EXPECT_NEAR(last.d_m, result.selected.d_pass_m, 1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning,
     ScalesPassSeparationWithoutScalingOpponentReferenceOffset) {
  auto config = enabledConfig();
  config.maximum_cross_track_error_m = 1.5;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.ego.y_m = 1.25;
  request.ego.d_m = 1.25;
  request.anchor_d_m = 1.25;
  request.pass_profile_origin_d_m = 1.25;
  request.pass_profile_scale_m = 2.0;
  request.nominal.d_pass_m = 1.60;
  request.bounds.minimum.d_pass_m = 1.60;
  request.bounds.maximum.d_pass_m = 1.60;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    auto &base = request.base_reference[index];
    base.minimum_d_m = -4.0;
    base.maximum_d_m = 4.0;
    base.active_d_m = 1.25;
    base.active_d_valid = true;
    base.pass_d_m = 3.25;
    base.pass_d_valid = true;
  }

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  const auto &last =
      result.selected_reference.points[result.selected_reference.count - 1U];
  // 1.25 + (3.25 - 1.25) * (1.60 / 2.0) = 2.85. The old formula
  // incorrectly scaled the opponent offset too and produced 2.60.
  EXPECT_NEAR(last.d_m, 2.85, 1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning,
     ScalesSeparationAboutEachPredictedOpponentPosition) {
  auto config = enabledConfig();
  config.maximum_cross_track_error_m = 1.5;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.ego.y_m = 1.25;
  request.ego.d_m = 1.25;
  request.anchor_d_m = 1.25;
  request.pass_profile_origin_d_m = 1.25;
  request.pass_profile_scale_m = 2.0;
  request.nominal.d_pass_m = 1.60;
  request.bounds.minimum.d_pass_m = 1.60;
  request.bounds.maximum.d_pass_m = 1.60;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    auto &base = request.base_reference[index];
    base.minimum_d_m = -4.0;
    base.maximum_d_m = 4.0;
    base.active_d_m = 1.25;
    base.active_d_valid = true;
    base.pass_origin_d_m = 1.25 - 0.005 * base.s_m;
    base.pass_origin_d_valid = true;
    base.pass_d_m = base.pass_origin_d_m + 2.0;
    base.pass_d_valid = true;
  }

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  const std::size_t index = result.selected_reference.count - 1U;
  EXPECT_NEAR(result.selected_reference.points[index].d_m,
              request.base_reference[index].pass_origin_d_m + 1.60, 1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning, MergePhaseReturnsHeldOffsetToBase) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::MERGE;
  request.complete_maneuver = false;
  request.ego.y_m = 1.6;
  request.ego.d_m = 1.6;
  request.anchor_d_m = 1.6;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    request.base_reference[index].active_d_m = 1.6;
    request.base_reference[index].active_d_valid = true;
  }
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  bool checked_merge = false;
  for (std::size_t index = 0U; index < result.selected_reference.count;
       ++index) {
    const auto &point = result.selected_reference.points[index];
    if (point.s_m > request.anchor_s_m + result.selected.l_merge_m + 0.5) {
      EXPECT_NEAR(point.d_m, 0.0, 1.0e-9);
      checked_merge = true;
      break;
    }
  }
  EXPECT_TRUE(checked_merge);
}

TEST(ReferenceSpaceMppiPlanning,
     BrainChecksDynamicObstacleWithinExecutionHorizon) {
  auto config = enabledConfig();
  config.horizon_steps = 120U;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  auto request = straightRequest(false);
  request.phase = Phase::OVERTAKE;
  request.complete_maneuver = true;
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  request.horizon_steps_override = 60U;
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{30.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05};
  const auto short_result = planner.plan(request, &scratch);
  ASSERT_TRUE(short_result.valid);
  // The same obstacle becomes a real collision when execution reaches it.
  request.horizon_steps_override = 120U;
  ReferenceSpaceMppiPlanner longer_planner(config);
  const auto result = longer_planner.plan(request, &scratch);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_GT(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      0U);
  EXPECT_EQ(totalRejectedSamples(result), config.sample_count);
  EXPECT_GT(result.selected_evaluation.reject_s_m, 20.0);
}

TEST(ReferenceSpaceMppiPlanning, PreservesAcceptedProfileAcrossFrozenPrefix) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    auto &base = request.base_reference[index];
    base.active_d_m = 0.08 * base.s_m;
    base.active_d_valid = true;
  }
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid);
  for (std::size_t index = 0U; index < result.selected_reference.count;
       ++index) {
    const auto &point = result.selected_reference.points[index];
    if (point.s_m <= request.anchor_s_m) {
      EXPECT_NEAR(point.d_m, request.base_reference[index].active_d_m, 1.0e-12);
    }
  }
}

TEST(ReferenceSpaceMppiPlanning, FollowsVaryingCorridorInRelativeSpace) {
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  auto request = straightRequest(false);
  request.corridor_nominal_d_m = 1.55;
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    request.base_reference[index].corridor_d_m =
        1.55 + 0.20 * std::sin(0.25 * request.base_reference[index].s_m);
  }
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid);
  const std::size_t first = 100U;
  const std::size_t second = 140U;
  ASSERT_GT(result.selected_reference.count, second);
  EXPECT_NEAR(result.selected_reference.points[second].d_m -
                  result.selected_reference.points[first].d_m,
              request.base_reference[second].corridor_d_m -
                  request.base_reference[first].corridor_d_m,
              1.0e-9);
}

TEST(ReferenceSpaceMppiPlanning, RejectsAllSamplesWhenSideIsBlocked) {
  auto request = straightRequest(false);
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    request.base_reference[index].minimum_d_m = -0.20;
    request.base_reference[index].maximum_d_m = 0.20;
  }
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_EQ(result.reject_counts[static_cast<std::size_t>(RejectReason::TRACK)],
            128U);
  EXPECT_GT(result.selected_reference.count, 2U);
  EXPECT_EQ(result.selected_evaluation.reject_reason, RejectReason::TRACK);
}

TEST(ReferenceSpaceMppiPlanning,
     CollisionOnlyModeKeepsFiniteTrackAndTrackingViolations) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  config.maximum_cross_track_error_m = 1.0e-4;
  auto request = straightRequest(false);
  for (std::size_t index = 0U; index < request.base_reference_count; ++index) {
    request.base_reference[index].minimum_d_m = -0.20;
    request.base_reference[index].maximum_d_m = 0.20;
  }
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_GT(result.selected_evaluation.maximum_cross_track_error_m,
            config.maximum_cross_track_error_m);
}

TEST(ReferenceSpaceMppiPlanning,
     CollisionOnlyModeStillRejectsPhysicalCollision) {
  auto config = enabledConfig();
  config.collision_only_rejection = true;
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{4.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_GT(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      0U);
}

TEST(ReferenceSpaceMppiPlanning,
     RejectsDiscontinuousGeometryDuringSampleEvaluation) {
  auto request = straightRequest(false);
  for (std::size_t index = 50U; index < request.base_reference_count; ++index) {
    request.base_reference[index].x_m += 2.0;
  }
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(
      result.reject_counts[static_cast<std::size_t>(RejectReason::GEOMETRY)],
      128U);
  EXPECT_EQ(result.selected_evaluation.reject_reason, RejectReason::GEOMETRY);
}

TEST(ReferenceSpaceMppiPlanning, StableSoftminHandlesLargeCostSeparation) {
  auto config = enabledConfig();
  config.temperature = 1.0e-6;
  config.cost_reference_change_weight = 1000.0;
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(straightRequest(false), &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_TRUE(std::isfinite(result.effective_sample_size));
  EXPECT_GE(result.effective_sample_size, 1.0);
  EXPECT_TRUE(std::isfinite(result.selected_evaluation.cost));
}

TEST(ReferenceSpaceMppiPlanning, FinalSweepDetectsObstacleBetweenSamples) {
  auto config = enabledConfig();
  config.obstacle_longitudinal_inflation_m = 0.0;
  config.obstacle_lateral_inflation_m = 0.0;
  config.final_swept_check_step_m = 0.05;
  auto request = straightRequest(false);
  request.static_obstacle_count = 1U;
  request.static_obstacles[0] = StaticObstacle{0.10, 0.10, 0.0, 0.0};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  // The coarse 0.20 m reference samples miss this obstacle, but the swept
  // validation now participates in every candidate evaluation.
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_GT(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      0U);
  EXPECT_EQ(totalRejectedSamples(result), config.sample_count);
}

TEST(ReferenceSpaceMppiPlanning,
     IntegratedPathConstraintSelectsAnotherFeasibleSample) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.sample_lateral_bounds = true;
  request.bounds.minimum.d_pass_m = 0.20;
  request.bounds.maximum.d_pass_m = 1.90;
  std::size_t validation_count = 0U;
  request.path_constraint_validator =
      [&validation_count](const TemporaryReference &reference) {
        ++validation_count;
        return reference.points[reference.count - 1U].d_m <= 1.0
                   ? RejectReason::NONE
                   : RejectReason::WALL;
      };
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_LE(
      result.selected_reference.points[result.selected_reference.count - 1U]
          .d_m,
      1.0);
  EXPECT_GT(result.reject_counts[static_cast<std::size_t>(RejectReason::WALL)],
            0U);
  EXPECT_GT(result.valid_sample_count, 0U);
  EXPECT_GE(
      validation_count,
      result.valid_sample_count +
          result.reject_counts[static_cast<std::size_t>(RejectReason::WALL)]);
}

TEST(ReferenceSpaceMppiPlanning,
     IntegratedPathConstraintRejectsEveryOtherwiseFeasibleSample) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.path_constraint_validator = [](const TemporaryReference &) {
    return RejectReason::WALL;
  };
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_GT(result.reject_counts[static_cast<std::size_t>(RejectReason::WALL)],
            0U);
  EXPECT_EQ(totalRejectedSamples(result), config.sample_count);
  EXPECT_EQ(result.selected_evaluation.reject_reason, RejectReason::WALL);
}

TEST(ReferenceSpaceMppiPlanning,
     RolloutConstraintChecksThePredictedTrackingPose) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  std::size_t validation_count = 0U;
  request.rollout_constraint_validator = [&validation_count,
                                          &config](const RolloutState &previous,
                                                   const RolloutState &state) {
    ++validation_count;
    EXPECT_LT(previous.time_sec, state.time_sec);
    EXPECT_GE(state.time_sec, config.dt_sec);
    return RejectReason::WALL;
  };
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;

  const auto result = planner.plan(request, &scratch);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_GT(validation_count, 0U);
  EXPECT_EQ(result.valid_sample_count, 0U);
  EXPECT_GT(result.reject_counts[static_cast<std::size_t>(RejectReason::WALL)],
            0U);
}

TEST(ReferenceSpaceMppiPlanning, RejectsTimeAlignedDynamicCollision) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  auto request = straightRequest(false);
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{4.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      config.sample_count);
  EXPECT_NE(result.selected_evaluation.reject_step_index,
            std::numeric_limits<std::size_t>::max());
  EXPECT_TRUE(std::isfinite(result.selected_evaluation.reject_s_m));
}

TEST(ReferenceSpaceMppiPlanning, RejectsTurningFootprintCornerCollision) {
  auto config = enabledConfig();
  config.minimum_valid_count = 1U;
  config.minimum_valid_ratio = 0.0;
  config.obstacle_lateral_inflation_m = 0.90;
  auto request = straightRequest(false);
  request.ego.yaw_rad = 0.50;
  request.nominal.d_pass_m = 0.0;
  request.bounds.minimum.d_pass_m = 0.0;
  request.bounds.maximum.d_pass_m = 0.0;
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{0.5, 1.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ReferenceSpaceMppiPlanner planner(config);
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, RejectReason::NO_VALID_SAMPLE);
  EXPECT_EQ(
      result.reject_counts[static_cast<std::size_t>(RejectReason::COLLISION)],
      config.sample_count);
}

TEST(ReferenceSpaceMppiPlanning, ReportsFiniteTtcForSafeClosingTrajectory) {
  auto request = straightRequest(false);
  request.dynamic_obstacle_count = 1U;
  request.dynamic_obstacles[0] =
      DynamicObstacle{15.0, 0.0, 4.0, 0.0, 0.0, 0.05, 0.05};
  ReferenceSpaceMppiPlanner planner(enabledConfig());
  Scratch scratch;
  const auto result = planner.plan(request, &scratch);
  ASSERT_TRUE(result.valid)
      << ReferenceSpaceMppiPlanner::toString(result.reject_reason);
  EXPECT_TRUE(std::isfinite(result.selected_evaluation.minimum_ttc_sec));
  EXPECT_GT(result.selected_evaluation.minimum_ttc_sec, 0.0);
  EXPECT_GT(result.selected_evaluation.minimum_clearance_m, 0.0);
}

} // namespace
} // namespace reference_space_mppi_planner::mppi
