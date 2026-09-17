#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include "reference_space_mppi_planner/side_selection.hpp"
#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
PlanRequest continuationRequest() {
  PlanRequest r;
  r.valid = true;
  r.side = 1;
  r.phase = Phase::OVERTAKE;
  r.sample_staged_speed = true;
  r.sample_control_sequence = true;
  r.nominal_only = true;
  r.defer_warm_update = true;
  r.ego.speed_mps = 1.0;
  r.nominal = Parameters{0.0, 8.0, 10.0, 8.0, 0.5};
  r.bounds.minimum = Parameters{0.0, 5.0, 0.0, 5.0, 0.0};
  r.bounds.maximum = Parameters{2.0, 12.0, 18.0, 12.0, 1.0};
  r.cost_preferred_pass_separation_m = 0.0;
  r.base_reference_count = 101;
  for (std::size_t i = 0; i < r.base_reference_count; ++i) {
    auto &b = r.base_reference[i];
    b.s_m = b.x_m = i * .3;
    b.speed_mps = 4.0;
    b.minimum_d_m = -5.0;
    b.maximum_d_m = 5.0;
    b.active_d_valid = b.active_speed_valid = true;
    b.active_speed_mps = 1.0;
  }
  return r;
}
Config continuationConfig() {
  Config c;
  c.enabled = true;
  c.steering_control_delay_sec = 0.0;
  return c;
}

TEST(ExecutionContinuation, IdenticalHistoryWithDifferentBreakpointsHasZeroChangeCost) {
  auto request=continuationRequest();
  TemporaryReference held;
  held.count=5;
  for(std::size_t i=0;i<held.count;++i) {
    auto &p=held.points[i];
    p.x_m=p.s_m=std::array<double,5>{0.,2.15,5.23,11.37,29.}[i];
    p.y_m=std::array<double,5>{0.,.6,1.3,.7,0.}[i];
    p.speed_mps=p.uncapped_speed_mps=1.+.1*i;
  }
  setExecutionHistory(request,held);
  const auto field=projectReferenceField(held,request);
  EXPECT_NEAR(executionFieldCosts(field,request,continuationConfig())[1],0.,1e-12);
  auto changed=field;
  // Change the physical path, not just an untrusted projected diagnostic.
  for(std::size_t i=0;i<changed.count;++i)changed.points[i].y_m+=.3;
  EXPECT_GT(executionFieldCosts(changed,request,continuationConfig())[1],0.);
  EXPECT_FALSE(request.base_reference[100].active_d_valid);
}

TEST(ExecutionContinuation, SpeedOverlayConsumesSourceStationWithoutRecommittingGeometry) {
  TemporaryReference source;
  source.count=31;
  for(std::size_t i=0;i<source.count;++i) {
    auto &p=source.points[i];p.s_m=p.x_m=i;p.y_m=.03*i;
    p.speed_mps=p.uncapped_speed_mps=.5;
  }
  EgoState ego;ego.x_m=2.5;ego.y_m=.5;
  auto remaining=remainingExecutionTrajectory(source,ego,20.);
  ASSERT_TRUE(remaining);
  for(std::size_t i=0;i<remaining->count;++i) {
    auto &p=remaining->points[i];p.speed_mps=1.+.1*p.source_s_m;
  }
  const auto profile=executionSpeedProfile(*remaining);
  ego.x_m=5.;ego.y_m=.4;
  const auto plain=remainingExecutionTrajectory(source,ego,10.);
  const auto updated=remainingExecutionTrajectory(source,ego,10.,&profile);
  ASSERT_TRUE(plain);ASSERT_TRUE(updated);ASSERT_EQ(plain->count,updated->count);
  for(std::size_t i=0;i<plain->count;++i) {
    EXPECT_DOUBLE_EQ(plain->points[i].x_m,updated->points[i].x_m);
    EXPECT_DOUBLE_EQ(plain->points[i].y_m,updated->points[i].y_m);
    EXPECT_NEAR(updated->points[i].speed_mps,1.+.1*updated->points[i].source_s_m,1e-12);
  }
  EXPECT_DOUBLE_EQ(source.points[10].speed_mps,.5);
}

TEST(ExecutionContinuation, SpeedSearchUsesUnchangedValidatorAndDoesNotAlterGeometry) {
  auto request=continuationRequest();
  TemporaryReference held;held.count=31;
  for(std::size_t i=0;i<held.count;++i) {
    auto &p=held.points[i];p.s_m=p.x_m=p.source_s_m=i;
    p.speed_mps=p.uncapped_speed_mps=.5;
  }
  setExecutionHistory(request,held);
  const ReferenceSpaceMppiPlanner planner(continuationConfig());
  const auto incumbent=planner.evaluateExecutionReference(held,request);
  ASSERT_TRUE(incumbent.valid);
  const auto result=optimizeExecutionSpeed(planner,held,request,incumbent);
  ASSERT_TRUE(result.improved);EXPECT_LE(result.evaluated,kMaximumExecutionSpeedEvaluations);
  EXPECT_GT(result.reference.points[0].speed_mps,.5);
  EXPECT_LT(result.evaluation.cost,incumbent.cost);
  for(std::size_t i=0;i<held.count;++i) {
    EXPECT_DOUBLE_EQ(result.reference.points[i].x_m,held.points[i].x_m);
    EXPECT_DOUBLE_EQ(result.reference.points[i].y_m,held.points[i].y_m);
  }
  request.path_constraint_validator=[](const TemporaryReference &) {return RejectReason::WALL;};
  const auto blocked=planner.evaluateExecutionReference(held,request);
  ASSERT_FALSE(blocked.valid);
  const auto rejected=optimizeExecutionSpeed(planner,held,request,blocked);
  EXPECT_FALSE(rejected.improved);EXPECT_EQ(rejected.valid,0U);
  EXPECT_FALSE(rejected.evaluation.valid);
}

TEST(ExecutionContinuation, SuppliedAndGeneratedReferenceHaveSameCost) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  const auto r = continuationRequest();
  Scratch scratch;
  const auto generated = planner.plan(r, &scratch);
  ASSERT_TRUE(generated.valid);
  const auto supplied = planner.validateReference(generated.selected_reference, r);
  ASSERT_TRUE(supplied.valid);
  EXPECT_GT(generated.selected_evaluation.cost_terms[9], 0.0);
  for (std::size_t i = 0; i < supplied.cost_terms.size(); ++i)
    EXPECT_NEAR(supplied.cost_terms[i], generated.selected_evaluation.cost_terms[i], 1e-10) << i;
  EXPECT_NEAR(supplied.cost, generated.selected_evaluation.cost, 1e-10);
}

TEST(ExecutionContinuation, LongCartesianPathDoesNotCollapseAtLocalReferenceEnd) {
  auto r = continuationRequest();
  r.base_reference_count = 21;
  TemporaryReference path;
  path.count = 61;
  for (std::size_t i = 0; i < path.count; ++i) {
    auto &p = path.points[i];
    p.x_m = p.s_m = i;
    p.speed_mps = p.uncapped_speed_mps = 2.;
    if (i < r.base_reference_count) r.base_reference[i].x_m = r.base_reference[i].s_m = i;
  }
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  ASSERT_TRUE(planner.validateReference(path,r).valid);
  EXPECT_TRUE(planner.evaluateExecutionReference(path,r).valid);
  // The whole path still reaches the static-wall callback, including its tail.
  r.path_constraint_validator = [](const TemporaryReference &ref) {
    return ref.points[ref.count-1].x_m > 30. ? RejectReason::WALL : RejectReason::NONE;
  };
  EXPECT_EQ(planner.evaluateExecutionReference(path,r).reject_reason, RejectReason::WALL);
}

TEST(ExecutionContinuation, ForwardCartesianPathCanProjectRepeatedlyToACorner) {
  auto r = continuationRequest();
  r.base_reference_count = 21;
  r.horizon_steps_override = 2;
  r.ego.x_m = 9.; r.ego.y_m = -2.; r.ego.yaw_rad = M_PI/4;
  TemporaryReference path;
  path.count = 41;
  for (std::size_t i = 0; i < r.base_reference_count; ++i) {
    auto &b = r.base_reference[i];
    b.s_m = i; b.x_m = std::min<std::size_t>(i,10); b.y_m = i>10 ? i-10 : 0;
    b.yaw_rad = i<10 ? 0. : M_PI/2;
  }
  for (std::size_t i = 0; i < path.count; ++i) {
    auto &p = path.points[i];
    p.x_m = 9.+.1*i; p.y_m = -2.+.1*i; p.s_m = std::sqrt(2.)*.1*i;
    p.yaw_rad = M_PI/4; p.speed_mps = p.uncapped_speed_mps = 1.;
  }
  auto c = continuationConfig(); c.collision_only_rejection = true;
  EXPECT_TRUE(ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r).valid);
}

TEST(ExecutionContinuation, NonzeroOffsetPhysicalCostIsIndependentOfProposalNominal) {
  auto r = continuationRequest();
  r.cost_preferred_pass_separation_m = 1.5;
  r.ego.y_m = r.ego.d_m = 1.;
  TemporaryReference path;
  path.count = r.base_reference_count;
  for (std::size_t i = 0; i < path.count; ++i) {
    auto &p = path.points[i];
    p.x_m = p.s_m = r.base_reference[i].s_m; p.y_m = p.d_m = 1.;
    p.speed_mps = p.uncapped_speed_mps = 2.;
  }
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  r.nominal.d_pass_m = 0.;
  const auto first = planner.evaluateExecutionReference(path,r);
  r.nominal.d_pass_m = 1.5;
  const auto second = planner.evaluateExecutionReference(path,r);
  ASSERT_TRUE(first.valid); ASSERT_TRUE(second.valid);
  EXPECT_GT(first.cost_terms[2],0.);
  for (std::size_t i = 0; i < first.cost_terms.size(); ++i)
    EXPECT_NEAR(first.cost_terms[i],second.cost_terms[i],1e-12) << i;
}

TEST(ExecutionContinuation, PassOriginLookupSupportsFoldedProjection) {
  auto r=continuationRequest();
  r.horizon_steps_override=2;
  r.ego.speed_mps=.1;
  for(std::size_t i=0;i<r.base_reference_count;++i) {
    auto &b=r.base_reference[i];b.pass_origin_d_valid=true;
    b.pass_origin_d_m=.1*b.s_m;
  }
  TemporaryReference path;path.count=4;
  for(std::size_t i=0;i<path.count;++i) {
    auto &p=path.points[i];p.x_m=std::array<double,4>{0.,8.,1.,10.}[i];
    p.y_m=.1*p.x_m;p.speed_mps=p.uncapped_speed_mps=.1;
    if(i)p.s_m=path.points[i-1].s_m+std::hypot(p.x_m-path.points[i-1].x_m,p.y_m-path.points[i-1].y_m);
  }
  auto c=continuationConfig();c.collision_only_rejection=true;
  const auto score=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  ASSERT_TRUE(score.valid);
  // d exactly equals the varying origin at every point, including x=1 after
  // x=8. A forward-only base cursor instead reads the origin near x=8.
  EXPECT_NEAR(score.cost_terms[6],0.,1e-12);
}

TEST(ExecutionContinuation, RecordedFreeRunConnectionDoesNotReverseAtPolylineTurn) {
  // Generation 822: immutable Reference points relative to its ego projection.
  const std::array<std::array<double,2>,7> base{{
    {{-.00137356200139,.00262572311476}}, {{-.11222083699249,.21452314811904}},
    {{-.22306811199815,.42642057312332}}, {{-.33391538700380,.63831799812033}},
    {{-.47487298700435,.81602244811802}}, {{-.61583058700489,.99372689812299}},
    {{-.75678818700544,1.17143134812068}}
  }};
  const std::array<double,2> translation{{-2.47386589982489,-1.29412282253907}};
  std::array<double,2> previous_base{{0.,0.}}, previous=translation, previous_segment{{0.,0.}};
  double arc=0.;
  for (std::size_t i=0;i<base.size();++i) {
    arc+=std::hypot(base[i][0]-previous_base[0],base[i][1]-previous_base[1]);
    previous_base=base[i];
    const auto offset=executionConnectionTranslation(translation[0],translation[1],arc,13.95955440740577);
    const std::array<double,2> point{{base[i][0]+offset[0],base[i][1]+offset[1]}};
    const std::array<double,2> segment{{point[0]-previous[0],point[1]-previous[1]}};
    if (i>0) {
      EXPECT_GT(segment[0]*previous_segment[0]+segment[1]*previous_segment[1],0.) << i;
    }
    previous=point; previous_segment=segment;
  }
  EXPECT_EQ(executionConnectionTranslation(translation[0],translation[1],0.,14.),translation);
  const std::array<double,2> zero{{0.,0.}};
  EXPECT_EQ(executionConnectionTranslation(translation[0],translation[1],20.,14.),zero);
}

TEST(ExecutionContinuation, GeneratedNonzeroCurveAndSerializedGeometryUseSamePhysicalFields) {
  auto r=continuationRequest();
  r.nominal.d_pass_m=1.5; r.cost_preferred_pass_separation_m=1.5;
  r.ego.y_m=r.ego.d_m=.2;
  auto c=continuationConfig(); c.collision_only_rejection=true;
  ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
  const auto planned=planner.plan(r,&scratch);
  ASSERT_TRUE(planned.valid);
  const auto adopted=planner.evaluateExecutionReference(planned.selected_reference,r);
  ASSERT_TRUE(adopted.valid);
  for (std::size_t i=0;i<adopted.cost_terms.size();++i)
    EXPECT_NEAR(adopted.cost_terms[i],planned.selected_evaluation.cost_terms[i],1e-10) << i;
}

TEST(ExecutionContinuation, SuppliedHistoryIsNotOmittedWithDifferentPointDensity) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  auto r = continuationRequest();
  TemporaryReference ref;
  ref.count = 4;
  for (std::size_t i = 0; i < ref.count; ++i) {
    auto &p = ref.points[i];
    p.x_m = p.s_m = std::array<double, 4>{0., 2., 11., 30.}[i];
    p.speed_mps = p.uncapped_speed_mps = 2.;
  }
  const auto score = planner.validateReference(ref, r);
  ASSERT_TRUE(score.valid);
  EXPECT_NEAR(score.cost_terms[9], continuationConfig().cost_control_change_weight / 16., 1e-10);
}

TemporaryReference straightField() {
  TemporaryReference r;
  r.count = 31;
  for (std::size_t i = 0; i < r.count; ++i) {
    auto &p = r.points[i];
    p.x_m = p.s_m = i;
    p.speed_mps = p.uncapped_speed_mps = 1.+std::clamp(double(i)-4.,0.,2.);
  }
  return r;
}

TEST(ExecutionContinuation, HistoryAndSmoothnessAreInvariantToLinearResampling) {
  auto r = continuationRequest();
  auto a = straightField();
  auto b = a;
  b.count = 0;
  for (std::size_t i = 1; i < a.count; ++i) {
    b.points[b.count++] = a.points[i-1];
    auto mid = a.points[i-1];
    mid.s_m = mid.x_m = .5*(a.points[i-1].x_m+a.points[i].x_m);
    mid.speed_mps = .5*(a.points[i-1].speed_mps+a.points[i].speed_mps);
    b.points[b.count++] = mid;
  }
  b.points[b.count++] = a.points[a.count-1];
  const auto ca = executionFieldCosts(a,r,continuationConfig());
  const auto cb = executionFieldCosts(b,r,continuationConfig());
  EXPECT_GT(ca[0],0.); EXPECT_GT(ca[1],0.);
  EXPECT_NEAR(ca[0],cb[0],1e-12); EXPECT_NEAR(ca[1],cb[1],1e-12);
}

TEST(ExecutionContinuation, PrivateWarmCacheAndNominalDoNotChangePhysicalCost) {
  auto r = continuationRequest();
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  Scratch scratch;
  const auto generated = planner.plan(r,&scratch);
  ASSERT_TRUE(generated.valid);
  const auto before = planner.validateReference(generated.selected_reference,r);
  planner.accept(r,generated);
  r.nominal.d_pass_m = 1.5;
  r.nominal.speed_scale = .95;
  const auto after = planner.validateReference(generated.selected_reference,r);
  ASSERT_TRUE(after.valid);
  EXPECT_NEAR(before.cost,after.cost,1e-12);
}

TEST(ExecutionContinuation, RetainsWorldAccelerationAnchorAndInterpolatesTrimSpeed) {
  const auto active = straightField();
  EgoState ego;
  ego.x_m = 2.5;
  const auto remaining = remainingExecutionTrajectory(active,ego,3.);
  ASSERT_TRUE(remaining);
  EXPECT_DOUBLE_EQ(remaining->points[0].x_m,2.5);
  EXPECT_DOUBLE_EQ(remaining->points[0].speed_mps,1.);
  const auto &last = remaining->points[remaining->count-1];
  EXPECT_DOUBLE_EQ(last.x_m,5.5);
  EXPECT_DOUBLE_EQ(last.speed_mps,2.5);
  EXPECT_DOUBLE_EQ(last.s_m,3.);
  EXPECT_DOUBLE_EQ(active.points[5].x_m,5.);
  EXPECT_DOUBLE_EQ(active.points[5].speed_mps,2.);
}

TEST(ExecutionContinuation, EmptyOrConsumedSourceDoesNotInventAnExtension) {
  EgoState ego;
  EXPECT_FALSE(remainingExecutionTrajectory(TemporaryReference{},ego,20.));
  ego.x_m = 30.;
  EXPECT_FALSE(remainingExecutionTrajectory(straightField(),ego,20.));
  ego.x_m = 31.;
  EXPECT_FALSE(remainingExecutionTrajectory(straightField(),ego,20.));
}

TEST(ExecutionContinuation, ExhaustedTailCannotWinByExtrapolatingItsLastSpeed) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  auto r = continuationRequest();
  auto short_path = straightField();
  short_path.count = 3;
  const auto score = planner.evaluateExecutionReference(short_path,r);
  EXPECT_FALSE(score.valid);
  EXPECT_TRUE(score.reject_reason == RejectReason::PP_TRACKABILITY ||
              score.reject_reason == RejectReason::INVALID_INPUT);
}

TEST(ExecutionContinuation, RechecksChangedOpponentAndWallBeforeContinuing) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  auto r = continuationRequest();
  const auto path = straightField();
  ASSERT_TRUE(planner.evaluateExecutionReference(path,r).valid);
  r.dynamic_obstacle_count = 1;
  r.dynamic_obstacles[0].s_m = 5.;
  auto blocked = planner.evaluateExecutionReference(path,r);
  EXPECT_FALSE(blocked.valid);
  EXPECT_EQ(blocked.reject_reason,RejectReason::COLLISION);
  r.dynamic_obstacle_count = 0;
  r.path_constraint_validator = [](const TemporaryReference &) { return RejectReason::WALL; };
  blocked = planner.evaluateExecutionReference(path,r);
  EXPECT_FALSE(blocked.valid);
  EXPECT_EQ(blocked.reject_reason,RejectReason::WALL);
}

TEST(ExecutionContinuation, FrameConversionUsesReferenceStationNotCartesianArc) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  auto r = continuationRequest();
  r.ego.x_m = 100.;
  for (std::size_t i = 0; i < r.base_reference_count; ++i)
    r.base_reference[i].x_m += 100.;
  auto path = straightField();
  for (std::size_t i = 0; i < path.count; ++i) {
    path.points[i].x_m += 100.;
    path.points[i].s_m = 300.+2*path.points[i].s_m; // Untrusted transport metadata.
    path.points[i].d_m = 10.;
  }
  const auto expressed = planner.evaluateExecutionReference(path,r);
  ASSERT_TRUE(expressed.valid);
  EXPECT_NEAR(expressed.terminal_d_m,0.,1e-8);
  EXPECT_GT(expressed.progress_m,1.);
  EXPECT_DOUBLE_EQ(path.points[0].s_m,300.); // No mutation of published geometry/metadata.
}

TEST(ExecutionContinuation, SafeIncumbentCompetesWithSameAndOppositeSide) {
  std::array<SideSelectionCandidate,2> fresh{{{1,true,-2.},{-1,true,-1.}}};
  auto selected = selectExecutionCandidate(fresh,2,{1,true,-2.},1,0.);
  ASSERT_TRUE(selected); EXPECT_TRUE(selected->continuation); // Exact tie.
  fresh[0].cost = -3.;
  selected = selectExecutionCandidate(fresh,2,{1,true,-2.},1,0.);
  ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  EXPECT_EQ(selected->new_candidate_index,0U);
  fresh[1].cost = -4.;
  selected = selectExecutionCandidate(fresh,2,{1,true,-2.},1,0.);
  ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  EXPECT_EQ(selected->new_candidate_index,1U);
}

TEST(ExecutionContinuation, PreparedConnectionKeepsRetimedShapeDespiteCheaperOppositeSide) {
  std::array<SideSelectionCandidate,3> candidates{{
      {1,true,-.5466792735}, {-1,false,0.}, {-1,true,-.5317916852}}};
  auto selected=selectExecutionCandidate(candidates,3,{-1,false,0.},-1,0.,true);
  ASSERT_TRUE(selected);EXPECT_FALSE(selected->continuation);
  EXPECT_EQ(selected->new_candidate_index,2U);
  // Changed traffic invalidates the retained geometry: a certified alternative
  // can replace it immediately, without a timer or relaxed collision check.
  candidates[2].certified=false;
  selected=selectExecutionCandidate(candidates,3,{-1,false,0.},-1,0.,true);
  ASSERT_TRUE(selected);EXPECT_EQ(selected->new_candidate_index,0U);
  candidates[0].certified=false;
  EXPECT_FALSE(selectExecutionCandidate(candidates,3,{-1,false,0.},-1,0.,true));
}
TEST(ExecutionContinuation, PreparedReplenishmentUsesSameSideWhileGeometryRemainsFeasible) {
  std::array<SideSelectionCandidate,3> candidates{{
      {1,true,-10.}, {-1,true,2.}, {-1,true,-1.}}};
  auto selected=selectExecutionCandidate(candidates,3,{-1,false,0.},-1,0.,true,true);
  ASSERT_TRUE(selected);EXPECT_EQ(selected->new_candidate_index,1U);
  candidates[1].certified=false;
  selected=selectExecutionCandidate(candidates,3,{-1,false,0.},-1,0.,true,true);
  ASSERT_TRUE(selected);EXPECT_EQ(selected->new_candidate_index,2U);
}
TEST(ExecutionContinuation, UnsafeOrNonFiniteIncumbentNeverForcesContinuation) {
  std::array<SideSelectionCandidate,1> fresh{{{-1,true,3.}}};
  for (auto hold : {SideSelectionCandidate{1,false,-1e6}, SideSelectionCandidate{1,true,NAN}}) {
    const auto selected = selectExecutionCandidate(fresh,1,hold,1,0.);
    ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  }
  fresh[0].certified = false;
  EXPECT_FALSE(selectExecutionCandidate(fresh,1,{1,false,-1e6},1,0.));
}

TEST(ExecutionContinuation, FreshGeometryDoesNotBypassBetterRetimingOrHold) {
  std::array<SideSelectionCandidate,3> candidates{{
      {1,true,4.}, {-1,true,5.}, {1,true,-20.}}};
  const SideSelectionCandidate hold{1,true,-10.};
  auto selected=selectExecutionCandidate(candidates,3,hold,1,0.);
  ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  EXPECT_EQ(selected->new_candidate_index,2U);
  candidates[2].certified=false;
  selected=selectExecutionCandidate(candidates,3,hold,1,0.);
  ASSERT_TRUE(selected); EXPECT_TRUE(selected->continuation);
  selected=selectExecutionCandidate(candidates,3,{1,false,0.},1,0.);
  ASSERT_TRUE(selected); EXPECT_FALSE(selected->continuation);
  EXPECT_EQ(selected->new_candidate_index,0U);
  candidates[0].certified=false; candidates[1].certified=false;
  EXPECT_FALSE(selectExecutionCandidate(candidates,3,{1,false,0.},1,0.));
}

TEST(ExecutionContinuation, RecordedSlowReplenishmentDoesNotDisplaceBetterHold) {
  // 20260910-014515, generation 2572: replenishment used new1 despite all
  // passes being incomplete and its lower speed/progress (1.675 m/s, 4.624 m)
  // than the valid incumbent (1.883 m/s, 5.184 m).
  std::array<SideSelectionCandidate,5> candidates{{
      {1,true,.09958158928}, {1,true,.0007549756994},
      {1,true,.001037298181}, {1,true,.02568880524}, {1,true,.03545008394}}};
  const SideSelectionCandidate hold{1,true,-.05077459165};
  const auto selected=selectExecutionCandidate(candidates,5,hold,1,.2);
  ASSERT_TRUE(selected); EXPECT_TRUE(selected->continuation);
}

TEST(ExecutionContinuation, ReplanningConsumesMatchingSectionRatherThanRestartingIt) {
  ReferenceSpaceMppiPlanner planner(continuationConfig());
  const auto accepted = straightField();
  double last_first_speed = 0.;
  for (double position : {0.,1.,2.,3.,4.,5.,6.}) {
    auto r = continuationRequest();
    r.ego.x_m = position;
    r.ego.speed_mps = 1.+std::clamp(position-4.,0.,2.);
    for (std::size_t i = 0; i < r.base_reference_count; ++i) {
      auto &p = r.base_reference[i];
      p.x_m += position;
      p.active_speed_mps = 1.+std::clamp(p.x_m-4.,0.,2.);
    }
    const auto remaining = remainingExecutionTrajectory(accepted,r.ego,24.);
    ASSERT_TRUE(remaining);
    auto delayed = *remaining;
    for (std::size_t i = 0; i < delayed.count; ++i)
      delayed.points[i].speed_mps = 1.+std::clamp(delayed.points[i].x_m-position-4.,0.,2.);
    const auto hold_score = planner.evaluateExecutionReference(*remaining,r);
    const auto new_score = planner.evaluateExecutionReference(delayed,r);
    ASSERT_TRUE(hold_score.valid) << position;
    ASSERT_TRUE(new_score.valid) << position;
    const std::array<SideSelectionCandidate,1> fresh{{{1,new_score.valid,new_score.cost}}};
    const auto selected = selectExecutionCandidate(fresh,1,{1,hold_score.valid,hold_score.cost},1,0.);
    ASSERT_TRUE(selected);
    EXPECT_TRUE(selected->continuation) << position << " " << hold_score.cost << " " << new_score.cost;
    last_first_speed = remaining->points[0].speed_mps;
  }
  EXPECT_DOUBLE_EQ(last_first_speed,3.);
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
