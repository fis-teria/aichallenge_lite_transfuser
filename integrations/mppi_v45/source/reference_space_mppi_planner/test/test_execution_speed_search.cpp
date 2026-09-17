#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include "reference_space_mppi_planner/batch_optimizer.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::mppi {
namespace {
// Isolate search coverage from vehicle physics. Production still calls the
// real evaluateExecutionReference for every proposal (integration tests).
struct ScalarEvaluator {
  double optimum{.25};
  bool blocked{false};
  Config config_{};
  const Config &config() const { return config_; }
  Evaluation evaluateExecutionReference(const TemporaryReference &r,const PlanRequest &) const {
    Evaluation e;
    const double v=r.points[0].speed_mps;
    e.valid=!blocked && v<=.5;
    e.reject_reason=e.valid?RejectReason::NONE:RejectReason::WALL;
    if(e.valid)e.cost=(v-optimum)*(v-optimum);
    return e;
  }
};
PlanRequest speedRequest() {
  PlanRequest r;r.base_reference_count=2;
  r.base_reference[0].speed_mps=r.base_reference[1].speed_mps=10.;
  r.base_reference[1].s_m=r.base_reference[1].x_m=20.;
  r.bounds.maximum.speed_scale=1.;
  r.preferred_matching_speed_mps=.833333333;
  r.ego.speed_mps=.06;
  return r;
}
TemporaryReference speedPath() {
  TemporaryReference r;r.count=3;
  for(std::size_t i=0;i<r.count;++i) {
    auto &p=r.points[i];p.s_m=p.x_m=p.source_s_m=10.*i;
    p.speed_mps=p.uncapped_speed_mps=static_cast<float>(.209);
  }
  return r;
}
TEST(ExecutionSpeedSearch, RefinesIncumbentEvenWhenNoCoarseCandidateWins) {
  const auto r=speedRequest();const auto path=speedPath();const ScalarEvaluator evaluator;
  const auto incumbent=evaluator.evaluateExecutionReference(path,r);
  const auto result=optimizeExecutionSpeed(evaluator,path,r,incumbent);
  EXPECT_TRUE(result.improved);
  EXPECT_LT(result.evaluation.cost,incumbent.cost);
  EXPECT_NEAR(result.reference.points[0].speed_mps,.25,.03);
  EXPECT_LE(result.evaluated,kMaximumExecutionSpeedEvaluations);
}

TEST(ExecutionSpeedSearch, FullSearchReusesTheEvaluatedConstantIncumbent) {
  struct Evaluator : ScalarEvaluator {
    mutable std::size_t incumbent_calls{0};
    Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &r) const {
      bool same=true;
      for(std::size_t i=0;i<p.count;++i) same &= p.points[i].speed_mps==.25;
      if(same) ++incumbent_calls;
      return ScalarEvaluator::evaluateExecutionReference(p,r);
    }
  } evaluator;
  auto request=speedRequest();request.ego.speed_mps=.25;
  auto path=speedPath();
  for(std::size_t i=0;i<path.count;++i)path.points[i].speed_mps=path.points[i].uncapped_speed_mps=.25;
  const auto incumbent=evaluator.evaluateExecutionReference(path,request);
  const auto result=optimizeExecutionSpeed(evaluator,path,request,incumbent);
  EXPECT_EQ(evaluator.incumbent_calls,1U);
  EXPECT_FALSE(result.improved);
  EXPECT_DOUBLE_EQ(result.evaluation.cost,incumbent.cost);
}
TEST(ExecutionSpeedSearch, NewGeometryCoarseSearchIncludesMeasuredSpeedWithinFourEvaluations) {
  auto r=speedRequest();r.ego.speed_mps=.25;
  const auto path=speedPath();const ScalarEvaluator evaluator;
  const auto incumbent=evaluator.evaluateExecutionReference(path,r);
  const auto result=optimizeExecutionSpeed(evaluator,path,r,incumbent,ExecutionSpeedSearch::Coarse);
  EXPECT_TRUE(result.improved);
  EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,.25);
  EXPECT_LE(result.evaluated,4U);
  for(std::size_t i=0;i<path.count;++i) {
    EXPECT_DOUBLE_EQ(result.reference.points[i].x_m,path.points[i].x_m);
    EXPECT_DOUBLE_EQ(result.reference.points[i].y_m,path.points[i].y_m);
  }
}
TEST(ExecutionSpeedSearch, ReusesExactConstantIncumbentButEvaluatesDifferentProfiles) {
  struct Evaluator {
    mutable std::size_t calls{0};
    Config config_{};
    const Config &config() const {return config_;}
    Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &) const {
      ++calls;Evaluation e;e.valid=true;e.cost=0.;
      for(std::size_t i=0;i<p.count;++i)e.cost+=std::pow(p.points[i].speed_mps-8.,2);
      return e;
    }
  } evaluator;
  auto request=speedRequest();request.ego.speed_mps=8.;auto path=speedPath();
  for(std::size_t i=0;i<path.count;++i)path.points[i].speed_mps=path.points[i].uncapped_speed_mps=8.;
  auto incumbent=evaluator.evaluateExecutionReference(path,request);
  evaluator.calls=0;
  const auto reused=optimizeExecutionSpeed(evaluator,path,request,incumbent,ExecutionSpeedSearch::Coarse);
  EXPECT_EQ(evaluator.calls,2U);EXPECT_EQ(reused.evaluated,evaluator.calls);
  EXPECT_FALSE(reused.improved);EXPECT_DOUBLE_EQ(reused.evaluation.cost,incumbent.cost);
  path.points[1].speed_mps=path.points[1].uncapped_speed_mps=4.;
  incumbent=evaluator.evaluateExecutionReference(path,request);evaluator.calls=0;
  const auto changed=optimizeExecutionSpeed(evaluator,path,request,incumbent,ExecutionSpeedSearch::Coarse);
  EXPECT_EQ(evaluator.calls,3U);EXPECT_EQ(changed.evaluated,evaluator.calls);
  EXPECT_TRUE(changed.improved);EXPECT_DOUBLE_EQ(changed.evaluation.cost,0.);
}
TEST(ExecutionSpeedSearch, CoarseFindsSlowProgressWithoutReplacingBetterOrSaferProfiles) {
  struct Evaluator {
    double optimum;
    bool reject_low;
    bool tie_low{false};
    Config config_{};
    const Config &config() const {return config_;}
    Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &) const {
      const double speed=p.points[0].speed_mps;
      Evaluation e;e.valid=!(reject_low && (speed==2. || speed==4.));
      e.reject_reason=e.valid?RejectReason::NONE:RejectReason::COLLISION;
      if(e.valid)e.cost=tie_low && speed==2.?0.:(speed-optimum)*(speed-optimum);
      return e;
    }
  };
  auto request=speedRequest();request.ego.speed_mps=6.6;request.preferred_matching_speed_mps=5.2;
  auto path=speedPath();for(auto &p:path.points)p.speed_mps=p.uncapped_speed_mps=0.;
  for(double optimum:{2.,4.,8.}) {
    for(auto &p:path.points)p.speed_mps=p.uncapped_speed_mps=optimum-1.;
    const Evaluator evaluator{optimum,false};
    const auto result=optimizeExecutionSpeed(evaluator,path,request,
        evaluator.evaluateExecutionReference(path,request),ExecutionSpeedSearch::Coarse);
    EXPECT_TRUE(result.evaluation.valid);
    EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,optimum);
    EXPECT_LE(result.evaluated,4U);
  }
  for(auto &p:path.points)p.speed_mps=p.uncapped_speed_mps=0.;
  const Evaluator blocked{2.,true};
  const auto result=optimizeExecutionSpeed(blocked,path,request,
      blocked.evaluateExecutionReference(path,request),ExecutionSpeedSearch::Coarse);
  EXPECT_TRUE(result.evaluation.valid);
  EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,0.);
  Evaluation better;better.valid=true;better.cost=-1.;
  const auto kept=optimizeExecutionSpeed(blocked,path,request,better,ExecutionSpeedSearch::Coarse);
  EXPECT_FALSE(kept.improved);EXPECT_DOUBLE_EQ(kept.evaluation.cost,-1.);
  const Evaluator tied{8.,false,true};
  const auto same_rank=optimizeExecutionSpeed(tied,path,request,
      tied.evaluateExecutionReference(path,request),ExecutionSpeedSearch::Coarse);
  EXPECT_DOUBLE_EQ(same_rank.reference.points[0].speed_mps,2.);
}
TEST(ExecutionSpeedSearch, CoarseRecoversLaunchSpeedBelowBlockedUpperNeighbor) {
  struct Evaluator {
    Config config_{};
    const Config &config() const {return config_;}
    Evaluation evaluateExecutionReference(const TemporaryReference &p,const PlanRequest &) const {
      Evaluation e;const double speed=p.points[0].speed_mps;
      e.valid=speed<=2.;e.reject_reason=e.valid?RejectReason::NONE:RejectReason::WALL;
      if(e.valid)e.cost=std::pow(speed-2.,2);
      return e;
    }
  } evaluator;
  auto request=speedRequest();request.ego.speed_mps=1.0289577649397634;
  auto path=speedPath();
  for(auto &p:path.points)p.speed_mps=p.uncapped_speed_mps=static_cast<float>(request.ego.speed_mps);
  const auto result=optimizeExecutionSpeed(evaluator,path,request,
      evaluator.evaluateExecutionReference(path,request),ExecutionSpeedSearch::Coarse);
  EXPECT_TRUE(result.evaluation.valid);
  EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,2.);
  EXPECT_LE(result.evaluated,4U);
}
TEST(ExecutionSpeedSearch, ZeroRemainsAnOrdinaryFeasibleOptimum) {
  const auto r=speedRequest();const auto path=speedPath();ScalarEvaluator evaluator;
  evaluator.optimum=0.;
  const auto result=optimizeExecutionSpeed(evaluator,path,r,evaluator.evaluateExecutionReference(path,r));
  EXPECT_TRUE(result.evaluation.valid);EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,0.);
}
TEST(ExecutionSpeedSearch, RejectedProposalsCannotDisplaceIncumbent) {
  const auto r=speedRequest();const auto path=speedPath();ScalarEvaluator evaluator;
  Evaluation incumbent;incumbent.valid=true;incumbent.cost=1.;evaluator.blocked=true;
  const auto result=optimizeExecutionSpeed(evaluator,path,r,incumbent);
  EXPECT_FALSE(result.improved);EXPECT_EQ(result.valid,0U);
  EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,path.points[0].speed_mps);
}
TEST(ExecutionSpeedSearch, ParallelGroupsPreserveRefinementAndIncumbentSelection) {
  ReferenceSpaceMppiBatchOptimizer workers;
  const ExecutionSpeedExecutor execute=[&](std::size_t count,const auto &task) {
    const auto n=std::min(count,kMaximumBatchCandidateCount);
    workers.executeCandidates(n,[&](std::size_t slot) {
      // Reverse completion order without changing the search reduction order.
      for(std::size_t i=slot;i<count;i+=n) task(count-1-i);
    });
  };
  for (double optimum:{0.,.209,.25,.31,.5}) for (bool blocked:{false,true}) {
    const auto request=speedRequest();const auto path=speedPath();
    ScalarEvaluator evaluator;evaluator.optimum=optimum;evaluator.blocked=blocked;
    Evaluation incumbent;incumbent.valid=true;incumbent.cost=.005;
    const auto serial=optimizeExecutionSpeed(evaluator,path,request,incumbent);
    const auto parallel=optimizeExecutionSpeed(evaluator,path,request,incumbent,
        ExecutionSpeedSearch::Full,execute);
    EXPECT_EQ(parallel.evaluated,serial.evaluated);
    EXPECT_EQ(parallel.valid,serial.valid);
    EXPECT_EQ(parallel.improved,serial.improved);
    EXPECT_EQ(parallel.evaluation.valid,serial.evaluation.valid);
    EXPECT_DOUBLE_EQ(parallel.evaluation.cost,serial.evaluation.cost);
    for (std::size_t i=0;i<path.count;++i) {
      EXPECT_DOUBLE_EQ(parallel.reference.points[i].speed_mps,serial.reference.points[i].speed_mps);
      EXPECT_DOUBLE_EQ(parallel.reference.points[i].source_s_m,serial.reference.points[i].source_s_m);
    }
  }
}
TEST(ExecutionSpeedSearch, TiesUseSearchOrderRatherThanCompletionOrder) {
  struct FlatEvaluator : ScalarEvaluator {
    Evaluation evaluateExecutionReference(const TemporaryReference &,const PlanRequest &) const {
      Evaluation e;e.valid=true;e.cost=0.;return e;
    }
  } evaluator;
  Evaluation incumbent;incumbent.valid=true;incumbent.cost=1.;
  const auto result=optimizeExecutionSpeed(evaluator,speedPath(),speedRequest(),incumbent,
      ExecutionSpeedSearch::Full,[](std::size_t count,const auto &task) {
        while(count) task(--count);
      });
  EXPECT_TRUE(result.improved);
  EXPECT_DOUBLE_EQ(result.reference.points[0].speed_mps,0.);
}
TEST(ExecutionSpeedSearch, PreparedEvaluatorCopiesOwnMutableValidatorsAndOutliveInputs) {
  Config config;config.enabled=true;config.shadow_only=false;
  config.collision_only_rejection=true;
  config.horizon_steps=10;
  auto make=[] {
    auto request=speedRequest();
    request.valid=true;request.side=1;request.phase=Phase::OVERTAKE;
    request.sample_staged_speed=true;request.ego.speed_mps=1.;
    request.nominal={0.,4.,4.,4.,1.,0.,1.};
    request.bounds.minimum={0.,4.,4.,4.,.1,0.,1.};
    request.bounds.maximum={0.,4.,4.,4.,1.,0.,1.};
    request.base_reference_count=101;
    for(std::size_t i=0;i<request.base_reference_count;++i) {
      request.base_reference[i].s_m=request.base_reference[i].x_m=.4*i;
      request.base_reference[i].speed_mps=10.;
      request.base_reference[i].minimum_d_m=-10.;
      request.base_reference[i].maximum_d_m=10.;
    }
    return request;
  };
  auto path=speedPath();
  path.count=101;
  for(std::size_t i=0;i<path.count;++i) {
    path.points[i].s_m=path.points[i].x_m=path.points[i].source_s_m=.4*i;
    path.points[i].speed_mps=path.points[i].uncapped_speed_mps=1.;
  }
  std::function<Evaluation(const TemporaryReference &)> evaluate;
  {
    ReferenceSpaceMppiPlanner planner(config);
    auto request=make();
    ASSERT_TRUE(planner.evaluateExecutionReference(path,request).valid);
    request.path_constraint_validator=[calls=0](const TemporaryReference &) mutable {
      return ++calls==1 ? RejectReason::NONE : RejectReason::WALL;
    };
    evaluate=planner.makeExecutionSpeedEvaluator(path,request);
  }
  auto second=evaluate;
  EXPECT_TRUE(evaluate(path).valid);
  EXPECT_TRUE(second(path).valid);
  EXPECT_EQ(evaluate(path).reject_reason,RejectReason::WALL);
  EXPECT_EQ(second(path).reject_reason,RejectReason::WALL);
}

TEST(ExecutionSpeedRecovery, SearchesIntermediateSpeedsWithoutAssumingMonotoneFeasibility) {
  auto path=speedPath();
  for (auto &p:path.points) p.speed_mps=p.uncapped_speed_mps=10.;
  std::vector<double> attempted;
  const auto result=recoverExecutionSpeed(path,[&](const TemporaryReference &r) {
    const double v=r.points[0].speed_mps; attempted.push_back(v);
    // A moving obstacle closes the low-speed window; a wall closes the high one.
    return v==6. ? RejectReason::NONE : v<6. ? RejectReason::COLLISION : RejectReason::WALL;
  });
  ASSERT_TRUE(result); EXPECT_DOUBLE_EQ(result->points[0].speed_mps,6.);
  EXPECT_EQ(attempted,(std::vector<double>{9.,8.,7.,6.}));
  for (std::size_t i=0; i<path.count; ++i) {
    EXPECT_DOUBLE_EQ(result->points[i].x_m,path.points[i].x_m);
    EXPECT_DOUBLE_EQ(result->points[i].s_m,path.points[i].s_m);
    EXPECT_DOUBLE_EQ(result->points[i].source_s_m,path.points[i].source_s_m);
  }
}
TEST(ExecutionSpeedRecovery, PreservesSpatialStopsAndDoesNotRaiseFollowSpeed) {
  auto path=speedPath(); path.points[0].speed_mps=.8;
  path.points[1].speed_mps=0.; path.points[2].speed_mps=3.;
  const auto result=recoverExecutionSpeed(path,[](const TemporaryReference &) {return RejectReason::NONE;});
  ASSERT_TRUE(result);
  for (std::size_t i=0; i<path.count; ++i) {
    EXPECT_LE(result->points[i].speed_mps,path.points[i].speed_mps);
    EXPECT_DOUBLE_EQ(result->points[i].speed_mps,static_cast<float>(path.points[i].speed_mps*.9));
    EXPECT_DOUBLE_EQ(result->points[i].speed_mps,result->points[i].uncapped_speed_mps);
  }
  EXPECT_DOUBLE_EQ(result->points[1].speed_mps,0.);
}
TEST(ExecutionSpeedRecovery, ExhaustsBoundedSearchWithoutCertifyingBraking) {
  std::size_t calls=0;
  const auto result=recoverExecutionSpeed(speedPath(),[&](const TemporaryReference &) {
    ++calls; return RejectReason::WALL;
  });
  EXPECT_FALSE(result); EXPECT_EQ(calls,9U);
  auto path=speedPath(); for (auto &p:path.points)p.speed_mps=0.;
  EXPECT_FALSE(recoverExecutionSpeed(path,[&](const TemporaryReference &){++calls;return RejectReason::NONE;}));
  EXPECT_EQ(calls,9U);
  path.count=path.points.size()+1;
  EXPECT_FALSE(recoverExecutionSpeed(path,[](const TemporaryReference &){return RejectReason::NONE;}));
}
}  // namespace
}  // namespace reference_space_mppi_planner::mppi
