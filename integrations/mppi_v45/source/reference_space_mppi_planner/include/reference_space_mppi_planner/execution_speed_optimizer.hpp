#pragma once

#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include <functional>
#include <map>
#include <vector>

namespace reference_space_mppi_planner::mppi {

struct ExecutionSpeedResult {
  TemporaryReference reference{};
  Evaluation evaluation{};
  std::size_t evaluated{0},valid{0};
  bool improved{false};
  bool preparation_selected{false};
};

// 10 coarse + 3*5 local levels + 2*2 refinement + 3 ramps, before dedup.
inline constexpr std::size_t kMaximumExecutionSpeedEvaluations=32;
enum class ExecutionSpeedSearch { Full, Coarse };
// Must complete every callback before returning. Reduction stays in search order.
using ExecutionSpeedExecutor = std::function<void(
    std::size_t, const std::function<void(std::size_t)> &)>;

// Heartbeat recovery has a separate, bounded latency budget from optimization.
// Search the existing spatial speed field, keeping zero sections and acceleration
// locations. Feasibility is not monotone in speed around moving obstacles: test
// each proposal, never infer validity from a speed bracket. No speed is guaranteed.
inline std::optional<TemporaryReference> recoverExecutionSpeed(
    const TemporaryReference &reference, const PathConstraintValidator &validate) {
  if (!validate || reference.count < 3 || reference.count > reference.points.size())
    return std::nullopt;
  double maximum = 0.;
  for (std::size_t i = 0; i < reference.count; ++i) {
    const double speed = reference.points[i].speed_mps;
    if (!std::isfinite(speed) || speed < 0.) return std::nullopt;
    maximum = std::max(maximum, speed);
  }
  if (maximum == 0.) return std::nullopt;
  for (int level = 9; level > 0; --level) {
    auto proposal = reference;
    for (std::size_t i = 0; i < proposal.count; ++i) {
      auto &p = proposal.points[i];
      p.speed_mps = static_cast<float>(p.speed_mps * (level / 10.));
      p.uncapped_speed_mps = p.speed_mps;
    }
    if (validate(proposal) == RejectReason::NONE) return proposal;
  }
  return std::nullopt;
}

// Bounded speed-only search, not an independent speed cap or minimum. Every
// proposal uses the measured initial state and the unchanged adoption checks.
template <class Planner>
auto executionSpeedEvaluator(const Planner &planner, const TemporaryReference &,
    const PlanRequest &request) {
  return [&planner,&request](const TemporaryReference &candidate) {
    return planner.evaluateExecutionReference(candidate,request);
  };
}
inline auto executionSpeedEvaluator(const ReferenceSpaceMppiPlanner &planner,
    const TemporaryReference &reference,const PlanRequest &request) {
  return planner.makeExecutionSpeedEvaluator(reference,request);
}

template <class Planner>
ExecutionSpeedResult optimizeExecutionSpeed(
    const Planner &planner,const TemporaryReference &remaining,
    const PlanRequest &request,const Evaluation &incumbent,
    ExecutionSpeedSearch search=ExecutionSpeedSearch::Full,
    const ExecutionSpeedExecutor &execute={}) {
  ExecutionSpeedResult best;
  best.reference=remaining;best.evaluation=incumbent;
  if(remaining.count<3 || request.base_reference_count<2)return best;
  double maximum=0.;
  for(std::size_t i=0;i<request.base_reference_count;++i)
    maximum=std::max(maximum,request.base_reference[i].speed_mps*request.bounds.maximum.speed_scale);
  if(!std::isfinite(maximum) || maximum<=0.)return best;
  const auto evaluate_reference=executionSpeedEvaluator(planner,remaining,request);
  std::vector<double> levels;
  levels.reserve(25);
  if(search==ExecutionSpeedSearch::Coarse) {
    // Absolute-grid neighbors retain useful launch speeds when the initial
    // profile is between grid levels, within a four-proposal budget.
    std::vector<double> grid;
    for(double u:{0.,.2,.4,.6,.8,1.})grid.push_back(static_cast<float>(u*maximum));
    const double first=remaining.points[0].speed_mps;
    const auto lower=std::lower_bound(grid.begin(),grid.end(),first);
    const auto upper=std::upper_bound(grid.begin(),grid.end(),first);
    levels={std::clamp(request.ego.speed_mps,0.,maximum),maximum};
    if(lower!=grid.begin())levels.push_back(*std::prev(lower));
    if(upper!=grid.end())levels.push_back(*upper);
    if (planner.config().cost_approach_speed_weight > 0.0)
      levels.push_back(std::clamp(request.preferred_matching_speed_mps,0.,maximum));
  } else {
  levels={0.,std::clamp(request.preferred_matching_speed_mps,0.,maximum)};
  for(double u:{.1,.15,.2,.3,.4,.6,.8,1.})levels.push_back(u*maximum);
  // Resolve the neighborhood of actual/commanded/matching speeds at the
  // model's one-step acceleration scale, irrespective of the maximum speed.
  // This is search resolution, not a reachable-speed cap or a speed floor.
  const auto &config=planner.config();
  const double step=std::max(config.maximum_acceleration_mps2,
      config.maximum_deceleration_mps2)*config.dt_sec;
  for(double center:{request.ego.speed_mps,remaining.points[0].speed_mps,
                     request.preferred_matching_speed_mps})
    for(double offset:{-1.,-.5,0.,.5,1.})
      levels.push_back(std::clamp(center+offset*step,0.,maximum));
  }
  for(auto &v:levels)v=static_cast<float>(v);
  std::sort(levels.begin(),levels.end());
  levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
  double best_constant=-1.;
  Evaluation best_constant_evaluation;
  std::map<float, Evaluation> constant_evaluations;
  bool incumbent_constant=true;
  for(std::size_t i=0;i<remaining.count;++i)
    incumbent_constant &= remaining.points[i].speed_mps==remaining.points[0].speed_mps &&
        remaining.points[i].uncapped_speed_mps==remaining.points[0].speed_mps;
  if(incumbent_constant)
    constant_evaluations.emplace(remaining.points[0].speed_mps,incumbent);
  const auto make_candidate=[&](double target,double ramp) {
    auto candidate=remaining;
    for(std::size_t i=0;i<candidate.count;++i) {
      auto &p=candidate.points[i];
      double u=ramp>0. ? std::clamp(p.s_m/ramp,0.,1.) : 1.;
      u=u*u*(3-2*u);
      p.speed_mps=static_cast<float>(u==1. ? target : p.speed_mps+u*(target-p.speed_mps));
      p.uncapped_speed_mps=p.speed_mps;
    }
    return candidate;
  };
  const auto reduce=[&](double target,double ramp,const Evaluation &evaluation,bool evaluated=true) {
    if(evaluated) {++best.evaluated;if(evaluation.valid)++best.valid;}
    // The best constant can equal (or lose to) the incumbent profile. Still
    // refine its bracket; otherwise a coarse grid hides a better nearby speed.
    if(ramp==0. && betterEvaluation(evaluation,best_constant_evaluation)) {
      best_constant=target;best_constant_evaluation=evaluation;
    }
    if(betterEvaluation(evaluation,best.evaluation)) {
      best.reference=make_candidate(target,ramp);best.evaluation=evaluation;best.improved=true;
    }
  };
  const auto evaluate_group=[&](std::vector<double> targets,double ramp) {
    if (search==ExecutionSpeedSearch::Coarse && ramp==0.) {
      // Joint sampling now supplies the same absolute command speeds. The
      // incumbent was already evaluated in this exact execution context.
      targets.erase(std::remove_if(targets.begin(),targets.end(),[&](double speed) {
        for(std::size_t i=0;i<remaining.count;++i)
          if(remaining.points[i].speed_mps!=speed ||
             remaining.points[i].uncapped_speed_mps!=speed)return false;
        return true;
      }),targets.end());
    }
    std::vector<Evaluation> evaluations(targets.size());
    std::vector<std::size_t> pending;
    for(std::size_t i=0;i<targets.size();++i) {
      if(ramp==0.) targets[i]=static_cast<float>(targets[i]);
      const auto cached=constant_evaluations.find(static_cast<float>(targets[i]));
      if(ramp==0. && cached!=constant_evaluations.end()) evaluations[i]=cached->second;
      else pending.push_back(i);
    }
    const auto evaluate_one=[&](std::size_t slot) {
      const auto i=pending[slot];
      if(execute) {
        // Concurrent proposals need independent mutable validator caches.
        auto local_evaluate=evaluate_reference;
        evaluations[i]=local_evaluate(make_candidate(targets[i],ramp));
      } else {
        evaluations[i]=evaluate_reference(make_candidate(targets[i],ramp));
      }
    };
    if(execute && !pending.empty()) execute(pending.size(),evaluate_one);
    else for(std::size_t i=0;i<pending.size();++i) evaluate_one(i);
    for (std::size_t i=0;i<targets.size();++i) {
      const bool evaluated=std::find(pending.begin(),pending.end(),i)!=pending.end();
      reduce(targets[i],ramp,evaluations[i],evaluated);
      if(ramp==0.) constant_evaluations[static_cast<float>(targets[i])]=evaluations[i];
    }
  };
  evaluate_group(levels,0.);
  if(search==ExecutionSpeedSearch::Coarse) {
    return best;
  }
  // Refine the best feasible constant inside its neighboring coarse bracket.
  if(best_constant>=0.) {
    auto upper=std::upper_bound(levels.begin(),levels.end(),best_constant);
    const auto index=static_cast<std::size_t>(upper-levels.begin());
    double low=index>=2?levels[index-2]:0.;
    double high=upper!=levels.end()?*upper:maximum;
    for(int iteration=0;iteration<2;++iteration) {
      const double center=best_constant;
      std::vector<double> refinement;
      if(center>low)refinement.push_back(.5*(low+center));
      if(center<high)refinement.push_back(.5*(center+high));
      evaluate_group(refinement,0.);
      if(best_constant<center)high=center;
      else if(best_constant>center)low=center;
      else {low=.5*(low+center);high=.5*(center+high);}
    }
  }
  // The incumbent carries existing acceleration locations. These spatial
  // proposals add gradual acceleration without repeatedly resetting its speed.
  evaluate_group({.3*maximum,.6*maximum,maximum},4.);
  return best;
}

inline TemporaryReference executionSpeedProfile(const TemporaryReference &retimed) {
  auto profile=retimed;
  for(std::size_t i=0;i<profile.count;++i)profile.points[i].s_m=profile.points[i].source_s_m;
  return profile;
}
} // namespace reference_space_mppi_planner::mppi
