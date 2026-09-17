#pragma once

#include "reference_space_mppi_planner/candidate_rank.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <limits>
#include <tuple>

namespace reference_space_mppi_planner {

struct SideSelectionCandidate {
  int side{0};
  bool certified{false};
  double cost{0.0};
  double overtake_time_sec{std::numeric_limits<double>::infinity()};
  double completion_time_sec{std::numeric_limits<double>::infinity()};
  double braking_mps{0.};
  double alongside_clearance_m{0.};
};

inline bool betterSideCandidate(const SideSelectionCandidate &a,const SideSelectionCandidate &b) {
  if(std::isfinite(a.completion_time_sec) && std::isfinite(b.completion_time_sec))
    return std::make_tuple(a.completion_time_sec,a.braking_mps,-a.alongside_clearance_m,a.cost)<
           std::make_tuple(b.completion_time_sec,b.braking_mps,-b.alongside_clearance_m,b.cost);
  return betterCandidate(a.cost,a.overtake_time_sec,b.cost,b.overtake_time_sec);
}

template <std::size_t Capacity>
std::optional<std::size_t> selectSideCandidate(
    const std::array<SideSelectionCandidate, Capacity> &candidates,
    std::size_t candidate_count, int preferred_side,
    double switch_minimum_cost_improvement) {
  const std::size_t count = std::min(candidate_count, Capacity);
  std::optional<std::size_t> best;
  std::optional<std::size_t> preferred;
  for (std::size_t index = 0U; index < count; ++index) {
    const auto &candidate = candidates[index];
    if (!candidate.certified || !std::isfinite(candidate.cost)) {
      continue;
    }
    const auto better = [&](std::size_t other) {
      return betterSideCandidate(candidate,candidates[other]);
    };
    if (!best.has_value() || better(*best)) {
      best = index;
    }
    if (preferred_side != 0 && candidate.side == preferred_side &&
        (!preferred.has_value() ||
         better(*preferred))) {
      preferred = index;
    }
  }
  if (!preferred.has_value() || !best.has_value() || *preferred == *best) {
    return best;
  }

  const double required_improvement =
      std::max(0.0, switch_minimum_cost_improvement);
  if(std::isfinite(candidates[*best].completion_time_sec) &&
      std::isfinite(candidates[*preferred].completion_time_sec)) return best;
  if (overtakeRank(candidates[*best].overtake_time_sec) <
      overtakeRank(candidates[*preferred].overtake_time_sec)) return best;
  return candidates[*best].cost + required_improvement <
                 candidates[*preferred].cost
             ? best
             : preferred;
}

// Normalize by one common batch scale before applying hysteresis. Works for
// negative costs and avoids subtracting extreme unnormalized values.
template <std::size_t Capacity>
std::optional<std::size_t> selectSideCandidateRelative(
    const std::array<SideSelectionCandidate, Capacity> &candidates,
    std::size_t candidate_count, int preferred_side, double relative_improvement) {
  if (relative_improvement == 0.0) {
    return selectSideCandidate(candidates, candidate_count, preferred_side, 0.0);
  }
  auto normalized = candidates;
  double scale = 0.0;
  for (std::size_t i = 0; i < std::min(candidate_count, Capacity); ++i) {
    if (candidates[i].certified && std::isfinite(candidates[i].cost)) {
      scale = std::max(scale, std::abs(candidates[i].cost));
    }
  }
  if (scale > 0.0) {
    for (auto &candidate : normalized) candidate.cost /= scale;
  }
  return selectSideCandidate(normalized, candidate_count, preferred_side,
                             std::max(0.0, relative_improvement));
}

struct ExecutionSelection {
  bool continuation{false};
  std::size_t new_candidate_index{0};
};

// An incumbent participates in the same ranking, including opposite-side
// alternatives. Exact ties keep it; no time lock or forced side is introduced.
template <std::size_t Capacity>
std::optional<ExecutionSelection> selectExecutionCandidate(
    const std::array<SideSelectionCandidate, Capacity> &candidates,
    std::size_t candidate_count, const SideSelectionCandidate &continuation,
    int preferred_side, double relative_improvement,
    bool preserve_prepared_geometry = false, bool replenish_geometry = false) {
  std::array<SideSelectionCandidate, Capacity+1> all{};
  all[0] = continuation;
  const auto count = std::min(candidate_count, Capacity);
  std::copy_n(candidates.begin(), count, all.begin()+1);
  if(preserve_prepared_geometry && preferred_side!=0 && count>0) {
    // The final candidate is the incumbent geometry with a new speed field.
    const auto &retimed=candidates[count-1];
    const bool retime_valid=retimed.certified && retimed.side==preferred_side &&
        std::isfinite(retimed.cost);
    const bool hold_valid=continuation.certified && std::isfinite(continuation.cost);
    if(retime_valid || hold_valid) {
      if(replenish_geometry) {
        auto same_side=candidates;
        for(std::size_t i=0;i<count;++i)
          same_side[i].certified=i+1<count && same_side[i].side==preferred_side &&
              same_side[i].certified;
        const auto extension=selectSideCandidate(same_side,count,preferred_side,0.);
        if(extension) return ExecutionSelection{false,*extension};
      }
      if(retime_valid && (!hold_valid || betterCandidate(retimed.cost,retimed.overtake_time_sec,
          continuation.cost,continuation.overtake_time_sec)))
        return ExecutionSelection{false,count-1};
      return ExecutionSelection{true,0};
    }
  }
  const auto selected = selectSideCandidateRelative(all, count+1, preferred_side, relative_improvement);
  if (!selected) return std::nullopt;
  return ExecutionSelection{*selected == 0, *selected == 0 ? 0 : *selected-1};
}

} // namespace reference_space_mppi_planner
