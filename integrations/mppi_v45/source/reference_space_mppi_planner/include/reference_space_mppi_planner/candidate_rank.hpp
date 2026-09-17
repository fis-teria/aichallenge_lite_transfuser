#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace reference_space_mppi_planner {

inline double overtakeRank(double time) {
  return std::isfinite(time) && time >= 0.0 ? time : std::numeric_limits<double>::infinity();
}

// Feasibility is enforced by callers; non-finite costs never win. Keep the
// original scalar objective as a secondary criterion, not an artificial bonus.
inline bool betterCandidate(double cost, double pass_time, double incumbent_cost,
                            double incumbent_pass_time) {
  if (!std::isfinite(cost)) return false;
  if (!std::isfinite(incumbent_cost)) return true;
  const double a = overtakeRank(pass_time), b = overtakeRank(incumbent_pass_time);
  return a != b ? a < b : cost < incumbent_cost;
}

// The path-integral update operates within the best primary rank. Otherwise
// cheap stopping samples can erase the feasible passing solution again.
inline double candidateWeight(double cost, double pass_time, double best_cost,
                              double best_pass_time, double temperature) {
  if (overtakeRank(pass_time) != overtakeRank(best_pass_time)) return 0.0;
  return std::exp(std::max(-700.0, -(cost-best_cost)/temperature));
}

} // namespace reference_space_mppi_planner
