#pragma once
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"

namespace reference_space_mppi_planner::mppi {
struct OtLaneEntryConfig {
  bool enabled{false};
  double entry_x_m{0.}, entry_y_m{0.}, exit_x_m{0.}, exit_y_m{0.};
  double target_speed_mps{30./3.6}, cost_weight{1.};
};
struct OtLaneEntryPlan {
  PassingPreparationPlan preparation;
  OtLaneEntryConfig config;
  double distance_m{0.}, predicted_speed_mps{0.};
  bool reached_entry{false}, reachable_free_road{false};
};
// Free-road guidance; every candidate retains the normal following and
// collision checks. Reachability here is not a collision-free certificate.
std::shared_ptr<const OtLaneEntryPlan> makeOtLaneEntryPlan(
    const PlanRequest &request, const Config &config, const OtLaneEntryConfig &entry);
double otLaneEntryCost(const PlanRequest &request, const Evaluation &evaluation);
} // namespace reference_space_mppi_planner::mppi
