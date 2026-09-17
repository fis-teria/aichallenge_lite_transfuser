#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"

namespace reference_space_mppi_planner::mppi {

struct ExecutionMotionSample {
  RolloutState before, after;
  RejectReason rejection{RejectReason::NONE};
  std::size_t control_reference_index{0}, physical_reference_index{0};
  double tire_steering_rad{0.}, commanded_steering_rad{0.};
  double bounded_steering_rad{0.}, acceleration_mps2{0.};
  double far_x_m{0.}, far_y_m{0.}, near_x_m{0.}, near_y_m{0.};
};

// The same delayed controller/plant drives local feasibility and long-range
// route timing. The visitor may stop at a collision, endpoint or common goal.
bool visitExecutionMotion(
    const TemporaryReference &reference, const PlanRequest &request,
    const Config &config, std::size_t steps,
    const std::function<bool(const ExecutionMotionSample &)> &visitor,
    const std::array<double, kMaximumReferencePoints> *preview_curvatures = nullptr);

}  // namespace reference_space_mppi_planner::mppi
