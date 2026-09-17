#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"

namespace reference_space_mppi_planner::mppi {

struct LongitudinalLead {
  double body_gap_m{0.0};
  double speed_mps{0.0};
  std::optional<double> observed_speed_mps{};
};

std::optional<LongitudinalLead> longitudinalLead(
    const DynamicObstacle &obstacle, double s, double d, double relative_yaw,
    double time, const Config &config);

// Desired net acceleration, before the controller/plant response.
double followingAcceleration(const LongitudinalLead &lead, double ego_speed,
                             const Config &config);

struct LongitudinalSpeedPlan {
  TemporaryReference reference;
  bool valid{false};
  std::size_t prediction_steps{0};
};

// Mark the first departing segment of a generated overtaking path. Source
// coordinates survive consumption of the accepted trajectory's prefix.
void markOvertakeStart(TemporaryReference &geometry);

// Pure, bounded planning on fixed geometry. This is a nominal speed proposal;
// the delayed PP/vehicle rollout remains the execution certificate.
LongitudinalSpeedPlan planLongitudinalSpeed(
    const TemporaryReference &geometry, const PlanRequest &request,
    const Config &config, bool use_preparation = false);

ExecutionSpeedResult planAndEvaluateLongitudinalSpeed(
    const ReferenceSpaceMppiPlanner &evaluator, const TemporaryReference &geometry,
    const PlanRequest &request, const Config &config);

}  // namespace reference_space_mppi_planner::mppi
