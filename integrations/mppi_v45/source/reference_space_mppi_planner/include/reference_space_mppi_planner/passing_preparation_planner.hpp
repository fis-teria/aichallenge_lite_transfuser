#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include <string>

namespace reference_space_mppi_planner::mppi {

struct PassingPreparationConfig {
  double match_window_sec{1.0};
  double maximum_position_error_m{0.5};
  double maximum_heading_error_rad{0.25};
  double maximum_pace_error_ratio{0.25};
};

struct PriorLapMatch {
  bool usable{false};
  double position_error_m{0.0}, heading_error_rad{0.0}, pace_error_ratio{0.0};
  double observed_duration_sec{0.0};
  const char *reason{"missing_history"};
};

PriorLapMatch matchPriorLap(
    const std::vector<opponent_prediction::PositionObservation> &history,
    double reference_length, const PassingPreparationConfig &config);

struct PreparationPoint {
  double time_sec{0.0}, station_m{0.0}, speed_mps{0.0}, acceleration_mps2{0.0};
};

struct PassingPreparationPlan {
  std::uint64_t id{0}, revision{0};
  std::string target_id;
  std::shared_ptr<const ReferencePoseIndex> world;
  std::shared_ptr<const PrecomputedWallLines> wall_lines;
  PriorLapMatch match;
  double epoch_sec{0.0}, origin_station_m{0.0};
  double entry_station_m{0.0}, exit_station_m{0.0};
  double acceleration_start_station_m{0.0}, acceleration_start_time_sec{0.0};
  double lateral_start_station_m{0.0}, lateral_start_time_sec{0.0};
  double pass_station_m{0.0}, pass_time_sec{0.0}, entry_speed_mps{0.0};
  bool complete_pass{true};
  bool front_merge{false};
  bool prior_lap_guidance{false};
  double prior_lap_source_stamp_sec{-1.};
  std::array<double,2> candidate_minimum_center_distance_m{{-1.,-1.}};
  double merge_start_station_m{0.0}, merge_end_station_m{0.0};
  double relative_gain_m{0.0}, terminal_relative_speed_mps{0.0};
  std::uint64_t geometry_generation{0};
  int side{0};
  std::shared_ptr<const TemporaryReference> connection;
  std::vector<double> connection_stations_m;
  double connection_start_station_m{0.0}, connection_end_station_m{0.0};
  double validated_until_sec{0.0};
  std::vector<PreparationPoint> schedule;
  double targetTimeSec() const {
    return complete_pass ? pass_time_sec : schedule.empty() ? -1. : schedule.back().time_sec;
  }
  double targetStationM() const {
    return complete_pass ? pass_station_m : schedule.empty() ? origin_station_m : schedule.back().station_m;
  }
};

struct PreparationDiagnostics {
  const char *reason{"missing_history"};
  std::size_t windows{0}, trials{0}, horizon_exhausted{0}, window_exited{0};
  std::size_t road_exhausted{0}, lateral_not_ready{0};
  double horizon_sec{0.0}, initial_body_gap_m{0.0}, maximum_progress_m{0.0};
  double best_body_deficit_m{std::numeric_limits<double>::infinity()};
};

std::shared_ptr<const PassingPreparationPlan> makeFrontMergeGoal(
    const PlanRequest &request, const Config &config, const std::string &target_id,
    std::uint64_t revision,
    const std::shared_ptr<const PassingPreparationPlan> &previous = {},
    double minimum_pass_time_sec = 0., bool preselect_side = true);

std::vector<std::shared_ptr<const PassingPreparationPlan>> makeFrontMergeGoals(
    const PlanRequest &request,const Config &config,const std::string &target_id,
    std::uint64_t revision);

struct FrontMergeMetrics {
  double completion_time_sec{std::numeric_limits<double>::infinity()};
  double braking_mps{0.};
  double alongside_clearance_m{std::numeric_limits<double>::infinity()};
};
// Delayed-controller tracking on the complete candidate. Prior-lap preparations
// use the leader's time-indexed route; first-lap fallback and other opponents
// retain observed road speed. The short PP sweep uses the snapshot's selected
// opponent model, including the prior-lap route while its match is usable.
RejectReason validateFrontMerge(const PlanRequest &request, const Config &config,
    const TemporaryReference &geometry, FrontMergeMetrics *metrics = nullptr);

// Long-range guidance only. Actual geometry/speed pairs still require the
// unchanged execution evaluator, using the snapshot's selected collision forecasts.
std::shared_ptr<const PassingPreparationPlan> makePassingPreparationGoal(
    const PlanRequest &request, const Config &config, double gentle_acceleration,
    const std::string &target_id, const PriorLapMatch &match, std::uint64_t revision,
    const std::shared_ptr<const PassingPreparationPlan> &previous = {},
    PreparationDiagnostics *diagnostics = nullptr);

std::optional<PreparationPoint> preparationTarget(
    const PassingPreparationPlan &plan, double absolute_time_sec);
bool samePreparationWindow(const PassingPreparationPlan &a, const PassingPreparationPlan &b);
bool preparationSearchDue(const PassingPreparationPlan &plan, double stamp_sec,
                          double planning_delay_sec);
void applyPassingPreparation(PlanRequest &request,
    const std::shared_ptr<const PassingPreparationPlan> &plan);
// A single opportunity value replaces the independent long-range estimate.
double preparationProgress(const PlanRequest &request, const Config &config,
                           const Evaluation &evaluation);
std::shared_ptr<const PassingPreparationPlan> refinePassingPreparation(
    const PlanRequest &request, const Config &config, const Evaluation &evaluation,
    std::uint64_t geometry_generation);

// Geometry ownership survives a refreshed timing/window estimate. Each use
// still requires the normal current-motion execution validation.
bool preparationConnectionCurrent(const PassingPreparationPlan &plan,
    const PlanRequest &request, const std::string &target_id);
std::optional<double> preparationStation(const PassingPreparationPlan &plan,
    double x_m, double y_m);
bool pastPreparationConnectionEnd(const PassingPreparationPlan &plan,const PlanRequest &request);
std::shared_ptr<const PassingPreparationPlan> bindPreparationConnection(
    const PlanRequest &request, const Config &config, const Evaluation &evaluation,
    const TemporaryReference &published_geometry, int side, std::uint64_t generation,
    bool extend_existing = false, RejectReason *rejection = nullptr);

}  // namespace reference_space_mppi_planner::mppi
