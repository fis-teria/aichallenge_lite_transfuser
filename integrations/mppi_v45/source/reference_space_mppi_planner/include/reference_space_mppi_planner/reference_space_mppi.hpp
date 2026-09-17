#pragma once

#include "simple_pure_pursuit/rotation_controller.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include "reference_space_mppi_planner/precomputed_wall_lines.hpp"
#include "reference_space_mppi_planner/passing_point_areas.hpp"
#include "reference_space_mppi_planner/candidate_rank.hpp"
#include "reference_space_mppi_planner/prior_lap_prediction.hpp"

namespace reference_space_mppi_planner::mppi {

class BaseProjectionIndex;
struct PassingPreparationPlan;
struct OtLaneEntryPlan;

struct PassingRoadSample { double distance, speed, curvature; };

// Speed advice after curvature and braking limits, in the request's road frame.
struct PassingRoadProfile {
  std::vector<PassingRoadSample> samples;
};

constexpr std::size_t kParameterCount = 7U;
constexpr std::size_t kMaximumSampleCount = 128U;
// Six seconds at the default 50 ms step covers the 40 m overtake trigger at
// the configured closing speeds. A three-second dynamic-obstacle horizon let
// laterally moving opponents enter the selected corridor after certification.
constexpr std::size_t kMaximumHorizonSteps = 120U;
// The upstream execution profile is normally sampled at 0.20--0.25 m. 192
// points truncated a requested 50--65 m brain horizon to roughly 38--48 m,
// making the configured horizon ineffective. 384 retains at least 65 m at
// 0.20 m spacing while keeping all storage fixed-capacity.
constexpr std::size_t kMaximumReferencePoints = 384U;
constexpr std::size_t kMaximumStaticObstacles = 16U;
constexpr std::size_t kMaximumDynamicObstacles = 16U;
constexpr std::size_t kMaximumControlKnotCount = 12U;
constexpr std::size_t kMaximumVisualizedSampleCount = 4U;

enum class Phase : std::uint8_t {
  PREPARE = 0,
  OVERTAKE = 1,
  PASS_CLEAR = 2,
  MERGE = 3,
};

enum class RejectReason : std::uint8_t {
  NONE = 0,
  DISABLED,
  INVALID_CONFIG,
  INVALID_INPUT,
  UNKNOWN_SIDE,
  PARAMETER,
  GEOMETRY,
  TRACK,
  COLLISION,
  WALL,
  VEHICLE_LIMIT,
  PP_TRACKABILITY,
  NON_FINITE,
  NO_VALID_SAMPLE,
  COUNT,
};

struct Parameters {
  double d_pass_m{0.0};
  double l_out_m{8.0};
  double l_hold_m{8.0};
  double l_merge_m{8.0};
  double speed_scale{1.0};
  // Interior lateral control points of the seventh-order Bezier transition.
  // Together with the terminal d_pass value these let MPPI search a spatially
  // varying path instead of only scaling one Reference-parallel offset.
  double lateral_control_near_scale{0.0};
  double lateral_control_far_scale{1.0};
};

struct ParameterBounds {
  Parameters minimum{-2.0, 2.0, 0.0, 2.0, 0.5};
  Parameters maximum{2.0, 20.0, 30.0, 20.0, 1.05};
};

struct Config {
  bool longitudinal_planning_enabled{false};
  bool enabled{false};
  bool shadow_only{true};
  bool allow_reference_switch{false};
  // Experimental mode: only physical collision constraints (opponents and
  // walls) remain hard. Track, geometry, lateral-acceleration and tracking
  // error limits continue to influence cost/diagnostics but do not discard an
  // otherwise finite sample.
  bool collision_only_rejection{false};
  std::size_t sample_count{kMaximumSampleCount};
  std::size_t horizon_steps{kMaximumHorizonSteps};
  double dt_sec{0.05};
  double temperature{1.0};
  double update_gain{0.7};
  std::uint64_t deterministic_seed{20260902U};
  std::size_t minimum_valid_count{8U};
  double minimum_valid_ratio{0.10};
  std::array<double, kParameterCount> sigma{
      {0.10, 1.50, 0.00, 0.00, 0.04, 0.25, 0.25}};
  double freeze_guard_time_sec{0.20};
  double freeze_guard_min_distance_m{1.0};
  double reference_spacing_m{0.20};
  double maximum_reference_segment_m{1.50};
  double final_swept_check_step_m{0.15};
  double wheel_base_m{1.087};
  double lookahead_gain{0.9730464856294982};
  double lookahead_min_distance_m{1.9056789445669908};
  double curvature_lookahead_min_distance_m{1.370599267535113};
  double curvature_lookahead_sensitivity{1.9799676729694742};
  double curvature_lookahead_smoothing_alpha{0.21596381301312476};
  double maneuver_lookahead_time_constant_sec{0.20};
  double actual_lookahead_distance_blend{0.0};
  double dual_preview_near_ratio{0.557938018486374};
  double dual_preview_blend{0.22853606092001316};
  bool continuous_preview_interpolation_enabled{false};
  double steering_gain{1.45};
  bool curvature_feedforward_enabled{true};
  double curvature_feedforward_gain{0.60};
  // Individual switches for core A/B tests. The node sets all three geometry
  // mechanisms together through prediction.cma_controller_geometry_enabled.
  // This does not reproduce CMA recovery or its persisted filter state.
  bool compare_cma_delay_compensation{false};
  bool compare_cma_preview_feedforward{false};
  bool cma_lookahead_curvature_enabled{false};
  double cma_curvature_preview_distance_m{21.422826964417297};
  bool steering_passthrough_enabled{false};
  double steering_control_delay_sec{0.20};
  double steering_time_constant_sec{0.30};
  bool awsim_vehicle_response_enabled{false};
  double yaw_response_time_constant_sec{0.10};
  double cma_prediction_delay_sec{0.20};
  double cma_prediction_steering_time_constant_sec{0.30};
  // Ackermann command angle and measured tire angle are different domains in
  // AWSIM. The measured response is approximately 0.60 times the command.
  double steering_command_to_tire_angle_ratio{0.60};
  double maximum_steering_angle_rad{0.5235987755982988};
  double maximum_tire_steering_angle_rad{0.3141592653589793};
  double maximum_steering_rate_radps{4.0};
  double physical_tire_steering_rate_radps{0.0};
  bool steering_demand_acceleration_hold_enabled{true};
  double steering_acceleration_hold_minimum_speed_mps{3.0};
  double steering_acceleration_hold_minimum_tire_angle_rad{0.15};
  double steering_acceleration_hold_tracking_error_rad{0.08};
  double steering_acceleration_hold_maximum_acceleration_mps2{0.0};
  double maximum_lateral_acceleration_mps2{6.0};
  double maximum_cross_track_error_m{0.80};
  double speed_proportional_gain{1.0};
  double acceleration_time_constant_sec{0.25};
  double maximum_acceleration_mps2{1.0};
  double maximum_deceleration_mps2{2.0};
  double vehicle_half_length_m{1.10};
  double vehicle_half_width_m{0.65};
  double clearance_target_m{0.35};
  double obstacle_longitudinal_inflation_m{1.10};
  double obstacle_lateral_inflation_m{0.90};
  double cost_progress_weight{1.0};
  double cost_terminal_lateral_weight{0.0};
  double cost_clearance_weight{4.0};
  double cost_approach_speed_weight{0.0};
  double cost_passing_progress_weight{0.0};
  double cost_passing_opportunity_weight{0.0};
  double follow_gap_m{5.0};
  double overtake_follow_gap_m{2.0};
  double approach_time_sec{2.0};
  bool approach_at_follow_gap_only{false};
  double cost_reference_weight{0.25};
  double cost_curvature_weight{0.20};
  double cost_pp_error_weight{2.0};
  double cost_steering_rate_weight{0.10};
  double cost_reference_change_weight{0.50};
  // Prefer the requested vehicle-to-vehicle passing separation while still
  // sampling the complete footprint-eroded corridor when the preferred line
  // is not executable.
  double cost_pass_separation_weight{50.0};
  double cost_wall_line_weight{0.0};
  // Nav2-style temporally correlated sampling of a spatial reference-control
  // sequence around each lateral mode. The resulting path is still evaluated
  // through the delayed CMA PP model before it can be selected.
  std::size_t control_knot_count{8U};
  double control_lateral_std_m{0.20};
  // Zero retains legacy per-knot noise. Otherwise lateral noise uses metres.
  double control_lateral_reference_spacing_m{0.0};
  double control_speed_scale_std{0.04};
  double control_noise_correlation{0.75};
  double control_maximum_lateral_adjustment_m{0.45};
  double control_maximum_speed_scale_adjustment{0.12};
  double control_entry_ramp_m{4.0};
  double cost_control_smoothness_weight{0.10};
  double cost_control_change_weight{0.15};
  // Multiplier for only the lateral part of ordered execution history cost.
  double history_lateral_cost_scale{1.0};
};

struct ReferenceControlSequence {
  std::array<double, kMaximumControlKnotCount> lateral_adjustment_m{};
  std::array<double, kMaximumControlKnotCount> speed_scale_adjustment{};
  std::size_t count{0U};
};

struct BaseReferencePoint {
  double s_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature_1pm{0.0};
  double speed_mps{0.0};
  // Vehicle-center bounds after footprint and hard-margin erosion.
  double minimum_d_m{-2.0};
  double maximum_d_m{2.0};
  // Center of the pass corridor. d_pass is sampled as a scalar displacement
  // about this varying profile instead of as one global Frenet offset.
  double corridor_d_m{0.0};
  // Already accepted reference copied verbatim through the frozen prefix and
  // used as the C2 source profile for the replannable suffix.
  double active_d_m{0.0};
  bool active_d_valid{false};
  // Absolute speed on the last published path at this same Reference normal.
  // Unlike optimizer warm state, this is an execution-owned continuation seed.
  double active_speed_mps{0.0};
  bool active_speed_valid{false};
  // Optional spatially varying pass profile. MPPI scales this profile with
  // d_pass instead of forcing one constant offset through narrow sections.
  double pass_d_m{0.0};
  bool pass_d_valid{false};
  // Predicted opponent lateral position at this spatial point.  A moving
  // opponent can follow a fixed physical lane whose d coordinate varies along
  // a curved global Reference, so one request-wide origin is insufficient.
  double pass_origin_d_m{0.0};
  bool pass_origin_d_valid{false};
};

struct StaticObstacle {
  double minimum_s_m{0.0};
  double maximum_s_m{0.0};
  double minimum_d_m{0.0};
  double maximum_d_m{0.0};
};

struct DynamicObstacle {
  double s_m{0.0};
  double d_m{0.0};
  double longitudinal_speed_mps{0.0};
  double lateral_speed_mps{0.0};
  double longitudinal_acceleration_bound_mps2{0.0};
  double longitudinal_uncertainty_m{0.0};
  double lateral_uncertainty_m{0.0};
  double heading_relative_to_reference_rad{0.0};
  double global_reference_s_m{0.0};
  std::shared_ptr<const opponent_prediction::Prediction> prediction{};
};

struct EgoState {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
  double steering_rad{0.0};
  std::array<double, kMaximumHorizonSteps> pending_steering_targets_rad{};
  std::size_t pending_steering_target_count{0U};
  double acceleration_mps2{0.0};
  double s_m{0.0};
  double d_m{0.0};
  double yaw_rate_radps{0.0};
};

// A soft preference for a longitudinally leading, laterally overlapping body.
// The target may be below the opponent speed; it is never a command limit.
std::optional<double> approachSpeedTarget(const DynamicObstacle &obstacle,
    double ego_s_m, double ego_d_m, double ego_relative_yaw_rad,
    double time_sec, const Config &config);

struct TemporaryReference;
using PathConstraintValidator =
    std::function<RejectReason(const TemporaryReference &)>;

struct RolloutState {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
  double time_sec{0.0};
};

// Environment-specific physical checks for the state predicted by the same
// delayed Pure Pursuit rollout used by the optimizer. This complements the
// generated-Reference sweep: a safe Reference alone does not prove that the
// tracked vehicle footprint remains inside the walls.
using RolloutConstraintValidator =
    std::function<RejectReason(const RolloutState &, const RolloutState &)>;

struct PlanRequest {
  simple_pure_pursuit::RotationPredictionSnapshot rotation_prediction;
  // Continue the currently executing reference until the new plan can arrive.
  // All routes in one comparison share this measured planning delay/history.
  double reference_activation_delay_sec{0.};
  std::shared_ptr<const TemporaryReference> prior_execution_reference{};
  bool speed_pair_diagnostics{false}; // Read-only counterfactuals; never selected.
  bool valid{false};
  // Zero preserves the legacy Config horizon. Shared by every batch sample.
  std::size_t horizon_steps_override{0U};
  // Evaluate request.nominal exactly once. Brain mode uses this for its small
  // deterministic trajectory library; the normal stochastic MPPI path remains
  // available for legacy refinement requests.
  bool nominal_only{false};
  // Zero uses Config::sample_count. A small even override lets the brain keep
  // its five lateral line seeds while running bounded MPPI optimization inside
  // each line instead of evaluating one frozen template.
  std::size_t sample_count_override{0U};
  // When true, OVERTAKE is a complete shift-hold-return maneuver. The legacy
  // StateLattice refinement path leaves this false and keeps its input side.
  bool complete_maneuver{false};
  bool front_merge_attack{false};
  bool compare_passing_points{false};
  // Null preserves legacy unrestricted placement. A populated empty set forbids new points.
  std::shared_ptr<const PassingPointAreas> passing_point_areas;
  std::uint64_t generation{0U};
  // Target/phase identity. A generation may be accepted with bounded lag only
  // inside the same semantic maneuver, and warm state must never cross this
  // boundary.
  std::uint64_t semantic_key{0U};
  double stamp_sec{0.0};
  int side{0};
  Phase phase{Phase::PREPARE};
  EgoState ego{};
  double anchor_s_m{0.0};
  double anchor_d_m{0.0};
  double corridor_nominal_d_m{0.0};
  // A non-negative value keeps every fixed library line comparable against
  // one common preferred passing distance. Negative preserves the stochastic
  // path's historical request.nominal-based behavior.
  double cost_preferred_pass_separation_m{-1.0};
  // Common spatial passing zone; a disabled interval preserves legacy scoring.
  double wall_line_start_s_m{0.0};
  double wall_line_end_s_m{0.0};
  // For a spatial pass profile, d_pass is the sampled center-to-center
  // separation while pass_d_m is an absolute Frenet position.  Scale only the
  // separation about this origin; scaling the opponent's own lateral offset
  // pushes an already offset opponent's pass lane into the wall.
  double pass_profile_origin_d_m{0.0};
  double pass_profile_scale_m{0.0};
  // Brain mode covers the complete wall/vehicle-derived lateral interval and
  // the lateral transition control-point bounds. Legacy refinement retains
  // antithetic Gaussian sampling when false.
  bool sample_lateral_bounds{false};
  // Sample a spatial sequence d[k], v[k] about this mode rather than relying
  // only on the seven global maneuver parameters.
  bool sample_control_sequence{false};
  // Sample absolute command speeds against the maximum base speed, including
  // matching, acceleration and coarse constant-speed profiles. Vehicle rollout
  // evaluates them without the legacy generated-curvature speed cap.
  bool sample_staged_speed{false};
  // Common Cartesian prefix for a comparison, chosen before speed proposals.
  // Infinity preserves whole-reference scoring for callers without a window.
  double curvature_evaluation_distance_m{std::numeric_limits<double>::infinity()};
  // Matching the opponent is a proposal, never a lower bound on feasible speed.
  double preferred_matching_speed_mps{0.0};
  // Brain workers may optimize unselected/superseded modes. Only publication
  // commits their warm state; legacy callers retain automatic warm updates.
  bool defer_warm_update{false};
  // Brain-mode geometry is joined to the measured Cartesian pose before any
  // yaw/curvature/speed calculation. Frenet normals need not be continuous.
  bool cartesian_measured_prefix{false};
  // Preserve a small, evenly distributed subset of raw samples for RViz.
  // This is diagnostic-only and never changes selection or feasibility.
  std::size_t visualized_sample_count{0U};
  // A non-zero value is a certified trajectory-speed floor. generateReference
  // applies it before the rollout so collision and vehicle-limit evaluation
  // use the same speed profile that is published to the controller.
  double minimum_speed_mps{0.0};
  Parameters nominal{};
  ParameterBounds bounds{};
  std::array<BaseReferencePoint, kMaximumReferencePoints> base_reference{};
  std::size_t base_reference_count{0U};
  std::array<StaticObstacle, kMaximumStaticObstacles> static_obstacles{};
  std::size_t static_obstacle_count{0U};
  std::array<DynamicObstacle, kMaximumDynamicObstacles> dynamic_obstacles{};
  std::size_t dynamic_obstacle_count{0U};
  // Explicit target in this immutable obstacle snapshot. Legacy/non-overtake
  // requests leave it unset and retain cost-only selection.
  std::size_t overtake_target_index{std::numeric_limits<std::size_t>::max()};
  // Brain requests (including hold/fallback) supply the immutable global
  // Reference. Standalone legacy requests retain their Frenet-only contract.
  std::shared_ptr<const ReferencePoseIndex> world_reference{};
  std::shared_ptr<const PrecomputedWallLines> precomputed_wall_lines{};
  bool connect_to_wall_line{false};
  double wall_line_origin_station_m{0.};
  // Common global-road opportunity, in metres ahead of the measured start.
  // A negative entry disables the terminal opportunity value.
  double passing_entry_m{-1.0};
  double passing_exit_m{-1.0};
  double passing_speed_mps{0.0};
  std::size_t leader_opportunity_index{std::numeric_limits<std::size_t>::max()};
  double leader_passing_entry_m{-1.0}, leader_passing_exit_m{-1.0};
  double leader_preparation_m{-1.0}, leader_pass_time_sec{-1.0};
  double leader_pass_distance_m{-1.0};
  std::shared_ptr<const PassingRoadProfile> leader_passing_road{};
  // Previous-lap timing guides distant opportunities; collision prediction
  // remains the current-motion prediction in dynamic_obstacles.
  std::shared_ptr<const opponent_prediction::Prediction> leader_passing_prediction{};
  std::shared_ptr<const PassingPreparationPlan> passing_preparation{};
  std::shared_ptr<const OtLaneEntryPlan> ot_lane_entry{};
  // Ordered remaining accepted Cartesian path, s = arc from measured start.
  // Not a single-valued d(Reference s) field: that projection may fold.
  // Shared immutable storage avoids copying it into every proposal/work item.
  std::shared_ptr<const TemporaryReference> execution_history_field{};
  // Environment-specific Cartesian checks (occupancy-grid footprint and
  // oriented opponent clearance) run for every feasible MPPI sample before
  // it can contribute to the weighted update or become the selected path.
  PathConstraintValidator path_constraint_validator{};
  // Explicit opt-in: this callback depends only on fixed path geometry and
  // the immutable request snapshot, never on the proposed speed fields.
  bool path_constraint_geometry_only{false};
  // Long route suffixes are guidance, not an immediate execution certificate.
  // Validate the prefix consumed by the rollout, preview and braking reserve.
  bool path_constraint_execution_prefix{false};
  RolloutConstraintValidator rollout_constraint_validator{};
};

struct TemporaryReferencePoint {
  double s_m{0.0};
  double d_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature_1pm{0.0};
  double speed_mps{0.0};
  double uncapped_speed_mps{0.0}; // Diagnostic only; before curvature cap.
  // Station on the immutable accepted Cartesian source, not current local s.
  double source_s_m{0.0};
};

struct TemporaryReference {
  std::array<TemporaryReferencePoint, kMaximumReferencePoints> points{};
  std::size_t count{0U};
  // Lateral departure on the immutable source arc. Infinity keeps following.
  double overtake_start_source_s_m{std::numeric_limits<double>::infinity()};
  // First unchanged wall-line sample after the measured Cartesian connector.
  std::size_t wall_connection_end_index{0U};
  std::size_t front_merge_end_index{0U};
};

struct Evaluation {
  double merge_completion_time_sec{std::numeric_limits<double>::infinity()};
  double merge_braking_mps{0.};
  double merge_alongside_clearance_m{0.};
  bool valid{false};
  RejectReason reject_reason{RejectReason::INVALID_INPUT};
  double cost{std::numeric_limits<double>::infinity()};
  // Nominal full-body passage sustained through rollout end; infinity means
  // no demonstrated passage. Only populated after all validators succeed.
  double overtake_time_sec{std::numeric_limits<double>::infinity()};
  // progress, clearance, reference, curvature, PP, steering, pass separation,
  // terminal tracking, control smoothness, control change, parameter change,
  // approach speed (mean squared excess in m/s, weighted).
  std::array<double, 15U> cost_terms{}; // + passing progress, opportunity, OT entry speed
  // Attribution of the existing minimum-clearance cost, not additive guards.
  std::array<double, 3U> clearance_cost_sources{}; // boundary, static, dynamic
  // Maximum step penalty witness: step,x,y,boundary gap,static gap,dynamic
  // gap,dynamic index,weighted penalty. Unavailable distances remain infinite.
  std::array<double, 8U> clearance_peak{};
  // Diagnostic quadrature attribution: history lateral in/out, history speed
  // in/out, speed gradient in/out. Shared full-field denominators are retained.
  std::array<double, 6U> field_cost_split{};
  double field_terminal_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double field_history_coverage_m{0.0};
  double field_gradient_coverage_m{0.0};
  double terminal_s_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_d_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_reference_d_m{std::numeric_limits<double>::quiet_NaN()};
  double progress_m{0.0};
  double validated_reference_distance_m{0.0};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_ttc_sec{std::numeric_limits<double>::infinity()};
  double maximum_cross_track_error_m{0.0};
  double maximum_lateral_acceleration_mps2{0.0};
  // World-frame states traced by the delayed Pure Pursuit execution model.
  // These make the certified path, not only the generated Reference, visible
  // to diagnostics and RViz.
  std::array<RolloutState, kMaximumHorizonSteps> predicted_rollout{};
  std::size_t predicted_rollout_count{0U};
  std::size_t reject_step_index{std::numeric_limits<std::size_t>::max()};
  std::size_t reject_reference_index{std::numeric_limits<std::size_t>::max()};
  double reject_s_m{std::numeric_limits<double>::quiet_NaN()};
  double reject_d_m{std::numeric_limits<double>::quiet_NaN()};
  // A witness from the actual execution-time sweep, not a Reference arrival.
  double reject_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double reject_x_m{std::numeric_limits<double>::quiet_NaN()};
  double reject_y_m{std::numeric_limits<double>::quiet_NaN()};
  double reject_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double reject_reference_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  bool reject_dynamic_obstacle{false};
  // Shadow witnesses at the original rejection time; never used for selection.
  std::array<double, 4U> reject_nominal_envelope{};
  std::array<double, 4U> reject_position_envelope{};
  std::size_t reject_obstacle_index{std::numeric_limits<std::size_t>::max()};
  std::array<double, 4U> reject_obstacle_envelope{}; // s_min,s_max,d_min,d_max
  const char *reject_stage{"input"};
  double reject_minimum_d_m{std::numeric_limits<double>::quiet_NaN()};
  double reject_maximum_d_m{std::numeric_limits<double>::quiet_NaN()};
  double reject_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double reject_frenet_jacobian{std::numeric_limits<double>::quiet_NaN()};
};

inline bool betterEvaluation(const Evaluation &candidate, const Evaluation &incumbent) {
  return candidate.valid && (!incumbent.valid ? std::isfinite(candidate.cost) :
      betterCandidate(candidate.cost, candidate.overtake_time_sec,
                      incumbent.cost, incumbent.overtake_time_sec));
}

struct SpeedPairDiagnostic {
  // Snapshot for optional visualization; never used by optimization/selection.
  std::shared_ptr<const TemporaryReference> reference;
  // Retain the original evaluation, including uncapped terminal error and
  // time-stamped physical rollout; never recompute it for log output.
  Evaluation evaluation{};
  bool valid{false};
  RejectReason reason{RejectReason::INVALID_INPUT};
  double cost{0.0};
  decltype(Evaluation::cost_terms) terms{};
  double first_speed{0.0};
  double first_uncapped_speed{0.0};
  double geometry_delta_m{0.0};
};

struct PlanResult {
  std::array<SpeedPairDiagnostic, 4U> speed_pairs{};
  std::size_t speed_pair_count{0U};
  bool valid{false};
  RejectReason reject_reason{RejectReason::INVALID_INPUT};
  Parameters selected{};
  Parameters updated_mean{};
  ReferenceControlSequence selected_control_sequence{};
  ReferenceControlSequence updated_control_sequence{};
  TemporaryReference selected_reference{};
  Evaluation selected_evaluation{};
  Evaluation dominant_rejection{};
  std::array<TemporaryReference, kMaximumVisualizedSampleCount>
      visualized_sample_references{};
  std::array<Evaluation, kMaximumVisualizedSampleCount>
      visualized_sample_evaluations{};
  std::array<std::size_t, kMaximumVisualizedSampleCount>
      visualized_sample_indices{};
  std::size_t visualized_sample_count{0U};
  std::size_t valid_sample_count{0U};
  std::size_t evaluated_sample_count{0U};
  std::size_t duplicate_sample_count{0U};
  double effective_sample_size{0.0};
  double best_raw_cost{std::numeric_limits<double>::infinity()};
  double updated_mean_cost{std::numeric_limits<double>::infinity()};
  std::array<std::size_t, static_cast<std::size_t>(RejectReason::COUNT)>
      reject_counts{};
  double elapsed_ms{0.0};
  std::uint64_t generation{0U};
};

struct Scratch {
  std::array<std::array<double, kParameterCount>, kMaximumSampleCount> noise{};
  std::array<double, kMaximumSampleCount> costs{};
  std::array<double, kMaximumSampleCount> overtake_times{};
  std::array<bool, kMaximumSampleCount> valid{};
  std::array<std::array<double, kMaximumControlKnotCount>, kMaximumSampleCount>
      lateral_control_noise{};
  std::array<std::array<double, kMaximumControlKnotCount>, kMaximumSampleCount>
      speed_control_noise{};
  TemporaryReference candidate_reference{};
  TemporaryReference updated_reference{};
  TemporaryReference best_raw_reference{};
};

class ReferenceSpaceMppiPlanner {
public:
  explicit ReferenceSpaceMppiPlanner(Config config = Config{});

  const Config &config() const { return config_; }
  const char *validateConfig() const;
  void reset();
  PlanResult plan(const PlanRequest &request, Scratch *scratch);

  // Validate exactly this supplied geometry/speed field with the same closed-
  // loop dynamics and time-domain collision checks used by optimization.
  // Does not generate, clamp, splice, or update optimizer state.
  Evaluation validateReference(const TemporaryReference &reference,
                               const PlanRequest &request) const;

  // Adoption boundary: preserve the supplied Cartesian geometry/speeds,
  // reconstruct ordered Cartesian arc separately from Reference-relative
  // cost fields, and reject rollout beyond an exhausted reference tail.
  Evaluation evaluateExecutionReference(const TemporaryReference &reference,
                                        const PlanRequest &request) const;
  // The callback owns an immutable request/path snapshot. Only speed fields of
  // its argument are used; geometry remains that of reference for this search.
  std::function<Evaluation(const TemporaryReference &)> makeExecutionSpeedEvaluator(
      const TemporaryReference &reference, const PlanRequest &request) const;
  void accept(const PlanRequest &request, const PlanResult &result);

  static double quinticBlend(double normalized_s);
  static double multiPointLateralBlend(double normalized_s,
                                       double near_control_scale,
                                       double far_control_scale);
  static double activeManeuverLength(Phase phase, double shift_length_m,
                                     double merge_length_m);
  static const char *toString(RejectReason reason);

private:
  struct PreparedExecutionGeometry {
    std::array<double, kMaximumReferencePoints> preview_curvatures{};
    double curvature_cost{0.0};
    double wall_line_cost{0.0};
    std::shared_ptr<const BaseProjectionIndex> base_projection;
  };
  struct ExecutionValidators {
    PathConstraintValidator path;
    RolloutConstraintValidator rollout;
  };
  double spatialCurvatureCost(const TemporaryReference &reference,
      const PlanRequest &request) const;
  Evaluation evaluateOrderedExecutionReference(const TemporaryReference &reference,
      const PlanRequest &request, const TemporaryReference *field,
      const PreparedExecutionGeometry *geometry = nullptr,
      const ExecutionValidators *validators = nullptr) const;
  ReferenceControlSequence stagedSpeedProfile(
      const PlanRequest &request, const Parameters &parameters,
      ReferenceControlSequence control, std::size_t sample) const;
  friend struct PlannerEvaluationTestAccess;
  std::size_t horizonSteps(const PlanRequest &request) const {
    return request.horizon_steps_override == 0U ? config_.horizon_steps
                                              : request.horizon_steps_override;
  }
  Parameters project(const Parameters &parameters,
                     const ParameterBounds &bounds, int side) const;
  bool generateReference(const Parameters &parameters,
                         const PlanRequest &request,
                         const ReferenceControlSequence *control_sequence,
                         TemporaryReference *reference,
                         Evaluation *evaluation) const;
  Evaluation evaluate(const Parameters &parameters, const PlanRequest &request,
                      const ReferenceControlSequence *control_sequence,
                      TemporaryReference *reference) const;
  Evaluation evaluateReference(const Parameters &parameters,
                               const PlanRequest &request,
                               const ReferenceControlSequence *control_sequence,
                               const TemporaryReference *reference,
                               const TemporaryReference *field = nullptr,
                               const PreparedExecutionGeometry *geometry = nullptr,
                               const ExecutionValidators *validators = nullptr) const;
  bool validateExecutionSegment(const RolloutState &previous,
                                const RolloutState &state,
                                const PlanRequest &request, std::size_t step,
                                Evaluation *evaluation,
                                const BaseProjectionIndex *projection = nullptr) const;
  bool finalSweptValidate(const TemporaryReference &reference,
                          const PlanRequest &request,
                          Evaluation *evaluation,
                          const BaseProjectionIndex *projection = nullptr) const;
  void generateNoise(const PlanRequest &request, std::size_t sample_count,
                     Scratch *scratch) const;
  void generateControlNoise(const PlanRequest &request,
                            std::size_t sample_count, Scratch *scratch) const;
  ReferenceControlSequence
  controlSequenceMean(const PlanRequest &request) const;
  ReferenceControlSequence
  projectControlSequence(const ReferenceControlSequence &sequence) const;

  Config config_{};
  Parameters warm_mean_{};
  bool warm_mean_valid_{false};
  int warm_side_{0};
  Phase warm_phase_{Phase::PREPARE};
  ReferenceControlSequence warm_control_sequence_{};
  bool warm_control_sequence_valid_{false};
  double warm_stamp_sec_{0.0};
  double warm_control_horizon_m_{1.0};
  double warm_anchor_s_m_{0.0};
  std::uint64_t warm_semantic_key_{0U};
  // Previous immutable Reference, for measured spatial transport across bends
  // and loop seams. No v*dt estimate and no arbitrary 0.5 s clipping.
  std::array<BaseReferencePoint, kMaximumReferencePoints> warm_base_reference_{};
  std::size_t warm_base_reference_count_{0U};
  double warm_speed_horizon_m_{1.0};
};

} // namespace reference_space_mppi_planner::mppi
