#include "reference_space_mppi_planner/batch_optimizer.hpp"
#include "reference_space_mppi_planner/continuous_prefix.hpp"
#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include "reference_space_mppi_planner/reference_geometry.hpp"
#include "reference_space_mppi_planner/generation_freshness.hpp"
#include "reference_space_mppi_planner/execution_revision.hpp"
#include "reference_space_mppi_planner/lateral_line_library.hpp"
#include "reference_space_mppi_planner/lateral_sampling.hpp"
#include "reference_space_mppi_planner/local_horizon.hpp"
#include "reference_space_mppi_planner/local_envelope_diagnostic.hpp"
#include "reference_space_mppi_planner/collision_uncertainty.hpp"
#include "reference_space_mppi_planner/opponent_prediction.hpp"
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/side_selection.hpp"
#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include "reference_space_mppi_planner/execution_speed_optimizer.hpp"
#include "reference_space_mppi_planner/longitudinal_planner.hpp"
#include "reference_space_mppi_planner/entry_connector.hpp"
#include "reference_space_mppi_planner/sequenced_output.hpp"
#include "reference_space_mppi_planner/static_wall_map.hpp"
#include "reference_space_mppi_planner/steering_delay.hpp"
#include "reference_space_mppi_planner/recent_motion_prediction.hpp"
#include "reference_space_mppi_planner/online_motion_prediction.hpp"
#include "reference_space_mppi_planner/collection_motion.hpp"
#include "reference_space_mppi_planner/leader_lap_prediction.hpp"
#include "reference_space_mppi_planner/leader_passing_opportunity.hpp"
#include "reference_space_mppi_planner/passing_opportunity_markers.hpp"
#include "reference_space_mppi_planner/passing_point_forbidden_markers.hpp"
#include "reference_space_mppi_planner/passing_preparation_planner.hpp"
#include "reference_space_mppi_planner/preparation_boost.hpp"
#include "reference_space_mppi_planner/ot_lane_entry_planner.hpp"
#include "reference_space_mppi_planner/target_selection.hpp"
#include "reference_space_mppi_planner/driving_fsm.hpp"
#include "reference_space_mppi_planner/maneuver_policy.hpp"
#include "reference_space_mppi_planner/trajectory_projection.hpp"
#include "simple_pure_pursuit/steering_actuator_model.hpp"
#include "simple_pure_pursuit/rotation_prediction_message.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_direct_trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace reference_space_mppi_planner {
namespace {

using Command = multi_purpose_mpc_ros_msgs::msg::StateLatticeDirectTrajectory;
using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Mppi = mppi::ReferenceSpaceMppiPlanner;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
constexpr std::size_t kSteeringCommandHistoryCapacity = 64U;
constexpr std::size_t kBrainTemplateLineCount = mppi::kMaximumBatchCandidateCount;
static_assert(kBrainTemplateLineCount >= lateral_line_library::kLineCount);

bool isManeuverMode(const std::string &mode) {
  return mode == "OVERTAKE_LEFT" || mode == "OVERTAKE_RIGHT" ||
         mode == "SIDE_BY_SIDE_KEEP";
}

std::uint64_t maneuverSemanticKey(mppi::Phase phase, DrivingMode mode=DrivingMode::AVOID) {
  return 3U*static_cast<std::uint64_t>(phase)+static_cast<std::uint64_t>(mode)+1U;
}

double distance(const geometry_msgs::msg::Point &a,
                const geometry_msgs::msg::Point &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct WorkItem {
  DrivingMode maneuver{DrivingMode::AVOID};
  Command command;
  nav_msgs::msg::Odometry odometry;
  nav_msgs::msg::OccupancyGrid::SharedPtr wall_map;
  mppi::PlanRequest request;
  std::array<std::size_t, mppi::kMaximumReferencePoints> source_indices{};
  double certified_half_width_m{0.0};
  bool brain_owned{false};
  double requested_horizon_m{0.0};
  double actual_horizon_m{0.0};
  bool reference_capacity_limited{false};
  int marker_id{0};
  int template_slot{-1};
};

struct ObservedVehicle {
  mppi::PriorLapMatch prior_lap_match{};
  std::shared_ptr<const opponent_prediction::Prediction> leader_lap_prediction{};
  bool matched_prior_lap_execution{false};
  std::string id;
  double x_m{0.0};
  double y_m{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double sigma_x_m{0.0};
  double sigma_y_m{0.0};
  double stamp_sec{0.0};
  double receive_sec{0.0};
  double source_stamp_sec{0.0};
  std::uint64_t input_sequence{0};
  std::shared_ptr<const std::vector<opponent_prediction::PositionObservation>> history{};
  std::shared_ptr<const opponent_prediction::Prediction> prediction{};
  std::shared_ptr<const opponent_prediction::OnlineMotionLearner> online_motion{};
};

struct WorkBatch {
  DrivingMode maneuver{DrivingMode::AVOID};
  std::string maneuver_target_id;
  bool allow_continuation{true};
  std::uint64_t driving_revision{0};
  std::uint64_t recovery_epoch{0};
  std::uint64_t input_sequence{0};
  std::chrono::steady_clock::time_point captured_at{};
  std::chrono::steady_clock::time_point queued_at{};
  std::uint64_t execution_revision{0};
  Command fallback;
  std::optional<Command> brain_hold;
  std::optional<Command> ordinary_hold;
  std::uint64_t reference_revision{0};
  std::shared_ptr<const mppi::TemporaryReference> speed_profile;
  std::uint64_t speed_generation{0};
  Trajectory::SharedPtr global_reference;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr recorded_wall_map;
  std::vector<WorkItem> candidates;
  std::vector<ObservedVehicle> opponents;
  mppi::PathConstraintValidator fallback_path_constraint_validator;
  std::size_t candidate_count{0U};
  bool brain_owned{false};
  std::uint64_t semantic_key{0U};
  int preferred_side{0};
};

using PathProjection = mppi::PathProjection;

struct FollowVehicle {
  ObservedVehicle vehicle;
  PathProjection projection;
  double relative_s_m{0.0};
  double longitudinal_speed_mps{0.0};
  double body_gap_m{std::numeric_limits<double>::quiet_NaN()};
};

struct BrainInputs {
  std::shared_ptr<const PrecomputedWallLines> wall_lines;
  std::optional<FollowVehicle> leading;
  std::string active_target_id;
  std::string prediction_target_id;
  std::shared_ptr<const mppi::PassingPreparationPlan> preparation;
  std::vector<std::shared_ptr<const mppi::PassingPreparationPlan>> preparation_candidates;
  std::shared_ptr<const mppi::OtLaneEntryPlan> ot_lane_entry;
  std::shared_ptr<const mppi::PassingPreparationPlan> active_preparation;
  std::string leader_vehicle_id;
  simple_pure_pursuit::RotationPredictionSnapshot rotation_prediction;
  std::shared_ptr<const mppi::TemporaryReference> prior_execution_reference;
  double reference_activation_delay_sec{0.};
  std::uint64_t recovery_epoch{0};
  bool recovery_active{false};
  DrivingFsm driving_fsm;
  std::uint64_t input_sequence{0};
  std::uint64_t odometry_sequence{0};
  std::uint64_t steering_sequence{0};
  std::uint64_t control_sequence{0};
  double steering_stamp_sec{-1.0};
  double steering_receive_sec{-1.0};
  std::chrono::steady_clock::time_point captured_at{};
  std::uint64_t execution_revision{0};
  std::optional<nav_msgs::msg::Odometry> odometry;
  Trajectory::SharedPtr base_reference;
  Trajectory::SharedPtr requested_reference;
  std::uint64_t reference_revision{0};
  std::uint64_t requested_reference_revision{0};
  std::uint64_t vehicle_observation_sequence{0};
  std::shared_ptr<const Command> published_command;
  std::optional<Command> ordinary_hold;
  bool reference_transition{false};
  bool alongside{false};
  bool alongside_unknown{false};
  nav_msgs::msg::OccupancyGrid::SharedPtr wall_map;
  std::shared_ptr<const OccupancyGridWallIndex> wall_index;
  std::vector<ObservedVehicle> opponents;
  double steering_rad{0.0};
  bool steering_fresh{false};
  mppi::PendingSteeringTargets<mppi::kMaximumHorizonSteps>
      pending_steering_targets;
  int preferred_side{0};
  std::optional<Command> active_command;
  std::shared_ptr<const mppi::TemporaryReference> speed_profile;
  std::uint64_t speed_generation{0};
};

} // namespace

class ReferenceSpaceMppiNode final : public rclcpp::Node {
public:
  ReferenceSpaceMppiNode() : Node("reference_space_mppi_planner") {
    mppi::Config config;
    config.enabled = declare_parameter<bool>("enabled", false);
    shadow_only_ = declare_parameter<bool>("shadow_only", false);
    config.shadow_only = shadow_only_;
    config.allow_reference_switch = !shadow_only_;
    config.collision_only_rejection =
        declare_parameter<bool>("collision_only_rejection", false);
    config.sample_count = static_cast<std::size_t>(std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("sample_count", 128)));
    config.horizon_steps = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("horizon_steps", 60)));
    config.dt_sec = declare_parameter<double>("dt_sec", 0.05);
    config.temperature = declare_parameter<double>("temperature", 1.0);
    config.update_gain = declare_parameter<double>("update_gain", 0.7);
    config.deterministic_seed = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, declare_parameter<std::int64_t>(
                                      "deterministic_seed", 20260902)));
    config.minimum_valid_count =
        static_cast<std::size_t>(std::max<std::int64_t>(
            1, declare_parameter<std::int64_t>("minimum_valid_count", 1)));
    config.minimum_valid_ratio =
        declare_parameter<double>("minimum_valid_ratio", 0.0);
    config.cost_progress_weight =
        declare_parameter<double>("brain.cost_progress_weight", 1.0);
    config.cost_terminal_lateral_weight =
        declare_parameter<double>("brain.cost_terminal_lateral_weight", 0.0);
    config.maximum_reference_segment_m =
        declare_parameter<double>("maximum_reference_segment_m", 1.50);
    config.wheel_base_m = declare_parameter<double>("wheel_base_m", 1.087);
    config.lookahead_gain = declare_parameter<double>("lookahead_gain", 0.20);
    config.lookahead_min_distance_m =
        declare_parameter<double>("lookahead_min_distance_m", 2.0);
    config.curvature_lookahead_min_distance_m =
        declare_parameter<double>("curvature_lookahead_min_distance_m", 2.0);
    config.actual_lookahead_distance_blend =
        declare_parameter<double>("actual_lookahead_distance_blend", 1.0);
    config.continuous_preview_interpolation_enabled = declare_parameter<bool>(
        "continuous_preview_interpolation_enabled", true);
    config.steering_gain = declare_parameter<double>("steering_gain", 1.45);
    config.curvature_feedforward_enabled =
        declare_parameter<bool>("curvature_feedforward_enabled", true);
    config.curvature_feedforward_gain =
        declare_parameter<double>("curvature_feedforward_gain", 0.60);
    // Keep the three controller-geometry mechanisms coherent in production;
    // the core retains individual switches for controlled regression tests.
    const bool cma_prediction_geometry = declare_parameter<bool>(
        "prediction.cma_controller_geometry_enabled", true);
    config.compare_cma_delay_compensation = cma_prediction_geometry;
    config.compare_cma_preview_feedforward = cma_prediction_geometry;
    config.cma_lookahead_curvature_enabled = cma_prediction_geometry;
    config.cma_curvature_preview_distance_m = declare_parameter<double>(
        "prediction.cma_curvature_preview_distance_m", 21.422826964417297);
    config.steering_control_delay_sec =
        declare_parameter<double>("steering_control_delay_sec", 0.20);
    config.steering_time_constant_sec =
        declare_parameter<double>("steering_time_constant_sec", 0.30);
    config.awsim_vehicle_response_enabled =
        declare_parameter<bool>("prediction.awsim_vehicle_response_enabled", false);
    config.yaw_response_time_constant_sec =
        declare_parameter<double>("prediction.yaw_response_time_constant_sec", 0.10);
    config.cma_prediction_delay_sec =
        declare_parameter<double>("prediction.cma_control_delay_sec", 0.20);
    config.cma_prediction_steering_time_constant_sec =
        declare_parameter<double>("prediction.cma_steering_time_constant_sec", 0.30);
    config.steering_command_to_tire_angle_ratio =
        declare_parameter<double>("steering_command_to_tire_angle_ratio", 0.60);
    config.maximum_steering_angle_rad = declare_parameter<double>(
        "maximum_steering_command_angle_rad", 0.5235987755982988);
    config.maximum_tire_steering_angle_rad = declare_parameter<double>(
        "maximum_tire_steering_angle_rad", 0.3141592653589793);
    config.maximum_steering_rate_radps =
        declare_parameter<double>("maximum_steering_command_rate_radps", 4.0);
    config.physical_tire_steering_rate_radps =
        declare_parameter<double>("physical_tire_steering_rate_radps", 0.0);
    config.steering_demand_acceleration_hold_enabled = declare_parameter<bool>(
        "steering_demand_acceleration_hold_enabled", true);
    config.steering_acceleration_hold_minimum_speed_mps =
        declare_parameter<double>(
            "steering_acceleration_hold_minimum_speed_mps", 3.0);
    config.steering_acceleration_hold_minimum_tire_angle_rad =
        declare_parameter<double>(
            "steering_acceleration_hold_minimum_tire_angle_rad", 0.15);
    config.steering_acceleration_hold_tracking_error_rad =
        declare_parameter<double>(
            "steering_acceleration_hold_tracking_error_rad", 0.08);
    config.steering_acceleration_hold_maximum_acceleration_mps2 =
        declare_parameter<double>(
            "steering_acceleration_hold_maximum_acceleration_mps2", 0.0);
    config.longitudinal_planning_enabled =
        declare_parameter<bool>("brain.longitudinal_planning_enabled", false);
    config.maximum_acceleration_mps2 =
        declare_parameter<double>("maximum_acceleration_mps2", 2.0);
    config.maximum_deceleration_mps2 =
        declare_parameter<double>("maximum_deceleration_mps2", 2.0);
    config.speed_proportional_gain =
        declare_parameter<double>("speed_proportional_gain", 3.0);
    config.acceleration_time_constant_sec =
        declare_parameter<double>("acceleration_time_constant_sec", 0.25);
    config.maximum_lateral_acceleration_mps2 =
        declare_parameter<double>("maximum_lateral_acceleration_mps2", 10.0);
    config.maximum_cross_track_error_m =
        declare_parameter<double>("maximum_cross_track_error_m", 0.8);
    config.clearance_target_m =
        declare_parameter<double>("clearance_target_m", 0.35);
    config.obstacle_longitudinal_inflation_m =
        declare_parameter<double>("obstacle_longitudinal_inflation_m", 1.10);
    config.obstacle_lateral_inflation_m =
        declare_parameter<double>("obstacle_lateral_inflation_m", 0.90);
    config.cost_pass_separation_weight =
        declare_parameter<double>("cost_pass_separation_weight", 50.0);
    config.cost_wall_line_weight =
        declare_parameter<double>("brain.cost_wall_line_weight", 0.0);
    brain_control_sequence_sampling_enabled_ = declare_parameter<bool>(
        "brain.control_sequence_sampling_enabled", true);
    const auto control_knot_count =
        declare_parameter<std::int64_t>("brain.control_knot_count", 8);
    if (control_knot_count < 2 ||
        control_knot_count >
            static_cast<std::int64_t>(mppi::kMaximumControlKnotCount)) {
      throw std::invalid_argument(
          "brain.control_knot_count must be within [2, 12]");
    }
    config.control_knot_count = static_cast<std::size_t>(control_knot_count);
    config.control_lateral_std_m =
        declare_parameter<double>("brain.control_lateral_std_m", 0.20);
    config.control_lateral_reference_spacing_m = declare_parameter<double>(
        "brain.control_lateral_reference_spacing_m", 0.0);
    config.control_speed_scale_std =
        declare_parameter<double>("brain.control_speed_scale_std", 0.04);
    config.control_noise_correlation =
        declare_parameter<double>("brain.control_noise_correlation", 0.75);
    config.control_maximum_lateral_adjustment_m = declare_parameter<double>(
        "brain.control_maximum_lateral_adjustment_m", 0.45);
    config.control_maximum_speed_scale_adjustment = declare_parameter<double>(
        "brain.control_maximum_speed_scale_adjustment", 0.12);
    config.control_entry_ramp_m =
        declare_parameter<double>("brain.control_entry_ramp_m", 4.0);
    config.cost_control_smoothness_weight =
        declare_parameter<double>("brain.cost_control_smoothness_weight", 0.10);
    config.cost_control_change_weight =
        declare_parameter<double>("brain.cost_control_change_weight", 0.15);
    declare_parameter<bool>("brain.cost_spatial_diagnostics_enabled", false);
    brain_reference_geometry_knot_spacing_m_ =
        declare_parameter<double>("brain.reference_geometry_knot_spacing_m", 2.0);
    if (!(brain_reference_geometry_knot_spacing_m_ > 0.) ||
        !std::isfinite(brain_reference_geometry_knot_spacing_m_))
      throw std::invalid_argument("brain.reference_geometry_knot_spacing_m must be positive");
    config.history_lateral_cost_scale =
        declare_parameter<double>("brain.history_lateral_cost_scale", 1.0);
    tube_max_half_width_m_ =
        declare_parameter<double>("tube_max_half_width_m", 0.25);
    tube_min_half_width_m_ =
        declare_parameter<double>("tube_min_half_width_m", 0.04);
    nominal_lateral_refinement_m_ =
        declare_parameter<double>("nominal_lateral_refinement_m", 0.08);
    base_curvature_limit_1pm_ =
        declare_parameter<double>("base_curvature_limit_1pm", 1.0);
    output_reason_prefix_ =
        declare_parameter<std::string>("output_reason_prefix", "mppi_refined:");
    brain_mode_ = declare_parameter<bool>("brain_mode", false);
    own_vehicle_id_ = declare_parameter<std::string>("own_vehicle_id", "");
    brain_trigger_distance_m_ =
        declare_parameter<double>("brain.trigger_distance_m", 25.0);
    leader_lap_prediction_enabled_ =
        declare_parameter<bool>("brain.leader_lap_prediction_enabled", false);
    passing_preparation_enabled_ = declare_parameter<bool>("brain.passing_preparation_enabled", false);
    passing_point_display_enabled_=declare_parameter<bool>("visualization.passing_point_forbidden.enabled",false);
    passing_point_areas_enabled_=declare_parameter<bool>("brain.passing_point_areas_enabled",false);
    passing_point_display_config_.corner_margin_m=declare_parameter<double>(
        "visualization.passing_point_forbidden.corner_margin_m",5.);
    passing_point_display_config_.straight_start_xy=declare_parameter<std::vector<double>>(
        "visualization.passing_point_forbidden.straight_start_xy",std::vector<double>{});
    passing_point_display_config_.corner_entry_xy=declare_parameter<std::vector<double>>(
        "visualization.passing_point_forbidden.corner_entry_xy",std::vector<double>{});
    preparation_boost_config_.enabled =
        declare_parameter<bool>("brain.preparation_boost_enabled", false);
    preparation_boost_config_.start_waypoint =
        declare_parameter<double>("brain.preparation_boost_start_wp", 0.0);
    preparation_boost_config_.end_waypoint =
        declare_parameter<double>("brain.preparation_boost_end_wp", 60.0);
    preparation_boost_config_.first_lap = {{
        declare_parameter<std::int64_t>("brain.preparation_boost_first_lap_min", 1),
        declare_parameter<std::int64_t>("brain.preparation_boost_second_lap_min", 5)}};
    preparation_boost_config_.last_lap = {{
        declare_parameter<std::int64_t>("brain.preparation_boost_first_lap_max", 4),
        declare_parameter<std::int64_t>("brain.preparation_boost_second_lap_max", 6)}};
    if (!std::isfinite(preparation_boost_config_.start_waypoint) ||
        !std::isfinite(preparation_boost_config_.end_waypoint) ||
        preparation_boost_config_.start_waypoint < 0. ||
        preparation_boost_config_.end_waypoint < preparation_boost_config_.start_waypoint ||
        preparation_boost_config_.first_lap[0] < 1 ||
        preparation_boost_config_.last_lap[0] < preparation_boost_config_.first_lap[0] ||
        preparation_boost_config_.first_lap[1] <= preparation_boost_config_.last_lap[0] ||
        preparation_boost_config_.last_lap[1] < preparation_boost_config_.first_lap[1])
      throw std::invalid_argument("Invalid preparation boost waypoint/lap intervals");
    ot_entry_config_.enabled=declare_parameter<bool>("brain.ot_entry_enabled",false);
    ot_entry_config_.entry_x_m=declare_parameter<double>("brain.ot_entry_x_m",0.);
    ot_entry_config_.entry_y_m=declare_parameter<double>("brain.ot_entry_y_m",0.);
    ot_entry_config_.exit_x_m=declare_parameter<double>("brain.ot_exit_x_m",0.);
    ot_entry_config_.exit_y_m=declare_parameter<double>("brain.ot_exit_y_m",0.);
    ot_entry_config_.target_speed_mps=declare_parameter<double>("brain.ot_entry_speed_kmph",30.)/3.6;
    ot_entry_config_.cost_weight=declare_parameter<double>("brain.ot_entry_speed_weight",1.);
    front_merge_attack_enabled_ = declare_parameter<bool>("brain.front_merge_attack_enabled", false);
    multiple_passing_points_enabled_=declare_parameter<bool>("brain.multiple_passing_points_enabled",false);
    preparation_config_.match_window_sec = declare_parameter<double>("brain.preparation_match_window_sec", 1.0);
    preparation_config_.maximum_position_error_m = declare_parameter<double>("brain.preparation_match_position_m", 0.5);
    preparation_config_.maximum_heading_error_rad = declare_parameter<double>("brain.preparation_match_heading_rad", 0.25);
    preparation_config_.maximum_pace_error_ratio = declare_parameter<double>("brain.preparation_match_pace_ratio", 0.25);
    prior_lap_prediction_enabled_ =
        declare_parameter<bool>("brain.prior_lap_prediction_enabled", false);
    recent_motion_prediction_enabled_ =
        declare_parameter<bool>("brain.recent_motion_prediction_enabled", false);
    online_motion_prediction_enabled_ =
        declare_parameter<bool>("brain.online_motion_prediction_enabled", false);
    collection_motion_enabled_ =
        declare_parameter<bool>("brain.collection_motion_enabled", false);
    collection_avoidance_continuation_ =
        declare_parameter<bool>("brain.collection_avoidance_continuation", false);
    motion_persistence_.acceleration_sec =
        declare_parameter<double>("brain.prediction_acceleration_persistence_sec", 0.6);
    motion_persistence_.yaw_rate_sec =
        declare_parameter<double>("brain.prediction_yaw_rate_persistence_sec", 0.6);
    if (!std::isfinite(motion_persistence_.acceleration_sec) || motion_persistence_.acceleration_sec < 0. ||
        !std::isfinite(motion_persistence_.yaw_rate_sec) || motion_persistence_.yaw_rate_sec < 0.)
      throw std::invalid_argument("Opponent motion persistence must be finite and nonnegative");
    brain_target_path_conflict_margin_m_ =
        declare_parameter<double>("brain.target_path_conflict_margin_m", 0.15);
    brain_prediction_horizon_sec_ = std::max(
        0.1, declare_parameter<double>("brain.prediction_horizon_sec", 6.0));
    brain_pass_offset_m_ =
        declare_parameter<double>("brain.pass_offset_m", 1.85);
    brain_min_pass_offset_m_ =
        declare_parameter<double>("brain.min_pass_offset_m", 0.0);
    brain_side_switch_minimum_cost_improvement_ = declare_parameter<double>(
        "brain.side_switch_minimum_relative_improvement", 0.0);
    brain_sample_diagnostics_enabled_ = declare_parameter<bool>(
        "brain.sample_diagnostics_enabled", false);
    decision_markers_enabled_ = declare_parameter<bool>(
        "brain.decision_markers_enabled", false);
    decision_slot_ = declare_parameter<int>("brain.decision_marker_slot", -1);
    decision_profile_ = declare_parameter<int>("brain.decision_marker_profile", 1);
    if(decision_slot_ < -1 || decision_slot_ >= static_cast<int>(kBrainTemplateLineCount) ||
       decision_profile_ < 0 || decision_profile_ > 4)
      throw std::invalid_argument("decision marker slot/profile out of range");
    if (!std::isfinite(brain_side_switch_minimum_cost_improvement_) ||
        brain_side_switch_minimum_cost_improvement_ < 0.0) {
      throw std::invalid_argument("side switch relative improvement must be finite and nonnegative");
    }
    const double legacy_switch = declare_parameter<double>(
        "brain.side_switch_minimum_cost_improvement", 0.0);
    if (legacy_switch != 0.0) {
      RCLCPP_WARN(get_logger(), "Legacy absolute side switch threshold is ignored; use brain.side_switch_minimum_relative_improvement");
    }
    const auto brain_samples_per_line =
        declare_parameter<std::int64_t>("brain.samples_per_line", 10);
    if (brain_samples_per_line < 4 ||
        brain_samples_per_line >
            static_cast<std::int64_t>(mppi::kMaximumSampleCount) ||
        brain_samples_per_line % 2 != 0) {
      throw std::invalid_argument(
          "brain.samples_per_line must be even and within [4, 128]");
    }
    brain_samples_per_line_ = static_cast<std::size_t>(brain_samples_per_line);
    const auto brain_visualized_samples_per_line =
        declare_parameter<std::int64_t>("brain.visualized_samples_per_line", 4);
    if (brain_visualized_samples_per_line < 0 ||
        brain_visualized_samples_per_line >
            static_cast<std::int64_t>(mppi::kMaximumVisualizedSampleCount)) {
      throw std::invalid_argument(
          "brain.visualized_samples_per_line must be within [0, 4]");
    }
    brain_visualized_samples_per_line_ =
        static_cast<std::size_t>(brain_visualized_samples_per_line);
    const bool parallel_batch_enabled =
        declare_parameter<bool>("brain.parallel_batch_enabled", true);
    brain_max_result_generation_lag_ = static_cast<std::size_t>(
        std::max<std::int64_t>(0, declare_parameter<std::int64_t>(
                                      "brain.max_result_generation_lag", 3)));
    brain_max_track_offset_m_ =
        declare_parameter<double>("brain.max_track_offset_m", 2.5);
    brain_footprint_radius_m_ =
        declare_parameter<double>("brain.footprint_radius_m", 0.65);
    brain_footprint_front_m_ =
        declare_parameter<double>("brain.footprint_front_m", 1.06);
    brain_footprint_rear_m_ =
        declare_parameter<double>("brain.footprint_rear_m", 1.10);
    auto wall_xy = declare_parameter<std::vector<double>>(
        "brain.wall_footprint_xy_m", std::vector<double>{});
    if (wall_xy.empty()) {
      wall_xy = {brain_footprint_front_m_, brain_footprint_radius_m_,
                 -brain_footprint_rear_m_, brain_footprint_radius_m_,
                 -brain_footprint_rear_m_, -brain_footprint_radius_m_,
                 brain_footprint_front_m_, -brain_footprint_radius_m_};
    }
    brain_wall_footprint_ = ConvexWallFootprint(wall_xy);
    brain_lateral_sample_step_m_ =
        declare_parameter<double>("brain.lateral_sample_step_m", 0.10);
    brain_cruise_speed_mps_ =
        declare_parameter<double>("brain.cruise_speed_mps", 10.0);
    config.follow_gap_m = declare_parameter<double>("brain.follow_gap_m", 5.0);
    config.overtake_follow_gap_m = declare_parameter<double>("brain.overtake_follow_gap_m", 2.0);
    driving_fsm_ = DrivingFsm(config.follow_gap_m,config.overtake_follow_gap_m);
    config.approach_time_sec = declare_parameter<double>("brain.approach_time_sec", 2.0);
    config.approach_at_follow_gap_only =
        declare_parameter<bool>("brain.approach_at_follow_gap_only", false);
    config.cost_approach_speed_weight =
        declare_parameter<double>("brain.cost_approach_speed_weight", 0.0);
    config.cost_passing_progress_weight =
        declare_parameter<double>("brain.cost_passing_progress_weight", 0.0);
    config.cost_passing_opportunity_weight =
        declare_parameter<double>("brain.cost_passing_opportunity_weight", 0.0);
    compare_return_continuation_ =
        declare_parameter<bool>("brain.compare_return_continuation", false);
    gentle_lateral_acceleration_mps2_ =
        declare_parameter<double>("brain.gentle_lateral_acceleration_mps2", 0.0);
    brain_overtake_speed_mps_ =
        declare_parameter<double>("brain.overtake_speed_mps", 10.0);
    brain_minimum_passing_speed_mps_ =
        declare_parameter<double>("brain.minimum_passing_speed_mps", 6.0);
    brain_minimum_passing_advantage_mps_ =
        declare_parameter<double>("brain.minimum_passing_advantage_mps", 0.8);
    brain_minimum_rolling_speed_mps_ =
        declare_parameter<double>("brain.minimum_rolling_speed_mps", 0.5);
    brain_minimum_proximity_speed_mps_ =
        declare_parameter<double>("brain.minimum_proximity_speed_mps", 10.0);
    brain_hold_minimum_opponent_clearance_m_ = declare_parameter<double>(
        "brain.hold_minimum_opponent_clearance_m", 0.0);
    brain_hold_revalidation_horizon_sec_ =
        declare_parameter<double>("brain.hold_revalidation_horizon_sec", 1.50);
    brain_input_timeout_sec_ =
        declare_parameter<double>("brain.input_timeout_sec", 0.5);
    brain_hard_clearance_m_ =
        declare_parameter<double>("brain.hard_clearance_m", 0.0);
    brain_reference_horizon_m_ =
        declare_parameter<double>("brain.reference_horizon_m", 60.0);
    local_minimum_m_ = declare_parameter<double>("brain.local_minimum_m", 20.0);
    local_maximum_m_ = declare_parameter<double>("brain.local_maximum_m", 30.0);
    local_lookahead_sec_ = declare_parameter<double>("brain.local_lookahead_sec", 3.0);
    reference_supply_sec_ = declare_parameter<double>("brain.reference_supply_sec", 1.0);
    if (!std::isfinite(local_minimum_m_) || !std::isfinite(local_maximum_m_) ||
        !std::isfinite(local_lookahead_sec_) || local_minimum_m_ < 15.0 ||
        local_maximum_m_ < local_minimum_m_ || local_lookahead_sec_ <= 0.0 ||
        !std::isfinite(reference_supply_sec_) || reference_supply_sec_ < 0.0) {
      throw std::invalid_argument("invalid brain local horizon configuration");
    }
    brain_internal_timer_enabled_ =
        declare_parameter<bool>("brain.internal_timer_enabled", false);
    brain_update_rate_hz_ =
        declare_parameter<double>("brain.update_rate_hz", 20.0);
    brain_static_wall_map_enabled_ =
        declare_parameter<bool>("brain.static_wall_map_enabled", false);
    config.vehicle_half_length_m =
        std::max(brain_footprint_front_m_, brain_footprint_rear_m_);
    config.vehicle_half_width_m = brain_footprint_radius_m_;
    const auto wall_map_package = declare_parameter<std::string>(
        "brain.wall_map_package", "reference_space_mppi_planner");
    const auto wall_map_yaml = declare_parameter<std::string>(
        "brain.wall_map_yaml", "config/awsim_wall_map/occupancy_grid_map.yaml");
    const auto wall_map_frame =
        declare_parameter<std::string>("brain.wall_map_frame", "map");
    enabled_ = config.enabled;
    config_ = config;
    if (declare_parameter<bool>("recovery_service_enabled", false)) {
      recovery_service_ = create_service<std_srvs::srv::SetBool>(
          "~/set_recovery_active",
          [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                 std_srvs::srv::SetBool::Response::SharedPtr response) {
            std::lock_guard<std::mutex> decision_lock(decision_mutex_);
            if (!brain_mode_) {
              response->success = false;
              response->message = "recovery requires standalone brain mode";
              return;
            }
            std::lock_guard<std::mutex> authority_lock(authority_mutex_);
            latest_semantic_key_.store(0U, std::memory_order_release);
            {
              std::lock_guard<std::mutex> input_lock(input_mutex_);
              recovery_active_ = request->data;
              ++recovery_epoch_;
              std::atomic_store(&published_execution_reference_,
                  std::shared_ptr<const mppi::TemporaryReference>{});
              std::atomic_store(&published_command_, std::shared_ptr<const Command>{});
              ordinary_hold_.reset();
              reference_transition_.reset();
              alongside_vehicle_ids_.clear();
              prediction_target_id_.clear();
              ++execution_revision_;
              clearBrainExecution();
              preferred_side_ = 0;
              driving_fsm_.reset();
            }
            {
              std::lock_guard<std::mutex> work_lock(work_mutex_);
              pending_batch_.reset();
            }
            // Warm starts are worker-owned. The next batch resets them using
            // this epoch; an in-flight old batch cannot commit after this reply.
            response->success = true;
            response->message = recovery_active_ ? "recovery owns control" : "fresh planning resumed";
            RCLCPP_INFO(get_logger(), "[MPPI_STUCK_RECOVERY_RESET] active=%d epoch=%lu",
                        recovery_active_, static_cast<unsigned long>(recovery_epoch_));
          });
    }
    left_planner_ = std::make_unique<Mppi>(config_);
    right_planner_ = std::make_unique<Mppi>(config_);
    batch_optimizer_ = std::make_unique<mppi::ReferenceSpaceMppiBatchOptimizer>(
        config_, parallel_batch_enabled);

    const auto reliable =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    if (brain_mode_ && preparation_boost_config_.enabled) {
      preparation_boost_pub_ =
          create_publisher<std_msgs::msg::Float32MultiArray>("/awsim/cmd", 10);
      preparation_boost_state_sub_ = create_subscription<std_msgs::msg::String>(
          "/awsim/state", rclcpp::QoS(10),
          [this](std_msgs::msg::String::ConstSharedPtr message) {
            receivePreparationBoostState(message->data);
          });
      preparation_boost_status_sub_ =
          create_subscription<std_msgs::msg::Float32MultiArray>(
              "/awsim/status", rclcpp::QoS(10),
              [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
                receivePreparationBoostStatus(*message);
              });
    }
    output_pub_ =
        create_publisher<Command>("output/trajectory_command", reliable);
    baseline_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/baseline", rclcpp::QoS(1));
    selected_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/selected", rclcpp::QoS(1).reliable().transient_local());
    candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/candidates", rclcpp::QoS(1).reliable().transient_local());
    opponent_predictions_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/opponent_predictions", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("debug/status", 10);
    decision_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/mppi/decision", rclcpp::QoS(1).reliable().transient_local());
    passing_opportunity_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/mppi/passing_opportunity", rclcpp::QoS(1).reliable().transient_local());
    passing_point_forbidden_pub_=create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/mppi/passing_point_forbidden",rclcpp::QoS(1).reliable().transient_local());
    wall_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "debug/wall_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    if (brain_mode_ && brain_static_wall_map_enabled_) {
      std::string error;
      try {
        const auto yaml_path =
            ament_index_cpp::get_package_share_directory(wall_map_package) +
            "/" + wall_map_yaml;
        auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        if (!loadStaticWallMap(yaml_path, wall_map_frame, map.get(), &error)) {
          throw std::runtime_error(error);
        }
        map->header.stamp = now();
        wall_map_ = map;
        wall_index_ = std::make_shared<OccupancyGridWallIndex>(map);
        wall_map_pub_->publish(*map);
        RCLCPP_INFO(get_logger(),
                    "MPPI static wall map loaded: %ux%u resolution=%.3f",
                    map->info.width, map->info.height, map->info.resolution);
      } catch (const std::exception &exception) {
        RCLCPP_ERROR(get_logger(), "MPPI static wall map unavailable: %s",
                     exception.what());
      }
    }
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "input/odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          odometry_ = std::move(*message);
          odometry_sequence_ = ++input_event_sequence_;
        });
    leader_sub_ = create_subscription<std_msgs::msg::String>(
        "/planning/reference/leader_vehicle_id", rclcpp::QoS(10),
        [this](std_msgs::msg::String::SharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          leader_vehicle_id_ = message->data;
          leader_receive_sec_ = now().seconds();
        });
    reference_sub_ = create_subscription<Trajectory>(
        "input/base_reference",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](Trajectory::SharedPtr message) { receiveBaseReference(std::move(message)); });
    reference_contract_pub_ = create_publisher<std_msgs::msg::String>(
        "~/reference_contract", rclcpp::QoS(1).reliable().transient_local());
    wall_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "input/wall_map", rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
          auto index = std::make_shared<OccupancyGridWallIndex>(message);
          std::lock_guard<std::mutex> lock(input_mutex_);
          wall_map_ = std::move(message);
          wall_index_ = std::move(index);
        });
    steering_sub_ =
        create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            "input/steering", rclcpp::SensorDataQoS(),
            [this](autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr
                       message) {
              receiveSteeringReport(*message);
            });
    control_command_sub_ = create_subscription<ControlCommand>(
        "input/control_command", rclcpp::QoS(32).best_effort(),
        [this](ControlCommand::SharedPtr message) {
          const double receive_sec = now().seconds();
          double stamp_sec = rclcpp::Time(message->stamp).seconds();
          if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
            stamp_sec = receive_sec;
          }
          const double steering_rad = message->lateral.steering_tire_angle;
          if (!std::isfinite(stamp_sec) || !std::isfinite(steering_rad)) {
            return;
          }
          const bool record_input = get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool();
          std::lock_guard<std::mutex> lock(input_mutex_);
          control_sequence_ = ++input_event_sequence_;
          if (!steering_command_history_.empty() &&
              stamp_sec + 1.0e-6 < steering_command_history_.back().stamp_sec) {
            steering_command_history_.clear();
          }
          steering_command_history_.push_back(
              mppi::TimedSteeringCommand{stamp_sec, steering_rad, control_sequence_});
          const double history_window_sec =
              std::max(1.0, config_.steering_control_delay_sec + 0.5);
          while (!steering_command_history_.empty() &&
                 steering_command_history_.front().stamp_sec <
                     receive_sec - history_window_sec) {
            steering_command_history_.pop_front();
          }
          if (record_input) RCLCPP_INFO(get_logger(),
              "[MPPI_CONTROL_RECEIVED] sequence=%lu stamp=%.17g receive=%.17g steering=%.17g speed=%.17g acceleration=%.17g",
              static_cast<unsigned long>(control_sequence_), stamp_sec, receive_sec,
              steering_rad, static_cast<double>(message->longitudinal.speed),
              static_cast<double>(message->longitudinal.acceleration));
        });
    v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        "input/vehicle_positions", rclcpp::QoS(10).reliable(),
        [this](v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr message) {
          receiveVehiclePositions(*message);
        });
    command_sub_ = create_subscription<Command>(
        "input/trajectory_command", reliable,
        [this](Command::SharedPtr message) { receiveCommand(*message); });

    if (brain_mode_ && brain_internal_timer_enabled_) {
      if (!std::isfinite(brain_update_rate_hz_) ||
          brain_update_rate_hz_ <= 0.0) {
        brain_update_rate_hz_ = 20.0;
        RCLCPP_ERROR(get_logger(),
                     "invalid MPPI brain update rate; using 20 Hz");
      }
      const auto period =
          std::chrono::duration<double>(1.0 / brain_update_rate_hz_);
      brain_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this]() { runInternalBrainCycle(); });
    }

    rotation_prediction_sub_ =
        create_subscription<simple_pure_pursuit::RotationPredictionMessage>(
            "/mppi/internal/cma_rotation_prediction_state", rclcpp::QoS(8),
            [this](simple_pure_pursuit::RotationPredictionMessage::ConstSharedPtr msg) {
              auto snapshot = simple_pure_pursuit::rotationPredictionSnapshot(*msg);
              std::lock_guard<std::mutex> lock(input_mutex_);
              if (!rotation_prediction_history_.empty() &&
                  snapshot.stamp_sec < rotation_prediction_history_.back().stamp_sec)
                rotation_prediction_history_.clear();
              rotation_prediction_history_.push_back(snapshot);
              while (rotation_prediction_history_.size() > kSteeringCommandHistoryCapacity)
                rotation_prediction_history_.pop_front();
            });
    worker_ = std::thread([this]() { workerLoop(); });
    RCLCPP_INFO(get_logger(),
                "Reference-space MPPI ready: enabled=%s shadow_only=%s "
                "brain_mode=%s internal_timer=%s steering_delay=%.3fs; "
                "CMA PP receives the atomic output command",
                enabled_ ? "true" : "false", shadow_only_ ? "true" : "false",
                brain_mode_ ? "true" : "false",
                brain_internal_timer_enabled_ ? "true" : "false",
                config_.steering_control_delay_sec);
  }

  ~ReferenceSpaceMppiNode() override {
    {
      std::lock_guard<std::mutex> lock(work_mutex_);
      stopping_ = true;
    }
    work_cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  static bool sameReference(const Trajectory::SharedPtr &a,
                            const Trajectory::SharedPtr &b) {
    if (a == b) return true;
    return a && b && a->header.frame_id == b->header.frame_id &&
        a->points.size() == b->points.size() &&
        std::equal(a->points.begin(), a->points.end(), b->points.begin(),
            [](const auto &x, const auto &y) {
              return x.pose.position.x == y.pose.position.x &&
                     x.pose.position.y == y.pose.position.y;
            });
  }

  void receiveBaseReference(Trajectory::SharedPtr message) {
    if (!brain_mode_) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      commitBaseReference(message);
      // Refinement mode also consumes speed-only reference updates.
      base_reference_=requested_reference_=std::move(message);
      return;
    }
    if (brain_mode_ && message->points.size() >= 3U) {
      auto smoothed = std::make_shared<Trajectory>(*message);
      std::vector<ReferenceXY> xy;
      for (const auto &p : message->points)
        xy.push_back({p.pose.position.x, p.pose.position.y});
      const bool closed = std::hypot(xy.back()[0]-xy.front()[0],
                                    xy.back()[1]-xy.front()[1]) < 5.;
      const auto geometry = smoothReferenceGeometry(
          xy, closed, brain_reference_geometry_knot_spacing_m_);
      for (std::size_t i=0; i<geometry.size(); ++i) {
        auto &p=smoothed->points[i];
        p.pose.position.x=geometry[i][0];p.pose.position.y=geometry[i][1];
        const auto &a=geometry[i==0?0:i-1];
        const auto &b=geometry[std::min(i+1,geometry.size()-1)];
        p.pose.orientation=quaternionFromYaw(std::atan2(b[1]-a[1],b[0]-a[0]));
      }
      message=std::move(smoothed);
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!sameReference(requested_reference_, message)) {
      requested_reference_=std::move(message);
      ++requested_reference_revision_;
    }
    // The first route initializes the world. Later arrivals are requests only.
    if (!brain_mode_ || !base_reference_ || base_reference_->points.size()<3U)
      commitBaseReference(requested_reference_);
  }

  // Caller owns input_mutex_ and (after initialization) publication authority.
  void commitBaseReference(const Trajectory::SharedPtr &reference) {
    if (sameReference(base_reference_,reference)) return;
    for (auto &[id, vehicle] : observed_vehicles_) {
      (void)id;
      if (vehicle.history && reference && reference->points.size()>=3U) {
        auto history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>();
        const double length=trajectoryArcLength(*reference);
        for (const auto &p:*vehicle.history) {
          const auto projected=projectOnTrajectory(*reference,p.x,p.y);
          if (!projected.valid || !(length>0.)) { history->clear();break; }
          const double station=history->empty()?projected.s_m:
              history->back().station+std::remainder(projected.s_m-history->back().station,length);
          history->push_back({p.stamp,p.x,p.y,station});
        }
        vehicle.history=std::move(history);
      } else vehicle.history.reset();
      vehicle.online_motion.reset();
      vehicle.prediction.reset();
    }
    base_reference_=reference;
    ++reference_revision_;
    ++execution_revision_;
    latest_preparation_.reset();
    latest_ot_lane_entry_.reset();
    active_preparation_.reset();
  }

  void runInternalBrainCycle() {
    Trajectory::SharedPtr reference;
    bool inputs_ready = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      reference = base_reference_;
      inputs_ready = odometry_.has_value() && reference != nullptr &&
                     reference->points.size() >= 3U && wall_map_ != nullptr;
    }
    if (!inputs_ready)
      return;

    Command heartbeat;
    heartbeat.header = reference->header;
    heartbeat.header.stamp = now();
    heartbeat.generation =
        internal_generation_.fetch_add(1U, std::memory_order_relaxed) + 1U;
    heartbeat.owner_commit_id = 1U;
    heartbeat.geometry_revision = heartbeat.generation;
    heartbeat.valid_until_sec = now().seconds() + 0.20;
    heartbeat.mode = "FREE_RUN";
    heartbeat.emergency_stop = false;
    heartbeat.reason = "mppi_brain:internal_tick";
    heartbeat.trajectory.header = heartbeat.header;
    receiveCommand(heartbeat);
  }

  bool setSemanticAuthority(std::uint64_t semantic_key, const BrainInputs &snapshot) {
    std::lock_guard<std::mutex> decision_lock(decision_mutex_);
    std::lock_guard<std::mutex> lock(authority_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    if (!heartbeatSnapshotCurrent(snapshot)) return false;
    latest_semantic_key_.store(semantic_key, std::memory_order_release);
    return true;
  }

  void receiveCommand(const Command &command) {
    latest_generation_.store(command.generation, std::memory_order_release);
    publishBaseline(command);

    if (!enabled_) {
      output_pub_->publish(command);
      publishStatus(command, nullptr, "disabled");
      return;
    }
    if (brain_mode_) {
      receiveBrainCommand(command);
      return;
    }

    // Compatibility refinement mode. The upstream command is published first;
    // if MPPI cannot certify a refinement, CMA PP continues on StateLattice.
    output_pub_->publish(command);
    if (command.emergency_stop || !command.corridor_clearance_profile ||
        !isManeuverMode(command.mode) ||
        command.trajectory.points.size() < 3U) {
      publishStatus(command, nullptr, "passthrough:not_eligible");
      return;
    }

    std::optional<nav_msgs::msg::Odometry> odometry;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      odometry = odometry_;
    }
    if (!odometry.has_value()) {
      publishStatus(command, nullptr, "passthrough:no_odometry");
      return;
    }

    auto work = makeWork(command, odometry.value());
    if (!work.has_value()) {
      publishStatus(command, nullptr, "passthrough:invalid_trajectory");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(work_mutex_);
      WorkBatch batch;
      batch.fallback = command;
      batch.candidates.push_back(std::move(work.value()));
      batch.candidate_count = 1U;
      pending_batch_ = std::move(batch);
    }
    work_cv_.notify_one();
  }

  std::uint64_t publishOutput(const Command &command) {
    std::uint64_t publication_generation = command.generation;
    if (brain_mode_) {
      sequenced_output_.publish(command, [this, &publication_generation, source_generation=command.generation](const Command &output) {
        output_pub_->publish(output);
        std::atomic_store(&published_execution_reference_,
            std::make_shared<const mppi::TemporaryReference>(temporaryReferenceFromCommand(output)));
        std::atomic_store(&published_command_, std::make_shared<const Command>(output));
        // Keep visualization in the same publication order as the command.
        publishAppliedCommand(output);
        publication_generation = output.generation;
        const auto &points = output.trajectory.points;
        if (points.empty()) return;
        if (output.reason.find("speed_fallback") != std::string::npos ||
            output.reason.find("speed_recovery") != std::string::npos ||
            output.reason.find("braking_fallback") != std::string::npos) {
          RCLCPP_INFO(get_logger(),
              "[MPPI_RECOVERY_APPLIED] publication=%lu source=%lu stamp=%.9f age_ms=%.3f reason=%s first=%.6f",
              static_cast<unsigned long>(output.generation), static_cast<unsigned long>(source_generation),
              rclcpp::Time(output.header.stamp).seconds(),
              (now().seconds()-rclcpp::Time(output.header.stamp).seconds())*1000.,
              output.reason.c_str(), points.front().longitudinal_velocity_mps);
        }
        double arc = 0.0;
        double acceleration_arc = -1.0;
        double acceleration_x = 0.0, acceleration_y = 0.0;
        for (std::size_t i = 1; i < points.size(); ++i) {
          arc += distance(points[i - 1].pose.position, points[i].pose.position);
          if (points[i].longitudinal_velocity_mps >=
              points.front().longitudinal_velocity_mps + 0.5) {
            acceleration_arc = arc;
            acceleration_x = points[i].pose.position.x;
            acceleration_y = points[i].pose.position.y;
            break;
          }
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[MPPI_OUTPUT_SPEED] generation=%lu source_generation=%lu source_stamp=%.9f valid_until=%.9f reason=%s first=%.3f end=%.3f "
            "rise_0_5_arc_m=%.3f rise_x=%.3f rise_y=%.3f",
            static_cast<unsigned long>(output.generation), static_cast<unsigned long>(source_generation),
            rclcpp::Time(output.header.stamp).seconds(), output.valid_until_sec, output.reason.c_str(),
            points.front().longitudinal_velocity_mps, points.back().longitudinal_velocity_mps,
            acceleration_arc, acceleration_x, acceleration_y);
      });
    } else {
      output_pub_->publish(command);
    }
    return publication_generation;
  }

  static Command executionPlanContent(Command plan) {
    plan.header.stamp = builtin_interfaces::msg::Time{};
    plan.trajectory.header.stamp = builtin_interfaces::msg::Time{};
    plan.generation = 0;
    plan.geometry_revision = 0;
    plan.valid_until_sec = 0.0;
    return plan;
  }

  std::pair<Command,int> heartbeatPlanContent(const BrainInputs &snapshot, const Command &command) {
    const auto content=executionPlanContent(command);
    // Prove provenance by reconstructing the exact projection and recovery
    // transformation. Owner equality or equal first speeds alone is insufficient.
    if (snapshot.active_command && snapshot.odometry && snapshot.base_reference &&
        (command.reason=="mppi_brain:hold_speed_recovery" ||
         command.reason=="mppi_brain:alongside_hold:speed_recovery")) {
      const auto projected=refreshedBrainHold(*snapshot.active_command,command,
          *snapshot.odometry,*snapshot.base_reference,snapshot.speed_profile.get());
      for(int level=9;level>0;--level) {
        auto expected=projected;
        expected.reason=command.reason;
        for(auto &point:expected.trajectory.points)
          point.longitudinal_velocity_mps=static_cast<float>(
              point.longitudinal_velocity_mps*(level/10.));
        if(executionPlanContent(expected)==content) {
          auto canonical=executionPlanContent(*snapshot.active_command);
          canonical.reason=command.reason;
          // Same immutable speed owner is also checked by the caller.
          // Encode the recovery transform independently of the projected suffix.
          return {canonical,level};
        }
      }
    }
    return {content,0};
  }

  // Caller holds decision/authority/input locks. Input arrivals alone do not
  // invalidate a prepared heartbeat; execution ownership and recovery do.
  bool heartbeatSnapshotCurrent(const BrainInputs &snapshot) const {
    const auto source = [](const std::optional<Command> &c) {
      return c ? std::optional<std::uint64_t>(c->generation) : std::nullopt;
    };
    return !recovery_active_ && snapshot.recovery_epoch == recovery_epoch_ &&
        snapshot.driving_fsm.revision() == driving_fsm_.revision() &&
        source(snapshot.active_command) == source(active_brain_command_) &&
        snapshot.speed_generation == active_brain_speed_generation_ &&
        snapshot.reference_revision == reference_revision_ &&
        snapshot.execution_revision == execution_revision_;
  }

  bool publishHeartbeatCommand(const BrainInputs &snapshot, const Command &command,
                              const std::function<void()> &commit = {},
                              bool reference_commit = false) {
    const bool recovery = command.reason.find("speed_fallback") != std::string::npos ||
        command.reason.find("speed_recovery") != std::string::npos ||
        command.reason.find("braking_fallback") != std::string::npos;
    const auto prepared_plan = recovery ?
        std::optional<std::pair<Command,int>>(heartbeatPlanContent(snapshot,command)) : std::nullopt;
    std::lock_guard<std::mutex> decision_lock(decision_mutex_);
    std::lock_guard<std::mutex> authority_lock(authority_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    const auto source = [](const std::optional<Command> &c) {
      return c ? std::optional<std::uint64_t>(c->generation) : std::nullopt;
    };
    if (!heartbeatSnapshotCurrent(snapshot)) {
      RCLCPP_INFO(get_logger(),
          "[MPPI_HEARTBEAT_SUPERSEDED] generation=%lu source=%lu live_source=%lu speed_generation=%lu live_speed_generation=%lu",
          static_cast<unsigned long>(command.generation),
          static_cast<unsigned long>(source(snapshot.active_command).value_or(0)),
          static_cast<unsigned long>(source(active_brain_command_).value_or(0)),
          static_cast<unsigned long>(snapshot.speed_generation),
          static_cast<unsigned long>(active_brain_speed_generation_));
      return false;
    }
    if (reference_commit &&
        (snapshot.requested_reference_revision != requested_reference_revision_ ||
         snapshot.requested_reference != requested_reference_ ||
         snapshot.vehicle_observation_sequence != vehicle_observation_sequence_)) return false;
    if (commit) commit();
    if (recovery || last_recovery_plan_) {
      auto plan = prepared_plan ? *prepared_plan : std::pair<Command,int>{};
      const auto shape = source(active_brain_command_).value_or(0);
      const bool repeated = recovery && last_recovery_plan_ &&
          *last_recovery_plan_ == plan && last_recovery_shape_ == shape &&
          last_recovery_speed_ == active_brain_speed_generation_;
      if (!repeated) ++execution_revision_;
      if (recovery) {
        last_recovery_plan_ = std::move(plan);
        last_recovery_shape_ = shape;
        last_recovery_speed_ = active_brain_speed_generation_;
      } else {
        last_recovery_plan_.reset();
      }
      RCLCPP_INFO(get_logger(), "[MPPI_RECOVERY_REVISION] generation=%lu revision=%lu repeated=%d recovery=%d projection_level=%d",
          static_cast<unsigned long>(command.generation),
          static_cast<unsigned long>(execution_revision_), repeated, recovery,
          recovery ? last_recovery_plan_->second : 0);
    }
    const auto publication = publishOutput(command);
    if (get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool())
      RCLCPP_INFO(get_logger(),
          "[MPPI_HEARTBEAT_APPLIED] publication=%lu input=%lu source=%lu stamp=%.9f",
          static_cast<unsigned long>(publication), static_cast<unsigned long>(snapshot.input_sequence),
          static_cast<unsigned long>(command.generation), rclcpp::Time(command.header.stamp).seconds());
    return true;
  }

  void receiveSteeringReport(
      const autoware_auto_vehicle_msgs::msg::SteeringReport &message) {
    const double receive_sec = now().seconds();
    const double stamp_sec = rclcpp::Time(message.stamp).seconds();
    std::lock_guard<std::mutex> lock(input_mutex_);
    steering_rad_ = message.steering_tire_angle;
    steering_receive_sec_ = receive_sec;
    steering_report_history_.push_back(
        {stamp_sec, receive_sec, steering_rad_, ++input_event_sequence_});
    // Retain same-stamp arrivals: the newest received sample wins at that stamp.
    while (steering_report_history_.size() > 256U)
      steering_report_history_.pop_front();
  }

  BrainInputs captureBrainInputs(bool update_contract = false) {
    BrainInputs inputs;
    inputs.captured_at = std::chrono::steady_clock::now();
    const double now_sec = now().seconds();
    std::unique_lock<std::mutex> contract_lock(decision_mutex_,std::defer_lock);
    if (update_contract) contract_lock.lock();
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      inputs.input_sequence = ++input_snapshot_sequence_;
      if (leader_receive_sec_ >= 0. && now_sec >= leader_receive_sec_ &&
          now_sec-leader_receive_sec_ <= brain_input_timeout_sec_)
        inputs.leader_vehicle_id = leader_vehicle_id_;
      inputs.odometry_sequence = odometry_sequence_;
      inputs.control_sequence = control_sequence_;
      inputs.odometry = odometry_;
      inputs.recovery_active = recovery_active_;
      inputs.recovery_epoch = recovery_epoch_;
      inputs.driving_fsm = driving_fsm_;
      inputs.active_target_id = active_maneuver_target_id_;
      inputs.prediction_target_id = prediction_target_id_;
      const double state_stamp = inputs.odometry.has_value() ?
          rclcpp::Time(inputs.odometry->header.stamp).seconds() : now_sec;
      inputs.prior_execution_reference=std::atomic_load(&published_execution_reference_);
      inputs.reference_activation_delay_sec=std::max(0.,now_sec-state_stamp)+
          preparation_planning_delay_sec_.load(std::memory_order_relaxed);
      for (auto it = rotation_prediction_history_.rbegin();
           it != rotation_prediction_history_.rend(); ++it) {
        if (it->stamp_sec <= state_stamp + 1e-9 &&
            state_stamp - it->stamp_sec <= brain_input_timeout_sec_) {
          inputs.rotation_prediction = *it;
          break;
        }
      }
      inputs.base_reference = base_reference_;
      inputs.requested_reference = requested_reference_;
      inputs.reference_revision = reference_revision_;
      inputs.requested_reference_revision = requested_reference_revision_;
      inputs.vehicle_observation_sequence = vehicle_observation_sequence_;
      inputs.published_command = std::atomic_load(&published_command_);
      inputs.wall_map = wall_map_;
      inputs.wall_index = wall_index_;
      inputs.steering_rad = steering_rad_;
      inputs.steering_fresh =
          steering_receive_sec_ >= 0.0 &&
          now_sec - steering_receive_sec_ <= brain_input_timeout_sec_;
      if (!steering_report_history_.empty()) {
        inputs.steering_fresh = false;
        for (auto it=steering_report_history_.rbegin();
             it!=steering_report_history_.rend(); ++it) {
          if (it->stamp_sec > state_stamp + 1e-9 ||
              (inputs.steering_sequence && it->stamp_sec <= inputs.steering_stamp_sec))
            continue;
          inputs.steering_rad = it->angle_rad;
          inputs.steering_stamp_sec = it->stamp_sec;
          inputs.steering_receive_sec = it->receive_sec;
          inputs.steering_sequence = it->sequence;
          inputs.steering_fresh =
              now_sec-it->receive_sec <= brain_input_timeout_sec_ &&
              state_stamp-it->stamp_sec <= brain_input_timeout_sec_;
        }
      }
      std::array<mppi::TimedSteeringCommand, kSteeringCommandHistoryCapacity>
          steering_history{};
      const std::size_t steering_history_count = std::min(
          steering_command_history_.size(), kSteeringCommandHistoryCapacity);
      const std::size_t steering_history_offset =
          steering_command_history_.size() - steering_history_count;
      for (std::size_t index = 0U; index < steering_history_count; ++index) {
        steering_history[index] =
            steering_command_history_[steering_history_offset + index];
      }
      inputs.pending_steering_targets =
          mppi::samplePendingSteeringTargets<kSteeringCommandHistoryCapacity,
                                             mppi::kMaximumHorizonSteps>(
              steering_history, steering_history_count, state_stamp,
              config_.steering_control_delay_sec, config_.dt_sec,
              inputs.steering_fresh
                  ? simple_pure_pursuit::tireToCommandSteeringAngle(
                        inputs.steering_rad,
                        config_.steering_command_to_tire_angle_ratio,
                        config_.maximum_steering_angle_rad)
                  : 0.0);
      inputs.preferred_side = preferred_side_;
      inputs.active_command = active_brain_command_;
      inputs.preparation = latest_preparation_;
      inputs.ot_lane_entry = latest_ot_lane_entry_;
      inputs.active_preparation = active_preparation_;
      inputs.speed_profile=active_brain_speed_profile_;
      inputs.speed_generation=active_brain_speed_generation_;
      inputs.execution_revision=execution_revision_;
      for (const auto &[id, vehicle] : observed_vehicles_) {
        (void)id;
        const double receive_age_sec = now_sec - vehicle.receive_sec;
        const double source_age_sec = now_sec - vehicle.stamp_sec;
        if (receive_age_sec >= -1.0e-6 &&
            receive_age_sec <= brain_input_timeout_sec_ &&
            source_age_sec >= -1.0e-6 &&
            source_age_sec <= brain_input_timeout_sec_) {
          auto aligned = vehicle;
          const double alignment_dt=state_stamp-aligned.stamp_sec;
          aligned.x_m += aligned.vx_mps * alignment_dt;
          aligned.y_m += aligned.vy_mps * alignment_dt;
          aligned.stamp_sec = state_stamp;
          inputs.opponents.push_back(std::move(aligned));
        }
      }
      if (brain_mode_ && inputs.odometry && inputs.base_reference &&
          inputs.base_reference->points.size()>=3U) {
        std::unordered_set<std::string> fresh_ids;
        for (const auto &other:inputs.opponents) {
          fresh_ids.insert(other.id);
          if (hasAlongsideVehicle(*inputs.base_reference,*inputs.odometry,{other})) {
            if (update_contract) alongside_vehicle_ids_.insert(other.id);
            inputs.alongside=true;
          } else if (update_contract) alongside_vehicle_ids_.erase(other.id);
        }
        for (const auto &id:alongside_vehicle_ids_)
          if (!fresh_ids.count(id)) inputs.alongside_unknown=true;
        if (update_contract && reference_transition_ && referenceTransitionComplete(*inputs.odometry)) {
          reference_transition_.reset();
          ++execution_revision_;
        }
        const bool hold_ordinary=!inputs.active_command &&
            (inputs.alongside || inputs.alongside_unknown);
        if (update_contract && hold_ordinary && !ordinary_hold_ && inputs.published_command) {
          ordinary_hold_=*inputs.published_command;
          ++execution_revision_;
        } else if (update_contract && !hold_ordinary && ordinary_hold_) {
          ordinary_hold_.reset();
          ++execution_revision_;
        }
        inputs.reference_transition=reference_transition_.has_value();
        inputs.ordinary_hold=ordinary_hold_ ? ordinary_hold_ : reference_transition_;
        inputs.execution_revision=execution_revision_;
      }
    }
    if(brain_mode_ && inputs.base_reference && inputs.wall_map)
      inputs.wall_lines=sharedWallLines(inputs.base_reference,inputs.wall_map,inputs.wall_index);
    publishPassingPointForbiddenAreas(inputs);
    std::sort(inputs.opponents.begin(),inputs.opponents.end(),
        [](const ObservedVehicle &a,const ObservedVehicle &b){return a.id<b.id;});
    if ((leader_lap_prediction_enabled_ || prior_lap_prediction_enabled_ || recent_motion_prediction_enabled_ ||
         online_motion_prediction_enabled_ || collection_motion_enabled_) &&
        inputs.base_reference && inputs.base_reference->points.size() >= 3U) {
      const auto shared_world = sharedReferencePoseIndex(inputs.base_reference);
      const auto &world = *shared_world;
      const double length = trajectoryArcLength(*inputs.base_reference);
      const double horizon = std::max(brain_prediction_horizon_sec_,
          config_.dt_sec * config_.horizon_steps);
      for (auto &vehicle : inputs.opponents) {
        if (!vehicle.history) continue;
        if (collection_motion_enabled_) {
          // Fit at the observed epoch, then align once to this snapshot.
          // Do not let the scalar-speed online model overwrite this vector fit.
          const auto fit = opponent_prediction::fitCollectionMotion(*vehicle.history);
          vehicle.prediction.reset();
          vehicle.vx_mps = vehicle.vy_mps = 0.;
          if (fit && !vehicle.history->empty()) {
            const double age = vehicle.stamp_sec - vehicle.history->back().stamp;
            if (age >= 0.) {
              vehicle.x_m = fit->x_m + age * fit->vx_mps;
              vehicle.y_m = fit->y_m + age * fit->vy_mps;
              vehicle.vx_mps = fit->vx_mps; vehicle.vy_mps = fit->vy_mps;
              vehicle.sigma_x_m = std::max(vehicle.sigma_x_m, fit->sigma_x_m);
              vehicle.sigma_y_m = std::max(vehicle.sigma_y_m, fit->sigma_y_m);
            }
          }
          continue;
        }
        auto persistence = motion_persistence_;
        if (!vehicle.prediction && online_motion_prediction_enabled_ && vehicle.online_motion) {
          const auto learned=vehicle.online_motion->summary();
          persistence=learned.persistence;
          if (learned.follows_reference && !vehicle.history->empty()) {
            const auto state=opponent_prediction::fitReferenceMotion(*vehicle.history,world);
            if (state) vehicle.prediction=opponent_prediction::buildReferenceMotionPrediction(
                *state,vehicle.stamp_sec,horizon,world,learned.lateral_persistence_sec,
                learned.persistence.acceleration_sec);
          }
        }
        if (!vehicle.prediction && (recent_motion_prediction_enabled_ || online_motion_prediction_enabled_))
          vehicle.prediction = opponent_prediction::buildRecentMotionPrediction(
              *vehicle.history, vehicle.stamp_sec, horizon, world, persistence);
        if (!vehicle.prediction && prior_lap_prediction_enabled_ &&
            distance(inputs.base_reference->points.front().pose.position,
                     inputs.base_reference->points.back().pose.position) < 5.0)
          vehicle.prediction = opponent_prediction::buildPriorLapPrediction(
              *vehicle.history, length, vehicle.stamp_sec, horizon, world);
        if (!vehicle.prediction) continue;
        const auto &p = vehicle.prediction->points.front();
        const auto pose = world.pose(p.global_s, p.d);
        vehicle.x_m = (*pose)[0]; vehicle.y_m = (*pose)[1];
        vehicle.vx_mps = vehicle.prediction->vx;
        vehicle.vy_mps = vehicle.prediction->vy;
      }
    }
    if(inputs.base_reference && inputs.base_reference->points.size()>=3U && inputs.odometry) {
      inputs.leading=selectFollowVehicle(*inputs.base_reference,*inputs.base_reference,*inputs.odometry,inputs);
      if(brain_mode_) {
        inputs.leader_vehicle_id=selectPredictionTarget(inputs);
        inputs.prediction_target_id=inputs.leader_vehicle_id;
        if(update_contract) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          if(prediction_target_id_!=inputs.prediction_target_id) {
            prediction_target_id_=inputs.prediction_target_id;
            ++execution_revision_;
          }
          inputs.execution_revision=execution_revision_;
        }
      }
      if (contract_lock.owns_lock()) contract_lock.unlock();
      if(leader_lap_prediction_enabled_ && !collection_motion_enabled_ &&
          distance(inputs.base_reference->points.front().pose.position,
                   inputs.base_reference->points.back().pose.position)<5.) {
        const auto world=sharedReferencePoseIndex(inputs.base_reference);
        const double length=trajectoryArcLength(*inputs.base_reference);
        for(auto &vehicle:inputs.opponents) {
          if(vehicle.id!=inputs.leader_vehicle_id || !vehicle.history) continue;
          vehicle.leader_lap_prediction=opponent_prediction::buildLeaderLapPrediction(
              *vehicle.history,length,vehicle.stamp_sec,std::max(20.,brain_prediction_horizon_sec_),*world);
          if(passing_preparation_enabled_)
            vehicle.prior_lap_match=mppi::matchPriorLap(*vehicle.history,length,preparation_config_);
          // Select once per input snapshot so candidate, hold, retiming and
          // rollout validation share the matched leader's time-indexed route.
          // A later mismatch naturally retains the newly built current model.
          const double execution_horizon=std::max(brain_prediction_horizon_sec_,
              config_.dt_sec*config_.horizon_steps);
          if(front_merge_attack_enabled_ &&
              std::hypot(vehicle.vx_mps,vehicle.vy_mps)>kAvoidanceMaximumLeaderSpeedMps &&
              vehicle.prior_lap_match.usable &&
              vehicle.leader_lap_prediction && vehicle.leader_lap_prediction->points.size()>=2U &&
              vehicle.leader_lap_prediction->points.back().time>=execution_horizon) {
            vehicle.prediction=vehicle.leader_lap_prediction;
            vehicle.matched_prior_lap_execution=true;
          }
        }
      }
    }
    if (contract_lock.owns_lock()) contract_lock.unlock();
    if (get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool()) {
      std::ostringstream record; record.precision(17);
      record << "input=" << inputs.input_sequence << " now=" << now_sec
          << " odom_seq=" << inputs.odometry_sequence
          << " steering_seq=" << inputs.steering_sequence
          << " control_seq=" << inputs.control_sequence
          << " steering_stamp=" << inputs.steering_stamp_sec
          << " steering_receive=" << inputs.steering_receive_sec
          << " steering=" << inputs.steering_rad
          << " steering_fresh=" << inputs.steering_fresh
          << " revision=" << inputs.execution_revision
          << " prior_model=" << prior_lap_prediction_enabled_
          << " recent_motion_model=" << recent_motion_prediction_enabled_
          << " online_motion_model=" << online_motion_prediction_enabled_
          << " collection_motion_model=" << collection_motion_enabled_;
      if (inputs.odometry) {
        const auto &o=*inputs.odometry;
        record << " stamp=" << rclcpp::Time(o.header.stamp).seconds()
            << " ego=[" << o.pose.pose.position.x << ',' << o.pose.pose.position.y
            << ',' << yawFromQuaternion(o.pose.pose.orientation) << ',' << o.twist.twist.linear.x
            << ',' << o.twist.twist.linear.y << ',' << o.twist.twist.angular.z << ']';
      }
      record << " pending_valid=" << inputs.pending_steering_targets.valid << " pending=[";
      for (std::size_t i=0; i<inputs.pending_steering_targets.count; ++i)
        record << inputs.pending_steering_targets.targets_rad[i] << ',';
      record << "] pending_sources=[";
      for (std::size_t i=0; i<inputs.pending_steering_targets.count; ++i)
        record << inputs.pending_steering_targets.source_stamps_sec[i] << ','
            << inputs.pending_steering_targets.source_sequences[i] << ';';
      record << "] opponents=[";
      for (const auto &v:inputs.opponents)
        record << v.id << ',' << v.input_sequence << ',' << v.source_stamp_sec
            << ',' << v.receive_sec << ',' << v.stamp_sec << ',' << v.x_m << ','
            << v.y_m << ',' << v.vx_mps << ',' << v.vy_mps << ',' << v.sigma_x_m
            << ',' << v.sigma_y_m << ';';
      record << "] leading=" << (inputs.leading?inputs.leading->vehicle.id:"")
          << " prediction_target=" << inputs.prediction_target_id
          << " leader=" << inputs.leader_vehicle_id << " leader_lap=[";
      for (const auto &v:inputs.opponents)
        if (v.leader_lap_prediction) record << v.id << ';';
      record << "] execution_predictions=[";
      for(const auto &v:inputs.opponents)
        record << v.id << ',' << (v.matched_prior_lap_execution ? "matched_prior_lap" : "current_motion")
            << ',' << v.prior_lap_match.reason << ',' << v.prior_lap_match.position_error_m
            << ',' << v.prior_lap_match.heading_error_rad << ',' << v.prior_lap_match.pace_error_ratio << ';';
      record << ']';
      if (online_motion_prediction_enabled_) {
        record << " online=[";
        for (const auto &v : inputs.opponents) {
          if (!v.online_motion) continue;
          const auto learned = v.online_motion->summary();
          record << v.id << ',' << learned.model << ','
              << learned.persistence.acceleration_sec << ',' << learned.persistence.yaw_rate_sec
              << ',' << learned.follows_reference << ',' << learned.lateral_persistence_sec
              << ',' << learned.mean_squared_error;
          for (auto n : learned.samples) record << ',' << n;
          record << ';';
        }
        record << ']';
      }
      RCLCPP_INFO(get_logger(), "[MPPI_INPUT_USED] %s", record.str().c_str());
    }
    return inputs;
  }

  bool referenceTransitionComplete(const nav_msgs::msg::Odometry &ego) const {
    const auto &p=ego.pose.pose.position;
    const auto progress=projectOnTrajectory(reference_transition_->trajectory,p.x,p.y);
    const auto aligned=projectOnTrajectory(*base_reference_,p.x,p.y);
    return progress.valid && progress.s_m>=reference_transition_end_m_ &&
        aligned.valid && std::abs(aligned.d_m)<=.30 &&
        std::abs(std::remainder(yawFromQuaternion(ego.pose.pose.orientation)-
            std::atan2(aligned.tangent_y,aligned.tangent_x),2*M_PI))<=.20;
  }

  bool tryReferenceTransition(const BrainInputs &inputs,const Command &heartbeat) {
    if (!inputs.requested_reference || sameReference(inputs.base_reference,inputs.requested_reference) ||
        inputs.requested_reference->points.size()<3U || !inputs.odometry ||
        inputs.reference_transition || inputs.active_command ||
        inputs.driving_fsm.mode()!=DrivingMode::FREE_RUN ||
        inputs.alongside || inputs.alongside_unknown) return false;
    auto connection=makeBrainBaseCommand(heartbeat,*inputs.requested_reference,*inputs.odometry,
        inputs.steering_fresh?inputs.steering_rad:0.,true);
    if (!connection || !isConnectedReturn(*connection)) return false;
    *connection=makeBrainFollowCommand(*connection,inputs.leading,inputs.driving_fsm);
    // Opponent predictions and station histories still belong to the committed world.
    const auto validator=makeBrainCommandValidator(inputs,*inputs.odometry,0.);
    if (validateBrainCommand(*connection,validator)!=mppi::RejectReason::NONE) {
      const auto recovered=recoverBrainCommandSpeed(*connection,validator);
      if (!recovered) return false;
      connection=*recovered;
    }
    connection->mode=inputs.driving_fsm.modeName();
    connection->reason="mppi_brain:reference_transition";
    const auto &p=inputs.odometry->pose.pose.position;
    const auto projection=projectOnTrajectory(*inputs.requested_reference,p.x,p.y);
    const double connection_end=std::clamp(5.*std::abs(projection.d_m),8.,25.)+4.;
    if (!setSemanticAuthority(0U,inputs)) return true;
    const bool applied=publishHeartbeatCommand(inputs,*connection,[&] {
      commitBaseReference(inputs.requested_reference);
      reference_transition_=*connection;
      reference_transition_end_m_=connection_end;
      ordinary_hold_.reset();
      latest_semantic_key_.store(0U,std::memory_order_release);
      RCLCPP_INFO(get_logger(),
          "[MPPI_REFERENCE_ADOPTED] requested=%lu committed=%lu stamp=%.9f",
          requested_reference_revision_,reference_revision_,
          rclcpp::Time(connection->header.stamp).seconds());
    },true);
    if (applied) {
      auto committed=captureBrainInputs();
      publishReferenceContract(committed,DrivingMode::FREE_RUN);
      publishStatus(*connection,nullptr,"brain:reference_transition");
    }
    return true;
  }

  void publishReferenceContract(const BrainInputs &inputs,DrivingMode intent) {
    std::ostringstream text;
    text << "{\"intent\":\"" << drivingModeName(intent)
         << "\",\"mode\":\"" << inputs.driving_fsm.modeName()
         << "\",\"alongside\":" << (inputs.alongside?"true":"false")
         << ",\"relationship_unknown\":" << (inputs.alongside_unknown?"true":"false")
         << ",\"requested_revision\":" << inputs.requested_reference_revision
         << ",\"committed_revision\":" << inputs.reference_revision
         << ",\"reference_pending\":" <<
             (!sameReference(inputs.base_reference,inputs.requested_reference)?"true":"false")
         << ",\"transition_active\":" << (inputs.reference_transition?"true":"false")
         << ",\"publication\":" << (inputs.published_command?inputs.published_command->generation:0)
         << ",\"published_geometry_revision\":" <<
             (inputs.published_command?inputs.published_command->geometry_revision:0)
         << ",\"ordinary_shape_source\":" << (inputs.ordinary_hold?inputs.ordinary_hold->generation:0)
         << ",\"active_shape_source\":" << (inputs.active_command?inputs.active_command->generation:0) << '}';
    std_msgs::msg::String status;status.data=text.str();
    reference_contract_pub_->publish(status);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,"[MPPI_REFERENCE_CONTRACT] %s",status.data.c_str());
  }

  Command ordinaryExecutionFallback(const BrainInputs &inputs,const Command &desired,
      const mppi::PathConstraintValidator &validator) const {
    auto hold=refreshedBrainHold(*inputs.ordinary_hold,desired,*inputs.odometry,*inputs.base_reference);
    if (hold.trajectory.points.size()>=3U) {
      if (const auto context=executionContext(inputs,*inputs.odometry))
        hold=planBrainFollowSpeed(hold,*context);
      hold=physicallyBoundedFallback(hold,validator);
      hold.mode=inputs.driving_fsm.modeName();
      hold.reason="mppi_brain:ordinary_hold:"+hold.reason;
      return hold;
    }
    // Exhaustion needs a newly validated continuation on the committed route.
    auto fallback=connectedReturnFallback(desired,*inputs.odometry,
        inputs.steering_fresh?inputs.steering_rad:0.,*inputs.base_reference,{},
        DrivingMode::AVOID,validator,inputs.opponents);
    fallback.mode=inputs.driving_fsm.modeName();
    fallback.reason="mppi_brain:ordinary_exhausted:"+fallback.reason;
    return fallback;
  }

  void receiveBrainCommand(const Command &command) {
    auto inputs = captureBrainInputs(true);
    if (opponent_predictions_pub_->get_subscription_count() > 0U ||
        opponent_predictions_pub_->get_intra_process_subscription_count() > 0U)
      opponent_predictions_pub_->publish(makeOpponentPredictionMarkers(inputs));
    if (inputs.recovery_active) return;
    const double now_sec = now().seconds();
    const auto publish_heartbeat = [&](const BrainInputs &snapshot, const Command &output,
                                      const std::function<void()> &commit = {}) {
      return publishHeartbeatCommand(snapshot, output, commit);
    };
    // Keep the latest completed template set visible while the worker evaluates
    // the next generation. Clearing here made a valid path visible for less
    // than one 20 Hz tick whenever collision evaluation ran long.
    if (!inputs.odometry.has_value() || inputs.base_reference == nullptr ||
        inputs.base_reference->points.size() < 3U ||
        now_sec-rclcpp::Time(inputs.odometry->header.stamp).seconds() < -1e-6 ||
        now_sec-rclcpp::Time(inputs.odometry->header.stamp).seconds() > brain_input_timeout_sec_) {
      if (!setSemanticAuthority(0U, inputs)) return;
      clearCandidateMarkers(command.header);
      auto fallback = command;
      fallback.mode = inputs.driving_fsm.modeName();
      publish_heartbeat(inputs, fallback);
      publishStatus(command, nullptr, "brain:fallback_missing_input");
      return;
    }

    if (tryReferenceTransition(inputs,command)) return;

    auto base_command = makeBrainBaseCommand(command, *inputs.base_reference,
        inputs.odometry.value(), inputs.steering_fresh ? inputs.steering_rad : 0.,
        false);
    if (!base_command.has_value()) {
      if (!setSemanticAuthority(0U, inputs)) return;
      clearCandidateMarkers(command.header);
      auto fallback = command;
      fallback.mode = inputs.driving_fsm.modeName();
      publish_heartbeat(inputs, fallback);
      publishStatus(command, nullptr, "brain:fallback_invalid_reference");
      return;
    }
    publishBaseline(base_command.value());
    const auto fallback_validator =
        makeBrainCommandValidator(inputs, inputs.odometry.value(), 0.0);
    const auto bounded_fallback = [&](const Command &desired) {
      if (!inputs.active_command && inputs.ordinary_hold)
        return ordinaryExecutionFallback(inputs,desired,fallback_validator);
      auto bounded = connectedReturnFallback(desired,*inputs.odometry,
          inputs.steering_fresh?inputs.steering_rad:0.,*inputs.base_reference,
          inputs.active_command,inputs.driving_fsm.mode(),fallback_validator,
          inputs.opponents,inputs.speed_profile.get());
      bounded.mode = inputs.driving_fsm.modeName();
      return bounded;
    };
    const auto publish_committed_hold = [&](const std::string &state) {
      if (!inputs.active_command.has_value()) {
        return false;
      }
      auto hold =
          refreshedBrainHold(inputs.active_command.value(), command,
                             inputs.odometry.value(), *inputs.base_reference,inputs.speed_profile.get());
      const auto validator = makeBrainCommandValidator(
          inputs, inputs.odometry.value(),
          brain_hold_minimum_opponent_clearance_m_);
      if (validateBrainCommand(hold, validator) != mppi::RejectReason::NONE) {
        const auto recovered = recoverBrainCommandSpeed(hold, validator);
        if (!recovered) return false;
        hold = *recovered;
        hold.reason = "mppi_brain:hold_speed_recovery";
      }
      // A newer owner already published while validation ran. Do not fall
      // through to an older follow/fallback command in that case either.
      if (!publish_heartbeat(inputs, hold)) return true;
      // active_brain_command_ remains the last trajectory actually accepted
      // by MPPI. Recursively refreshing an ego-anchored hold makes its shape
      // drift toward the base Reference on every heartbeat.
      publishStatus(hold, nullptr, state);
      return true;
    };

    const auto &leading = inputs.leading;
    auto follow_command = makeBrainFollowCommand(*base_command, leading, inputs.driving_fsm);
    const bool show_passing_opportunity =
        passing_opportunity_pub_->get_subscription_count() > 0U ||
        passing_opportunity_pub_->get_intra_process_subscription_count() > 0U;
    if (config_.longitudinal_planning_enabled || show_passing_opportunity || passing_preparation_enabled_) {
      auto context = executionContext(inputs, *inputs.odometry);
      if(context && passing_preparation_enabled_) {
        updatePassingPreparation(inputs,*context);
        maybeFirePreparationBoost(inputs);
      }
      if (show_passing_opportunity)
        publishPassingPreparationMarkers(inputs,context ? &*context : nullptr);
      if (context && config_.longitudinal_planning_enabled) {
        for (std::size_t i=0; i<context->base_reference_count; ++i)
          context->base_reference[i].speed_mps = brain_cruise_speed_mps_;
        follow_command=planBrainFollowSpeed(follow_command,*context);
      }
    }
    const auto projection = projectOnTrajectory(*inputs.base_reference,
        inputs.odometry->pose.pose.position.x, inputs.odometry->pose.pose.position.y);
    const double yaw_error = projection.valid ? std::abs(std::remainder(
        yawFromQuaternion(inputs.odometry->pose.pose.orientation) -
        std::atan2(projection.tangent_y, projection.tangent_x), 2.0 * M_PI)) : INFINITY;
    const bool reference_valid =
        validateBrainCommand(*base_command, fallback_validator) == mppi::RejectReason::NONE;
    const DrivingScene scene{reference_valid, leading.has_value(),
        projection.valid && std::abs(projection.d_m) <= 0.30 && yaw_error <= 0.20,
        leading ? std::optional<double>(leading->body_gap_m) : std::nullopt,
        leading ? std::optional<double>(std::hypot(
            leading->vehicle.vx_mps, leading->vehicle.vy_mps)) : std::nullopt,
        passing_preparation_enabled_ && inputs.preparation &&
            mppi::preparationSearchDue(*inputs.preparation,rclcpp::Time(inputs.odometry->header.stamp).seconds(),
                preparation_planning_delay_sec_.load(std::memory_order_relaxed)),
        overtakeTargetValid(inputs),inputs.preparation!=nullptr,
        continueCollectionAvoidance(inputs)};
    auto decision = inputs.driving_fsm.plan(scene);
    publishReferenceContract(inputs,decision.maneuver);
    if(front_merge_attack_enabled_ && inputs.active_preparation &&
        inputs.active_preparation->front_merge && inputs.active_command &&
        inputs.active_command->corridor_side!=0 && projection.valid) {
      const auto &plan=*inputs.active_preparation;
      const double station=mppi::preparationStation(plan,inputs.odometry->pose.pose.position.x,
          inputs.odometry->pose.pose.position.y).value_or(-INFINITY);
      decision.maneuver=DrivingMode::OVERTAKE;
      const double body_gap=targetBodyPassGap(inputs,plan.target_id);
      decision.finish_avoid=frontMergeComplete(inputs,station,scene.reference_aligned);
      decision.generate_return=false;
      decision.generate_lateral=!decision.finish_avoid;
      if(decision.finish_avoid) {
        RCLCPP_INFO(get_logger(),
            "[MPPI_FRONT_MERGE_COMPLETED] stamp=%.9f plan=%lu target=%s passed=%d body_gap=%.6f station=%.6f merge_end=%.6f",
            rclcpp::Time(inputs.odometry->header.stamp).seconds(),plan.id,plan.target_id.c_str(),
            body_gap>0.,body_gap,station,plan.connection_end_station_m);
      }
    }
    if(keepAdoptedReturn(inputs,decision)) {
      // The adopted return owns its geometry until measured alignment. A new
      // collection avoidance may search again when a fresh obstacle still
      // blocks the Reference. Adoption must pass the same swept validator.
      decision.generate_lateral=false;
      decision.finish_avoid=scene.reference_valid && scene.reference_aligned;
      decision.generate_return=!decision.finish_avoid;
    }
    if (get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool()) {
      const char *avoidance_trigger =
          decision.generate_lateral && decision.maneuver==DrivingMode::AVOID && leading ?
          (leading->body_gap_m>config_.follow_gap_m ? "early_slow" : "nearby") : "none";
      RCLCPP_INFO(get_logger(),
          "[MPPI_START_GATE] input=%lu stamp=%.9f mode=%s leading=%s body_gap=%.6f threshold=%.6f reference_valid=%d reference_obstructed=%d generate_lateral=%d generate_return=%d ego_speed=%.6f leading_speed=%.6f maneuver=%s overtake_target=%s overtake_valid=%d preparation=%d follow_gap=%.6f avoidance_trigger=%s early_avoidance_max_speed_mps=%.6f",
          inputs.input_sequence, rclcpp::Time(inputs.odometry->header.stamp).seconds(),
          inputs.driving_fsm.modeName(), leading ? leading->vehicle.id.c_str() : "none",
          leading ? leading->body_gap_m : INFINITY, config_.follow_gap_m,
          scene.reference_valid, scene.reference_obstructed,
          decision.generate_lateral, decision.generate_return,
          inputs.odometry->twist.twist.linear.x,
          scene.leading_speed_mps.value_or(INFINITY),drivingModeName(decision.maneuver),
          inputs.leader_vehicle_id.c_str(),scene.overtake_target_valid,scene.preparation_available,
          followingGapForSpeed(scene.leading_speed_mps.value_or(INFINITY),
              config_.follow_gap_m,config_.overtake_follow_gap_m),
          avoidance_trigger,kEarlyAvoidanceMaximumLeaderSpeedMps);
    }
    // Crossing the reference during a lateral maneuver does not finish AVOID.
    // First adopt a return candidate, then confirm measured alignment.
    if (compare_return_continuation_ && decision.finish_avoid &&
        !(inputs.active_preparation && inputs.active_preparation->front_merge) &&
        inputs.active_command && inputs.active_command->corridor_side != 0) {
      decision.finish_avoid = false;
      decision.generate_return = true;
    }
    if (decision.finish_avoid) {
      auto released = bounded_fallback(follow_command);
      released.mode = "FREE_RUN";
      released.reason = "mppi_brain:avoid_complete";
      if (!publish_heartbeat(inputs, released, [&] {
        driving_fsm_.finishAvoid();
        preferred_side_ = 0;
        clearBrainExecution();
        latest_semantic_key_.store(0U, std::memory_order_release);
      })) return;
      clearCandidateMarkers(command.header);
      publishStatus(released, nullptr, "brain:avoid_complete");
      return;
    }
    if (!decision.generate_lateral && !decision.generate_return) {
      if (!setSemanticAuthority(0U, inputs)) return;
      const auto free_run = bounded_fallback(follow_command);
      if (!publish_heartbeat(inputs, free_run)) return;
      clearCandidateMarkers(command.header);
      publishStatus(free_run, nullptr, "brain:free_run");
      return;
    }
    const auto phase = decision.generate_return ? mppi::Phase::MERGE : mppi::Phase::OVERTAKE;
    const auto live_semantic_key = maneuverSemanticKey(phase,decision.maneuver);
    if (!setSemanticAuthority(live_semantic_key, inputs)) return;
    follow_command.mode = inputs.driving_fsm.modeName();

    WorkBatch batch;
    batch.maneuver=decision.maneuver;
    batch.maneuver_target_id=decision.maneuver==DrivingMode::OVERTAKE ?
        (inputs.active_preparation && inputs.active_preparation->front_merge ?
          inputs.active_preparation->target_id : inputs.leader_vehicle_id) : leading ? leading->vehicle.id : "";
    batch.allow_continuation=inputs.driving_fsm.mode()==decision.maneuver &&
        (!decision.generate_return || !inputs.active_command || inputs.active_command->corridor_side==0);
    // In brain mode StateLattice is only the transport heartbeat. Its tactical
    // SAFE_STOP/ABORT decisions are based on a different path and must not
    // override the MPPI-owned route.
    batch.fallback = follow_command;
    batch.brain_owned = true;
    batch.recovery_epoch = inputs.recovery_epoch;
    batch.input_sequence = inputs.input_sequence;
    batch.captured_at = inputs.captured_at;
    batch.recorded_wall_map = inputs.wall_map;
    batch.execution_revision=inputs.execution_revision;
    batch.reference_revision=inputs.reference_revision;
    batch.ordinary_hold=inputs.ordinary_hold;
    batch.driving_revision=inputs.driving_fsm.revision();
    batch.semantic_key = live_semantic_key;
    batch.preferred_side = inputs.preferred_side;
    if (inputs.active_command.has_value()) {
      batch.brain_hold = inputs.active_command;
      batch.speed_profile=inputs.speed_profile;
      batch.speed_generation=inputs.speed_generation;
    }
    batch.global_reference = inputs.base_reference;
    batch.opponents = inputs.opponents;
    batch.fallback_path_constraint_validator = fallback_validator;
    const auto append_template = [&](const WorkItem &source,
                                     double signed_separation_m,
                                     int template_slot) {
      if (batch.candidate_count >= kBrainTemplateLineCount)
        return;
      WorkItem work = source;
      work.request.cost_preferred_pass_separation_m =
          work.request.phase == mppi::Phase::MERGE ? 0.0 : brain_pass_offset_m_;
      work.request.nominal.d_pass_m = signed_separation_m;
      // Keep the five explicit lateral seeds, but optimize transition length,
      // speed and the two Bezier timing controls inside each line. Fixing only
      // d_pass preserves full track-width coverage without multiplying the
      // line library itself.
      work.request.bounds.minimum.d_pass_m = signed_separation_m;
      work.request.bounds.maximum.d_pass_m = signed_separation_m;
      applyGentleLateralEntry(work.request);
      // A prepared front merge uses one existing wall line per side. Its
      // geometry and timed speed proposal are fixed, so do not resample copies.
      work.request.nominal_only = work.request.front_merge_attack;
      work.request.sample_count_override = brain_samples_per_line_;
      work.request.sample_control_sequence =
          brain_control_sequence_sampling_enabled_ && !work.request.front_merge_attack;
      work.request.visualized_sample_count = brain_visualized_samples_per_line_;
      work.marker_id = template_slot + 1;
      work.template_slot = template_slot;
      batch.candidates.push_back(std::move(work));
      ++batch.candidate_count;
    };

    const bool merge_back = decision.generate_return;
    if (merge_back) {
      const int merge_side =
          batch.preferred_side != 0 ? batch.preferred_side : 1;
      const auto merge_work =
          makeBrainWork(base_command.value(), inputs.odometry.value(), inputs,
                        merge_side, phase,decision.maneuver);
      if (merge_work.has_value()) {
        append_template(*merge_work, 0.0, 2);
      }
    } else if(decision.maneuver==DrivingMode::OVERTAKE && multiple_passing_points_enabled_ &&
              !inputs.preparation_candidates.empty()) {
      for(const auto &plan:inputs.preparation_candidates) {
        auto candidate_inputs=inputs;candidate_inputs.preparation=plan;
        for(int side:{1,-1}) {
          const auto work=makeBrainWork(*base_command,*inputs.odometry,candidate_inputs,
              side,phase,decision.maneuver);
          if(work) append_template(*work,work->request.nominal.d_pass_m,
              static_cast<int>(batch.candidate_count));
        }
      }
    } else {
      const auto left =
          makeBrainWork(base_command.value(), inputs.odometry.value(), inputs,
                        1, phase,decision.maneuver);
      const auto right =
          makeBrainWork(base_command.value(), inputs.odometry.value(), inputs,
                        -1, phase,decision.maneuver);
      const auto interval_for = [](const std::optional<WorkItem> &work,
                                   int side) {
        return lateral_line_library::SideInterval{
            side, work.has_value(),
            work.has_value() ? work->request.bounds.minimum.d_pass_m : 0.0,
            work.has_value() ? work->request.bounds.maximum.d_pass_m : 0.0};
      };
      const auto library = decision.maneuver==DrivingMode::OVERTAKE ?
          lateral_line_library::wallEdges(interval_for(left,1),interval_for(right,-1)) :
          lateral_line_library::make(interval_for(left,1),interval_for(right,-1),batch.preferred_side,false);
      for (std::size_t slot = 0U; slot < library.count; ++slot) {
        const auto &line = library.lines[slot];
        const auto &source = line.side > 0 ? left.value() : right.value();
        append_template(source, line.d_m, static_cast<int>(slot));
      }
    }
    if (batch.candidate_count == 0U) {
      clearCandidateMarkers(command.header);
      bool published_safe_hold = false;
      if (inputs.active_command.has_value()) {
        const auto hold =
            refreshedBrainHold(inputs.active_command.value(), command,
                               inputs.odometry.value(), *inputs.base_reference,inputs.speed_profile.get());
        const auto validator = makeBrainCommandValidator(
            inputs, inputs.odometry.value(),
            brain_hold_minimum_opponent_clearance_m_);
        if (validateBrainCommand(hold, validator) == mppi::RejectReason::NONE) {
          if (!publish_heartbeat(inputs, hold)) return;
          publishStatus(hold, nullptr, "brain:hold_last_valid");
          published_safe_hold = true;
        }
      }
      if (!published_safe_hold) {
        const bool tracking_abort = inputs.active_command.has_value();
        const auto fallback = bounded_fallback(follow_command);
        if (!publish_heartbeat(inputs, fallback, [&] {
          if(isConnectedReturn(fallback)) adoptBrainReturn(fallback);
        })) return;
        publishStatus(follow_command, nullptr,
                      tracking_abort ? "brain:abort_opponent_clearance"
                                     : "brain:fallback_no_safe_corridor");
      }
      return;
    }
    {
      // Publish work before computing the independent heartbeat. Its existing
      // revision check prevents a slow heartbeat from overwriting a worker commit.
      std::lock_guard<std::mutex> decision_lock(decision_mutex_);
      if (recovery_active_ || batch.recovery_epoch != recovery_epoch_) return;
      std::lock_guard<std::mutex> lock(work_mutex_);
      batch.queued_at = std::chrono::steady_clock::now();
      pending_batch_ = std::move(batch);
    }
    work_cv_.notify_one();
    // Continue the accepted path (or bounded follow path) while planning.
    if (!publish_committed_hold("brain:hold_planning")) {
      const auto fallback = bounded_fallback(follow_command);
      publish_heartbeat(inputs, fallback,[&] {
        if(isConnectedReturn(fallback)) adoptBrainReturn(fallback);
      });
      publishStatus(fallback, nullptr, "brain:follow_planning");
    }
  }

  Command planBrainFollowSpeed(Command command,const mppi::PlanRequest &context) const {
    const auto geometry=temporaryReferenceFromCommand(command);
    auto planned=mppi::planLongitudinalSpeed(geometry,context,config_);
    bool preparation_selected=false;
    if(context.passing_preparation || context.ot_lane_entry) {
      const auto start=std::chrono::steady_clock::now();
      const auto choice=mppi::planAndEvaluateLongitudinalSpeed(
          mppi::ReferenceSpaceMppiPlanner(config_),geometry,context,config_);
      if(choice.evaluation.valid) {
        planned.reference=choice.reference;planned.valid=true;
        preparation_selected=choice.preparation_selected;
      }
      RCLCPP_INFO(get_logger(),
          "[MPPI_PREPARE_SPEED] plan=%lu stamp=%.9f evaluated=%zu valid=%zu selected=%d first_speed=%.6f lateral_start=%.6f elapsed_ms=%.3f",
          context.passing_preparation ? context.passing_preparation->id : 0,context.stamp_sec,choice.evaluated,choice.valid,
          preparation_selected,planned.valid ? planned.reference.points[0].speed_mps : -1.,
          context.passing_preparation ? context.passing_preparation->lateral_start_time_sec : -1.,
          std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    }
    if(planned.valid) {
      for(std::size_t i=0;i<planned.reference.count;++i)
        command.trajectory.points[i].longitudinal_velocity_mps=
            static_cast<float>(planned.reference.points[i].speed_mps);
      command.reason=preparation_selected ? "mppi_brain:preparation_speed" : "mppi_brain:longitudinal_follow";
    }
    return command;
  }

  std::shared_ptr<const PassingPointAreas> configuredPassingPointAreas(const BrainInputs &inputs) const {
    if(!passing_point_areas_enabled_) return {};
    auto areas=std::make_shared<PassingPointAreas>();
    if(inputs.wall_lines && inputs.wall_lines->world) {
      const auto projected=passingPointAreas(*inputs.wall_lines->world,
          inputs.wall_lines->length_m,inputs.wall_lines->closed,passing_point_display_config_);
      if(projected) *areas=*projected;
    }
    return areas;
  }

  void updatePassingPreparation(BrainInputs &inputs,mppi::PlanRequest &context) {
    context.passing_point_areas=configuredPassingPointAreas(inputs);
    inputs.ot_lane_entry=mppi::makeOtLaneEntryPlan(context,config_,ot_entry_config_);
    context.ot_lane_entry=inputs.ot_lane_entry;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_ot_lane_entry_=inputs.ot_lane_entry;
    }
    if(inputs.ot_lane_entry) {
      const auto &e=*inputs.ot_lane_entry;
      RCLCPP_INFO(get_logger(),
          "[MPPI_OT_ENTRY] input=%lu stamp=%.9f distance=%.6f target_kmph=%.6f predicted_kmph=%.6f reached=%d reachable_free_road=%d acceleration_start_m=%.6f",
          inputs.input_sequence,context.stamp_sec,e.distance_m,e.config.target_speed_mps*3.6,
          e.predicted_speed_mps*3.6,e.reached_entry,e.reachable_free_road,
          e.preparation.acceleration_start_station_m-e.preparation.origin_station_m);
    }
    if(front_merge_attack_enabled_) {
      inputs.preparation_candidates.clear();
      if(inputs.active_preparation && inputs.active_preparation->front_merge &&
          inputs.active_command && inputs.active_command->corridor_side!=0 &&
          mppi::preparationConnectionCurrent(*inputs.active_preparation,context,
              inputs.active_preparation->target_id)) {
        inputs.preparation=inputs.active_preparation;
        if(multiple_passing_points_enabled_) inputs.preparation_candidates={inputs.preparation};
      } else if(overtakeTargetValid(inputs) && multiple_passing_points_enabled_) {
        inputs.preparation_candidates=mppi::makeFrontMergeGoals(context,config_,
            inputs.leader_vehicle_id,inputs.input_sequence);
        inputs.preparation=inputs.preparation_candidates.empty() ? nullptr : inputs.preparation_candidates.front();
      } else if(overtakeTargetValid(inputs)) {
        inputs.preparation=mppi::makeFrontMergeGoal(context,config_,inputs.leader_vehicle_id,
            inputs.input_sequence,inputs.preparation);
      } else inputs.preparation.reset();
      mppi::applyPassingPreparation(context,inputs.preparation);
      const auto &p=inputs.preparation;
      RCLCPP_INFO(get_logger(),
          "[MPPI_FRONT_MERGE] input=%lu target=%s planned=%d owned=%d origin=%.6f pass=%.6f merge_end=%.6f supply_end=%.6f pass_time=%.6f guidance=%s side=%d plan=%lu revision=%lu prior_stamp=%.9f left_distance=%.6f right_distance=%.6f",
          inputs.input_sequence,p ? p->target_id.c_str() : inputs.leader_vehicle_id.c_str(),
          p!=nullptr,p && p->connection,p ? p->origin_station_m : -1.,
          p ? p->merge_start_station_m : -1.,p ? p->merge_end_station_m : -1.,
          p ? p->exit_station_m : -1.,p ? p->pass_time_sec : -1.,
          p ? p->prior_lap_guidance ? "prior_lap" : "current_speed_fallback" :
              context.leader_passing_prediction ? "prior_lap_no_candidate" : "no_candidate",
          p ? p->side : 0,p ? p->id : 0,p ? p->revision : 0,
          p ? p->prior_lap_source_stamp_sec : -1.,
          p ? p->candidate_minimum_center_distance_m[0] : -1.,
          p ? p->candidate_minimum_center_distance_m[1] : -1.);
      std::lock_guard<std::mutex> lock(input_mutex_);
      if(inputs.input_sequence>latest_preparation_input_) {
        latest_preparation_input_=inputs.input_sequence;latest_preparation_=inputs.preparation;
      }
      return;
    }
    mppi::PriorLapMatch match;
    for(const auto &opponent:inputs.opponents)
      if(opponent.id==inputs.leader_vehicle_id) match=opponent.prior_lap_match;
    const auto start=std::chrono::steady_clock::now();
    mppi::PreparationDiagnostics diagnostics;
    if(inputs.wall_lines && overtakeTargetValid(inputs) &&
        !(inputs.leading && isAvoidanceSpeed(std::hypot(inputs.leading->vehicle.vx_mps,inputs.leading->vehicle.vy_mps)))) {
      if(inputs.active_preparation && inputs.active_command &&
          inputs.driving_fsm.mode()==DrivingMode::OVERTAKE &&
          mppi::preparationConnectionCurrent(*inputs.active_preparation,context,inputs.leader_vehicle_id)) {
        inputs.preparation=inputs.active_preparation;
        diagnostics.reason="connection_owned";
      } else {
        inputs.preparation=mppi::makePassingPreparationGoal(context,config_,gentle_lateral_acceleration_mps2_,
            inputs.leader_vehicle_id,match,inputs.input_sequence,inputs.preparation,&diagnostics);
      }
    } else {
      inputs.preparation.reset();
      diagnostics.reason="not_fast_target";
    }
    context.passing_preparation.reset();
    mppi::applyPassingPreparation(context,inputs.preparation);
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if(inputs.input_sequence>latest_preparation_input_) {
        latest_preparation_input_=inputs.input_sequence;
        latest_preparation_=inputs.preparation;
      }
    }
    const auto &p=inputs.preparation;
    RCLCPP_INFO(get_logger(),
        "[MPPI_PREPARATION] input=%lu id=%lu revision=%lu target=%s reason=%s position_error=%.6f heading_error=%.6f pace_error=%.6f epoch=%.9f entry=%.6f exit=%.6f accel_start=%.6f lateral_start=%.6f pass=%.6f pass_time=%.6f elapsed_ms=%.3f windows=%zu trials=%zu horizon_end=%zu window_end=%zu road_end=%zu lateral_not_ready=%zu initial_gap=%.6f best_deficit=%.6f max_progress=%.6f horizon=%.6f complete_pass=%d goal_station=%.6f goal_time=%.6f relative_gain=%.6f relative_speed=%.6f",
        inputs.input_sequence,p?p->id:0,p?p->revision:0,inputs.leader_vehicle_id.c_str(),
        diagnostics.reason,match.position_error_m,match.heading_error_rad,match.pace_error_ratio,
        context.stamp_sec,p?p->entry_station_m:-1.,p?p->exit_station_m:-1.,
        p?p->acceleration_start_station_m:-1.,p?p->lateral_start_station_m:-1.,
        p?p->pass_station_m:-1.,p?p->pass_time_sec:-1.,
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(),
        diagnostics.windows,diagnostics.trials,diagnostics.horizon_exhausted,diagnostics.window_exited,
        diagnostics.road_exhausted,diagnostics.lateral_not_ready,diagnostics.initial_body_gap_m,
        diagnostics.best_body_deficit_m,diagnostics.maximum_progress_m,diagnostics.horizon_sec,
        p ? p->complete_pass : false,p ? p->targetStationM() : -1.,p ? p->targetTimeSec() : -1.,
        p ? p->relative_gain_m : 0.,p ? p->terminal_relative_speed_mps : 0.);
  }

  void receivePreparationBoostState(std::string state) {
    state.erase(std::remove_if(state.begin(), state.end(),
        [](unsigned char c) { return std::isspace(c); }), state.end());
    std::transform(state.begin(), state.end(), state.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::lock_guard<std::mutex> lock(input_mutex_);
    preparation_boost_policy_.observeState(state);
  }

  void receivePreparationBoostStatus(const std_msgs::msg::Float32MultiArray &message) {
    if (message.data.size() < 7U) return;
    for (const auto i : {1U, 5U, 6U})
      if (!std::isfinite(message.data[i]) || message.data[i] < 0. ||
          static_cast<double>(message.data[i]) >= std::numeric_limits<int>::max()) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    const int lap = static_cast<int>(message.data[1]);
    const int remaining = static_cast<int>(message.data[5]);
    const bool active = message.data[6] >= 0.5F;
    const auto &policy = preparation_boost_policy_;
    if (!policy.status_received || policy.lap != lap ||
        policy.remaining != remaining || policy.is_boosting != active)
      RCLCPP_INFO(get_logger(),
          "[MPPI_BOOST_STATUS] stamp=%.9f lap=%d remaining=%d active=%d",
          now().seconds(), lap, remaining, active);
    preparation_boost_policy_.observeStatus(lap, remaining, active);
  }

  bool maybeFirePreparationBoost(const BrainInputs &inputs) {
    if (!preparation_boost_config_.enabled || !brain_mode_ || !enabled_ ||
        shadow_only_ || !preparation_boost_pub_ || !inputs.preparation) return false;
    std::lock_guard<std::mutex> decision_lock(decision_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    if (recovery_active_ || inputs.recovery_epoch != recovery_epoch_ ||
        inputs.input_sequence != latest_preparation_input_ ||
        inputs.preparation != latest_preparation_ ||
        !odometry_ || !base_reference_) return false;
    const auto projection = projectOnTrajectory(*base_reference_,
        odometry_->pose.pose.position.x, odometry_->pose.pose.position.y);
    if (!projection.valid) return false;
    const double waypoint = projection.lower_index + projection.ratio;
    const auto &plan = *inputs.preparation;
    const auto slot = preparation_boost_policy_.select(preparation_boost_config_,
        !plan.target_id.empty() && !plan.schedule.empty(), waypoint);
    if (!slot) return false;
    // AWSIM consumes a rising edge; the following zero rearms that input.
    // Preparation is the trigger, before a connection is necessarily adopted.
    std_msgs::msg::Float32MultiArray edge;
    edge.data = {1.0F};
    preparation_boost_pub_->publish(edge);
    edge.data[0] = 0.0F;
    preparation_boost_pub_->publish(edge);
    preparation_boost_policy_.fired[*slot] = true;
    RCLCPP_INFO(get_logger(),
        "[MPPI_PREPARATION_BOOST] slot=%zu lap=%d wp=%.6f plan=%lu target=%s input=%lu stamp=%.9f remaining_before=%d",
        *slot + 1U, preparation_boost_policy_.lap, waypoint, plan.id,
        plan.target_id.c_str(), inputs.input_sequence, now().seconds(),
        preparation_boost_policy_.remaining);
    return true;
  }

  void publishPassingPointForbiddenAreas(const BrainInputs &inputs) {
    if (!passing_point_display_enabled_) return;
    std::lock_guard<std::mutex> lock(passing_point_display_mutex_);
    if (passing_point_display_published_ && displayed_passing_point_lines_==inputs.wall_lines) return;
    std_msgs::msg::Header header;
    header.frame_id=inputs.base_reference ? inputs.base_reference->header.frame_id : "map";
    auto markers=makePassingPointForbiddenMarkers(inputs.wall_lines.get(),
        passing_point_display_config_,header);
    passing_point_forbidden_pub_->publish(markers);
    displayed_passing_point_lines_=inputs.wall_lines;
    passing_point_display_published_=true;
  }

  void publishPassingPreparationMarkers(const BrainInputs &inputs,const mppi::PlanRequest *context) {
    if(!passing_preparation_enabled_) {
      passing_opportunity_pub_->publish(makePassingOpportunityMarkers(context,inputs.odometry->header,inputs.leader_vehicle_id));
      return;
    }
    const bool active=context && inputs.active_preparation &&
        mppi::preparationConnectionCurrent(*inputs.active_preparation,*context,inputs.leader_vehicle_id);
    auto markers=makePreparationPlanMarkers(
        active ? inputs.active_preparation.get() : inputs.preparation.get(),
        inputs.odometry->header,active);
    if(inputs.wall_lines) {
      for(int side=0;side<2;++side) {
        visualization_msgs::msg::Marker marker;
        marker.header=inputs.odometry->header;marker.header.frame_id="map";
        marker.ns="precomputed_wall_lines";marker.id=side;
        marker.type=visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1.;
        marker.scale.x=.06;marker.color.a=.75;
        marker.color.r=side==0 ? .1 : 1.;marker.color.g=side==0 ? .8 : .55;marker.color.b=.9;
        for(const auto &sample:inputs.wall_lines->samples) {
          geometry_msgs::msg::Point point;point.x=sample.xy[side][0];point.y=sample.xy[side][1];point.z=.15;
          marker.points.push_back(point);
        }
        markers.markers.push_back(std::move(marker));
      }
    }
    passing_opportunity_pub_->publish(markers);
  }

  void applyGentleLateralEntry(mppi::PlanRequest &request) const {
    if (!(gentle_lateral_acceleration_mps2_ > 0.0) ||
        request.phase == mppi::Phase::MERGE || request.front_merge_attack) return;
    const double length = lateral_sampling::gentleTransitionLength(
        request.ego.speed_mps, request.nominal.d_pass_m-request.anchor_d_m,
        gentle_lateral_acceleration_mps2_);
    request.nominal.l_out_m = request.bounds.minimum.l_out_m = length;
    request.bounds.maximum.l_out_m = std::min(35.0, length+5.0);
    // Degree-elevated quintic: gentle at both ends without an early-entry
    // timing extreme. Keep the existing speed and transition-length search.
    request.nominal.lateral_control_near_scale =
        request.bounds.minimum.lateral_control_near_scale =
        request.bounds.maximum.lateral_control_near_scale = 2.0/7.0;
    request.nominal.lateral_control_far_scale =
        request.bounds.minimum.lateral_control_far_scale =
        request.bounds.maximum.lateral_control_far_scale = 5.0/7.0;
  }

  void receiveVehiclePositions(
      const v2x_msgs::msg::V2XVehiclePositionArray &message) {
    const double receive_sec = now().seconds();
    std::lock_guard<std::mutex> lock(input_mutex_);
    ++vehicle_observation_sequence_;
    for (const auto &vehicle : message.vehicles) {
      if ((!own_vehicle_id_.empty() && vehicle.vehicle_id == own_vehicle_id_) ||
          vehicle.vehicle_id.empty() || !std::isfinite(vehicle.position.x) ||
          !std::isfinite(vehicle.position.y)) {
        continue;
      }
      const double stamp_sec = rclcpp::Time(vehicle.header.stamp).seconds();
      ObservedVehicle observed;
      observed.id = vehicle.vehicle_id;
      observed.x_m = vehicle.position.x;
      observed.y_m = vehicle.position.y;
      observed.sigma_x_m = std::max(0.0, vehicle.covariance.x);
      observed.sigma_y_m = std::max(0.0, vehicle.covariance.y);
      observed.stamp_sec =
          std::isfinite(stamp_sec) && stamp_sec > 0.0 ? stamp_sec : receive_sec;
      observed.receive_sec = receive_sec;
      observed.source_stamp_sec = observed.stamp_sec;
      observed.input_sequence = ++input_event_sequence_;
      const auto previous = observed_vehicles_.find(observed.id);
      if (previous != observed_vehicles_.end()) {
        if (observed.stamp_sec + 1.0e-6 < previous->second.stamp_sec) {
          continue;
        }
        const double dt = observed.stamp_sec - previous->second.stamp_sec;
        const double displacement =
            std::hypot(observed.x_m - previous->second.x_m,
                       observed.y_m - previous->second.y_m);
        if (dt > 1.0e-3 && dt <= 1.0 && displacement <= 10.0) {
          const double raw_vx = (observed.x_m - previous->second.x_m) / dt;
          const double raw_vy = (observed.y_m - previous->second.y_m) / dt;
          const double alpha = std::clamp(dt / (0.30 + dt), 0.05, 1.0);
          observed.vx_mps = previous->second.vx_mps +
                            alpha * (raw_vx - previous->second.vx_mps);
          observed.vy_mps = previous->second.vy_mps +
                            alpha * (raw_vy - previous->second.vy_mps);
        } else {
          observed.vx_mps = previous->second.vx_mps;
          observed.vy_mps = previous->second.vy_mps;
        }
        observed.history = previous->second.history;
        observed.online_motion = previous->second.online_motion;
      }
      if ((leader_lap_prediction_enabled_ || prior_lap_prediction_enabled_ || recent_motion_prediction_enabled_ ||
           online_motion_prediction_enabled_ || collection_motion_enabled_) &&
          base_reference_ && base_reference_->points.size() >= 3U) {
        const auto p = projectOnTrajectory(*base_reference_, observed.x_m, observed.y_m);
        const double length = trajectoryArcLength(*base_reference_);
        if (p.valid && length > 0.0) {
          auto history = std::make_shared<std::vector<opponent_prediction::PositionObservation>>(
              observed.history ? *observed.history :
              std::vector<opponent_prediction::PositionObservation>{});
          if (history->empty() || observed.stamp_sec + 1e-6 >= history->back().stamp) {
            if (!history->empty() && observed.stamp_sec <= history->back().stamp + 1e-6)
              history->pop_back();
            const double station = history->empty() ? p.s_m : history->back().station +
                std::remainder(p.s_m-history->back().station, length);
            history->push_back({observed.stamp_sec, observed.x_m, observed.y_m, station});
            history->erase(history->begin(), std::find_if(history->begin(), history->end(),
                [&](const auto &sample) { return (!collection_motion_enabled_ &&
                    (leader_lap_prediction_enabled_ || prior_lap_prediction_enabled_)) ?
                    sample.station >= station-2.*length : sample.stamp >= observed.stamp_sec-.65; }));
            // Distance retention alone grows forever for a parked opponent.
            constexpr std::size_t history_capacity = 8192U;
            if (history->size() > history_capacity)
              history->erase(history->begin(), history->end()-history_capacity);
            observed.history = std::move(history);
            if (online_motion_prediction_enabled_ && !collection_motion_enabled_) {
              auto learner = observed.online_motion ?
                  std::make_shared<opponent_prediction::OnlineMotionLearner>(*observed.online_motion) :
                  std::make_shared<opponent_prediction::OnlineMotionLearner>();
              const auto world=sharedReferencePoseIndex(base_reference_);
              learner->observe(*observed.history,world.get());
              observed.online_motion = std::move(learner);
            }
          }
        }
      }
      observed_vehicles_[observed.id] = std::move(observed);
    }
  }

  static PathProjection projectOnTrajectory(const Trajectory &trajectory,
                                            double x_m, double y_m) {
    return mppi::projectTrajectory(trajectory.points, [](const auto &point) {
      return std::array<double,2>{point.pose.position.x,point.pose.position.y};
    }, x_m, y_m);
  }

  static double trajectoryArcLength(const Trajectory &trajectory) {
    double length_m = 0.0;
    for (std::size_t index = 0U; index + 1U < trajectory.points.size();
         ++index) {
      const double segment_m =
          distance(trajectory.points[index].pose.position,
                   trajectory.points[index + 1U].pose.position);
      if (std::isfinite(segment_m)) {
        length_m += segment_m;
      }
    }
    return length_m;
  }

  static double signedReferenceDelta(double target_s_m, double ego_s_m,
                                     double reference_length_m, bool closed) {
    double delta_m = target_s_m - ego_s_m;
    if (closed && reference_length_m > 1.0) {
      delta_m = std::remainder(delta_m, reference_length_m);
    }
    return delta_m;
  }

  static ReferencePoseIndex referencePoseIndex(const Trajectory &trajectory) {
    return ReferencePoseIndex(trajectory.points, [](const auto &point) {
      return std::array<double, 2U>{point.pose.position.x, point.pose.position.y};
    });
  }

  std::shared_ptr<const ReferencePoseIndex> sharedReferencePoseIndex(
      const Trajectory::SharedPtr &trajectory) const {
    std::lock_guard<std::mutex> lock(reference_index_mutex_);
    if (indexed_reference_ != trajectory) {
      const bool same_geometry = indexed_reference_ && trajectory &&
          indexed_reference_->points.size() == trajectory->points.size() &&
          std::equal(indexed_reference_->points.begin(), indexed_reference_->points.end(),
              trajectory->points.begin(), [](const auto &a, const auto &b) {
                return a.pose.position.x == b.pose.position.x &&
                       a.pose.position.y == b.pose.position.y;
              });
      if (!same_geometry)
        reference_index_ = trajectory ?
            std::make_shared<const ReferencePoseIndex>(referencePoseIndex(*trajectory)) : nullptr;
      indexed_reference_ = trajectory;
    }
    // Returned indices own their geometry; a newer Reference cannot change
    // an in-flight prediction or validator built from an older snapshot.
    return reference_index_;
  }

  std::optional<Command>
  makeBrainBaseCommand(const Command &source,
                       const Trajectory &global_reference,
                       const nav_msgs::msg::Odometry &odometry,
                       double steering_rad = 0., bool maneuver_return = true) const {
    if (global_reference.points.size() < 3U)
      return std::nullopt;
    const auto &ego = odometry.pose.pose.position;
    const auto ego_projection =
        projectOnTrajectory(global_reference, ego.x, ego.y);
    if (!ego_projection.valid || !std::isfinite(ego_projection.distance_m) ||
        ego_projection.distance_m > 8.0) {
      return std::nullopt;
    }

    Command output = source;
    output.mode = "FREE_RUN";
    output.emergency_stop = false;
    output.reason = "mppi_brain:base_reference";
    output.corridor_clearance_profile = false;
    output.corridor_side = 0;
    output.wall_edge_profile = false;
    output.wall_side = 0;
    output.wall_clearance_reserve_m = 0.0F;
    output.opponent_clearance_reserve_m = 0.0F;
    output.corridor_clearance_reserve_m = 0.0F;
    output.desired_control_reserve_m = 0.0F;
    output.header.stamp=odometry.header.stamp;
    output.valid_until_sec=rclcpp::Time(odometry.header.stamp).seconds()+0.20;
    output.trajectory.header = output.header;
    output.trajectory.header.frame_id = global_reference.header.frame_id;
    output.trajectory.points.clear();

    const bool closed =
        distance(global_reference.points.front().pose.position,
                 global_reference.points.back().pose.position) < 5.0;
    const std::size_t point_count = global_reference.points.size();
    const std::size_t start = ego_projection.lower_index + 1U;
    double accumulated = 0.0;
    double reference_accumulated = 0.0;
    geometry_msgs::msg::Point previous = ego;
    geometry_msgs::msg::Point previous_reference;
    previous_reference.x =
        ego.x + ego_projection.tangent_y * ego_projection.d_m;
    previous_reference.y =
        ego.y - ego_projection.tangent_x * ego_projection.d_m;
    previous_reference.z = ego.z;
    const double connector_length_m =
        std::clamp(5.0 * std::abs(ego_projection.d_m), 8.0, 25.0);
    const double translation_x = ego.x-previous_reference.x;
    const double translation_y = ego.y-previous_reference.y;

    auto first =
        global_reference.points[closed ? start % point_count
                                       : std::min(start, point_count - 1U)];
    first.pose = odometry.pose.pose;
    first.longitudinal_velocity_mps =
        static_cast<float>(brain_cruise_speed_mps_);
    first.acceleration_mps2 =
        static_cast<float>(config_.maximum_acceleration_mps2);
    output.trajectory.points.push_back(std::move(first));

    for (std::size_t step = 0U;
         step < point_count &&
         output.trajectory.points.size() < mppi::kMaximumReferencePoints;
         ++step) {
      const std::size_t index =
          closed ? (start + step) % point_count : start + step;
      if (index >= point_count)
        break;
      auto point = global_reference.points[index];
      const double reference_ds =
          distance(previous_reference, point.pose.position);
      if (!std::isfinite(reference_ds) || reference_ds < 1.0e-3)
        continue;
      reference_accumulated += reference_ds;
      previous_reference = point.pose.position;
      const auto translation = mppi::executionConnectionTranslation(
          translation_x, translation_y, reference_accumulated, connector_length_m);
      point.pose.position.x += translation[0];
      point.pose.position.y += translation[1];
      const double ds = distance(previous, point.pose.position);
      if (!std::isfinite(ds) || ds < 1.0e-3)
        continue;
      accumulated += ds;
      if (accumulated > brain_reference_horizon_m_ &&
          output.trajectory.points.size() >= 3U) {
        break;
      }
      // MPPI owns the longitudinal target in brain mode.  Retaining a lower
      // speed embedded in the global route silently capped FREE_RUN below the
      // configured cruise target. CMA PP may still reduce this target from
      // curvature and lateral-acceleration constraints.
      point.longitudinal_velocity_mps =
          static_cast<float>(brain_cruise_speed_mps_);
      point.acceleration_mps2 =
          static_cast<float>(config_.maximum_acceleration_mps2);
      output.trajectory.points.push_back(std::move(point));
      previous = output.trajectory.points.back().pose.position;
    }
    if (output.trajectory.points.size() < 3U || accumulated < 10.0) {
      return std::nullopt;
    }
    for (std::size_t index = 1U; index + 1U < output.trajectory.points.size();
         ++index) {
      const auto &a = output.trajectory.points[index - 1U].pose.position;
      const auto &b = output.trajectory.points[index + 1U].pose.position;
      output.trajectory.points[index].pose.orientation =
          quaternionFromYaw(std::atan2(b.y - a.y, b.x - a.x));
    }
    if (output.trajectory.points.size() >= 2U) {
      const auto &a =
          output.trajectory.points[output.trajectory.points.size() - 2U]
              .pose.position;
      const auto &b = output.trajectory.points.back().pose.position;
      output.trajectory.points.back().pose.orientation =
          quaternionFromYaw(std::atan2(b.y - a.y, b.x - a.x));
    }
    output.remaining_arc_m = static_cast<float>(accumulated);
    // Ordinary route tracking must retain its route curvature feedback. A new
    // measured-curvature prefix at every heartbeat postpones the turn itself.
    // The C2 proposal belongs to a return from the committed maneuver geometry.
    if (!maneuver_return) return output;
    // The translation is only a proposal. Its zero initial derivative leaves
    // the route's tangent at the ego, even if the first pose contains ego yaw.
    // Connect XY with the same measured-pose/curvature contract as MPPI seeds,
    // before any validation; keep the route suffix and speed field unchanged.
    auto reference = temporaryReferenceFromCommand(output);
    mppi::EgoState initial;
    initial.x_m = ego.x; initial.y_m = ego.y;
    initial.yaw_rad = yawFromQuaternion(odometry.pose.pose.orientation);
    initial.steering_rad = steering_rad;
    initial.speed_mps = std::abs(odometry.twist.twist.linear.x);
    if (!mppi::entry_connector::connect(initial, config_, 4., &reference)) {
      // Failure of this optional return proposal does not invalidate the global
      // route or prevent revalidation of an otherwise usable committed hold.
      // The unconnected proposal still goes through the same execution checks.
      output.reason="mppi_brain:entry_connection_unavailable";
      return output;
    }
    mppi::updateExecutionGeometry(reference);
    accumulated = 0.;
    for (std::size_t i = 0; i < reference.count; ++i) {
      const auto &p = reference.points[i];
      auto &pose = output.trajectory.points[i].pose;
      pose.position.x = p.x_m; pose.position.y = p.y_m;
      pose.orientation = quaternionFromYaw(p.yaw_rad);
      if (i) accumulated += distance(output.trajectory.points[i-1].pose.position, pose.position);
    }
    output.remaining_arc_m = static_cast<float>(accumulated);
    output.reason="mppi_brain:return_connection";
    return output;
  }

  std::optional<FollowVehicle>
  selectFollowVehicle(const Trajectory &global_reference,
                    const Trajectory &free_run_reference,
                    const nav_msgs::msg::Odometry &odometry,
                    const BrainInputs &inputs) const {
    const auto ego_projection =
        projectOnTrajectory(global_reference, odometry.pose.pose.position.x,
                            odometry.pose.pose.position.y);
    if (!ego_projection.valid)
      return std::nullopt;
    const bool closed =
        distance(global_reference.points.front().pose.position,
                 global_reference.points.back().pose.position) < 5.0;
    const double reference_length_m = trajectoryArcLength(global_reference);
    std::optional<FollowVehicle> selected;
    for (const auto &vehicle : inputs.opponents) {
      const auto projection =
          projectOnTrajectory(global_reference, vehicle.x_m, vehicle.y_m);
      if (!projection.valid || projection.distance_m > 5.0)
        continue;
      FollowVehicle candidate;
      candidate.vehicle = vehicle;
      candidate.projection = projection;
      candidate.relative_s_m = signedReferenceDelta(
          projection.s_m, ego_projection.s_m, reference_length_m, closed);
      candidate.longitudinal_speed_mps = vehicle.vx_mps * projection.tangent_x +
                                         vehicle.vy_mps * projection.tangent_y;
      const double lateral_speed_mps = -vehicle.vx_mps * projection.tangent_y +
                                       vehicle.vy_mps * projection.tangent_x;
      const auto free_run_projection =
          projectOnTrajectory(free_run_reference, vehicle.x_m, vehicle.y_m);
      const bool current_path_conflict =
          free_run_projection.valid &&
          target_selection::pathFootprintsConflict(
              free_run_projection.distance_m, brain_footprint_radius_m_,
              brain_footprint_radius_m_, brain_target_path_conflict_margin_m_,
              std::max(vehicle.sigma_x_m, vehicle.sigma_y_m));
      bool predicted_path_conflict = false;
      if (free_run_projection.valid && candidate.relative_s_m > 0.0) {
        const double arrival_time = opponent_prediction::boundedArrivalTime(
                candidate.relative_s_m,
                std::abs(odometry.twist.twist.linear.x),
                brain_overtake_speed_mps_, config_.maximum_acceleration_mps2,
                brain_prediction_horizon_sec_);
        const double predicted_target_d_m = vehicle.prediction ?
            vehicle.prediction->at(arrival_time).d :
            projection.d_m + lateral_speed_mps * arrival_time;
        // The free-run command follows the global Reference beyond its short
        // ego connector. Preserve any measured local offset between those two
        // frames, then test where the moving opponent will be when ego arrives.
        const double free_run_d_m = projection.d_m - free_run_projection.d_m;
        predicted_path_conflict = target_selection::pathFootprintsConflict(
            std::abs(predicted_target_d_m - free_run_d_m),
            brain_footprint_radius_m_, brain_footprint_radius_m_,
            brain_target_path_conflict_margin_m_,
            std::max(vehicle.sigma_x_m, vehicle.sigma_y_m));
      }
      const bool path_conflict =
          current_path_conflict || predicted_path_conflict;
      if (candidate.relative_s_m <= 0.0 ||
          candidate.relative_s_m > brain_trigger_distance_m_ || !path_conflict) continue;
      if (!selected.has_value() ||
          candidate.relative_s_m < selected->relative_s_m) {
        selected = std::move(candidate);
      }
    }
    if (selected) {
      selected->body_gap_m = bodyFollowGap(global_reference, odometry, ego_projection,
                                          *selected, reference_length_m, closed);
    }
    return selected;
  }

  double bodyStationExtent(const Trajectory &reference,double x,double y,double yaw,
      double anchor_s,double reference_length_m,bool closed,bool front) const {
    // Following and pass completion use the same projected body corners,
    // unwrapped about each body anchor across a lap boundary.
    double result = front ? -std::numeric_limits<double>::infinity()
                          : std::numeric_limits<double>::infinity();
    for (const double longitudinal : {brain_footprint_front_m_, -brain_footprint_rear_m_}) {
      for (const double lateral : {-brain_footprint_radius_m_, brain_footprint_radius_m_}) {
        const auto corner = projectOnTrajectory(reference,
            x + std::cos(yaw) * longitudinal - std::sin(yaw) * lateral,
            y + std::sin(yaw) * longitudinal + std::cos(yaw) * lateral);
        if (!corner.valid) return std::numeric_limits<double>::quiet_NaN();
        const double offset = signedReferenceDelta(corner.s_m, anchor_s,
                                                   reference_length_m, closed);
        result = front ? std::max(result, offset) : std::min(result, offset);
      }
    }
    return result;
  }

  double bodyFollowGap(const Trajectory &reference,
                       const nav_msgs::msg::Odometry &ego,
                       const PathProjection &ego_projection,
                       const FollowVehicle &target, double reference_length_m,
                       bool closed) const {
    const auto &vehicle = target.vehicle;
    const double target_yaw = std::hypot(vehicle.vx_mps, vehicle.vy_mps) >= 0.20
        ? std::atan2(vehicle.vy_mps, vehicle.vx_mps)
        : std::atan2(target.projection.tangent_y, target.projection.tangent_x);
    const double ego_front = bodyStationExtent(reference,ego.pose.pose.position.x,ego.pose.pose.position.y,
        yawFromQuaternion(ego.pose.pose.orientation),ego_projection.s_m,reference_length_m,closed,true);
    const double target_rear = bodyStationExtent(reference,vehicle.x_m,vehicle.y_m,target_yaw,
        target.projection.s_m,reference_length_m,closed,false);
    return target.relative_s_m + target_rear - ego_front;
  }

  double targetBodyPassGap(const BrainInputs &inputs,const std::string &id) const {
    if(!inputs.base_reference || !inputs.odometry) return NAN;
    const auto target=std::find_if(inputs.opponents.begin(),inputs.opponents.end(),
        [&](const auto &v){return v.id==id;});
    if(target==inputs.opponents.end()) return NAN;
    const auto &reference=*inputs.base_reference;
    if(reference.points.size()<3U) return NAN;
    const auto &ego=*inputs.odometry;
    const auto ep=projectOnTrajectory(reference,ego.pose.pose.position.x,ego.pose.pose.position.y);
    const auto tp=projectOnTrajectory(reference,target->x_m,target->y_m);
    if(!ep.valid || !tp.valid) return NAN;
    const double length=trajectoryArcLength(reference);
    const bool closed=distance(reference.points.front().pose.position,reference.points.back().pose.position)<5.;
    const double yaw=std::hypot(target->vx_mps,target->vy_mps)>=.20 ?
        std::atan2(target->vy_mps,target->vx_mps):std::atan2(tp.tangent_y,tp.tangent_x);
    return signedReferenceDelta(ep.s_m,tp.s_m,length,closed)+
        bodyStationExtent(reference,ego.pose.pose.position.x,ego.pose.pose.position.y,
            yawFromQuaternion(ego.pose.pose.orientation),ep.s_m,length,closed,false)-
        bodyStationExtent(reference,target->x_m,target->y_m,yaw,tp.s_m,length,closed,true);
  }

  std::string selectPredictionTarget(const BrainInputs &inputs) const {
    // The adopted maneuver owns its opponent through continuation and return.
    // Fresh observations still determine whether a forecast can be built.
    if(inputs.driving_fsm.mode()==DrivingMode::OVERTAKE) {
      if(!inputs.active_target_id.empty()) return inputs.active_target_id;
      if(inputs.active_command && !inputs.prediction_target_id.empty()) return inputs.prediction_target_id;
    }
    const std::string next=inputs.leading?inputs.leading->vehicle.id:"";
    const auto &held=inputs.prediction_target_id;
    if(held.empty()) return next;
    const auto previous=std::find_if(inputs.opponents.begin(),inputs.opponents.end(),
        [&](const auto &v){return v.id==held;});
    if(previous==inputs.opponents.end()) return next.empty()?held:next;
    if(targetBodyPassGap(inputs,held)>0.) return next;
    if(next.empty() || next==held || !inputs.base_reference || !inputs.odometry) return held;
    const auto &reference=*inputs.base_reference;
    const auto &ego=*inputs.odometry;
    const auto ep=projectOnTrajectory(reference,ego.pose.pose.position.x,ego.pose.pose.position.y);
    const auto tp=projectOnTrajectory(reference,previous->x_m,previous->y_m);
    if(!ep.valid || !tp.valid) return held;
    const double length=trajectoryArcLength(reference);
    const bool closed=distance(reference.points.front().pose.position,reference.points.back().pose.position)<5.;
    FollowVehicle target;target.vehicle=*previous;target.projection=tp;
    target.relative_s_m=signedReferenceDelta(tp.s_m,ep.s_m,length,closed);
    // Do not hand off to a farther car while the current bodies still overlap
    // longitudinally, even if the held center is already behind the ego.
    return bodyFollowGap(reference,ego,ep,target,length,closed)>0.?next:held;
  }

  bool frontMergeComplete(const BrainInputs &inputs,double station,bool aligned) const {
    return inputs.active_preparation && inputs.active_preparation->front_merge &&
        station>=inputs.active_preparation->connection_end_station_m && aligned &&
        targetBodyPassGap(inputs,inputs.active_preparation->target_id)>0.;
  }

  Command makeBrainFollowCommand(const Command &base,
                                 const std::optional<FollowVehicle> &leading,
                                 const DrivingFsm &fsm) const {
    Command follow = base;
    const auto observation = leading ? std::optional<FollowObservation>(FollowObservation{
        leading->body_gap_m, std::max(0.0, leading->longitudinal_speed_mps),
        std::hypot(leading->vehicle.vx_mps,leading->vehicle.vy_mps)}) : std::nullopt;
    const double speed = fsm.freeRunSpeed(brain_cruise_speed_mps_, observation);
    for (auto &point : follow.trajectory.points) {
      point.longitudinal_velocity_mps = static_cast<float>(speed);
    }
    follow.mode = "FREE_RUN";
    follow.reason = speed < brain_cruise_speed_mps_ ? "mppi_brain:tactical_follow"
                                                  : "mppi_brain:base_reference";
    return follow;
  }

  bool footprintFree(const nav_msgs::msg::OccupancyGrid &map, double x_m,
                     double y_m, double yaw_rad,
                     const OccupancyGridWallIndex *index = nullptr) const {
    return occupancyGridFootprintFree(
        map, WallFootprintPose{x_m, y_m, yaw_rad}, brain_wall_footprint_, index);
  }

  double lateralLimit(const nav_msgs::msg::OccupancyGrid &map,
                      const mppi::BaseReferencePoint &base, int side,
                      const OccupancyGridWallIndex *index = nullptr) const {
    double limit = 0.0;
    for (double next = brain_lateral_sample_step_m_;
         limit < brain_max_track_offset_m_; next += brain_lateral_sample_step_m_) {
      const double offset = std::min(next, brain_max_track_offset_m_);
      const double signed_offset = static_cast<double>(side) * offset;
      const double x_m = base.x_m - std::sin(base.yaw_rad) * signed_offset;
      const double y_m = base.y_m + std::cos(base.yaw_rad) * signed_offset;
      if (!footprintFree(map, x_m, y_m, base.yaw_rad, index)) {
        // Refine the first blocked interval without stepping across a wall.
        // Always return a tested free footprint, with at most 1 mm of
        // numerical under-approximation instead of a whole sampling step.
        double blocked = offset;
        while (blocked - limit > 0.001) {
          const double middle = 0.5 * (limit + blocked);
          const double d = side * middle;
          if (footprintFree(map, base.x_m - std::sin(base.yaw_rad) * d,
                            base.y_m + std::cos(base.yaw_rad) * d, base.yaw_rad, index))
            limit = middle;
          else
            blocked = middle;
        }
        break;
      }
      limit = offset;
    }
    return limit;
  }

  std::shared_ptr<const PrecomputedWallLines> sharedWallLines(
      const Trajectory::SharedPtr &reference,
      const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
      const std::shared_ptr<const OccupancyGridWallIndex> &index) const {
    if(!reference || reference->points.size()<3 || !map) return {};
    const auto world=sharedReferencePoseIndex(reference);
    std::lock_guard<std::mutex> lock(wall_lines_mutex_);
    const bool same_map=wall_lines_map_ && (wall_lines_map_==map ||
        (wall_lines_map_->info.resolution==map->info.resolution &&
         wall_lines_map_->info.width==map->info.width && wall_lines_map_->info.height==map->info.height &&
         wall_lines_map_->info.origin==map->info.origin && wall_lines_map_->data==map->data));
    if(wall_lines_ && wall_lines_->world==world && same_map) {
      wall_lines_map_=map;
      return wall_lines_;
    }
    auto lines=std::make_shared<PrecomputedWallLines>();
    lines->world=world;lines->generation=++wall_lines_generation_;
    lines->length_m=trajectoryArcLength(*reference);
    lines->closed=distance(reference->points.front().pose.position,
                           reference->points.back().pose.position)<5.;
    std::vector<mppi::BaseReferencePoint> bases;
    std::array<std::vector<double>,2> widths;
    double station=0.;
    for(std::size_t i=0;i<reference->points.size();++i) {
      if(i) station+=distance(reference->points[i-1].pose.position,reference->points[i].pose.position);
      if(!bases.empty() && station<=bases.back().s_m) continue;
      const auto &p=reference->points[i].pose.position;
      const auto &a=reference->points[i ? i-1 : lines->closed ? reference->points.size()-2 : 0].pose.position;
      const auto &b=reference->points[i+1<reference->points.size() ? i+1 : lines->closed ? 1 : i].pose.position;
      mppi::BaseReferencePoint base;
      base.s_m=station;base.x_m=p.x;base.y_m=p.y;base.yaw_rad=std::atan2(b.y-a.y,b.x-a.x);
      const bool free=footprintFree(*map,base.x_m,base.y_m,base.yaw_rad,index.get());
      widths[0].push_back(free ? lateralLimit(*map,base,1,index.get()) : 0.);
      widths[1].push_back(free ? lateralLimit(*map,base,-1,index.get()) : 0.);
      bases.push_back(base);
    }
    if(bases.size()<3) return {};
    // Apply the existing 0.12 lateral slope on the whole course, including
    // the lap seam. It must not depend on the local planning window.
    for(auto &width:widths) {
      if(lines->closed) width.front()=width.back()=std::min(width.front(),width.back());
      for(int sweep=0;sweep<(lines->closed ? 2 : 1);++sweep) {
        for(std::size_t i=1;i<bases.size();++i)
          width[i]=std::min(width[i],width[i-1]+.12*(bases[i].s_m-bases[i-1].s_m));
        if(lines->closed) width.front()=width.back()=std::min(width.front(),width.back());
        for(std::size_t i=bases.size()-1;i>0;--i)
          width[i-1]=std::min(width[i-1],width[i]+.12*(bases[i].s_m-bases[i-1].s_m));
        if(lines->closed) width.front()=width.back()=std::min(width.front(),width.back());
      }
    }
    for(std::size_t i=0;i<bases.size();++i) {
      PrecomputedWallLines::Sample sample;sample.station_m=bases[i].s_m;
      for(std::size_t side=0;side<2;++side) {
        const double d=(side==0 ? 1. : -1.)*widths[side][i];
        sample.xy[side]={bases[i].x_m-std::sin(bases[i].yaw_rad)*d,
                         bases[i].y_m+std::cos(bases[i].yaw_rad)*d};
      }
      lines->samples.push_back(sample);
    }
    if(lines->closed) lines->samples.back().xy=lines->samples.front().xy;
    wall_lines_=lines;wall_lines_map_=map;
    RCLCPP_INFO(get_logger(),"[MPPI_WALL_LINES] generation=%lu lines=2 samples=%zu length=%.6f closed=%d",
        lines->generation,lines->samples.size(),lines->length_m,lines->closed);
    return lines;
  }

  double localDistance(double speed) const {
    return localHorizonDistance(speed, local_minimum_m_, local_maximum_m_,
                                local_lookahead_sec_);
  }

  double suppliedDistance(double speed) const {
    return suppliedReferenceDistance(speed, local_minimum_m_, local_maximum_m_,
        local_lookahead_sec_, reference_supply_sec_, brain_overtake_speed_mps_,
        config_.maximum_acceleration_mps2);
  }

  std::size_t localSteps(double distance_m, double speed) const {
    const double upper_speed = std::max(std::abs(speed), brain_overtake_speed_mps_);
    const double preview = std::max(config_.lookahead_min_distance_m,
                                    config_.lookahead_gain * upper_speed) +
                           config_.wheel_base_m;
    return localHorizonSteps(distance_m, preview, speed, upper_speed,
                             config_.maximum_acceleration_mps2, config_.dt_sec,
                             config_.horizon_steps);
  }

  bool overtakeTargetValid(const BrainInputs &inputs) const {
    if(!inputs.odometry || !inputs.base_reference || inputs.base_reference->points.size()<3U) return false;
    const auto target=std::find_if(inputs.opponents.begin(),inputs.opponents.end(),
        [&](const auto &o){return o.id==(front_merge_attack_enabled_ && inputs.active_preparation &&
            inputs.active_preparation->front_merge ? inputs.active_preparation->target_id : inputs.leader_vehicle_id);});
    if(target==inputs.opponents.end() ||
        !(std::hypot(target->vx_mps,target->vy_mps)>kAvoidanceMaximumLeaderSpeedMps)) return false;
    const auto &reference=*inputs.base_reference;
    const auto ego=projectOnTrajectory(reference,inputs.odometry->pose.pose.position.x,inputs.odometry->pose.pose.position.y);
    const auto other=projectOnTrajectory(reference,target->x_m,target->y_m);
    if(!ego.valid || !other.valid) return false;
    const bool closed=distance(reference.points.front().pose.position,reference.points.back().pose.position)<5.;
    const double delta=signedReferenceDelta(other.s_m,ego.s_m,trajectoryArcLength(reference),closed);
    if(front_merge_attack_enabled_ && inputs.active_preparation && inputs.active_preparation->front_merge &&
        inputs.active_command && inputs.active_command->corridor_side!=0) return true;
    return delta>0. && delta<=brain_trigger_distance_m_;
  }

  bool keepAdoptedReturn(const BrainInputs &inputs,const DrivingPlan &decision) const {
    const bool collection_replan=collection_avoidance_continuation_ &&
        decision.maneuver==DrivingMode::AVOID && decision.generate_lateral;
    return inputs.active_command && inputs.active_command->corridor_side==0 &&
        inputs.driving_fsm.mode()!=DrivingMode::FREE_RUN && !collection_replan;
  }

  bool continueCollectionAvoidance(const BrainInputs &inputs) const {
    // Existing freshness, forward selection, and path-conflict tests still
    // apply. No identity substitution or extension of the stale-track timeout.
    return collection_avoidance_continuation_ && inputs.leading &&
        inputs.driving_fsm.mode()==DrivingMode::AVOID &&
        inputs.active_target_id==inputs.leading->vehicle.id;
  }

  bool maneuverCurrent(const WorkBatch &batch,const BrainInputs &inputs) const {
    if(batch.candidates.empty()) return false;
    if(batch.candidates.front().request.phase==mppi::Phase::MERGE) return true;
    if(batch.maneuver==DrivingMode::AVOID) {
      return inputs.leading && inputs.leading->vehicle.id==batch.maneuver_target_id &&
          (isAvoidanceEligible(std::hypot(inputs.leading->vehicle.vx_mps,inputs.leading->vehicle.vy_mps),
              inputs.leading->body_gap_m,config_.follow_gap_m) ||
           (continueCollectionAvoidance(inputs) && std::isfinite(inputs.leading->body_gap_m) &&
            isAvoidanceSpeed(std::hypot(inputs.leading->vehicle.vx_mps,inputs.leading->vehicle.vy_mps))));
    }
    return batch.maneuver==DrivingMode::OVERTAKE &&
        (inputs.leader_vehicle_id==batch.maneuver_target_id ||
         (inputs.active_preparation && inputs.active_preparation->front_merge &&
          inputs.active_preparation->target_id==batch.maneuver_target_id)) && overtakeTargetValid(inputs) &&
        !(inputs.leading && isAvoidanceSpeed(std::hypot(inputs.leading->vehicle.vx_mps,inputs.leading->vehicle.vy_mps)));
  }

  bool populateDynamicObstacles(mppi::PlanRequest &request,const BrainInputs &inputs,
      const PathProjection &ego_projection,double reference_length,bool closed,bool passing_guidance=true) const {
    passing_guidance=passing_guidance && overtakeTargetValid(inputs);
    request.passing_preparation.reset();
    request.dynamic_obstacle_count=0;
    request.leader_opportunity_index=std::numeric_limits<std::size_t>::max();
    request.leader_passing_entry_m=request.leader_passing_exit_m=-1.;
    request.leader_preparation_m=request.leader_pass_time_sec=-1.;
    request.leader_pass_distance_m=-1.;
    request.leader_passing_road.reset();
    request.leader_passing_prediction.reset();
    request.world_reference=sharedReferencePoseIndex(inputs.base_reference);
    request.ot_lane_entry=inputs.ot_lane_entry &&
        inputs.ot_lane_entry->preparation.world==request.world_reference ? inputs.ot_lane_entry : nullptr;
    request.precomputed_wall_lines=inputs.wall_lines;
    request.passing_point_areas=configuredPassingPointAreas(inputs);
    request.passing_entry_m = request.passing_exit_m = -1.0;
    request.passing_speed_mps = brain_overtake_speed_mps_;
    if (config_.cost_passing_opportunity_weight > 0.0) {
      // Smooth global-road curvature over 4 m; retain the global speed advice
      // only for opportunity scoring, never as an execution speed override.
      double entry = -1.0, speed_sum = 0.0, count = 0.0;
      for (double s=0.0; s<=50.0; s+=1.0) {
        const auto a=request.world_reference->pose(ego_projection.s_m+s-2.0,0.0);
        const auto b=request.world_reference->pose(ego_projection.s_m+s+2.0,0.0);
        const auto center=request.world_reference->pose(ego_projection.s_m+s,0.0);
        if (!a || !b || !center) break;
        const double curvature=std::abs(std::remainder((*b)[2]-(*a)[2],2.0*M_PI))/4.0;
        const auto projection=projectOnTrajectory(*inputs.base_reference,(*center)[0],(*center)[1]);
        if (!projection.valid) break;
        const double speed=std::min(brain_overtake_speed_mps_,std::abs(static_cast<double>(
            inputs.base_reference->points[projection.lower_index].longitudinal_velocity_mps)));
        const bool opportunity=curvature<=0.035 && speed>=3.0 &&
            speed*speed*curvature<=config_.maximum_lateral_acceleration_mps2;
        if (opportunity) {
          if(entry<0.0) entry=s;
          speed_sum+=speed; count+=1.0;
        }
        if (!opportunity || s==50.0) {
          if (entry>=0.0 && s-entry>=8.0) {
            request.passing_entry_m=entry; request.passing_exit_m=s;
            request.passing_speed_mps=speed_sum/count;
            break;
          }
          entry=-1.0; speed_sum=count=0.0;
        }
      }
    }
    for(const auto &opponent:inputs.opponents) {
      if(request.dynamic_obstacle_count>=mppi::kMaximumDynamicObstacles) return false;
      const auto projection=projectOnTrajectory(*inputs.base_reference,opponent.x_m,opponent.y_m);
      if(!projection.valid) return false;
      auto &o=request.dynamic_obstacles[request.dynamic_obstacle_count++];
      o.s_m=signedReferenceDelta(projection.s_m,ego_projection.s_m,reference_length,closed);
      o.global_reference_s_m=projection.s_m;
      o.d_m=projection.d_m;
      o.longitudinal_speed_mps=opponent.vx_mps*projection.tangent_x+opponent.vy_mps*projection.tangent_y;
      o.lateral_speed_mps=-opponent.vx_mps*projection.tangent_y+opponent.vy_mps*projection.tangent_x;
      o.longitudinal_acceleration_bound_mps2=0.5;
      o.longitudinal_uncertainty_m=std::max(opponent.sigma_x_m,opponent.sigma_y_m);
      o.lateral_uncertainty_m=o.longitudinal_uncertainty_m;
      o.prediction=opponent.prediction;
      if(front_merge_attack_enabled_ && opponent.id==(inputs.active_preparation &&
          inputs.active_preparation->front_merge ? inputs.active_preparation->target_id : inputs.leader_vehicle_id))
        request.leader_opportunity_index=request.dynamic_obstacle_count-1U;
      if (passing_guidance && opponent.id==inputs.leader_vehicle_id && opponent.leader_lap_prediction) {
        request.leader_passing_prediction=opponent.leader_lap_prediction;
        std::vector<mppi::PassingRoadSample> road;
        for (double s=0.; s<=80.; s+=1.) {
          const auto a=request.world_reference->pose(ego_projection.s_m+s-2.,0.);
          const auto b=request.world_reference->pose(ego_projection.s_m+s+2.,0.);
          const auto center=request.world_reference->pose(ego_projection.s_m+s,0.);
          if (!a || !b || !center) break;
          const auto p=projectOnTrajectory(*inputs.base_reference,(*center)[0],(*center)[1]);
          if (!p.valid) break;
          const double speed=std::min(brain_overtake_speed_mps_,std::abs(static_cast<double>(
              inputs.base_reference->points[p.lower_index].longitudinal_velocity_mps)));
          road.push_back({s,speed,std::remainder((*b)[2]-(*a)[2],2.*M_PI)/4.});
        }
        request.leader_passing_road=std::make_shared<const mppi::PassingRoadProfile>(
            mppi::passingRoadProfile(road,config_));
        auto guidance=o;
        guidance.prediction=request.leader_passing_prediction;
        const auto window=mppi::selectLeaderPassingOpportunity(
            *request.leader_passing_road,guidance,request.ego,config_,gentle_lateral_acceleration_mps2_);
        request.leader_opportunity_index=request.dynamic_obstacle_count-1U;
        request.leader_passing_entry_m=window.entry;
        request.leader_passing_exit_m=window.exit;
        request.leader_preparation_m=window.preparation;
        request.leader_pass_time_sec=window.pass_time;
        request.leader_pass_distance_m=window.pass_distance;
      }
      if(std::hypot(o.longitudinal_speed_mps,o.lateral_speed_mps)>=0.20)
        o.heading_relative_to_reference_rad=std::atan2(o.lateral_speed_mps,o.longitudinal_speed_mps);
    }
    if (config_.cost_approach_speed_weight > 0.0) {
      std::optional<double> matching;
      const double relative_yaw = request.ego.yaw_rad -
          std::atan2(ego_projection.tangent_y, ego_projection.tangent_x);
      for (std::size_t i = 0; i < request.dynamic_obstacle_count; ++i) {
        const auto target = mppi::approachSpeedTarget(request.dynamic_obstacles[i],
            request.ego.s_m, request.ego.d_m, relative_yaw, 0.0, config_);
        if (target) matching = matching ? std::min(*matching, *target) : *target;
      }
      request.preferred_matching_speed_mps = std::min(brain_overtake_speed_mps_,
          matching.value_or(request.ego.speed_mps));
    }
    if(passing_guidance && front_merge_attack_enabled_ && inputs.preparation) {
      mppi::applyPassingPreparation(request,inputs.preparation);
    } else if(passing_guidance && passing_preparation_enabled_ && inputs.preparation && inputs.preparation->target_id==inputs.leader_vehicle_id) {
      const auto target=std::find_if(inputs.opponents.begin(),inputs.opponents.end(),
          [&](const auto &o){return o.id==inputs.leader_vehicle_id;});
      if(target!=inputs.opponents.end() && (target->prior_lap_match.usable ||
          mppi::preparationConnectionCurrent(*inputs.preparation,request,inputs.leader_vehicle_id)))
        mppi::applyPassingPreparation(request,inputs.preparation);
    }
    return true;
  }

  // Build a dynamics-only context for a held/fallback path. The supplied path
  // is NOT used as the obstacle Frenet frame. Both remain in the immutable
  // global Reference, with t=0 at the measured odometry stamp.
  std::optional<mppi::PlanRequest> executionContext(const BrainInputs &inputs,
      const nav_msgs::msg::Odometry &odometry) const {
    if(!inputs.base_reference || !inputs.wall_map || inputs.base_reference->points.size()<3U)
      return std::nullopt;
    const auto &global=*inputs.base_reference;
    const auto projection=projectOnTrajectory(global,odometry.pose.pose.position.x,odometry.pose.pose.position.y);
    if(!projection.valid) return std::nullopt;
    mppi::PlanRequest r;
    r.valid=true; r.side=1; r.phase=mppi::Phase::OVERTAKE;
    r.stamp_sec=rclcpp::Time(odometry.header.stamp).seconds();
    r.ego.x_m=odometry.pose.pose.position.x; r.ego.y_m=odometry.pose.pose.position.y;
    r.ego.yaw_rad=yawFromQuaternion(odometry.pose.pose.orientation);
    r.ego.speed_mps=std::abs(odometry.twist.twist.linear.x); r.ego.d_m=projection.d_m;
    r.ego.steering_rad=inputs.steering_fresh?inputs.steering_rad:0.0;
    r.ego.yaw_rate_radps = odometry.twist.twist.angular.z;
    r.rotation_prediction = inputs.rotation_prediction;
    r.prior_execution_reference=inputs.prior_execution_reference;
    r.reference_activation_delay_sec=inputs.reference_activation_delay_sec;
    if(inputs.pending_steering_targets.valid) {
      r.ego.pending_steering_targets_rad=inputs.pending_steering_targets.targets_rad;
      r.ego.pending_steering_target_count=inputs.pending_steering_targets.count;
    }
    auto &first=r.base_reference[0];
    first.x_m=r.ego.x_m+projection.tangent_y*projection.d_m;
    first.y_m=r.ego.y_m-projection.tangent_x*projection.d_m;
    first.yaw_rad=std::atan2(projection.tangent_y,projection.tangent_x);
    first.speed_mps=brain_overtake_speed_mps_;
    first.minimum_d_m=-brain_max_track_offset_m_; first.maximum_d_m=brain_max_track_offset_m_;
    r.base_reference_count=1;
    const bool closed=distance(global.points.front().pose.position,global.points.back().pose.position)<5.0;
    const double validation_distance=localDistance(r.ego.speed_mps);
    const double limit=suppliedDistance(r.ego.speed_mps);
    const std::size_t start=projection.lower_index+1U;
    for(std::size_t step=0;step<global.points.size() && r.base_reference_count<mppi::kMaximumReferencePoints;++step) {
      const std::size_t index=closed?(start+step)%global.points.size():start+step;
      if(index>=global.points.size()) break;
      const auto &point=global.points[index].pose.position;
      const auto &a=r.base_reference[r.base_reference_count-1];
      const double ds=std::hypot(point.x-a.x_m,point.y-a.y_m);
      if(!std::isfinite(ds)) return std::nullopt;
      if(ds<1e-3) continue;
      const double remaining=limit-a.s_m;
      if(remaining<=1e-9) break;
      auto &b=r.base_reference[r.base_reference_count++];
      const double t=std::min(1.0,remaining/ds);
      b.s_m=a.s_m+t*ds; b.x_m=a.x_m+t*(point.x-a.x_m); b.y_m=a.y_m+t*(point.y-a.y_m);
      b.yaw_rad=std::atan2(point.y-a.y_m,point.x-a.x_m); b.speed_mps=brain_overtake_speed_mps_;
      b.minimum_d_m=-brain_max_track_offset_m_; b.maximum_d_m=brain_max_track_offset_m_;
    }
    if(r.base_reference_count<3U) return std::nullopt;
    r.horizon_steps_override=localSteps(std::min(validation_distance,
        r.base_reference[r.base_reference_count-1].s_m),r.ego.speed_mps);
    r.curvature_evaluation_distance_m=validation_distance;
    if(r.horizon_steps_override<2U || !populateDynamicObstacles(r,inputs,projection,trajectoryArcLength(global),closed))
      return std::nullopt;
    r.path_constraint_validator=makeBrainPathConstraintValidator(inputs,odometry,0.0);
    r.path_constraint_geometry_only = true;
    r.path_constraint_execution_prefix = true;
    r.rollout_constraint_validator=makeBrainRolloutConstraintValidator(inputs);
    return r;
  }

  mppi::PathConstraintValidator makeBrainCommandValidator(const BrainInputs &inputs,
      const nav_msgs::msg::Odometry &odometry,double required_clearance) const {
    // Keep the old parameter for configuration compatibility, not a second
    // opponent clock. Physical bounds are identical to candidate evaluation.
    (void)required_clearance;
    const auto context=executionContext(inputs,odometry);
    const auto config=config_;
    const auto logger=get_logger();
    const auto shape=inputs.active_command ? inputs.active_command->generation : 0;
    const auto speed_generation=inputs.speed_generation;
    return [context,config,logger,shape,speed_generation](const mppi::TemporaryReference &reference) {
      if(!context) return mppi::RejectReason::INVALID_INPUT;
      const auto evaluation=Mppi(config).evaluateExecutionReference(reference,*context);
      if (!evaluation.valid) RCLCPP_INFO(logger,
          "[MPPI_COMMAND_REJECT] stamp=%.9f shape_source=%lu speed_generation=%lu first=%.6f reason=%s stage=%s time=%.6f x=%.6f y=%.6f reference_check_m=%.6f",
          context->stamp_sec, static_cast<unsigned long>(shape), static_cast<unsigned long>(speed_generation),
          reference.count ? reference.points[0].speed_mps : 0., Mppi::toString(evaluation.reject_reason),
          evaluation.reject_stage, evaluation.reject_time_sec, evaluation.reject_x_m, evaluation.reject_y_m,
          evaluation.validated_reference_distance_m);
      return evaluation.reject_reason;
    };
  }

  std::optional<WorkItem> makeBrainWork(const Command &base_command,
                                        const nav_msgs::msg::Odometry &odometry,
                                        const BrainInputs &inputs,
                                        int side,
                                        mppi::Phase phase = mppi::Phase::OVERTAKE,
                                        DrivingMode maneuver = DrivingMode::AVOID) const {
    if (inputs.wall_map == nullptr || side == 0)
      return std::nullopt;
    WorkItem work;
    work.maneuver=maneuver;
    work.command = base_command;
    work.odometry = odometry;
    work.wall_map = inputs.wall_map;
    work.brain_owned = true;
    auto &request = work.request;
    request.generation = base_command.generation;
    request.stamp_sec = rclcpp::Time(base_command.header.stamp).seconds();
    request.complete_maneuver = false;
    request.side = side;
    const bool returning = phase == mppi::Phase::MERGE;
    const bool fixed_wall_line = maneuver == DrivingMode::OVERTAKE && !returning;
    if(fixed_wall_line && !inputs.wall_lines) return std::nullopt;
    request.connect_to_wall_line=fixed_wall_line;
    request.front_merge_attack=front_merge_attack_enabled_ && fixed_wall_line;
    request.compare_passing_points=multiple_passing_points_enabled_ && request.front_merge_attack;
    request.complete_maneuver=request.front_merge_attack;
    if(request.front_merge_attack && !inputs.preparation) return std::nullopt;
    if(request.front_merge_attack && inputs.preparation->prior_lap_guidance && inputs.preparation->side!=0 &&
        inputs.preparation->side!=side) return std::nullopt;
    request.phase = phase;
    request.semantic_key = maneuverSemanticKey(phase,maneuver);
    request.minimum_speed_mps = 0.0;
    request.preferred_matching_speed_mps = std::min(
        std::abs(odometry.twist.twist.linear.x), brain_overtake_speed_mps_);
    request.sample_staged_speed = config_.longitudinal_planning_enabled || !returning;
    request.defer_warm_update = true;
    request.cartesian_measured_prefix = true;
    request.ego.x_m = odometry.pose.pose.position.x;
    request.ego.y_m = odometry.pose.pose.position.y;
    request.ego.yaw_rad = yawFromQuaternion(odometry.pose.pose.orientation);
    request.ego.speed_mps = std::abs(odometry.twist.twist.linear.x);
    request.ego.steering_rad =
        inputs.steering_fresh ? inputs.steering_rad : 0.0;
    if (inputs.pending_steering_targets.valid) {
      request.ego.pending_steering_targets_rad =
          inputs.pending_steering_targets.targets_rad;
      request.ego.pending_steering_target_count =
          inputs.pending_steering_targets.count;
    }
    request.ego.acceleration_mps2 = 0.0;
    request.ego.yaw_rate_radps = odometry.twist.twist.angular.z;
    request.rotation_prediction = inputs.rotation_prediction;
    request.prior_execution_reference=inputs.prior_execution_reference;
    request.reference_activation_delay_sec=inputs.reference_activation_delay_sec;

    const double local_distance_m = localDistance(request.ego.speed_mps);
    double supplied_distance_m = suppliedDistance(request.ego.speed_mps);
    if(request.front_merge_attack) {
      const auto &plan=*inputs.preparation;
      const auto station=mppi::preparationStation(plan,request.ego.x_m,request.ego.y_m);
      if(!station) return std::nullopt;
      const double origin=*station;
      // A refreshed connector may merge later than the initial goal. Binding
      // requires its endpoint before exit; keep a full normal supply beyond
      // that exit so execution cannot run out just before completing the merge.
      supplied_distance_m=std::max(supplied_distance_m,plan.exit_station_m-origin+supplied_distance_m);
    }
    request.curvature_evaluation_distance_m = local_distance_m;
    request.horizon_steps_override = localSteps(local_distance_m, request.ego.speed_mps);
    if (request.horizon_steps_override < 2U) return std::nullopt;

    const auto &global_reference = *inputs.base_reference;
    const auto ego_projection =
        projectOnTrajectory(global_reference, request.ego.x_m, request.ego.y_m);
    if (!ego_projection.valid || ego_projection.distance_m > 8.0)
      return std::nullopt;
    request.wall_line_origin_station_m=ego_projection.s_m;

    // Optimize in the immutable global-Reference Frenet frame.  The output
    // still begins at the measured kart pose through anchor_d_m below, but the
    // reference itself must not be an ego-to-centerline connector. Using that
    // connector as the Frenet base redefined the pass offset every cycle and
    // periodically pulled an ongoing overtake back onto the centerline.
    work.command.trajectory.points.clear();
    const bool closed =
        distance(global_reference.points.front().pose.position,
                 global_reference.points.back().pose.position) < 5.0;
    const double global_reference_length_m =
        trajectoryArcLength(global_reference);
    const std::size_t point_count = global_reference.points.size();
    const std::size_t start = ego_projection.lower_index + 1U;
    double accumulated_s = 0.0;
    geometry_msgs::msg::Point previous;
    previous.x = ego_projection.x_m;
    previous.y = ego_projection.y_m;
    previous.z = odometry.pose.pose.position.z;
    {
      const std::size_t source_index =
          closed ? start % point_count : std::min(start, point_count - 1U);
      auto source = global_reference.points[source_index];
      source.pose.position = previous;
      source.pose.orientation = quaternionFromYaw(
          std::atan2(ego_projection.tangent_y, ego_projection.tangent_x));
      source.longitudinal_velocity_mps =
          static_cast<float>(brain_overtake_speed_mps_);
      source.acceleration_mps2 =
          static_cast<float>(config_.maximum_acceleration_mps2);
      work.command.trajectory.points.push_back(source);
      const std::size_t index = request.base_reference_count++;
      work.source_indices[index] = index;
      auto &base = request.base_reference[index];
      base.s_m = 0.0;
      base.x_m = previous.x;
      base.y_m = previous.y;
      base.yaw_rad =
          std::atan2(ego_projection.tangent_y, ego_projection.tangent_x);
      base.speed_mps = brain_overtake_speed_mps_;
      base.corridor_d_m = 0.0;
    }
    if(request.front_merge_attack) {
      // The native smoothed Reference is denser than the long attack needs.
      // Sample its entire station interval within capacity, including the
      // post-merge supply, instead of silently cutting it at point 384.
      const double spacing=std::max(config_.reference_spacing_m,
          supplied_distance_m/static_cast<double>(mppi::kMaximumReferencePoints-1U));
      const auto intervals=std::min(mppi::kMaximumReferencePoints-1U,
          static_cast<std::size_t>(std::ceil(supplied_distance_m/spacing)));
      for(std::size_t i=1;i<=intervals;++i) {
        const double station=supplied_distance_m*static_cast<double>(i)/intervals;
        const auto pose=inputs.wall_lines->world->pose(ego_projection.s_m+station,0.);
        if(!pose) return std::nullopt;
        auto source=work.command.trajectory.points.front();
        source.pose.position.x=(*pose)[0];source.pose.position.y=(*pose)[1];
        source.pose.orientation=quaternionFromYaw((*pose)[2]);
        work.command.trajectory.points.push_back(source);
        const auto index=request.base_reference_count++;
        work.source_indices[index]=index;
        auto &base=request.base_reference[index];
        base.s_m=station;base.x_m=(*pose)[0];base.y_m=(*pose)[1];base.yaw_rad=(*pose)[2];
        base.speed_mps=brain_overtake_speed_mps_;
        accumulated_s=station;
      }
    }
    for (std::size_t step = 0U;
         !request.front_merge_attack && step < point_count &&
         request.base_reference_count < mppi::kMaximumReferencePoints;
         ++step) {
      const std::size_t source_index =
          closed ? (start + step) % point_count : start + step;
      if (source_index >= point_count)
        break;
      auto source = global_reference.points[source_index];
      const double ds = distance(previous, source.pose.position);
      if (!std::isfinite(ds) || ds < 1.0e-3)
        continue;
      const double remaining = supplied_distance_m - accumulated_s;
      if (remaining <= 1.0e-9) break;
      if (ds > remaining) {
        const double ratio = remaining / ds;
        source.pose.position.x = previous.x + ratio * (source.pose.position.x - previous.x);
        source.pose.position.y = previous.y + ratio * (source.pose.position.y - previous.y);
        source.pose.position.z = previous.z + ratio * (source.pose.position.z - previous.z);
      }
      accumulated_s += std::min(ds, remaining);
      source.longitudinal_velocity_mps =
          static_cast<float>(brain_overtake_speed_mps_);
      source.acceleration_mps2 =
          static_cast<float>(config_.maximum_acceleration_mps2);
      work.command.trajectory.points.push_back(source);
      const std::size_t index = request.base_reference_count++;
      work.source_indices[index] = index;
      auto &base = request.base_reference[index];
      base.s_m = accumulated_s;
      base.x_m = source.pose.position.x;
      base.y_m = source.pose.position.y;
      base.yaw_rad = yawFromQuaternion(source.pose.orientation);
      // MPPI owns the longitudinal proposal in this independent entry;
      // upstream geometry is not an independently authoritative speed plan.
      base.speed_mps = brain_overtake_speed_mps_;
      base.corridor_d_m = 0.0;
      previous = source.pose.position;
    }
    if (request.base_reference_count < 3U || accumulated_s < 15.0) {
      return std::nullopt;
    }
    work.requested_horizon_m = supplied_distance_m;
    work.actual_horizon_m = accumulated_s;
    work.reference_capacity_limited =
        request.base_reference_count == mppi::kMaximumReferencePoints &&
        accumulated_s + 1.0e-6 < supplied_distance_m;
    request.horizon_steps_override = localSteps(std::min(local_distance_m, accumulated_s), request.ego.speed_mps);
    if (request.horizon_steps_override < 2U) return std::nullopt;
    for (std::size_t index = 0U; index < request.base_reference_count;
         ++index) {
      const std::size_t lower = index == 0U ? 0U : index - 1U;
      const std::size_t upper =
          std::min(index + 1U, request.base_reference_count - 1U);
      const auto &a = request.base_reference[lower];
      const auto &b = request.base_reference[upper];
      auto &base = request.base_reference[index];
      base.yaw_rad = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
    }
    for (std::size_t index = 1U; index + 1U < request.base_reference_count;
         ++index) {
      const auto &a = request.base_reference[index - 1U];
      const auto &b = request.base_reference[index + 1U];
      const double ds = b.s_m - a.s_m;
      request.base_reference[index].curvature_1pm =
          std::clamp(std::remainder(b.yaw_rad - a.yaw_rad, 2.0 * M_PI) /
                         std::max(ds, 1.0e-3),
                     -base_curvature_limit_1pm_, base_curvature_limit_1pm_);
    }
    request.base_reference.front().curvature_1pm =
        request.base_reference[1U].curvature_1pm;
    request.base_reference[request.base_reference_count - 1U].curvature_1pm =
        request.base_reference[request.base_reference_count - 2U].curvature_1pm;

    // At a clamped corner the remaining polyline can start on the outgoing
    // segment. Express ego/anchor in the very same frame used by core costs;
    // the global incoming normal need not equal that outgoing normal.
    const auto local_ego = mppi::projectOnBase(request.base_reference.data(),
        request.base_reference_count, request.ego.x_m, request.ego.y_m);
    request.ego.s_m = local_ego.s;
    request.ego.d_m = local_ego.d;
    request.anchor_s_m = std::max(0.5, request.ego.speed_mps * 0.20);
    request.anchor_d_m = local_ego.d;
    request.corridor_nominal_d_m = 0.0;

    for (std::size_t index = 0U; index < request.base_reference_count;
         ++index) {
      auto &base = request.base_reference[index];
      if (!footprintFree(*inputs.wall_map, base.x_m, base.y_m, base.yaw_rad,
                         inputs.wall_index.get())) {
        return std::nullopt;
      }
      const double left = fixed_wall_line ? brain_max_track_offset_m_ :
          lateralLimit(*inputs.wall_map, base, 1, inputs.wall_index.get());
      const double right = fixed_wall_line ? brain_max_track_offset_m_ :
          lateralLimit(*inputs.wall_map, base, -1, inputs.wall_index.get());
      base.minimum_d_m = -right;
      base.maximum_d_m = left;
      // Freeze the prefix to the measured/accepted global Frenet offset. This
      // is the state carried between replans; zero would mean "return to the
      // Reference" and caused the visible intermittent snap-back.
      base.active_d_m = local_ego.d;
      base.active_d_valid = true;
    }
    if(inputs.active_command) {
      const auto remaining=refreshedBrainHold(*inputs.active_command,base_command,odometry,
          global_reference,inputs.speed_profile.get());
      mppi::setExecutionHistory(request,temporaryReferenceFromCommand(remaining));
      if (request.execution_history_field->count>1)
        request.curvature_evaluation_distance_m=std::min(local_distance_m,
            request.execution_history_field->points[request.execution_history_field->count-1].s_m);
    }
    // The core connects the Cartesian entry once, before curvature, speed
    // caps and feasibility. Do not stack a separate Frenet prefix conditioner.

    lateral_sampling::Interval lateral_interval;
    double widest_pass_s_m = request.anchor_s_m;
    if(fixed_wall_line) {
      const double join_station=std::min(inputs.wall_lines->closed ? INFINITY : inputs.wall_lines->length_m,
          ego_projection.s_m+request.ego.speed_mps*2.5);
      const auto xy=inputs.wall_lines->at(join_station,side);
      const auto center=inputs.wall_lines->world->pose(join_station,0.);
      if(!xy || !center) return std::nullopt;
      const double separation=side*(-((*xy)[0]-(*center)[0])*std::sin((*center)[2])+
          ((*xy)[1]-(*center)[1])*std::cos((*center)[2]));
      if(!(separation>0.)) return std::nullopt;
      lateral_interval.feasible=true;
      lateral_interval.minimum_separation_m=separation;
      if(request.front_merge_attack)
        lateral_interval.minimum_separation_m=std::min(brain_min_pass_offset_m_,separation);
      lateral_interval.maximum_separation_m=separation;
      lateral_interval.nominal_separation_m=separation;
      request.pass_profile_scale_m=side*separation;
      widest_pass_s_m=request.anchor_s_m+request.ego.speed_mps*2.5;
    } else {
    if (!returning) {
      request.wall_line_start_s_m = request.anchor_s_m;
      request.wall_line_end_s_m = std::min(accumulated_s, local_distance_m);
      double width = 0.0;
      for (std::size_t index = 0; index < request.base_reference_count; ++index) {
        const auto &base = request.base_reference[index];
        if (base.s_m < request.anchor_s_m || base.s_m > local_distance_m) continue;
        const double available = side > 0 ? base.maximum_d_m : -base.minimum_d_m;
        if (available > width) {
          width = available;
          widest_pass_s_m = base.s_m;
        }
      }
      if (!(width > 0.0)) return std::nullopt;
      lateral_interval.feasible = true;
      lateral_interval.maximum_separation_m = width;
      lateral_interval.minimum_separation_m = std::min(brain_min_pass_offset_m_, width);
      lateral_interval.nominal_separation_m = std::clamp(brain_pass_offset_m_,
          lateral_interval.minimum_separation_m, width);
      request.pass_profile_origin_d_m = 0.0;
      request.pass_profile_scale_m = static_cast<double>(side) * width;
      request.sample_lateral_bounds = true;
    }

    for (std::size_t index = 0U; index < request.base_reference_count;
         ++index) {
      auto &base = request.base_reference[index];
      if (returning) {
        base.pass_d_m = 0.0;
      } else {
        base.pass_origin_d_m = 0.0;
        base.pass_origin_d_valid = true;
        const double local_available_separation_m =
            side > 0 ? base.maximum_d_m - base.pass_origin_d_m
                     : base.pass_origin_d_m - base.minimum_d_m;
        const double local_profile_separation_m =
            std::clamp(local_available_separation_m, 0.0,
                       lateral_interval.maximum_separation_m);
        base.pass_d_m = base.pass_origin_d_m +
                        static_cast<double>(side) * local_profile_separation_m;
      }
      base.pass_d_m =
          std::clamp(base.pass_d_m, base.minimum_d_m, base.maximum_d_m);
      base.pass_d_valid = true;
    }
    constexpr double kMaximumPassProfileSlope = 0.12;
    const auto limit_profile_growth = [&](std::size_t from, std::size_t to) {
      const double ds = std::abs(request.base_reference[to].s_m -
                                 request.base_reference[from].s_m);
      const double previous = request.base_reference[from].pass_d_m;
      request.base_reference[to].pass_d_m =
          std::clamp(request.base_reference[to].pass_d_m,
                     previous - kMaximumPassProfileSlope * ds,
                     previous + kMaximumPassProfileSlope * ds);
    };
    for (std::size_t index = 1U; index < request.base_reference_count;
         ++index) {
      limit_profile_growth(index - 1U, index);
    }
    for (std::size_t index = request.base_reference_count - 1U; index > 0U;
         --index) {
      limit_profile_growth(index, index - 1U);
    }
    for (std::size_t index = 0U; index < request.base_reference_count;
         ++index) {
      auto &base = request.base_reference[index];
      base.pass_d_m =
          std::clamp(base.pass_d_m, base.minimum_d_m, base.maximum_d_m);
    }
    }

    // Keep high-speed transitions gentle; a slow kart needs shorter proposals
    // to enter a nearby gap. All proposals retain the same delayed PP rollout.
    // Place the terminal lateral control point near the widest reachable
    // section. The rollout still time-aligns the moving opponent, so this is
    // an opportunity proposal rather than an assumption that the pass is safe.
    const double minimum_shift_length =
        lateral_sampling::minimumTransitionLength(request.ego.speed_mps);
    const double shift_length = std::clamp(
        widest_pass_s_m - request.anchor_s_m, minimum_shift_length,
        std::clamp(request.ego.speed_mps * 2.5, minimum_shift_length, 30.0));
    // Returning a full kart-width offset in 8 m saturates steering at the
    // 6--10 m/s passing speed.  Besides producing a visibly oscillatory merge,
    // that leaves the next receding-horizon request outside the MPPI vehicle
    // and PP-trackability limits.  Scale the post-pass merge with measured
    // speed; OVERTAKE does not use l_merge while complete_maneuver is false.
    const double merge_length =
        returning ? std::clamp(request.ego.speed_mps * 2.5, 15.0, 22.0)
                       : 8.0;
    const double available_hold_length_m =
        std::max(5.0, accumulated_s - request.anchor_s_m - shift_length -
                          merge_length - 0.5);
    const double hold_length =
        std::min(45.0, available_hold_length_m);
    // Publish the local prefix of a gentle shift/merge. It need not finish in
    // this window; completion remains based on the measured lateral state.
    const double signed_offset =
        returning
            ? 0.0
            : static_cast<double>(side) * lateral_interval.nominal_separation_m;
    // Ten metres per second is the overtake advisory target. The sampled
    // target and the certified output profile both retain the configured
    // passing floor; infeasible candidates are rejected by the rollout.
    constexpr double kMaximumBrainSpeedScale = 1.00;
    const double nominal_speed_scale = 1.0;
    const double minimum_speed_scale = 0.0;
    request.nominal = {
        signed_offset,       shift_length, hold_length, merge_length,
        nominal_speed_scale, 0.0,          1.0};
    const double minimum_signed_separation_m =
        returning
            ? 0.0
            : static_cast<double>(side) * lateral_interval.minimum_separation_m;
    const double maximum_signed_separation_m =
        returning
            ? 0.0
            : static_cast<double>(side) * lateral_interval.maximum_separation_m;
    request.bounds.minimum = mppi::Parameters{
        side > 0 ? minimum_signed_separation_m : maximum_signed_separation_m,
        // Explore an earlier lateral entry alongside the gentle nominal and
        // longer transitions. The delayed vehicle rollout still certifies it.
        returning ? minimum_shift_length : std::min(minimum_shift_length, 12.0),
        hold_length,
        merge_length,
        minimum_speed_scale,
        0.0,
        returning ? 1.0 : 0.55};
    request.bounds.maximum = mppi::Parameters{
        side > 0 ? maximum_signed_separation_m : minimum_signed_separation_m,
        std::min(35.0, shift_length + 8.0),
        hold_length,
        merge_length,
        kMaximumBrainSpeedScale,
        returning ? 0.0 : 0.45,
        1.0};

    if(request.front_merge_attack) {
      const auto &plan=*inputs.preparation;
      const auto station=mppi::preparationStation(plan,request.ego.x_m,request.ego.y_m);
      if(!station) return std::nullopt;
      const double origin=*station;
      if(plan.entry_station_m-origin>config_.reference_spacing_m)
        request.nominal.l_out_m=plan.entry_station_m-origin;
      request.nominal.d_pass_m=request.pass_profile_scale_m;
      request.bounds.minimum.d_pass_m=request.bounds.maximum.d_pass_m=request.nominal.d_pass_m;
      request.bounds.minimum.l_out_m=request.bounds.maximum.l_out_m=request.nominal.l_out_m;
      const double merge_start=plan.merge_start_station_m-origin;
      request.nominal.l_hold_m=std::max(0.,merge_start-request.nominal.l_out_m);
      request.nominal.l_merge_m=plan.merge_end_station_m-plan.merge_start_station_m;
      request.bounds.minimum.l_hold_m=request.nominal.l_hold_m;
      request.bounds.maximum.l_hold_m=request.nominal.l_hold_m;
      request.bounds.minimum.l_merge_m=request.nominal.l_merge_m;
      request.bounds.maximum.l_merge_m=request.nominal.l_merge_m;
    }

    if(!populateDynamicObstacles(request,inputs,ego_projection,global_reference_length_m,closed,
        maneuver==DrivingMode::OVERTAKE && !returning))
      return std::nullopt;
    if(request.front_merge_attack && request.passing_preparation!=inputs.preparation)
      return std::nullopt;
    // Candidate feasibility uses physical overlap only. The extra clearance
    // used for keeping an old command is a hold policy and must not shrink the
    // MPPI search space itself.
    request.path_constraint_validator =
        makeBrainPathConstraintValidator(inputs, odometry, 0.0);
    request.path_constraint_geometry_only = true;
    request.path_constraint_execution_prefix = true;
    request.rollout_constraint_validator =
        makeBrainRolloutConstraintValidator(inputs);
    request.valid = true;
    return work;
  }

  std::optional<WorkItem>
  makeWork(const Command &command,
           const nav_msgs::msg::Odometry &odometry) const {
    WorkItem work;
    work.command = command;
    work.odometry = odometry;
    auto &request = work.request;
    request.generation = command.generation;
    request.stamp_sec = rclcpp::Time(command.header.stamp).seconds();
    request.side = command.corridor_side > 0 ? 1 : -1;
    request.phase = mppi::Phase::OVERTAKE;
    request.ego.x_m = odometry.pose.pose.position.x;
    request.ego.y_m = odometry.pose.pose.position.y;
    request.ego.yaw_rad = yawFromQuaternion(odometry.pose.pose.orientation);
    request.ego.speed_mps = std::abs(odometry.twist.twist.linear.x);
    request.ego.steering_rad = 0.0;
    request.ego.acceleration_mps2 = 0.0;
    request.ego.s_m = 0.0;
    request.ego.d_m = 0.0;

    const double certified_reserve = std::max(
        0.0,
        std::min({static_cast<double>(command.wall_clearance_reserve_m),
                  static_cast<double>(command.opponent_clearance_reserve_m),
                  static_cast<double>(command.corridor_clearance_reserve_m)}));
    const double half_width =
        std::clamp(0.5 * certified_reserve, tube_min_half_width_m_,
                   tube_max_half_width_m_);
    if (!std::isfinite(half_width) ||
        certified_reserve < tube_min_half_width_m_) {
      return std::nullopt;
    }
    work.certified_half_width_m = half_width;

    double accumulated_s = 0.0;
    geometry_msgs::msg::Point previous;
    bool have_previous = false;
    for (std::size_t source_index = 0U;
         source_index < command.trajectory.points.size() &&
         request.base_reference_count < mppi::kMaximumReferencePoints;
         ++source_index) {
      const auto &source = command.trajectory.points[source_index];
      if (have_previous) {
        const double ds = distance(previous, source.pose.position);
        if (!std::isfinite(ds) || ds < 1.0e-3) {
          continue;
        }
        accumulated_s += ds;
      }
      const std::size_t index = request.base_reference_count++;
      work.source_indices[index] = source_index;
      auto &base = request.base_reference[index];
      base.s_m = accumulated_s;
      base.x_m = source.pose.position.x;
      base.y_m = source.pose.position.y;
      base.yaw_rad = yawFromQuaternion(source.pose.orientation);
      base.speed_mps = std::abs(source.longitudinal_velocity_mps);
      // The MPPI parameter itself is limited to the certified tube below.
      // Keep the internal Frenet construction envelope wider so discretized
      // input-heading spikes cannot reject every sample before the final,
      // Cartesian/tube validation.
      base.minimum_d_m = -1.0;
      base.maximum_d_m = 1.0;
      base.corridor_d_m = 0.0;
      base.active_d_m = 0.0;
      base.active_d_valid = true;
      previous = source.pose.position;
      have_previous = true;
    }
    if (request.base_reference_count < 3U || accumulated_s < 2.0) {
      return std::nullopt;
    }
    for (std::size_t index = 0U; index < request.base_reference_count;
         ++index) {
      const std::size_t lower = index == 0U ? 0U : index - 1U;
      const std::size_t upper =
          std::min(index + 1U, request.base_reference_count - 1U);
      const auto &a = request.base_reference[lower];
      const auto &b = request.base_reference[upper];
      request.base_reference[index].yaw_rad =
          std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
    }
    for (std::size_t index = 1U; index + 1U < request.base_reference_count;
         ++index) {
      const double yaw_delta =
          std::remainder(request.base_reference[index + 1U].yaw_rad -
                             request.base_reference[index - 1U].yaw_rad,
                         2.0 * M_PI);
      const double ds = request.base_reference[index + 1U].s_m -
                        request.base_reference[index - 1U].s_m;
      request.base_reference[index].curvature_1pm =
          std::clamp(yaw_delta / std::max(ds, 1.0e-3),
                     -base_curvature_limit_1pm_, base_curvature_limit_1pm_);
    }
    request.base_reference.front().curvature_1pm =
        request.base_reference[1U].curvature_1pm;
    request.base_reference[request.base_reference_count - 1U].curvature_1pm =
        request.base_reference[request.base_reference_count - 2U].curvature_1pm;

    const double nominal_offset =
        request.side *
        std::min(nominal_lateral_refinement_m_, 0.5 * half_width);
    const double transition = std::clamp(0.35 * accumulated_s, 2.0, 10.0);
    request.anchor_s_m = std::min(accumulated_s * 0.2,
                                  std::max(0.5, request.ego.speed_mps * 0.2));
    request.anchor_d_m = 0.0;
    request.corridor_nominal_d_m = 0.0;
    request.nominal = {nominal_offset, transition, 2.0, transition, 1.0};
    request.bounds.minimum =
        request.side > 0 ? mppi::Parameters{0.0, 2.0, 0.0, 2.0, 0.85}
                         : mppi::Parameters{-half_width, 2.0, 0.0, 2.0, 0.85};
    request.bounds.maximum =
        request.side > 0 ? mppi::Parameters{half_width, 12.0, 8.0, 12.0, 1.02}
                         : mppi::Parameters{0.0, 12.0, 8.0, 12.0, 1.02};
    request.valid = true;
    return work;
  }

  void recordExecutionInputs(const WorkBatch &batch, const mppi::PlanRequest &r) {
    std::ostringstream out; out.precision(17);
    out << "generation=" << r.generation << " input=" << batch.input_sequence;
#define MPPI_RECORD_FIELD(name) out << " " #name "=" << r.name
    MPPI_RECORD_FIELD(valid); MPPI_RECORD_FIELD(side);
    out << " phase=" << static_cast<int>(r.phase);
    MPPI_RECORD_FIELD(stamp_sec); MPPI_RECORD_FIELD(horizon_steps_override);
    MPPI_RECORD_FIELD(reference_activation_delay_sec);
    MPPI_RECORD_FIELD(anchor_s_m); MPPI_RECORD_FIELD(anchor_d_m);
    MPPI_RECORD_FIELD(corridor_nominal_d_m); MPPI_RECORD_FIELD(cost_preferred_pass_separation_m);
    MPPI_RECORD_FIELD(wall_line_start_s_m); MPPI_RECORD_FIELD(wall_line_end_s_m);
    MPPI_RECORD_FIELD(connect_to_wall_line); MPPI_RECORD_FIELD(wall_line_origin_station_m);
    out << " wall_lines_generation=" << (r.precomputed_wall_lines ? r.precomputed_wall_lines->generation : 0);
    MPPI_RECORD_FIELD(pass_profile_origin_d_m); MPPI_RECORD_FIELD(pass_profile_scale_m);
    MPPI_RECORD_FIELD(sample_staged_speed); MPPI_RECORD_FIELD(curvature_evaluation_distance_m);
    MPPI_RECORD_FIELD(preferred_matching_speed_mps); MPPI_RECORD_FIELD(minimum_speed_mps);
    MPPI_RECORD_FIELD(complete_maneuver); MPPI_RECORD_FIELD(cartesian_measured_prefix);
    MPPI_RECORD_FIELD(path_constraint_geometry_only); MPPI_RECORD_FIELD(path_constraint_execution_prefix);
    MPPI_RECORD_FIELD(overtake_target_index);
    MPPI_RECORD_FIELD(passing_entry_m); MPPI_RECORD_FIELD(passing_exit_m);
    MPPI_RECORD_FIELD(leader_opportunity_index);
    MPPI_RECORD_FIELD(leader_passing_entry_m); MPPI_RECORD_FIELD(leader_passing_exit_m);
    MPPI_RECORD_FIELD(leader_preparation_m); MPPI_RECORD_FIELD(leader_pass_time_sec);
    MPPI_RECORD_FIELD(leader_pass_distance_m);
    MPPI_RECORD_FIELD(passing_speed_mps);
#undef MPPI_RECORD_FIELD
    out << " ego=[" << r.ego.x_m << ',' << r.ego.y_m << ',' << r.ego.yaw_rad << ','
        << r.ego.speed_mps << ',' << r.ego.steering_rad << ',' << r.ego.acceleration_mps2
        << ',' << r.ego.s_m << ',' << r.ego.d_m << ',' << r.ego.yaw_rate_radps << "] pending=[";
    for (std::size_t i=0;i<r.ego.pending_steering_target_count;++i)
      out << r.ego.pending_steering_targets_rad[i] << ',';
    const auto parameters=[&](const char *key,const mppi::Parameters &p) {
      out << "] " << key << "=[" << p.d_pass_m << ',' << p.l_out_m << ',' << p.l_hold_m
          << ',' << p.l_merge_m << ',' << p.speed_scale << ',' << p.lateral_control_near_scale
          << ',' << p.lateral_control_far_scale;
    };
    parameters("nominal",r.nominal); parameters("minimum",r.bounds.minimum);
    parameters("maximum",r.bounds.maximum);
    out << "] base=[";
    for (std::size_t i=0;i<r.base_reference_count;++i) {
      const auto &p=r.base_reference[i];
      out << p.s_m << ',' << p.x_m << ',' << p.y_m << ',' << p.yaw_rad << ','
          << p.curvature_1pm << ',' << p.speed_mps << ',' << p.minimum_d_m << ','
          << p.maximum_d_m << ',' << p.corridor_d_m << ',' << p.active_d_m << ','
          << p.active_d_valid << ',' << p.active_speed_mps << ',' << p.active_speed_valid
          << ',' << p.pass_d_m << ',' << p.pass_d_valid << ',' << p.pass_origin_d_m
          << ',' << p.pass_origin_d_valid << ';';
    }
    out << "] dynamic=[";
    for (std::size_t i=0;i<r.dynamic_obstacle_count;++i) {
      const auto &o=r.dynamic_obstacles[i];
      out << o.s_m << ',' << o.d_m << ',' << o.longitudinal_speed_mps << ','
          << o.lateral_speed_mps << ',' << o.longitudinal_acceleration_bound_mps2 << ','
          << o.longitudinal_uncertainty_m << ',' << o.lateral_uncertainty_m << ','
          << o.heading_relative_to_reference_rad << ',' << o.global_reference_s_m << ','
          << bool(o.prediction) << ';';
    }
    out << "] static=[";
    for (std::size_t i=0;i<r.static_obstacle_count;++i) {
      const auto &o=r.static_obstacles[i];
      out << o.minimum_s_m << ',' << o.maximum_s_m << ',' << o.minimum_d_m << ',' << o.maximum_d_m << ';';
    }
    out << "] history=[";
    if (r.execution_history_field) for (std::size_t i=0;i<r.execution_history_field->count;++i) {
      const auto &p=r.execution_history_field->points[i];
      out << p.s_m << ',' << p.d_m << ',' << p.x_m << ',' << p.y_m << ',' << p.yaw_rad
          << ',' << p.curvature_1pm << ',' << p.speed_mps << ',' << p.uncapped_speed_mps
          << ',' << p.source_s_m << ';';
    }
    out << "] prior_execution=[";
    if (r.prior_execution_reference) for (std::size_t i=0;i<r.prior_execution_reference->count;++i) {
      const auto &p=r.prior_execution_reference->points[i];
      out << p.s_m << ',' << p.d_m << ',' << p.x_m << ',' << p.y_m << ',' << p.yaw_rad
          << ',' << p.curvature_1pm << ',' << p.speed_mps << ',' << p.uncapped_speed_mps
          << ',' << p.source_s_m << ';';
    }
    out << "] global=[";
    if (batch.global_reference) for (const auto &p:batch.global_reference->points)
      out << p.pose.position.x << ',' << p.pose.position.y << ';';
    out << ']';
    // Hash only when the immutable map snapshot changes; its data are preserved
    // separately in the installed map artifact used by the replay.
    if (batch.recorded_wall_map) {
      if (recorded_wall_map_ != batch.recorded_wall_map) {
        recorded_wall_map_=batch.recorded_wall_map;
        recorded_wall_map_hash_=14695981039346656037ULL;
        for (const auto cell:recorded_wall_map_->data) {
          recorded_wall_map_hash_ ^= static_cast<std::uint8_t>(cell);
          recorded_wall_map_hash_ *= 1099511628211ULL;
        }
      }
      const auto &m=*batch.recorded_wall_map;
      out << " map_hash=" << recorded_wall_map_hash_ << " map=[" << m.info.width << ','
          << m.info.height << ',' << m.info.resolution << ',' << m.info.origin.position.x
          << ',' << m.info.origin.position.y << ',' << yawFromQuaternion(m.info.origin.orientation) << ']';
    }
    RCLCPP_INFO(get_logger(),"[MPPI_EXECUTION_INPUT] %s",out.str().c_str());
  }

  static mppi::PlanRequest executionRequestForCandidate(
      const mppi::PlanRequest &shared,const mppi::PlanRequest &candidate) {
    auto result=shared;
    result.front_merge_attack=candidate.front_merge_attack;
    result.compare_passing_points=candidate.compare_passing_points;
    mppi::applyPassingPreparation(result,candidate.passing_preparation);
    result.side=candidate.side;
    return result;
  }

  void workerLoop() {
    while (true) {
      WorkBatch batch;
      {
        std::unique_lock<std::mutex> lock(work_mutex_);
        work_cv_.wait(
            lock, [this]() { return stopping_ || pending_batch_.has_value(); });
        if (stopping_) {
          return;
        }
        batch = std::move(pending_batch_.value());
        pending_batch_.reset();
      }

      if (batch.brain_owned && worker_recovery_epoch_ != batch.recovery_epoch) {
        batch_optimizer_->reset();
        worker_recovery_epoch_ = batch.recovery_epoch;
      }
      const auto worker_started = std::chrono::steady_clock::now();
      const double diagnostic_now = now().seconds();
      const bool diagnostic_due = batch.brain_owned &&
          (brain_sample_diagnostics_enabled_ || decision_markers_enabled_) &&
          (diagnostic_now < last_sample_diagnostic_sec_ ||
           diagnostic_now - last_sample_diagnostic_sec_ >= 1.0);
      if (diagnostic_due) last_sample_diagnostic_sec_ = diagnostic_now;
      for (std::size_t index = 0; index < batch.candidate_count; ++index)
        batch.candidates[index].request.speed_pair_diagnostics = diagnostic_due;
      std::array<mppi::PlanResult, kBrainTemplateLineCount> results{};
      std::array<bool, kBrainTemplateLineCount> certified{};
      std::array<SideSelectionCandidate, kBrainTemplateLineCount+1>
          side_candidates{};
      double batch_elapsed_ms = 0.0;
      double candidate_elapsed_sum_ms = 0.0;
      if (batch.brain_owned) {
        std::array<const mppi::PlanRequest *, mppi::kMaximumBatchCandidateCount>
            requests{};
        std::array<std::size_t, mppi::kMaximumBatchCandidateCount>
            warm_start_slots{};
        for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
          requests[index] = &batch.candidates[index].request;
          warm_start_slots[index] =
              batch.candidates[index].template_slot >= 0
                  ? static_cast<std::size_t>(
                        batch.candidates[index].template_slot)
                  : index;
        }
        mppi::BatchScratch scratch;
        const auto batch_result = batch_optimizer_->plan(
            requests, warm_start_slots, batch.candidate_count, &scratch);
        batch_elapsed_ms = batch_result.elapsed_ms;
        candidate_elapsed_sum_ms = batch_result.candidate_elapsed_sum_ms;
        for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
          results[index] = batch_result.candidates[index];
        }
      } else {
        const auto batch_started = std::chrono::steady_clock::now();
        for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
          mppi::Scratch scratch;
          auto &work = batch.candidates[index];
          auto &planner =
              work.request.side > 0 ? left_planner_ : right_planner_;
          results[index] = planner->plan(work.request, &scratch);
          candidate_elapsed_sum_ms += results[index].elapsed_ms;
        }
        batch_elapsed_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - batch_started)
                               .count();
      }
      for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
        auto &work = batch.candidates[index];
        if (work.brain_owned) {
          // Sample feasibility remains part of optimization. Adoption below
          // also checks the exact serialized command alongside the incumbent
          // in one common frame, including remaining-reference coverage.
          certified[index] = results[index].valid;
        } else {
          certified[index] =
              withinCertifiedTube(results[index], work.certified_half_width_m);
        }
        side_candidates[index] = SideSelectionCandidate{
            work.request.side, results[index].valid && certified[index],
            results[index].selected_evaluation.cost,
            results[index].selected_evaluation.overtake_time_sec};
      }
      const auto selection_started = std::chrono::steady_clock::now();
      const auto elapsed_since = [](const auto &start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
      };
      std::array<std::optional<Command>, kBrainTemplateLineCount> execution_commands{};
      std::array<mppi::Evaluation, kBrainTemplateLineCount> execution_evaluations{};
      std::array<bool, kBrainTemplateLineCount> execution_evaluated{};
      std::array<std::shared_ptr<const mppi::PassingPreparationPlan>,kBrainTemplateLineCount>
          execution_preparations{};
      // One immutable snapshot, frame, horizon, history and objective for ALL
      // publication candidates. Re-evaluate the exact float-valued output,
      // not a reference that will be reshaped after selection.
      auto execution_request = batch.candidates[0].request;
      execution_request.front_merge_attack=false; // Held prefixes retain the short execution certificate.
      execution_request.sample_staged_speed = true;
      const Mppi execution_evaluator(config_);
      std::optional<Command> comparison_hold;
      if (batch.brain_owned) {
        const auto include_length=[&](const Command &command) {
          const double length=trajectoryArcLength(command.trajectory);
          if(length>1e-6)execution_request.curvature_evaluation_distance_m=
              std::min(execution_request.curvature_evaluation_distance_m,length);
        };
        if(batch.allow_continuation && batch.brain_hold && batch.global_reference &&
            (compare_return_continuation_ || ((batch.brain_hold->corridor_side==0)==
             (execution_request.phase==mppi::Phase::MERGE)))) {
          comparison_hold=refreshedBrainHold(*batch.brain_hold,batch.fallback,
              batch.candidates[0].odometry,*batch.global_reference,batch.speed_profile.get());
          include_length(*comparison_hold);
        }
        for (std::size_t i = 0; i < batch.candidate_count; ++i) {
          if (!results[i].valid || !certified[i]) continue;
          execution_commands[i] = refinedCommand(batch.candidates[i], results[i]);
          include_length(*execution_commands[i]);
        }
        std::array<mppi::ExecutionSpeedResult, kBrainTemplateLineCount> speed_results{};
        std::array<double, kBrainTemplateLineCount> speed_elapsed_ms{};
        std::array<double, kBrainTemplateLineCount> speed_before{};
        batch_optimizer_->executeCandidates(batch.candidate_count, [&](std::size_t i) {
          if (!execution_commands[i]) return;
          auto reference = temporaryReferenceFromCommand(*execution_commands[i]);
          reference.overtake_start_source_s_m=results[i].selected_reference.overtake_start_source_s_m;
          reference.front_merge_end_index=results[i].selected_reference.front_merge_end_index;
          const auto candidate_request=executionRequestForCandidate(execution_request,batch.candidates[i].request);
          if (!config_.longitudinal_planning_enabled)
            execution_evaluations[i] = execution_evaluator.evaluateExecutionReference(
                reference, candidate_request);
          const auto speed_started = std::chrono::steady_clock::now();
          auto &optimized = speed_results[i];
          if (config_.longitudinal_planning_enabled) {
            optimized=mppi::planAndEvaluateLongitudinalSpeed(execution_evaluator,reference,candidate_request,config_);
            execution_evaluations[i]=optimized.evaluation;
          } else {
            optimized = mppi::optimizeExecutionSpeed(execution_evaluator, reference,
                candidate_request, execution_evaluations[i], mppi::ExecutionSpeedSearch::Coarse);
          }
          if (optimized.improved) {
            for (std::size_t j = 0; j < optimized.reference.count; ++j)
              execution_commands[i]->trajectory.points[j].longitudinal_velocity_mps =
                  static_cast<float>(optimized.reference.points[j].speed_mps);
            execution_evaluations[i] = optimized.evaluation;
          }
          speed_elapsed_ms[i] = elapsed_since(speed_started);
          speed_before[i] = reference.points[0].speed_mps;
        });
        for (std::size_t i = 0; i < batch.candidate_count; ++i) {
          if (!execution_commands[i]) continue;
          const auto &optimized = speed_results[i];
          const auto candidate_request=executionRequestForCandidate(execution_request,batch.candidates[i].request);
          RCLCPP_INFO(get_logger(),
              "[MPPI_NEW_SPEED] generation=%lu slot=%zu evaluated=%zu valid=%zu improved=%d first_before=%.6f first_after=%.6f elapsed_ms=%.3f preparation=%d plan=%lu revision=%lu follow_release_m=%.6f",
              static_cast<unsigned long>(execution_request.generation), i,
              optimized.evaluated, optimized.valid, optimized.improved,
              speed_before[i],
              execution_commands[i]->trajectory.points.front().longitudinal_velocity_mps,
              speed_elapsed_ms[i],optimized.preparation_selected,
              candidate_request.passing_preparation ? candidate_request.passing_preparation->id : 0,
              candidate_request.passing_preparation ? candidate_request.passing_preparation->revision : 0,
              results[i].selected_reference.overtake_start_source_s_m);
          execution_evaluated[i] = true;
          certified[i] = execution_evaluations[i].valid;
          side_candidates[i].certified = execution_evaluations[i].valid;
          side_candidates[i].cost = execution_evaluations[i].cost;
          side_candidates[i].overtake_time_sec = execution_evaluations[i].overtake_time_sec;
          side_candidates[i].completion_time_sec=execution_evaluations[i].merge_completion_time_sec;
          side_candidates[i].braking_mps=execution_evaluations[i].merge_braking_mps;
          side_candidates[i].alongside_clearance_m=execution_evaluations[i].merge_alongside_clearance_m;
          if(candidate_request.compare_passing_points) RCLCPP_INFO(get_logger(),
              "[MPPI_PASS_OPTIONS] slot=%zu plan=%lu side=%d pass=%.6f valid=%d completion=%.6f braking=%.6f alongside_gap=%.6f",
              i,candidate_request.passing_preparation ? candidate_request.passing_preparation->id : 0,
              candidate_request.side,candidate_request.passing_preparation ? candidate_request.passing_preparation->pass_station_m : -1.,
              certified[i],side_candidates[i].completion_time_sec,side_candidates[i].braking_mps,
              side_candidates[i].alongside_clearance_m);
          if(batch.candidates[i].request.connect_to_wall_line && candidate_request.passing_preparation) {
            auto geometry=temporaryReferenceFromCommand(*execution_commands[i]);
            geometry.wall_connection_end_index=results[i].selected_reference.wall_connection_end_index;
            geometry.front_merge_end_index=results[i].selected_reference.front_merge_end_index;
            geometry.overtake_start_source_s_m=results[i].selected_reference.overtake_start_source_s_m;
            mppi::RejectReason preparation_rejection=mppi::RejectReason::NONE;
            execution_preparations[i]=mppi::bindPreparationConnection(candidate_request,config_,
                execution_evaluations[i],geometry,batch.candidates[i].request.side,
                execution_commands[i]->generation,false,&preparation_rejection);
            certified[i]=side_candidates[i].certified=execution_preparations[i]!=nullptr;
            if(execution_evaluations[i].valid && !execution_preparations[i]) {
              execution_evaluations[i].valid=false;
              execution_evaluations[i].reject_reason=preparation_rejection;
              execution_evaluations[i].reject_stage="preparation_connection";
            }
          }
        }
      }
      const double new_validation_ms = elapsed_since(selection_started);
      const auto marker_started = std::chrono::steady_clock::now();
      visualization_msgs::msg::MarkerArray candidate_markers;
      visualization_msgs::msg::Marker clear;
      clear.header = batch.fallback.header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      candidate_markers.markers.push_back(std::move(clear));
      for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
        publishCandidate(batch.candidates[index], results[index],
                         certified[index], candidate_markers,
                         execution_evaluated[index] ? &execution_evaluations[index] : nullptr);
      }
      // One complete snapshot: KeepLast(1) must never drop an earlier line or
      // deliver only DELETEALL while the next batch is being evaluated.
      candidates_pub_->publish(candidate_markers);
      const double marker_ms = elapsed_since(marker_started);
      const double hold_remaining_m=comparison_hold ?
          trajectoryArcLength(comparison_hold->trajectory) : 0.;
      const bool replenish_geometry=comparison_hold &&
          (!execution_request.passing_preparation || !execution_request.passing_preparation->connection ||
           mppi::pastPreparationConnectionEnd(*execution_request.passing_preparation,execution_request)) &&
          hold_remaining_m < localDistance(execution_request.ego.speed_mps);
      const auto hold_started = std::chrono::steady_clock::now();
      double retime_ms = 0.0;
      std::size_t selected_index = batch.candidate_count;
      const int effective_preferred_side = batch.preferred_side;
      const auto &effective_brain_hold = batch.brain_hold;
      std::optional<Command> safe_refreshed_hold;
      std::optional<Command> retimed_command;
      mppi::ExecutionSpeedResult retimed;
      mppi::Evaluation hold_evaluation;
      mppi::RejectReason hold_reject_reason = mppi::RejectReason::NONE;
      if (batch.brain_owned && batch.allow_continuation && effective_brain_hold.has_value() &&
          batch.global_reference != nullptr &&
          (compare_return_continuation_ || ((effective_brain_hold->corridor_side == 0) ==
           (execution_request.phase == mppi::Phase::MERGE)))) {
        auto refreshed = *comparison_hold;
        if (!config_.longitudinal_planning_enabled)
          hold_evaluation = execution_evaluator.evaluateExecutionReference(
              temporaryReferenceFromCommand(refreshed), execution_request);
        hold_reject_reason = hold_evaluation.reject_reason;
        if (hold_evaluation.valid) {
          safe_refreshed_hold = refreshed;
        }
        auto reference=temporaryReferenceFromCommand(refreshed);
        const auto source_remaining=mppi::remainingExecutionTrajectory(
            temporaryReferenceFromCommand(*effective_brain_hold),execution_request.ego,
            suppliedDistance(execution_request.ego.speed_mps),batch.speed_profile.get());
        if(source_remaining && source_remaining->count==reference.count) {
          reference.overtake_start_source_s_m=source_remaining->overtake_start_source_s_m;
          for(std::size_t i=0;i<reference.count;++i)
            reference.points[i].source_s_m=source_remaining->points[i].source_s_m;
          const auto retime_started = std::chrono::steady_clock::now();
          if (config_.longitudinal_planning_enabled) {
            retimed=mppi::planAndEvaluateLongitudinalSpeed(execution_evaluator,reference,execution_request,config_);
            hold_reject_reason=retimed.evaluation.reject_reason;
          } else {
          retimed=mppi::optimizeExecutionSpeed(execution_evaluator,reference,execution_request,
              hold_evaluation,mppi::ExecutionSpeedSearch::Full,
              [this](std::size_t count,const std::function<void(std::size_t)> &evaluate) {
                const auto workers=std::min(count,mppi::kMaximumBatchCandidateCount);
                batch_optimizer_->executeCandidates(workers,[&](std::size_t slot) {
                  for (std::size_t i=slot;i<count;i+=workers) evaluate(i);
                });
              });
          }
          retime_ms = elapsed_since(retime_started);
          if(retimed.improved) {
            for(std::size_t i=0;i<reference.count;++i)
              refreshed.trajectory.points[i].longitudinal_velocity_mps=static_cast<float>(retimed.reference.points[i].speed_mps);
            refreshed.reason="mppi_brain:retimed_continuation";
            retimed_command=std::move(refreshed);
          }
        }
      }
      const double hold_ms = elapsed_since(hold_started) - retime_ms;
      const auto retime_index=batch.candidate_count;
      side_candidates[retime_index]={effective_preferred_side,retimed_command.has_value(),
          retimed.evaluation.cost,retimed.evaluation.overtake_time_sec};
      const auto execution_selection = selectExecutionCandidate(
          side_candidates, batch.candidate_count+1,
          SideSelectionCandidate{effective_preferred_side,
              batch.allow_continuation && safe_refreshed_hold.has_value(), hold_evaluation.cost,
              hold_evaluation.overtake_time_sec}, effective_preferred_side,
          brain_side_switch_minimum_cost_improvement_,
          batch.allow_continuation &&
              ((execution_request.phase==mppi::Phase::MERGE && effective_brain_hold &&
                effective_brain_hold->corridor_side==0) ||
               (batch.maneuver==DrivingMode::OVERTAKE && execution_request.passing_preparation &&
                execution_request.passing_preparation->connection &&
                execution_request.passing_preparation->side==effective_preferred_side)),
          replenish_geometry);
      const bool continue_execution = execution_selection && execution_selection->continuation;
      const bool retime_execution=execution_selection && !continue_execution &&
          execution_selection->new_candidate_index==retime_index;
      if (execution_selection && !continue_execution) {
        selected_index = execution_selection->new_candidate_index;
      }
      // Fallback cannot affect ranking. Only construct/validate it when no
      // certified new, continued or retimed execution was selected.
      const auto fallback_started = std::chrono::steady_clock::now();
      auto bounded_batch_fallback = makeBatchFallback(batch, execution_selection);
      const double fallback_ms = elapsed_since(fallback_started);
      const double selection_elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-selection_started).count();
      const auto diagnostic_started = std::chrono::steady_clock::now();
      if (batch.brain_owned && get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool())
        recordExecutionInputs(batch, execution_request);
      if (diagnostic_due && decision_markers_enabled_)
        publishDecisionSnapshot(batch, results, selected_index);

      if (batch.brain_owned) {
        std::ostringstream template_summary;
        for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
          if (diagnostic_due) {
            const auto dump_prediction = [&](const mppi::Evaluation &evaluation,
                                             std::size_t profile) {
              std::ostringstream diag;
              diag.precision(12);
              diag << "generation=" << batch.candidates[index].request.generation
                   << " slot=" << index << " profile=" << profile
                   << " selected_slot=" << (index == selected_index)
                   << " stamp=" << batch.candidates[index].request.stamp_sec
                   << " valid=" << evaluation.valid
                   << " reason=" << Mppi::toString(evaluation.reject_reason)
                   << " terminal_s=" << evaluation.terminal_s_m
                   << " terminal_d=" << evaluation.terminal_d_m
                   << " terminal_reference_d=" << evaluation.terminal_reference_d_m
                   << " reject_time=" << evaluation.reject_time_sec
                   << " reject_stage=" << evaluation.reject_stage
                   << " reject_obstacle=" << evaluation.reject_obstacle_index
                   << " reject_x=" << evaluation.reject_x_m
                   << " reject_y=" << evaluation.reject_y_m
                   << " rollout_count=" << evaluation.predicted_rollout_count
                   << " rollout=[";
              for (std::size_t i = 0; i < evaluation.predicted_rollout_count; ++i) {
                const auto &state = evaluation.predicted_rollout[i];
                diag << state.time_sec << ',' << state.x_m << ',' << state.y_m
                     << ',' << state.yaw_rad << ',' << state.speed_mps << ';';
              }
              diag << ']';
              RCLCPP_INFO(get_logger(), "[MPPI_PREDICTION_DIAG] %s", diag.str().c_str());
              const auto obstacle_index=evaluation.reject_obstacle_index;
              const auto &request=batch.candidates[index].request;
              if (profile==1U && index==selected_index &&
                  evaluation.reject_dynamic_obstacle &&
                  std::string(evaluation.reject_stage)=="execution_sweep" &&
                  obstacle_index<request.dynamic_obstacle_count &&
                  obstacle_index<batch.opponents.size() && batch.global_reference) {
                const auto contains=[&](const std::array<double,4U> &box) {
                  return evaluation.reject_s_m>=box[0] && evaluation.reject_s_m<=box[1] &&
                      evaluation.reject_d_m>=box[2] && evaluation.reject_d_m<=box[3];
                };
                const auto &opponent=batch.opponents[obstacle_index];
                const auto &initial = request.dynamic_obstacles[obstacle_index];
                const auto predicted = opponent_prediction::positionAt(initial,evaluation.reject_time_sec);
                int body_overlap=-1;
                double ox=NAN,oy=NAN,oyaw=NAN;
                {
                  const auto pose=referencePoseIndex(*batch.global_reference).pose(
                      predicted.global_s,predicted.d);
                  if (pose) {
                    ox=pose->at(0); oy=pose->at(1);
                    oyaw=pose->at(2)+predicted.relative_yaw;
                    body_overlap=!footprintsHaveClearance(evaluation.reject_x_m,evaluation.reject_y_m,
                        evaluation.reject_yaw_rad,ox,oy,oyaw,0.0);
                  }
                }
                std::ostringstream witness;
                // Log exactly the input used by this candidate, not a newly
                // received V2X state or a reprojected velocity.
                const double t=evaluation.reject_time_sec;
                const double eyaw=evaluation.reject_yaw_rad-evaluation.reject_reference_yaw_rad;
                const double oyaw_local=predicted.relative_yaw;
                const double length=config_.vehicle_half_length_m,width=config_.vehicle_half_width_m;
                const double projected_s=length*(std::abs(std::cos(eyaw))+std::abs(std::cos(oyaw_local)))+
                    width*(std::abs(std::sin(eyaw))+std::abs(std::sin(oyaw_local)));
                const double projected_d=length*(std::abs(std::sin(eyaw))+std::abs(std::sin(oyaw_local)))+
                    width*(std::abs(std::cos(eyaw))+std::abs(std::cos(oyaw_local)));
                const double extra_s=std::max(0.0,config_.obstacle_longitudinal_inflation_m-projected_s);
                const double extra_d=std::max(0.0,config_.obstacle_lateral_inflation_m-projected_d);
                const double scale=mppi::collisionUncertaintyScale(t);
                const double a=scale*initial.longitudinal_acceleration_bound_mps2,v=initial.longitudinal_speed_mps;
                const double us=extra_s+scale*initial.longitudinal_uncertainty_m+.5*a*t*t;
                const double ud=extra_d+scale*initial.lateral_uncertainty_m;
                const double ds=evaluation.reject_s_m-predicted.s;
                const double dd=evaluation.reject_d_m-predicted.d;
                const auto local=mppi::localEnvelopeDiagnostic(ds,dd,eyaw,oyaw_local,length,width,us,ud);
                const double stop_shift=(a>0 && v>=0)? .5*a*std::pow(std::max(0.0,t-v/a),2):0;
                const auto forward=v>=0 ? mppi::localEnvelopeDiagnostic(
                    ds-.5*stop_shift,dd,eyaw,oyaw_local,length,width,us-.5*stop_shift,ud)
                    : mppi::LocalEnvelopeDiagnostic{};
                witness.precision(12);
                witness << "generation=" << request.generation << " slot=" << index
                    << " profile=" << profile << " stamp=" << request.stamp_sec
                    << " opponent=" << opponent.id << " time=" << evaluation.reject_time_sec
                    << " initial_s=" << initial.s_m << " initial_d=" << initial.d_m
                    << " global_initial_s=" << initial.global_reference_s_m
                    << " collision_model=" << (request.world_reference?"world_union":"legacy_frenet")
                    << " opponent_prediction=" << (initial.prediction ?
                        (opponent.matched_prior_lap_execution ? "matched_prior_lap" :
                         online_motion_prediction_enabled_ ? "online_motion" :
                         recent_motion_prediction_enabled_ ? "recent_motion" : "prior_lap_arc") : "constant_velocity")
                    << " prediction_source_stamp=" << (initial.prediction?initial.prediction->source_stamp:opponent.stamp_sec)
                    << " predicted_global_s=" << predicted.global_s
                    << " predicted_d=" << predicted.d
                    << " predicted_relative_yaw=" << predicted.relative_yaw
                    << " initial_vs=" << initial.longitudinal_speed_mps
                    << " initial_vd=" << initial.lateral_speed_mps
                    << " acceleration_bound=" << initial.longitudinal_acceleration_bound_mps2
                    << " collision_uncertainty_scale=" << scale
                    << " uncertainty_s=" << initial.longitudinal_uncertainty_m
                    << " uncertainty_d=" << initial.lateral_uncertainty_m
                    << " initial_relative_yaw=" << initial.heading_relative_to_reference_rad
                    << " opponent_aligned_stamp=" << opponent.stamp_sec
                    << " reference_yaw=" << evaluation.reject_reference_yaw_rad
                    << " ego_relative_yaw=" << eyaw
                    << " half_length=" << length << " half_width=" << width
                    << " extra_s=" << extra_s << " extra_d=" << extra_d
                    << " local_full_overlap=" << (local.valid?int(local.overlap):-1)
                    << " local_full_axis_gap=" << local.separating_gap_m
                    << " local_forward_overlap=" << (forward.valid?int(forward.overlap):-1)
                    << " local_forward_axis_gap=" << forward.separating_gap_m
                    << " body=" << body_overlap << " nominal=" << contains(evaluation.reject_nominal_envelope)
                    << " position=" << contains(evaluation.reject_position_envelope)
                    << " full=" << contains(evaluation.reject_obstacle_envelope)
                    << " ego=[" << evaluation.reject_x_m << ',' << evaluation.reject_y_m << ',' << evaluation.reject_yaw_rad << ']'
                    << " opponent_pose=[" << ox << ',' << oy << ',' << oyaw << ']'
                    << " sd=[" << evaluation.reject_s_m << ',' << evaluation.reject_d_m << ']';
                for (const auto &entry : {std::make_pair("nominal_box",evaluation.reject_nominal_envelope),
                     std::make_pair("position_box",evaluation.reject_position_envelope),
                     std::make_pair("full_box",evaluation.reject_obstacle_envelope)}) {
                  witness << ' ' << entry.first << "=[";
                  for (double value:entry.second) witness << value << ',';
                  witness << ']';
                }
                RCLCPP_INFO(get_logger(), "[MPPI_COLLISION_WITNESS] %s", witness.str().c_str());
              }
            };
            // Profile 0 is the slot winner, not necessarily the published path.
            dump_prediction(results[index].selected_evaluation, 0U);
            for (std::size_t p = 0; p < results[index].speed_pair_count; ++p) {
              const auto &pair = results[index].speed_pairs[p];
              dump_prediction(pair.evaluation, p + 1U);
              std::ostringstream diag;
              diag.precision(12);
              diag << "generation=" << batch.candidates[index].request.generation
                   << " slot=" << index << " profile=" << p + 1
                   << " selected=" << (index == selected_index)
                   << " valid=" << pair.valid << " reason=" << Mppi::toString(pair.reason)
                   << " cost=" << pair.cost << " selected_cost=" << results[index].selected_evaluation.cost
                   << " first=" << pair.first_speed << " pre=" << pair.first_uncapped_speed
                   << " geometry_delta=" << pair.geometry_delta_m << " terms=[";
              for (double term : pair.terms) diag << term << ',';
              diag << ']';
              RCLCPP_INFO(get_logger(), "[MPPI_SPEED_PAIR] %s", diag.str().c_str());
            }
            const auto dump = [&](const mppi::TemporaryReference &reference,
                                  const mppi::Evaluation &evaluation, int sample) {
              std::ostringstream diag;
              diag.precision(12);
              diag << "generation=" << batch.candidates[index].request.generation
                   << " slot=" << index << " sample=" << sample
                   << " valid=" << evaluation.valid
                   << " reason=" << Mppi::toString(evaluation.reject_reason)
                   << " reject_stage=" << evaluation.reject_stage
                   << " reject_step=" << evaluation.reject_step_index
                   << " reject_time=" << evaluation.reject_time_sec
                   << " terminal_s=" << evaluation.terminal_s_m
                   << " terminal_d=" << evaluation.terminal_d_m
                   << " terminal_reference_d=" << evaluation.terminal_reference_d_m
                   << " reject_x=" << evaluation.reject_x_m << " reject_y=" << evaluation.reject_y_m
                   << " reject_obstacle=" << evaluation.reject_obstacle_index
                   << " reject_obstacle_id=" << (evaluation.reject_dynamic_obstacle && evaluation.reject_obstacle_index<batch.opponents.size()?
                         batch.opponents[evaluation.reject_obstacle_index].id:std::string("unavailable"))
                   << " reject_envelope=[" << evaluation.reject_obstacle_envelope[0] << ','
                   << evaluation.reject_obstacle_envelope[1] << ','
                   << evaluation.reject_obstacle_envelope[2] << ','
                   << evaluation.reject_obstacle_envelope[3] << ']'
                   << " ego_v=" << batch.candidates[index].request.ego.speed_mps
                   << " terms=[";
              for (double term : evaluation.cost_terms) diag << term << ',';
              diag << "] points=[";
              for (std::size_t p = 0; p < reference.count; ++p) {
                const auto &v = reference.points[p];
                if (p) diag << ';';
                diag << v.s_m << ',' << v.x_m << ',' << v.y_m << ',' << v.d_m << ','
                     << v.curvature_1pm << ',' << v.uncapped_speed_mps << ',' << v.speed_mps;
              }
              diag << ']';
              RCLCPP_INFO(get_logger(), "[MPPI_SAMPLE_DIAG] %s", diag.str().c_str());
            };
            dump(results[index].selected_reference, results[index].selected_evaluation, -1);
            for (std::size_t sample = 0; sample < results[index].visualized_sample_count; ++sample) {
              dump(results[index].visualized_sample_references[sample],
                   results[index].visualized_sample_evaluations[sample],
                   static_cast<int>(results[index].visualized_sample_indices[sample]));
            }
          }
          if (index > 0U)
            template_summary << ';';
          const auto &summary_evaluation = execution_evaluated[index] ?
              execution_evaluations[index] : results[index].selected_evaluation;
          template_summary
              << "slot=" << batch.candidates[index].template_slot
              << ",side=" << batch.candidates[index].request.side
              << ",d=" << batch.candidates[index].request.nominal.d_pass_m
              << ",samples=" << results[index].valid_sample_count
              << ",evaluated=" << results[index].evaluated_sample_count
              << ",duplicates=" << results[index].duplicate_sample_count
              << ",l_out=" << results[index].selected.l_out_m
              << ",speed_scale=" << results[index].selected.speed_scale
              << ",control_knots="
              << results[index].selected_control_sequence.count << ",valid="
              << (side_candidates[index].certified ? "true" : "false")
              << ",horizon=" << batch.candidates[index].actual_horizon_m << '/'
              << batch.candidates[index].requested_horizon_m
              << ",prediction_sec=" << config_.dt_sec *
                     batch.candidates[index].request.horizon_steps_override
              << ",capacity_limited="
              << (batch.candidates[index].reference_capacity_limited ? "true"
                                                                     : "false")
              << ",evaluation=" << (execution_evaluated[index] ? "adoption" : "raw")
              << ",raw_valid=" << results[index].valid
              << ",cost=" << side_candidates[index].cost << ",reason="
              << Mppi::toString(summary_evaluation.reject_reason);
          template_summary << ",cost_terms=[";
          const auto &terms = summary_evaluation.cost_terms;
          for (std::size_t term = 0; term < terms.size(); ++term) {
            if (term != 0U) template_summary << ',';
            template_summary << terms[term];
          }
          template_summary << ']';
        }
        const auto summary = template_summary.str();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MPPI_TEMPLATE_SELECTION] count=%zu preferred=%d selected=%d "
            "batch_ms=%.3f candidate_sum_ms=%.3f relative_switch=%.6f templates=[%s]",
            batch.candidate_count, effective_preferred_side,
            selected_index < batch.candidate_count
                ? batch.candidates[selected_index].request.side
                : 0,
            batch_elapsed_ms, candidate_elapsed_sum_ms,
            brain_side_switch_minimum_cost_improvement_, summary.c_str());
      }

      // Only the final selected-command validation is serialized. Sampling,
      // candidate evaluation and speed search remain outside this transaction.
      const auto commit_wait_started=std::chrono::steady_clock::now();
      std::unique_lock<std::mutex> decision_lock(decision_mutex_);
      const double commit_wait_ms=elapsed_since(commit_wait_started);
      const auto final_validation_started=std::chrono::steady_clock::now();
      ExecutionRevisionCheck revision_check{batch.execution_revision};
      bool proposal_revalidated = false;
      double publication_deadline = batch.candidates[0U].command.valid_until_sec;
      if (batch.brain_owned) {
        const auto live = captureBrainInputs();
        {
          if (!execution_selection && isConnectedReturn(bounded_batch_fallback) &&
              live.odometry && live.base_reference && live.active_command &&
              hasAlongsideVehicle(*live.base_reference,*live.odometry,live.opponents)) {
            bounded_batch_fallback=connectedReturnFallback(batch.fallback,*live.odometry,
                live.steering_fresh?live.steering_rad:0.,*live.base_reference,live.active_command,
                live.driving_fsm.mode(),makeBrainCommandValidator(live,*live.odometry,0.),
                live.opponents,live.speed_profile.get());
          }
          Command *proposal = retime_execution ? &*retimed_command :
              continue_execution ? &*safe_refreshed_hold :
              selected_index < batch.candidate_count ? &*execution_commands[selected_index] :
              &bounded_batch_fallback;
          bool valid = false;
          if (live.odometry && live.base_reference) {
            const auto validator = makeBrainCommandValidator(live, *live.odometry, 0.0);
            valid = (isConnectedReturn(*proposal) || isAlongsideHoldFallback(*proposal) ||
                maneuverCurrent(batch,live)) &&
                validateBrainCommand(*proposal, validator) == mppi::RejectReason::NONE;
            if(valid && execution_request.passing_preparation && !continue_execution &&
                (retime_execution || selected_index<batch.candidate_count)) {
              const auto target=std::find_if(live.opponents.begin(),live.opponents.end(),
                  [&](const auto &o){return o.id==execution_request.passing_preparation->target_id;});
              valid=live.preparation &&
                  mppi::samePreparationWindow(*execution_request.passing_preparation,*live.preparation) &&
                  target!=live.opponents.end() &&
                  (execution_request.passing_preparation->front_merge ||
                   (live.leader_vehicle_id==execution_request.passing_preparation->target_id && target->prior_lap_match.usable));
            }
            if (valid) {
              proposal->header.stamp = live.odometry->header.stamp;
              proposal->trajectory.header = proposal->header;
              proposal->valid_until_sec = rclcpp::Time(live.odometry->header.stamp).seconds()+0.20;
              publication_deadline = proposal->valid_until_sec;
              proposal_revalidated = true;
            }
          }
          revision_check.recordValidation(live.execution_revision, valid);
          RCLCPP_INFO(get_logger(),
              "[MPPI_EXECUTION_REVALIDATED] generation=%lu revision=%lu valid=%d input=%lu",
              static_cast<unsigned long>(batch.candidates[0].command.generation),
              static_cast<unsigned long>(live.execution_revision), valid,
              static_cast<unsigned long>(live.input_sequence));
        }
      }
      const double final_validation_ms=elapsed_since(final_validation_started);
      std::unique_lock<std::mutex> authority_lock(authority_mutex_);
      const auto generation = batch.candidates[0U].command.generation;
      const auto latest_generation =
          latest_generation_.load(std::memory_order_acquire);
      const auto latest_semantic_key =
          latest_semantic_key_.load(std::memory_order_acquire);
      // Successful live validation supplies a new 0.20 s deadline from the
      // observed vehicle epoch. Generation lag alone cannot invalidate that
      // proof; owner, semantic and execution-revision checks still apply.
      // Compatibility refinement retains its exact-generation contract.
      const bool generation_current = generationAdoptable(
          generation, latest_generation,
          batch.brain_owned ? brain_max_result_generation_lag_ : 0U,
          batch.brain_owned && proposal_revalidated);
      const bool semantic_current =
          !batch.brain_owned || (batch.semantic_key != 0U &&
                                 batch.semantic_key == latest_semantic_key);
      bool owner_current = true;
      bool execution_current = true;
      if (batch.brain_owned) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        const std::optional<std::uint64_t> snapshot_source = batch.brain_hold ?
            std::optional<std::uint64_t>(batch.brain_hold->generation) : std::nullopt;
        const std::optional<std::uint64_t> live_source = active_brain_command_ ?
            std::optional<std::uint64_t>(active_brain_command_->generation) : std::nullopt;
        const std::uint64_t live_speed_generation =
            active_brain_command_ ? active_brain_speed_generation_ : 0U;
        const auto ordinary_source=[](const std::optional<Command> &c) {
          return c ? c->generation : 0U;
        };
        owner_current = snapshot_source == live_source &&
            batch.reference_revision == reference_revision_ &&
            ordinary_source(batch.ordinary_hold) ==
                ordinary_source(ordinary_hold_ ? ordinary_hold_ : reference_transition_) &&
            batch.speed_generation == live_speed_generation &&
            batch.driving_revision == driving_fsm_.revision();
        execution_current = revision_check.canCommit(execution_revision_);
      }
      // A queued snapshot may predate a newer accepted path. It cannot compare
      // against the old incumbent and then roll execution back to that source.
      const bool recovery_current = !batch.brain_owned ||
          (!recovery_active_ && batch.recovery_epoch == recovery_epoch_);
      const bool current = generation_current && semantic_current && owner_current && execution_current && recovery_current;
      if(batch.brain_owned) RCLCPP_INFO(get_logger(),
          "[MPPI_COMMIT_TIMING] generation=%lu wait_ms=%.3f validation_ms=%.3f generation_current=%d semantic_current=%d owner_current=%d execution_current=%d recovery_current=%d",
          static_cast<unsigned long>(generation),commit_wait_ms,final_validation_ms,
          generation_current,semantic_current,owner_current,execution_current,recovery_current);
      if (batch.brain_owned) RCLCPP_INFO(get_logger(),
          "[MPPI_INPUT_LATENCY] generation=%lu input=%lu prepare_ms=%.3f queue_ms=%.3f capture_to_gate_ms=%.3f current=%d",
          static_cast<unsigned long>(generation), static_cast<unsigned long>(batch.input_sequence),
          std::chrono::duration<double,std::milli>(batch.queued_at-batch.captured_at).count(),
          std::chrono::duration<double,std::milli>(worker_started-batch.queued_at).count(),
          elapsed_since(batch.captured_at), current);
      if (batch.brain_owned && !execution_current) {
        RCLCPP_INFO(get_logger(), "[MPPI_EXECUTION_SUPERSEDED] generation=%lu validated_revision=%lu",
            static_cast<unsigned long>(generation),
            static_cast<unsigned long>(revision_check.validated_revision));
      }
      const bool unexpired =
          publication_deadline <= 0.0 || now().seconds() <= publication_deadline;
      const double diagnostics_and_authority_ms = elapsed_since(diagnostic_started);
      // Capture the adoption verdict before publication, but format the large
      // spatial diagnostics only after releasing the control transaction.
      const auto emit_execution_diagnostics = [&] {
      if (batch.brain_owned) {
        RCLCPP_INFO(get_logger(),
            "[MPPI_SELECTION_TIMING] generation=%lu stamp=%.9f new_validation_ms=%.3f marker_ms=%.3f hold_ms=%.3f retime_ms=%.3f retime_evaluated=%lu fallback_ms=%.3f fallback_evaluated=%d diagnostics_and_authority_ms=%.3f",
            static_cast<unsigned long>(generation), execution_request.stamp_sec,
            new_validation_ms, marker_ms, hold_ms, retime_ms,
            static_cast<unsigned long>(retimed.evaluated), fallback_ms,
            !execution_selection, diagnostics_and_authority_ms);
        std::ostringstream comparison;
        comparison.precision(10);
        std::ostringstream decision;
        decision.precision(10);
        const auto describe = [&](const char *kind, std::size_t index,
                                  const mppi::Evaluation &evaluation, double speed,
                                  const Command *command=nullptr) {
          decision << " " << kind << index << "={valid:" << evaluation.valid
              << ",first:" << speed << ",cost:" << evaluation.cost
              << ",reason:" << Mppi::toString(evaluation.reject_reason)
              << ",stage:" << evaluation.reject_stage << ",terms:[";
          for (double term : evaluation.cost_terms) decision << term << ',';
          decision << "],terminal_arc:" << evaluation.field_terminal_arc_m
              << ",history_coverage:" << evaluation.field_history_coverage_m
              << ",gradient_coverage:" << evaluation.field_gradient_coverage_m << ",field_split:[";
          for (double term : evaluation.field_cost_split) decision << term << ',';
          decision << "]}";
          if(command && get_parameter("brain.cost_spatial_diagnostics_enabled").as_bool()) {
            const auto path=temporaryReferenceFromCommand(*command);
            std::ostringstream spatial;spatial.precision(17);
            spatial << "generation=" << generation << " candidate=" << kind << index
                << " input=" << batch.input_sequence
                << " stamp=" << execution_request.stamp_sec << " valid=" << evaluation.valid
                << " cost=" << evaluation.cost << " reason=" << Mppi::toString(evaluation.reject_reason)
                << " stage=" << evaluation.reject_stage << " pass_time=" << evaluation.overtake_time_sec
                << " phase=" << static_cast<int>(execution_request.phase)
                << " curvature_window=" << execution_request.curvature_evaluation_distance_m
                << " clearance=[";
            for(double v:evaluation.clearance_cost_sources)spatial<<v<<',';
            spatial<<"] peak=[";
            for(double v:evaluation.clearance_peak)spatial<<v<<',';
            spatial<<"] ids=[";
            for(const auto &opponent:batch.opponents)spatial<<opponent.id<<',';
            spatial<<"] path=[";
            for(std::size_t j=0;j<path.count;++j){const auto &p=path.points[j];
              spatial<<p.s_m<<','<<p.x_m<<','<<p.y_m<<','<<p.curvature_1pm<<','<<p.speed_mps<<','<<p.yaw_rad<<';';}
            spatial<<"] rollout=[";
            for(std::size_t j=0;j<evaluation.predicted_rollout_count;++j){const auto &p=evaluation.predicted_rollout[j];
              spatial<<p.time_sec<<','<<p.x_m<<','<<p.y_m<<','<<p.yaw_rad<<','<<p.speed_mps<<';';}
            spatial<<']';
            RCLCPP_INFO(get_logger(),"[MPPI_SPATIAL_COST] %s",spatial.str().c_str());
          }
        };
        for (std::size_t i = 0; i < batch.candidate_count; ++i) {
          if (execution_evaluated[i]) {
            describe("new", i, execution_evaluations[i],
                execution_commands[i]->trajectory.points.front().longitudinal_velocity_mps,
                &*execution_commands[i]);
          } else {
            describe("raw_rejected", i, results[i].selected_evaluation,
                results[i].selected_reference.count ? results[i].selected_reference.points[0].speed_mps : 0.);
          }
          comparison << " slot" << i << "={valid:" << execution_evaluations[i].valid
              << ",evaluated:" << execution_evaluated[i]
              << ",raw_valid:" << results[i].valid
              << ",raw_reason:" << Mppi::toString(results[i].reject_reason)
              << ",cost:" << execution_evaluations[i].cost
              << ",pass_time:" << execution_evaluations[i].overtake_time_sec
              << ",reason:" << (execution_evaluated[i] ?
                  Mppi::toString(execution_evaluations[i].reject_reason) : "not_evaluated")
              << ",stage:" << (execution_evaluated[i] ?
                  execution_evaluations[i].reject_stage : "not_evaluated") << "}";
        }
        const char *action = continue_execution ? "continue" : retime_execution ? "retime" :
            selected_index >= batch.candidate_count ? "fallback" :
            effective_preferred_side == 0 ? "new" :
            batch.candidates[selected_index].request.side == effective_preferred_side ? "update" : "switch";
        if (safe_refreshed_hold)
          describe("hold", 0, hold_evaluation,
              safe_refreshed_hold->trajectory.points.front().longitudinal_velocity_mps,
              &*safe_refreshed_hold);
        if (retimed_command)
          describe("retime", 0, retimed.evaluation,
              retimed_command->trajectory.points.front().longitudinal_velocity_mps,
              &*retimed_command);
        RCLCPP_INFO(get_logger(),
            "[MPPI_SELECTION_CANDIDATES] generation=%lu stamp=%.9f action=%s selected_slot=%d eligible=%d%s",
            static_cast<unsigned long>(generation), execution_request.stamp_sec, action,
            selected_index < batch.candidate_count ? static_cast<int>(selected_index) : -1,
            current && unexpired, decision.str().c_str());
        RCLCPP_INFO(get_logger(),
            "[MPPI_EXECUTION] generation=%lu prior_source_generation=%lu stamp=%.9f initial_speed=%.6f action=%s eligible=%d owner_current=%d hold_valid=%d hold_cost=%.9f hold_reason=%s hold_stage=%s selected_slot=%d selection_ms=%.3f batch_ms=%.3f hold_pass_time=%.6f replenish_required=%d hold_remaining_m=%.6f%s",
            static_cast<unsigned long>(generation),
            static_cast<unsigned long>(batch.speed_generation ? batch.speed_generation :
                effective_brain_hold ? effective_brain_hold->generation : 0),
            execution_request.stamp_sec, execution_request.ego.speed_mps, action,
            current && unexpired && !shadow_only_, owner_current, hold_evaluation.valid, hold_evaluation.cost,
            Mppi::toString(hold_reject_reason), hold_evaluation.reject_stage,
            selected_index < batch.candidate_count ? static_cast<int>(selected_index) : -1,
            selection_elapsed_ms, batch_elapsed_ms, hold_evaluation.overtake_time_sec,
            replenish_geometry, hold_remaining_m,
            comparison.str().c_str());
        RCLCPP_INFO(get_logger(),
            "[MPPI_RETIME] generation=%lu shape_source=%lu speed_generation=%lu evaluated=%lu valid=%lu improved=%d selected=%d cost=%.9f initial_speed=%.6f first_speed=%.6f history=%.9f pass_time=%.6f",
            static_cast<unsigned long>(generation),
            static_cast<unsigned long>(effective_brain_hold?effective_brain_hold->generation:0),
            static_cast<unsigned long>(batch.speed_generation),static_cast<unsigned long>(retimed.evaluated),
            static_cast<unsigned long>(retimed.valid),retimed.improved,retime_execution,retimed.evaluation.cost,
            execution_request.ego.speed_mps,retimed.reference.count?retimed.reference.points[0].speed_mps:0.,
            hold_evaluation.cost_terms[9],retimed.evaluation.overtake_time_sec);
      }
      };
      const auto record_publication_latency = [&](std::uint64_t publication) {
        if(batch.brain_owned) preparation_planning_delay_sec_.store(
            elapsed_since(batch.captured_at)/1000.,std::memory_order_relaxed);
        if (batch.brain_owned) RCLCPP_INFO(get_logger(),
            "[MPPI_INPUT_PUBLISHED] generation=%lu input=%lu publication=%lu capture_to_publish_ms=%.3f",
            static_cast<unsigned long>(generation), static_cast<unsigned long>(batch.input_sequence),
            static_cast<unsigned long>(publication), elapsed_since(batch.captured_at));
      };
      if(retime_execution && current && unexpired && !shadow_only_) {
        { std::lock_guard<std::mutex> lock(input_mutex_); last_recovery_plan_.reset(); }
        const auto &command=*retimed_command;
        const auto publication=publishOutput(command);
        record_publication_latency(publication);
        {
          std::lock_guard<std::mutex> lock(input_mutex_);
          active_brain_speed_profile_=std::make_shared<const mppi::TemporaryReference>(
              mppi::executionSpeedProfile(retimed.reference));
          active_brain_speed_generation_=generation;
          active_preparation_=mppi::refinePassingPreparation(execution_request,config_,retimed.evaluation,
              effective_brain_hold->generation);
        }
        RCLCPP_INFO(get_logger(),
            "[MPPI_EXECUTION_APPLIED] publication=%lu snapshot=%lu source=%lu action=retime shape_source=%lu first_speed=%.6f",
            static_cast<unsigned long>(publication),static_cast<unsigned long>(generation),
            static_cast<unsigned long>(generation),static_cast<unsigned long>(effective_brain_hold->generation),
            command.trajectory.points.front().longitudinal_velocity_mps);
        authority_lock.unlock();
        decision_lock.unlock();
        emit_execution_diagnostics();
        publishStatus(command,nullptr,"brain:retimed_continuation");
        continue;
      }
      if (continue_execution && current && unexpired && !shadow_only_) {
        { std::lock_guard<std::mutex> lock(input_mutex_); last_recovery_plan_.reset(); }
        const auto &hold = *safe_refreshed_hold;
        const auto publication = publishOutput(hold);
        record_publication_latency(publication);
        RCLCPP_INFO(get_logger(),
            "[MPPI_EXECUTION_APPLIED] publication=%lu snapshot=%lu source=%lu action=continue first_speed=%.6f",
            static_cast<unsigned long>(publication), static_cast<unsigned long>(generation),
            static_cast<unsigned long>(batch.speed_generation?batch.speed_generation:effective_brain_hold->generation),
            hold.trajectory.points.front().longitudinal_velocity_mps);
        // Keep source geometry/speed and its origin generation unchanged.
        // No warm cache update: no new proposal was accepted.
        authority_lock.unlock();
        decision_lock.unlock();
        emit_execution_diagnostics();
        publishStatus(hold, nullptr, "brain:continue_selected");
        continue;
      }
      if (selected_index < batch.candidate_count && current && unexpired &&
          !shadow_only_) {
        { std::lock_guard<std::mutex> lock(input_mutex_); last_recovery_plan_.reset(); }
        const auto index = selected_index;
        const auto &work = batch.candidates[index];
        const auto &result = results[index];
        auto refined = work.brain_owned ? *execution_commands[index] : refinedCommand(work, result);
        const auto &guarded = refined;
        const auto publication = publishOutput(guarded);
        record_publication_latency(publication);
        if (work.brain_owned) RCLCPP_INFO(get_logger(),
            "[MPPI_EXECUTION_APPLIED] publication=%lu snapshot=%lu source=%lu action=%s slot=%lu first_speed=%.6f",
            static_cast<unsigned long>(publication), static_cast<unsigned long>(generation),
            static_cast<unsigned long>(generation),
            effective_preferred_side == 0 ? "new" :
                work.request.side == effective_preferred_side ? "update" : "switch",
            static_cast<unsigned long>(index), guarded.trajectory.points.front().longitudinal_velocity_mps);
        if(work.brain_owned) batch_optimizer_->accept(
            static_cast<std::size_t>(work.template_slot),work.request,result);
        if (!brain_mode_) publishApplied(work, result);
        if (work.brain_owned) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          if (work.request.phase != mppi::Phase::MERGE) {
            driving_fsm_.acceptLateral(work.maneuver);
            active_maneuver_target_id_=batch.maneuver_target_id;
          }
          preferred_side_ = work.request.side;
          // Preserve exactly the published geometry and speed field. Future
          // holds are reprojected and revalidated, not recursively committed.
          active_brain_command_ = refined;
          ordinary_hold_.reset();
          reference_transition_.reset();
          auto speed_profile=temporaryReferenceFromCommand(refined);
          speed_profile.overtake_start_source_s_m=result.selected_reference.overtake_start_source_s_m;
          speed_profile.wall_connection_end_index=result.selected_reference.wall_connection_end_index;
          speed_profile.front_merge_end_index=result.selected_reference.front_merge_end_index;
          active_brain_speed_profile_=std::make_shared<const mppi::TemporaryReference>(
              mppi::executionSpeedProfile(speed_profile));
          active_brain_speed_generation_=0;
          const auto adopted_request=executionRequestForCandidate(execution_request,work.request);
          active_preparation_=work.request.connect_to_wall_line ?
              mppi::bindPreparationConnection(adopted_request,config_,execution_evaluations[index],
                  speed_profile,work.request.side,refined.generation,
                  replenish_geometry && (retimed_command || safe_refreshed_hold)) : nullptr;
          if(active_preparation_) {
            const auto &plan=*active_preparation_;
            RCLCPP_INFO(get_logger(),
                "[MPPI_PREPARATION_CONNECTION] publication=%lu plan=%lu side=%d shape=%lu start_station=%.6f end_station=%.6f points=%zu validated_until=%.6f",
                publication,plan.id,plan.side,plan.geometry_generation,
                plan.connection_start_station_m,plan.connection_end_station_m,
                plan.connection->count,plan.validated_until_sec);
          }
        }
        authority_lock.unlock();
        decision_lock.unlock();
        emit_execution_diagnostics();
        publishStatus(guarded, &result,
                      work.brain_owned ? "brain:active" : "active:refined");
        continue;
      }

      std::size_t diagnostic_index =
          selected_index < batch.candidate_count ? selected_index : 0U;
      if (selected_index >= batch.candidate_count) {
        bool preferred_found = false;
        if (effective_preferred_side != 0) {
          for (std::size_t index = 0U; index < batch.candidate_count; ++index) {
            if (batch.candidates[index].request.side ==
                effective_preferred_side) {
              diagnostic_index = index;
              preferred_found = true;
              break;
            }
          }
        }
        if (!preferred_found) {
          for (std::size_t index = 1U; index < batch.candidate_count; ++index) {
            const double current_reject_s =
                results[diagnostic_index].selected_evaluation.reject_s_m;
            const double candidate_reject_s =
                results[index].selected_evaluation.reject_s_m;
            if (std::isfinite(candidate_reject_s) &&
                (!std::isfinite(current_reject_s) ||
                 candidate_reject_s > current_reject_s)) {
              diagnostic_index = index;
            }
          }
        }
      }
      const auto &diagnostic_work = batch.candidates[diagnostic_index];
      const auto &diagnostic_result = results[diagnostic_index];
      bool brain_hold_aborted = false;
      if (current && unexpired && !shadow_only_) {
        if (batch.brain_owned && effective_brain_hold.has_value()) {
          // A valid continuation would already have participated in selection.
          // No second, differently scored hold path can override that decision.
          record_publication_latency(publishOutput(bounded_batch_fallback));
          std::lock_guard<std::mutex> lock(input_mutex_);
          ++execution_revision_;
          if(isConnectedReturn(bounded_batch_fallback)) adoptBrainReturn(bounded_batch_fallback);
          brain_hold_aborted = true;
        } else {
          record_publication_latency(publishOutput(bounded_batch_fallback));
          if (batch.brain_owned) {
            std::lock_guard<std::mutex> lock(input_mutex_);
            ++execution_revision_;
            last_recovery_plan_.reset();
          }
        }
      }
      authority_lock.unlock();
      decision_lock.unlock();
      emit_execution_diagnostics();
      publishStatus(
          diagnostic_work.command, &diagnostic_result,
          !diagnostic_result.valid
              ? (brain_hold_aborted
                     ? (hold_reject_reason == mppi::RejectReason::COLLISION
                            ? "brain:abort_opponent_clearance"
                            : "brain:abort_wall_clearance")
                 : batch.brain_owned && effective_brain_hold.has_value()
                     ? "brain:hold_last_valid"
                 : batch.brain_owned ? "brain:fallback_rejected"
                                     : "fallback:rejected")
          : !certified[diagnostic_index]
              ? (batch.brain_owned ? "brain:fallback_internal_consistency"
                                   : "fallback:tube_violation")
          : shadow_only_ ? "shadow:valid"
          : !current     ? "fallback:superseded"
          : !unexpired   ? "fallback:expired"
                         : "fallback:no_selection");
    }
  }

  double wallPathCheckStep(const nav_msgs::msg::OccupancyGrid &map) const {
    return std::clamp(0.5 * static_cast<double>(map.info.resolution), 0.02,
                      0.10);
  }

  bool commandFootprintPathFree(
      const Command &command,
      const nav_msgs::msg::OccupancyGrid::SharedPtr &wall_map,
      const OccupancyGridWallIndex *index = nullptr) const {
    if (wall_map == nullptr || command.trajectory.points.size() < 2U) {
      return false;
    }
    std::vector<WallFootprintPose> path;
    path.reserve(command.trajectory.points.size());
    for (const auto &point : command.trajectory.points) {
      path.push_back({point.pose.position.x, point.pose.position.y,
                      yawFromQuaternion(point.pose.orientation)});
    }
    return occupancyGridFootprintPathFree(
        *wall_map, path, brain_wall_footprint_, wallPathCheckStep(*wall_map), index);
  }

  mppi::TemporaryReference
  temporaryReferenceFromCommand(const Command &command) const {
    mppi::TemporaryReference reference;
    const std::size_t count =
        std::min(command.trajectory.points.size(),
                 static_cast<std::size_t>(mppi::kMaximumReferencePoints));
    for (std::size_t index = 0U; index < count; ++index) {
      const auto &source = command.trajectory.points[index];
      auto &point = reference.points[reference.count];
      if (reference.count > 0U) {
        const auto &previous =
            command.trajectory.points[reference.count - 1U].pose.position;
        point.s_m = reference.points[reference.count - 1U].s_m +
                    distance(previous, source.pose.position);
      }
      point.x_m = source.pose.position.x;
      point.source_s_m=point.s_m;
      point.y_m = source.pose.position.y;
      point.yaw_rad = yawFromQuaternion(source.pose.orientation);
      point.speed_mps = source.longitudinal_velocity_mps;
      point.uncapped_speed_mps=point.speed_mps;
      ++reference.count;
    }
    for(std::size_t i=1;i+1<reference.count;++i) {
      const auto &a=reference.points[i-1]; auto &b=reference.points[i]; const auto &c=reference.points[i+1];
      const double ab=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m),bc=std::hypot(c.x_m-b.x_m,c.y_m-b.y_m);
      const double ca=std::hypot(a.x_m-c.x_m,a.y_m-c.y_m);
      b.curvature_1pm=ab*bc*ca>1e-9 ?
          2*((b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m))/(ab*bc*ca):0.0;
    }
    if(reference.count>=3) {
      reference.points[0].curvature_1pm=reference.points[1].curvature_1pm;
      reference.points[reference.count-1].curvature_1pm=reference.points[reference.count-2].curvature_1pm;
    }
    return reference;
  }

  mppi::RejectReason brainPathConstraintRejectReason(
      const mppi::TemporaryReference &reference,
      const nav_msgs::msg::OccupancyGrid::SharedPtr &wall_map,
      const std::vector<ObservedVehicle> &opponents, double ego_speed_mps,
      const Trajectory::SharedPtr &global_reference,
      const geometry_msgs::msg::Pose &ego_pose,
      double required_opponent_clearance_m,
      const OccupancyGridWallIndex *index = nullptr) const {
    if (reference.count < 2U || wall_map == nullptr ||
        global_reference == nullptr || global_reference->points.empty()) {
      return mppi::RejectReason::INVALID_INPUT;
    }
    Command command;
    command.trajectory.points.reserve(reference.count);
    for (std::size_t index = 0U; index < reference.count; ++index) {
      auto point =
          global_reference
              ->points[std::min(index, global_reference->points.size() - 1U)];
      const auto &source = reference.points[index];
      point.pose.position.x = source.x_m;
      point.pose.position.y = source.y_m;
      point.pose.orientation = quaternionFromYaw(source.yaw_rad);
      point.longitudinal_velocity_mps = static_cast<float>(source.speed_mps);
      command.trajectory.points.push_back(std::move(point));
    }
    (void)opponents; (void)ego_speed_mps; (void)ego_pose;
    (void)required_opponent_clearance_m;
    // Check exactly the supplied geometry. Dynamic interactions belong only
    // to the execution-time sweep; no independent segmentArrival clock here.
    if (!commandFootprintPathFree(command, wall_map, index)) return mppi::RejectReason::WALL;
    return mppi::RejectReason::NONE;
  }

  mppi::PathConstraintValidator
  makeBrainPathConstraintValidator(const BrainInputs &inputs,
                                   const nav_msgs::msg::Odometry &odometry,
                                   double required_opponent_clearance_m) const {
    const auto wall_map = inputs.wall_map;
    const auto wall_index = inputs.wall_index;
    const auto opponents = inputs.opponents;
    const auto global_reference = inputs.base_reference;
    const auto ego_pose = odometry.pose.pose;
    const double ego_speed_mps = std::abs(odometry.twist.twist.linear.x);
    return [this, wall_map, wall_index, opponents, global_reference, ego_pose,
            ego_speed_mps, required_opponent_clearance_m](
               const mppi::TemporaryReference &reference) {
      return brainPathConstraintRejectReason(
          reference, wall_map, opponents, ego_speed_mps, global_reference,
          ego_pose, required_opponent_clearance_m, wall_index.get());
    };
  }

  mppi::RolloutConstraintValidator
  makeBrainRolloutConstraintValidator(const BrainInputs &inputs) const {
    const auto wall_map = inputs.wall_map;
    const auto wall_index = inputs.wall_index;
    const auto global_reference = inputs.base_reference;
    if (global_reference == nullptr) {
      return [](const mppi::RolloutState &, const mppi::RolloutState &) {
        return mppi::RejectReason::INVALID_INPUT;
      };
    }
    const auto pose_index = sharedReferencePoseIndex(global_reference);
    std::vector<mppi::DynamicObstacle> opponents;
    if (global_reference != nullptr) {
      opponents.reserve(inputs.opponents.size());
      for (const auto &opponent : inputs.opponents) {
        const auto projection =
            projectOnTrajectory(*global_reference, opponent.x_m, opponent.y_m);
        if (!projection.valid) {
          continue;
        }
        const double longitudinal_speed =
            opponent.vx_mps * projection.tangent_x +
            opponent.vy_mps * projection.tangent_y;
        const double lateral_speed = -opponent.vx_mps * projection.tangent_y +
                                     opponent.vy_mps * projection.tangent_x;
        const double relative_yaw =
            std::hypot(longitudinal_speed, lateral_speed) >= 0.20
                ? std::atan2(lateral_speed, longitudinal_speed)
                : 0.0;
        mppi::DynamicObstacle predicted;
        predicted.s_m = predicted.global_reference_s_m = projection.s_m;
        predicted.d_m = projection.d_m;
        predicted.longitudinal_speed_mps = longitudinal_speed;
        predicted.lateral_speed_mps = lateral_speed;
        predicted.heading_relative_to_reference_rad = relative_yaw;
        predicted.prediction = opponent.prediction;
        opponents.push_back(std::move(predicted));
      }
    }
    // Copies of this callback execute on different line workers. Cache exact
    // times within this immutable input snapshot, never across new V2X data.
    const auto cache = std::make_shared<OpponentPoseCache>();
    return [this, wall_map, wall_index, global_reference, pose_index, cache,
            opponents = std::move(opponents)](
               const mppi::RolloutState &previous,
               const mppi::RolloutState &state) {
      if (wall_map == nullptr || global_reference == nullptr ||
          !std::isfinite(previous.x_m) || !std::isfinite(previous.y_m) ||
          !std::isfinite(previous.yaw_rad) ||
          !std::isfinite(previous.time_sec) || !std::isfinite(state.x_m) ||
          !std::isfinite(state.y_m) || !std::isfinite(state.yaw_rad) ||
          !std::isfinite(state.time_sec) ||
          state.time_sec <= previous.time_sec) {
        return mppi::RejectReason::INVALID_INPUT;
      }
      const double yaw_delta =
          std::remainder(state.yaw_rad - previous.yaw_rad, 2.0 * M_PI);
      const double body_radius_m = std::max(brain_wall_footprint_.radius(), std::hypot(
          std::max(brain_footprint_front_m_, brain_footprint_rear_m_),
          brain_footprint_radius_m_));
      const double translation_m =
          std::hypot(state.x_m - previous.x_m, state.y_m - previous.y_m);
      const double wall_step_m = wallPathCheckStep(*wall_map);
      const std::size_t subdivisions = std::max(
          {std::size_t{1U},
           static_cast<std::size_t>(std::ceil(translation_m / wall_step_m)),
           static_cast<std::size_t>(
               std::ceil(std::abs(yaw_delta) * body_radius_m / wall_step_m)),
           static_cast<std::size_t>(
               std::ceil((state.time_sec - previous.time_sec) / 0.025))});
      for (std::size_t sample = 0U; sample <= subdivisions; ++sample) {
        const double ratio =
            static_cast<double>(sample) / static_cast<double>(subdivisions);
        const double sample_time =
            previous.time_sec + ratio * (state.time_sec - previous.time_sec);
        const double ego_x = previous.x_m + ratio * (state.x_m - previous.x_m);
        const double ego_y = previous.y_m + ratio * (state.y_m - previous.y_m);
        const double ego_yaw = previous.yaw_rad + ratio * yaw_delta;
        if (!footprintFree(*wall_map, ego_x, ego_y, ego_yaw, wall_index.get())) {
          return mppi::RejectReason::WALL;
        }
        const auto predicted = cache->at(sample_time, [&] {
          OpponentPoseCache::Poses poses;
          poses.reserve(opponents.size());
          for (const auto &opponent : opponents) {
            const auto p = opponent_prediction::positionAt(opponent, sample_time);
            auto pose = pose_index->pose(p.global_s, p.d);
            if (pose) (*pose)[2] += p.relative_yaw;
            poses.push_back(std::move(pose));
          }
          return poses;
        });
        for (std::size_t i = 0U; i < opponents.size(); ++i) {
          const auto &pose = (*predicted)[i];
          if (!pose.has_value()) {
            return mppi::RejectReason::INVALID_INPUT;
          }
          if (!footprintsHaveClearance(
                  ego_x, ego_y, ego_yaw, pose->at(0U), pose->at(1U),
                  pose->at(2U), 0.0)) {
            return mppi::RejectReason::COLLISION;
          }
        }
      }
      return mppi::RejectReason::NONE;
    };
  }

  mppi::RejectReason
  validateBrainCommand(const Command &command,
                       const mppi::PathConstraintValidator &validator) const {
    if (!validator) {
      return mppi::RejectReason::INVALID_INPUT;
    }
    return validator(temporaryReferenceFromCommand(command));
  }

  std::optional<Command> recoverBrainCommandSpeed(
      const Command &desired, const mppi::PathConstraintValidator &validator) const {
    const auto recovered=mppi::recoverExecutionSpeed(temporaryReferenceFromCommand(desired),validator);
    if (!recovered) return std::nullopt;
    auto output=desired;
    for (std::size_t i=0; i<recovered->count; ++i)
      output.trajectory.points[i].longitudinal_velocity_mps=recovered->points[i].speed_mps;
    output.reason="mppi_brain:feasible_speed_fallback";
    return output;
  }

  Command makeBatchFallback(const WorkBatch &batch,
                           const std::optional<ExecutionSelection> &selection) const {
    if(!batch.brain_owned || selection) return batch.fallback;
    if(batch.global_reference && !batch.candidates.empty()) {
      const auto &work=batch.candidates.front();
      if (!batch.brain_hold && batch.ordinary_hold) {
        BrainInputs snapshot;
        snapshot.base_reference=batch.global_reference;
        snapshot.odometry=work.odometry;
        snapshot.ordinary_hold=batch.ordinary_hold;
        snapshot.opponents=batch.opponents;
        snapshot.steering_rad=work.request.ego.steering_rad;
        snapshot.steering_fresh=true;
        return ordinaryExecutionFallback(snapshot,batch.fallback,batch.fallback_path_constraint_validator);
      }
      return connectedReturnFallback(batch.fallback,work.odometry,
          work.request.ego.steering_rad,*batch.global_reference,batch.brain_hold,
          batch.allow_continuation || batch.brain_hold ? batch.maneuver : DrivingMode::FREE_RUN,
          batch.fallback_path_constraint_validator,batch.opponents,batch.speed_profile.get());
    }
    return physicallyBoundedFallback(batch.fallback,batch.fallback_path_constraint_validator);
  }

  static bool isConnectedReturn(const Command &command) {
    return command.reason.rfind("mppi_brain:return_connection",0)==0;
  }

  static bool isAlongsideHoldFallback(const Command &command) {
    return command.reason.rfind("mppi_brain:alongside_hold",0)==0;
  }

  bool hasAlongsideVehicle(const Trajectory &reference,
      const nav_msgs::msg::Odometry &odometry,
      const std::vector<ObservedVehicle> &opponents) const {
    const auto ego=projectOnTrajectory(reference,odometry.pose.pose.position.x,
        odometry.pose.pose.position.y);
    if(!ego.valid) return false;
    const bool closed=distance(reference.points.front().pose.position,
        reference.points.back().pose.position)<5.;
    const double length=trajectoryArcLength(reference);
    const auto extent=[&](double yaw) {
      return config_.vehicle_half_length_m*std::abs(std::cos(yaw))+
          config_.vehicle_half_width_m*std::abs(std::sin(yaw));
    };
    const double ego_extent=extent(yawFromQuaternion(odometry.pose.pose.orientation)-
        std::atan2(ego.tangent_y,ego.tangent_x));
    for(const auto &other:opponents) {
      const auto projected=projectOnTrajectory(reference,other.x_m,other.y_m);
      if(!projected.valid || projected.distance_m>5.) continue;
      const double tangent=std::atan2(projected.tangent_y,projected.tangent_x);
      const double yaw=std::hypot(other.vx_mps,other.vy_mps)>=.20 ?
          std::atan2(other.vy_mps,other.vx_mps) : tangent;
      // Same course station with overlapping body lengths, displaced sideways.
      // This includes a car whose center has just fallen behind the ego.
      if(std::abs(signedReferenceDelta(projected.s_m,ego.s_m,length,closed))<=
          ego_extent+extent(yaw-tangent) &&
          std::abs(projected.d_m-ego.d_m)>=brain_footprint_radius_m_)
        return true;
    }
    return false;
  }

  Command connectedReturnFallback(const Command &desired,
      const nav_msgs::msg::Odometry &odometry,double steering,
      const Trajectory &global,const std::optional<Command> &active,
      DrivingMode mode,const mppi::PathConstraintValidator &validator,
      const std::vector<ObservedVehicle> &opponents={},
      const mppi::TemporaryReference *speed_profile=nullptr) const {
    const bool keep_geometry=active && hasAlongsideVehicle(global,odometry,opponents);
    if(keep_geometry) {
      auto hold=refreshedBrainHold(*active,desired,odometry,global,speed_profile);
      if(hold.trajectory.points.size()>=3U) {
        if(validateBrainCommand(hold,validator)==mppi::RejectReason::NONE) {
          hold.reason="mppi_brain:alongside_hold";
          return hold;
        }
        if(auto recovered=recoverBrainCommandSpeed(hold,validator)) {
          recovered->reason="mppi_brain:alongside_hold:speed_recovery";
          return *recovered;
        }
      }
      // Keep the committed geometry for the existing braking fallback as well;
      // a failed candidate is not a reason to steer into the adjacent car.
    } else {
      if(mode==DrivingMode::FREE_RUN) return physicallyBoundedFallback(desired,validator);
      auto connected=makeBrainBaseCommand(desired,global,odometry,steering,true);
      if(connected && isConnectedReturn(*connected)) {
        const double speed=desired.trajectory.points.empty()?0.:
            desired.trajectory.points.front().longitudinal_velocity_mps;
        for(auto &p:connected->trajectory.points)
          p.longitudinal_velocity_mps=std::min<double>(p.longitudinal_velocity_mps,std::max(0.,speed));
        connected->mode=drivingModeName(mode);
        auto bounded=physicallyBoundedFallback(*connected,validator);
        if(validateBrainCommand(bounded,validator)==mppi::RejectReason::NONE) {
          bounded.reason="mppi_brain:return_connection:"+bounded.reason;
          return bounded;
        }
      }
    }
    Command braking=active?refreshedBrainHold(*active,desired,odometry,global,speed_profile):desired;
    if(!active || braking.trajectory.points.size()<3) {
      // If no committed geometry remains, braking follows the measured
      // curvature from the current pose, not an unrelated global path.
      braking.trajectory.points.clear();
      const auto &pose=odometry.pose.pose;
      const double yaw=yawFromQuaternion(pose.orientation);
      const double k=std::tan(std::clamp(steering,-config_.maximum_tire_steering_angle_rad,
          config_.maximum_tire_steering_angle_rad))/config_.wheel_base_m;
      const double length=std::min(60.,std::max(10.,suppliedDistance(odometry.twist.twist.linear.x)));
      const auto seed=desired.trajectory.points.empty()?autoware_auto_planning_msgs::msg::TrajectoryPoint{}:
          desired.trajectory.points.front();
      for(std::size_t i=0;i<301;++i) {
        const double s=length*i/300.;auto p=seed;p.pose=pose;
        p.pose.position.x+=std::abs(k)>1e-8?(std::sin(yaw+k*s)-std::sin(yaw))/k:s*std::cos(yaw);
        p.pose.position.y+=std::abs(k)>1e-8?(-std::cos(yaw+k*s)+std::cos(yaw))/k:s*std::sin(yaw);
        p.pose.orientation=quaternionFromYaw(yaw+k*s);
        braking.trajectory.points.push_back(p);
      }
    }
    braking.mode=drivingModeName(mode);
    for(auto &p:braking.trajectory.points) {
      p.longitudinal_velocity_mps=0.;p.acceleration_mps2=-config_.maximum_deceleration_mps2;
    }
    braking.reason=keep_geometry ? "mppi_brain:alongside_hold:" : "mppi_brain:return_unavailable:";
    braking.reason+=validateBrainCommand(braking,validator)==mppi::RejectReason::NONE?
        "validated_braking_fallback":"infeasible_braking_fallback";
    return braking;
  }

  // Caller owns input_mutex_. A fallback return is an adopted execution shape
  // and participates in the same continuation/retiming checks as other paths.
  void adoptBrainReturn(const Command &command) {
    clearBrainExecution();
    active_brain_command_=command;
    active_brain_speed_profile_=std::make_shared<const mppi::TemporaryReference>(
        mppi::executionSpeedProfile(temporaryReferenceFromCommand(command)));
    ++execution_revision_;
    const auto &p=command.trajectory.points.front().pose.position;
    RCLCPP_INFO(get_logger(),
        "[MPPI_RETURN_ADOPTED] generation=%lu stamp=%.9f start=[%.6f,%.6f] points=%zu reason=%s",
        command.generation,rclcpp::Time(command.header.stamp).seconds(),p.x,p.y,
        command.trajectory.points.size(),command.reason.c_str());
  }

  Command physicallyBoundedFallback(
      const Command &desired,
      const mppi::PathConstraintValidator &validator) const {
    if (validateBrainCommand(desired, validator) == mppi::RejectReason::NONE) {
      return desired;
    }
    if (const auto recovered=recoverBrainCommandSpeed(desired,validator)) return *recovered;
    Command blocked = desired;
    // No feasible desired path: retain the existing maximum-braking output.
    // A zero target does NOT imply an immediately stopped or collision-free
    // vehicle. Re-evaluate it and report infeasibility instead of certifying it.
    for (auto &point : blocked.trajectory.points) {
      point.longitudinal_velocity_mps = 0.0F;
      point.acceleration_mps2 =
          static_cast<float>(-config_.maximum_deceleration_mps2);
    }
    blocked.mode = desired.mode;
    blocked.emergency_stop = false;
    const auto braking_reason=validateBrainCommand(blocked,validator);
    blocked.reason=braking_reason==mppi::RejectReason::NONE ?
        "mppi_brain:validated_braking_fallback":"mppi_brain:infeasible_braking_fallback";
    blocked.valid_until_sec=desired.valid_until_sec;
    return blocked;
  }

  static bool withinCertifiedTube(const mppi::PlanResult &result,
                                  double half_width_m) {
    if (!result.valid || !std::isfinite(half_width_m) || half_width_m <= 0.0) {
      return false;
    }
    for (std::size_t index = 0U; index < result.selected_reference.count;
         ++index) {
      if (!std::isfinite(result.selected_reference.points[index].d_m) ||
          std::abs(result.selected_reference.points[index].d_m) >
              half_width_m + 1.0e-6) {
        return false;
      }
    }
    return result.selected_reference.count >= 3U;
  }

  // Caller owns input_mutex_; geometry and speed state have one reset boundary.
  void clearBrainExecution() {
    last_recovery_plan_.reset();
    active_brain_command_.reset();
    active_brain_speed_profile_.reset();
    active_brain_speed_generation_=0;
    active_preparation_.reset();
    latest_preparation_.reset();
    latest_ot_lane_entry_.reset();
    active_maneuver_target_id_.clear();
  }

  Command refreshedBrainHold(const Command &active, const Command &heartbeat,
                             const nav_msgs::msg::Odometry &odometry,
                             const Trajectory & /*global_reference*/,
                             const mppi::TemporaryReference *speed_profile=nullptr) const {
    Command hold = active;
    hold.header = heartbeat.header;
    hold.header.stamp = odometry.header.stamp;
    hold.trajectory.header = hold.header;
    hold.generation = heartbeat.generation;
    hold.valid_until_sec = rclcpp::Time(odometry.header.stamp).seconds()+0.20;
    hold.reason = "mppi_brain:hold_last_valid";
    mppi::EgoState ego;
    ego.x_m = odometry.pose.pose.position.x;
    ego.y_m = odometry.pose.pose.position.y;
    ego.yaw_rad = yawFromQuaternion(odometry.pose.pose.orientation);
    const auto remaining = mppi::remainingExecutionTrajectory(
        temporaryReferenceFromCommand(active), ego, suppliedDistance(odometry.twist.twist.linear.x),speed_profile);
    hold.trajectory.points.clear();
    hold.remaining_arc_m = 0.;
    if (!remaining) {
      hold.reason = "mppi_brain:continuation_exhausted";
      return hold;
    }
    for (std::size_t i = 0; i < remaining->count; ++i) {
      auto point = active.trajectory.points.front();
      const auto &p = remaining->points[i];
      point.pose.position.x = p.x_m;
      point.pose.position.y = p.y_m;
      point.pose.orientation = quaternionFromYaw(p.yaw_rad);
      point.longitudinal_velocity_mps = static_cast<float>(p.speed_mps);
      hold.trajectory.points.push_back(std::move(point));
    }
    hold.remaining_arc_m = static_cast<float>(remaining->points[remaining->count-1].s_m);
    return hold;
  }

  bool footprintsHaveClearance(double first_x_m, double first_y_m,
                               double first_yaw_rad, double second_x_m,
                               double second_y_m, double second_yaw_rad,
                               double required_clearance_m) const {
    const double center_offset_m =
        0.5 * (brain_footprint_front_m_ - brain_footprint_rear_m_);
    const double half_length_m =
        0.5 * (brain_footprint_front_m_ + brain_footprint_rear_m_);
    const double first_cosine = std::cos(first_yaw_rad);
    const double first_sine = std::sin(first_yaw_rad);
    const double second_cosine = std::cos(second_yaw_rad);
    const double second_sine = std::sin(second_yaw_rad);
    first_x_m += center_offset_m * first_cosine;
    first_y_m += center_offset_m * first_sine;
    second_x_m += center_offset_m * second_cosine;
    second_y_m += center_offset_m * second_sine;
    const double delta_x_m = second_x_m - first_x_m;
    const double delta_y_m = second_y_m - first_y_m;
    const std::array<std::array<double, 2U>, 4U> axes{{
        {{first_cosine, first_sine}},
        {{-first_sine, first_cosine}},
        {{second_cosine, second_sine}},
        {{-second_sine, second_cosine}},
    }};
    for (const auto &axis : axes) {
      const double center_separation_m =
          std::abs(delta_x_m * axis[0U] + delta_y_m * axis[1U]);
      const double first_radius_m =
          half_length_m *
              std::abs(axis[0U] * first_cosine + axis[1U] * first_sine) +
          brain_footprint_radius_m_ *
              std::abs(-axis[0U] * first_sine + axis[1U] * first_cosine);
      const double second_radius_m =
          half_length_m *
              std::abs(axis[0U] * second_cosine + axis[1U] * second_sine) +
          brain_footprint_radius_m_ *
              std::abs(-axis[0U] * second_sine + axis[1U] * second_cosine);
      if (center_separation_m >
          first_radius_m + second_radius_m + required_clearance_m) {
        return true;
      }
    }
    return false;
  }

  Command refinedCommand(const WorkItem &work,
                         const mppi::PlanResult &result) const {
    Command output = work.command;
    output.trajectory.points.clear();
    for (std::size_t index = 0U; index < result.selected_reference.count;
         ++index) {
      auto point = work.command.trajectory.points[work.source_indices[index]];
      const auto &selected = result.selected_reference.points[index];
      point.pose.position.x = selected.x_m;
      point.pose.position.y = selected.y_m;
      point.pose.orientation = quaternionFromYaw(selected.yaw_rad);
      point.longitudinal_velocity_mps = static_cast<float>(selected.speed_mps);
      output.trajectory.points.push_back(std::move(point));
    }
    if (work.brain_owned) {
      output.mode = drivingModeName(work.maneuver);
      output.emergency_stop = false;
      output.corridor_clearance_profile = true;
      output.corridor_side = work.request.phase == mppi::Phase::MERGE ? 0 :
          static_cast<std::int8_t>(work.request.side);
      output.wall_edge_profile = false;
      output.wall_side = 0;
      const float reserve =
          std::isfinite(result.selected_evaluation.minimum_clearance_m)
              ? static_cast<float>(std::clamp(
                    result.selected_evaluation.minimum_clearance_m, 0.0, 2.0))
              : 2.0F;
      output.wall_clearance_reserve_m = reserve;
      output.opponent_clearance_reserve_m = reserve;
      output.corridor_clearance_reserve_m = reserve;
      output.desired_control_reserve_m = std::min(0.10F, reserve);
      output.valid_until_sec = work.command.valid_until_sec;
      output.remaining_arc_m = static_cast<float>(trajectoryArcLength(output.trajectory));
      output.reason = work.request.phase == mppi::Phase::MERGE ? "mppi_brain:return" :
          work.maneuver==DrivingMode::OVERTAKE ?
              (work.request.side>0 ? "mppi_brain:overtake_left" : "mppi_brain:overtake_right") :
          work.request.side > 0 ? "mppi_brain:avoid_left" : "mppi_brain:avoid_right";
    } else {
      const float consumed =
          static_cast<float>(std::abs(result.selected.d_pass_m));
      output.wall_clearance_reserve_m =
          std::max(0.0F, output.wall_clearance_reserve_m - consumed);
      output.opponent_clearance_reserve_m =
          std::max(0.0F, output.opponent_clearance_reserve_m - consumed);
      output.corridor_clearance_reserve_m =
          std::max(0.0F, output.corridor_clearance_reserve_m - consumed);
      output.reason = (output_reason_prefix_ + std::string(work.command.reason))
                          .substr(0U, 128U);
    }
    return output;
  }

  void publishBaseline(const Command &command) const {
    visualization_msgs::msg::Marker marker;
    marker.header = command.header;
    marker.ns = "mppi_state_lattice_input";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.07;
    marker.color.r = 0.15F;
    marker.color.g = 0.45F;
    marker.color.b = 1.0F;
    marker.color.a = 0.85F;
    for (const auto &point : command.trajectory.points) {
      marker.points.push_back(point.pose.position);
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    baseline_pub_->publish(markers);
  }

  visualization_msgs::msg::MarkerArray
  makeOpponentPredictionMarkers(const BrainInputs &inputs) const {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray out;
    std_msgs::msg::Header header;
    if (inputs.base_reference) header = inputs.base_reference->header;
    if (inputs.odometry) header.stamp = inputs.odometry->header.stamp;
    if (header.frame_id.empty()) header.frame_id = "map";
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);
    if (!inputs.base_reference || inputs.base_reference->points.size() < 3U)
      return out;

    const auto shared_world = sharedReferencePoseIndex(inputs.base_reference);
    const auto &world = *shared_world;
    constexpr std::array<std::array<float, 3U>, 3U> colors{{
        {1.0F, .8F, .1F}, {.25F, 1.0F, .45F}, {1.0F, .35F, .2F}}};
    for (const auto &opponent : inputs.opponents) {
      if (!opponent.prediction || opponent.prediction->points.empty()) continue;
      const auto &prediction = *opponent.prediction;
      const double horizon = prediction.points.back().time;
      std::uint32_t color_key = 0U;
      for (const unsigned char c : opponent.id) color_key = color_key * 31U + c;
      const auto &color = colors[color_key % colors.size()];
      const auto marker = [&](int id, int type) {
        Marker m;
        m.header = header;
        m.ns = "mppi_opponent_prediction/" + opponent.id;
        m.id = id; m.type = type; m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2];
        m.color.a = .9F;
        m.lifetime = rclcpp::Duration::from_seconds(.5);
        return m;
      };
      const auto position = [&](double time) -> std::optional<geometry_msgs::msg::Point> {
        // Prediction time zero is already aligned to the planning input.
        // Do not fit the history again or apply observation age a second time.
        const auto p = prediction.at(time);
        const auto pose = world.pose(p.global_s, p.d);
        if (!pose) return {};
        geometry_msgs::msg::Point point;
        point.x = (*pose)[0]; point.y = (*pose)[1]; point.z = .35;
        return point;
      };
      auto path = marker(0, Marker::LINE_STRIP);
      path.scale.x = .10;
      const auto steps = std::max(1U, static_cast<unsigned>(std::ceil(horizon/.1)));
      for (unsigned i = 0; i <= steps; ++i) {
        const auto point = position(horizon * i / steps);
        if (point) path.points.push_back(*point);
      }
      if (path.points.empty()) continue;
      auto name = marker(2, Marker::TEXT_VIEW_FACING);
      name.pose.position = path.points.front(); name.pose.position.z = 1.2;
      name.scale.z = .6;
      std::ostringstream text;
      text << opponent.id << " (0-" << horizon << " s)";
      name.text = text.str();
      out.markers.push_back(std::move(name));

      auto ticks = marker(1, Marker::SPHERE_LIST);
      ticks.scale.x = ticks.scale.y = ticks.scale.z = .24;
      auto last_label = path.points.front();
      for (int second = 1; second <= static_cast<int>(std::floor(horizon)); ++second) {
        const auto point = position(second);
        if (!point) continue;
        ticks.points.push_back(*point);
        // A stopped forecast has coincident future points; avoid stacked text.
        if (std::hypot(point->x-last_label.x, point->y-last_label.y) < .5) continue;
        auto label = marker(2 + second, Marker::TEXT_VIEW_FACING);
        label.pose.position = *point; label.pose.position.z = .8;
        label.scale.z = .45; label.text = "+" + std::to_string(second) + "s";
        out.markers.push_back(std::move(label));
        last_label = *point;
      }
      out.markers.push_back(std::move(ticks));
      out.markers.push_back(std::move(path));
    }
    return out;
  }

  // A diagnostic snapshot is not a publication verdict. All geometry below
  // comes from one completed batch, never from live ego/opponent inputs.
  void publishDecisionSnapshot(
      const WorkBatch &batch,
      const std::array<mppi::PlanResult, kBrainTemplateLineCount> &results,
      std::size_t selected_slot) const {
    const auto started = std::chrono::steady_clock::now();
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray out;
    Marker clear;
    clear.header = batch.fallback.header;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);
    const auto point = [](double x, double y, double z) {
      geometry_msgs::msg::Point p; p.x=x; p.y=y; p.z=z; return p;
    };
    for (std::size_t slot=0; slot<batch.candidate_count; ++slot) {
      const auto focus=decision_slot_>=0?static_cast<std::size_t>(decision_slot_):
          (selected_slot<batch.candidate_count?selected_slot:0U);
      if(slot!=focus) continue;
      const auto &work=batch.candidates[slot];
      const auto &result=results[slot];
      const auto draw = [&](const mppi::TemporaryReference &reference,
                            const mppi::Evaluation &ev, int profile) {
        const std::string key="g"+std::to_string(work.request.generation)+
            "/s"+std::to_string(slot)+"/p"+std::to_string(profile);
        int id=0;
        const auto marker = [&](const std::string &kind, int type,
                                float r,float g,float b) {
          Marker m; m.header=work.command.header; m.ns=key+"/"+kind;
          m.id=id++; m.type=type; m.action=Marker::ADD;
          m.pose.orientation.w=1; m.scale.x=.055;
          m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=.95;
          m.lifetime=rclcpp::Duration::from_seconds(1.5);
          return m;
        };
        auto ref=marker("reference_not_safety_certificate",Marker::LINE_STRIP,.3,.5,1);
        for(std::size_t i=0;i<reference.count;++i)
          ref.points.push_back(point(reference.points[i].x_m,reference.points[i].y_m,.35));
        if(ref.points.size()>=2) out.markers.push_back(ref);
        auto predicted=marker("predicted_prefix_only",Marker::LINE_STRIP,1,.8,.1);
        for(std::size_t i=0;i<ev.predicted_rollout_count;++i)
          predicted.points.push_back(point(ev.predicted_rollout[i].x_m,ev.predicted_rollout[i].y_m,.42));
        if(predicted.points.size()>=2) out.markers.push_back(predicted);
        const bool witness=ev.reject_dynamic_obstacle && std::string(ev.reject_stage)=="execution_sweep" &&
            ev.reject_obstacle_index<work.request.dynamic_obstacle_count &&
            ev.reject_obstacle_index<batch.opponents.size() && batch.global_reference;
        int body=-1;
        const auto body_marker = [&](double x,double y,double yaw,
                                     const std::string &name,float r,float g,float b) {
          auto m=marker(name,Marker::LINE_STRIP,r,g,b);
          for(const auto &q:std::array<std::array<double,2>,5>{{
              {{brain_footprint_front_m_,brain_footprint_radius_m_}},
              {{brain_footprint_front_m_,-brain_footprint_radius_m_}},
              {{-brain_footprint_rear_m_,-brain_footprint_radius_m_}},
              {{-brain_footprint_rear_m_,brain_footprint_radius_m_}},
              {{brain_footprint_front_m_,brain_footprint_radius_m_}}}})
            m.points.push_back(point(x+std::cos(yaw)*q[0]-std::sin(yaw)*q[1],
                y+std::sin(yaw)*q[0]+std::cos(yaw)*q[1],.5));
          out.markers.push_back(m);
        };
        if(witness) {
          body_marker(ev.reject_x_m,ev.reject_y_m,ev.reject_yaw_rad,"ego_at_reject",1,1,1);
          const auto predicted=opponent_prediction::positionAt(
              work.request.dynamic_obstacles[ev.reject_obstacle_index],ev.reject_time_sec);
          const auto poses=referencePoseIndex(*batch.global_reference);
          {
            const auto pose=poses.pose(predicted.global_s,predicted.d);
            if(pose) {
              const double yaw=pose->at(2)+predicted.relative_yaw;
              body=!footprintsHaveClearance(ev.reject_x_m,ev.reject_y_m,ev.reject_yaw_rad,
                  pose->at(0),pose->at(1),yaw,0);
              body_marker(pose->at(0),pose->at(1),yaw,"nominal_opponent_at_reject",1,.15,.15);
            }
          }
          // Same fixed-time union and dimensions as the production decision.
          // These polygons contain both bodies: compare the EGO POINT, not
          // the white ego rectangle, against them. Never hull across a bend.
          if(work.request.world_reference) for(int which=0;which<2;++which) {
            const auto &initial=work.request.dynamic_obstacles[ev.reject_obstacle_index];
            const double t=ev.reject_time_sec;
            const auto predicted=opponent_prediction::positionAt(initial,t);
            const double s=predicted.global_s;
            const double d=predicted.d;
            const double scale=mppi::collisionUncertaintyScale(t);
            const double hs=which?scale*(std::max(0.0,initial.longitudinal_uncertainty_m)+
                .5*std::max(0.0,initial.longitudinal_acceleration_bound_mps2)*t*t):0;
            const double hd=which?scale*std::max(0.0,initial.lateral_uncertainty_m):0;
            work.request.world_reference->visitOccupancy(s-hs,s+hs,d-hd,d+hd,
                [&](const ReferencePoseIndex::OccupancyTile &tile) {
              auto m=marker(which?"world_full_ego_point_forbidden":"world_nominal_ego_point_forbidden",
                            Marker::LINE_STRIP,1,which?.25F:.65F,which?1.F:.1F);
              for(const auto &p:ReferencePoseIndex::forbiddenPolygon(tile,ev.reject_yaw_rad,
                  predicted.relative_yaw,config_.vehicle_half_length_m,
                  config_.vehicle_half_width_m,config_.obstacle_longitudinal_inflation_m,
                  config_.obstacle_lateral_inflation_m))
                m.points.push_back(point(p[0],p[1],.47));
              out.markers.push_back(std::move(m));
            });
          }
        }
        auto label=marker("verdict",Marker::TEXT_VIEW_FACING,1,1,1);
        label.scale.z=.35;
        label.pose.position=point(work.request.ego.x_m,work.request.ego.y_m,1.+profile*.45+slot*.15);
        if(!predicted.points.empty()) label.pose.position=predicted.points.back();
        if(!ev.valid && std::isfinite(ev.reject_time_sec) && ev.reject_time_sec>=0 &&
            std::isfinite(ev.reject_x_m) && std::isfinite(ev.reject_y_m)) {
          label.pose.position=point(ev.reject_x_m,ev.reject_y_m,.8);
          auto hit=marker("rejection",Marker::SPHERE,1,.1,.1);
          hit.pose.position=label.pose.position; hit.scale.x=hit.scale.y=hit.scale.z=.25;
          out.markers.push_back(hit);
        }
        label.pose.position.z=1.+profile*.45;
        std::ostringstream text;
        text << key << "\n";
        if(ev.valid) text << (profile==0 && slot==selected_slot?"SLOT_CHOICE (not publish verdict)":"FEASIBLE (not publish verdict)");
        else text << Mppi::toString(ev.reject_reason) << "/" << ev.reject_stage;
        if(witness) text << (body==1?"\nBODY_OVERLAP":body==0?"\nENVELOPE_ONLY_HERE":"\nBODY_UNKNOWN")
                        << "\nworld union: EGO POINT forbidden";
        text << " t=" << ev.reject_time_sec << " cost=" << ev.cost
             << " prefix=" << ev.predicted_rollout_count
             << "\ntail UNVERIFIED; blue=ref yellow=prediction";
        label.text=text.str(); out.markers.push_back(label);
      };
      // One inspectable candidate, instead of overlapping world-space labels.
      const auto profile=static_cast<std::size_t>(decision_profile_);
      if(profile>0 && profile<=result.speed_pair_count && result.speed_pairs[profile-1].reference)
        draw(*result.speed_pairs[profile-1].reference,result.speed_pairs[profile-1].evaluation,profile);
      else draw(result.selected_reference,result.selected_evaluation,0);
    }
    decision_pub_->publish(out);
    RCLCPP_INFO(get_logger(),"[MPPI_DECISION_MARKERS] generation=%lu markers=%zu elapsed_ms=%.3f",
        static_cast<unsigned long>(batch.candidates[0].request.generation),out.markers.size(),
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
  }

  void publishCandidate(const WorkItem &work, const mppi::PlanResult &result,
                        bool certified,
                        visualization_msgs::msg::MarkerArray &markers,
                        const mppi::Evaluation *adoption = nullptr) const {
    const auto &display_evaluation = adoption ? *adoption : result.selected_evaluation;
    for (const auto *marker_namespace :
         {"mppi_candidate_valid", "mppi_candidate_rejected",
          "mppi_pp_predicted_rollout"}) {
      visualization_msgs::msg::Marker removal;
      removal.header = work.command.header;
      removal.ns = marker_namespace;
      removal.id = work.marker_id;
      removal.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(std::move(removal));
    }
    for (std::size_t sample_slot = 0U;
         sample_slot < result.visualized_sample_count; ++sample_slot) {
      const auto &reference = result.visualized_sample_references[sample_slot];
      if (reference.count < 2U) {
        continue;
      }
      const auto &evaluation =
          result.visualized_sample_evaluations[sample_slot];
      const bool collision =
          evaluation.reject_reason == mppi::RejectReason::COLLISION ||
          evaluation.reject_reason == mppi::RejectReason::WALL;
      visualization_msgs::msg::Marker rollout;
      rollout.header = work.command.header;
      rollout.ns = evaluation.valid ? "mppi_rollout_valid"
                   : collision      ? "mppi_rollout_collision"
                                    : "mppi_rollout_rejected";
      rollout.id =
          work.marker_id * 100 +
          static_cast<int>(result.visualized_sample_indices[sample_slot]);
      rollout.type = visualization_msgs::msg::Marker::LINE_STRIP;
      rollout.action = visualization_msgs::msg::Marker::ADD;
      rollout.scale.x = 0.025;
      rollout.color.r = evaluation.valid ? 0.20F : 1.0F;
      rollout.color.g = evaluation.valid ? 0.65F : (collision ? 0.05F : 0.55F);
      rollout.color.b =
          evaluation.valid && work.request.side > 0 ? 1.0F : 0.10F;
      rollout.color.a = evaluation.valid ? 0.50F : 0.65F;
      for (std::size_t index = 0U; index < reference.count; ++index) {
        geometry_msgs::msg::Point point;
        point.x = reference.points[index].x_m;
        point.y = reference.points[index].y_m;
        point.z = 0.12;
        rollout.points.push_back(point);
      }
      markers.markers.push_back(std::move(rollout));
    }
    if (result.selected_reference.count < 2U) {
      return;
    }
    const bool accepted = result.valid && certified;
    visualization_msgs::msg::Marker marker;
    marker.header = work.command.header;
    marker.ns = accepted ? "mppi_candidate_valid" : "mppi_candidate_rejected";
    marker.id = work.marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.07;
    marker.color.r = accepted ? 0.15F : 1.0F;
    marker.color.g = accepted ? 0.85F : 0.18F;
    marker.color.b = accepted && work.request.side > 0 ? 1.0F : 0.15F;
    marker.color.a = accepted ? 0.78F : 0.58F;
    for (std::size_t index = 0U; index < result.selected_reference.count;
         ++index) {
      geometry_msgs::msg::Point point;
      point.x = result.selected_reference.points[index].x_m;
      point.y = result.selected_reference.points[index].y_m;
      point.z = 0.18;
      marker.points.push_back(point);
    }
    if (work.brain_owned && !marker.points.empty()) {
      visualization_msgs::msg::Marker label;
      label.header = marker.header;
      label.ns = "mppi_local_horizon";
      label.id = work.marker_id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = marker.points.back();
      label.pose.position.z = 1.0;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.45;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
      std::ostringstream text;
      text << "local " << work.actual_horizon_m << "m / "
           << work.request.horizon_steps_override * config_.dt_sec << "s"
           << " slot=" << work.template_slot << " g=" << work.command.generation
           << " v=" << result.selected_reference.points.front().speed_mps
           << "->" << result.selected_reference.points[result.selected_reference.count - 1U].speed_mps
           << (adoption ? " adoption" : " raw")
           << (accepted ? " feasible (not publication)" : " rejected:")
           << (accepted ? "" : Mppi::toString(display_evaluation.reject_reason));
      label.text = text.str();
      markers.markers.push_back(std::move(label));
    }
    markers.markers.push_back(std::move(marker));
    const auto &predicted = display_evaluation.predicted_rollout;
    const auto predicted_count =
        display_evaluation.predicted_rollout_count;
    if (predicted_count >= 2U) {
      visualization_msgs::msg::Marker pp_rollout;
      pp_rollout.header = work.command.header;
      pp_rollout.ns = "mppi_pp_predicted_rollout";
      pp_rollout.id = work.marker_id;
      pp_rollout.type = visualization_msgs::msg::Marker::LINE_STRIP;
      pp_rollout.action = visualization_msgs::msg::Marker::ADD;
      pp_rollout.scale.x = 0.055;
      pp_rollout.color.r = 1.0F;
      pp_rollout.color.g = 0.75F;
      pp_rollout.color.b = 0.05F;
      pp_rollout.color.a = accepted ? 0.95F : 0.55F;
      for (std::size_t index = 0U; index < predicted_count; ++index) {
        geometry_msgs::msg::Point point;
        point.x = predicted[index].x_m;
        point.y = predicted[index].y_m;
        point.z = 0.24;
        pp_rollout.points.push_back(point);
      }
      markers.markers.push_back(std::move(pp_rollout));
    }
  }

  void clearCandidateMarkers(const std_msgs::msg::Header &header) const {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    candidates_pub_->publish(markers);
  }

  void publishApplied(const WorkItem &work,
                      const mppi::PlanResult &result) const {
    visualization_msgs::msg::Marker marker;
    marker.header = work.command.header;
    marker.ns = "mppi_applied";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.14;
    marker.color.r = 1.0F;
    marker.color.g = 0.15F;
    marker.color.b = 0.85F;
    marker.color.a = 1.0F;
    for (std::size_t index = 0U; index < result.selected_reference.count;
         ++index) {
      geometry_msgs::msg::Point point;
      point.x = result.selected_reference.points[index].x_m;
      point.y = result.selected_reference.points[index].y_m;
      point.z = 0.22;
      marker.points.push_back(point);
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    selected_pub_->publish(markers);
  }

  void publishAppliedCommand(const Command &command) const {
    if (command.trajectory.points.size() < 2U) {
      clearAppliedMarker(command.header);
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header = command.header;
    marker.ns = "mppi_applied";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.14;
    marker.color.r = 1.0F;
    marker.color.g = 0.15F;
    marker.color.b = 0.85F;
    marker.color.a = 1.0F;
    for (const auto &trajectory_point : command.trajectory.points) {
      auto point = trajectory_point.pose.position;
      point.z = 0.22;
      marker.points.push_back(std::move(point));
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    selected_pub_->publish(markers);
  }

  void clearAppliedMarker(const std_msgs::msg::Header &header) const {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    selected_pub_->publish(markers);
  }

  void publishStatus(const Command &command, const mppi::PlanResult *result,
                     const std::string &state) {
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"state\":\"" << state
         << "\",\"generation\":" << command.generation << ",\"mode\":\""
         << command.mode << "\"";
    if (result != nullptr) {
      json << ",\"valid\":" << (result->valid ? "true" : "false")
           << ",\"reason\":\"" << Mppi::toString(result->reject_reason)
           << "\",\"valid_samples\":" << result->valid_sample_count
           << ",\"elapsed_ms\":" << result->elapsed_ms
           << ",\"d_pass_m\":" << result->selected.d_pass_m
           << ",\"speed_start_mps\":" << (result->selected_reference.count > 0U ?
               result->selected_reference.points.front().speed_mps : 0.0)
           << ",\"speed_end_mps\":" << (result->selected_reference.count > 0U ?
               result->selected_reference.points[result->selected_reference.count - 1U].speed_mps : 0.0)
           << ",\"lateral_control_near\":"
           << result->selected.lateral_control_near_scale
           << ",\"lateral_control_far\":"
           << result->selected.lateral_control_far_scale << ",\"reject_step\":"
           << result->selected_evaluation.reject_step_index;
    }
    json << "}";
    message.data = json.str();
    status_pub_->publish(message);
    if (result != nullptr) {
      RCLCPP_INFO(get_logger(),
                  "[REFERENCE_SPACE_MPPI] state=%s generation=%lu mode=%s "
                  "valid=%s reason=%s dominant=%s valid_samples=%zu "
                  "elapsed_ms=%.3f d_pass_m=%.3f lateral_controls=[%.3f,%.3f] "
                  "reject_ref=%zu "
                  "reject_step=%zu reject_s=%.3f reject_d=%.3f "
                  "reject_bounds=[%.3f,%.3f] reject_k=%.3f reject_j=%.3f",
                  state.c_str(), static_cast<unsigned long>(command.generation),
                  command.mode.c_str(), result->valid ? "true" : "false",
                  Mppi::toString(result->reject_reason),
                  Mppi::toString(result->dominant_rejection.reject_reason),
                  result->valid_sample_count, result->elapsed_ms,
                  result->selected.d_pass_m,
                  result->selected.lateral_control_near_scale,
                  result->selected.lateral_control_far_scale,
                  result->selected_evaluation.reject_reference_index,
                  result->selected_evaluation.reject_step_index,
                  result->selected_evaluation.reject_s_m,
                  result->selected_evaluation.reject_d_m,
                  result->selected_evaluation.reject_minimum_d_m,
                  result->selected_evaluation.reject_maximum_d_m,
                  result->selected_evaluation.reject_curvature_1pm,
                  result->selected_evaluation.reject_frenet_jacobian);
    } else {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[REFERENCE_SPACE_MPPI] state=%s generation=%lu mode=%s",
          state.c_str(), static_cast<unsigned long>(command.generation),
          command.mode.c_str());
    }
  }

  bool enabled_{false};
  bool shadow_only_{false};
  bool brain_mode_{false};
  bool brain_control_sequence_sampling_enabled_{true};
  double tube_max_half_width_m_{0.25};
  double tube_min_half_width_m_{0.04};
  double nominal_lateral_refinement_m_{0.08};
  double base_curvature_limit_1pm_{1.0};
  std::string own_vehicle_id_;
  double brain_trigger_distance_m_{25.0};
  bool prior_lap_prediction_enabled_{false};
  bool leader_lap_prediction_enabled_{false};
  std::string leader_vehicle_id_;
  std::string active_maneuver_target_id_;
  std::string prediction_target_id_;
  double leader_receive_sec_{-1.};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr leader_sub_;
  bool recent_motion_prediction_enabled_{false};
  bool online_motion_prediction_enabled_{false};
  bool collection_motion_enabled_{false};
  bool collection_avoidance_continuation_{false};
  opponent_prediction::MotionPersistence motion_persistence_{};
  double brain_target_path_conflict_margin_m_{0.15};
  double brain_prediction_horizon_sec_{6.0};
  double brain_pass_offset_m_{1.85};
  double brain_min_pass_offset_m_{0.0};
  double brain_side_switch_minimum_cost_improvement_{0.0};
  double last_sample_diagnostic_sec_{-1.0}; // Written only by workerLoop.
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr recorded_wall_map_; // Worker diagnostics only.
  std::uint64_t recorded_wall_map_hash_{0};
  bool brain_sample_diagnostics_enabled_{false};
  std::size_t brain_samples_per_line_{10U};
  std::size_t brain_visualized_samples_per_line_{4U};
  std::size_t brain_max_result_generation_lag_{3U};
  double brain_max_track_offset_m_{2.5};
  double brain_footprint_radius_m_{0.65};
  double brain_footprint_front_m_{1.06};
  double brain_footprint_rear_m_{1.10};
  ConvexWallFootprint brain_wall_footprint_{
      {1.06, .65, -1.10, .65, -1.10, -.65, 1.06, -.65}};
  double brain_lateral_sample_step_m_{0.10};
  double brain_cruise_speed_mps_{10.0};
  double brain_reference_geometry_knot_spacing_m_{2.0};
  // Only the serialized brain heartbeat updates this target-scoped latch.
  DrivingFsm driving_fsm_;
  double brain_overtake_speed_mps_{10.0};
  double brain_minimum_passing_speed_mps_{6.0};
  double brain_minimum_passing_advantage_mps_{0.8};
  double brain_minimum_rolling_speed_mps_{0.5};
  double brain_minimum_proximity_speed_mps_{10.0};
  bool compare_return_continuation_{false};
  double gentle_lateral_acceleration_mps2_{0.0};
  double brain_hold_minimum_opponent_clearance_m_{0.0};
  double brain_hold_revalidation_horizon_sec_{1.50};
  double brain_input_timeout_sec_{0.5};
  double brain_hard_clearance_m_{0.0};
  double brain_reference_horizon_m_{60.0};
  double local_minimum_m_{20.0};
  double local_maximum_m_{30.0};
  double local_lookahead_sec_{3.0};
  double reference_supply_sec_{1.0};
  bool brain_internal_timer_enabled_{false};
  double brain_update_rate_hz_{20.0};
  bool brain_static_wall_map_enabled_{false};
  std::string output_reason_prefix_;
  mppi::Config config_{};
  bool decision_markers_enabled_{false};
  int decision_slot_{-1};
  int decision_profile_{1};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr decision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr passing_opportunity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr passing_point_forbidden_pub_;
  bool passing_point_display_enabled_{false},passing_point_display_published_{false};
  bool passing_point_areas_enabled_{false};
  bool multiple_passing_points_enabled_{false};
  PassingPointDisplayConfig passing_point_display_config_;
  std::mutex passing_point_display_mutex_;
  std::shared_ptr<const PrecomputedWallLines> displayed_passing_point_lines_;
  std::unique_ptr<Mppi> left_planner_;
  std::unique_ptr<Mppi> right_planner_;
  std::unique_ptr<mppi::ReferenceSpaceMppiBatchOptimizer> batch_optimizer_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  Trajectory::SharedPtr base_reference_;
  Trajectory::SharedPtr requested_reference_;
  std::uint64_t reference_revision_{0}, requested_reference_revision_{0};
  std::uint64_t vehicle_observation_sequence_{0};
  std::shared_ptr<const Command> published_command_;
  std::optional<Command> ordinary_hold_, reference_transition_;
  double reference_transition_end_m_{0.};
  std::unordered_set<std::string> alongside_vehicle_ids_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reference_contract_pub_;
  mutable std::mutex reference_index_mutex_;
  mutable Trajectory::SharedPtr indexed_reference_;
  mutable std::shared_ptr<const ReferencePoseIndex> reference_index_;
  mutable std::mutex wall_lines_mutex_;
  mutable std::shared_ptr<const PrecomputedWallLines> wall_lines_;
  mutable nav_msgs::msg::OccupancyGrid::SharedPtr wall_lines_map_;
  mutable std::uint64_t wall_lines_generation_{0};
  nav_msgs::msg::OccupancyGrid::SharedPtr wall_map_;
  std::shared_ptr<const OccupancyGridWallIndex> wall_index_;
  double steering_rad_{0.0};
  double steering_receive_sec_{-1.0};
  struct SteeringObservation {
    double stamp_sec, receive_sec, angle_rad;
    std::uint64_t sequence;
  };
  std::deque<SteeringObservation> steering_report_history_;
  std::uint64_t input_event_sequence_{0}, input_snapshot_sequence_{0};
  std::uint64_t odometry_sequence_{0}, control_sequence_{0};
  std::deque<mppi::TimedSteeringCommand> steering_command_history_;
  std::unordered_map<std::string, ObservedVehicle> observed_vehicles_;
  int preferred_side_{0};
  std::optional<Command> active_brain_command_;
  bool passing_preparation_enabled_{false};
  PreparationBoostConfig preparation_boost_config_;
  PreparationBoostPolicy preparation_boost_policy_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr preparation_boost_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr preparation_boost_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr preparation_boost_status_sub_;
  bool front_merge_attack_enabled_{false};
  mppi::PassingPreparationConfig preparation_config_;
  mppi::OtLaneEntryConfig ot_entry_config_;
  std::shared_ptr<const mppi::OtLaneEntryPlan> latest_ot_lane_entry_;
  std::shared_ptr<const mppi::PassingPreparationPlan> latest_preparation_,active_preparation_;
  std::uint64_t latest_preparation_input_{0};
  std::atomic<double> preparation_planning_delay_sec_{0.05};
  std::shared_ptr<const mppi::TemporaryReference> published_execution_reference_;
  std::shared_ptr<const mppi::TemporaryReference> active_brain_speed_profile_;
  std::uint64_t active_brain_speed_generation_{0};
  std::uint64_t execution_revision_{0}; // Protected by input_mutex_ and publication authority.
  std::optional<std::pair<Command,int>> last_recovery_plan_;
  std::uint64_t last_recovery_shape_{0}, last_recovery_speed_{0};
  // Heartbeat refresh and worker commit are single execution-owner decisions.
  // Expensive optimization remains outside these locks. Order when combined:
  // decision -> authority -> input. Never wait for a planner under input_mutex_.
  std::mutex decision_mutex_;
  std::mutex authority_mutex_;
  mutable std::mutex input_mutex_;
  std::mutex work_mutex_;
  std::condition_variable work_cv_;
  std::optional<WorkBatch> pending_batch_;
  bool stopping_{false};
  std::thread worker_;
  std::atomic<std::uint64_t> latest_generation_{0U};
  SequencedOutput sequenced_output_;
  std::atomic<std::uint64_t> latest_semantic_key_{0U};
  bool recovery_active_{false}; // Protected by decision_mutex_.
  std::uint64_t recovery_epoch_{0}; // Protected by decision_mutex_.
  std::uint64_t worker_recovery_epoch_{0}; // Worker thread only.
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recovery_service_;
  std::atomic<std::uint64_t> internal_generation_{0U};
  rclcpp::Publisher<Command>::SharedPtr output_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      baseline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      selected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      candidates_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      opponent_predictions_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<simple_pure_pursuit::RotationPredictionMessage>::SharedPtr
      rotation_prediction_sub_;
  std::deque<simple_pure_pursuit::RotationPredictionSnapshot> rotation_prediction_history_;
  rclcpp::Subscription<Trajectory>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_map_sub_;
  rclcpp::Subscription<
      autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<ControlCommand>::SharedPtr control_command_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr
      v2x_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr brain_timer_;
};

} // namespace reference_space_mppi_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<reference_space_mppi_planner::ReferenceSpaceMppiNode>());
  rclcpp::shutdown();
  return 0;
}
