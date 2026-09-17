#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_direct_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "simple_pure_pursuit/rotation_prediction_message.hpp"
#include "simple_pure_pursuit/steering_tracking_speed_gate.hpp"
#include "simple_pure_pursuit/wall_edge_tracking_speed.hpp"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using multi_purpose_mpc_ros_msgs::msg::StateLatticeDirectTrajectory;
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node {
public:
  SimplePurePursuit();

private:
  RotationControllerState rotation_controller_state_;
  rclcpp::Publisher<RotationPredictionMessage>::SharedPtr rotation_prediction_pub_;
  struct RuntimeTimingSample {
    double dispatch_interval_ms{0.0};
    double callback_wall_ms{0.0};
    double callback_thread_cpu_ms{0.0};
    double lock_wait_ms{0.0};
    double precheck_ms{0.0};
    double input_and_delay_ms{0.0};
    double path_preparation_ms{0.0};
    double preview_selection_ms{0.0};
    double control_ms{0.0};
    double publish_ms{0.0};
    double diagnostic_ms{0.0};
  };

  struct TuningParameters {
    double lookahead_gain{0.5};
    double lookahead_min_distance{3.5};
    double actual_lookahead_distance_blend{0.0};
    double dual_preview_near_ratio{0.5};
    double dual_preview_blend{0.0};
    double steering_tire_angle_gain{1.5};
    double curvature_lookahead_min_distance{2.0};
    double curvature_lookahead_sensitivity{8.0};
    double curvature_lookahead_smoothing_alpha{0.35};
    double max_lateral_acceleration{6.0};
    double minimum_corner_speed{5.0};
    double corner_speed_retention{0.0};
    bool lateral_error_speed_gate_enabled{true};
    double corner_speed_retention_lateral_error_soft{0.5};
    double corner_speed_retention_lateral_error_hard{1.0};
    double curvature_speed_preview_distance{12.0};
    double speed_proportional_gain{1.0};
    double longitudinal_acceleration_limit{1.0};
    double longitudinal_deceleration_limit{2.0};
    double safe_stop_deceleration{3.0};
    bool steering_command_passthrough_enabled{false};
    double steering_command_to_tire_angle_ratio{0.60};
    double hard_steering_angle_limit_rad{0.3665191429188092};
    double maximum_tire_steering_angle_rad{0.3141592653589793};
    double hard_steering_rate_limit_radps{4.0};
    double physical_tire_steering_rate_radps{0.0};
    bool steering_demand_acceleration_hold_enabled{false};
    double steering_acceleration_hold_minimum_speed_mps{3.0};
    double steering_acceleration_hold_minimum_tire_angle_rad{0.15};
    double steering_acceleration_hold_tracking_error_rad{0.08};
    double steering_acceleration_hold_maximum_acceleration_mps2{0.0};
    double external_target_vel{9.722222222222};
    bool continuous_preview_interpolation_enabled{false};
    bool delay_compensation_enabled{false};
    double pp_control_delay_sec{0.2};
    double pp_prediction_dt_sec{0.02};
    double steering_time_constant_sec{0.30};
    double steering_status_timeout_sec{0.20};
    double min_velocity_for_delay_compensation_mps{0.20};
    bool curvature_feedforward_enabled{false};
    double curvature_feedforward_gain{0.20};
    double curvature_feedforward_preview_distance_m{3.0};
    double curvature_feedforward_time_constant_sec{0.20};
    double maneuver_lookahead_time_constant_sec{0.20};
    bool exit_unwind_enabled{false};
    double exit_unwind_far_preview_boost{0.20};
    double exit_unwind_curvature_drop_threshold{0.02};
    bool rotation_gate_enabled{false};
    double rotation_gate_min_curvature{0.06};
    double rotation_gate_min_steering_angle{0.18};
    double rotation_gate_yaw_rate_error_threshold{0.30};
    double rotation_gate_yaw_rate_error_release_ratio{0.60};
    double rotation_gate_max_lateral_error{1.00};
    double rotation_gate_min_duration_sec{0.05};
    double rotation_gate_max_duration_sec{0.20};
    double rotation_gate_cooldown_sec{0.50};
    double rotation_gate_countersteer_gain{0.35};
    double rotation_gate_max_countersteer_rad{0.12};
    bool rotation_prediction_enabled{false};
    double rotation_prediction_lead_time_sec{0.15};
    double rotation_prediction_yaw_acceleration_filter_time_constant_sec{0.08};
    double rotation_prediction_max_yaw_acceleration_radps2{6.0};
    double rotation_prediction_entry_threshold_radps{0.18};
    double rotation_prediction_min_yaw_acceleration_radps2{0.30};
    double rotation_prediction_slip_angle_threshold_rad{0.04};
    double rotation_prediction_slip_yaw_rate_gain{2.0};
    bool steering_tracking_speed_gate_enabled{false};
    double steering_tracking_entry_error_rad{0.18};
    double steering_tracking_release_error_rad{0.08};
    double steering_tracking_entry_duration_sec{0.10};
    double steering_tracking_release_duration_sec{0.10};
    double steering_tracking_deceleration_mps2{1.0};
    bool wall_edge_tracking_speed_enabled{true};
    double wall_edge_lateral_velocity_preview_sec{0.20};
    double wall_edge_minimum_speed_mps{0.50};
    double wall_edge_tracking_deceleration_mps2{3.0};
    double wall_edge_tracking_release_reserve_ratio{0.75};
    double wall_edge_tracking_release_duration_sec{0.15};
  };

  void onTimer();
  bool subscribeMessageAvailable() const;
  bool inputSamplesFresh(double now_sec) const;
  void resetExperimentState();
  double estimateCurvature(std::size_t nearest_index) const;
  double estimateSignedCurvature(std::size_t nearest_index) const;
  double estimatePreviewCurvature(std::size_t nearest_index,
                                  double preview_distance) const;
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(const std::vector<rclcpp::Parameter> &parameters);
  void onSetEnabled(const std_srvs::srv::SetBool::Request::SharedPtr request,
                    std_srvs::srv::SetBool::Response::SharedPtr response);
  void onResetState(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    std_srvs::srv::Trigger::Response::SharedPtr response);
  void recordRuntimeTiming(const RuntimeTimingSample &sample,
                           double callback_end_steady_sec);
  void selectSplitTrajectory(double now_sec);
  void restoreDirectMetadata();
  bool holdingDirectTrajectory() const;

  const double wheel_base_;
  const bool use_external_target_vel_;
  const bool ga_experiment_mode_;
  const bool curvature_feedforward_maneuver_only_;
  const bool maneuver_lookahead_uses_measured_speed_;
  const bool gear_relative_reverse_command_enabled_;
  const bool use_atomic_direct_trajectory_command_;
  const bool hold_last_direct_trajectory_enabled_;
  const bool split_trajectory_inputs_enabled_;
  const double odometry_timeout_sec_;
  const double trajectory_timeout_sec_;
  const double overtake_mode_timeout_sec_;
  const double path_source_timeout_sec_;
  const bool diagnostic_trace_enabled_;
  const bool diagnostic_trace_maneuver_only_;
  const double diagnostic_trace_period_sec_;
  const bool runtime_timing_metrics_enabled_;
  const double runtime_timing_report_period_sec_;
  const double runtime_timing_deadline_sec_;

  mutable std::mutex state_mutex_;
  TuningParameters tuning_;
  bool controller_enabled_;
  bool recovery_paused_{false};
  bool recovery_waiting_reference_{false};
  double recovery_reference_not_before_sec_{-1.0};
  std::string ga_run_id_;
  std::string ga_candidate_id_;
  std::string ga_parameter_hash_;
  std::uint64_t reset_epoch_{0};
  double smoothed_curvature_{0.0};
  bool curvature_initialized_{false};
  double smoothed_feedforward_curvature_{0.0};
  bool feedforward_curvature_initialized_{false};
  double smoothed_maneuver_lookahead_distance_m_{0.0};
  bool maneuver_lookahead_distance_initialized_{false};
  double previous_steering_{0.0};
  bool steering_limiter_initialized_{false};
  // Nominal PP history remains separate from the final countersteered command
  // so recovery cannot walk the next nominal request across zero.
  double last_bounded_steering_rad_{0.0};
  double last_control_publish_sec_{-1.0};
  double last_diagnostic_trace_sec_{-1.0e9};
  bool runtime_timing_callback_started_{false};
  std::chrono::steady_clock::time_point last_runtime_timing_callback_start_;
  double last_runtime_timing_report_steady_sec_{-1.0};
  std::vector<RuntimeTimingSample> runtime_timing_samples_;
  SteeringTrackingSpeedGateState steering_tracking_speed_gate_state_;
  WallEdgeTrackingSpeedState wall_edge_tracking_speed_state_;
  std::string overtake_mode_{"UNKNOWN"};
  rclcpp::Time overtake_mode_received_at_;
  double odometry_received_sec_{-1.0};
  double trajectory_received_sec_{-1.0};
  double baseline_trajectory_received_sec_{-1.0};
  double maneuver_trajectory_received_sec_{-1.0};
  double path_source_received_sec_{-1.0};
  std::uint64_t direct_trajectory_generation_{0U};
  std::uint64_t direct_owner_commit_id_{0U};
  std::uint64_t direct_geometry_revision_{0U};
  double direct_valid_until_sec_{-1.0};
  double direct_remaining_arc_m_{0.0};
  bool direct_corridor_clearance_profile_{false};
  int direct_corridor_side_{0};
  bool direct_wall_edge_profile_{false};
  int direct_wall_side_{0};
  double direct_wall_clearance_reserve_m_{0.0};
  double direct_opponent_clearance_reserve_m_{0.0};
  double direct_corridor_clearance_reserve_m_{0.0};
  double direct_desired_control_reserve_m_{0.0};

  Trajectory::SharedPtr trajectory_;
  Trajectory::SharedPtr direct_trajectory_;
  StateLatticeDirectTrajectory::SharedPtr direct_command_;
  double direct_received_sec_{-1.0};
  Trajectory::SharedPtr baseline_trajectory_;
  Trajectory::SharedPtr maneuver_trajectory_;
  std::string path_source_{"UNKNOWN"};
  std::string selected_trajectory_source_{"legacy"};
  Odometry::SharedPtr odometry_;
  SteeringReport::SharedPtr steering_status_;
  GearReport::SharedPtr gear_status_;
  PoseWithCovarianceStamped::SharedPtr previous_gnss_pose_;
  double gnss_velocity_x_{0.0};
  double gnss_velocity_y_{0.0};
  bool gnss_velocity_valid_{false};

  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_baseline_trajectory_;
  rclcpp::Subscription<StateLatticeDirectTrajectory>::SharedPtr
      sub_direct_trajectory_command_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_status_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_status_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_overtake_mode_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_path_source_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_near_lookahead_point_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_enabled_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_state_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_recovery_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_HPP_
