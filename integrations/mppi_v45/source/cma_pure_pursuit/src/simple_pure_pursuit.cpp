#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include "simple_pure_pursuit/curvature_feedforward.hpp"
#include "simple_pure_pursuit/curvature_feedforward_mode.hpp"
#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/direct_driving_mode.hpp"
#include "simple_pure_pursuit/gear_relative_command.hpp"
#include "simple_pure_pursuit/input_freshness.hpp"
#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/lookahead_speed_basis.hpp"
#include "simple_pure_pursuit/path_source_mapping.hpp"
#include "simple_pure_pursuit/pure_pursuit_core.hpp"
#include "simple_pure_pursuit/rotation_recovery.hpp"
#include "simple_pure_pursuit/steering_actuator_model.hpp"
#include "simple_pure_pursuit/travel_direction.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tf2/utils.h>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace simple_pure_pursuit {

namespace {

AckermannControlCommand zeroCommand(const rclcpp::Time &stamp) {
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.lateral.stamp = stamp;
  return cmd;
}

bool finiteInRange(const double value, const double minimum,
                   const double maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

std::string jsonEscape(const std::string &value) {
  std::ostringstream out;
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      out << '\\';
    }
    out << character;
  }
  return out.str();
}

double threadCpuSeconds() {
  timespec timestamp{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timestamp) != 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(timestamp.tv_sec) +
         static_cast<double>(timestamp.tv_nsec) * 1.0e-9;
}

struct TimingDistribution {
  double p50_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double maximum_ms{0.0};
};

TimingDistribution summarizeTiming(std::vector<double> values) {
  values.erase(
      std::remove_if(values.begin(), values.end(),
                     [](const double value) { return !std::isfinite(value); }),
      values.end());
  if (values.empty()) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return TimingDistribution{nan, nan, nan, nan};
  }
  std::sort(values.begin(), values.end());
  const auto percentile = [&values](const double quantile) {
    const double position = quantile * static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] + fraction * (values[upper] - values[lower]);
  };
  return TimingDistribution{percentile(0.50), percentile(0.95),
                            percentile(0.99), values.back()};
}

} // namespace

SimplePurePursuit::SimplePurePursuit()
    : Node("simple_pure_pursuit"),
      wheel_base_(declare_parameter<double>("wheel_base", 2.14)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      ga_experiment_mode_(declare_parameter<bool>("ga_experiment_mode", false)),
      curvature_feedforward_maneuver_only_(declare_parameter<bool>(
          "curvature_feedforward_maneuver_only", false)),
      maneuver_lookahead_uses_measured_speed_(declare_parameter<bool>(
          "maneuver_lookahead_uses_measured_speed", false)),
      gear_relative_reverse_command_enabled_(declare_parameter<bool>(
          "gear_relative_reverse_command_enabled", false)),
      use_atomic_direct_trajectory_command_(declare_parameter<bool>(
          "use_atomic_direct_trajectory_command", false)),
      hold_last_direct_trajectory_enabled_(declare_parameter<bool>(
          "hold_last_direct_trajectory_enabled", false)),
      split_trajectory_inputs_enabled_(
          declare_parameter<bool>("split_trajectory_inputs_enabled", false)),
      odometry_timeout_sec_(
          declare_parameter<double>("odometry_timeout_sec", 0.5)),
      trajectory_timeout_sec_(
          declare_parameter<double>("trajectory_timeout_sec", 0.5)),
      overtake_mode_timeout_sec_(
          declare_parameter<double>("overtake_mode_timeout_sec", 0.5)),
      path_source_timeout_sec_(
          declare_parameter<double>("path_source_timeout_sec", 0.5)),
      diagnostic_trace_enabled_(
          declare_parameter<bool>("diagnostic_trace_enabled", false)),
      diagnostic_trace_maneuver_only_(
          declare_parameter<bool>("diagnostic_trace_maneuver_only", true)),
      diagnostic_trace_period_sec_(
          declare_parameter<double>("diagnostic_trace_period_sec", 0.02)),
      runtime_timing_metrics_enabled_(
          declare_parameter<bool>("runtime_timing_metrics_enabled", false)),
      runtime_timing_report_period_sec_(
          declare_parameter<double>("runtime_timing_report_period_sec", 1.0)),
      runtime_timing_deadline_sec_(declare_parameter<double>(
          "runtime_timing_deadline_sec", 0.033333333)),
      controller_enabled_(!ga_experiment_mode_) {
  rotation_prediction_pub_=create_publisher<RotationPredictionMessage>(
      "/mppi/internal/cma_rotation_prediction_state", rclcpp::QoS(8));
  if (!std::isfinite(diagnostic_trace_period_sec_) ||
      diagnostic_trace_period_sec_ <= 0.0) {
    throw std::invalid_argument(
        "diagnostic_trace_period_sec must be finite and positive");
  }
  if (!std::isfinite(runtime_timing_report_period_sec_) ||
      runtime_timing_report_period_sec_ <= 0.0) {
    throw std::invalid_argument(
        "runtime_timing_report_period_sec must be finite and positive");
  }
  if (!std::isfinite(runtime_timing_deadline_sec_) ||
      runtime_timing_deadline_sec_ <= 0.0) {
    throw std::invalid_argument(
        "runtime_timing_deadline_sec must be finite and positive");
  }
  if (!std::isfinite(path_source_timeout_sec_) ||
      path_source_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
        "path_source_timeout_sec must be finite and positive");
  }
  if (use_atomic_direct_trajectory_command_ &&
      split_trajectory_inputs_enabled_) {
    throw std::invalid_argument("atomic direct trajectory and split trajectory "
                                "inputs are mutually exclusive");
  }
  if (runtime_timing_metrics_enabled_) {
    runtime_timing_samples_.reserve(256U);
  }
  tuning_.lookahead_gain = declare_parameter<double>("lookahead_gain", 0.5);
  tuning_.lookahead_min_distance =
      declare_parameter<double>("lookahead_min_distance", 3.5);
  tuning_.actual_lookahead_distance_blend =
      declare_parameter<double>("actual_lookahead_distance_blend", 0.0);
  tuning_.dual_preview_near_ratio =
      declare_parameter<double>("dual_preview_near_ratio", 0.5);
  tuning_.dual_preview_blend =
      declare_parameter<double>("dual_preview_blend", 0.0);
  tuning_.steering_tire_angle_gain =
      declare_parameter<double>("steering_tire_angle_gain", 1.5);
  tuning_.curvature_lookahead_min_distance =
      declare_parameter<double>("curvature_lookahead_min_distance", 2.0);
  tuning_.curvature_lookahead_sensitivity =
      declare_parameter<double>("curvature_lookahead_sensitivity", 8.0);
  tuning_.curvature_lookahead_smoothing_alpha =
      declare_parameter<double>("curvature_lookahead_smoothing_alpha", 0.35);
  tuning_.max_lateral_acceleration =
      declare_parameter<double>("max_lateral_acceleration", 6.0);
  tuning_.minimum_corner_speed =
      declare_parameter<double>("minimum_corner_speed", 5.0);
  tuning_.corner_speed_retention =
      declare_parameter<double>("corner_speed_retention", 0.0);
  tuning_.lateral_error_speed_gate_enabled =
      declare_parameter<bool>("lateral_error_speed_gate_enabled", true);
  tuning_.corner_speed_retention_lateral_error_soft = declare_parameter<double>(
      "corner_speed_retention_lateral_error_soft", 0.5);
  tuning_.corner_speed_retention_lateral_error_hard = declare_parameter<double>(
      "corner_speed_retention_lateral_error_hard", 1.0);
  tuning_.curvature_speed_preview_distance =
      declare_parameter<double>("curvature_speed_preview_distance", 12.0);
  tuning_.speed_proportional_gain =
      declare_parameter<double>("speed_proportional_gain", 1.0);
  tuning_.longitudinal_acceleration_limit =
      declare_parameter<double>("longitudinal_acceleration_limit", 1.0);
  tuning_.longitudinal_deceleration_limit =
      declare_parameter<double>("longitudinal_deceleration_limit", 2.0);
  tuning_.safe_stop_deceleration =
      declare_parameter<double>("safe_stop_deceleration", 3.0);
  tuning_.steering_command_passthrough_enabled =
      declare_parameter<bool>("steering_command_passthrough_enabled", false);
  tuning_.steering_command_to_tire_angle_ratio =
      declare_parameter<double>("steering_command_to_tire_angle_ratio", 0.60);
  tuning_.hard_steering_angle_limit_rad = declare_parameter<double>(
      "hard_steering_angle_limit_rad", 0.3665191429188092);
  tuning_.maximum_tire_steering_angle_rad = declare_parameter<double>(
      "maximum_tire_steering_angle_rad", 0.3141592653589793);
  tuning_.hard_steering_rate_limit_radps =
      declare_parameter<double>("hard_steering_rate_limit_radps", 4.0);
  tuning_.physical_tire_steering_rate_radps =
      declare_parameter<double>("physical_tire_steering_rate_radps", 0.0);
  tuning_.steering_demand_acceleration_hold_enabled = declare_parameter<bool>(
      "steering_demand_acceleration_hold_enabled", false);
  tuning_.steering_acceleration_hold_minimum_speed_mps =
      declare_parameter<double>("steering_acceleration_hold_minimum_speed_mps",
                                3.0);
  tuning_.steering_acceleration_hold_minimum_tire_angle_rad =
      declare_parameter<double>(
          "steering_acceleration_hold_minimum_tire_angle_rad", 0.15);
  tuning_.steering_acceleration_hold_tracking_error_rad =
      declare_parameter<double>("steering_acceleration_hold_tracking_error_rad",
                                0.08);
  tuning_.steering_acceleration_hold_maximum_acceleration_mps2 =
      declare_parameter<double>(
          "steering_acceleration_hold_maximum_acceleration_mps2", 0.0);
  tuning_.external_target_vel =
      declare_parameter<double>("external_target_vel", 9.722222222222);
  tuning_.continuous_preview_interpolation_enabled = declare_parameter<bool>(
      "continuous_preview_interpolation_enabled", false);
  tuning_.delay_compensation_enabled =
      declare_parameter<bool>("delay_compensation_enabled", false);
  tuning_.pp_control_delay_sec =
      declare_parameter<double>("pp_control_delay_sec", 0.2);
  tuning_.pp_prediction_dt_sec =
      declare_parameter<double>("pp_prediction_dt_sec", 0.02);
  tuning_.steering_time_constant_sec =
      declare_parameter<double>("steering_time_constant_sec", 0.30);
  tuning_.steering_status_timeout_sec =
      declare_parameter<double>("steering_status_timeout_sec", 0.20);
  tuning_.min_velocity_for_delay_compensation_mps = declare_parameter<double>(
      "min_velocity_for_delay_compensation_mps", 0.20);
  tuning_.curvature_feedforward_enabled =
      declare_parameter<bool>("curvature_feedforward_enabled", false);
  tuning_.curvature_feedforward_gain =
      declare_parameter<double>("curvature_feedforward_gain", 0.20);
  tuning_.curvature_feedforward_preview_distance_m = declare_parameter<double>(
      "curvature_feedforward_preview_distance_m", 3.0);
  tuning_.curvature_feedforward_time_constant_sec = declare_parameter<double>(
      "curvature_feedforward_time_constant_sec", 0.20);
  tuning_.maneuver_lookahead_time_constant_sec =
      declare_parameter<double>("maneuver_lookahead_time_constant_sec", 0.20);
  tuning_.exit_unwind_enabled =
      declare_parameter<bool>("exit_unwind_enabled", false);
  tuning_.exit_unwind_far_preview_boost =
      declare_parameter<double>("exit_unwind_far_preview_boost", 0.20);
  tuning_.exit_unwind_curvature_drop_threshold =
      declare_parameter<double>("exit_unwind_curvature_drop_threshold", 0.02);
  tuning_.rotation_gate_enabled =
      declare_parameter<bool>("rotation_gate_enabled", false);
  tuning_.rotation_gate_min_curvature =
      declare_parameter<double>("rotation_gate_min_curvature", 0.06);
  tuning_.rotation_gate_min_steering_angle =
      declare_parameter<double>("rotation_gate_min_steering_angle", 0.18);
  tuning_.rotation_gate_yaw_rate_error_threshold =
      declare_parameter<double>("rotation_gate_yaw_rate_error_threshold", 0.30);
  tuning_.rotation_gate_yaw_rate_error_release_ratio =
      declare_parameter<double>("rotation_gate_yaw_rate_error_release_ratio",
                                0.60);
  tuning_.rotation_gate_max_lateral_error =
      declare_parameter<double>("rotation_gate_max_lateral_error", 1.00);
  tuning_.rotation_gate_min_duration_sec =
      declare_parameter<double>("rotation_gate_min_duration_sec", 0.05);
  tuning_.rotation_gate_max_duration_sec =
      declare_parameter<double>("rotation_gate_max_duration_sec", 0.20);
  tuning_.rotation_gate_cooldown_sec =
      declare_parameter<double>("rotation_gate_cooldown_sec", 0.50);
  tuning_.rotation_gate_countersteer_gain =
      declare_parameter<double>("rotation_gate_countersteer_gain", 0.35);
  tuning_.rotation_gate_max_countersteer_rad =
      declare_parameter<double>("rotation_gate_max_countersteer_rad", 0.12);
  tuning_.rotation_prediction_enabled =
      declare_parameter<bool>("rotation_prediction_enabled", false);
  tuning_.rotation_prediction_lead_time_sec =
      declare_parameter<double>("rotation_prediction_lead_time_sec", 0.15);
  tuning_.rotation_prediction_yaw_acceleration_filter_time_constant_sec =
      declare_parameter<double>(
          "rotation_prediction_yaw_acceleration_filter_time_constant_sec",
          0.08);
  tuning_.rotation_prediction_max_yaw_acceleration_radps2 =
      declare_parameter<double>(
          "rotation_prediction_max_yaw_acceleration_radps2", 6.0);
  tuning_.rotation_prediction_entry_threshold_radps = declare_parameter<double>(
      "rotation_prediction_entry_threshold_radps", 0.18);
  tuning_.rotation_prediction_min_yaw_acceleration_radps2 =
      declare_parameter<double>(
          "rotation_prediction_min_yaw_acceleration_radps2", 0.30);
  tuning_.rotation_prediction_slip_angle_threshold_rad =
      declare_parameter<double>("rotation_prediction_slip_angle_threshold_rad",
                                0.04);
  tuning_.rotation_prediction_slip_yaw_rate_gain =
      declare_parameter<double>("rotation_prediction_slip_yaw_rate_gain", 2.0);
  tuning_.steering_tracking_speed_gate_enabled =
      declare_parameter<bool>("steering_tracking_speed_gate_enabled", false);
  tuning_.steering_tracking_entry_error_rad =
      declare_parameter<double>("steering_tracking_entry_error_rad", 0.18);
  tuning_.steering_tracking_release_error_rad =
      declare_parameter<double>("steering_tracking_release_error_rad", 0.08);
  tuning_.steering_tracking_entry_duration_sec =
      declare_parameter<double>("steering_tracking_entry_duration_sec", 0.10);
  tuning_.steering_tracking_release_duration_sec =
      declare_parameter<double>("steering_tracking_release_duration_sec", 0.10);
  tuning_.steering_tracking_deceleration_mps2 =
      declare_parameter<double>("steering_tracking_deceleration_mps2", 1.0);
  tuning_.wall_edge_tracking_speed_enabled =
      declare_parameter<bool>("wall_edge_tracking_speed_enabled", true);
  tuning_.wall_edge_lateral_velocity_preview_sec =
      declare_parameter<double>("wall_edge_lateral_velocity_preview_sec", 0.20);
  tuning_.wall_edge_minimum_speed_mps =
      declare_parameter<double>("wall_edge_minimum_speed_mps", 0.50);
  tuning_.wall_edge_tracking_deceleration_mps2 =
      declare_parameter<double>("wall_edge_tracking_deceleration_mps2", 3.0);
  tuning_.wall_edge_tracking_release_reserve_ratio = declare_parameter<double>(
      "wall_edge_tracking_release_reserve_ratio", 0.75);
  tuning_.wall_edge_tracking_release_duration_sec = declare_parameter<double>(
      "wall_edge_tracking_release_duration_sec", 0.15);
  ga_run_id_ = declare_parameter<std::string>("ga_run_id", "");
  ga_candidate_id_ = declare_parameter<std::string>("ga_candidate_id", "");
  ga_parameter_hash_ = declare_parameter<std::string>("ga_parameter_hash", "");

  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ =
      create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ =
      create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_near_lookahead_point_ =
      create_publisher<PointStamped>("/control/debug/near_lookahead_point", 1);
  pub_debug_ =
      create_publisher<std_msgs::msg::String>("/pure_pursuit/debug", 10);

  const auto best_effort =
      rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ =
      create_subscription<Odometry>("input/kinematics", best_effort,
                                    [this](const Odometry::SharedPtr message) {
                                      odometry_ = message;
                                      odometry_received_sec_ =
                                          get_clock()->now().seconds();
                                    });
  const auto update_mode = [this](const std::string &mode) {
    if (mode != overtake_mode_) {
      if (mode == "SAFE_STOP") {
        RCLCPP_WARN(get_logger(), "[PURE_PURSUIT_MODE] previous=%s current=%s",
                    overtake_mode_.c_str(), mode.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "[PURE_PURSUIT_MODE] previous=%s current=%s",
                    overtake_mode_.c_str(), mode.c_str());
      }
    }
    overtake_mode_ = mode;
    overtake_mode_received_at_ = get_clock()->now();
  };
  if (use_atomic_direct_trajectory_command_) {
    const auto direct_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    sub_direct_trajectory_command_ =
        create_subscription<StateLatticeDirectTrajectory>(
            "input/direct_trajectory_command", direct_qos,
            [this, update_mode](
                const StateLatticeDirectTrajectory::SharedPtr message) {
              std::lock_guard<std::mutex> lock(state_mutex_);
              if (recovery_paused_ ||
                  rclcpp::Time(message->header.stamp).seconds() <= recovery_reference_not_before_sec_) {
                return;
              }
              if (!directTrajectoryGenerationAccepted(
                      trajectory_ != nullptr, direct_trajectory_generation_,
                      message->generation)) {
                RCLCPP_WARN(
                    get_logger(),
                    "[PURE_PURSUIT_STALE_TRAJECTORY] current=%llu "
                    "incoming=%llu",
                    static_cast<unsigned long long>(
                        direct_trajectory_generation_),
                    static_cast<unsigned long long>(message->generation));
                return;
              }
              trajectory_ = std::make_shared<Trajectory>(message->trajectory);
              trajectory_received_sec_ = get_clock()->now().seconds();
              direct_trajectory_ = trajectory_;
              direct_command_ = message;
              direct_received_sec_ = trajectory_received_sec_;
              direct_trajectory_generation_ = message->generation;
              if (!message->trajectory.points.empty()) recovery_waiting_reference_ = false;
              direct_owner_commit_id_ = message->owner_commit_id;
              direct_geometry_revision_ = message->geometry_revision;
              direct_valid_until_sec_ = message->valid_until_sec;
              direct_remaining_arc_m_ = message->remaining_arc_m;
              direct_corridor_clearance_profile_ =
                  message->corridor_clearance_profile;
              direct_corridor_side_ = static_cast<int>(message->corridor_side);
              direct_wall_edge_profile_ = message->wall_edge_profile;
              direct_wall_side_ = static_cast<int>(message->wall_side);
              direct_wall_clearance_reserve_m_ =
                  message->wall_clearance_reserve_m;
              direct_opponent_clearance_reserve_m_ =
                  message->opponent_clearance_reserve_m;
              direct_corridor_clearance_reserve_m_ =
                  message->corridor_clearance_reserve_m;
              direct_desired_control_reserve_m_ =
                  message->desired_control_reserve_m;
              update_mode(directTrackingMode(message->mode, direct_corridor_side_));
            });
  } else {
    if (split_trajectory_inputs_enabled_) {
      sub_baseline_trajectory_ = create_subscription<Trajectory>(
          "input/baseline_trajectory", best_effort,
          [this](const Trajectory::SharedPtr message) {
            baseline_trajectory_ = message;
            baseline_trajectory_received_sec_ = get_clock()->now().seconds();
          });
      sub_trajectory_ = create_subscription<Trajectory>(
          "input/trajectory", best_effort,
          [this](const Trajectory::SharedPtr message) {
            maneuver_trajectory_ = message;
            maneuver_trajectory_received_sec_ = get_clock()->now().seconds();
          });
      sub_path_source_ = create_subscription<std_msgs::msg::String>(
          "input/path_source", best_effort,
          [this](const std_msgs::msg::String::SharedPtr message) {
            path_source_ = message->data;
            path_source_received_sec_ = get_clock()->now().seconds();
          });
    } else {
      sub_trajectory_ = create_subscription<Trajectory>(
          "input/trajectory", best_effort,
          [this](const Trajectory::SharedPtr message) {
            trajectory_ = message;
            trajectory_received_sec_ = get_clock()->now().seconds();
          });
    }
    sub_overtake_mode_ = create_subscription<std_msgs::msg::String>(
        "input/overtake_mode", best_effort,
        [update_mode](const std_msgs::msg::String::SharedPtr message) {
          update_mode(message->data);
        });
  }
  sub_steering_status_ = create_subscription<SteeringReport>(
      "input/steering_status", best_effort,
      [this](const SteeringReport::SharedPtr message) {
        steering_status_ = message;
      });
  sub_gear_status_ = create_subscription<GearReport>(
      "input/gear_status", best_effort,
      [this](const GearReport::SharedPtr message) { gear_status_ = message; });
  sub_gnss_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "/sensing/gnss/pose_with_covariance", best_effort,
      [this](const PoseWithCovarianceStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (previous_gnss_pose_) {
          const rclcpp::Time current_stamp(message->header.stamp);
          const rclcpp::Time previous_stamp(previous_gnss_pose_->header.stamp);
          const double dt = (current_stamp - previous_stamp).seconds();
          if (dt >= 0.02 && dt <= 1.0) {
            const double vx = (message->pose.pose.position.x -
                               previous_gnss_pose_->pose.pose.position.x) /
                              dt;
            const double vy = (message->pose.pose.position.y -
                               previous_gnss_pose_->pose.pose.position.y) /
                              dt;
            const double speed = std::hypot(vx, vy);
            if (std::isfinite(speed) && speed <= 20.0) {
              constexpr double alpha = 0.25;
              gnss_velocity_x_ =
                  gnss_velocity_valid_
                      ? alpha * vx + (1.0 - alpha) * gnss_velocity_x_
                      : vx;
              gnss_velocity_y_ =
                  gnss_velocity_valid_
                      ? alpha * vy + (1.0 - alpha) * gnss_velocity_y_
                      : vy;
              gnss_velocity_valid_ = true;
            }
          }
        }
        previous_gnss_pose_ = message;
      });

  const bool recovery_service_enabled = declare_parameter<bool>("recovery_service_enabled", false);
  parameter_callback_ = add_on_set_parameters_callback(std::bind(
      &SimplePurePursuit::onSetParameters, this, std::placeholders::_1));
  srv_set_enabled_ = create_service<std_srvs::srv::SetBool>(
      "~/ga/set_enabled",
      std::bind(&SimplePurePursuit::onSetEnabled, this, std::placeholders::_1,
                std::placeholders::_2));
  srv_reset_state_ = create_service<std_srvs::srv::Trigger>(
      "~/ga/reset_state",
      std::bind(&SimplePurePursuit::onResetState, this, std::placeholders::_1,
                std::placeholders::_2));

  if (recovery_service_enabled) {
    srv_recovery_ = create_service<std_srvs::srv::SetBool>(
        "~/set_recovery_active",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (!use_atomic_direct_trajectory_command_) {
            response->success = false;
            response->message = "recovery requires atomic trajectory input";
            return;
          }
          recovery_paused_ = request->data;
          recovery_waiting_reference_ = !request->data;
          resetExperimentState();
          trajectory_.reset();
          trajectory_received_sec_ = -1.0;
          recovery_reference_not_before_sec_ = get_clock()->now().seconds();
          response->success = true;
          response->message = recovery_paused_ ? "recovery owns control" : "waiting for fresh trajectory";
        });
  }

  using namespace std::chrono_literals;
  timer_ =
      create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult SimplePurePursuit::onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  if (!ga_experiment_mode_) {
    result.reason = "runtime tuning requires ga_experiment_mode=true";
    return result;
  }
  if (controller_enabled_) {
    result.reason = "disable the controller before changing a candidate";
    return result;
  }

  TuningParameters candidate = tuning_;
  std::string run_id = ga_run_id_;
  std::string candidate_id = ga_candidate_id_;
  std::string parameter_hash = ga_parameter_hash_;
  try {
    for (const auto &parameter : parameters) {
      const auto &name = parameter.get_name();
      if (name == "lookahead_gain") {
        candidate.lookahead_gain = parameter.as_double();
      } else if (name == "lookahead_min_distance") {
        candidate.lookahead_min_distance = parameter.as_double();
      } else if (name == "actual_lookahead_distance_blend") {
        candidate.actual_lookahead_distance_blend = parameter.as_double();
      } else if (name == "dual_preview_near_ratio") {
        candidate.dual_preview_near_ratio = parameter.as_double();
      } else if (name == "dual_preview_blend") {
        candidate.dual_preview_blend = parameter.as_double();
      } else if (name == "steering_tire_angle_gain") {
        candidate.steering_tire_angle_gain = parameter.as_double();
      } else if (name == "curvature_lookahead_min_distance") {
        candidate.curvature_lookahead_min_distance = parameter.as_double();
      } else if (name == "curvature_lookahead_sensitivity") {
        candidate.curvature_lookahead_sensitivity = parameter.as_double();
      } else if (name == "curvature_lookahead_smoothing_alpha") {
        candidate.curvature_lookahead_smoothing_alpha = parameter.as_double();
      } else if (name == "max_lateral_acceleration") {
        candidate.max_lateral_acceleration = parameter.as_double();
      } else if (name == "minimum_corner_speed") {
        candidate.minimum_corner_speed = parameter.as_double();
      } else if (name == "corner_speed_retention") {
        candidate.corner_speed_retention = parameter.as_double();
      } else if (name == "lateral_error_speed_gate_enabled") {
        candidate.lateral_error_speed_gate_enabled = parameter.as_bool();
      } else if (name == "corner_speed_retention_lateral_error_soft") {
        candidate.corner_speed_retention_lateral_error_soft =
            parameter.as_double();
      } else if (name == "corner_speed_retention_lateral_error_hard") {
        candidate.corner_speed_retention_lateral_error_hard =
            parameter.as_double();
      } else if (name == "curvature_speed_preview_distance") {
        candidate.curvature_speed_preview_distance = parameter.as_double();
      } else if (name == "speed_proportional_gain") {
        candidate.speed_proportional_gain = parameter.as_double();
      } else if (name == "longitudinal_acceleration_limit") {
        candidate.longitudinal_acceleration_limit = parameter.as_double();
      } else if (name == "longitudinal_deceleration_limit") {
        candidate.longitudinal_deceleration_limit = parameter.as_double();
      } else if (name == "safe_stop_deceleration") {
        candidate.safe_stop_deceleration = parameter.as_double();
      } else if (name == "steering_command_passthrough_enabled") {
        candidate.steering_command_passthrough_enabled = parameter.as_bool();
      } else if (name == "steering_command_to_tire_angle_ratio") {
        candidate.steering_command_to_tire_angle_ratio = parameter.as_double();
      } else if (name == "hard_steering_angle_limit_rad") {
        candidate.hard_steering_angle_limit_rad = parameter.as_double();
      } else if (name == "maximum_tire_steering_angle_rad") {
        candidate.maximum_tire_steering_angle_rad = parameter.as_double();
      } else if (name == "hard_steering_rate_limit_radps") {
        candidate.hard_steering_rate_limit_radps = parameter.as_double();
      } else if (name == "physical_tire_steering_rate_radps") {
        candidate.physical_tire_steering_rate_radps = parameter.as_double();
      } else if (name == "steering_demand_acceleration_hold_enabled") {
        candidate.steering_demand_acceleration_hold_enabled =
            parameter.as_bool();
      } else if (name == "steering_acceleration_hold_minimum_speed_mps") {
        candidate.steering_acceleration_hold_minimum_speed_mps =
            parameter.as_double();
      } else if (name == "steering_acceleration_hold_minimum_tire_angle_rad") {
        candidate.steering_acceleration_hold_minimum_tire_angle_rad =
            parameter.as_double();
      } else if (name == "steering_acceleration_hold_tracking_error_rad") {
        candidate.steering_acceleration_hold_tracking_error_rad =
            parameter.as_double();
      } else if (name == "steering_acceleration_hold_maximum_acceleration_mps2") {
        candidate.steering_acceleration_hold_maximum_acceleration_mps2 =
            parameter.as_double();
      } else if (name == "external_target_vel") {
        candidate.external_target_vel = parameter.as_double();
      } else if (name == "continuous_preview_interpolation_enabled") {
        candidate.continuous_preview_interpolation_enabled =
            parameter.as_bool();
      } else if (name == "delay_compensation_enabled") {
        candidate.delay_compensation_enabled = parameter.as_bool();
      } else if (name == "pp_control_delay_sec") {
        candidate.pp_control_delay_sec = parameter.as_double();
      } else if (name == "pp_prediction_dt_sec") {
        candidate.pp_prediction_dt_sec = parameter.as_double();
      } else if (name == "steering_time_constant_sec") {
        candidate.steering_time_constant_sec = parameter.as_double();
      } else if (name == "steering_status_timeout_sec") {
        candidate.steering_status_timeout_sec = parameter.as_double();
      } else if (name == "min_velocity_for_delay_compensation_mps") {
        candidate.min_velocity_for_delay_compensation_mps =
            parameter.as_double();
      } else if (name == "curvature_feedforward_enabled") {
        candidate.curvature_feedforward_enabled = parameter.as_bool();
      } else if (name == "curvature_feedforward_gain") {
        candidate.curvature_feedforward_gain = parameter.as_double();
      } else if (name == "curvature_feedforward_preview_distance_m") {
        candidate.curvature_feedforward_preview_distance_m =
            parameter.as_double();
      } else if (name == "curvature_feedforward_time_constant_sec") {
        candidate.curvature_feedforward_time_constant_sec =
            parameter.as_double();
      } else if (name == "maneuver_lookahead_time_constant_sec") {
        candidate.maneuver_lookahead_time_constant_sec = parameter.as_double();
      } else if (name == "exit_unwind_enabled") {
        candidate.exit_unwind_enabled = parameter.as_bool();
      } else if (name == "exit_unwind_far_preview_boost") {
        candidate.exit_unwind_far_preview_boost = parameter.as_double();
      } else if (name == "exit_unwind_curvature_drop_threshold") {
        candidate.exit_unwind_curvature_drop_threshold = parameter.as_double();
      } else if (name == "rotation_gate_enabled") {
        candidate.rotation_gate_enabled = parameter.as_bool();
      } else if (name == "rotation_gate_min_curvature") {
        candidate.rotation_gate_min_curvature = parameter.as_double();
      } else if (name == "rotation_gate_min_steering_angle") {
        candidate.rotation_gate_min_steering_angle = parameter.as_double();
      } else if (name == "rotation_gate_yaw_rate_error_threshold") {
        candidate.rotation_gate_yaw_rate_error_threshold =
            parameter.as_double();
      } else if (name == "rotation_gate_yaw_rate_error_release_ratio") {
        candidate.rotation_gate_yaw_rate_error_release_ratio =
            parameter.as_double();
      } else if (name == "rotation_gate_max_lateral_error") {
        candidate.rotation_gate_max_lateral_error = parameter.as_double();
      } else if (name == "rotation_gate_min_duration_sec") {
        candidate.rotation_gate_min_duration_sec = parameter.as_double();
      } else if (name == "rotation_gate_max_duration_sec") {
        candidate.rotation_gate_max_duration_sec = parameter.as_double();
      } else if (name == "rotation_gate_cooldown_sec") {
        candidate.rotation_gate_cooldown_sec = parameter.as_double();
      } else if (name == "rotation_gate_countersteer_gain") {
        candidate.rotation_gate_countersteer_gain = parameter.as_double();
      } else if (name == "rotation_gate_max_countersteer_rad") {
        candidate.rotation_gate_max_countersteer_rad = parameter.as_double();
      } else if (name == "rotation_prediction_enabled") {
        candidate.rotation_prediction_enabled = parameter.as_bool();
      } else if (name == "rotation_prediction_lead_time_sec") {
        candidate.rotation_prediction_lead_time_sec = parameter.as_double();
      } else if (name == "rotation_prediction_yaw_acceleration_filter_time_"
                         "constant_sec") {
        candidate
            .rotation_prediction_yaw_acceleration_filter_time_constant_sec =
            parameter.as_double();
      } else if (name == "rotation_prediction_max_yaw_acceleration_radps2") {
        candidate.rotation_prediction_max_yaw_acceleration_radps2 =
            parameter.as_double();
      } else if (name == "rotation_prediction_entry_threshold_radps") {
        candidate.rotation_prediction_entry_threshold_radps =
            parameter.as_double();
      } else if (name == "rotation_prediction_min_yaw_acceleration_radps2") {
        candidate.rotation_prediction_min_yaw_acceleration_radps2 =
            parameter.as_double();
      } else if (name == "rotation_prediction_slip_angle_threshold_rad") {
        candidate.rotation_prediction_slip_angle_threshold_rad =
            parameter.as_double();
      } else if (name == "rotation_prediction_slip_yaw_rate_gain") {
        candidate.rotation_prediction_slip_yaw_rate_gain =
            parameter.as_double();
      } else if (name == "steering_tracking_speed_gate_enabled") {
        candidate.steering_tracking_speed_gate_enabled = parameter.as_bool();
      } else if (name == "steering_tracking_entry_error_rad") {
        candidate.steering_tracking_entry_error_rad = parameter.as_double();
      } else if (name == "steering_tracking_release_error_rad") {
        candidate.steering_tracking_release_error_rad = parameter.as_double();
      } else if (name == "steering_tracking_entry_duration_sec") {
        candidate.steering_tracking_entry_duration_sec = parameter.as_double();
      } else if (name == "steering_tracking_release_duration_sec") {
        candidate.steering_tracking_release_duration_sec =
            parameter.as_double();
      } else if (name == "steering_tracking_deceleration_mps2") {
        candidate.steering_tracking_deceleration_mps2 = parameter.as_double();
      } else if (name == "wall_edge_tracking_speed_enabled") {
        candidate.wall_edge_tracking_speed_enabled = parameter.as_bool();
      } else if (name == "wall_edge_lateral_velocity_preview_sec") {
        candidate.wall_edge_lateral_velocity_preview_sec =
            parameter.as_double();
      } else if (name == "wall_edge_minimum_speed_mps") {
        candidate.wall_edge_minimum_speed_mps = parameter.as_double();
      } else if (name == "wall_edge_tracking_deceleration_mps2") {
        candidate.wall_edge_tracking_deceleration_mps2 = parameter.as_double();
      } else if (name == "wall_edge_tracking_release_reserve_ratio") {
        candidate.wall_edge_tracking_release_reserve_ratio =
            parameter.as_double();
      } else if (name == "wall_edge_tracking_release_duration_sec") {
        candidate.wall_edge_tracking_release_duration_sec =
            parameter.as_double();
      } else if (name == "ga_run_id") {
        run_id = parameter.as_string();
      } else if (name == "ga_candidate_id") {
        candidate_id = parameter.as_string();
      } else if (name == "ga_parameter_hash") {
        parameter_hash = parameter.as_string();
      } else {
        result.reason = "parameter is immutable during a GA run: " + name;
        return result;
      }
    }
  } catch (const rclcpp::ParameterTypeException &error) {
    result.reason = error.what();
    return result;
  }

  if (!finiteInRange(candidate.lookahead_gain, 0.05, 3.0) ||
      !finiteInRange(candidate.lookahead_min_distance, 0.5, 20.0) ||
      !finiteInRange(candidate.actual_lookahead_distance_blend, 0.0, 1.0) ||
      !finiteInRange(candidate.dual_preview_near_ratio, 0.1, 1.0) ||
      !finiteInRange(candidate.dual_preview_blend, 0.0, 1.0) ||
      !finiteInRange(candidate.steering_tire_angle_gain, 0.1, 3.0) ||
      !finiteInRange(candidate.curvature_lookahead_min_distance, 0.5, 20.0) ||
      !finiteInRange(candidate.curvature_lookahead_sensitivity, 0.0, 100.0) ||
      !finiteInRange(candidate.curvature_lookahead_smoothing_alpha, 0.0, 1.0) ||
      !finiteInRange(candidate.max_lateral_acceleration, 0.5, 20.0) ||
      !finiteInRange(candidate.minimum_corner_speed, 0.0,
                     candidate.external_target_vel) ||
      !finiteInRange(candidate.corner_speed_retention, 0.0, 1.0) ||
      !finiteInRange(candidate.corner_speed_retention_lateral_error_soft, 0.0,
                     5.0) ||
      !finiteInRange(candidate.corner_speed_retention_lateral_error_hard, 0.0,
                     5.0) ||
      candidate.corner_speed_retention_lateral_error_hard <=
          candidate.corner_speed_retention_lateral_error_soft ||
      !finiteInRange(candidate.curvature_speed_preview_distance, 1.0, 50.0) ||
      !finiteInRange(candidate.speed_proportional_gain, 0.0, 10.0) ||
      !finiteInRange(candidate.longitudinal_acceleration_limit, 0.0, 3.0) ||
      !finiteInRange(candidate.longitudinal_deceleration_limit, 0.1, 10.0) ||
      !finiteInRange(candidate.safe_stop_deceleration, 0.1, 10.0) ||
      !finiteInRange(candidate.steering_command_to_tire_angle_ratio, 0.1,
                     1.0) ||
      !finiteInRange(candidate.hard_steering_angle_limit_rad, 0.05, 1.0) ||
      !finiteInRange(candidate.maximum_tire_steering_angle_rad, 0.05, 1.0) ||
      !finiteInRange(candidate.hard_steering_rate_limit_radps, 0.1, 20.0) ||
      !finiteInRange(candidate.physical_tire_steering_rate_radps, 0.0, 20.0) ||
      !finiteInRange(candidate.steering_acceleration_hold_minimum_speed_mps,
                     0.0, 20.0) ||
      !finiteInRange(
          candidate.steering_acceleration_hold_minimum_tire_angle_rad, 0.0,
          candidate.maximum_tire_steering_angle_rad) ||
      !finiteInRange(candidate.steering_acceleration_hold_tracking_error_rad,
                     0.0, 1.0) ||
      !finiteInRange(
          candidate.steering_acceleration_hold_maximum_acceleration_mps2,
          0.0, 3.0) ||
      !finiteInRange(candidate.external_target_vel, 0.0, 30.0) ||
      !finiteInRange(candidate.pp_control_delay_sec, 0.0, 1.0) ||
      !finiteInRange(candidate.pp_prediction_dt_sec, 0.001, 0.10) ||
      !finiteInRange(candidate.steering_time_constant_sec, 0.001, 2.0) ||
      !finiteInRange(candidate.steering_status_timeout_sec, 0.01, 2.0) ||
      !finiteInRange(candidate.min_velocity_for_delay_compensation_mps, 0.0,
                     10.0) ||
      !finiteInRange(candidate.curvature_feedforward_gain, 0.0, 2.0) ||
      !finiteInRange(candidate.curvature_feedforward_preview_distance_m, 0.5,
                     20.0) ||
      !finiteInRange(candidate.curvature_feedforward_time_constant_sec, 0.0,
                     2.0) ||
      !finiteInRange(candidate.maneuver_lookahead_time_constant_sec, 0.0,
                     2.0) ||
      !finiteInRange(candidate.exit_unwind_far_preview_boost, 0.0, 1.0) ||
      !finiteInRange(candidate.exit_unwind_curvature_drop_threshold, 0.001,
                     1.0) ||
      !finiteInRange(candidate.rotation_gate_min_curvature, 0.001, 1.0) ||
      !finiteInRange(candidate.rotation_gate_min_steering_angle, 0.01, 1.0) ||
      !finiteInRange(candidate.rotation_gate_yaw_rate_error_threshold, 0.01,
                     10.0) ||
      !finiteInRange(candidate.rotation_gate_yaw_rate_error_release_ratio, 0.1,
                     1.0) ||
      !finiteInRange(candidate.rotation_gate_max_lateral_error, 0.1, 5.0) ||
      !finiteInRange(candidate.rotation_gate_min_duration_sec, 0.01, 1.0) ||
      !finiteInRange(candidate.rotation_gate_max_duration_sec, 0.01, 2.0) ||
      candidate.rotation_gate_max_duration_sec <
          candidate.rotation_gate_min_duration_sec ||
      !finiteInRange(candidate.rotation_gate_cooldown_sec, 0.0, 5.0) ||
      !finiteInRange(candidate.rotation_gate_countersteer_gain, 0.0, 2.0) ||
      !finiteInRange(candidate.rotation_gate_max_countersteer_rad, 0.0, 0.5) ||
      !finiteInRange(candidate.rotation_prediction_lead_time_sec, 0.0, 1.0) ||
      !finiteInRange(
          candidate
              .rotation_prediction_yaw_acceleration_filter_time_constant_sec,
          0.001, 1.0) ||
      !finiteInRange(candidate.rotation_prediction_max_yaw_acceleration_radps2,
                     0.1, 30.0) ||
      !finiteInRange(candidate.rotation_prediction_entry_threshold_radps, 0.01,
                     10.0) ||
      candidate.rotation_prediction_entry_threshold_radps >
          candidate.rotation_gate_yaw_rate_error_threshold ||
      !finiteInRange(candidate.rotation_prediction_min_yaw_acceleration_radps2,
                     0.0, 30.0) ||
      !finiteInRange(candidate.rotation_prediction_slip_angle_threshold_rad,
                     0.0, 1.0) ||
      !finiteInRange(candidate.rotation_prediction_slip_yaw_rate_gain, 0.0,
                     20.0) ||
      !finiteInRange(candidate.steering_tracking_entry_error_rad, 0.01, 1.0) ||
      !finiteInRange(candidate.steering_tracking_release_error_rad, 0.0, 1.0) ||
      candidate.steering_tracking_release_error_rad >=
          candidate.steering_tracking_entry_error_rad ||
      !finiteInRange(candidate.steering_tracking_entry_duration_sec, 0.0,
                     2.0) ||
      !finiteInRange(candidate.steering_tracking_release_duration_sec, 0.0,
                     2.0) ||
      !finiteInRange(candidate.steering_tracking_deceleration_mps2, 0.0,
                     10.0) ||
      !finiteInRange(candidate.wall_edge_lateral_velocity_preview_sec, 0.0,
                     2.0) ||
      !finiteInRange(candidate.wall_edge_minimum_speed_mps, 0.0, 10.0) ||
      !finiteInRange(candidate.wall_edge_tracking_deceleration_mps2, 0.0,
                     10.0) ||
      !finiteInRange(candidate.wall_edge_tracking_release_reserve_ratio, 0.0,
                     1.0) ||
      !finiteInRange(candidate.wall_edge_tracking_release_duration_sec, 0.0,
                     2.0)) {
    result.reason = "candidate contains a non-finite or out-of-range value";
    return result;
  }

  tuning_ = candidate;
  ga_run_id_ = run_id;
  ga_candidate_id_ = candidate_id;
  ga_parameter_hash_ = parameter_hash;
  result.successful = true;
  result.reason = "candidate applied atomically";
  return result;
}

void SimplePurePursuit::onSetEnabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!ga_experiment_mode_) {
    response->success = false;
    response->message = "service requires ga_experiment_mode=true";
    return;
  }
  controller_enabled_ = request->data;
  if (!controller_enabled_) {
    pub_cmd_->publish(zeroCommand(get_clock()->now()));
  }
  response->success = true;
  response->message =
      controller_enabled_ ? "controller enabled" : "controller disabled";
}

void SimplePurePursuit::onResetState(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!ga_experiment_mode_ || controller_enabled_) {
    response->success = false;
    response->message = "reset requires GA mode with the controller disabled";
    return;
  }
  resetExperimentState();
  response->success = true;
  response->message = "controller history reset";
}

void SimplePurePursuit::resetExperimentState() {
  direct_trajectory_.reset();
  direct_command_.reset();
  direct_received_sec_ = -1.0;
  smoothed_curvature_ = 0.0;
  curvature_initialized_ = false;
  smoothed_feedforward_curvature_ = 0.0;
  feedforward_curvature_initialized_ = false;
  smoothed_maneuver_lookahead_distance_m_ = 0.0;
  maneuver_lookahead_distance_initialized_ = false;
  previous_steering_ = 0.0;
  steering_limiter_initialized_ = false;
  last_bounded_steering_rad_ = 0.0;
  last_control_publish_sec_ = -1.0;
  rotation_controller_state_.active = false;
  rotation_controller_state_.started_sec = 0.0;
  rotation_controller_state_.cooldown_until_sec = 0.0;
  rotation_controller_state_.yaw_acceleration_initialized = false;
  rotation_controller_state_.previous_yaw_rate_radps = 0.0;
  rotation_controller_state_.filtered_yaw_acceleration_radps2 = 0.0;
  steering_tracking_speed_gate_state_ = SteeringTrackingSpeedGateState{};
  wall_edge_tracking_speed_state_ = WallEdgeTrackingSpeedState{};
  direct_corridor_clearance_profile_ = false;
  direct_owner_commit_id_ = 0U;
  direct_geometry_revision_ = 0U;
  direct_valid_until_sec_ = -1.0;
  direct_remaining_arc_m_ = 0.0;
  direct_corridor_side_ = 0;
  direct_wall_edge_profile_ = false;
  direct_wall_side_ = 0;
  direct_wall_clearance_reserve_m_ = 0.0;
  direct_opponent_clearance_reserve_m_ = 0.0;
  direct_corridor_clearance_reserve_m_ = 0.0;
  direct_desired_control_reserve_m_ = 0.0;
  previous_gnss_pose_.reset();
  gnss_velocity_x_ = 0.0;
  gnss_velocity_y_ = 0.0;
  gnss_velocity_valid_ = false;
  ++reset_epoch_;
}

bool SimplePurePursuit::subscribeMessageAvailable() const {
  return odometry_ && trajectory_ && !trajectory_->points.empty();
}

void SimplePurePursuit::selectSplitTrajectory(const double now_sec) {
  if (use_atomic_direct_trajectory_command_ && hold_last_direct_trajectory_enabled_) {
    trajectory_ = direct_trajectory_;
    trajectory_received_sec_ = direct_received_sec_;
    restoreDirectMetadata();
    if (holdingDirectTrajectory() &&
        ((direct_valid_until_sec_ >= 0.0 && now_sec > direct_valid_until_sec_) ||
         !inputSampleFresh(true, now_sec, direct_received_sec_, trajectory_timeout_sec_))) {
      selected_trajectory_source_ = "direct_hold";
    }
    return;
  }
  if (!split_trajectory_inputs_enabled_) {
    return;
  }
  const bool source_fresh =
      inputSampleFresh(path_source_received_sec_ >= 0.0, now_sec,
                       path_source_received_sec_, path_source_timeout_sec_);
  if (!source_fresh) {
    trajectory_.reset();
    trajectory_received_sec_ = -1.0;
    selected_trajectory_source_ = "path_source_stale";
    return;
  }

  const SplitTrajectoryInput requested_input =
      splitTrajectoryInputForPathSource(path_source_);
  if (requested_input == SplitTrajectoryInput::kBaseline) {
    trajectory_ = baseline_trajectory_;
    trajectory_received_sec_ = baseline_trajectory_received_sec_;
    selected_trajectory_source_ = "baseline";
  } else if (requested_input == SplitTrajectoryInput::kManeuver) {
    trajectory_ = maneuver_trajectory_;
    trajectory_received_sec_ = maneuver_trajectory_received_sec_;
    selected_trajectory_source_ = "maneuver";
  } else {
    // Active stuck recovery and unknown sources do not select a CMA path.
    trajectory_.reset();
    trajectory_received_sec_ = -1.0;
    selected_trajectory_source_ = "mpc_owner";
  }
}

void SimplePurePursuit::restoreDirectMetadata() {
  if (!direct_command_) return;
  const auto &message = *direct_command_;
  direct_owner_commit_id_ = message.owner_commit_id;
  direct_geometry_revision_ = message.geometry_revision;
  direct_valid_until_sec_ = message.valid_until_sec;
  direct_remaining_arc_m_ = message.remaining_arc_m;
  direct_corridor_clearance_profile_ = message.corridor_clearance_profile;
  direct_corridor_side_ = message.corridor_side;
  direct_wall_edge_profile_ = message.wall_edge_profile;
  direct_wall_side_ = message.wall_side;
  direct_wall_clearance_reserve_m_ = message.wall_clearance_reserve_m;
  direct_opponent_clearance_reserve_m_ = message.opponent_clearance_reserve_m;
  direct_corridor_clearance_reserve_m_ = message.corridor_clearance_reserve_m;
  direct_desired_control_reserve_m_ = message.desired_control_reserve_m;
  overtake_mode_ = directTrackingMode(message.mode,direct_corridor_side_);
  overtake_mode_received_at_ = rclcpp::Time(
      static_cast<int64_t>(direct_received_sec_*1e9),get_clock()->get_clock_type());
  selected_trajectory_source_ = "direct";
}

bool SimplePurePursuit::holdingDirectTrajectory() const {
  return hold_last_direct_trajectory_enabled_ && use_atomic_direct_trajectory_command_ &&
         !recovery_waiting_reference_ && direct_command_ && direct_trajectory_ &&
         trajectory_ == direct_trajectory_ && !direct_trajectory_->points.empty();
}

bool SimplePurePursuit::inputSamplesFresh(const double now_sec) const {
  // Expiry does not authorize a different path or speed. Retain the accepted
  // atomic trajectory, including its mode, until replacement or recovery/reset.
  const bool hold_direct = holdingDirectTrajectory();
  const bool lease_fresh = hold_direct || !use_atomic_direct_trajectory_command_ ||
                           direct_valid_until_sec_ < 0.0 ||
                           now_sec <= direct_valid_until_sec_;
  return lease_fresh &&
         inputSampleFresh(odometry_ != nullptr, now_sec, odometry_received_sec_,
                          odometry_timeout_sec_) &&
         (hold_direct || inputSampleFresh(trajectory_ != nullptr, now_sec,
                          trajectory_received_sec_, trajectory_timeout_sec_));
}

double
SimplePurePursuit::estimateCurvature(const std::size_t nearest_index) const {
  const auto &points = trajectory_->points;
  if (points.size() < 3) {
    return 0.0;
  }
  const std::size_t first = nearest_index > 1 ? nearest_index - 1 : 0;
  const std::size_t last = std::min(points.size() - 1, nearest_index + 4);
  const std::size_t middle = (first + last) / 2;
  const auto &a = points[first].pose.position;
  const auto &b = points[middle].pose.position;
  const auto &c = points[last].pose.position;
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double denominator = ab * bc * ca;
  if (denominator <= 1.0e-6) {
    return 0.0;
  }
  const double twice_area =
      std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  return 2.0 * twice_area / denominator;
}

double SimplePurePursuit::estimateSignedCurvature(
    const std::size_t nearest_index) const {
  const auto &points = trajectory_->points;
  return rotationSignedCurvature(points.size(), nearest_index,
      [&](std::size_t index) {
        const auto &p = points[index].pose.position;
        return std::pair<double,double>{p.x,p.y};
      });
}

double SimplePurePursuit::estimatePreviewCurvature(
    const std::size_t nearest_index, const double preview_distance) const {
  const auto &points = trajectory_->points;
  double maximum_curvature = estimateCurvature(nearest_index);
  double distance = 0.0;
  for (std::size_t index = nearest_index + 1; index < points.size(); ++index) {
    const auto &previous = points[index - 1].pose.position;
    const auto &current = points[index].pose.position;
    distance += std::hypot(current.x - previous.x, current.y - previous.y);
    if (distance > preview_distance) {
      break;
    }
    maximum_curvature = std::max(maximum_curvature, estimateCurvature(index));
  }
  return maximum_curvature;
}

void SimplePurePursuit::recordRuntimeTiming(
    const RuntimeTimingSample &sample, const double callback_end_steady_sec) {
  if (!runtime_timing_metrics_enabled_) {
    return;
  }
  runtime_timing_samples_.push_back(sample);
  if (last_runtime_timing_report_steady_sec_ < 0.0) {
    last_runtime_timing_report_steady_sec_ = callback_end_steady_sec;
    return;
  }
  if (callback_end_steady_sec - last_runtime_timing_report_steady_sec_ <
      runtime_timing_report_period_sec_) {
    return;
  }

  const auto collect = [this](double RuntimeTimingSample::*member) {
    std::vector<double> values;
    values.reserve(runtime_timing_samples_.size());
    for (const auto &entry : runtime_timing_samples_) {
      values.push_back(entry.*member);
    }
    return values;
  };
  const auto callback =
      summarizeTiming(collect(&RuntimeTimingSample::callback_wall_ms));
  const auto thread_cpu =
      summarizeTiming(collect(&RuntimeTimingSample::callback_thread_cpu_ms));
  const auto dispatch =
      summarizeTiming(collect(&RuntimeTimingSample::dispatch_interval_ms));
  const auto lock_wait =
      summarizeTiming(collect(&RuntimeTimingSample::lock_wait_ms));
  const auto precheck =
      summarizeTiming(collect(&RuntimeTimingSample::precheck_ms));
  const auto input_delay =
      summarizeTiming(collect(&RuntimeTimingSample::input_and_delay_ms));
  const auto path_preparation =
      summarizeTiming(collect(&RuntimeTimingSample::path_preparation_ms));
  const auto preview_selection =
      summarizeTiming(collect(&RuntimeTimingSample::preview_selection_ms));
  const auto control =
      summarizeTiming(collect(&RuntimeTimingSample::control_ms));
  const auto publish =
      summarizeTiming(collect(&RuntimeTimingSample::publish_ms));
  const auto diagnostic =
      summarizeTiming(collect(&RuntimeTimingSample::diagnostic_ms));

  const double deadline_ms = runtime_timing_deadline_sec_ * 1000.0;
  std::size_t callback_deadline_exceeded = 0U;
  std::size_t dispatch_deadline_exceeded = 0U;
  std::size_t dispatch_sample_count = 0U;
  for (const auto &entry : runtime_timing_samples_) {
    if (entry.callback_wall_ms > deadline_ms) {
      ++callback_deadline_exceeded;
    }
    if (std::isfinite(entry.dispatch_interval_ms)) {
      ++dispatch_sample_count;
      if (entry.dispatch_interval_ms > deadline_ms) {
        ++dispatch_deadline_exceeded;
      }
    }
  }
  const double callback_exceeded_ratio =
      static_cast<double>(callback_deadline_exceeded) /
      static_cast<double>(runtime_timing_samples_.size());
  const double dispatch_exceeded_ratio =
      dispatch_sample_count > 0U
          ? static_cast<double>(dispatch_deadline_exceeded) /
                static_cast<double>(dispatch_sample_count)
          : std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(
      get_logger(),
      "[cma_pp_timing] samples=%zu deadline_ms=%.3f "
      "callback_ms=(p50=%.3f,p95=%.3f,p99=%.3f,max=%.3f) "
      "thread_cpu_ms=(p50=%.3f,p95=%.3f,p99=%.3f,max=%.3f) "
      "dispatch_ms=(p50=%.3f,p95=%.3f,p99=%.3f,max=%.3f) "
      "deadline_exceeded=(callback=%zu,callback_ratio=%.6f,dispatch=%zu,"
      "dispatch_samples=%zu,dispatch_ratio=%.6f) "
      "stage_p95_ms=(lock=%.3f,precheck=%.3f,input_delay=%.3f,path=%.3f,"
      "preview=%.3f,control=%.3f,publish=%.3f,diagnostic=%.3f) "
      "stage_max_ms=(lock=%.3f,precheck=%.3f,input_delay=%.3f,path=%.3f,"
      "preview=%.3f,control=%.3f,publish=%.3f,diagnostic=%.3f)",
      runtime_timing_samples_.size(), deadline_ms, callback.p50_ms,
      callback.p95_ms, callback.p99_ms, callback.maximum_ms, thread_cpu.p50_ms,
      thread_cpu.p95_ms, thread_cpu.p99_ms, thread_cpu.maximum_ms,
      dispatch.p50_ms, dispatch.p95_ms, dispatch.p99_ms, dispatch.maximum_ms,
      callback_deadline_exceeded, callback_exceeded_ratio,
      dispatch_deadline_exceeded, dispatch_sample_count,
      dispatch_exceeded_ratio, lock_wait.p95_ms, precheck.p95_ms,
      input_delay.p95_ms, path_preparation.p95_ms, preview_selection.p95_ms,
      control.p95_ms, publish.p95_ms, diagnostic.p95_ms, lock_wait.maximum_ms,
      precheck.maximum_ms, input_delay.maximum_ms, path_preparation.maximum_ms,
      preview_selection.maximum_ms, control.maximum_ms, publish.maximum_ms,
      diagnostic.maximum_ms);

  runtime_timing_samples_.clear();
  last_runtime_timing_report_steady_sec_ = callback_end_steady_sec;
}

void SimplePurePursuit::onTimer() {
  using TimingClock = std::chrono::steady_clock;
  using TimingPoint = TimingClock::time_point;
  const auto timing_now = [this]() {
    return runtime_timing_metrics_enabled_ ? TimingClock::now() : TimingPoint{};
  };
  const auto elapsed_ms = [](const TimingPoint &start, const TimingPoint &end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };
  RuntimeTimingSample timing_sample;
  timing_sample.dispatch_interval_ms = std::numeric_limits<double>::quiet_NaN();
  const TimingPoint callback_start = timing_now();
  const double callback_thread_cpu_start =
      runtime_timing_metrics_enabled_
          ? threadCpuSeconds()
          : std::numeric_limits<double>::quiet_NaN();
  if (runtime_timing_metrics_enabled_) {
    if (runtime_timing_callback_started_) {
      timing_sample.dispatch_interval_ms =
          elapsed_ms(last_runtime_timing_callback_start_, callback_start);
    }
    last_runtime_timing_callback_start_ = callback_start;
    runtime_timing_callback_started_ = true;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  const TimingPoint after_lock = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.lock_wait_ms = elapsed_ms(callback_start, after_lock);
  }
  const auto finalize_timing = [&]() {
    if (!runtime_timing_metrics_enabled_) {
      return;
    }
    const TimingPoint callback_end = TimingClock::now();
    const double callback_thread_cpu_end = threadCpuSeconds();
    timing_sample.callback_wall_ms = elapsed_ms(callback_start, callback_end);
    timing_sample.callback_thread_cpu_ms =
        std::isfinite(callback_thread_cpu_start) &&
                std::isfinite(callback_thread_cpu_end)
            ? (callback_thread_cpu_end - callback_thread_cpu_start) * 1000.0
            : std::numeric_limits<double>::quiet_NaN();
    recordRuntimeTiming(
        timing_sample,
        std::chrono::duration<double>(callback_end.time_since_epoch()).count());
  };
  const rclcpp::Time now = get_clock()->now();
  const double now_sec = now.seconds();
  selectSplitTrajectory(now_sec);
  if (!controller_enabled_ || recovery_paused_) {
    const TimingPoint publish_start = timing_now();
    if (runtime_timing_metrics_enabled_) {
      timing_sample.precheck_ms = elapsed_ms(after_lock, publish_start);
    }
    const auto command = zeroCommand(now);
    pub_cmd_->publish(command);
    pub_raw_cmd_->publish(command);
    const TimingPoint publish_end = timing_now();
    if (runtime_timing_metrics_enabled_) {
      timing_sample.publish_ms = elapsed_ms(publish_start, publish_end);
    }
    finalize_timing();
    return;
  }
  if (!subscribeMessageAvailable() || !inputSamplesFresh(now_sec)) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "odometry or trajectory is unavailable or stale "
        "(odom_age=%.3f trajectory_age=%.3f selected=%s path_source=%s)",
        odometry_received_sec_ >= 0.0 ? now_sec - odometry_received_sec_ : -1.0,
        trajectory_received_sec_ >= 0.0 ? now_sec - trajectory_received_sec_
                                        : -1.0,
        selected_trajectory_source_.c_str(), path_source_.c_str());
    const TimingPoint publish_start = timing_now();
    if (runtime_timing_metrics_enabled_) {
      timing_sample.precheck_ms = elapsed_ms(after_lock, publish_start);
    }
    auto command = zeroCommand(now);
    if (recovery_waiting_reference_) {
      command.longitudinal.acceleration = -tuning_.safe_stop_deceleration;
    }
    pub_cmd_->publish(command);
    pub_raw_cmd_->publish(command);
    last_control_publish_sec_ = now_sec;
    const TimingPoint publish_end = timing_now();
    if (runtime_timing_metrics_enabled_) {
      timing_sample.publish_ms = elapsed_ms(publish_start, publish_end);
    }
    finalize_timing();
    return;
  }

  const TimingPoint precheck_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.precheck_ms = elapsed_ms(after_lock, precheck_end);
  }

  const double current_velocity = odometry_->twist.twist.linear.x;
  const double current_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double current_rear_x = odometry_->pose.pose.position.x -
                                wheel_base_ * 0.5 * std::cos(current_yaw);
  const double current_rear_y = odometry_->pose.pose.position.y -
                                wheel_base_ * 0.5 * std::sin(current_yaw);
  double control_yaw = current_yaw;
  double control_rear_x = current_rear_x;
  double control_rear_y = current_rear_y;
  bool steering_status_fresh = false;
  bool delay_compensation_used = false;
  DelayedPosePrediction delay_prediction;
  const double measured_control_dt_sec =
      last_control_publish_sec_ >= 0.0 ? now_sec - last_control_publish_sec_
                                       : 0.01;
  // PP and the planner intentionally run at different rates. Filtering uses
  // the measured PP interval and never assumes a matching planner cycle.
  const double control_dt_sec =
      std::clamp(measured_control_dt_sec, 1.0e-3, 0.20);
  const bool overtake_mode_fresh = holdingDirectTrajectory() || (
      overtake_mode_received_at_.nanoseconds() > 0 &&
      (now - overtake_mode_received_at_).seconds() >= 0.0 &&
      (now - overtake_mode_received_at_).seconds() <=
          overtake_mode_timeout_sec_);
  if (steering_status_ &&
      std::isfinite(steering_status_->steering_tire_angle)) {
    const rclcpp::Time steering_stamp(steering_status_->stamp);
    const double steering_age_sec = (now - steering_stamp).seconds();
    steering_status_fresh =
        steering_stamp.nanoseconds() > 0 && steering_age_sec >= 0.0 &&
        steering_age_sec <= tuning_.steering_status_timeout_sec;
  }

  if (tuning_.delay_compensation_enabled &&
      tuning_.pp_control_delay_sec > 0.0 &&
      current_velocity >= tuning_.min_velocity_for_delay_compensation_mps &&
      steering_status_fresh) {
    const EgoControlState current_state{current_rear_x, current_rear_y,
                                        current_yaw, current_velocity};
    const double previous_target_tire_angle = commandToTireSteeringAngle(
        previous_steering_, tuning_.steering_command_to_tire_angle_ratio,
        tuning_.maximum_tire_steering_angle_rad);
    delay_prediction = predictDelayedPose(
        current_state, steering_status_->steering_tire_angle,
        previous_target_tire_angle, tuning_.pp_control_delay_sec,
        tuning_.pp_prediction_dt_sec, tuning_.steering_time_constant_sec,
        wheel_base_, tuning_.maximum_tire_steering_angle_rad,
        physicalTireSteeringRate(tuning_.physical_tire_steering_rate_radps,
            tuning_.hard_steering_rate_limit_radps,
            tuning_.steering_command_to_tire_angle_ratio));
    if (delay_prediction.shifted) {
      control_rear_x = delay_prediction.x;
      control_rear_y = delay_prediction.y;
      control_yaw = delay_prediction.yaw;
      delay_compensation_used = true;
    }
  }

  const TimingPoint input_and_delay_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.input_and_delay_ms =
        elapsed_ms(precheck_end, input_and_delay_end);
  }

  geometry_msgs::msg::Point control_position = odometry_->pose.pose.position;
  if (delay_compensation_used) {
    control_position.x =
        control_rear_x + wheel_base_ * 0.5 * std::cos(control_yaw);
    control_position.y =
        control_rear_y + wheel_base_ * 0.5 * std::sin(control_yaw);
  }
  const std::size_t nearest_index =
      motion_utils::findNearestIndex(trajectory_->points, control_position);
  const TrajectoryPoint &nearest = trajectory_->points.at(nearest_index);
  const double maximum_velocity =
      (ga_experiment_mode_ || use_external_target_vel_)
          ? tuning_.external_target_vel
          : nearest.longitudinal_velocity_mps;

  const double curvature = estimatePreviewCurvature(
      nearest_index, tuning_.curvature_speed_preview_distance);
  const double local_signed_curvature = estimateSignedCurvature(nearest_index);
  const double preview_signed_curvature = estimateDistanceWindowSignedCurvature(
      trajectory_->points, nearest_index,
      tuning_.curvature_feedforward_preview_distance_m,
      [](const TrajectoryPoint &point) {
        return std::pair<double, double>{point.pose.position.x,
                                         point.pose.position.y};
      });
  const std::size_t exit_preview_index = selectForwardTrajectoryIndex(
      *trajectory_, nearest_index, tuning_.curvature_speed_preview_distance);
  const double exit_signed_curvature =
      estimateSignedCurvature(exit_preview_index);
  const double alpha = tuning_.curvature_lookahead_smoothing_alpha;
  if (!curvature_initialized_) {
    smoothed_curvature_ = curvature;
    curvature_initialized_ = true;
  } else {
    smoothed_curvature_ =
        alpha * curvature + (1.0 - alpha) * smoothed_curvature_;
  }
  const double curvature_speed_limit = std::sqrt(
      tuning_.max_lateral_acceleration / std::max(smoothed_curvature_, 1.0e-4));
  const double curvature_target_velocity =
      std::min(maximum_velocity,
               std::max(tuning_.minimum_corner_speed, curvature_speed_limit));
  const double lateral_error = tier4_autoware_utils::calcLateralDeviation(
      nearest.pose, control_position);
  const double absolute_lateral_error = std::abs(lateral_error);
  const double retention_gate =
      tuning_.lateral_error_speed_gate_enabled
          ? std::clamp((tuning_.corner_speed_retention_lateral_error_hard -
                        absolute_lateral_error) /
                           (tuning_.corner_speed_retention_lateral_error_hard -
                            tuning_.corner_speed_retention_lateral_error_soft),
                       0.0, 1.0)
          : 1.0;
  const double effective_corner_speed_retention =
      tuning_.corner_speed_retention * retention_gate;
  const double target_velocity = curvature_target_velocity +
      effective_corner_speed_retention *
          (maximum_velocity - curvature_target_velocity);
  const double lookahead_speed_basis_mps =
      selectLookaheadSpeedBasis(target_velocity, current_velocity,
                                maneuver_lookahead_uses_measured_speed_,
                                overtake_mode_fresh, overtake_mode_);
  const PurePursuitLookaheadPolicy lookahead_policy{
      tuning_.lookahead_gain, tuning_.lookahead_min_distance,
      tuning_.curvature_lookahead_min_distance,
      tuning_.curvature_lookahead_sensitivity};
  const double raw_lookahead_distance = computePurePursuitRawLookahead(
      lookahead_speed_basis_mps, smoothed_curvature_, lookahead_policy);
  const double lookahead_distance = smoothManeuverLookaheadDistance(
      raw_lookahead_distance, smoothed_maneuver_lookahead_distance_m_,
      maneuver_lookahead_distance_initialized_, control_dt_sec,
      tuning_.maneuver_lookahead_time_constant_sec,
      maneuver_lookahead_uses_measured_speed_, overtake_mode_fresh,
      overtake_mode_);
  smoothed_maneuver_lookahead_distance_m_ = lookahead_distance;
  maneuver_lookahead_distance_initialized_ = true;

  const TimingPoint path_preparation_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.path_preparation_ms =
        elapsed_ms(input_and_delay_end, path_preparation_end);
  }

  struct PreviewTarget {
    TrajectoryPoint point;
    std::size_t lower_index;
    std::size_t upper_index;
    bool endpoint_fallback;
  };
  const auto find_preview_point = [&](const double distance) {
    const auto selected = selectPurePursuitPreview(
        &trajectory_->points[0U], trajectory_->points.size(), nearest_index,
        distance, control_rear_x, control_rear_y,
        tuning_.continuous_preview_interpolation_enabled,
        [](const TrajectoryPoint &point) { return point.pose.position.x; },
        [](const TrajectoryPoint &point) { return point.pose.position.y; });
    TrajectoryPoint point = trajectory_->points[std::min(
        selected.lower_index, trajectory_->points.size() - 1U)];
    point.pose.position.x = selected.x;
    point.pose.position.y = selected.y;
    if (selected.upper_index < trajectory_->points.size() &&
        selected.lower_index < trajectory_->points.size()) {
      const auto &lower = trajectory_->points[selected.lower_index];
      const auto &upper = trajectory_->points[selected.upper_index];
      point.pose.position.z =
          lower.pose.position.z +
          selected.interpolation_ratio *
              (upper.pose.position.z - lower.pose.position.z);
    }
    return PreviewTarget{point, selected.lower_index, selected.upper_index,
                         selected.endpoint_fallback};
  };
  const PreviewTarget lookahead = find_preview_point(lookahead_distance);
  const double near_preview_distance =
      std::max(0.5, lookahead_distance * tuning_.dual_preview_near_ratio);
  const PreviewTarget near_preview = find_preview_point(near_preview_distance);
  const TimingPoint preview_selection_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.preview_selection_ms =
        elapsed_ms(path_preparation_end, preview_selection_end);
  }
  const double curvature_drop = std::max(
      0.0, std::abs(local_signed_curvature) - std::abs(exit_signed_curvature));
  const double exit_unwind_progress =
      tuning_.exit_unwind_enabled
          ? std::clamp(curvature_drop /
                           tuning_.exit_unwind_curvature_drop_threshold,
                       0.0, 1.0)
          : 0.0;
  const double effective_dual_preview_blend = std::clamp(
      tuning_.dual_preview_blend -
          tuning_.exit_unwind_far_preview_boost * exit_unwind_progress,
      0.0, 1.0);
  const bool curvature_feedforward_active =
      tuning_.curvature_feedforward_enabled &&
      (!curvature_feedforward_maneuver_only_ ||
       (overtake_mode_fresh &&
        isCurvatureFeedforwardManeuverMode(overtake_mode_)));
  if (!curvature_feedforward_active) {
    smoothed_feedforward_curvature_ = 0.0;
    feedforward_curvature_initialized_ = false;
  } else if (!feedforward_curvature_initialized_) {
    smoothed_feedforward_curvature_ =
        timeDomainLowPass(preview_signed_curvature, 0.0, control_dt_sec,
                          tuning_.curvature_feedforward_time_constant_sec);
    feedforward_curvature_initialized_ = true;
  } else {
    smoothed_feedforward_curvature_ = timeDomainLowPass(
        preview_signed_curvature, smoothed_feedforward_curvature_,
        control_dt_sec, tuning_.curvature_feedforward_time_constant_sec);
  }

  AckermannControlCommand command = zeroCommand(get_clock()->now());
  command.longitudinal.speed = target_velocity;
  const bool safe_stop_mode =
      overtake_mode_fresh && overtake_mode_ == "SAFE_STOP";
  const auto bounded_acceleration = boundLongitudinalAcceleration(
      target_velocity, current_velocity, tuning_.speed_proportional_gain,
      tuning_.longitudinal_acceleration_limit,
      tuning_.longitudinal_deceleration_limit, safe_stop_mode,
      tuning_.safe_stop_deceleration);
  command.longitudinal.acceleration =
      bounded_acceleration.valid ? bounded_acceleration.bounded_mps2 : 0.0;
  const double actual_lookahead_distance =
      std::hypot(lookahead.point.pose.position.x - control_rear_x,
                 lookahead.point.pose.position.y - control_rear_y);
  const double actual_near_preview_distance =
      std::hypot(near_preview.point.pose.position.x - control_rear_x,
                 near_preview.point.pose.position.y - control_rear_y);
  const double heading_error =
      std::atan2(lookahead.point.pose.position.y - control_rear_y,
                 lookahead.point.pose.position.x - control_rear_x) -
      control_yaw;
  const double near_heading_error =
      std::atan2(near_preview.point.pose.position.y - control_rear_y,
                 near_preview.point.pose.position.x - control_rear_x) -
      control_yaw;
  const double curvature_feedforward_angle =
      curvature_feedforward_active
          ? tuning_.curvature_feedforward_gain *
                std::atan(wheel_base_ * smoothed_feedforward_curvature_)
          : 0.0;
  const double steering_reference =
      steering_limiter_initialized_
          ? last_bounded_steering_rad_
          : (steering_status_fresh
                 ? tireToCommandSteeringAngle(
                       steering_status_->steering_tire_angle,
                       tuning_.steering_command_to_tire_angle_ratio,
                       tuning_.hard_steering_angle_limit_rad)
                 : 0.0);
  const PurePursuitCoreInput pure_pursuit_input{
      control_rear_x,
      control_rear_y,
      control_yaw,
      lookahead.point.pose.position.x,
      lookahead.point.pose.position.y,
      near_preview.point.pose.position.x,
      near_preview.point.pose.position.y,
      lookahead_distance,
      near_preview_distance,
      tuning_.actual_lookahead_distance_blend,
      effective_dual_preview_blend,
      wheel_base_,
      tuning_.steering_tire_angle_gain /
          tuning_.steering_command_to_tire_angle_ratio,
      target_velocity,
      curvature_feedforward_angle /
          tuning_.steering_command_to_tire_angle_ratio,
      steering_reference,
      control_dt_sec,
      tuning_.hard_steering_angle_limit_rad,
      tuning_.hard_steering_rate_limit_radps,
      tuning_.steering_command_passthrough_enabled};
  const auto bounded_steering = computePurePursuitCore(pure_pursuit_input);
  const double steering_lookahead_distance =
      bounded_steering.far_steering_distance_m;
  const double far_target_local_x = bounded_steering.far_target_local_x;
  const double far_target_local_y = bounded_steering.far_target_local_y;
  const double near_target_local_x = bounded_steering.near_target_local_x;
  const double near_target_local_y = bounded_steering.near_target_local_y;
  const double far_steering_angle = bounded_steering.far_steering_rad;
  const double near_steering_angle = bounded_steering.near_steering_rad;
  const double pure_pursuit_steering_angle =
      bounded_steering.pure_pursuit_steering_rad;
  const double requested_steering_angle =
      bounded_steering.requested_steering_rad;
  command.lateral.steering_tire_angle =
      bounded_steering.valid ? bounded_steering.bounded_steering_rad : 0.0;
  const double nominal_target_tire_angle =
      commandToTireSteeringAngle(command.lateral.steering_tire_angle,
                                 tuning_.steering_command_to_tire_angle_ratio,
                                 tuning_.maximum_tire_steering_angle_rad);

  const SteeringTrackingSpeedGateConfig steering_tracking_gate_config{
      tuning_.steering_tracking_speed_gate_enabled,
      tuning_.steering_tracking_entry_error_rad,
      tuning_.steering_tracking_release_error_rad,
      tuning_.steering_tracking_entry_duration_sec,
      tuning_.steering_tracking_release_duration_sec,
      tuning_.steering_tracking_deceleration_mps2};
  const bool steering_tracking_maneuver_active =
      overtake_mode_fresh && isCurvatureFeedforwardManeuverMode(overtake_mode_);
  const auto steering_tracking_gate = applySteeringTrackingSpeedGate(
      steering_tracking_gate_config, steering_tracking_maneuver_active,
      steering_status_fresh, nominal_target_tire_angle,
      steering_status_fresh ? steering_status_->steering_tire_angle : 0.0,
      current_velocity, command.longitudinal.speed,
      command.longitudinal.acceleration, control_dt_sec,
      &steering_tracking_speed_gate_state_);
  command.longitudinal.speed = steering_tracking_gate.speed_mps;
  command.longitudinal.acceleration = steering_tracking_gate.acceleration_mps2;

  const double measured_yaw_rate = odometry_->twist.twist.angular.z;
  const double reference_yaw_rate = current_velocity * local_signed_curvature;
  const double ground_speed = std::hypot(gnss_velocity_x_, gnss_velocity_y_);
  const bool slip_angle_valid = gnss_velocity_valid_ && ground_speed >= 1.0;
  const double ground_track_yaw =
      slip_angle_valid ? std::atan2(gnss_velocity_y_, gnss_velocity_x_)
                       : current_yaw;
  const double slip_angle =
      slip_angle_valid ? std::atan2(std::sin(ground_track_yaw - current_yaw),
                                    std::cos(ground_track_yaw - current_yaw))
                       : 0.0;
  const double longitudinal_velocity = ground_speed * std::cos(slip_angle);
  const double lateral_velocity = ground_speed * std::sin(slip_angle);
  const double trajectory_yaw = tf2::getYaw(nearest.pose.orientation);
  const double corridor_lateral_velocity =
      gnss_velocity_valid_
          ? -gnss_velocity_x_ * std::sin(trajectory_yaw) +
                gnss_velocity_y_ * std::cos(trajectory_yaw)
          : current_velocity * std::sin(current_yaw - trajectory_yaw);
  const bool directional_wall_edge_profile =
      use_atomic_direct_trajectory_command_ &&
      direct_corridor_clearance_profile_ && direct_wall_edge_profile_ &&
      (direct_wall_side_ == -1 || direct_wall_side_ == 1);
  const WallEdgeTrackingSpeedConfig wall_edge_speed_config{
      tuning_.wall_edge_tracking_speed_enabled,
      tuning_.wall_edge_lateral_velocity_preview_sec,
      tuning_.wall_edge_minimum_speed_mps,
      tuning_.wall_edge_tracking_deceleration_mps2,
      tuning_.wall_edge_tracking_release_reserve_ratio,
      tuning_.wall_edge_tracking_release_duration_sec};
  const auto wall_edge_speed = [&]() {
    if (directional_wall_edge_profile) {
      return applyDirectionalCorridorTrackingSpeed(
          wall_edge_speed_config, true, direct_wall_side_,
          direct_wall_clearance_reserve_m_,
          direct_opponent_clearance_reserve_m_,
          direct_desired_control_reserve_m_, lateral_error,
          corridor_lateral_velocity, current_velocity,
          command.longitudinal.speed, tuning_.speed_proportional_gain,
          command.longitudinal.acceleration, direct_geometry_revision_,
          control_dt_sec, &wall_edge_tracking_speed_state_);
    }
    // A normal PASS d(s) may cross Reference after the target, so its pass
    // side is not a stable wall direction over the full path. Preserve the
    // symmetric gate unless Planner explicitly publishes wall_side.
    return applyWallEdgeTrackingSpeed(
        wall_edge_speed_config,
        use_atomic_direct_trajectory_command_ &&
            direct_corridor_clearance_profile_ &&
            (direct_corridor_side_ == -1 || direct_corridor_side_ == 1),
        direct_corridor_clearance_reserve_m_, direct_desired_control_reserve_m_,
        lateral_error, lateral_velocity, current_velocity,
        command.longitudinal.speed, tuning_.speed_proportional_gain,
        command.longitudinal.acceleration, direct_geometry_revision_,
        control_dt_sec, &wall_edge_tracking_speed_state_);
  }();
  command.longitudinal.speed = wall_edge_speed.speed_mps;
  command.longitudinal.acceleration = wall_edge_speed.acceleration_mps2;
  const double estimated_lateral_acceleration =
      current_velocity * measured_yaw_rate;
  const double rotation_prediction_horizon_sec = tuning_.rotation_prediction_enabled ?
      tuning_.pp_control_delay_sec+tuning_.rotation_prediction_lead_time_sec : 0.;
  const double commanded_yaw_rate =
      current_velocity * std::tan(nominal_target_tire_angle) / wheel_base_;
  const auto rotation_result = stepRotationController(tuning_,
      RotationControllerInput{now_sec, control_dt_sec, local_signed_curvature,
          reference_yaw_rate, measured_yaw_rate, commanded_yaw_rate, slip_angle,
          nominal_target_tire_angle,
          steering_status_fresh ? steering_status_->steering_tire_angle : 0.,
          absolute_lateral_error, slip_angle_valid, steering_status_fresh},
      rotation_controller_state_);
  const auto &rotation_recovery = rotation_result.recovery;
  const double signed_yaw_rate_excess = rotation_result.signed_yaw_rate_excess_radps;
  auto rotation_message=makeRotationPredictionMessage(
      tuning_,rotation_controller_state_,slip_angle,slip_angle_valid);
  rotation_message.header.stamp=command.stamp;
  rotation_prediction_pub_->publish(rotation_message);
  if (rotation_controller_state_.active && rotation_recovery.valid) {
    const double recovery_command_target =
        tireToCommandSteeringAngle(rotation_recovery.requested_steering_rad,
                                   tuning_.steering_command_to_tire_angle_ratio,
                                   tuning_.hard_steering_angle_limit_rad);
    const auto recovery_steering = applySteeringExecutionContract(
        recovery_command_target, previous_steering_, control_dt_sec,
        tuning_.hard_steering_angle_limit_rad,
        tuning_.hard_steering_rate_limit_radps,
        tuning_.steering_command_passthrough_enabled);
    if (recovery_steering.valid) {
      command.lateral.steering_tire_angle = recovery_steering.bounded_angle_rad;
    }
    // Suppress propulsion without erasing braking already requested by the
    // trajectory. When rotation returns inside the hysteresis band, normal
    // acceleration resumes.
    command.longitudinal.acceleration = suppressPositiveAccelerationForRotation(
        command.longitudinal.acceleration);
  }

  const double final_target_tire_angle =
      commandToTireSteeringAngle(command.lateral.steering_tire_angle,
                                 tuning_.steering_command_to_tire_angle_ratio,
                                 tuning_.maximum_tire_steering_angle_rad);
  const auto steering_acceleration_hold =
      holdPositiveAccelerationForSteeringDemand(
          tuning_.steering_demand_acceleration_hold_enabled,
          std::abs(current_velocity),
          tuning_.steering_acceleration_hold_minimum_speed_mps,
          bounded_steering.angle_limited, steering_status_fresh,
          final_target_tire_angle,
          steering_status_fresh ? steering_status_->steering_tire_angle : 0.0,
          tuning_.steering_acceleration_hold_minimum_tire_angle_rad,
          tuning_.steering_acceleration_hold_tracking_error_rad,
          command.longitudinal.acceleration,
          tuning_.steering_acceleration_hold_maximum_acceleration_mps2);
  if (steering_acceleration_hold.valid) {
    command.longitudinal.acceleration =
        steering_acceleration_hold.acceleration_mps2;
  }

  // Preserve the ordinary speed-feedback command except when the SAFE_STOP
  // trajectory explicitly requests brake hold. The raw vehicle converter
  // consumes this before a recovery GearCommand::REVERSE is permitted.
  command.longitudinal.acceleration = applySafeStopTrajectoryDeceleration(
      command.longitudinal.acceleration, nearest.acceleration_mps2,
      safe_stop_mode);

  const TimingPoint control_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.control_ms = elapsed_ms(preview_selection_end, control_end);
  }

  PointStamped lookahead_message;
  lookahead_message.header.stamp = get_clock()->now();
  lookahead_message.header.frame_id = "map";
  lookahead_message.point = lookahead.point.pose.position;
  pub_lookahead_point_->publish(lookahead_message);
  PointStamped near_lookahead_message;
  near_lookahead_message.header = lookahead_message.header;
  near_lookahead_message.point = near_preview.point.pose.position;
  pub_near_lookahead_point_->publish(near_lookahead_message);
  const auto vehicle_longitudinal_command = applyGearRelativeReverseCommand(
      command.longitudinal.speed, command.longitudinal.acceleration,
      gear_relative_reverse_command_enabled_,
      gear_status_ && (gear_status_->report == GearReport::REVERSE ||
                       gear_status_->report == GearReport::REVERSE_2));
  if (!vehicle_longitudinal_command.valid) {
    command.longitudinal.speed = 0.0;
    command.longitudinal.acceleration = 0.0;
  } else {
    command.longitudinal.speed = vehicle_longitudinal_command.speed_mps;
    command.longitudinal.acceleration =
        vehicle_longitudinal_command.acceleration_mps2;
  }
  pub_cmd_->publish(command);
  AckermannControlCommand raw_command = command;
  raw_command.lateral.steering_tire_angle /= tuning_.steering_tire_angle_gain;
  pub_raw_cmd_->publish(raw_command);

  const TimingPoint publish_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.publish_ms = elapsed_ms(control_end, publish_end);
  }

  std_msgs::msg::String debug;
  std::ostringstream json;
  const rclcpp::Time trajectory_stamp(trajectory_->header.stamp);
  const double trajectory_age_sec =
      trajectory_stamp.nanoseconds() > 0
          ? (now - trajectory_stamp).seconds()
          : std::numeric_limits<double>::quiet_NaN();
  const double measured_steering_angle =
      steering_status_fresh ? steering_status_->steering_tire_angle
                            : std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[PURE_PURSUIT_CTRL] mode=%s mode_fresh=%s trajectory_points=%zu "
      "trajectory_age=%.3f ego_speed=%.3f target_speed=%.3f command_speed=%.3f "
      "command_accel=%.3f lateral_error=%.3f lookahead=%.3f "
      "steer_requested=%.4f steer_command=%.4f steer_target_tire=%.4f "
      "steer_measured=%.4f "
      "curvature_ff_active=%s steering_tracking_gate=%s "
      "wall_edge_speed_gate=%s usable_corridor_reserve=%.3f "
      "usable_wall_reserve=%.3f usable_opponent_reserve=%.3f "
      "delay_compensation_used=%s",
      overtake_mode_.c_str(), overtake_mode_fresh ? "true" : "false",
      trajectory_->points.size(), trajectory_age_sec, current_velocity,
      target_velocity, command.longitudinal.speed,
      command.longitudinal.acceleration, lateral_error, lookahead_distance,
      requested_steering_angle, command.lateral.steering_tire_angle,
      final_target_tire_angle, measured_steering_angle,
      curvature_feedforward_active ? "true" : "false",
      steering_tracking_gate.active ? "true" : "false",
      wall_edge_speed.active ? "true" : "false",
      wall_edge_speed.usable_corridor_reserve_m,
      wall_edge_speed.usable_wall_reserve_m,
      wall_edge_speed.usable_opponent_reserve_m,
      delay_compensation_used ? "true" : "false");
  json << "{\"candidate_id\":\"" << jsonEscape(ga_candidate_id_)
       << "\",\"parameter_hash\":\"" << jsonEscape(ga_parameter_hash_)
       << "\",\"reset_epoch\":" << reset_epoch_
       << ",\"atomic_direct_trajectory_command\":"
       << (use_atomic_direct_trajectory_command_ ? "true" : "false")
       << ",\"direct_trajectory_generation\":" << direct_trajectory_generation_
       << ",\"direct_owner_commit_id\":" << direct_owner_commit_id_
       << ",\"direct_geometry_revision\":" << direct_geometry_revision_
       << ",\"direct_valid_until_sec\":" << direct_valid_until_sec_
       << ",\"direct_trajectory_held\":" << (selected_trajectory_source_ == "direct_hold" ? "true" : "false")
       << ",\"direct_remaining_arc_m\":" << direct_remaining_arc_m_
       << ",\"corridor_clearance_profile\":"
       << (direct_corridor_clearance_profile_ ? "true" : "false")
       << ",\"corridor_side\":" << direct_corridor_side_
       << ",\"wall_edge_profile\":"
       << (direct_wall_edge_profile_ ? "true" : "false")
       << ",\"wall_edge_side\":" << direct_wall_side_
       << ",\"wall_clearance_reserve_m\":" << direct_wall_clearance_reserve_m_
       << ",\"opponent_clearance_reserve_m\":"
       << direct_opponent_clearance_reserve_m_
       << ",\"corridor_clearance_reserve_m\":"
       << direct_corridor_clearance_reserve_m_
       << ",\"desired_control_reserve_m\":" << direct_desired_control_reserve_m_
       << ",\"tracking_reserve_consumption_m\":"
       << wall_edge_speed.tracking_reserve_consumption_m
       << ",\"projected_lateral_error_m\":"
       << wall_edge_speed.projected_lateral_error_m
       << ",\"corridor_lateral_velocity_mps\":" << corridor_lateral_velocity
       << ",\"wall_tracking_reserve_consumption_m\":"
       << wall_edge_speed.wall_tracking_reserve_consumption_m
       << ",\"opponent_tracking_reserve_consumption_m\":"
       << wall_edge_speed.opponent_tracking_reserve_consumption_m
       << ",\"usable_wall_reserve_m\":" << wall_edge_speed.usable_wall_reserve_m
       << ",\"usable_opponent_reserve_m\":"
       << wall_edge_speed.usable_opponent_reserve_m
       << ",\"usable_corridor_reserve_m\":"
       << wall_edge_speed.usable_corridor_reserve_m
       << ",\"wall_edge_speed_gate_active\":"
       << (wall_edge_speed.active ? "true" : "false")
       << ",\"wall_edge_speed_cap_mps\":" << wall_edge_speed.speed_cap_mps
       << ",\"odometry_receive_age_sec\":" << now_sec - odometry_received_sec_
       << ",\"trajectory_receive_age_sec\":"
       << now_sec - trajectory_received_sec_
       << ",\"trajectory_stamp_sec\":" << trajectory_stamp.seconds()
       << ",\"trajectory_age_sec\":" << trajectory_age_sec
       << ",\"trajectory_point_count\":" << trajectory_->points.size()
       << ",\"control_command_stamp_sec\":" << now_sec
       << ",\"current_pose_x\":" << odometry_->pose.pose.position.x
       << ",\"current_pose_y\":" << odometry_->pose.pose.position.y
       << ",\"current_rear_x\":" << current_rear_x
       << ",\"current_rear_y\":" << current_rear_y
       << ",\"current_yaw\":" << current_yaw
       << ",\"nearest_trajectory_index\":" << nearest_index
       << ",\"nearest_trajectory_x\":" << nearest.pose.position.x
       << ",\"nearest_trajectory_y\":" << nearest.pose.position.y
       << ",\"continuous_preview_interpolation_enabled\":"
       << (tuning_.continuous_preview_interpolation_enabled ? "true" : "false")
       << ",\"lookahead_lower_trajectory_index\":" << lookahead.lower_index
       << ",\"lookahead_upper_trajectory_index\":" << lookahead.upper_index
       << ",\"lookahead_endpoint_fallback\":"
       << (lookahead.endpoint_fallback ? "true" : "false")
       << ",\"lookahead_target_x\":" << lookahead.point.pose.position.x
       << ",\"lookahead_target_y\":" << lookahead.point.pose.position.y
       << ",\"lookahead_target_local_x\":" << far_target_local_x
       << ",\"lookahead_target_local_y\":" << far_target_local_y
       << ",\"near_lower_trajectory_index\":" << near_preview.lower_index
       << ",\"near_upper_trajectory_index\":" << near_preview.upper_index
       << ",\"near_endpoint_fallback\":"
       << (near_preview.endpoint_fallback ? "true" : "false")
       << ",\"near_target_x\":" << near_preview.point.pose.position.x
       << ",\"near_target_y\":" << near_preview.point.pose.position.y
       << ",\"near_target_local_x\":" << near_target_local_x
       << ",\"near_target_local_y\":" << near_target_local_y
       << ",\"delay_compensation_enabled\":"
       << (tuning_.delay_compensation_enabled ? "true" : "false")
       << ",\"delay_compensation_used\":"
       << (delay_compensation_used ? "true" : "false")
       << ",\"delay_prediction_requested_steering_rad\":"
       << delay_prediction.requested_target_steering_rad
       << ",\"delay_prediction_bounded_target_steering_rad\":"
       << delay_prediction.bounded_target_steering_rad
       << ",\"delay_prediction_applied_steering_rad\":"
       << delay_prediction.applied_steering_rad
       << ",\"delay_prediction_target_angle_limited\":"
       << (delay_prediction.target_angle_limited ? "true" : "false")
       << ",\"delay_prediction_steering_rate_limited\":"
       << (delay_prediction.steering_rate_limited ? "true" : "false")
       << ",\"steering_status_fresh\":"
       << (steering_status_fresh ? "true" : "false")
       << ",\"measured_steering_angle_rad\":" << measured_steering_angle
       << ",\"control_rear_x\":" << control_rear_x
       << ",\"control_rear_y\":" << control_rear_y
       << ",\"control_yaw\":" << control_yaw
       << ",\"lateral_error_m\":" << lateral_error
       << ",\"longitudinal_velocity_mps\":" << longitudinal_velocity
       << ",\"lateral_velocity_mps\":" << lateral_velocity
       << ",\"slip_angle_rad\":" << slip_angle
       << ",\"slip_angle_valid\":" << (slip_angle_valid ? "true" : "false")
       << ",\"gnss_ground_speed_mps\":" << ground_speed
       << ",\"gnss_ground_track_yaw_rad\":" << ground_track_yaw
       << ",\"measured_yaw_rate_radps\":" << measured_yaw_rate
       << ",\"reference_yaw_rate_radps\":" << reference_yaw_rate
       << ",\"estimated_lateral_acceleration_mps2\":"
       << estimated_lateral_acceleration
       << ",\"lookahead_distance_m\":" << lookahead_distance
       << ",\"raw_lookahead_distance_m\":" << raw_lookahead_distance
       << ",\"lookahead_speed_basis_mps\":" << lookahead_speed_basis_mps
       << ",\"maneuver_lookahead_uses_measured_speed\":"
       << (maneuver_lookahead_uses_measured_speed_ ? "true" : "false")
       << ",\"actual_lookahead_distance_m\":" << actual_lookahead_distance
       << ",\"heading_error_rad\":" << heading_error
       << ",\"steering_lookahead_distance_m\":" << steering_lookahead_distance
       << ",\"near_preview_distance_m\":" << near_preview_distance
       << ",\"actual_near_preview_distance_m\":" << actual_near_preview_distance
       << ",\"near_heading_error_rad\":" << near_heading_error
       << ",\"dual_preview_near_ratio\":" << tuning_.dual_preview_near_ratio
       << ",\"dual_preview_blend\":" << tuning_.dual_preview_blend
       << ",\"effective_dual_preview_blend\":" << effective_dual_preview_blend
       << ",\"exit_unwind_enabled\":"
       << (tuning_.exit_unwind_enabled ? "true" : "false")
       << ",\"exit_unwind_progress\":" << exit_unwind_progress
       << ",\"far_steering_angle_rad\":" << far_steering_angle
       << ",\"near_steering_angle_rad\":" << near_steering_angle
       << ",\"pure_pursuit_steering_angle_rad\":" << pure_pursuit_steering_angle
       << ",\"curvature_feedforward_enabled\":"
       << (tuning_.curvature_feedforward_enabled ? "true" : "false")
       << ",\"curvature_feedforward_maneuver_only\":"
       << (curvature_feedforward_maneuver_only_ ? "true" : "false")
       << ",\"curvature_feedforward_active\":"
       << (curvature_feedforward_active ? "true" : "false")
       << ",\"overtake_mode\":\"" << jsonEscape(overtake_mode_) << "\""
       << ",\"overtake_mode_fresh\":"
       << (overtake_mode_fresh ? "true" : "false")
       << ",\"signed_curvature_1pm\":" << local_signed_curvature
       << ",\"feedforward_preview_curvature_1pm\":" << preview_signed_curvature
       << ",\"feedforward_smoothed_curvature_1pm\":"
       << smoothed_feedforward_curvature_
       << ",\"curvature_feedforward_angle_rad\":" << curvature_feedforward_angle
       << ",\"curvature\":" << smoothed_curvature_
       << ",\"target_velocity_mps\":" << target_velocity
       << ",\"vehicle_command_speed_mps\":" << command.longitudinal.speed
       << ",\"gear_relative_reverse_command_enabled\":"
       << (gear_relative_reverse_command_enabled_ ? "true" : "false")
       << ",\"gear_relative_reverse_conversion_applied\":"
       << (vehicle_longitudinal_command.reverse_conversion_applied ? "true"
                                                                   : "false")
       << ",\"gear_trajectory_direction_mismatch\":"
       << (vehicle_longitudinal_command.direction_mismatch ? "true" : "false")
       << ",\"maximum_velocity_mps\":" << maximum_velocity
       << ",\"curvature_speed_limit_mps\":" << curvature_speed_limit
       << ",\"curvature_target_velocity_mps\":" << curvature_target_velocity
       << ",\"corner_speed_retention\":" << tuning_.corner_speed_retention
       << ",\"lateral_error_speed_gate_enabled\":"
       << (tuning_.lateral_error_speed_gate_enabled ? "true" : "false")
       << ",\"effective_corner_speed_retention\":"
       << effective_corner_speed_retention
       << ",\"corner_speed_retention_gate\":" << retention_gate
       << ",\"requested_acceleration_mps2\":"
       << bounded_acceleration.requested_mps2
       << ",\"commanded_acceleration_mps2\":"
       << command.longitudinal.acceleration << ",\"safe_stop_braking_active\":"
       << (safe_stop_mode ? "true" : "false")
       << ",\"requested_steering_angle_rad\":" << requested_steering_angle
       << ",\"bounded_steering_angle_rad\":"
       << command.lateral.steering_tire_angle
       << ",\"steering_command_to_tire_angle_ratio\":"
       << tuning_.steering_command_to_tire_angle_ratio
       << ",\"target_tire_steering_angle_rad\":" << final_target_tire_angle
       << ",\"maximum_tire_steering_angle_rad\":"
       << tuning_.maximum_tire_steering_angle_rad
       << ",\"steering_command_passthrough_enabled\":"
       << (tuning_.steering_command_passthrough_enabled ? "true" : "false")
       << ",\"steering_control_dt_sec\":" << control_dt_sec
       << ",\"hard_steering_angle_limit_rad\":"
       << tuning_.hard_steering_angle_limit_rad
       << ",\"hard_steering_rate_limit_radps\":"
       << tuning_.hard_steering_rate_limit_radps
       << ",\"steering_angle_limited\":"
       << (bounded_steering.angle_limited ? "true" : "false")
       << ",\"steering_tracking_speed_gate_enabled\":"
       << (tuning_.steering_tracking_speed_gate_enabled ? "true" : "false")
       << ",\"steering_tracking_speed_gate_active\":"
       << (steering_tracking_gate.active ? "true" : "false")
       << ",\"steering_tracking_error_rad\":"
       << steering_tracking_gate.steering_error_rad
       << ",\"steering_acceleration_hold_enabled\":"
       << (tuning_.steering_demand_acceleration_hold_enabled ? "true" : "false")
       << ",\"steering_acceleration_hold_active\":"
       << (steering_acceleration_hold.active ? "true" : "false")
       << ",\"steering_acceleration_hold_tracking_error_rad\":"
       << steering_acceleration_hold.tracking_error_rad
       << ",\"steering_acceleration_hold_maximum_acceleration_mps2\":"
       << tuning_.steering_acceleration_hold_maximum_acceleration_mps2
       << ",\"steering_acceleration_hold_allowed_acceleration_mps2\":"
       << steering_acceleration_hold.allowed_acceleration_mps2
       << ",\"steering_tracking_entry_elapsed_sec\":"
       << steering_tracking_speed_gate_state_.entry_elapsed_sec
       << ",\"steering_tracking_release_elapsed_sec\":"
       << steering_tracking_speed_gate_state_.release_elapsed_sec
       << ",\"measured_yaw_rate_radps\":" << measured_yaw_rate
       << ",\"reference_yaw_rate_radps\":" << reference_yaw_rate
       << ",\"signed_yaw_rate_excess_radps\":" << signed_yaw_rate_excess
       << ",\"filtered_yaw_acceleration_radps2\":"
       << rotation_controller_state_.filtered_yaw_acceleration_radps2
       << ",\"commanded_yaw_rate_radps\":" << commanded_yaw_rate
       << ",\"predicted_inertial_yaw_rate_radps\":"
       << rotation_recovery.predicted_inertial_yaw_rate_radps
       << ",\"predicted_steering_yaw_rate_radps\":"
       << rotation_recovery.predicted_steering_yaw_rate_radps
       << ",\"predicted_yaw_rate_excess_radps\":"
       << rotation_recovery.predicted_signed_yaw_rate_excess_radps
       << ",\"rotation_risk_yaw_rate_excess_radps\":"
       << rotation_recovery.risk_yaw_rate_excess_radps
       << ",\"oversteer_slip_angle_rad\":"
       << rotation_recovery.oversteer_slip_angle_rad
       << ",\"rotation_gate_enabled\":"
       << (tuning_.rotation_gate_enabled ? "true" : "false")
       << ",\"rotation_prediction_enabled\":"
       << (tuning_.rotation_prediction_enabled ? "true" : "false")
       << ",\"rotation_prediction_horizon_sec\":"
       << rotation_prediction_horizon_sec << ",\"rotation_prediction_trigger\":"
       << (rotation_recovery.predictive_trigger ? "true" : "false")
       << ",\"rotation_gate_active\":"
       << (rotation_controller_state_.active ? "true" : "false")
       << ",\"rotation_countersteer_rad\":"
       << rotation_recovery.countersteer_rad
       << ",\"rotation_gate_entry_condition\":"
       << (rotation_result.entry_condition ? "true" : "false")
       << ",\"speed_limited\":" << (target_velocity < maximum_velocity)
       << ",\"steering_rate_limited\":"
       << (bounded_steering.rate_limited ? "true" : "false") << "}";
  debug.data = json.str();
  pub_debug_->publish(debug);
  const bool trace_mode_selected =
      !diagnostic_trace_maneuver_only_ ||
      (overtake_mode_fresh &&
       (isCurvatureFeedforwardManeuverMode(overtake_mode_) ||
        overtake_mode_ == "MERGE_BACK"));
  if (diagnostic_trace_enabled_ && trace_mode_selected &&
      now_sec - last_diagnostic_trace_sec_ >= diagnostic_trace_period_sec_) {
    RCLCPP_INFO(
        get_logger(),
        "[cma_pp_cycle.trace] authority_changed=false stamp=%.9f "
        "trajectory_stamp=%.9f mode=%s mode_fresh=%s points=%zu "
        "current_pose=(%.6f,%.6f,%.6f) current_rear=(%.6f,%.6f) "
        "control_rear=(%.6f,%.6f,%.6f) delay_compensated=%s "
        "nearest=%zu nearest_xy=(%.6f,%.6f) "
        "far_indices=(%zu,%zu) far_endpoint=%s "
        "far_world=(%.6f,%.6f) far_local=(%.6f,%.6f) "
        "near_indices=(%zu,%zu) near_endpoint=%s "
        "near_world=(%.6f,%.6f) near_local=(%.6f,%.6f) "
        "lookahead=(%.6f,%.6f) heading_error=(%.6f,%.6f) "
        "steer_components=(far=%.6f,near=%.6f,pp=%.6f,ff=%.6f) "
        "steer=(requested_cmd=%.6f,bounded_cmd=%.6f,target_tire=%.6f,"
        "measured_tire=%.6f) "
        "recovery=(active=%s,predictive=%s,excess=%.6f,predicted=%.6f,"
        "risk=%.6f,yaw_accel=%.6f,slip=%.6f,counter=%.6f) "
        "limited=(angle=%s,rate=%s) control_dt=%.6f",
        now_sec, trajectory_stamp.seconds(), overtake_mode_.c_str(),
        overtake_mode_fresh ? "true" : "false", trajectory_->points.size(),
        odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
        current_yaw, current_rear_x, current_rear_y, control_rear_x,
        control_rear_y, control_yaw, delay_compensation_used ? "true" : "false",
        nearest_index, nearest.pose.position.x, nearest.pose.position.y,
        lookahead.lower_index, lookahead.upper_index,
        lookahead.endpoint_fallback ? "true" : "false",
        lookahead.point.pose.position.x, lookahead.point.pose.position.y,
        far_target_local_x, far_target_local_y, near_preview.lower_index,
        near_preview.upper_index,
        near_preview.endpoint_fallback ? "true" : "false",
        near_preview.point.pose.position.x, near_preview.point.pose.position.y,
        near_target_local_x, near_target_local_y, lookahead_distance,
        near_preview_distance, heading_error, near_heading_error,
        far_steering_angle, near_steering_angle, pure_pursuit_steering_angle,
        curvature_feedforward_angle, requested_steering_angle,
        command.lateral.steering_tire_angle, final_target_tire_angle,
        measured_steering_angle, rotation_controller_state_.active ? "true" : "false",
        rotation_recovery.predictive_trigger ? "true" : "false",
        signed_yaw_rate_excess,
        rotation_recovery.predicted_signed_yaw_rate_excess_radps,
        rotation_recovery.risk_yaw_rate_excess_radps,
        rotation_controller_state_.filtered_yaw_acceleration_radps2,
        rotation_recovery.oversteer_slip_angle_rad,
        rotation_recovery.countersteer_rad,
        bounded_steering.angle_limited ? "true" : "false",
        bounded_steering.rate_limited ? "true" : "false", control_dt_sec);
    last_diagnostic_trace_sec_ = now_sec;
  }
  previous_steering_ = command.lateral.steering_tire_angle;
  steering_limiter_initialized_ = bounded_steering.valid;
  last_bounded_steering_rad_ = bounded_steering.bounded_steering_rad;
  last_control_publish_sec_ = now_sec;
  const TimingPoint diagnostic_end = timing_now();
  if (runtime_timing_metrics_enabled_) {
    timing_sample.diagnostic_ms = elapsed_ms(publish_end, diagnostic_end);
  }
  finalize_timing();
}

} // namespace simple_pure_pursuit

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
