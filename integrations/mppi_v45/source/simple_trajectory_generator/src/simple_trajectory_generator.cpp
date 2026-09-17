// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_trajectory_generator/dual_reference.hpp"
#include "simple_trajectory_generator/execution_profile.hpp"
#include "simple_trajectory_generator/race_reference.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class CSVToTrajectory : public rclcpp::Node {
public:
  CSVToTrajectory() : Node("csv_to_trajectory_node") {
    const auto local_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    const auto full_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                              .reliable()
                              .transient_local();
    pub_ = create_publisher<Trajectory>("trajectory", local_qos);
    cma_trajectory_pub_ =
        create_publisher<Trajectory>("cma_trajectory", local_qos);
    full_reference_pub_ = create_publisher<Trajectory>(
        "reference/full_trajectory", full_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "reference/status", rclcpp::QoS(10).reliable());
    leader_pub_ = create_publisher<std_msgs::msg::String>(
        "/planning/reference/leader_vehicle_id", 10);

    declare_parameter("csv_path", "");
    simulation_ = declare_parameter<bool>("simulation", true);
    dual_reference_enabled_ =
        declare_parameter<bool>("dual_reference.enabled", false);
    const std::string lap1_csv_path =
        declare_parameter<std::string>("dual_reference.lap1_csv_path", "");
    const std::string lap2plus_csv_path = declare_parameter<std::string>(
        "dual_reference.lap2plus_csv_path", "");
    race_reference_enabled_ = declare_parameter<bool>("race_reference.enabled", false);
    const std::string leader_csv_path =
        declare_parameter<std::string>("race_reference.leader_csv_path", "");
    race_own_vehicle_id_ =
        declare_parameter<std::string>("race_reference.own_vehicle_id", "");
    z_ = declare_parameter<float>("z", 0.0F);
    execution_profile_config_.max_speed_mps =
        declare_parameter<double>("execution_profile.max_speed_mps", 10.0);
    execution_profile_config_.max_arc_spacing_m =
        declare_parameter<double>("execution_profile.max_arc_spacing_m", 0.25);
    execution_profile_config_.max_yaw_step_rad =
        declare_parameter<double>("execution_profile.max_yaw_step_rad", 0.05);
    publish_rate_hz_ =
        declare_parameter<double>("dual_reference.publish_rate_hz", 50.0);
    status_rate_hz_ =
        declare_parameter<double>("dual_reference.status_rate_hz", 10.0);
    local_distance_behind_m_ = declare_parameter<double>(
        "dual_reference.local_distance_behind_m", 2.0);
    local_forward_distance_m_ = declare_parameter<double>(
        "dual_reference.local_forward_distance_m", 40.0);
    maximum_projection_distance_m_ = declare_parameter<double>(
        "dual_reference.maximum_projection_distance_m", 5.0);
    source_timeout_sec_ =
        declare_parameter<double>("dual_reference.source_timeout_sec", 0.5);
    defer_switch_during_maneuver_ = declare_parameter<bool>(
        "dual_reference.defer_switch_during_maneuver", true);
    const double start_window_m = declare_parameter<double>(
        "dual_reference.start_window_m", 5.0);
    const double end_window_m =
        declare_parameter<double>("dual_reference.end_window_m", 8.0);
    const double minimum_coverage_ratio = declare_parameter<double>(
        "dual_reference.minimum_coverage_ratio", 0.95);
    const double maximum_forward_step_m = declare_parameter<double>(
        "dual_reference.maximum_forward_step_m", 5.0);
    const double reverse_jitter_tolerance_m = declare_parameter<double>(
        "dual_reference.reverse_jitter_tolerance_m", 0.5);

    validateRuntimeConfiguration();
    if (race_reference_enabled_) {
      if (dual_reference_enabled_) {
        throw std::invalid_argument("race and MPC dual reference are mutually exclusive");
      }
      initializeRaceReference(lap1_csv_path, lap2plus_csv_path, leader_csv_path,
          {0.0, start_window_m, end_window_m, minimum_coverage_ratio,
           maximum_forward_step_m, reverse_jitter_tolerance_m});
    } else if (dual_reference_enabled_) {
      initializeDualReference(lap1_csv_path, lap2plus_csv_path, start_window_m,
                              end_window_m, minimum_coverage_ratio,
                              maximum_forward_step_m,
                              reverse_jitter_tolerance_m);
    } else {
      const std::string csv_path = get_parameter("csv_path").as_string();
      if (csv_path.empty()) {
        throw std::invalid_argument("csv_path is not specified");
      }
      if (!loadLegacyTrajectory(csv_path)) {
        throw std::runtime_error("failed to load CSV trajectory: " + csv_path);
      }
      current_csv_path_ = csv_path;
      timer_ = rclcpp::create_timer(
          this, get_clock(), std::chrono::seconds(1),
          std::bind(&CSVToTrajectory::publishLegacyTrajectory, this));
    }

    set_parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
        &CSVToTrajectory::onParameterEvent, this, std::placeholders::_1));
  }

private:
  enum class CSVFormat { kPoseWithQuaternion, kReferencePath };

  static geometry_msgs::msg::Quaternion
  createQuaternionFromYaw(const double yaw_rad) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.z = std::sin(yaw_rad * 0.5);
    quaternion.w = std::cos(yaw_rad * 0.5);
    return quaternion;
  }

  static std::vector<std::string> splitCSVLine(const std::string &line) {
    std::stringstream stream(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(stream, token, ',')) {
      if (!token.empty() && token.back() == '\r') {
        token.pop_back();
      }
      tokens.push_back(token);
    }
    return tokens;
  }

  static std::vector<double> parseCSVValues(const std::string &line) {
    const auto tokens = splitCSVLine(line);
    std::vector<double> values;
    values.reserve(tokens.size());
    for (const auto &token : tokens) {
      values.push_back(std::stod(token));
    }
    return values;
  }

  static bool isBlankLine(const std::string &line) {
    return line.find_first_not_of(" \t\r\n") == std::string::npos;
  }

  static CSVFormat detectCSVFormat(const std::string &header_line) {
    const auto header = splitCSVLine(header_line);
    if (header.size() >= 7U && header[0] == "s_m" && header[1] == "x_m" &&
        header[2] == "y_m" && header[3] == "psi_rad" &&
        header[5] == "vx_mps") {
      return CSVFormat::kReferencePath;
    }
    return CSVFormat::kPoseWithQuaternion;
  }

  Trajectory loadCSVSource(const std::string &csv_path) const {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      throw std::runtime_error("cannot open CSV: " + csv_path);
    }
    std::string line;
    if (!std::getline(file, line)) {
      throw std::runtime_error("CSV is empty: " + csv_path);
    }
    const CSVFormat csv_format = detectCSVFormat(line);
    Trajectory source;
    source.header.frame_id = "map";
    std::size_t line_number = 1U;
    while (std::getline(file, line)) {
      ++line_number;
      if (isBlankLine(line)) {
        continue;
      }
      std::vector<double> values;
      try {
        values = parseCSVValues(line);
      } catch (const std::exception &exception) {
        throw std::runtime_error("invalid CSV numeric value at line " +
                                 std::to_string(line_number) + ": " +
                                 exception.what());
      }
      if (csv_format == CSVFormat::kPoseWithQuaternion && values.size() != 8U) {
        throw std::runtime_error("invalid pose CSV column count at line " +
                                 std::to_string(line_number));
      }
      if (csv_format == CSVFormat::kReferencePath && values.size() < 7U) {
        throw std::runtime_error(
            "invalid reference CSV column count at line " +
            std::to_string(line_number));
      }

      TrajectoryPoint point;
      point.pose.position.z = z_;
      if (csv_format == CSVFormat::kReferencePath) {
        point.pose.position.x = values[1];
        point.pose.position.y = values[2];
        point.pose.orientation = createQuaternionFromYaw(values[3]);
        point.longitudinal_velocity_mps = static_cast<float>(values[5]);
        point.acceleration_mps2 = static_cast<float>(values[6]);
      } else {
        point.pose.position.x = values[0];
        point.pose.position.y = values[1];
        point.pose.orientation.x = values[3];
        point.pose.orientation.y = values[4];
        point.pose.orientation.z = values[5];
        point.pose.orientation.w = values[6];
        point.longitudinal_velocity_mps = static_cast<float>(values[7]);
      }
      point.lateral_velocity_mps = 0.0F;
      point.heading_rate_rps = 0.0F;
      source.points.push_back(point);
    }
    if (source.points.size() < 2U) {
      throw std::runtime_error("CSV has fewer than two trajectory points: " +
                               csv_path);
    }
    return source;
  }

  bool loadLegacyTrajectory(const std::string &csv_path) {
    try {
      auto source = loadCSVSource(csv_path);
      source.header.stamp = now();
      const auto profile = simple_trajectory_generator::buildExecutionProfile(
          source, execution_profile_config_);
      if (!profile.valid) {
        RCLCPP_ERROR(get_logger(), "Execution profile rejected CSV: %s",
                     profile.reason.c_str());
        return false;
      }
      csv_trajectory_ = profile.trajectory;
      RCLCPP_INFO(get_logger(),
                  "Built legacy execution profile: source_points=%zu "
                  "output_points=%zu speed=%.3f m/s",
                  source.points.size(), csv_trajectory_.points.size(),
                  profile.execution_speed_mps);
      return true;
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "Failed to load CSV: %s", exception.what());
      return false;
    }
  }

  void initializeRaceReference(const std::string &lap1_path,
                               const std::string &normal_path,
                               const std::string &leader_path,
                               simple_trajectory_generator::LapProgressConfig lap_config) {
    race_csv_paths_ = {lap1_path, normal_path, leader_path};
    for (std::size_t i = 0; i < race_csv_paths_.size(); ++i) {
      const auto source = loadCSVSource(race_csv_paths_[i]);
      const auto result = simple_trajectory_generator::buildExecutionProfile(
          source, execution_profile_config_);
      if (!result.valid) throw std::runtime_error("race reference rejected: " + result.reason);
      race_trajectories_[i] = result.trajectory;
      if (i == 0U) {
        const auto &anchor = source.points.front().pose.position;
        const auto course = simple_trajectory_generator::buildCircularReference(
            source, {execution_profile_config_.max_speed_mps,
                     execution_profile_config_.max_arc_spacing_m,
                     execution_profile_config_.max_output_points},
            anchor.x, anchor.y, false);
        if (!course.valid) throw std::runtime_error("race projection course rejected: " + course.reason);
        lap1_reference_ = course.reference;
      }
    }
    race_ranking_ = std::make_unique<simple_trajectory_generator::RaceProgressRanking>(
        lap1_reference_.length_m);
    csv_trajectory_ = race_trajectories_[0];
    current_csv_path_ = race_csv_paths_[0];
    if (!simulation_) {
      lap_config.path_length_m = lap1_reference_.length_m;
      lap_progress_tracker_ =
          std::make_unique<simple_trajectory_generator::LapProgressTracker>(lap_config);
    }
    odometry_sub_ = create_subscription<Odometry>(
        "input/kinematics", rclcpp::SensorDataQoS(),
        [this](Odometry::SharedPtr message) {
          odometry_ = message;
          const auto &p = message->pose.pose.position;
          const auto projection = simple_trajectory_generator::projectToCircularReference(
              lap1_reference_, p.x, p.y);
          if (projection.valid && projection.distance_m <= maximum_projection_distance_m_) {
            if (!race_anchor_s_m_) race_anchor_s_m_ = projection.s_m;
            race_ranking_->update("__ego", projection.s_m,
                rclcpp::Time(message->header.stamp).seconds(), *race_anchor_s_m_);
            if (lap_progress_tracker_) {
              const auto progress = lap_progress_tracker_->update(
                  projection.s_m, rclcpp::Time(message->header.stamp).seconds());
              if (progress.seam_crossed) ++race_current_lap_;
            }
          }
          updateRaceReference();
        });
    if (simulation_) {
      race_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "input/race_status", rclcpp::QoS(10),
        [this](std_msgs::msg::Float32MultiArray::SharedPtr message) {
          if (message->data.size() < 2U || !std::isfinite(message->data[1]) ||
              message->data[1] < 0.0F ||
              message->data[1] >= static_cast<float>(std::numeric_limits<int>::max())) return;
          const int lap = std::max(1, static_cast<int>(message->data[1]));
          if (lap < race_current_lap_) {
            race_ranking_->reset();
            race_anchor_s_m_.reset();
            race_v2x_received_sec_.reset();
          }
          race_current_lap_ = lap;
          updateRaceReference();
        });
    }
    race_vehicles_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        "input/vehicle_positions", rclcpp::QoS(10).reliable(),
        [this](v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr message) {
          if (!race_anchor_s_m_) return;
          const double receive_sec = now().seconds();
          race_v2x_received_sec_ = receive_sec;
          // AWSIM's vehicle-domain V2X excludes ego; its progress comes from
          // odometry. Keep the explicit ID exclusion for other publishers.
          for (const auto &vehicle : message->vehicles) {
            if (vehicle.vehicle_id == race_own_vehicle_id_) continue;
            const auto projection = simple_trajectory_generator::projectToCircularReference(
                lap1_reference_, vehicle.position.x, vehicle.position.y);
            if (!projection.valid || projection.distance_m > maximum_projection_distance_m_) continue;
            const double stamp = rclcpp::Time(vehicle.header.stamp).seconds();
            race_ranking_->update(vehicle.vehicle_id, projection.s_m,
                stamp > 0.0 ? stamp : receive_sec, *race_anchor_s_m_);
          }
          updateRaceReference();
        });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1),
        std::bind(&CSVToTrajectory::publishLegacyTrajectory, this));
    status_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100),
        [this]() { updateRaceReference(); publishRaceStatus(); });
  }

  void updateRaceReference() {
    const double now_sec = now().seconds();
    race_rank_.reset();
    if (race_v2x_received_sec_ && now_sec >= *race_v2x_received_sec_ &&
        now_sec - *race_v2x_received_sec_ <= source_timeout_sec_) {
      race_rank_ = race_ranking_->rank("__ego", now_sec, source_timeout_sec_);
    }
    const auto selected = simple_trajectory_generator::selectRaceReference(
        race_current_lap_, race_rank_);
    const auto index = static_cast<std::size_t>(selected);
    if (index == race_active_index_) return;
    race_active_index_ = index;
    csv_trajectory_ = race_trajectories_[index];
    current_csv_path_ = race_csv_paths_[index];
    ++reference_generation_;
    publishLegacyTrajectory();
    publishRaceStatus();
    RCLCPP_INFO(get_logger(), "[reference.switch] lap=%d estimated_rank=%d csv=%s generation=%lu",
        race_current_lap_, race_rank_.value_or(0), current_csv_path_.c_str(),
        static_cast<unsigned long>(reference_generation_));
  }

  void publishRaceStatus() {
    std_msgs::msg::String leader;
    if (race_rank_) leader.data = race_ranking_->leader(
        now().seconds(), source_timeout_sec_).value_or("");
    leader_pub_->publish(leader);
    static const std::array<const char *, 3> names{
        "IN_CORCE_LAP1", "CMA_NORMAL_LAP2PLUS", "CMA_LEADER_LAP2PLUS"};
    std::ostringstream stream;
    stream << "{\"active_reference_id\":\"" << names[race_active_index_]
           << "\",\"generation\":" << reference_generation_
           << ",\"current_lap\":" << race_current_lap_
           << ",\"lap_source\":\"" << (simulation_ ? "awsim_status" : "odometry_progress") << "\""
           << ",\"estimated_rank\":" << (race_rank_ ? std::to_string(*race_rank_) : "null")
           << ",\"rank_source\":\"v2x_progress\"}";
    std_msgs::msg::String status;
    status.data = stream.str();
    status_pub_->publish(status);
  }

  void initializeDualReference(
      const std::string &lap1_csv_path,
      const std::string &lap2plus_csv_path, const double start_window_m,
      const double end_window_m, const double minimum_coverage_ratio,
      const double maximum_forward_step_m,
      const double reverse_jitter_tolerance_m) {
    if (lap1_csv_path.empty() || lap2plus_csv_path.empty()) {
      throw std::invalid_argument(
          "dual reference requires lap1 and lap2plus CSV paths");
    }
    const auto lap1_source = loadCSVSource(lap1_csv_path);
    const auto lap2_source = loadCSVSource(lap2plus_csv_path);
    const auto &lap1_anchor = lap1_source.points.front().pose.position;
    const simple_trajectory_generator::CircularReferenceConfig config{
        execution_profile_config_.max_speed_mps,
        execution_profile_config_.max_arc_spacing_m,
        execution_profile_config_.max_output_points};
    const auto lap1_result = simple_trajectory_generator::buildCircularReference(
        lap1_source, config, lap1_anchor.x, lap1_anchor.y, false);
    if (!lap1_result.valid) {
      throw std::runtime_error("lap1 reference rejected: " +
                               lap1_result.reason);
    }
    const auto lap2_result = simple_trajectory_generator::buildCircularReference(
        lap2_source, config, lap1_anchor.x, lap1_anchor.y, true);
    if (!lap2_result.valid) {
      throw std::runtime_error("lap2plus reference rejected: " +
                               lap2_result.reason);
    }
    lap1_reference_ = lap1_result.reference;
    lap2plus_reference_ = lap2_result.reference;
    active_reference_ = &lap1_reference_;
    lap_progress_tracker_ =
        std::make_unique<simple_trajectory_generator::LapProgressTracker>(
            simple_trajectory_generator::LapProgressConfig{
                lap1_reference_.length_m,
                start_window_m,
                end_window_m,
                minimum_coverage_ratio,
                maximum_forward_step_m,
                reverse_jitter_tolerance_m});

    const auto input_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    odometry_sub_ = create_subscription<Odometry>(
        "input/kinematics", input_qos,
        std::bind(&CSVToTrajectory::onOdometry, this, std::placeholders::_1));
    source_sub_ = create_subscription<std_msgs::msg::String>(
        "input/path_source", rclcpp::QoS(10),
        std::bind(&CSVToTrajectory::onPathSource, this,
                  std::placeholders::_1));

    const auto local_period =
        std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(local_period),
        std::bind(&CSVToTrajectory::publishLocalReference, this));
    const auto status_period =
        std::chrono::duration<double>(1.0 / status_rate_hz_);
    status_timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
        std::bind(&CSVToTrajectory::publishStatus, this));

    publishFullReference();
    RCLCPP_INFO(
        get_logger(),
        "Dual reference ready: lap1=in_corce points=%zu length=%.3f m; "
        "lap2plus=cma_es points=%zu length=%.3f m; local_rate=%.1f Hz",
        lap1_reference_.trajectory.points.size(), lap1_reference_.length_m,
        lap2plus_reference_.trajectory.points.size(),
        lap2plus_reference_.length_m, publish_rate_hz_);
  }

  void validateRuntimeConfiguration() const {
    const auto positive = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
    if (!positive(publish_rate_hz_) || !positive(status_rate_hz_) ||
        !positive(local_forward_distance_m_) ||
        !positive(maximum_projection_distance_m_) ||
        !positive(source_timeout_sec_) ||
        !std::isfinite(local_distance_behind_m_) ||
        local_distance_behind_m_ < 0.0) {
      throw std::invalid_argument("invalid dual reference runtime parameter");
    }
  }

  void onOdometry(const Odometry::SharedPtr message) {
    odometry_ = message;
    if (!dual_reference_enabled_ || active_reference_ == nullptr) {
      return;
    }
    active_projection_ =
        simple_trajectory_generator::projectToCircularReference(
            *active_reference_, message->pose.pose.position.x,
            message->pose.pose.position.y);
    lap1_projection_ =
        simple_trajectory_generator::projectToCircularReference(
            lap1_reference_, message->pose.pose.position.x,
            message->pose.pose.position.y);
    if (active_reference_id_ != "IN_CORCE_LAP1" ||
        lap_progress_tracker_ == nullptr) {
      return;
    }
    if (!lap1_projection_.valid ||
        lap1_projection_.distance_m > maximum_projection_distance_m_) {
      last_progress_reason_ = "projection_out_of_range";
      return;
    }
    const rclcpp::Time measurement_stamp(message->header.stamp);
    const double measurement_time_sec =
        measurement_stamp.nanoseconds() > 0
            ? measurement_stamp.seconds()
            : get_clock()->now().seconds();
    last_progress_update_ = lap_progress_tracker_->update(
        lap1_projection_.s_m, measurement_time_sec);
    last_progress_reason_ = last_progress_update_.reason;
    if (last_progress_update_.lap_complete) {
      switch_pending_ = true;
    }
    // Reference authority may change only on the forward end-to-zero seam.
    // If a maneuver owns the vehicle at this seam, keep the request pending
    // and retry at the next seam instead of switching mid-lap when the source
    // later becomes baseline-ready.
    if (last_progress_update_.seam_crossed) {
      tryActivateLap2Reference();
    }
  }

  void onPathSource(const std_msgs::msg::String::SharedPtr message) {
    path_source_ = message->data;
    path_source_received_at_ = get_clock()->now();
  }

  bool baselineSourceReady() {
    if (!defer_switch_during_maneuver_) {
      return true;
    }
    if (path_source_received_at_.nanoseconds() <= 0) {
      return false;
    }
    const double age_sec =
        (get_clock()->now() - path_source_received_at_).seconds();
    if (age_sec < 0.0 || age_sec > source_timeout_sec_) {
      return false;
    }
    static const std::unordered_set<std::string> baseline_sources{
        "CMA_REFERENCE_FREE_RUN", "CMA_REFERENCE_CURVE"};
    return baseline_sources.count(path_source_) != 0U;
  }

  void tryActivateLap2Reference() {
    if (!switch_pending_ || active_reference_id_ != "IN_CORCE_LAP1") {
      return;
    }
    if (!baselineSourceReady()) {
      switch_pending_reason_ = "maneuver_or_source_not_ready";
      return;
    }
    active_reference_ = &lap2plus_reference_;
    active_reference_id_ = "CMA_ES_LAP2PLUS";
    ++reference_generation_;
    switch_pending_ = false;
    switch_pending_reason_ = "none";
    if (odometry_) {
      active_projection_ =
          simple_trajectory_generator::projectToCircularReference(
              *active_reference_, odometry_->pose.pose.position.x,
              odometry_->pose.pose.position.y);
    }
    publishFullReference();
    RCLCPP_INFO(get_logger(),
                "[reference.switch] active=CMA_ES_LAP2PLUS generation=%lu "
                "path_source=%s",
                static_cast<unsigned long>(reference_generation_),
                path_source_.c_str());
  }

  void publishFullReference() {
    if (active_reference_ == nullptr) {
      return;
    }
    Trajectory full = active_reference_->trajectory;
    full.header.stamp = now();
    full_reference_pub_->publish(full);
  }

  void publishLocalReference() {
    if (!dual_reference_enabled_) {
      return;
    }
    if (active_reference_ == nullptr || !active_projection_.valid ||
        active_projection_.distance_m > maximum_projection_distance_m_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Cannot publish local reference: pose projection is unavailable or "
          "out of range");
      return;
    }
    Trajectory cma_local =
        simple_trajectory_generator::buildLocalCircularTrajectory(
            *active_reference_, active_projection_.nearest_index,
            local_distance_behind_m_, local_forward_distance_m_);
    if (cma_local.points.size() < 2U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Cannot publish CMA local reference: too few points");
      return;
    }
    cma_local.header.stamp = now();
    cma_trajectory_pub_->publish(cma_local);

    // Keep the established planning topic on the fixed MPC safety reference.
    // Autostart and RViz consume this topic, while the lap-switched execution
    // reference is isolated on cma_trajectory above.
    if (!lap1_projection_.valid ||
        lap1_projection_.distance_m > maximum_projection_distance_m_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Cannot publish fixed planning reference: projection out of range");
      return;
    }
    Trajectory planning_local =
        simple_trajectory_generator::buildLocalCircularTrajectory(
            lap1_reference_, lap1_projection_.nearest_index,
            local_distance_behind_m_, local_forward_distance_m_);
    if (planning_local.points.size() < 2U) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Cannot publish fixed planning reference: too few points");
      return;
    }
    planning_local.header.stamp = cma_local.header.stamp;
    pub_->publish(planning_local);
  }

  void publishLegacyTrajectory() {
    if (csv_trajectory_.points.empty()) {
      return;
    }
    csv_trajectory_.header.stamp = now();
    pub_->publish(csv_trajectory_);
  }

  void publishStatus() {
    if (!dual_reference_enabled_) {
      return;
    }
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "{\"active_reference_id\":\"" << active_reference_id_
           << "\",\"planning_reference_id\":\"IN_CORCE_FIXED"
           << "\",\"generation\":" << reference_generation_
           << ",\"switch_pending\":"
           << (switch_pending_ ? "true" : "false")
           << ",\"switch_pending_reason\":\"" << switch_pending_reason_
           << "\",\"path_source\":\"" << path_source_
           << "\",\"projection_valid\":"
           << (active_projection_.valid ? "true" : "false")
           << ",\"nearest_index\":" << active_projection_.nearest_index
           << ",\"s_m\":" << active_projection_.s_m
           << ",\"projection_distance_m\":"
           << active_projection_.distance_m
           << ",\"lap_start_armed\":"
           << (last_progress_update_.start_armed ? "true" : "false")
           << ",\"lap_seam_armed\":"
           << (last_progress_update_.seam_armed ? "true" : "false")
           << ",\"lap_complete\":"
           << (last_progress_update_.lap_complete ? "true" : "false")
           << ",\"accumulated_forward_m\":"
           << last_progress_update_.accumulated_forward_m
           << ",\"progress_reason\":\"" << last_progress_reason_ << "\"}";
    status.data = stream.str();
    status_pub_->publish(status);
  }

  rcl_interfaces::msg::SetParametersResult onParameterEvent(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "z") {
        if (dual_reference_enabled_ || race_reference_enabled_) {
          result.successful = false;
          result.reason = "reference geometry is immutable after startup";
        } else {
          z_ = static_cast<float>(parameter.as_double());
        }
      } else if (parameter.get_name() == "csv_path") {
        if (dual_reference_enabled_ || race_reference_enabled_) {
          result.successful = false;
          result.reason = "reference CSV paths are immutable after startup";
          continue;
        }
        const std::string csv_path = parameter.as_string();
        if (!std::filesystem::exists(csv_path) ||
            !loadLegacyTrajectory(csv_path)) {
          result.successful = false;
          result.reason = "failed to load CSV file";
        } else {
          current_csv_path_ = csv_path;
        }
      } else if (parameter.get_name() == "simulation" ||
                 parameter.get_name().rfind("execution_profile.", 0U) == 0U ||
                 parameter.get_name().rfind("race_reference.", 0U) == 0U ||
                 parameter.get_name().rfind("dual_reference.", 0U) == 0U) {
        result.successful = false;
        result.reason = "reference parameters are immutable after startup";
      }
    }
    return result;
  }

  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr cma_trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr full_reference_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leader_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr race_status_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr race_vehicles_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  bool dual_reference_enabled_{false};
  bool simulation_{true};
  bool race_reference_enabled_{false};
  std::array<Trajectory, 3> race_trajectories_;
  std::array<std::string, 3> race_csv_paths_;
  std::string race_own_vehicle_id_;
  std::unique_ptr<simple_trajectory_generator::RaceProgressRanking> race_ranking_;
  std::optional<double> race_anchor_s_m_;
  std::optional<double> race_v2x_received_sec_;
  std::optional<int> race_rank_;
  int race_current_lap_{1};
  std::size_t race_active_index_{0};
  bool defer_switch_during_maneuver_{true};
  double publish_rate_hz_{50.0};
  double status_rate_hz_{10.0};
  double local_distance_behind_m_{2.0};
  double local_forward_distance_m_{40.0};
  double maximum_projection_distance_m_{5.0};
  double source_timeout_sec_{0.5};
  float z_{0.0F};
  simple_trajectory_generator::ExecutionProfileConfig execution_profile_config_;

  Trajectory csv_trajectory_;
  std::string current_csv_path_;
  simple_trajectory_generator::CircularReference lap1_reference_;
  simple_trajectory_generator::CircularReference lap2plus_reference_;
  const simple_trajectory_generator::CircularReference *active_reference_{
      nullptr};
  std::unique_ptr<simple_trajectory_generator::LapProgressTracker>
      lap_progress_tracker_;
  simple_trajectory_generator::PathProjection active_projection_;
  simple_trajectory_generator::PathProjection lap1_projection_;
  simple_trajectory_generator::LapProgressUpdate last_progress_update_;
  Odometry::SharedPtr odometry_;
  std::string active_reference_id_{"IN_CORCE_LAP1"};
  std::uint64_t reference_generation_{1U};
  bool switch_pending_{false};
  std::string switch_pending_reason_{"none"};
  std::string last_progress_reason_{"not_started"};
  std::string path_source_{"UNKNOWN"};
  rclcpp::Time path_source_received_at_;
  OnSetParametersCallbackHandle::SharedPtr set_parameter_callback_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CSVToTrajectory>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("simple_trajectory_generator"), "%s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
