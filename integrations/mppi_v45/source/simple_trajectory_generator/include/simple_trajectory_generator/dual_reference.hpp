#ifndef SIMPLE_TRAJECTORY_GENERATOR__DUAL_REFERENCE_HPP_
#define SIMPLE_TRAJECTORY_GENERATOR__DUAL_REFERENCE_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace simple_trajectory_generator {

struct CircularReferenceConfig {
  double execution_speed_mps{10.0};
  double maximum_arc_spacing_m{0.25};
  std::size_t maximum_output_points{10000U};
};

struct CircularReference {
  autoware_auto_planning_msgs::msg::Trajectory trajectory{};
  std::vector<double> cumulative_s_m{};
  double length_m{0.0};
};

struct CircularReferenceResult {
  bool valid{false};
  std::string reason{};
  CircularReference reference{};
};

struct PathProjection {
  bool valid{false};
  std::size_t nearest_index{0U};
  double s_m{0.0};
  double distance_m{0.0};
};

CircularReferenceResult buildCircularReference(
    const autoware_auto_planning_msgs::msg::Trajectory &source,
    const CircularReferenceConfig &config, double anchor_x_m,
    double anchor_y_m, bool rotate_to_anchor);

PathProjection projectToCircularReference(const CircularReference &reference,
                                          double x_m, double y_m);

autoware_auto_planning_msgs::msg::Trajectory buildLocalCircularTrajectory(
    const CircularReference &reference, std::size_t nearest_index,
    double distance_behind_m, double forward_distance_m,
    std::size_t maximum_output_points = 1000U);

struct LapProgressConfig {
  double path_length_m{0.0};
  double start_window_m{5.0};
  double end_window_m{8.0};
  double minimum_coverage_ratio{0.95};
  double maximum_forward_step_m{5.0};
  double reverse_jitter_tolerance_m{0.5};
};

struct LapProgressUpdate {
  bool accepted{false};
  bool start_armed{false};
  bool seam_armed{false};
  bool seam_crossed{false};
  bool lap_complete{false};
  double accumulated_forward_m{0.0};
  std::string reason{};
};

class LapProgressTracker {
public:
  explicit LapProgressTracker(LapProgressConfig config);

  LapProgressUpdate update(double s_m, double measurement_time_sec);
  void reset();

private:
  LapProgressConfig config_;
  bool initialized_{false};
  bool start_armed_{false};
  bool seam_armed_{false};
  bool lap_complete_{false};
  double last_s_m_{0.0};
  double last_measurement_time_sec_{0.0};
  double accumulated_forward_m_{0.0};
};

} // namespace simple_trajectory_generator

#endif // SIMPLE_TRAJECTORY_GENERATOR__DUAL_REFERENCE_HPP_
