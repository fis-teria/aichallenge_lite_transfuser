#pragma once

#include <cstdint>
#include <optional>

namespace reference_space_mppi_planner {

enum class DrivingMode { FREE_RUN, AVOID, OVERTAKE };

const char *drivingModeName(DrivingMode mode) noexcept;

struct FollowObservation {
  double body_gap_m{0.0};
  double speed_mps{0.0};
  std::optional<double> observed_speed_mps{};
};

struct DrivingScene {
  bool reference_valid{false};
  bool reference_obstructed{false};
  bool reference_aligned{false};
  std::optional<double> leading_body_gap_m{};
  std::optional<double> leading_speed_mps{};
  bool preparation_search_due{false};
  bool overtake_target_valid{false};
  bool preparation_available{false};
  // Collection only: same fresh, still path-conflicting adopted target.
  bool continue_collection_avoidance{false};
};

struct DrivingPlan {
  bool generate_lateral{false};
  bool generate_return{false};
  bool finish_avoid{false};
  DrivingMode maneuver{DrivingMode::FREE_RUN};
};

// No ROS, optimizer, vehicle identity, or candidate geometry is owned here.
// Plans are advisory; mode changes are committed with the adopted output.
class DrivingFsm {
public:
  explicit DrivingFsm(double follow_distance_m = 5.0, double overtake_follow_distance_m = 2.0);
  DrivingMode mode() const noexcept { return mode_; }
  std::uint64_t revision() const noexcept { return revision_; }
  const char *modeName() const noexcept;
  DrivingPlan plan(const DrivingScene &scene) const noexcept;
  double freeRunSpeed(double cruise_speed_mps,
                      const std::optional<FollowObservation> &leading) const noexcept;
  void acceptLateral(DrivingMode maneuver = DrivingMode::AVOID) noexcept;
  void finishAvoid() noexcept;
  void reset() noexcept;

private:
  double follow_distance_m_;
  double overtake_follow_distance_m_;
  DrivingMode mode_{DrivingMode::FREE_RUN};
  std::uint64_t revision_{0};
};

}  // namespace reference_space_mppi_planner
