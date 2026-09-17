#include "reference_space_mppi_planner/driving_fsm.hpp"
#include "reference_space_mppi_planner/maneuver_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reference_space_mppi_planner {

DrivingFsm::DrivingFsm(double follow_distance_m, double overtake_follow_distance_m)
    : follow_distance_m_(follow_distance_m), overtake_follow_distance_m_(overtake_follow_distance_m) {
  if (!std::isfinite(follow_distance_m) || follow_distance_m <= 0.0 ||
      !std::isfinite(overtake_follow_distance_m) || overtake_follow_distance_m <= 0.0) {
    throw std::invalid_argument("follow distance must be positive and finite");
  }
}

const char *drivingModeName(DrivingMode mode) noexcept {
  switch(mode) {
    case DrivingMode::AVOID: return "AVOID";
    case DrivingMode::OVERTAKE: return "OVERTAKE";
    default: return "FREE_RUN";
  }
}

const char *DrivingFsm::modeName() const noexcept { return drivingModeName(mode_); }

DrivingPlan DrivingFsm::plan(const DrivingScene &scene) const noexcept {
  const bool slow_leader=scene.leading_speed_mps && isAvoidanceSpeed(*scene.leading_speed_mps);
  // An adopted overtake keeps its mode through continuation or return until release.
  const bool avoid=mode_!=DrivingMode::OVERTAKE &&
      scene.reference_obstructed && scene.leading_speed_mps && scene.leading_body_gap_m &&
      isAvoidanceEligible(*scene.leading_speed_mps,*scene.leading_body_gap_m,follow_distance_m_);
  if(avoid) return {true,false,false,DrivingMode::AVOID};
  const bool overtake=scene.overtake_target_valid && scene.preparation_available &&
      (!scene.reference_obstructed || !slow_leader) &&
      (scene.preparation_search_due || mode_==DrivingMode::OVERTAKE);
  if(overtake) return {true,false,false,DrivingMode::OVERTAKE};
  if(mode_==DrivingMode::FREE_RUN) return {};
  // Eligibility is checked on every snapshot, including an active maneuver.
  // Returning uses a separately validated path even when a distant car still
  // intersects the reference; that car cannot keep avoidance searching alive.
  const bool finished=scene.reference_valid && scene.reference_aligned;
  return {false,!finished,finished,mode_};
}

double DrivingFsm::freeRunSpeed(
    double cruise_speed_mps, const std::optional<FollowObservation> &leading) const noexcept {
  const double cruise = std::isfinite(cruise_speed_mps) ? std::max(0.0, cruise_speed_mps) : 0.0;
  if (!leading || !std::isfinite(leading->body_gap_m) ||
      !std::isfinite(leading->speed_mps) || leading->body_gap_m > followingGapForSpeed(
          leading->observed_speed_mps.value_or(leading->speed_mps),
          follow_distance_m_,overtake_follow_distance_m_)) {
    return cruise;
  }
  return std::clamp(leading->speed_mps, 0.0, cruise);
}

void DrivingFsm::acceptLateral(DrivingMode maneuver) noexcept {
  if (maneuver != DrivingMode::FREE_RUN && mode_ != maneuver) {
    mode_ = maneuver;
    ++revision_;
  }
}

void DrivingFsm::finishAvoid() noexcept {
  if (mode_ != DrivingMode::FREE_RUN) {
    mode_ = DrivingMode::FREE_RUN;
    ++revision_;
  }
}

void DrivingFsm::reset() noexcept {
  mode_ = DrivingMode::FREE_RUN;
  ++revision_;
}

}  // namespace reference_space_mppi_planner
