#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace reference_space_mppi_planner {

struct FollowSpeedConfig {
  double gap_m{3.0};
  double correction_enter_m{0.5};
  double correction_exit_m{0.2};
  double approach_speed_mps{0.0};
};

// A target-scoped Schmitt latch for gap correction, not a maneuver state machine
// or an enforced speed floor. Every proposal needs the execution validator.
class FollowSpeed {
public:
  explicit FollowSpeed(FollowSpeedConfig config = {}) : config_(config) {
    if (!std::isfinite(config.gap_m) || !std::isfinite(config.correction_enter_m) ||
        !std::isfinite(config.correction_exit_m) || config.correction_exit_m < 0.0 ||
        config.correction_enter_m <= config.correction_exit_m ||
        config.gap_m <= config.correction_enter_m ||
        !std::isfinite(config.approach_speed_mps) || config.approach_speed_mps < 0.0) {
      throw std::invalid_argument("follow gap requires gap > enter > exit >= 0 (metres)");
    }
  }

  void reset() {
    target_id_.clear();
    correcting_ = false;
  }

  double propose(const std::string &target_id, double body_gap_m,
                 double target_speed_mps, double cruise_speed_mps) {
    if (target_id.empty() || !std::isfinite(body_gap_m) ||
        !std::isfinite(target_speed_mps) || target_speed_mps < 0.0 ||
        !std::isfinite(cruise_speed_mps) || cruise_speed_mps < 0.0) {
      reset();
      return 0.0;
    }
    if (target_id != target_id_) {
      reset();
      target_id_ = target_id;
    }
    const double error_m = body_gap_m - config_.gap_m;
    if (body_gap_m > config_.gap_m + config_.correction_enter_m ||
        body_gap_m < config_.gap_m - config_.correction_enter_m) {
      correcting_ = true;
    } else if (body_gap_m >= config_.gap_m - config_.correction_exit_m &&
               body_gap_m <= config_.gap_m + config_.correction_exit_m) {
      correcting_ = false;
    }
    // Preserve the old 0.30 1/s approach gain. Matching means the latest target
    // velocity, never the previous command (which could outlive a target stop).
    double proposed_speed = target_speed_mps + (correcting_ ? 0.30 * error_m : 0.0);
    // Prefer maintaining approach speed outside the body-gap boundary. This
    // proposal can still be reduced or rejected by physicallyBoundedFallback.
    if (body_gap_m > config_.gap_m) {
      proposed_speed = std::max(proposed_speed, config_.approach_speed_mps);
    }
    return std::clamp(proposed_speed, 0.0, cruise_speed_mps);
  }

private:
  FollowSpeedConfig config_;
  std::string target_id_;
  bool correcting_{false};
};

} // namespace reference_space_mppi_planner
